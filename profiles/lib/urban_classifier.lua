-- Urban classifier: tags each way as 'urban', 'suburban' or neither (rural)
-- using OSRM's named-class mechanism. The resulting class bits are consumed by
-- osrm-contract to derive the per-edge urban_meters side-car that powers the
-- /table annotation `urban_share` (share of a matrix cell's fastest path that
-- runs through built-up area).
--
-- All classification tuning lives in this file; the class -> weight mapping
-- lives in the profile's `urban_share_weights` table. Neither requires a
-- rebuild of OSRM, only re-running osrm-extract + osrm-contract.
--
-- A profile opts in by
--   1. adding 'urban' (and optionally 'suburban') to its `classes` Sequence,
--   2. declaring `urban_share_weights = { urban = 1.0, suburban = 0.5 }`,
--   3. requiring this module and inserting `UrbanClassifier.classify` into the
--      process_way handler chain (right after WayHandlers.classes).
-- Profiles that do none of this are completely unaffected.

local Set = require('lib/set')
local Sequence = require('lib/sequence')

local UrbanClassifier = {}

-- Tunables ------------------------------------------------------------------

-- Legal-zone keys stating which national speed-limit regime applies. The union
-- is required: StreetComplete migrated source:maxspeed -> maxspeed:type in
-- 2021, zone:traffic is essentially unused in Austria, and implicit limits are
-- frequently written into maxspeed itself (e.g. maxspeed=DE:urban).
UrbanClassifier.zone_keys = Sequence {
  'source:maxspeed', 'maxspeed:type', 'zone:maxspeed', 'zone:traffic', 'maxspeed'
}

-- Keys whose value is a zone statement even when it is a bare number
-- (zone:maxspeed=30 means a 30-zone; maxspeed=30 is just a limit and must go
-- through the numeric tiers with their directional-minimum handling instead).
UrbanClassifier.bare_number_zone_keys = Set {
  'zone:maxspeed', 'zone:traffic'
}

-- Highway types that are inherently built-up regardless of tags.
UrbanClassifier.urban_highways = Set {
  'residential', 'living_street', 'pedestrian'
}

-- Highway types that are inherently rural/interurban regardless of zone tags
-- (interchanges inside cities must not read as urban).
UrbanClassifier.rural_highways = Set {
  'motorway', 'motorway_link', 'trunk', 'trunk_link'
}

-- Numeric maxspeed tiers (km/h): <= urban_max_kmh -> urban,
-- <= suburban_max_kmh -> suburban, above -> rural.
UrbanClassifier.urban_max_kmh = 40
UrbanClassifier.suburban_max_kmh = 70

-- Street-lit fallback: these highway types with lit=yes and no other signal
-- are treated as suburban. lit tagging covers only ~40% of DACH ways (even on
-- residential), so this tier has false negatives by design — but where the tag
-- is present it is a clean built-up signal: 94.6% of lit-tagged German
-- residential ways say yes, and lit=yes sits at 35-40% of primary, secondary
-- and tertiary alike (vs 0.87% of motorway).
UrbanClassifier.lit_fallback_highways = Set {
  'unclassified', 'tertiary', 'tertiary_link', 'secondary', 'primary'
}

-- Zone-value matching -------------------------------------------------------

-- Classifies a legal-zone tag value: 'urban', 'suburban', 'rural' or nil (no
-- statement). Matching is case-insensitive.
-- Examples: DE:urban, AT:urban, urban        -> urban
--           DE:zone30, DE:zone:30            -> urban (30-zones are built-up)
--           DE:30, AT:30, CH:30              -> urban (the wiki-documented
--                                               zone:maxspeed number form)
--           AT:city_limit30, AT:city_limit40 -> urban (limit posted at the
--                                               Ortstafel, applies to the whole
--                                               built-up area)
--           DE:living_street, DE:bicycle_road-> urban
--           DE:rural, AT:rural, rural        -> rural
--           DE:motorway, AT:motorway         -> rural
--           sign, DE:sign:274-70, DE:274.1   -> nil (explicit signs and
--                                               traffic-sign codes carry no
--                                               urban statement; the number
--                                               parse is anchored so sign codes
--                                               never sneak through it)
local function classify_zone_value(v, allow_bare_number)
  if not v or v == '' then
    return nil
  end
  v = v:lower()
  if v == 'urban' or v:sub(-6) == ':urban' or
     v:find('zone', 1, true) or v:find('city_limit', 1, true) or
     v:sub(-14) == ':living_street' or v:sub(-13) == ':bicycle_road' then
    return 'urban'
  end
  if v == 'rural' or v:sub(-6) == ':rural' or
     v == 'motorway' or v:sub(-9) == ':motorway' or
     v:sub(-6) == ':trunk' then
    return 'rural'
  end
  -- country-prefixed zone number (zone:maxspeed=DE:30 is the standard German
  -- 30-zone form, ~163k ways): tier it like a numeric limit
  local kmh = tonumber(v:match('^%a%a:(%d+)$'))
  if not kmh and allow_bare_number then
    kmh = tonumber(v:match('^(%d+)$'))
  end
  if kmh then
    if kmh <= UrbanClassifier.urban_max_kmh then
      return 'urban'
    elseif kmh <= UrbanClassifier.suburban_max_kmh then
      return 'suburban'
    else
      return 'rural'
    end
  end
  return nil
end

-- Parses a maxspeed-ish value to km/h, or nil if not numeric.
local function parse_kmh(v)
  if not v or v == '' then
    return nil
  end
  if v == 'walk' or v:sub(-5) == ':walk' then
    return 7
  end
  if v == 'none' then
    return 999
  end
  local mph = v:match('^(%d+)%s*mph$')
  if mph then
    return tonumber(mph) * 1.609
  end
  return tonumber(v:match('^%d+$'))
end

-- Classification ------------------------------------------------------------

-- Pure function: way tags in, tier out ('urban' | 'suburban' | nil = rural).
-- Deliberately direction-symmetric: a road is inside or outside a built-up
-- area for both travel directions.
function UrbanClassifier.tier(way, data)
  local highway = data.highway or way:get_value_by_key('highway')

  -- 1. hard rural: motorways/trunks and their ramps, regardless of zone tags
  if highway and UrbanClassifier.rural_highways[highway] then
    return nil
  end

  -- 2. explicit legal-zone statement wins
  for _, key in ipairs(UrbanClassifier.zone_keys) do
    local zone = classify_zone_value(way:get_value_by_key(key),
                                     UrbanClassifier.bare_number_zone_keys[key])
    if zone == 'rural' then
      return nil
    elseif zone then
      return zone
    end
  end

  -- 3. inherently built-up highway types
  if highway and UrbanClassifier.urban_highways[highway] then
    return 'urban'
  end

  -- 4. numeric speed limit tiers (min over plain/forward/backward)
  local kmh = parse_kmh(way:get_value_by_key('maxspeed'))
  for _, key in ipairs({'maxspeed:forward', 'maxspeed:backward'}) do
    local directional = parse_kmh(way:get_value_by_key(key))
    if directional and (not kmh or directional < kmh) then
      kmh = directional
    end
  end
  if kmh then
    if kmh <= UrbanClassifier.urban_max_kmh then
      return 'urban'
    elseif kmh <= UrbanClassifier.suburban_max_kmh then
      return 'suburban'
    else
      return nil
    end
  end

  -- 5. street lighting as a weak built-up signal for minor roads
  if highway and UrbanClassifier.lit_fallback_highways[highway] and
     way:get_value_by_key('lit') == 'yes' then
    return 'suburban'
  end

  -- 6. default: rural
  return nil
end

-- Handler for the process_way chain. No-op unless the profile declares the
-- 'urban' class; a tier the profile does not declare is treated as rural.
function UrbanClassifier.classify(profile, way, result, data)
  if not profile.classes then
    return
  end

  -- memoized on the profile: rebuilding this Set per way is measurable on
  -- planet-sized extracts
  local allowed_classes = profile._urban_allowed_classes
  if not allowed_classes then
    allowed_classes = Set {}
    for k, v in pairs(profile.classes) do
      allowed_classes[v] = true
    end
    profile._urban_allowed_classes = allowed_classes
  end
  if not allowed_classes['urban'] then
    return
  end

  local tier = UrbanClassifier.tier(way, data)
  if tier and allowed_classes[tier] then
    result.forward_classes[tier] = true
    result.backward_classes[tier] = true
  end
end

return UrbanClassifier
