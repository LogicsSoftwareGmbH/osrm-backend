#ifndef OSRM_EXTRACTOR_URBAN_CLASSES_HPP
#define OSRM_EXTRACTOR_URBAN_CLASSES_HPP

#include "extractor/class_data.hpp"
#include "extractor/profile_properties.hpp"

#include <zlib.h>

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>

namespace osrm::extractor
{

// Per-class-bit weights used to derive the urban_meters side-car consumed by
// the /table annotation `urban_share`: an edge contributes
// urban_meters = weight(classes) * distance, where weight(classes) is the
// maximum weight over the edge's set class bits.
//
// The LUT is resolved from the profile's `urban_share_weights` Lua table by
// osrm-extract (which owns the class-name -> bit mapping), serialized to
// .osrm.urban_config, and consumed by osrm-contract. Bit-indexed so that no
// downstream tool has to deal with class names.
//
// Deliberately sized to the full 8-bit ClassData byte, one float per bit: only
// bits 0..MAX_CLASS_INDEX are usable classes, so the last slot is permanently 0
// padding — do not shrink this without changing the serialized LUT format.
using UrbanClassWeights = std::array<float, MAX_CLASS_INDEX + 2>;

// WeightsT is any random-access container of float with at least
// MAX_CLASS_INDEX + 1 entries (UrbanClassWeights or a vector view over the
// serialized LUT)
template <typename WeightsT>
inline float urbanClassRatio(const ClassData classes, const WeightsT &weights)
{
    float ratio = 0.f;
    for (std::uint8_t bit = 0; bit <= MAX_CLASS_INDEX; ++bit)
    {
        if (classes & (1u << bit))
        {
            ratio = std::max(ratio, weights[bit]);
        }
    }
    return ratio;
}

// Identity of a resolved urban-weight LUT: a CRC32 over the class-name -> bit
// mapping it was resolved under plus the weight values themselves. The LUT's
// bit indices are meaningless under any other class mapping, so both side-car
// files record this identity and every load path recomputes it from the
// .osrm.properties actually being loaded — a config left behind by a different
// extract run, or a contract payload derived from weights that were changed
// afterwards, is rejected instead of serving plausible but wrong ratios.
inline std::uint32_t computeUrbanConfigIdentity(const ProfileProperties &properties,
                                                const UrbanClassWeights &weights)
{
    auto crc = crc32(0L, Z_NULL, 0);
    for (std::size_t index = 0; index <= MAX_CLASS_INDEX; ++index)
    {
        const auto name = properties.GetClassNameForIndex(index);
        // include the terminator so adjacent names cannot alias each other
        crc = crc32(crc,
                    reinterpret_cast<const unsigned char *>(name.c_str()),
                    static_cast<uInt>(name.size() + 1));
    }
    crc = crc32(crc,
                reinterpret_cast<const unsigned char *>(weights.data()),
                static_cast<uInt>(weights.size() * sizeof(float)));
    return static_cast<std::uint32_t>(crc);
}

} // namespace osrm::extractor

#endif // OSRM_EXTRACTOR_URBAN_CLASSES_HPP
