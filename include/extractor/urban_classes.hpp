#ifndef OSRM_EXTRACTOR_URBAN_CLASSES_HPP
#define OSRM_EXTRACTOR_URBAN_CLASSES_HPP

#include "extractor/class_data.hpp"

#include <algorithm>
#include <array>
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
using UrbanClassWeights = std::array<float, MAX_CLASS_INDEX + 2>;

inline float urbanClassRatio(const ClassData classes, const UrbanClassWeights &weights)
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

} // namespace osrm::extractor

#endif // OSRM_EXTRACTOR_URBAN_CLASSES_HPP
