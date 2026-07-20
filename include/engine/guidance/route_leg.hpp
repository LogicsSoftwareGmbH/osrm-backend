#ifndef ROUTE_LEG_HPP
#define ROUTE_LEG_HPP

#include "engine/guidance/route_step.hpp"

#include <optional>

#include <string>
#include <vector>

namespace osrm::engine::guidance
{

struct RouteLeg
{
    double distance;
    double duration;
    double weight;
    std::string summary;
    std::vector<RouteStep> steps;
    // set only when annotations=urban_share was requested and the leg has length:
    // urban-weighted share of the leg in [0, 1], the /table cell equivalent
    std::optional<double> urban_share;
};
} // namespace osrm::engine::guidance

#endif
