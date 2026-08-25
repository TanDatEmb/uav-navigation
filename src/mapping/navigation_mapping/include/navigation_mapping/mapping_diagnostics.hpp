#pragma once

#include <string_view>

#include <rog_map/rog_map.h>

namespace navigation_mapping {

// Vendor representation is confined to the mapping implementation package.
// Runtime consumers use these product names and do not include the map vendor.
using PointCloud = rog_map::PointCloud;
using MapUpdateOutcome = rog_map::MapUpdateOutcome;
using RaycastDiagnostics = rog_map::ProbMap::RaycastDiagnostics;

[[nodiscard]] inline bool worldUpdateAdvanced(MapUpdateOutcome outcome) noexcept {
  return rog_map::mapUpdateAdvancedWorld(outcome);
}

[[nodiscard]] inline std::string_view worldUpdateOutcomeName(
    MapUpdateOutcome outcome) noexcept {
  return rog_map::mapUpdateOutcomeName(outcome);
}

}  // namespace navigation_mapping
