#pragma once

#include <cmath>
#include <cstddef>

namespace navigation_planning_backend {

// Return the guide time for the junction after a marked route-boundary cell.
// A direct look-ahead can expose only its final endpoint after the mission
// waypoint; split the remaining interval so the final SFC piece is not
// reduced to the generic numerical 0.01 s duration clamp.
inline double routeBoundaryJunctionTime(
    const bool outgoing_from_route_gate,
    const int nearest_guide_index,
    const std::size_t guide_sample_count,
    const int junction_index,
    const double previous_junction_time,
    const double final_guide_time,
    const double nearest_guide_time) noexcept {
  if (outgoing_from_route_gate && nearest_guide_index >= 0 &&
      static_cast<std::size_t>(nearest_guide_index + 1) == guide_sample_count &&
      junction_index > 0 && std::isfinite(previous_junction_time) &&
      std::isfinite(final_guide_time) && final_guide_time > previous_junction_time + 0.02) {
    return previous_junction_time +
        0.5 * (final_guide_time - previous_junction_time);
  }
  return nearest_guide_time;
}

}  // namespace navigation_planning_backend
