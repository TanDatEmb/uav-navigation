#pragma once

#include <cstdint>

namespace navigation_mission {

// Once a finite PASS_THROUGH command has reached its declared endpoint, the
// planner must continue from measured state while MissionProgress completes
// the measured waypoint handoff. STOP remains terminal and a frontier
// trajectory follows the existing non-goal completion path.
inline bool completedPassThroughRequiresContinuation(
    bool trajectory_completed, bool endpoint_valid,
    bool pass_through_waypoint, bool has_outgoing_route) noexcept {
  return trajectory_completed && endpoint_valid && pass_through_waypoint &&
         has_outgoing_route;
}

// A finite PASS_THROUGH waypoint is meaningful only when the immutable route
// contains an outgoing leg. Treating a missing continuation as STOP silently
// changes mission behavior and can commit a zero-velocity terminal state.
inline bool waypointBehaviorContractValid(
    bool pass_through_waypoint, bool has_next_waypoint) noexcept {
  return !pass_through_waypoint || has_next_waypoint;
}

}  // namespace navigation_mission
