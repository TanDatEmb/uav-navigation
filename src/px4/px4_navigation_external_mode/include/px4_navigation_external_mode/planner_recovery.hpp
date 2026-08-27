#pragma once

#include <cmath>
#include <cstdint>

namespace px4_navigation_external_mode {

// A completed backup suffix is allowed one bounded scheduling window for the
// runtime planner to publish a replacement PVA command. This is a hold-only
// recovery window; it is not a new trajectory-validity or tracking budget.
inline bool plannerRecoveryWaitExpired(
    bool pending, std::int64_t now_ns, std::int64_t deadline_ns) noexcept {
  return pending && deadline_ns > 0 && now_ns >= deadline_ns;
}

// A completed backup endpoint may sit close to the edge of the waypoint
// acceptance disk.  Treat it as a hold candidate when the command itself is
// accepted and the measured state is still within the existing command-anchor
// envelope.  MissionController continues to require the measured position and
// speed gates before advancing the mission.
inline bool backupEndpointHoldIsAnchored(
    bool command_inside_acceptance, bool measured_finite, bool command_finite,
    double measured_command_error_m, double max_anchor_error_m) noexcept {
  return command_inside_acceptance && measured_finite && command_finite &&
         std::isfinite(measured_command_error_m) &&
         std::isfinite(max_anchor_error_m) && max_anchor_error_m > 0.0 &&
         measured_command_error_m <= max_anchor_error_m;
}

}  // namespace px4_navigation_external_mode
