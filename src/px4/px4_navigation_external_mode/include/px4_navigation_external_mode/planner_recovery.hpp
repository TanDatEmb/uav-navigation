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

// A previously accepted terminal command may be held after its normal command
// validity interval only while the existing bounded planner-recovery window is
// open. This is endpoint hold, never permission to sample or extend a future
// trajectory.
inline bool terminalRecoveryCommandMayBeHeld(
    bool completed_command, bool pending, std::int64_t now_ns,
    std::int64_t deadline_ns) noexcept {
  return completed_command && pending && now_ns > 0 && deadline_ns > 0 &&
         now_ns < deadline_ns;
}

// A completed backup endpoint is geometrically anchored only when the command
// itself is accepted and the measured state remains within the existing
// command-anchor envelope. The caller must independently require measured
// waypoint acceptance before treating this as a settled terminal hold.
inline bool backupEndpointHoldIsAnchored(
    bool command_inside_acceptance, bool measured_finite, bool command_finite,
    double measured_command_error_m, double max_anchor_error_m) noexcept {
  return command_inside_acceptance && measured_finite && command_finite &&
         std::isfinite(measured_command_error_m) &&
         std::isfinite(max_anchor_error_m) && max_anchor_error_m > 0.0 &&
         measured_command_error_m <= max_anchor_error_m;
}

}  // namespace px4_navigation_external_mode
