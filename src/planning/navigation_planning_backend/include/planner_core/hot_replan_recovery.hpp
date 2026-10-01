#pragma once

#include <cmath>
#include <limits>

namespace navigation_planning_backend {

struct HotReplanSpliceCompatibility {
  bool finite{false};
  bool current_position_within_budget{false};
  bool future_position_within_envelope{false};
  bool future_velocity_within_envelope{false};
  double future_position_allowance_m{
      std::numeric_limits<double>::quiet_NaN()};
  double future_velocity_allowance_mps{
      std::numeric_limits<double>::quiet_NaN()};

  [[nodiscard]] bool requiresMeasuredStateRestart() const noexcept {
    return !finite || !current_position_within_budget ||
           !future_position_within_envelope ||
           !future_velocity_within_envelope;
  }
};

// A hot splice starts from the committed command at a short future horizon.
// Position proximity at the current instant is insufficient: the command and
// vehicle may already have opposing velocities and become detached before the
// splice point. This classifier rejects states outside two necessary
// acceleration-bounded reachability envelopes over that exact horizon. Passing
// these scalar envelopes is not a proof that a joint position/velocity boundary
// value problem is feasible; the normal continuous dynamics and world
// certificates remain authoritative. It introduces no independently tuned tolerance:
// the position tube is the existing tracking budget and the additional
// position/velocity reach is derived from the active mission acceleration
// limit.
inline HotReplanSpliceCompatibility assessHotReplanSpliceCompatibility(
    const double current_position_error_m,
    const double future_position_error_from_constant_velocity_m,
    const double future_velocity_error_mps,
    const double horizon_s,
    const double maximum_acceleration_mps2,
    const double tracking_position_budget_m) noexcept {
  HotReplanSpliceCompatibility result;
  if (!std::isfinite(current_position_error_m) ||
      !std::isfinite(future_position_error_from_constant_velocity_m) ||
      !std::isfinite(future_velocity_error_mps) ||
      !std::isfinite(horizon_s) || horizon_s < 0.0 ||
      !std::isfinite(maximum_acceleration_mps2) ||
      maximum_acceleration_mps2 <= 0.0 ||
      !std::isfinite(tracking_position_budget_m) ||
      tracking_position_budget_m < 0.0) {
    return result;
  }

  result.finite = true;
  result.future_position_allowance_m =
      tracking_position_budget_m +
      0.5 * maximum_acceleration_mps2 * horizon_s * horizon_s;
  result.future_velocity_allowance_mps =
      maximum_acceleration_mps2 * horizon_s;
  result.current_position_within_budget =
      current_position_error_m <= tracking_position_budget_m;
  result.future_position_within_envelope =
      future_position_error_from_constant_velocity_m <=
      result.future_position_allowance_m;
  result.future_velocity_within_envelope =
      future_velocity_error_mps <= result.future_velocity_allowance_mps;
  return result;
}

enum class HotReplanTrackingRecovery {
  kContinueHotStitch,
  kRetainCommittedCommand,
  kFailClosed,
};

// Once a hot splice has left the acceleration-bounded compatibility envelope,
// replacing the live reference with measured P/V and zero estimated A/J is
// itself a discontinuous command transition. Keep the independently certified
// command visible and let the runtime's bounded anchor-recovery state own any
// measured-state transition. A non-traversable measured pose remains
// fail-closed.
inline HotReplanTrackingRecovery classifyHotReplanTrackingRecovery(
    const bool tracking_budget_exceeded,
    const bool measured_start_traversable) noexcept {
  if (!tracking_budget_exceeded) {
    return HotReplanTrackingRecovery::kContinueHotStitch;
  }
  return measured_start_traversable
      ? HotReplanTrackingRecovery::kRetainCommittedCommand
      : HotReplanTrackingRecovery::kFailClosed;
}

}  // namespace navigation_planning_backend
