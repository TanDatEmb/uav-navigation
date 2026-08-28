#pragma once

namespace navigation_planning_backend {

enum class HotReplanTrackingRecovery {
  kContinueHotStitch,
  kRestartFromMeasuredState,
  kFailClosed,
};

// Once the measured vehicle has left the command-tracking budget, a connector
// from the command state ahead of the vehicle back to the historical measured
// state is not a recovery trajectory: it explicitly encodes route regression.
// End hot stitching and let the runtime start a new solve from a fresh measured
// state. A non-traversable measured pose remains fail-closed.
inline HotReplanTrackingRecovery classifyHotReplanTrackingRecovery(
    const bool tracking_budget_exceeded,
    const bool measured_start_traversable) noexcept {
  if (!tracking_budget_exceeded) {
    return HotReplanTrackingRecovery::kContinueHotStitch;
  }
  return measured_start_traversable
      ? HotReplanTrackingRecovery::kRestartFromMeasuredState
      : HotReplanTrackingRecovery::kFailClosed;
}

}  // namespace navigation_planning_backend
