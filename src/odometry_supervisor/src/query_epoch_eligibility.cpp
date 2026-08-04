#include "odometry_supervisor/query_epoch_eligibility.hpp"

#include <algorithm>

namespace odometry_supervisor {

QueryEpochEligibility latest_propagated_epoch_available_to_px4(
    const std::deque<OdometryState>& propagated_history,
    const std::optional<OdometryState>& latest_px4) {
  if (propagated_history.empty()) {
    return {std::nullopt, QueryEpochEligibilityReason::kNoPropagatedHistory};
  }
  if (!latest_px4) {
    return {std::nullopt, QueryEpochEligibilityReason::kNoPx4History};
  }
  const auto upper = std::upper_bound(
      propagated_history.begin(), propagated_history.end(), latest_px4->timestamp_ns,
      [](std::int64_t timestamp, const OdometryState& state) {
        return timestamp < state.timestamp_ns;
      });
  if (upper == propagated_history.begin()) {
    return {std::nullopt, QueryEpochEligibilityReason::kLioEpochAheadOfPx4};
  }
  return {*std::prev(upper), QueryEpochEligibilityReason::kEligible};
}

QueryEpochKey query_epoch_key(const OdometryState& state, std::uint64_t lio_generation,
                              std::uint64_t frame_generation,
                              std::uint64_t time_generation) {
  return {state.timestamp_ns, lio_generation, frame_generation, time_generation};
}

}  // namespace odometry_supervisor
