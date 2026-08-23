#pragma once

#include <cmath>
#include <cstdint>
#include <iostream>
#include <stdexcept>

#include <super_utils/type_utils.hpp>

namespace navigation_runtime {

enum class PlannerResultDisposition {
  CommandReady,
  RestartFromRest,
  RetryFromRest,
  RetainCommittedCommand,
  FailClosed,
};

// Bounds consecutive rest-to-rest solve failures for one logical waypoint.
// This state belongs to the mission/planner FSM, not to the optimizer: a
// transient startup miss may be retried, but an unreachable goal must not be
// allowed to retry forever and later commit an arbitrary frontier trajectory.
class ConsecutiveFailureBudget {
 public:
  explicit ConsecutiveFailureBudget(std::uint32_t maximum_failures)
      : maximum_failures_(maximum_failures) {
    if (maximum_failures_ == 0U) {
      throw std::invalid_argument("maximum_failures must be positive");
    }
  }

  bool recordFailure() {
    if (failure_count_ < maximum_failures_) ++failure_count_;
    return exhausted();
  }

  void reset() { failure_count_ = 0U; }

  [[nodiscard]] bool exhausted() const { return failure_count_ >= maximum_failures_; }
  [[nodiscard]] std::uint32_t failureCount() const { return failure_count_; }
  [[nodiscard]] std::uint32_t maximumFailures() const { return maximum_failures_; }

 private:
  std::uint32_t maximum_failures_;
  std::uint32_t failure_count_{0U};
};

inline PlannerResultDisposition classifyPlannerResult(
    super_utils::RET_CODE result, bool plan_from_rest, bool command_available) {
  if (result == super_utils::SUCCESS || result == super_utils::NO_NEED ||
      result == super_utils::FINISH) {
    return PlannerResultDisposition::CommandReady;
  }
  if (result == super_utils::NEW_TRAJ) {
    return PlannerResultDisposition::RestartFromRest;
  }
  // SUPER's native FSM retries a failed rest-to-rest solve.  A single
  // optimizer timeout is not an emergency and no trajectory has been
  // committed yet for this goal.
  if (result == super_utils::FAILED && plan_from_rest) {
    return PlannerResultDisposition::RetryFromRest;
  }
  // A hot-replan failure leaves CmdTraj untouched.  That committed command
  // contains SUPER's main-to-backup switch and remains the only safe command
  // source while the committed safety suffix is drained.
  if (result == super_utils::FAILED && command_available) {
    return PlannerResultDisposition::RetainCommittedCommand;
  }
  return PlannerResultDisposition::FailClosed;
}

inline bool committedSafetySuffixIsUsable(
    bool safety_trajectory_available, double elapsed_s, double total_duration_s,
    double safety_transition_s, double anchor_error_m, double maximum_anchor_error_m,
    bool sampled_path_clear) {
  return safety_trajectory_available && std::isfinite(elapsed_s) &&
         std::isfinite(total_duration_s) && std::isfinite(safety_transition_s) &&
         std::isfinite(anchor_error_m) && std::isfinite(maximum_anchor_error_m) &&
         elapsed_s >= 0.0 && total_duration_s > elapsed_s &&
         safety_transition_s >= elapsed_s && safety_transition_s <= total_duration_s &&
         maximum_anchor_error_m > 0.0 &&
         anchor_error_m <= maximum_anchor_error_m && sampled_path_clear;
}

}  // namespace navigation_runtime
