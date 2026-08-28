#pragma once

#include <cmath>
#include <cstdint>
#include <iostream>
#include <limits>
#include <stdexcept>

#include <navigation_planning/planner_status.hpp>

namespace navigation_runtime {

inline bool canHotRetargetAtWaypointTransition(
    bool same_logical_goal, bool previous_goal_was_pass_through,
    bool command_available, bool planner_failure_latched, bool safety_suffix_active) {
  // A committed safety suffix is a world-certified braking command, not a
  // goal-geometry claim. At a measured pass-through transition it may retain
  // physical command ownership while the exact bundle identity is rebound to
  // the new epoch. It remains safety-owned and finite; the planner must still
  // replace it with a new-goal candidate before its declared end.
  (void)safety_suffix_active;
  return !same_logical_goal && previous_goal_was_pass_through && command_available &&
         !planner_failure_latched;
}

// A pass-through goal transition changes the route boundary. Rebase on the
// newest measured PVA state rather than hot-stitching through the previous
// endpoint. The remaining-time rule also covers an expired command when the
// identity is not available; its interval is supplied by the runtime timer
// rather than introduced as a second safety threshold.
inline bool hotRetargetNeedsMeasuredStatePlan(
    bool hot_goal_transition, bool command_available,
    bool goal_identity_changed,
    double command_elapsed_s, double command_duration_s,
    double planning_interval_s) noexcept {
  if (!hot_goal_transition || !command_available) {
    return false;
  }
  // A measured pass-through acceptance is the route-boundary proof. Keep the
  // existing nominal command as the hot-replan history while it still has
  // time remaining; forcing PlanFromRest here discards the very velocity
  // continuity that pass-through semantics are intended to preserve. The
  // caller still rebases when the retained command is at its lease boundary.
  (void)goal_identity_changed;
  if (!std::isfinite(command_elapsed_s) || !std::isfinite(command_duration_s) ||
      !std::isfinite(planning_interval_s) || planning_interval_s <= 0.0 ||
      command_duration_s <= 0.0) {
    return false;
  }
  const double remaining_s = command_duration_s - command_elapsed_s;
  // The command clock is reconstructed from nanosecond timestamps and stored
  // in seconds for the backend, so an exact one-interval boundary can differ
  // by a few representable floating-point units.
  return remaining_s <= planning_interval_s + 1.0e-9;
}

enum class PlannerResultDisposition {
  CommandReady,
  RestartFromRest,
  RetryFromRest,
  ValidateRetainedCommand,
  RetainCommittedCommand,
  FailClosed,
};

enum class RetainedValidationTransition {
  PreserveExistingState,
  FailClosed,
};

inline RetainedValidationTransition retainedValidationTransition(bool usable) noexcept {
  return usable ? RetainedValidationTransition::PreserveExistingState
                : RetainedValidationTransition::FailClosed;
}

// Once the command publisher has observed a certified endpoint inside the
// active waypoint acceptance ball, a rest-to-rest retry must not revoke the
// terminal hold merely because the world advanced during a replacement solve.
// The hold remains limited to that exact bundle generation and is still
// revalidated by the mapping publication boundary.
inline bool terminalHoldIsPending(
    bool command_available, bool reaches_goal,
    std::uint64_t terminal_bundle_generation) noexcept {
  return command_available && reaches_goal && terminal_bundle_generation != 0U;
}

// The publisher can miss the single wall-clock tick at which a finite bundle
// returns its finished sample.  It may still publish that exact declared
// endpoint after the execution interval, but only when the endpoint is a
// finite, current known-free sample and remains inside the active goal
// acceptance ball.  This authorizes no future trajectory sample.
inline bool terminalEndpointHoldIsCertified(
    bool endpoint_valid, bool endpoint_reaches_goal,
    bool endpoint_known_free) noexcept {
  return endpoint_valid && endpoint_reaches_goal && endpoint_known_free;
}

// A finite local trajectory may end at a known-free sensing frontier rather
// than at the mission goal.  If the publisher misses its finished tick, it
// may replay that exact endpoint only while the measured execution state is
// still close enough to the endpoint for the command-anchor contract.  The
// planner then observes completion and starts a new PlanFromRest; this does
// not authorize any future sample or claim waypoint acceptance.
inline bool expiredEndpointMayBeReplayed(
    bool endpoint_valid, bool endpoint_known_free,
    bool endpoint_near_execution_state) noexcept {
  return endpoint_valid && endpoint_known_free && endpoint_near_execution_state;
}

// A stale world snapshot suspends command publication but does not mutate the
// immutable committed bundle. Publication may resume without a replacement
// solve only when that exact generation has subsequently been recertified on
// a fresh world and still belongs to the active localization/goal epochs.
inline bool worldFreshnessSuspendedCommandMayResume(
    std::uint64_t suspended_generation,
    std::uint64_t recertified_generation,
    std::uint64_t bundle_localization_epoch,
    std::uint64_t bundle_goal_epoch,
    std::uint64_t active_localization_epoch,
    std::uint64_t active_goal_epoch,
    std::int64_t valid_until_ns,
    std::int64_t now_ns,
    bool bundle_valid,
    bool planner_failure_latched,
    bool execution_lease_allows_command) noexcept {
  return suspended_generation != 0U &&
         recertified_generation == suspended_generation && bundle_valid &&
         bundle_localization_epoch == active_localization_epoch &&
         bundle_goal_epoch == active_goal_epoch && valid_until_ns >= now_ns &&
         !planner_failure_latched && execution_lease_allows_command;
}

// Mapping recertification replaces an immutable bundle with a certified copy.
// A command timer that sampled the previous pointer must skip that one
// publication, but it must not revoke the current command merely because the
// pointer changed. The same rule covers a newer planner commit winning the
// race. Continued availability is allowed only for a non-older, valid bundle
// that still belongs to both active epochs and its declared time interval.
inline bool supersedingBundleMayRemainAvailable(
    std::uint64_t sampled_generation,
    std::uint64_t current_generation,
    std::uint64_t bundle_localization_epoch,
    std::uint64_t bundle_goal_epoch,
    std::uint64_t active_localization_epoch,
    std::uint64_t active_goal_epoch,
    std::int64_t valid_until_ns,
    std::int64_t now_ns,
    bool bundle_valid,
    bool planner_failure_latched,
    bool execution_lease_allows_command) noexcept {
  return sampled_generation != 0U &&
         current_generation >= sampled_generation && bundle_valid &&
         bundle_localization_epoch == active_localization_epoch &&
         bundle_goal_epoch == active_goal_epoch && valid_until_ns >= now_ns &&
         !planner_failure_latched && execution_lease_allows_command;
}

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
    navigation_planning::PlannerStatus result, bool plan_from_rest, bool command_available,
    bool commit_observed) {
  if ((result == navigation_planning::PlannerStatus::kSuccess ||
       result == navigation_planning::PlannerStatus::kFinished) && commit_observed) {
    return PlannerResultDisposition::CommandReady;
  }
  if (result == navigation_planning::PlannerStatus::kNoNeed && command_available) {
    return PlannerResultDisposition::ValidateRetainedCommand;
  }
  if (result == navigation_planning::PlannerStatus::kRestartFromRest) {
    return PlannerResultDisposition::RestartFromRest;
  }
  // A failed replacement solve leaves the execution bundle untouched. This is
  // true even when the planner has intentionally restarted from measured
  // state: the existing immutable main-to-backup command remains the only
  // certified source while the replacement is retried. Revalidate/retain it
  // before charging the no-command PlanFromRest failure budget.
  if (result == navigation_planning::PlannerStatus::kFailed && command_available) {
    return PlannerResultDisposition::RetainCommittedCommand;
  }
  // planner backend's native FSM retries a failed rest-to-rest solve when no
  // executable command exists. A single optimizer timeout is not yet an
  // emergency, but repeated confirmed failures remain fail-closed.
  if (result == navigation_planning::PlannerStatus::kFailed && plan_from_rest) {
    return PlannerResultDisposition::RetryFromRest;
  }
  return PlannerResultDisposition::FailClosed;
}

inline bool committedSafetySuffixIsUsable(
    bool safety_trajectory_available, double elapsed_s, double total_duration_s,
    double safety_transition_s, double anchor_error_m, double maximum_anchor_error_m,
    bool sampled_path_clear) {
  const bool common_contract = std::isfinite(elapsed_s) &&
                               std::isfinite(total_duration_s) &&
                               std::isfinite(safety_transition_s) &&
                               std::isfinite(anchor_error_m) &&
                               std::isfinite(maximum_anchor_error_m) &&
                               elapsed_s >= 0.0 && total_duration_s > elapsed_s &&
                               maximum_anchor_error_m > 0.0 &&
                               anchor_error_m <= maximum_anchor_error_m && sampled_path_clear;
  if (!common_contract) return false;

  // planner backend intentionally commits main-only when the complete EXP trajectory is
  // visible and no braking branch is needed.  A transient optimizer miss must
  // not invalidate that still-visible command.  With an explicit backup the
  // main-to-backup transition remains part of the contract.
  if (!safety_trajectory_available) {
    return std::abs(safety_transition_s - elapsed_s) <= 1.0e-9;
  }
  return safety_transition_s >= elapsed_s && safety_transition_s <= total_duration_s;
}

}  // namespace navigation_runtime
