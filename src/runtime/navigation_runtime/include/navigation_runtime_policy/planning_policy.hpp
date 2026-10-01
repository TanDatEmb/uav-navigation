#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <iostream>
#include <limits>
#include <mutex>
#include <optional>

#include <navigation_contracts/msg/navigation_goal.hpp>
#include <navigation_execution/execution_recovery_state.hpp>
#include <navigation_planning/candidate_bundle.hpp>
#include <navigation_planning/planner_status.hpp>
#include <navigation_planning/planning_timing.hpp>

namespace navigation_runtime {

// Small sole-owner model for the runtime's single pending request. It makes
// callback/handoff interleavings explicit: enqueue, snapshot and consume are
// each linearizable operations under the caller's input/transition critical
// section, and consumption removes the exact request exactly once.
class PendingGoalHandoffOwner {
 public:
  using GoalConstPtr = navigation_contracts::msg::NavigationGoal::ConstSharedPtr;

  // The owner stores one immutable message, not a parallel identity and
  // message optional.  The caller must hold the runtime input/transition
  // transaction while deciding suffix ownership and invoking this method.
  bool enqueueGoal(const GoalConstPtr& candidate,
                  const std::optional<navigation_contracts::msg::NavigationGoal>& active,
                  bool safety_suffix_active) {
    if (!safety_suffix_active || !candidate || !active.has_value() ||
        candidate->mission_id != active->mission_id ||
        !goalMessageNewer(*candidate, *active)) return false;
    std::lock_guard<std::mutex> lock(goal_mutex_);
    if (pending_goal_ && !goalMessageNewer(*candidate, *pending_goal_)) {
      return false;
    }
    pending_goal_ = candidate;
    return true;
  }

  [[nodiscard]] GoalConstPtr goalSnapshot() const {
    std::lock_guard<std::mutex> lock(goal_mutex_);
    return pending_goal_;
  }

  // Peek and consume are separate so a stopped-suffix handoff can prepare a
  // new candidate before the pending request is removed from its sole owner.
  // The caller must consume the exact pointer only after the replacement
  // command has committed successfully.
  [[nodiscard]] bool consumeGoal(const GoalConstPtr& expected) {
    if (!expected) return false;
    std::lock_guard<std::mutex> lock(goal_mutex_);
    if (!pending_goal_ || pending_goal_.get() != expected.get()) return false;
    pending_goal_.reset();
    return true;
  }

  [[nodiscard]] bool goalMatchesStatus(
      const std::string& mission_id, std::uint32_t waypoint_index,
      std::uint64_t request_id) const {
    std::lock_guard<std::mutex> lock(goal_mutex_);
    return pending_goal_ && pending_goal_->mission_id == mission_id &&
           pending_goal_->waypoint_index == waypoint_index &&
           pending_goal_->request_id == request_id;
  }

  void clearGoal() {
    std::lock_guard<std::mutex> lock(goal_mutex_);
    pending_goal_.reset();
  }

  [[nodiscard]] bool clearIfCurrent(const GoalConstPtr& expected) {
    std::lock_guard<std::mutex> lock(goal_mutex_);
    if (!expected || !pending_goal_ || pending_goal_.get() != expected.get()) {
      return false;
    }
    pending_goal_.reset();
    return true;
  }

 private:
  static bool goalMessageNewer(
      const navigation_contracts::msg::NavigationGoal& candidate,
      const navigation_contracts::msg::NavigationGoal& current) noexcept {
    if (candidate.mission_id != current.mission_id) return false;
    if (candidate.request_id != current.request_id) {
      return candidate.request_id > current.request_id;
    }
    if (candidate.waypoint_index != current.waypoint_index) {
      return candidate.waypoint_index > current.waypoint_index;
    }
    return candidate.route.route_revision > current.route.route_revision;
  }

  mutable std::mutex goal_mutex_;
  GoalConstPtr pending_goal_;
};

inline bool canHotRetargetAtWaypointTransition(
    bool same_logical_goal, bool previous_goal_was_pass_through,
    bool command_available, bool planner_failure_latched, bool safety_suffix_active) {
  // A committed safety suffix is a world-certified braking command, not a
  // goal-geometry claim. It cannot be rebound to a new goal: doing so would
  // let a new nominal solve relabel a moving BACKUP/EMERGENCY command as
  // TrackMain before certified stop/handover. Defer the retarget until the
  // suffix reaches a certified stop (or fail closed if it cannot).
  return !same_logical_goal && previous_goal_was_pass_through && command_available &&
         !planner_failure_latched && !safety_suffix_active;
}
inline bool watchdogTimeoutMayRetainSafetySuffix(
    navigation_execution::ExecutionRecoveryState state, bool command_available,
    bool safety_suffix_active) noexcept {
  const bool safety_state =
      state == navigation_execution::ExecutionRecoveryState::kTrackBackup ||
      state == navigation_execution::ExecutionRecoveryState::kEmergencyBrake;
  return safety_state && command_available && safety_suffix_active;
}

// A stopped recovery endpoint is a bounded, known-free hold while a measured
// state PlanFromRest solve is retried. This predicate only identifies when the
// watchdog may retain that hold; the existing stopped-recovery timeout and its
// failure handling remain the terminal authority.
inline bool watchdogTimeoutMayRetainStoppedRecoveryHold(
    navigation_execution::ExecutionRecoveryState state, bool command_available,
    bool restart_from_rest) noexcept {
  return state == navigation_execution::ExecutionRecoveryState::kStoppedRecovery &&
         command_available && restart_from_rest;
}
inline bool pendingGoalTerminalStatusMayClear(
    bool matches_pending, bool safety_suffix_active) noexcept {
  // A queued request can report PAUSED/COMPLETED while the preceding
  // BACKUP/EMERGENCY command is still moving. That status is not a certified
  // handoff boundary; retain the request until the suffix-stop transaction
  // promotes it.
  return matches_pending && !safety_suffix_active;
}

// A hot-retarget flag authorizes exactly one forced solve for the new
// checkpoint. Once either the measured-state or hot-stitch transition has
// committed, leaving the flag set would bypass horizon-driven renewal on every
// later timer tick and repeatedly move a future route-boundary junction away
// from the executing vehicle.
inline bool clearHotGoalTransitionAfterCommit(
    bool measured_state_transition_committed,
    bool hot_stitch_transition_committed) noexcept {
  return measured_state_transition_committed || hot_stitch_transition_committed;
}

// A pass-through goal transition may reuse the committed future state only
// while an available MAIN command has a finite anchor within the supplied
// existing limit. Missing or invalid identity/anchor evidence makes the
// runtime use measured-state planning or fail closed; this helper adds no
// timing threshold and does not cover an expired command.
inline bool hotRetargetUsesCommittedFutureState(
    bool hot_goal_transition, bool command_available,
    navigation_planning::CandidateRole command_role,
    double command_anchor_error_m, double maximum_anchor_error_m) noexcept {
  return hot_goal_transition && command_available &&
         command_role == navigation_planning::CandidateRole::kMain &&
         std::isfinite(command_anchor_error_m) &&
         std::isfinite(maximum_anchor_error_m) && maximum_anchor_error_m > 0.0 &&
         command_anchor_error_m <= maximum_anchor_error_m;
}
enum class PlannerResultDisposition {
  CommandReady,
  RestartFromRest,
  RetryFromRest,
  ValidateRetainedCommand,
  RetainCommittedCommand,
  FailClosed,
};

enum class PlannerRenewalReason : std::uint8_t {
  kRetainCertifiedMain,
  kForcedTransition,
  kNoCommand,
  kSafetyRecovery,
  kInvalidHorizon,
  kRenewalDue,
  kQualityRefinement,
};

struct PlannerRenewalDecision {
  bool run_optimizer{true};
  PlannerRenewalReason reason{PlannerRenewalReason::kInvalidHorizon};
  double remaining_main_horizon_s{std::numeric_limits<double>::quiet_NaN()};
  double required_lead_time_s{std::numeric_limits<double>::quiet_NaN()};
};

// Return a conservative upper bound for the distance needed to stop from the
// configured cruise speed. The jerk term deliberately over-approximates the
// triangular/trapezoidal jerk-limited profile; it is only a phase-selection
// condition and never replaces the planner's exact braking certificate.
inline double plannerTerminalStopBrakingDistanceM(
    double speed_mps, double acceleration_mps2, double jerk_mps3) noexcept {
  if (!std::isfinite(speed_mps) || speed_mps <= 0.0 ||
      !std::isfinite(acceleration_mps2) || acceleration_mps2 <= 0.0 ||
      !std::isfinite(jerk_mps3) || jerk_mps3 <= 0.0) {
    return std::numeric_limits<double>::infinity();
  }
  const double distance = speed_mps * speed_mps / (2.0 * acceleration_mps2) +
      speed_mps * acceleration_mps2 / jerk_mps3;
  return std::isfinite(distance) && distance > 0.0
      ? distance : std::numeric_limits<double>::infinity();
}

// A terminal STOP owns the reduced-speed approach envelope only when the
// remaining route arc is inside the conservative stopping horizon. A distant
// STOP is still a cruise leg; applying a terminal scale to the whole
// PlanFromRest solve creates the observed half-speed/fast-replan oscillation.
// Invalid route/dynamics evidence fails conservative by selecting the slower
// phase, while candidate V/A/J and world certificates remain authoritative.
inline bool plannerTerminalStopApproachDue(
    bool terminal_stop, double remaining_route_m, double speed_mps,
    double acceleration_mps2, double jerk_mps3) noexcept {
  if (!terminal_stop) return false;
  const double stopping_distance = plannerTerminalStopBrakingDistanceM(
      speed_mps, acceleration_mps2, jerk_mps3);
  if (!std::isfinite(stopping_distance)) return true;
  if (!std::isfinite(remaining_route_m) || remaining_route_m < 0.0) return true;
  return remaining_route_m <= stopping_distance;
}

// World recertification and command sampling remain active independently of
// this gate. The expensive optimizer is deferred only while the exact MAIN
// bundle still has enough certified time before its declared BACKUP switch (or
// main-only endpoint) to cover one scheduling interval, one complete solve
// deadline, and the future hot-stitch interval.
inline PlannerRenewalDecision classifyPlannerRenewal(
    bool forced_transition, bool command_available,
    bool safety_suffix_active, navigation_planning::CandidateRole command_role,
    bool trajectory_metadata_valid, double command_elapsed_s,
    double backup_start_s, double solve_deadline_s,
    double replan_forward_s, double scheduling_interval_s) noexcept {
  if (forced_transition) {
    return {true, PlannerRenewalReason::kForcedTransition};
  }
  if (!command_available) {
    return {true, PlannerRenewalReason::kNoCommand};
  }
  if (safety_suffix_active ||
      command_role != navigation_planning::CandidateRole::kMain) {
    return {true, PlannerRenewalReason::kSafetyRecovery};
  }
  if (!trajectory_metadata_valid || !std::isfinite(command_elapsed_s) ||
      command_elapsed_s < 0.0 || !std::isfinite(backup_start_s) ||
      backup_start_s < 0.0 || !std::isfinite(solve_deadline_s) ||
      solve_deadline_s <= 0.0 || !std::isfinite(replan_forward_s) ||
      replan_forward_s <= 0.0 || !std::isfinite(scheduling_interval_s) ||
      scheduling_interval_s <= 0.0) {
    return {true, PlannerRenewalReason::kInvalidHorizon};
  }

  // Renew before the command enters its braking/backup phase.  The two
  // forward intervals cover the committed future splice and solver timing.
  // Candidate admission independently enforces its derived MAIN reserve
  // contract; the scheduler must not invent a second reserve value here.
  const long double lead_time =
      static_cast<long double>(solve_deadline_s) +
      2.0L * static_cast<long double>(replan_forward_s) +
      static_cast<long double>(scheduling_interval_s) +
      static_cast<long double>(
          navigation_planning::PlanningTimingContract::kCommitGuardS);
  const long double remaining =
      static_cast<long double>(backup_start_s) -
      static_cast<long double>(command_elapsed_s);
  if (!std::isfinite(lead_time) || !std::isfinite(remaining) ||
      lead_time <= 0.0L) {
    return {true, PlannerRenewalReason::kInvalidHorizon};
  }

  const auto remaining_s = static_cast<double>(remaining);
  const auto lead_time_s = static_cast<double>(lead_time);
  if (!std::isfinite(remaining_s) || !std::isfinite(lead_time_s)) {
    return {true, PlannerRenewalReason::kInvalidHorizon};
  }
  // Both elapsed and backup-start originate from a nanosecond command clock.
  // At their exact boundary, decimal-to-binary rounding may differ by a few
  // ulps; one nanosecond resolves that representation ambiguity only toward an
  // earlier, conservative renewal.
  constexpr long double kCommandClockResolutionSeconds = 1.0e-9L;
  if (remaining <= lead_time + kCommandClockResolutionSeconds) {
    return {true, PlannerRenewalReason::kRenewalDue,
            remaining_s, lead_time_s};
  }
  return {false, PlannerRenewalReason::kRetainCertifiedMain,
          remaining_s, lead_time_s};
}

inline PlannerResultDisposition classifyPlannerResult(
    navigation_planning::PlannerStatus result, bool plan_from_rest, bool command_available,
    bool commit_observed) {
  using Status = navigation_planning::PlannerStatus;
  switch (result) {
    case Status::kSuccess:
    case Status::kFinished:
      if (commit_observed) return PlannerResultDisposition::CommandReady;
      // A completed replacement without an admitted candidate did not revoke
      // the incumbent. Validate it through the normal retained-command path.
      if (command_available) return PlannerResultDisposition::RetainCommittedCommand;
      return PlannerResultDisposition::FailClosed;
    case Status::kNoNeed:
      return command_available ? PlannerResultDisposition::ValidateRetainedCommand
                               : PlannerResultDisposition::FailClosed;
    case Status::kRestartFromRest:
      return PlannerResultDisposition::RestartFromRest;
    case Status::kFailed:
      // HG-023 applies to every failed replacement status, including the
      // backend's typed optimizer failure below. Retention still requires the full
      // latest-world/anchor/suffix/lease validation in RuntimeNode.
      if (command_available) return PlannerResultDisposition::RetainCommittedCommand;
      return plan_from_rest ? PlannerResultDisposition::RetryFromRest
                            : PlannerResultDisposition::FailClosed;
    case Status::kOptimizationFailed:
      return command_available ? PlannerResultDisposition::RetainCommittedCommand
                               : PlannerResultDisposition::FailClosed;
    case Status::kEmergency:
      // The retained-command path attempts the existing one-shot measured
      // emergency transition; it still fails closed if certification fails.
      return command_available ? PlannerResultDisposition::RetainCommittedCommand
                               : PlannerResultDisposition::FailClosed;
  }
  // An unknown future status cannot gain authority to revoke a certified
  // incumbent by falling through a destructive default.
  if (command_available) return PlannerResultDisposition::RetainCommittedCommand;
  return PlannerResultDisposition::FailClosed;
}

}  // namespace navigation_runtime
