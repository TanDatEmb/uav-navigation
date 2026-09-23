#pragma once

#include <cstdint>
#include <mutex>

#include <navigation_planning/candidate_bundle.hpp>
#include <navigation_runtime/execution_recovery_state.hpp>
#include <navigation_execution/execution_lifecycle.hpp>

namespace navigation_runtime {

using navigation_execution::ExecutionEpisodePhase;
using navigation_execution::ExecutionEpisodeSnapshot;
using navigation_execution::executionEpisodePhaseTelemetryCodeV1;

// One lifecycle record for an execution episode. Policy code reads this
// snapshot instead of reconstructing physical state from worker flags.
class ExecutionEpisode final {
 public:
  [[nodiscard]] ExecutionEpisodeSnapshot snapshot() const noexcept {
    std::lock_guard lock(mutex_);
    return state_;
  }

  void reset(std::uint64_t localization_epoch) noexcept {
    std::lock_guard lock(mutex_);
    state_ = {};
    state_.localization_epoch = localization_epoch;
    state_.phase = ExecutionEpisodePhase::kInitialHold;
    state_.recovery_state = ExecutionRecoveryState::kInitialHold;
  }

  void beginGoal(std::uint64_t localization_epoch, std::uint64_t goal_epoch,
                 std::uint64_t request_id, bool retain_command) noexcept {
    std::lock_guard lock(mutex_);
    const bool retaining_active_command = retain_command && state_.command_available &&
        state_.active_generation != 0U &&
        state_.localization_epoch == localization_epoch;
    state_.localization_epoch = localization_epoch;
    state_.goal_epoch = goal_epoch;
    state_.request_id = request_id;
    if (retaining_active_command) {
      // A hot-retarget changes desired ownership, not the identity/role of the
      // command still being sampled. Its observations remain valid until the
      // replacement command wins timeline activation.
      return;
    }
    state_.active_command_goal_epoch = 0U;
    state_.active_command_request_id = 0U;
    state_.phase = ExecutionEpisodePhase::kInitialHold;
    state_.command_available = false;
    state_.failure_latched = false;
    state_.safety_suffix_active = false;
    state_.restart_from_rest = false;
    state_.recovery_state = ExecutionRecoveryState::kInitialHold;
    state_.active_generation = 0U;
  }

  void commandCommitted(
      const navigation_planning::CandidateBundle& bundle) noexcept {
    std::lock_guard lock(mutex_);
    if (state_.failure_latched || !bundleIdentityMatchesEpisode(bundle) ||
        bundle.bundle_generation <= state_.active_generation) {
      return;
    }
    state_.active_command_goal_epoch = bundle.goal_epoch;
    state_.active_command_request_id = bundle.request_id;
    state_.active_generation = bundle.bundle_generation;
    state_.command_available = true;
    state_.failure_latched = false;
    state_.safety_suffix_active =
        bundle.role == navigation_planning::CandidateRole::kBackup ||
        bundle.role == navigation_planning::CandidateRole::kEmergency;
    state_.phase = bundle.role == navigation_planning::CandidateRole::kEmergency ||
        bundle.role == navigation_planning::CandidateRole::kBackup
        ? ExecutionEpisodePhase::kTrackingBackup
        : ExecutionEpisodePhase::kTrackingMain;
    const auto recovery_event = bundle.role == navigation_planning::CandidateRole::kEmergency
        ? ExecutionRecoveryEvent::kEmergencyCommitted
        : bundle.role == navigation_planning::CandidateRole::kBackup
        ? ExecutionRecoveryEvent::kBackupActivated
        : ExecutionRecoveryEvent::kMainCommitted;
    state_.recovery_state = transitionExecutionRecovery(
        state_.recovery_state, recovery_event);
  }

  bool observeRetainedCommand(
      const navigation_planning::CandidateBundle& bundle,
      bool safety_suffix_active) noexcept {
    std::lock_guard lock(mutex_);
    if (state_.failure_latched || !activeBundleIdentityMatches(bundle)) {
      return false;
    }
    state_.command_available = true;
    state_.safety_suffix_active = safety_suffix_active;
    if (bundle.role == navigation_planning::CandidateRole::kEmergency ||
        bundle.role == navigation_planning::CandidateRole::kBackup) {
      state_.phase = ExecutionEpisodePhase::kTrackingBackup;
    } else {
      state_.phase = ExecutionEpisodePhase::kTrackingMain;
    }
    return true;
  }

  bool observeSampledSafetyRole(
      const navigation_planning::CandidateBundle& bundle,
      navigation_planning::CandidateRole sampled_role) noexcept {
    std::lock_guard lock(mutex_);
    if (state_.failure_latched || !activeBundleIdentityMatches(bundle) ||
        (sampled_role != navigation_planning::CandidateRole::kBackup &&
         sampled_role != navigation_planning::CandidateRole::kEmergency)) {
      return false;
    }
    state_.phase = ExecutionEpisodePhase::kTrackingBackup;
    state_.safety_suffix_active = true;
    const auto event = sampled_role == navigation_planning::CandidateRole::kEmergency
        ? ExecutionRecoveryEvent::kEmergencyCommitted
        : ExecutionRecoveryEvent::kBackupActivated;
    state_.recovery_state = transitionExecutionRecovery(
        state_.recovery_state, event);
    return true;
  }

  bool preserveSafetySuffix(
      const navigation_planning::CandidateBundle& bundle) noexcept {
    std::lock_guard lock(mutex_);
    if (state_.failure_latched || !state_.command_available ||
        !activeBundleIdentityMatches(bundle)) {
      return false;
    }
    state_.safety_suffix_active = true;
    return true;
  }

  bool requestRestartFromRest(
      const navigation_planning::CandidateBundle& bundle) noexcept {
    std::lock_guard lock(mutex_);
    if (state_.failure_latched || !activeBundleIdentityMatches(bundle)) return false;
    state_.restart_from_rest = true;
    return true;
  }

  void clearRestartFromRest() noexcept {
    std::lock_guard lock(mutex_);
    state_.restart_from_rest = false;
  }

  bool applyRecoveryEvent(
      ExecutionRecoveryEvent event,
      const navigation_planning::CandidateBundle& bundle) noexcept {
    std::lock_guard lock(mutex_);
    if (state_.failure_latched || !activeBundleIdentityMatches(bundle)) return false;
    state_.recovery_state = transitionExecutionRecovery(
        state_.recovery_state, event);
    return true;
  }

  bool stoppedHold(
      const navigation_planning::CandidateBundle& bundle) noexcept {
    std::lock_guard lock(mutex_);
    if (state_.failure_latched || !activeBundleIdentityMatches(bundle)) return false;
    // A stopped hold is also the command publisher's representation while a
    // measured-state PlanFromRest retry is in flight.  Do not erase that
    // lifecycle request from the 50 Hz hold samples; otherwise the next
    // non-zero odometry sample is misclassified as motion without an
    // authorized recovery and the node falls through to PX4 Hold.
    const bool restart_requested = state_.restart_from_rest;
    state_.active_generation = bundle.bundle_generation;
    state_.phase = ExecutionEpisodePhase::kStoppedHold;
    state_.command_available = true;
    state_.safety_suffix_active = false;
    state_.restart_from_rest = restart_requested;
    return true;
  }

  void failClosed() noexcept {
    std::lock_guard lock(mutex_);
    state_.phase = ExecutionEpisodePhase::kPx4Hold;
    state_.command_available = false;
    state_.failure_latched = true;
    state_.safety_suffix_active = false;
    state_.restart_from_rest = false;
    state_.recovery_state = ExecutionRecoveryState::kPx4Hold;
  }

  void suspendCommand() noexcept {
    std::lock_guard lock(mutex_);
    state_.command_available = false;
  }

  void clearGoal(std::uint64_t localization_epoch) noexcept {
    std::lock_guard lock(mutex_);
    state_ = {};
    state_.localization_epoch = localization_epoch;
    state_.phase = ExecutionEpisodePhase::kInitialHold;
    state_.recovery_state = ExecutionRecoveryState::kInitialHold;
  }

 private:
  [[nodiscard]] bool bundleIdentityMatchesEpisode(
      const navigation_planning::CandidateBundle& bundle) const noexcept {
    return bundle.localization_epoch != 0U && bundle.goal_epoch != 0U &&
           bundle.request_id != 0U && bundle.bundle_generation != 0U &&
           state_.localization_epoch == bundle.localization_epoch &&
           state_.goal_epoch == bundle.goal_epoch &&
           state_.request_id == bundle.request_id;
  }

  [[nodiscard]] bool activeBundleIdentityMatches(
      const navigation_planning::CandidateBundle& bundle) const noexcept {
    return bundle.localization_epoch != 0U && bundle.goal_epoch != 0U &&
           bundle.request_id != 0U && bundle.bundle_generation != 0U &&
           state_.localization_epoch == bundle.localization_epoch &&
           state_.active_command_goal_epoch == bundle.goal_epoch &&
           state_.active_command_request_id == bundle.request_id &&
           state_.active_generation == bundle.bundle_generation;
  }

  mutable std::mutex mutex_;
  ExecutionEpisodeSnapshot state_{};
};

}  // namespace navigation_runtime
