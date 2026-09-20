#pragma once

#include <cstdint>
#include <mutex>

#include <navigation_planning/candidate_bundle.hpp>
#include <navigation_runtime/execution_recovery_state.hpp>

namespace navigation_runtime {

enum class ExecutionEpisodePhase : std::uint8_t {
  // Internal physical execution lifecycle. Planning-worker activity remains
  // orthogonal to this episode.
  kInitialHold = 0,
  kTrackingMain = 1,
  kTrackingBackup = 2,
  kStoppedHold = 3,
  kPx4Hold = 4,
};

// The diagnostic key execution_episode_phase is a v1 telemetry field. Keep
// its historical wire codes at this publication boundary while the internal
// lifecycle enum remains contiguous and free to evolve independently.
constexpr std::uint8_t executionEpisodePhaseTelemetryCodeV1(
    ExecutionEpisodePhase phase) noexcept {
  switch (phase) {
    case ExecutionEpisodePhase::kInitialHold:
      return 0U;
    case ExecutionEpisodePhase::kTrackingMain:
      return 2U;
    case ExecutionEpisodePhase::kTrackingBackup:
      return 3U;
    case ExecutionEpisodePhase::kStoppedHold:
      return 4U;
    case ExecutionEpisodePhase::kPx4Hold:
      return 5U;
  }
  return 0xFFU;
}

struct ExecutionEpisodeSnapshot final {
  std::uint64_t localization_epoch{0U};
  std::uint64_t goal_epoch{0U};
  std::uint64_t request_id{0U};
  // goal_epoch/request_id describe the latest accepted desired goal. The
  // active identity remains pinned to the executing command across a hot
  // retarget until the successor is actually committed.
  std::uint64_t active_goal_epoch{0U};
  std::uint64_t active_request_id{0U};
  std::uint64_t active_generation{0U};
  ExecutionEpisodePhase phase{ExecutionEpisodePhase::kInitialHold};
  bool command_available{false};
  bool failure_latched{false};
  bool safety_suffix_active{false};
  bool restart_from_rest{false};
  // Recovery policy is part of this same physical-execution snapshot. It is
  // intentionally distinct from sampled phase (for example, a terminal MAIN
  // can already be emitting STOPPED_HOLD while completion is still pending),
  // but it no longer has an independently mutable owner.
  ExecutionRecoveryState recovery_state{ExecutionRecoveryState::kInitialHold};
};

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
    const bool preserve_execution = retain_command && state_.command_available &&
        !state_.failure_latched &&
        state_.localization_epoch == localization_epoch &&
        state_.active_generation != 0U;
    state_.localization_epoch = localization_epoch;
    state_.goal_epoch = goal_epoch;
    state_.request_id = request_id;
    if (preserve_execution || (retain_command && state_.failure_latched)) {
      // Desired identity can advance before execution ownership. Preserve a
      // moving BACKUP/EMERGENCY phase, restart request, and fail-closed latch
      // until the successor is committed or a distinct reset transition runs.
      return;
    }
    state_.phase = ExecutionEpisodePhase::kInitialHold;
    state_.command_available = false;
    state_.failure_latched = false;
    state_.safety_suffix_active = false;
    state_.restart_from_rest = false;
    state_.recovery_state = retain_command
        ? ExecutionRecoveryState::kTrackMain
        : ExecutionRecoveryState::kInitialHold;
    state_.active_generation = 0U;
    state_.active_goal_epoch = 0U;
    state_.active_request_id = 0U;
  }

  bool commandCommitted(
      const navigation_planning::CandidateBundle& bundle) noexcept {
    std::lock_guard lock(mutex_);
    const bool main_would_cut_moving_suffix =
        bundle.role == navigation_planning::CandidateRole::kMain &&
        (state_.safety_suffix_active ||
         state_.recovery_state == ExecutionRecoveryState::kTrackBackup ||
         state_.recovery_state == ExecutionRecoveryState::kEmergencyBrake);
    if (state_.failure_latched || !bundleIdentityMatchesDesiredGoal(bundle) ||
        bundle.bundle_generation == 0U ||
        bundle.bundle_generation <= state_.active_generation ||
        main_would_cut_moving_suffix) {
      return false;
    }
    state_.active_generation = bundle.bundle_generation;
    state_.active_goal_epoch = bundle.goal_epoch;
    state_.active_request_id = bundle.request_id;
    state_.command_available = true;
    state_.failure_latched = false;
    state_.restart_from_rest = false;
    state_.safety_suffix_active =
        bundle.role == navigation_planning::CandidateRole::kBackup ||
        bundle.role == navigation_planning::CandidateRole::kEmergency;
    state_.phase = bundle.kind == navigation_planning::CandidateBundleKind::kEmergencyBrake
        ? ExecutionEpisodePhase::kTrackingBackup
        : ExecutionEpisodePhase::kTrackingMain;
    const auto recovery_event = bundle.role == navigation_planning::CandidateRole::kEmergency
        ? ExecutionRecoveryEvent::kEmergencyCommitted
        : bundle.role == navigation_planning::CandidateRole::kBackup
        ? ExecutionRecoveryEvent::kBackupActivated
        : ExecutionRecoveryEvent::kMainCommitted;
    state_.recovery_state = transitionExecutionRecovery(
        state_.recovery_state, recovery_event);
    return true;
  }

  bool observeRetainedCommand(
      const navigation_planning::CandidateBundle& bundle,
      bool safety_suffix_active) noexcept {
    std::lock_guard lock(mutex_);
    if (state_.failure_latched || !activeBundleIdentityMatches(bundle)) {
      return false;
    }
    state_.command_available = true;
    const bool safety_transition_already_observed =
        state_.recovery_state == ExecutionRecoveryState::kTrackBackup ||
        state_.recovery_state == ExecutionRecoveryState::kEmergencyBrake ||
        state_.safety_suffix_active;
    state_.safety_suffix_active =
        safety_suffix_active || safety_transition_already_observed;
    if (state_.safety_suffix_active ||
        bundle.role == navigation_planning::CandidateRole::kEmergency ||
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

  bool requestRestartFromRest(
      const ExecutionEpisodeSnapshot& expected) noexcept {
    std::lock_guard lock(mutex_);
    if (state_.failure_latched || !sameEpisodeIdentity(state_, expected)) {
      return false;
    }
    state_.restart_from_rest = true;
    return true;
  }

  bool clearRestartFromRest(
      const ExecutionEpisodeSnapshot& expected) noexcept {
    std::lock_guard lock(mutex_);
    if (!sameEpisodeIdentity(state_, expected)) return false;
    state_.restart_from_rest = false;
    return true;
  }

  bool applyRecoveryEvent(
      ExecutionRecoveryEvent event,
      const navigation_planning::CandidateBundle& bundle) noexcept {
    std::lock_guard lock(mutex_);
    if (state_.failure_latched || !activeBundleIdentityMatches(bundle)) return false;
    state_.recovery_state = transitionExecutionRecovery(
        state_.recovery_state, event);
    if (event == ExecutionRecoveryEvent::kCertifiedStopObserved) {
      // Measured stop drains safety ownership, but does not claim that the
      // command publisher has entered its explicit STOPPED_HOLD phase.
      state_.safety_suffix_active = false;
    }
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

  bool suspendCommand(const ExecutionEpisodeSnapshot& expected) noexcept {
    std::lock_guard lock(mutex_);
    if (state_.failure_latched || !sameEpisodeIdentity(state_, expected)) {
      return false;
    }
    state_.command_available = false;
    return true;
  }

  void clearGoal(std::uint64_t localization_epoch) noexcept {
    std::lock_guard lock(mutex_);
    state_ = {};
    state_.localization_epoch = localization_epoch;
    state_.phase = ExecutionEpisodePhase::kInitialHold;
    state_.recovery_state = ExecutionRecoveryState::kInitialHold;
  }

 private:
  [[nodiscard]] bool bundleIdentityMatchesDesiredGoal(
      const navigation_planning::CandidateBundle& bundle) const noexcept {
    return bundle.localization_epoch != 0U && bundle.goal_epoch != 0U &&
           bundle.request_id != 0U && bundle.bundle_generation != 0U &&
           state_.localization_epoch == bundle.localization_epoch &&
           state_.goal_epoch == bundle.goal_epoch &&
           state_.request_id == bundle.request_id;
  }

  [[nodiscard]] bool activeBundleIdentityMatches(
      const navigation_planning::CandidateBundle& bundle) const noexcept {
    return bundle.localization_epoch == state_.localization_epoch &&
           bundle.goal_epoch == state_.active_goal_epoch &&
           bundle.request_id == state_.active_request_id &&
           bundle.bundle_generation != 0U &&
           bundle.bundle_generation == state_.active_generation;
  }

  [[nodiscard]] static bool sameEpisodeIdentity(
      const ExecutionEpisodeSnapshot& lhs,
      const ExecutionEpisodeSnapshot& rhs) noexcept {
    return lhs.localization_epoch == rhs.localization_epoch &&
           lhs.goal_epoch == rhs.goal_epoch && lhs.request_id == rhs.request_id &&
           lhs.active_goal_epoch == rhs.active_goal_epoch &&
           lhs.active_request_id == rhs.active_request_id &&
           lhs.active_generation == rhs.active_generation;
  }

  mutable std::mutex mutex_;
  ExecutionEpisodeSnapshot state_{};
};

}  // namespace navigation_runtime
