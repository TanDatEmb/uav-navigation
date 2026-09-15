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
    state_.localization_epoch = localization_epoch;
    state_.goal_epoch = goal_epoch;
    state_.request_id = request_id;
    state_.phase = retain_command ? ExecutionEpisodePhase::kTrackingMain
                                  : ExecutionEpisodePhase::kInitialHold;
    state_.command_available = retain_command;
    state_.failure_latched = false;
    state_.safety_suffix_active = false;
    state_.restart_from_rest = false;
    state_.recovery_state = retain_command
        ? ExecutionRecoveryState::kTrackMain
        : ExecutionRecoveryState::kInitialHold;
    if (!retain_command) state_.active_generation = 0U;
  }

  void commandCommitted(
      const navigation_planning::CandidateBundle& bundle) noexcept {
    std::lock_guard lock(mutex_);
    if (state_.failure_latched) return;
    state_.active_generation = bundle.bundle_generation;
    state_.command_available = true;
    state_.failure_latched = false;
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
  }

  void roleObserved(navigation_planning::CandidateRole role,
                    std::uint64_t generation) noexcept {
    std::lock_guard lock(mutex_);
    if (state_.failure_latched) return;
    state_.active_generation = generation;
    state_.command_available = true;
    if (role == navigation_planning::CandidateRole::kEmergency ||
        role == navigation_planning::CandidateRole::kBackup) {
      state_.phase = ExecutionEpisodePhase::kTrackingBackup;
      state_.safety_suffix_active = true;
    } else {
      state_.phase = ExecutionEpisodePhase::kTrackingMain;
    }
  }

  void sampledSafetyRoleObserved(
      navigation_planning::CandidateRole role,
      std::uint64_t generation) noexcept {
    std::lock_guard lock(mutex_);
    if (state_.failure_latched || state_.active_generation != generation ||
        (role != navigation_planning::CandidateRole::kBackup &&
         role != navigation_planning::CandidateRole::kEmergency)) {
      return;
    }
    state_.phase = ExecutionEpisodePhase::kTrackingBackup;
    state_.safety_suffix_active = true;
    const auto event = role == navigation_planning::CandidateRole::kEmergency
        ? ExecutionRecoveryEvent::kEmergencyCommitted
        : ExecutionRecoveryEvent::kBackupActivated;
    state_.recovery_state = transitionExecutionRecovery(
        state_.recovery_state, event);
  }

  void setSafetySuffix(bool active) noexcept {
    std::lock_guard lock(mutex_);
    if (state_.failure_latched) return;
    state_.safety_suffix_active = active;
  }

  void requestRestartFromRest() noexcept {
    std::lock_guard lock(mutex_);
    if (state_.failure_latched) return;
    state_.restart_from_rest = true;
  }

  void clearRestartFromRest() noexcept {
    std::lock_guard lock(mutex_);
    state_.restart_from_rest = false;
  }

  void applyRecoveryEvent(ExecutionRecoveryEvent event) noexcept {
    std::lock_guard lock(mutex_);
    state_.recovery_state = transitionExecutionRecovery(
        state_.recovery_state, event);
  }

  void stoppedHold(std::uint64_t generation) noexcept {
    std::lock_guard lock(mutex_);
    if (state_.failure_latched) return;
    // A stopped hold is also the command publisher's representation while a
    // measured-state PlanFromRest retry is in flight.  Do not erase that
    // lifecycle request from the 50 Hz hold samples; otherwise the next
    // non-zero odometry sample is misclassified as motion without an
    // authorized recovery and the node falls through to PX4 Hold.
    const bool restart_requested = state_.restart_from_rest;
    state_.active_generation = generation;
    state_.phase = ExecutionEpisodePhase::kStoppedHold;
    state_.command_available = true;
    state_.safety_suffix_active = false;
    state_.restart_from_rest = restart_requested;
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
  mutable std::mutex mutex_;
  ExecutionEpisodeSnapshot state_{};
};

}  // namespace navigation_runtime
