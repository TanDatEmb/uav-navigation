#pragma once

#include <cstdint>
#include <mutex>

#include <navigation_planning/candidate_bundle.hpp>

namespace navigation_runtime {

enum class ExecutionEpisodePhase : std::uint8_t {
  // These values are serialized in runtime telemetry.  Keep the historical
  // ordinals while planning-worker activity remains orthogonal to the
  // physical execution episode.
  kInitialHold = 0,
  kTrackingMain = 2,
  kTrackingBackup = 3,
  kStoppedHold = 4,
  kPx4Hold = 5,
};

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
};

// One serialized lifecycle record for an execution episode. Policy code reads
// this snapshot instead of reconstructing physical state from worker flags.
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
    if (!retain_command) state_.active_generation = 0U;
  }

  void commandCommitted(
      const navigation_planning::CandidateBundle& bundle) noexcept {
    std::lock_guard lock(mutex_);
    state_.active_generation = bundle.bundle_generation;
    state_.command_available = true;
    state_.failure_latched = false;
    state_.safety_suffix_active =
        bundle.role == navigation_planning::CandidateRole::kBackup ||
        bundle.role == navigation_planning::CandidateRole::kEmergency;
    state_.phase = bundle.kind == navigation_planning::CandidateBundleKind::kEmergencyBrake
        ? ExecutionEpisodePhase::kTrackingBackup
        : ExecutionEpisodePhase::kTrackingMain;
  }

  void roleObserved(navigation_planning::CandidateRole role,
                    std::uint64_t generation) noexcept {
    std::lock_guard lock(mutex_);
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

  void setSafetySuffix(bool active) noexcept {
    std::lock_guard lock(mutex_);
    state_.safety_suffix_active = active;
  }

  void requestRestartFromRest() noexcept {
    std::lock_guard lock(mutex_);
    state_.restart_from_rest = true;
  }

  void clearRestartFromRest() noexcept {
    std::lock_guard lock(mutex_);
    state_.restart_from_rest = false;
  }

  void stoppedHold(std::uint64_t generation) noexcept {
    std::lock_guard lock(mutex_);
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
  }

 private:
  mutable std::mutex mutex_;
  ExecutionEpisodeSnapshot state_{};
};

}  // namespace navigation_runtime
