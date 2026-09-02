#pragma once

#include <cstdint>
#include <mutex>

#include <navigation_planning/candidate_bundle.hpp>

namespace navigation_runtime {

enum class ExecutionEpisodePhase : std::uint8_t {
  kInitialHold,
  kPlanning,
  kTrackingMain,
  kTrackingBackup,
  kStoppedHold,
  kPx4Hold,
};

struct ExecutionEpisodeSnapshot final {
  std::uint64_t localization_epoch{0U};
  std::uint64_t goal_epoch{0U};
  std::uint64_t request_id{0U};
  std::uint64_t active_generation{0U};
  ExecutionEpisodePhase phase{ExecutionEpisodePhase::kInitialHold};
  bool command_available{false};
  bool failure_latched{false};
};

// One serialized lifecycle record for an execution episode. The legacy atomics
// remain diagnostic compatibility mirrors during migration; policy code can
// progressively consume this snapshot without reconstructing state from
// unrelated flags.
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
    if (!retain_command) state_.active_generation = 0U;
  }

  void planningStarted() noexcept {
    std::lock_guard lock(mutex_);
    if (state_.phase != ExecutionEpisodePhase::kPx4Hold) {
      state_.phase = ExecutionEpisodePhase::kPlanning;
    }
  }

  void commandCommitted(
      const navigation_planning::CandidateBundle& bundle) noexcept {
    std::lock_guard lock(mutex_);
    state_.active_generation = bundle.bundle_generation;
    state_.command_available = true;
    state_.failure_latched = false;
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
    } else {
      state_.phase = ExecutionEpisodePhase::kTrackingMain;
    }
  }

  void stoppedHold(std::uint64_t generation) noexcept {
    std::lock_guard lock(mutex_);
    state_.active_generation = generation;
    state_.phase = ExecutionEpisodePhase::kStoppedHold;
    state_.command_available = true;
  }

  void failClosed() noexcept {
    std::lock_guard lock(mutex_);
    state_.phase = ExecutionEpisodePhase::kPx4Hold;
    state_.command_available = false;
    state_.failure_latched = true;
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
