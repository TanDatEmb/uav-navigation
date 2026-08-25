#pragma once

#include <cstdint>
#include <memory>
#include <mutex>
#include <optional>

#include <navigation_planning/candidate_bundle.hpp>
#include <navigation_world_model/world_model_view.hpp>

namespace navigation_execution {

struct CommitToken {
  navigation_world_model::WorldSnapshotIdentity world_identity;
  std::uint64_t goal_epoch{0};
  std::uint64_t transaction_id{0};
};

enum class CommitDecision : std::uint8_t {
  kCommitted,
  kNoActiveGoal,
  kWorldAdvanced,
  kGoalAdvanced,
  kInvalidCandidate,
  kCancelled,
};

// Sole owner of the product command candidate that is allowed to reach the
// sampler. Candidate construction and validation happen before tryCommit();
// the critical section only compares identities and swaps one shared pointer.
class CommittedBundleStore final {
 public:
  CommittedBundleStore() = default;
  CommittedBundleStore(const CommittedBundleStore&) = delete;
  CommittedBundleStore& operator=(const CommittedBundleStore&) = delete;

  bool setActiveGoalEpoch(std::uint64_t goal_epoch,
                          bool retain_committed_bundle = false) noexcept {
    if (goal_epoch == 0) return false;
    std::lock_guard lock(mutex_);
    if (goal_epoch < active_goal_epoch_) return false;
    active_goal_epoch_ = goal_epoch;
    if (!retain_committed_bundle) committed_.reset();
    return true;
  }

  bool publishWorldIdentity(
      const navigation_world_model::WorldSnapshotIdentity& identity) noexcept {
    if (identity.localization_epoch == 0 || identity.generation == 0 ||
        identity.revision == 0 || identity.observation_stamp_ns <= 0) {
      return false;
    }
    std::lock_guard lock(mutex_);
    if (world_identity_ && !advances(*world_identity_, identity)) return false;
    world_identity_ = identity;
    return true;
  }

  [[nodiscard]] std::shared_ptr<const navigation_planning::CandidateBundle> load()
      const noexcept {
    std::lock_guard lock(mutex_);
    return committed_;
  }

  CommitDecision tryCommit(
      const CommitToken& expected,
      std::shared_ptr<const navigation_planning::CandidateBundle> candidate) noexcept {
    if (!candidate || !candidate->valid() || expected.goal_epoch == 0 ||
        expected.transaction_id == 0) {
      return CommitDecision::kInvalidCandidate;
    }
    std::lock_guard lock(mutex_);
    if (active_goal_epoch_ == 0) return CommitDecision::kNoActiveGoal;
    if (active_goal_epoch_ != expected.goal_epoch ||
        candidate->goal_epoch != expected.goal_epoch) {
      return CommitDecision::kGoalAdvanced;
    }
    if (!world_identity_ ||
        !navigation_world_model::sameWorldSnapshotIdentity(
            *world_identity_, expected.world_identity) ||
        !navigation_world_model::sameWorldSnapshotIdentity(
            *world_identity_, candidate->world_identity)) {
      return CommitDecision::kWorldAdvanced;
    }
    committed_ = std::move(candidate);
    return CommitDecision::kCommitted;
  }

  void invalidate() noexcept {
    std::lock_guard lock(mutex_);
    committed_.reset();
  }

 private:
  [[nodiscard]] static bool advances(
      const navigation_world_model::WorldSnapshotIdentity& current,
      const navigation_world_model::WorldSnapshotIdentity& next) noexcept {
    if (next.localization_epoch != current.localization_epoch) {
      return next.localization_epoch > current.localization_epoch;
    }
    if (next.generation != current.generation) return next.generation > current.generation;
    return next.revision > current.revision &&
           next.observation_stamp_ns >= current.observation_stamp_ns;
  }

  mutable std::mutex mutex_;
  std::uint64_t active_goal_epoch_{0};
  std::optional<navigation_world_model::WorldSnapshotIdentity> world_identity_;
  std::shared_ptr<const navigation_planning::CandidateBundle> committed_;
};

}  // namespace navigation_execution
