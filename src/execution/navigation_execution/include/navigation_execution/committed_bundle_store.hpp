#pragma once

#include <cmath>
#include <cstdint>
#include <functional>
#include <memory>
#include <mutex>
#include <optional>
#include <limits>

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

  // A measured pass-through acceptance changes the mission identity without
  // changing the physical command that is already being executed. Rebind a
  // retained immutable bundle to that new identity only when the caller
  // supplies the exact previous identity. The world certificate, executable
  // interval and evaluator are copied unchanged; this operation does not
  // extend or otherwise alter the certified command.
  bool rebindRetainedBundle(std::uint64_t new_goal_epoch,
                            std::uint64_t new_request_id,
                            std::uint64_t previous_goal_epoch) noexcept {
    if (new_goal_epoch == 0U || new_request_id == 0U ||
        previous_goal_epoch == 0U) {
      return false;
    }
    std::lock_guard lock(mutex_);
    if (active_goal_epoch_ != new_goal_epoch || !committed_ ||
        committed_->goal_epoch != previous_goal_epoch) {
      return false;
    }
    auto rebound = std::make_shared<navigation_planning::CandidateBundle>(*committed_);
    rebound->goal_epoch = new_goal_epoch;
    rebound->request_id = new_request_id;
    if (!rebound->valid()) {
      committed_.reset();
      return false;
    }
    committed_ = std::shared_ptr<const navigation_planning::CandidateBundle>(
        std::move(rebound));
    return true;
  }

  bool publishWorldIdentity(
      const navigation_world_model::WorldSnapshotIdentity& identity) noexcept {
    return publishWorldIdentity(identity, {}, false);
  }

  // Replace the world certificate for the currently exposed bundle only after
  // an external validator has checked that bundle against the new immutable
  // snapshot.  Pointer identity is part of the precondition: if a newer
  // bundle was committed while validation was running, the new world must not
  // inherit a certificate that was never checked against it.
  bool publishWorldIdentity(
      const navigation_world_model::WorldSnapshotIdentity& identity,
      const std::shared_ptr<const navigation_planning::CandidateBundle>&
          expected_bundle,
      bool retain_validated_bundle,
      std::int64_t refreshed_valid_until_ns = 0) noexcept {
    if (identity.localization_epoch == 0 || identity.generation == 0 ||
        identity.revision == 0 || identity.observation_stamp_ns <= 0) {
      return false;
    }
    std::lock_guard lock(mutex_);
    if (world_identity_ && !advances(*world_identity_, identity)) return false;
    if (retain_validated_bundle && expected_bundle && committed_ &&
        committed_.get() == expected_bundle.get() && world_identity_ &&
        active_goal_epoch_ == expected_bundle->goal_epoch &&
        navigation_world_model::sameWorldSnapshotIdentity(
            *world_identity_, expected_bundle->world_identity)) {
      auto recertified = std::make_shared<navigation_planning::CandidateBundle>(
          *committed_);
      recertified->world_identity = identity;
      if (refreshed_valid_until_ns > recertified->valid_until_ns) {
        // A revalidated bundle may continue to be sampled only for the new
        // fresh-world window. Never extend it beyond the declared trajectory
        // endpoint, and never widen the interval without a successful full
        // candidate validation in the caller.
        auto renewed_until_ns = refreshed_valid_until_ns;
        if (recertified->hasDeclaredEndpointMetadata()) {
          const double end_wall_time_s = recertified->start_wall_time_s +
              recertified->duration_s;
          if (!std::isfinite(end_wall_time_s) || end_wall_time_s <= 0.0 ||
              end_wall_time_s > static_cast<double>(
                  std::numeric_limits<std::int64_t>::max()) * 1.0e-9) {
            return false;
          }
          const auto endpoint_ns = static_cast<std::int64_t>(end_wall_time_s * 1.0e9);
          renewed_until_ns = std::min(renewed_until_ns, endpoint_ns);
        }
        recertified->valid_until_ns = renewed_until_ns;
      }
      committed_ = recertified->valid()
          ? std::shared_ptr<const navigation_planning::CandidateBundle>(
                std::move(recertified))
          : nullptr;
    } else {
      // A candidate is certified against one immutable snapshot.  If no
      // matching validation certificate was supplied for the newer snapshot,
      // clear it rather than retaining an uncertified command.
      committed_.reset();
    }
    world_identity_ = identity;
    return true;
  }

  [[nodiscard]] std::shared_ptr<const navigation_planning::CandidateBundle> load()
      const noexcept {
    std::lock_guard lock(mutex_);
    return committed_;
  }

  // Execute the exposure callback while the same transaction lock protects
  // the committed bundle, goal epoch and world identity.  A sampler may have
  // loaded a shared_ptr just before a map update invalidated it; pointer and
  // identity revalidation at this boundary prevents that stale command from
  // reaching the transport.
  template <typename ExposureFn>
  bool publishIfCurrent(
      const std::shared_ptr<const navigation_planning::CandidateBundle>& expected,
      std::uint64_t expected_goal_epoch,
      ExposureFn&& expose) noexcept {
    std::lock_guard lock(mutex_);
    if (!expected || !committed_ || committed_.get() != expected.get() ||
        active_goal_epoch_ != expected_goal_epoch || !world_identity_ ||
        !navigation_world_model::sameWorldSnapshotIdentity(
            *world_identity_, expected->world_identity)) {
      return false;
    }
    try {
      std::forward<ExposureFn>(expose)();
    } catch (...) {
      return false;
    }
    return true;
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
