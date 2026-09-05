#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <functional>
#include <memory>
#include <mutex>
#include <optional>
#include <limits>
#include <utility>

#include <navigation_common/time.hpp>
#include <navigation_planning/candidate_bundle.hpp>
#include <navigation_execution/execution_anchor.hpp>
#include <navigation_world_model/world_commit_authorizer.hpp>
#include <navigation_world_model/world_model_view.hpp>

namespace navigation_execution {

struct CommitToken {
  navigation_world_model::WorldSnapshotIdentity world_identity;
  std::uint64_t goal_epoch{0};
  std::uint64_t transaction_id{0};
};

struct ExecutionTimelineSnapshot {
  std::uint64_t version{0};
  std::optional<navigation_world_model::WorldSnapshotIdentity> world_identity;
  std::shared_ptr<const navigation_planning::CandidateBundle> active;
  std::shared_ptr<const navigation_planning::CandidateBundle> pending;
  std::int64_t pending_activation_ns{0};
};

enum class CommitDecision : std::uint8_t {
  kCommitted,
  kNoActiveGoal,
  kWorldAdvanced,
  kGoalAdvanced,
  kInvalidCandidate,
  kCancelled,
  // The execution pointer was restored because the post-commit finalizer
  // could not complete.  This is distinct from a stale/cancelled candidate so
  // callers can account for an internal transaction fault without treating
  // the previous command as lost.
  kFinalizationFailed,
};

enum class StageDecision : std::uint8_t {
  kStaged,
  kNoActiveGoal,
  kWorldAdvanced,
  kGoalAdvanced,
  kInvalidCandidate,
  kInvalidAnchor,
  kActivationTooLate,
  kCancelled,
  kFinalizationFailed,
  // The execution command used to produce the successor anchor is no longer
  // the current predecessor.  This is distinct from a goal/world rejection:
  // the same mission and world may still be active while a newer bundle has
  // replaced the predecessor. Keep this appended so existing diagnostic
  // ordinals remain stable.
  kPredecessorAdvanced,
};

// Sole owner of the product command candidate that is allowed to reach the
// sampler. Candidate construction and validation happen before tryCommit();
// the store critical section compares identities and swaps one shared pointer.
class ExecutionTimelineStore final {
 public:
  ExecutionTimelineStore() = default;
  ExecutionTimelineStore(const ExecutionTimelineStore&) = delete;
  ExecutionTimelineStore& operator=(const ExecutionTimelineStore&) = delete;

  bool setActiveGoalEpoch(std::uint64_t goal_epoch,
                          bool retain_committed_bundle = false) noexcept {
    if (goal_epoch == 0) return false;
    std::lock_guard lock(mutex_);
    if (goal_epoch < active_goal_epoch_) return false;
    active_goal_epoch_ = goal_epoch;
    if (!retain_committed_bundle) committed_.reset();
    pending_.reset();
    pending_activation_ns_ = 0;
    ++timeline_version_;
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
      std::int64_t refreshed_valid_until_ns = 0,
      const std::shared_ptr<const navigation_planning::CandidateBundle>&
          expected_pending = {},
      bool retain_validated_pending = false) noexcept {
    if (identity.localization_epoch == 0 || identity.generation == 0 ||
        identity.revision == 0 || identity.observation_stamp_ns <= 0) {
      return false;
    }
    std::lock_guard lock(mutex_);
    if (world_identity_ && !advances(*world_identity_, identity)) return false;
    if (retain_validated_bundle && expected_bundle && committed_ &&
        committed_.get() == expected_bundle.get() && world_identity_ &&
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
          const auto endpoint_ns = navigation_common::secondsSumToNanoseconds(
              recertified->start_wall_time_s, recertified->duration_s);
          if (!endpoint_ns || *endpoint_ns <= 0) {
            return false;
          }
          renewed_until_ns = std::min(renewed_until_ns, *endpoint_ns);
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
    const bool pending_recertified = retain_validated_pending && expected_pending &&
        pending_ && pending_.get() == expected_pending.get() &&
        navigation_world_model::sameWorldSnapshotIdentity(
            pending_->world_identity, identity) && pending_->valid();
    // A pending successor was certified against the previous immutable
    // snapshot. It may survive only when the caller has proven that all
    // changes in the new immutable snapshot are disjoint from its protected
    // execution region and has already replaced its world identity.
    if (!pending_recertified) {
      pending_.reset();
      pending_activation_ns_ = 0;
    }
    world_identity_ = identity;
    ++timeline_version_;
    return true;
  }

  // Apply a world refresh only if the exact execution timeline observed before
  // validation is still current.  The active and pending pointers are
  // checked independently: an invalid pending successor must not discard a
  // valid active command.  A superseded refresh is a no-op.
  navigation_world_model::WorldCommitDecision publishWorldIdentityIfCurrent(
      const navigation_world_model::WorldSnapshotIdentity& identity,
      std::uint64_t expected_timeline_version,
      const std::shared_ptr<const navigation_planning::CandidateBundle>& expected_bundle,
      bool retain_validated_bundle,
      std::int64_t refreshed_valid_until_ns = 0,
      const std::shared_ptr<const navigation_planning::CandidateBundle>& expected_pending = {},
      bool retain_validated_pending = false) noexcept {
    if (identity.localization_epoch == 0U || identity.generation == 0U ||
        identity.revision == 0U || identity.observation_stamp_ns <= 0) {
      return navigation_world_model::WorldCommitDecision::kCandidateRejected;
    }
    std::lock_guard lock(mutex_);
    if (timeline_version_ != expected_timeline_version) {
      return navigation_world_model::WorldCommitDecision::kSuperseded;
    }
    if (world_identity_ && !advances(*world_identity_, identity)) {
      return navigation_world_model::WorldCommitDecision::kWorldAdvanced;
    }

    const bool active_matches = retain_validated_bundle && expected_bundle && committed_ &&
        committed_.get() == expected_bundle.get() && world_identity_ &&
        navigation_world_model::sameWorldSnapshotIdentity(
            *world_identity_, expected_bundle->world_identity);
    if (active_matches) {
      auto recertified = std::make_shared<navigation_planning::CandidateBundle>(*committed_);
      recertified->world_identity = identity;
      if (refreshed_valid_until_ns > recertified->valid_until_ns) {
        auto renewed_until_ns = refreshed_valid_until_ns;
        if (recertified->hasDeclaredEndpointMetadata()) {
          const auto endpoint_ns = navigation_common::secondsSumToNanoseconds(
              recertified->start_wall_time_s, recertified->duration_s);
          if (!endpoint_ns || *endpoint_ns <= 0) {
            return navigation_world_model::WorldCommitDecision::kCandidateRejected;
          }
          renewed_until_ns = std::min(renewed_until_ns, *endpoint_ns);
        }
        recertified->valid_until_ns = renewed_until_ns;
      }
      if (!recertified->valid()) {
        return navigation_world_model::WorldCommitDecision::kCandidateRejected;
      }
      committed_ = std::shared_ptr<const navigation_planning::CandidateBundle>(
          std::move(recertified));
    } else {
      committed_.reset();
    }

    const bool pending_matches = retain_validated_pending && expected_pending && pending_ &&
        pending_.get() == expected_pending.get() && pending_->valid() && world_identity_ &&
        navigation_world_model::sameWorldSnapshotIdentity(
            pending_->world_identity, *world_identity_);
    if (pending_matches) {
      auto recertified = std::make_shared<navigation_planning::CandidateBundle>(*pending_);
      recertified->world_identity = identity;
      if (!recertified->valid()) {
        pending_.reset();
        pending_activation_ns_ = 0;
      } else {
        pending_ = std::shared_ptr<const navigation_planning::CandidateBundle>(
            std::move(recertified));
      }
    } else {
      pending_.reset();
      pending_activation_ns_ = 0;
    }
    world_identity_ = identity;
    ++timeline_version_;
    return navigation_world_model::WorldCommitDecision::kCommitted;
  }

  // Replace a pending successor's world identity after an external immutable
  // snapshot has proved its protected region unchanged. The proof belongs to
  // the world-view owner; this method only linearizes the exact pointer and
  // identity transition so an older pending candidate cannot be relabelled.
  std::optional<std::shared_ptr<const navigation_planning::CandidateBundle>>
  recertifyPendingWorldIdentity(
      const navigation_world_model::WorldSnapshotIdentity& identity,
      const std::shared_ptr<const navigation_planning::CandidateBundle>&
          expected_pending) noexcept {
    if (identity.localization_epoch == 0U || identity.generation == 0U ||
        identity.revision == 0U || identity.observation_stamp_ns <= 0 ||
        !expected_pending) {
      return std::nullopt;
    }
    std::lock_guard lock(mutex_);
    if (!world_identity_ || !pending_ || pending_.get() != expected_pending.get() ||
        !pending_->valid() ||
        !navigation_world_model::sameWorldSnapshotIdentity(
            pending_->world_identity, *world_identity_) ||
        identity.localization_epoch != world_identity_->localization_epoch ||
        identity.generation != world_identity_->generation ||
        !advances(*world_identity_, identity)) {
      return std::nullopt;
    }
    auto recertified = std::make_shared<navigation_planning::CandidateBundle>(*pending_);
    recertified->world_identity = identity;
    if (!recertified->valid()) return std::nullopt;
    pending_ = std::move(recertified);
    return pending_;
  }

  [[nodiscard]] std::shared_ptr<const navigation_planning::CandidateBundle> load()
      const noexcept {
    std::lock_guard lock(mutex_);
    return committed_;
  }

  [[nodiscard]] ExecutionTimelineSnapshot snapshot() const noexcept {
    std::lock_guard lock(mutex_);
    return {timeline_version_, world_identity_, committed_, pending_,
            pending_activation_ns_};
  }

  [[nodiscard]] std::shared_ptr<const navigation_planning::CandidateBundle>
  loadPending() const noexcept {
    std::lock_guard lock(mutex_);
    return pending_;
  }

  [[nodiscard]] bool hasPending() const noexcept {
    std::lock_guard lock(mutex_);
    return static_cast<bool>(pending_);
  }

  [[nodiscard]] std::int64_t pendingActivationNs() const noexcept {
    std::lock_guard lock(mutex_);
    return pending_activation_ns_;
  }

  // Sample the execution-owned active command at a future splice point. The
  // old command must remain valid through the point; otherwise the planner
  // receives no anchor and the runtime fails closed.
  [[nodiscard]] std::optional<ExecutionAnchor> reserveAnchor(
      std::int64_t request_stamp_ns, std::int64_t activation_stamp_ns) const noexcept {
    if (request_stamp_ns <= 0 || activation_stamp_ns < request_stamp_ns) return std::nullopt;
    std::lock_guard lock(mutex_);
    if (!committed_ || !world_identity_ ||
        activation_stamp_ns < committed_->valid_from_ns ||
        activation_stamp_ns > committed_->valid_until_ns) {
      return std::nullopt;
    }
    const auto point = committed_->sample(activation_stamp_ns);
    if (!point) return std::nullopt;
    const auto end_ns = committed_->declared_end_ns > 0
        ? committed_->declared_end_ns : committed_->valid_until_ns;
    const auto main_end_ns = committed_->backup_available
        ? navigation_common::secondsSumToNanoseconds(
              committed_->start_wall_time_s, committed_->backup_start_time_s)
        : std::optional<std::int64_t>{end_ns};
    if (!main_end_ns || *main_end_ns < activation_stamp_ns || end_ns < activation_stamp_ns) {
      return std::nullopt;
    }
    ExecutionAnchor anchor;
    anchor.active_bundle_generation = committed_->bundle_generation;
    anchor.localization_epoch = committed_->localization_epoch;
    anchor.goal_epoch = committed_->goal_epoch;
    anchor.request_id = committed_->request_id;
    anchor.request_stamp_ns = request_stamp_ns;
    anchor.activation_stamp_ns = activation_stamp_ns;
    anchor.state = *point;
    anchor.active_role = point->role;
    anchor.active_main_end_ns = *main_end_ns;
    anchor.active_bundle_end_ns = end_ns;
    anchor.command_world = committed_->world_identity;
    return anchor.valid() ? std::optional<ExecutionAnchor>{std::move(anchor)} : std::nullopt;
  }

  // Stage, but do not expose, a complete successor. The transaction watermark
  // is consumed at staging so an older result cannot overwrite a newer
  // pending command while activation is waiting for the reserved boundary.
  StageDecision stagePending(
      const CommitToken& expected, const ExecutionAnchor& anchor,
      std::shared_ptr<const navigation_planning::CandidateBundle> candidate) noexcept {
    if (!candidate || !candidate->valid() || expected.goal_epoch == 0U ||
        expected.transaction_id == 0U) return StageDecision::kInvalidCandidate;
    if (!anchor.valid() || candidate->valid_from_ns != anchor.activation_stamp_ns ||
        candidate->activation_stamp_ns != anchor.activation_stamp_ns ||
        candidate->localization_epoch != anchor.localization_epoch ||
        !navigation_world_model::sameWorldSnapshotIdentity(
            candidate->world_identity, expected.world_identity)) {
      return StageDecision::kInvalidAnchor;
    }
    std::lock_guard lock(mutex_);
    if (active_goal_epoch_ == 0U) return StageDecision::kNoActiveGoal;
    if (active_goal_epoch_ != expected.goal_epoch ||
        candidate->goal_epoch != expected.goal_epoch) return StageDecision::kGoalAdvanced;
    if (!world_identity_ ||
        !navigation_world_model::sameWorldSnapshotIdentity(
            *world_identity_, expected.world_identity)) return StageDecision::kWorldAdvanced;
    if (expected.transaction_id <= last_transaction_id_) return StageDecision::kCancelled;
    if (anchor.active_main_end_ns < anchor.activation_stamp_ns ||
        candidate->valid_until_ns < anchor.activation_stamp_ns) {
      return StageDecision::kActivationTooLate;
    }
    if (!committed_ || !predecessorMatchesAnchor(*committed_, anchor)) {
      return StageDecision::kPredecessorAdvanced;
    }
    pending_ = std::move(candidate);
    pending_activation_ns_ = anchor.activation_stamp_ns;
    last_transaction_id_ = expected.transaction_id;
    ++timeline_version_;
    return StageDecision::kStaged;
  }

  template <typename FinalizeFn>
  StageDecision stagePendingAndFinalize(
      const CommitToken& expected, const ExecutionAnchor& anchor,
      std::shared_ptr<const navigation_planning::CandidateBundle> candidate,
      FinalizeFn&& finalize) noexcept {
    std::lock_guard lock(mutex_);
    if (!candidate || !candidate->valid() || expected.goal_epoch == 0U ||
        expected.transaction_id == 0U) return StageDecision::kInvalidCandidate;
    if (!anchor.valid() || candidate->valid_from_ns != anchor.activation_stamp_ns ||
        candidate->activation_stamp_ns != anchor.activation_stamp_ns ||
        candidate->localization_epoch != anchor.localization_epoch ||
        !navigation_world_model::sameWorldSnapshotIdentity(
            candidate->world_identity, expected.world_identity)) {
      return StageDecision::kInvalidAnchor;
    }
    if (active_goal_epoch_ == 0U) return StageDecision::kNoActiveGoal;
    if (active_goal_epoch_ != expected.goal_epoch ||
        candidate->goal_epoch != expected.goal_epoch) return StageDecision::kGoalAdvanced;
    if (!world_identity_ ||
        !navigation_world_model::sameWorldSnapshotIdentity(
            *world_identity_, expected.world_identity)) return StageDecision::kWorldAdvanced;
    if (expected.transaction_id <= last_transaction_id_) return StageDecision::kCancelled;
    if (anchor.active_main_end_ns < anchor.activation_stamp_ns ||
        candidate->valid_until_ns < anchor.activation_stamp_ns) {
      return StageDecision::kActivationTooLate;
    }
    if (!committed_ || !predecessorMatchesAnchor(*committed_, anchor)) {
      return StageDecision::kPredecessorAdvanced;
    }
    const auto previous_pending = pending_;
    const auto previous_pending_activation = pending_activation_ns_;
    const auto previous_transaction_id = last_transaction_id_;
    pending_ = std::move(candidate);
    pending_activation_ns_ = anchor.activation_stamp_ns;
    last_transaction_id_ = expected.transaction_id;
    ++timeline_version_;
    bool finalized = false;
    try {
      finalized = static_cast<bool>(std::forward<FinalizeFn>(finalize)());
    } catch (...) {
      finalized = false;
    }
    if (!finalized) {
      pending_ = previous_pending;
      pending_activation_ns_ = previous_pending_activation;
      last_transaction_id_ = previous_transaction_id;
      ++timeline_version_;
      return StageDecision::kFinalizationFailed;
    }
    return StageDecision::kStaged;
  }

  // The command timer calls this operation before sampling. No callback or
  // planner code can replace active outside this single atomic boundary.
  template <typename FinalizeFn>
  bool activatePendingIfDueAndFinalize(
      std::int64_t now_ns, FinalizeFn&& finalize) const noexcept {
    std::lock_guard lock(mutex_);
    if (!pending_ || pending_activation_ns_ <= 0 || now_ns < pending_activation_ns_) {
      return false;
    }
    if (!world_identity_ ||
        !navigation_world_model::sameWorldSnapshotIdentity(
            *world_identity_, pending_->world_identity) ||
        active_goal_epoch_ != pending_->goal_epoch ||
        now_ns > pending_->valid_until_ns || !pending_->valid() || !committed_ ||
        !committed_->valid() ||
        !navigation_world_model::sameWorldSnapshotIdentity(
            *world_identity_, committed_->world_identity)) {
      pending_.reset();
      pending_activation_ns_ = 0;
      return false;
    }
    const auto previous = committed_;
    const auto successor = pending_;
    committed_ = successor;
    pending_.reset();
    pending_activation_ns_ = 0;
    ++timeline_version_;
    bool finalized = false;
    try {
      finalized = static_cast<bool>(std::forward<FinalizeFn>(finalize)(
          successor->bundle_generation));
    } catch (...) {
      finalized = false;
    }
    if (!finalized) {
      committed_ = previous;
      // The finalize callback owns an external transaction (planner history,
      // for example). Once it rejects, retaining the pending pointer would
      // leave an unfinalizable candidate parked ahead of future solves.
      pending_.reset();
      pending_activation_ns_ = 0;
      ++timeline_version_;
      return false;
    }
    return true;
  }

  bool activatePendingIfDue(std::int64_t now_ns) const noexcept {
    return activatePendingIfDueAndFinalize(now_ns, [](std::uint64_t) {
      return true;
    });
  }

  // Execute the exposure callback while the same transaction lock protects
  // the committed bundle, goal epoch and world identity. A sampler may have
  // loaded a shared_ptr just before a map update invalidated it; pointer and
  // identity revalidation at this boundary prevents that stale command from
  // reaching the transport. The callback is intentionally inside the lock so
  // invalidation cannot complete before an already-authorized exposure; the
  // caller must keep this callback bounded because it serializes store writes.
  template <typename ExposureFn>
  bool publishIfCurrent(
      const std::shared_ptr<const navigation_planning::CandidateBundle>& expected,
      std::uint64_t expected_goal_epoch,
      ExposureFn&& expose) noexcept {
    std::lock_guard lock(mutex_);
    if (!expected || !committed_ || committed_.get() != expected.get() ||
        committed_->goal_epoch != expected_goal_epoch || !world_identity_ ||
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
    if (expected.transaction_id <= last_transaction_id_) {
      return CommitDecision::kCancelled;
    }
    committed_ = std::move(candidate);
    pending_.reset();
    pending_activation_ns_ = 0;
    last_transaction_id_ = expected.transaction_id;
    ++timeline_version_;
    return CommitDecision::kCommitted;
  }

  // Commit the execution candidate and run the planner-history finalizer as
  // one rollback-safe transaction.  The execution pointer remains the sole
  // authority: if the cache/history update fails, restore the exact previous
  // pointer and transaction watermark instead of invalidating a command that
  // was already accepted for execution.
  template <typename FinalizeFn>
  CommitDecision tryCommitAndFinalize(
      const CommitToken& expected,
      std::shared_ptr<const navigation_planning::CandidateBundle> candidate,
      FinalizeFn&& finalize) noexcept {
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
    if (expected.transaction_id <= last_transaction_id_) {
      return CommitDecision::kCancelled;
    }

    const auto previous = committed_;
    const auto previous_pending = pending_;
    const auto previous_pending_activation = pending_activation_ns_;
    const auto previous_transaction_id = last_transaction_id_;
    committed_ = std::move(candidate);
    pending_.reset();
    pending_activation_ns_ = 0;
    last_transaction_id_ = expected.transaction_id;
    ++timeline_version_;
    bool finalized = false;
    try {
      finalized = static_cast<bool>(std::forward<FinalizeFn>(finalize)());
    } catch (...) {
      finalized = false;
    }
    if (!finalized) {
      committed_ = previous;
      pending_ = previous_pending;
      pending_activation_ns_ = previous_pending_activation;
      last_transaction_id_ = previous_transaction_id;
      ++timeline_version_;
      return CommitDecision::kFinalizationFailed;
    }
    return CommitDecision::kCommitted;
  }

  void invalidate() noexcept {
    std::lock_guard lock(mutex_);
    committed_.reset();
    pending_.reset();
    pending_activation_ns_ = 0;
    ++timeline_version_;
  }

 private:
  // Validate the immutable predecessor anchor without invoking its evaluator
  // while holding the store mutex. All operations which replace or invalidate
  // semantic replacements of committed_ clear pending_; a world-only
  // recertification may copy the same trajectory identity. This stage check
  // plus activation's current active/world check closes the interleaving
  // without a second witness field.
  [[nodiscard]] static bool predecessorMatchesAnchor(
      const navigation_planning::CandidateBundle& predecessor,
      const ExecutionAnchor& anchor) noexcept {
    if (!anchor.valid() || !predecessor.valid() ||
        predecessor.bundle_generation != anchor.active_bundle_generation ||
        predecessor.localization_epoch != anchor.localization_epoch ||
        predecessor.goal_epoch != anchor.goal_epoch ||
        predecessor.request_id != anchor.request_id ||
        !navigation_world_model::sameWorldSnapshotIdentity(
            predecessor.world_identity, anchor.command_world)) {
      return false;
    }
    const auto start_ns = navigation_common::secondsToNanoseconds(
        predecessor.start_wall_time_s);
    const auto end_ns = predecessor.declared_end_ns > 0
        ? std::optional<std::int64_t>{predecessor.declared_end_ns}
        : std::optional<std::int64_t>{predecessor.valid_until_ns};
    const auto main_end_ns = predecessor.backup_available
        ? navigation_common::secondsSumToNanoseconds(
              predecessor.start_wall_time_s, predecessor.backup_start_time_s)
        : end_ns;
    if (!start_ns || !main_end_ns || !end_ns ||
        anchor.activation_stamp_ns < predecessor.valid_from_ns ||
        anchor.activation_stamp_ns > predecessor.valid_until_ns ||
        *start_ns > anchor.activation_stamp_ns ||
        *main_end_ns != anchor.active_main_end_ns ||
        *end_ns != anchor.active_bundle_end_ns ||
        anchor.activation_stamp_ns > *end_ns) {
      return false;
    }
    const auto elapsed_ns = anchor.activation_stamp_ns - *start_ns;
    if (elapsed_ns < 0) return false;
    const auto scheduled = predecessor.scheduledRole(
        static_cast<double>(elapsed_ns) * 1.0e-9);
    return scheduled.has_value() && *scheduled == anchor.active_role;
  }

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
  mutable std::uint64_t timeline_version_{0};
  std::uint64_t last_transaction_id_{0};
  std::optional<navigation_world_model::WorldSnapshotIdentity> world_identity_;
  mutable std::shared_ptr<const navigation_planning::CandidateBundle> committed_;
  mutable std::shared_ptr<const navigation_planning::CandidateBundle> pending_;
  mutable std::int64_t pending_activation_ns_{0};
};

// Compatibility name for code that only consumes the active-command API.
using CommittedBundleStore = ExecutionTimelineStore;

}  // namespace navigation_execution
