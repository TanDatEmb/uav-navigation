#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <functional>
#include <memory>
#include <mutex>
#include <optional>
#include <limits>
#include <type_traits>
#include <utility>

#include <navigation_common/time.hpp>
#include <navigation_contracts/msg/navigation_goal.hpp>
#include <navigation_execution/execution_lifecycle.hpp>
#include <navigation_planning/candidate_bundle.hpp>
#include <navigation_execution/execution_anchor.hpp>
#include <navigation_world_model/world_commit_authorizer.hpp>
#include <navigation_world_model/world_model_view.hpp>

namespace navigation_execution {

struct ExecutionTimelineStoreTestAccess;

struct CommitToken {
  navigation_world_model::WorldSnapshotIdentity world_identity;
  std::uint64_t goal_epoch{0};
  std::uint64_t transaction_id{0};
};

struct ExecutionAuthoritySnapshot {
  std::uint64_t version{0};
  std::optional<navigation_world_model::WorldSnapshotIdentity> world_identity;
  std::shared_ptr<const navigation_planning::CandidateBundle> active;
  std::shared_ptr<const navigation_planning::CandidateBundle> pending;
  std::int64_t pending_activation_ns{0};
  // Read-only projections of the same active/staged records. The bundle
  // alone does not contain the mission, waypoint behavior or route payload.
  std::shared_ptr<const navigation_contracts::msg::NavigationGoal> active_goal;
  std::shared_ptr<const navigation_contracts::msg::NavigationGoal> pending_goal;
  std::uint64_t active_lineage{0};
  // Admission scope is deliberately separate from the active bundle epoch.
  std::uint64_t admission_goal_epoch{0};
  std::uint64_t admission_localization_epoch{0};
  ExecutionLifecycleState lifecycle{};
};

using ExecutionTimelineSnapshot = ExecutionAuthoritySnapshot;

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
  // Exact predecessor/pending ownership changed before conditional admission.
  // Append decisions to preserve existing diagnostic ordinals.
  kPredecessorAdvanced,
  // The bounded admission predicate returned false or threw. No timeline or
  // transaction-watermark mutation occurred.
  kAdmissionRejected,
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
class ExecutionAuthority {
 public:
  ExecutionAuthority() = default;
  ExecutionAuthority(const ExecutionAuthority&) = delete;
  ExecutionAuthority& operator=(const ExecutionAuthority&) = delete;

  bool setAdmissionGoalEpoch(std::uint64_t goal_epoch,
                          bool retain_committed_bundle = false) noexcept {
    if (goal_epoch == 0) return false;
    std::lock_guard lock(mutex_);
    if (goal_epoch < admission_goal_epoch_) return false;
    admission_goal_epoch_ = goal_epoch;
    if (world_identity_) {
      admission_localization_epoch_ = world_identity_->localization_epoch;
    }
    if (!retain_committed_bundle && active_.bundle) {
      clearActiveLocked();
      ++active_lineage_version_;
    }
    if (!retain_committed_bundle) lifecycle_ = {};
    clearStagedLocked();
    staged_.activation_ns = 0;
    enforceInvariantLocked();
    ++timeline_version_;
    return true;
  }

  // Desired/admission scope advances independently of the active predecessor.
  // This is one owner mutation, including lifecycle retention or reset.
  bool beginGoal(std::uint64_t localization_epoch, std::uint64_t goal_epoch,
                 bool retain_active) noexcept {
    if (localization_epoch == 0U || goal_epoch == 0U) return false;
    std::lock_guard lock(mutex_);
    if (goal_epoch < admission_goal_epoch_ ||
        localization_epoch < admission_localization_epoch_) return false;
    const bool retain = retain_active && active_.bundle && active_.goal &&
        lifecycle_.exposure == ExecutionExposure::kAvailable &&
        active_.bundle->localization_epoch == localization_epoch;
    admission_localization_epoch_ = localization_epoch;
    admission_goal_epoch_ = goal_epoch;
    if (!retain) {
      if (active_.bundle) ++active_lineage_version_;
      clearActiveLocked();
      lifecycle_ = {};
    }
    clearStagedLocked();
    ++timeline_version_;
    return true;
  }

  void reset(std::uint64_t localization_epoch) noexcept {
    std::lock_guard lock(mutex_);
    if (active_.bundle) ++active_lineage_version_;
    clearActiveLocked();
    clearStagedLocked();
    admission_localization_epoch_ = localization_epoch;
    admission_goal_epoch_ = 0U;
    lifecycle_ = {};
    ++timeline_version_;
  }

  void clearGoal(std::uint64_t localization_epoch) noexcept {
    reset(localization_epoch);
  }

  void failClosed() noexcept {
    std::lock_guard lock(mutex_);
    const auto before = lifecycle_;
    failClosedLifecycleLocked();
    noteLifecycleChangeLocked(before);
  }

  void suspendCommand() noexcept {
    std::lock_guard lock(mutex_);
    if (lifecycle_.exposure != ExecutionExposure::kFailed) {
      const auto before = lifecycle_;
      lifecycle_.exposure = ExecutionExposure::kSuspended;
      noteLifecycleChangeLocked(before);
    }
  }

  bool observeRetainedCommand(
      const navigation_planning::CandidateBundle& bundle,
      bool safety_suffix_active) noexcept {
    std::lock_guard lock(mutex_);
    if (lifecycle_.exposure == ExecutionExposure::kFailed ||
        !activeBundleIdentityMatchesLocked(bundle)) return false;
    const auto before = lifecycle_;
    lifecycle_.exposure = ExecutionExposure::kAvailable;
    lifecycle_.safety = safety_suffix_active
        ? ExecutionSafetyOwnership::kSafetySuffix
        : ExecutionSafetyOwnership::kNominal;
    lifecycle_.phase = bundle.role == navigation_planning::CandidateRole::kEmergency ||
                       bundle.role == navigation_planning::CandidateRole::kBackup
        ? ExecutionEpisodePhase::kTrackingBackup
        : ExecutionEpisodePhase::kTrackingMain;
    noteLifecycleChangeLocked(before);
    return true;
  }

  bool observeSampledSafetyRole(
      const navigation_planning::CandidateBundle& bundle,
      navigation_planning::CandidateRole sampled_role) noexcept {
    std::lock_guard lock(mutex_);
    if (lifecycle_.exposure == ExecutionExposure::kFailed ||
        !activeBundleIdentityMatchesLocked(bundle) ||
        (sampled_role != navigation_planning::CandidateRole::kBackup &&
         sampled_role != navigation_planning::CandidateRole::kEmergency)) return false;
    const auto before = lifecycle_;
    lifecycle_.phase = ExecutionEpisodePhase::kTrackingBackup;
    lifecycle_.safety = ExecutionSafetyOwnership::kSafetySuffix;
    lifecycle_.recovery = transitionExecutionRecovery(
        lifecycle_.recovery,
        sampled_role == navigation_planning::CandidateRole::kEmergency
            ? ExecutionRecoveryEvent::kEmergencyCommitted
            : ExecutionRecoveryEvent::kBackupActivated);
    noteLifecycleChangeLocked(before);
    return true;
  }

  bool preserveSafetySuffix(
      const navigation_planning::CandidateBundle& bundle) noexcept {
    std::lock_guard lock(mutex_);
    if (lifecycle_.exposure != ExecutionExposure::kAvailable ||
        !activeBundleIdentityMatchesLocked(bundle)) return false;
    const auto before = lifecycle_;
    lifecycle_.safety = ExecutionSafetyOwnership::kSafetySuffix;
    noteLifecycleChangeLocked(before);
    return true;
  }

  bool requestRestartFromRest(
      const navigation_planning::CandidateBundle& bundle) noexcept {
    std::lock_guard lock(mutex_);
    if (lifecycle_.exposure == ExecutionExposure::kFailed ||
        !activeBundleIdentityMatchesLocked(bundle)) return false;
    const auto before = lifecycle_;
    lifecycle_.restart = ExecutionRestartRequest::kFromRest;
    noteLifecycleChangeLocked(before);
    return true;
  }

  void clearRestartFromRest() noexcept {
    std::lock_guard lock(mutex_);
    if (lifecycle_.restart != ExecutionRestartRequest::kNone) {
      const auto before = lifecycle_;
      lifecycle_.restart = ExecutionRestartRequest::kNone;
      noteLifecycleChangeLocked(before);
    }
  }

  bool applyRecoveryEvent(
      ExecutionRecoveryEvent event,
      const navigation_planning::CandidateBundle& bundle) noexcept {
    std::lock_guard lock(mutex_);
    if (lifecycle_.exposure == ExecutionExposure::kFailed ||
        !activeBundleIdentityMatchesLocked(bundle)) return false;
    const auto before = lifecycle_;
    lifecycle_.recovery = transitionExecutionRecovery(lifecycle_.recovery, event);
    noteLifecycleChangeLocked(before);
    return true;
  }

  bool stoppedHold(const navigation_planning::CandidateBundle& bundle) noexcept {
    std::lock_guard lock(mutex_);
    if (lifecycle_.exposure == ExecutionExposure::kFailed ||
        !activeBundleIdentityMatchesLocked(bundle)) return false;
    const auto before = lifecycle_;
    lifecycle_.phase = ExecutionEpisodePhase::kStoppedHold;
    lifecycle_.exposure = ExecutionExposure::kAvailable;
    lifecycle_.safety = ExecutionSafetyOwnership::kNominal;
    // Deliberately preserve restart request and recovery policy.
    noteLifecycleChangeLocked(before);
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
    return publishWorldIdentityIfCurrentAndFinalizeRevocation(
        identity, expected_timeline_version, expected_bundle,
        retain_validated_bundle, []() noexcept {}, refreshed_valid_until_ns,
        expected_pending, retain_validated_pending);
  }

  // The owner holds its lifecycle locks before entering the world publication
  // gate. This callback must be noexcept, bounded and must not re-enter the
  // store or a planner backend. It finalizes only this exact active revocation,
  // not a superseded publication or a rejected pending successor.
  template <typename FinalizeRevocation>
  navigation_world_model::WorldCommitDecision
  publishWorldIdentityIfCurrentAndFinalizeRevocation(
      const navigation_world_model::WorldSnapshotIdentity& identity,
      std::uint64_t expected_timeline_version,
      const std::shared_ptr<const navigation_planning::CandidateBundle>& expected_bundle,
      bool retain_validated_bundle,
      FinalizeRevocation&& finalize_revocation,
      std::int64_t refreshed_valid_until_ns = 0,
      const std::shared_ptr<const navigation_planning::CandidateBundle>& expected_pending = {},
      bool retain_validated_pending = false) noexcept {
    static_assert(std::is_nothrow_invocable_v<FinalizeRevocation&>);
    if (identity.localization_epoch == 0U || identity.generation == 0U ||
        identity.revision == 0U || identity.observation_stamp_ns <= 0) {
      return navigation_world_model::WorldCommitDecision::kCandidateRejected;
    }
    const auto prepare = [](const navigation_planning::CandidateBundle& source,
                            const navigation_world_model::WorldSnapshotIdentity& next,
                            const std::int64_t refreshed_until_ns)
        -> std::shared_ptr<const navigation_planning::CandidateBundle> {
      auto copy = std::make_shared<navigation_planning::CandidateBundle>(source);
      copy->world_identity = next;
      if (refreshed_until_ns > copy->valid_until_ns) {
        auto renewed_until_ns = refreshed_until_ns;
        if (copy->hasDeclaredEndpointMetadata()) {
          const auto endpoint_ns = navigation_common::secondsSumToNanoseconds(
              copy->start_wall_time_s, copy->duration_s);
          if (!endpoint_ns || *endpoint_ns <= 0) return {};
          renewed_until_ns = std::min(renewed_until_ns, *endpoint_ns);
        }
        copy->valid_until_ns = renewed_until_ns;
      }
      if (!copy->valid()) return {};
      return std::shared_ptr<const navigation_planning::CandidateBundle>(
          std::move(copy));
    };
    return publishWorldIdentityIfCurrentAndFinalizeRevocationImpl(
        identity, expected_timeline_version, expected_bundle,
        retain_validated_bundle, std::forward<FinalizeRevocation>(finalize_revocation),
        refreshed_valid_until_ns, expected_pending, retain_validated_pending, prepare);
  }

  [[nodiscard]] std::shared_ptr<const navigation_planning::CandidateBundle> load()
      const noexcept {
    std::lock_guard lock(mutex_);
    return active_.bundle;
  }

  [[nodiscard]] ExecutionAuthoritySnapshot snapshot() const noexcept {
    std::lock_guard lock(mutex_);
    return {timeline_version_, world_identity_, active_.bundle, staged_.bundle,
            staged_.activation_ns, active_.goal, staged_.goal,
            active_lineage_version_, admission_goal_epoch_,
            admission_localization_epoch_, lifecycle_};
  }

  [[nodiscard]] ExecutionEpisodeSnapshot episodeSnapshot() const noexcept {
    std::lock_guard lock(mutex_);
    return episodeSnapshotLocked();
  }

  [[nodiscard]] std::shared_ptr<const navigation_contracts::msg::NavigationGoal>
  executingGoal() const noexcept {
    std::lock_guard lock(mutex_);
    return active_.goal;
  }

  [[nodiscard]] std::uint64_t executingGoalEpoch() const noexcept {
    std::lock_guard lock(mutex_);
    return active_.bundle ? active_.bundle->goal_epoch : 0U;
  }

  [[nodiscard]] bool invariantHolds() const noexcept {
    std::lock_guard lock(mutex_);
    return (!staged_.bundle || static_cast<bool>(active_.bundle)) &&
           static_cast<bool>(active_.goal) == static_cast<bool>(active_.bundle) &&
           static_cast<bool>(staged_.goal) == static_cast<bool>(staged_.bundle);
  }

  // Sample the execution-owned active command at a future splice point. The
  // old command must remain valid through the point; otherwise the planner
  // receives no anchor and the runtime fails closed.
  [[nodiscard]] std::optional<ExecutionAnchor> reserveAnchor(
      std::int64_t request_stamp_ns, std::int64_t activation_stamp_ns) const noexcept {
    if (request_stamp_ns <= 0 || activation_stamp_ns < request_stamp_ns) return std::nullopt;
    std::shared_ptr<const navigation_planning::CandidateBundle> predecessor;
    navigation_world_model::WorldSnapshotIdentity expected_world;
    std::uint64_t expected_version = 0U;
    std::uint64_t expected_lineage_version = 0U;
    {
      std::lock_guard lock(mutex_);
      if (!active_.bundle || !world_identity_ ||
          !navigation_world_model::sameWorldSnapshotIdentity(
              *world_identity_, active_.bundle->world_identity) ||
          activation_stamp_ns < active_.bundle->valid_from_ns ||
          activation_stamp_ns > active_.bundle->valid_until_ns) {
        return std::nullopt;
      }
      predecessor = active_.bundle;
      expected_world = *world_identity_;
      expected_version = timeline_version_;
      expected_lineage_version = active_lineage_version_;
    }

    // The pinned evaluator is not declared noexcept and may own an expensive
    // backend value. Evaluate it outside the store mutex, convert any failure
    // to an unavailable anchor, then require the exact timeline to still be
    // current before returning the witness.
    std::optional<ExecutionAnchor> result;
    try {
      const auto point = predecessor->sample(activation_stamp_ns);
      if (!point) return std::nullopt;
      const auto end_ns = predecessor->declared_end_ns > 0
          ? predecessor->declared_end_ns : predecessor->valid_until_ns;
      const auto main_end_ns = predecessor->backup_available
          ? navigation_common::secondsSumToNanoseconds(
                predecessor->start_wall_time_s, predecessor->backup_start_time_s)
          : std::optional<std::int64_t>{end_ns};
      if (!main_end_ns || *main_end_ns < activation_stamp_ns ||
          end_ns < activation_stamp_ns) {
        return std::nullopt;
      }
      ExecutionAnchor anchor;
      anchor.active_bundle_generation = predecessor->bundle_generation;
      anchor.execution_lineage_version = expected_lineage_version;
      anchor.localization_epoch = predecessor->localization_epoch;
      anchor.goal_epoch = predecessor->goal_epoch;
      anchor.request_id = predecessor->request_id;
      anchor.request_stamp_ns = request_stamp_ns;
      anchor.activation_stamp_ns = activation_stamp_ns;
      anchor.state = *point;
      anchor.active_role = point->role;
      anchor.active_main_end_ns = *main_end_ns;
      anchor.active_bundle_end_ns = end_ns;
      anchor.command_world = predecessor->world_identity;
      if (!anchor.valid()) return std::nullopt;
      result = std::move(anchor);
    } catch (...) {
      return std::nullopt;
    }

    std::lock_guard lock(mutex_);
    if (timeline_version_ != expected_version || !active_.bundle ||
        active_.bundle.get() != predecessor.get() || !world_identity_ ||
        !navigation_world_model::sameWorldSnapshotIdentity(
            *world_identity_, expected_world)) {
      return std::nullopt;
    }
    return result;
  }

  // Stage, but do not expose, a complete successor. The transaction watermark
  // is consumed at staging so an older result cannot overwrite a newer
  // pending command while activation is waiting for the reserved boundary.
  StageDecision stagePending(
      const CommitToken& expected, const ExecutionAnchor& anchor,
      std::shared_ptr<const navigation_contracts::msg::NavigationGoal> goal,
      std::shared_ptr<const navigation_planning::CandidateBundle> candidate) noexcept {
    if (!goalMatchesCandidate(goal, candidate) || !candidate->valid() || expected.goal_epoch == 0U ||
        expected.transaction_id == 0U) return StageDecision::kInvalidCandidate;
    if (!anchor.valid() || candidate->valid_from_ns != anchor.activation_stamp_ns ||
        candidate->activation_stamp_ns != anchor.activation_stamp_ns ||
        candidate->localization_epoch != anchor.localization_epoch ||
        !navigation_world_model::sameWorldSnapshotIdentity(
            candidate->world_identity, expected.world_identity)) {
      return StageDecision::kInvalidAnchor;
    }
    std::lock_guard lock(mutex_);
    if (admission_goal_epoch_ == 0U) return StageDecision::kNoActiveGoal;
    if (lifecycle_.exposure == ExecutionExposure::kFailed) return StageDecision::kCancelled;
    if (admission_goal_epoch_ != expected.goal_epoch ||
        candidate->goal_epoch != expected.goal_epoch) return StageDecision::kGoalAdvanced;
    if (!world_identity_ ||
        !navigation_world_model::sameWorldSnapshotIdentity(
            *world_identity_, expected.world_identity)) return StageDecision::kWorldAdvanced;
    if (expected.transaction_id <= last_transaction_id_) return StageDecision::kCancelled;
    if (anchor.active_main_end_ns < anchor.activation_stamp_ns ||
        candidate->valid_until_ns < anchor.activation_stamp_ns) {
      return StageDecision::kActivationTooLate;
    }
    if (active_lineage_version_ != anchor.execution_lineage_version || !active_.bundle ||
        !predecessorMatchesAnchor(*active_.bundle, anchor)) {
      return StageDecision::kPredecessorAdvanced;
    }
    staged_.goal = std::move(goal);
    staged_.bundle = std::move(candidate);
    staged_.activation_ns = anchor.activation_stamp_ns;
    last_transaction_id_ = expected.transaction_id;
    ++timeline_version_;
    return StageDecision::kStaged;
  }

  template <typename FinalizeFn>
  StageDecision stagePendingAndFinalize(
      const CommitToken& expected, const ExecutionAnchor& anchor,
      std::shared_ptr<const navigation_contracts::msg::NavigationGoal> goal,
      std::shared_ptr<const navigation_planning::CandidateBundle> candidate,
      FinalizeFn&& finalize) noexcept {
    std::lock_guard lock(mutex_);
    if (!goalMatchesCandidate(goal, candidate) || !candidate->valid() || expected.goal_epoch == 0U ||
        expected.transaction_id == 0U) return StageDecision::kInvalidCandidate;
    if (!anchor.valid() || candidate->valid_from_ns != anchor.activation_stamp_ns ||
        candidate->activation_stamp_ns != anchor.activation_stamp_ns ||
        candidate->localization_epoch != anchor.localization_epoch ||
        !navigation_world_model::sameWorldSnapshotIdentity(
            candidate->world_identity, expected.world_identity)) {
      return StageDecision::kInvalidAnchor;
    }
    if (admission_goal_epoch_ == 0U) return StageDecision::kNoActiveGoal;
    if (lifecycle_.exposure == ExecutionExposure::kFailed) return StageDecision::kCancelled;
    if (admission_goal_epoch_ != expected.goal_epoch ||
        candidate->goal_epoch != expected.goal_epoch) return StageDecision::kGoalAdvanced;
    if (!world_identity_ ||
        !navigation_world_model::sameWorldSnapshotIdentity(
            *world_identity_, expected.world_identity)) return StageDecision::kWorldAdvanced;
    if (expected.transaction_id <= last_transaction_id_) return StageDecision::kCancelled;
    if (anchor.active_main_end_ns < anchor.activation_stamp_ns ||
        candidate->valid_until_ns < anchor.activation_stamp_ns) {
      return StageDecision::kActivationTooLate;
    }
    if (active_lineage_version_ != anchor.execution_lineage_version || !active_.bundle ||
        !predecessorMatchesAnchor(*active_.bundle, anchor)) {
      return StageDecision::kPredecessorAdvanced;
    }
    const auto previous_pending_goal = staged_.goal;
    const auto previous_pending = staged_.bundle;
    const auto previous_pending_activation = staged_.activation_ns;
    const auto previous_transaction_id = last_transaction_id_;
    staged_.goal = std::move(goal);
    staged_.bundle = std::move(candidate);
    staged_.activation_ns = anchor.activation_stamp_ns;
    last_transaction_id_ = expected.transaction_id;
    ++timeline_version_;
    bool finalized = false;
    try {
      finalized = static_cast<bool>(std::forward<FinalizeFn>(finalize)());
    } catch (...) {
      finalized = false;
    }
    if (!finalized) {
      staged_.goal = previous_pending_goal;
      staged_.bundle = previous_pending;
      staged_.activation_ns = previous_pending_activation;
      last_transaction_id_ = previous_transaction_id;
      ++timeline_version_;
      return StageDecision::kFinalizationFailed;
    }
    return StageDecision::kStaged;
  }

  // The command timer calls this operation before sampling. No callback or
  // planner code can replace active outside this single atomic boundary.
  // The exact timeline snapshot is a transaction token. Its version,
  // pending pointer/generation and activation instant must still match under
  // the store mutex before any activation or rollback is attempted.
  template <typename FinalizeFn>
  bool activatePendingIfDueAndFinalize(
      std::int64_t now_ns, const ExecutionTimelineSnapshot& expected,
      FinalizeFn&& finalize) const noexcept {
    std::lock_guard lock(mutex_);
    return activatePendingIfDueAndFinalizeLocked(
        now_ns, expected.pending, expected.version, expected.pending_activation_ns,
        true, std::forward<FinalizeFn>(finalize));
  }

  // Execute the exposure callback while the same transaction lock protects
  // the committed bundle, goal epoch and world identity. A sampler may have
  // loaded a shared_ptr just before a map update invalidated it; pointer and
  // identity revalidation at this boundary prevents that stale command from
  // reaching the transport. The callback is intentionally inside the lock so
  // invalidation cannot complete before an already-authorized exposure; the
  // caller must keep this callback bounded because it serializes store writes.
  // Returning false rejects exposure at this exact linearization point; a
  // freshness or transport lease checked before waiting for this mutex is not
  // sufficient authorization.
  template <typename ExposureFn>
  bool publishIfCurrent(
      const std::shared_ptr<const navigation_planning::CandidateBundle>& expected,
      std::uint64_t expected_goal_epoch,
      ExposureFn&& expose) noexcept {
    std::lock_guard lock(mutex_);
    if (!expected || !active_.bundle || active_.bundle.get() != expected.get() ||
        lifecycle_.exposure != ExecutionExposure::kAvailable ||
        active_.bundle->goal_epoch != expected_goal_epoch || !world_identity_ ||
        !navigation_world_model::sameWorldSnapshotIdentity(
            *world_identity_, expected->world_identity)) {
      return false;
    }
    try {
      if constexpr (std::is_invocable_v<ExposureFn&,
                                        const ExecutionLifecycleState&>) {
        if (!static_cast<bool>(std::invoke(expose, lifecycle_))) return false;
      } else {
        if (!static_cast<bool>(std::invoke(expose))) return false;
      }
    } catch (...) {
      return false;
    }
    return true;
  }

  // Revoke the exact timeline observed by an execution owner. A changed
  // version means a commit, activation, recertification or pending mutation
  // won the race; preserve that newer timeline rather than clearing it.
  bool invalidateIfCurrent(const ExecutionTimelineSnapshot& expected) const noexcept {
    std::lock_guard lock(mutex_);
    if (timeline_version_ != expected.version ||
        active_.bundle.get() != expected.active.get()) {
      return false;
    }
    clearActiveLocked();
    ++active_lineage_version_;
    clearStagedLocked();
    staged_.activation_ns = 0;
    ++timeline_version_;
    return true;
  }

  CommitDecision tryCommit(
      const CommitToken& expected,
      std::shared_ptr<const navigation_contracts::msg::NavigationGoal> goal,
      std::shared_ptr<const navigation_planning::CandidateBundle> candidate) noexcept {
    if (!goalMatchesCandidate(goal, candidate) || !candidate->valid() || expected.goal_epoch == 0 ||
        expected.transaction_id == 0) {
      return CommitDecision::kInvalidCandidate;
    }
    std::lock_guard lock(mutex_);
    if (admission_goal_epoch_ == 0) return CommitDecision::kNoActiveGoal;
    if (lifecycle_.exposure == ExecutionExposure::kFailed) return CommitDecision::kCancelled;
    if (admission_goal_epoch_ != expected.goal_epoch ||
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
    // Planner generations are monotonic. A late result may never replace the
    // active record with an older generation using a newer queue ID. Exact
    // same-generation identity-preserving replacements remain legal.
    if (active_.bundle &&
        candidate->bundle_generation < active_.bundle->bundle_generation) {
      return CommitDecision::kPredecessorAdvanced;
    }
    const auto prior_generation = active_.bundle
        ? active_.bundle->bundle_generation : 0U;
    active_.goal = std::move(goal);
    active_.bundle = std::move(candidate);
    committedLifecycleLocked(*active_.bundle, prior_generation);
    ++active_lineage_version_;
    clearStagedLocked();
    staged_.activation_ns = 0;
    last_transaction_id_ = expected.transaction_id;
    ++timeline_version_;
    return CommitDecision::kCommitted;
  }

  // An immediate replacement prepared outside the store lock may depend on
  // both an exact predecessor and a phase/deadline that can expire while it
  // waits. Check both at the cutover, before any mutation. Unlike a rollback
  // finalizer, rejected admission neither advances the timeline/lineage nor
  // consumes the transaction watermark.
  //
  // admit MUST be bounded: only read a clock/captured metadata; no allocation,
  // I/O, owner/backend/world locks, clock updates, or store re-entry. A clock
  // exception is an explicit rejection, not termination. Validation and
  // candidate construction still happen outside this critical section.
  template <typename AdmissionFn>
  CommitDecision tryCommitIfCurrent(
      const CommitToken& expected,
      const ExecutionTimelineSnapshot& predecessor,
      std::shared_ptr<const navigation_contracts::msg::NavigationGoal> goal,
      std::shared_ptr<const navigation_planning::CandidateBundle> candidate,
      AdmissionFn&& admit) noexcept {
    if (!goalMatchesCandidate(goal, candidate) || !candidate->valid() || expected.goal_epoch == 0U ||
        expected.transaction_id == 0U) {
      return CommitDecision::kInvalidCandidate;
    }
    std::lock_guard lock(mutex_);
    if (admission_goal_epoch_ == 0U) return CommitDecision::kNoActiveGoal;
    if (lifecycle_.exposure == ExecutionExposure::kFailed) return CommitDecision::kCancelled;
    if (admission_goal_epoch_ != expected.goal_epoch ||
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
    // Planner generations are monotonic. A late result may never replace the
    // active record with an older generation using a newer queue ID. Exact
    // same-generation identity-preserving replacements remain legal.
    if (active_.bundle &&
        candidate->bundle_generation < active_.bundle->bundle_generation) {
      return CommitDecision::kPredecessorAdvanced;
    }
    if (timeline_version_ != predecessor.version ||
        active_.bundle.get() != predecessor.active.get() ||
        staged_.bundle.get() != predecessor.pending.get() ||
        staged_.activation_ns != predecessor.pending_activation_ns) {
      return CommitDecision::kPredecessorAdvanced;
    }
    try {
      if (!static_cast<bool>(std::forward<AdmissionFn>(admit)())) {
        return CommitDecision::kAdmissionRejected;
      }
    } catch (...) {
      return CommitDecision::kAdmissionRejected;
    }
    const auto prior_generation = active_.bundle
        ? active_.bundle->bundle_generation : 0U;
    active_.goal = std::move(goal);
    active_.bundle = std::move(candidate);
    committedLifecycleLocked(*active_.bundle, prior_generation);
    ++active_lineage_version_;
    clearStagedLocked();
    staged_.activation_ns = 0;
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
      std::shared_ptr<const navigation_contracts::msg::NavigationGoal> goal,
      std::shared_ptr<const navigation_planning::CandidateBundle> candidate,
      FinalizeFn&& finalize) noexcept {
    if (!goalMatchesCandidate(goal, candidate) || !candidate->valid() || expected.goal_epoch == 0 ||
        expected.transaction_id == 0) {
      return CommitDecision::kInvalidCandidate;
    }
    std::lock_guard lock(mutex_);
    if (admission_goal_epoch_ == 0) return CommitDecision::kNoActiveGoal;
    if (lifecycle_.exposure == ExecutionExposure::kFailed) return CommitDecision::kCancelled;
    if (admission_goal_epoch_ != expected.goal_epoch ||
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
    // Planner generations are monotonic. A late result may never replace the
    // active record with an older generation using a newer queue ID. Exact
    // same-generation identity-preserving replacements remain legal.
    if (active_.bundle &&
        candidate->bundle_generation < active_.bundle->bundle_generation) {
      return CommitDecision::kPredecessorAdvanced;
    }

    const auto previous_lifecycle = lifecycle_;
    const auto previous_goal = active_.goal;
    const auto previous = active_.bundle;
    const auto previous_pending_goal = staged_.goal;
    const auto previous_pending = staged_.bundle;
    const auto previous_pending_activation = staged_.activation_ns;
    const auto previous_transaction_id = last_transaction_id_;
    active_.goal = std::move(goal);
    active_.bundle = std::move(candidate);
    committedLifecycleLocked(
        *active_.bundle, previous ? previous->bundle_generation : 0U);
    clearStagedLocked();
    staged_.activation_ns = 0;
    last_transaction_id_ = expected.transaction_id;
    ++timeline_version_;
    bool finalized = false;
    try {
      finalized = static_cast<bool>(std::forward<FinalizeFn>(finalize)());
    } catch (...) {
      finalized = false;
    }
    if (!finalized) {
      lifecycle_ = previous_lifecycle;
      active_.goal = previous_goal;
      active_.bundle = previous;
      staged_.goal = previous_pending_goal;
      staged_.bundle = previous_pending;
      staged_.activation_ns = previous_pending_activation;
      last_transaction_id_ = previous_transaction_id;
      ++timeline_version_;
      return CommitDecision::kFinalizationFailed;
    }
    ++active_lineage_version_;
    return CommitDecision::kCommitted;
  }

  void invalidate() noexcept {
    std::lock_guard lock(mutex_);
    if (active_.bundle) ++active_lineage_version_;
    clearActiveLocked();
    clearStagedLocked();
    staged_.activation_ns = 0;
    enforceInvariantLocked();
    ++timeline_version_;
  }

 private:
  friend struct ExecutionTimelineStoreTestAccess;

  void noteLifecycleChangeLocked(
      const ExecutionLifecycleState& before) const noexcept {
    if (!(lifecycle_ == before)) ++timeline_version_;
  }

  [[nodiscard]] ExecutionEpisodeSnapshot episodeSnapshotLocked() const noexcept {
    ExecutionEpisodeSnapshot view;
    view.localization_epoch = admission_localization_epoch_;
    view.goal_epoch = admission_goal_epoch_;
    if (active_.bundle) {
      view.active_command_goal_epoch = active_.bundle->goal_epoch;
      view.active_command_request_id = active_.bundle->request_id;
      view.active_generation = active_.bundle->bundle_generation;
      if (active_.bundle->goal_epoch == admission_goal_epoch_) {
        view.request_id = active_.bundle->request_id;
      }
    }
    view.phase = lifecycle_.phase;
    view.command_available = lifecycle_.exposure == ExecutionExposure::kAvailable;
    view.failure_latched = lifecycle_.exposure == ExecutionExposure::kFailed;
    view.safety_suffix_active =
        lifecycle_.safety == ExecutionSafetyOwnership::kSafetySuffix;
    view.restart_from_rest =
        lifecycle_.restart == ExecutionRestartRequest::kFromRest;
    view.recovery_state = lifecycle_.recovery;
    return view;
  }

  [[nodiscard]] bool activeBundleIdentityMatchesLocked(
      const navigation_planning::CandidateBundle& bundle) const noexcept {
    return active_.bundle && active_.goal &&
           bundle.localization_epoch != 0U && bundle.goal_epoch != 0U &&
           bundle.request_id != 0U && bundle.bundle_generation != 0U &&
           active_.bundle->localization_epoch == bundle.localization_epoch &&
           active_.bundle->goal_epoch == bundle.goal_epoch &&
           active_.bundle->request_id == bundle.request_id &&
           active_.bundle->bundle_generation == bundle.bundle_generation;
  }

  void failClosedLifecycleLocked() const noexcept {
    lifecycle_.phase = ExecutionEpisodePhase::kPx4Hold;
    lifecycle_.recovery = ExecutionRecoveryState::kPx4Hold;
    lifecycle_.exposure = ExecutionExposure::kFailed;
    lifecycle_.safety = ExecutionSafetyOwnership::kNominal;
    lifecycle_.restart = ExecutionRestartRequest::kNone;
  }

  void committedLifecycleLocked(
      const navigation_planning::CandidateBundle& bundle,
      std::uint64_t previous_generation) const noexcept {
    // The planner reserves generations monotonically. A same-generation
    // identity-preserving replacement is a recertification, not a new episode.
    if (lifecycle_.exposure == ExecutionExposure::kFailed ||
        bundle.bundle_generation <= previous_generation ||
        bundle.localization_epoch != admission_localization_epoch_ ||
        bundle.goal_epoch != admission_goal_epoch_ || !active_.goal ||
        active_.goal->request_id != bundle.request_id) return;
    lifecycle_.exposure = ExecutionExposure::kAvailable;
    lifecycle_.safety =
        bundle.role == navigation_planning::CandidateRole::kBackup ||
        bundle.role == navigation_planning::CandidateRole::kEmergency
            ? ExecutionSafetyOwnership::kSafetySuffix
            : ExecutionSafetyOwnership::kNominal;
    lifecycle_.phase =
        bundle.role == navigation_planning::CandidateRole::kBackup ||
        bundle.role == navigation_planning::CandidateRole::kEmergency
            ? ExecutionEpisodePhase::kTrackingBackup
            : ExecutionEpisodePhase::kTrackingMain;
    lifecycle_.recovery = transitionExecutionRecovery(
        lifecycle_.recovery,
        bundle.role == navigation_planning::CandidateRole::kEmergency
            ? ExecutionRecoveryEvent::kEmergencyCommitted
            : bundle.role == navigation_planning::CandidateRole::kBackup
                  ? ExecutionRecoveryEvent::kBackupActivated
                  : ExecutionRecoveryEvent::kMainCommitted);
  }

  [[nodiscard]] static bool goalMatchesCandidate(
      const std::shared_ptr<const navigation_contracts::msg::NavigationGoal>& goal,
      const std::shared_ptr<const navigation_planning::CandidateBundle>& candidate) noexcept {
    return goal && candidate && !goal->mission_id.empty() &&
           goal->request_id != 0U && goal->request_id == candidate->request_id;
  }

  void clearActiveLocked() const noexcept {
    active_.goal.reset();
    active_.bundle.reset();
    if (lifecycle_.exposure == ExecutionExposure::kAvailable) {
      lifecycle_.exposure = ExecutionExposure::kUnavailable;
    }
  }

  void clearStagedLocked() const noexcept {
    staged_.goal.reset();
    staged_.bundle.reset();
    staged_.activation_ns = 0;
  }

  template <typename FinalizeRevocation, typename PrepareRecertification>
  navigation_world_model::WorldCommitDecision
  publishWorldIdentityIfCurrentAndFinalizeRevocationImpl(
      const navigation_world_model::WorldSnapshotIdentity& identity,
      const std::uint64_t expected_timeline_version,
      const std::shared_ptr<const navigation_planning::CandidateBundle>& expected_bundle,
      const bool retain_validated_bundle,
      FinalizeRevocation&& finalize_revocation,
      const std::int64_t refreshed_valid_until_ns,
      const std::shared_ptr<const navigation_planning::CandidateBundle>& expected_pending,
      const bool retain_validated_pending,
      PrepareRecertification&& prepare) noexcept {
    static_assert(std::is_nothrow_invocable_v<FinalizeRevocation&>);

    // Pin and validate the exact optimistic snapshot before preparing copies.
    // No candidate work or payload allocation runs under this lock.
    std::optional<navigation_world_model::WorldSnapshotIdentity> prior_world;
    std::shared_ptr<const navigation_planning::CandidateBundle> observed_active;
    std::shared_ptr<const navigation_planning::CandidateBundle> observed_pending;
    {
      std::lock_guard lock(mutex_);
      if (timeline_version_ != expected_timeline_version) {
        return navigation_world_model::WorldCommitDecision::kSuperseded;
      }
      if (world_identity_ && !advances(*world_identity_, identity)) {
        return navigation_world_model::WorldCommitDecision::kWorldAdvanced;
      }
      prior_world = world_identity_;
      observed_active = active_.bundle;
      observed_pending = staged_.bundle;
    }

    const bool expected_active_matches = expected_bundle && observed_active &&
        expected_bundle.get() == observed_active.get();
    const bool expected_pending_matches = expected_pending && observed_pending &&
        expected_pending.get() == observed_pending.get();
    const bool exact_active_owner = expected_active_matches && prior_world &&
        navigation_world_model::sameWorldSnapshotIdentity(
            *prior_world, observed_active->world_identity);
    const bool prepare_active = retain_validated_bundle && exact_active_owner;
    const bool prepare_pending = prepare_active && retain_validated_pending &&
        expected_pending_matches && prior_world &&
        navigation_world_model::sameWorldSnapshotIdentity(
            observed_pending->world_identity, *prior_world);
    std::shared_ptr<const navigation_planning::CandidateBundle> recertified_active;
    std::shared_ptr<const navigation_planning::CandidateBundle> recertified_pending;
    bool active_preparation_failed = false;
    if (prepare_active) {
      try {
        recertified_active = std::invoke(
            prepare, *observed_active, identity, refreshed_valid_until_ns);
        active_preparation_failed = !recertified_active;
      } catch (...) {
        active_preparation_failed = true;
      }
    }
    if (prepare_pending && !active_preparation_failed) {
      try {
        recertified_pending = std::invoke(
            prepare, *observed_pending, identity, std::int64_t{0});
      } catch (...) {
        // The successor is independently disposable; the validated active
        // command remains eligible for recertification and execution.
        recertified_pending.reset();
      }
    }

    // These holders defer payload destruction until after unlocking. In
    // particular, revocation must not run CandidateBundle/evaluator teardown
    // inside the timeline critical section.
    std::shared_ptr<const navigation_planning::CandidateBundle> retired_active;
    std::shared_ptr<const navigation_planning::CandidateBundle> retired_pending;
    std::shared_ptr<const navigation_contracts::msg::NavigationGoal> retired_active_goal;
    std::shared_ptr<const navigation_contracts::msg::NavigationGoal> retired_pending_goal;
    std::lock_guard lock(mutex_);
    const bool prior_world_still_current =
        world_identity_.has_value() == prior_world.has_value() &&
        (!world_identity_ || navigation_world_model::sameWorldSnapshotIdentity(
            *world_identity_, *prior_world));
    if (timeline_version_ != expected_timeline_version ||
        active_.bundle.get() != observed_active.get() ||
        staged_.bundle.get() != observed_pending.get() ||
        !prior_world_still_current) {
      return navigation_world_model::WorldCommitDecision::kSuperseded;
    }
    if (world_identity_ && !advances(*world_identity_, identity)) {
      return navigation_world_model::WorldCommitDecision::kWorldAdvanced;
    }

    const bool revoke_exact_active = exact_active_owner &&
        (!prepare_active || active_preparation_failed);
    if (active_preparation_failed) {
      retired_active = std::move(active_.bundle);
      retired_active_goal = std::move(active_.goal);
      retired_pending = std::move(staged_.bundle);
      retired_pending_goal = std::move(staged_.goal);
      staged_.activation_ns = 0;
      if (retired_active) ++active_lineage_version_;
      if (retired_active) failClosedLifecycleLocked();
      ++timeline_version_;
      if (revoke_exact_active) std::invoke(finalize_revocation);
      // The world transaction is deliberately not consumed. A later refresh
      // can retry publication without leaving a known-invalid command live.
      return navigation_world_model::WorldCommitDecision::kCandidateRejected;
    }

    if (prepare_active) {
      retired_active = std::move(active_.bundle);
      active_.bundle = std::move(recertified_active);
    } else {
      retired_active = std::move(active_.bundle);
      retired_active_goal = std::move(active_.goal);
      if (retired_active) ++active_lineage_version_;
      if (retired_active) failClosedLifecycleLocked();
    }

    if (prepare_active && recertified_pending) {
      retired_pending = std::move(staged_.bundle);
      staged_.bundle = std::move(recertified_pending);
    } else {
      retired_pending = std::move(staged_.bundle);
      retired_pending_goal = std::move(staged_.goal);
      staged_.activation_ns = 0;
    }
    if (!active_.bundle) {
      retired_pending = std::move(staged_.bundle);
      retired_pending_goal = std::move(staged_.goal);
      staged_.activation_ns = 0;
    }
    world_identity_ = identity;
    ++timeline_version_;
    if (revoke_exact_active) std::invoke(finalize_revocation);
    return navigation_world_model::WorldCommitDecision::kCommitted;
  }

  template <typename FinalizeFn>
  bool activatePendingIfDueAndFinalizeLocked(
      std::int64_t now_ns,
      const std::shared_ptr<const navigation_planning::CandidateBundle>& expected_pending,
      std::uint64_t expected_version, std::int64_t expected_activation_ns,
      bool require_exact_token, FinalizeFn&& finalize) const noexcept {
    if (lifecycle_.exposure == ExecutionExposure::kFailed) return false;
    if (staged_.bundle && !active_.bundle) {
      clearStagedLocked();
      staged_.activation_ns = 0;
      ++timeline_version_;
      return false;
    }
    if (!staged_.bundle || staged_.activation_ns <= 0 || now_ns < staged_.activation_ns) {
      return false;
    }
    if (require_exact_token &&
        (timeline_version_ != expected_version ||
         staged_.activation_ns != expected_activation_ns)) {
      return false;
    }
    if (expected_pending &&
        (staged_.bundle.get() != expected_pending.get() ||
         staged_.bundle->bundle_generation != expected_pending->bundle_generation)) {
      return false;
    }
    if (!staged_.goal || !active_.goal ||
        staged_.goal->request_id != staged_.bundle->request_id ||
        !world_identity_ ||
        !navigation_world_model::sameWorldSnapshotIdentity(
            *world_identity_, staged_.bundle->world_identity) ||
        admission_goal_epoch_ != staged_.bundle->goal_epoch ||
        now_ns > staged_.bundle->valid_until_ns || !staged_.bundle->valid() || !active_.bundle ||
        !active_.bundle->valid() ||
        !navigation_world_model::sameWorldSnapshotIdentity(
            *world_identity_, active_.bundle->world_identity)) {
      clearStagedLocked();
      staged_.activation_ns = 0;
      ++timeline_version_;
      return false;
    }
    const auto previous_lifecycle = lifecycle_;
    const auto previous_goal = active_.goal;
    const auto previous = active_.bundle;
    const auto successor_goal = staged_.goal;
    const auto successor = staged_.bundle;
    active_.goal = successor_goal;
    active_.bundle = successor;
    clearStagedLocked();
    staged_.activation_ns = 0;
    ++timeline_version_;
    bool finalized = false;
    try {
      finalized = static_cast<bool>(std::forward<FinalizeFn>(finalize)(
          successor->bundle_generation));
    } catch (...) {
      finalized = false;
    }
    if (!finalized) {
      lifecycle_ = previous_lifecycle;
      active_.goal = previous_goal;
      active_.bundle = previous;
      // The finalize callback owns an external transaction (planner history,
      // for example). Once it rejects, retaining the pending pointer would
      // leave an unfinalizable candidate parked ahead of future solves.
      clearStagedLocked();
      staged_.activation_ns = 0;
      enforceInvariantLocked();
      ++timeline_version_;
      return false;
    }
    committedLifecycleLocked(
        *active_.bundle, previous ? previous->bundle_generation : 0U);
    ++active_lineage_version_;
    return true;
  }

  // Validate the immutable predecessor anchor without invoking its evaluator
  // while holding the store mutex. The caller first checks the opaque active
  // lineage reservation; these semantic fields independently guard accidental
  // corruption and world recertification. Activation then checks the current
  // active/world transaction before exposure.
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

  void enforceInvariantLocked() const noexcept {
    if (!active_.bundle) {
      clearStagedLocked();
      staged_.activation_ns = 0;
    }
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

  struct ActiveRecord {
    std::shared_ptr<const navigation_contracts::msg::NavigationGoal> goal;
    std::shared_ptr<const navigation_planning::CandidateBundle> bundle;
  };
  struct StagedRecord {
    std::shared_ptr<const navigation_contracts::msg::NavigationGoal> goal;
    std::shared_ptr<const navigation_planning::CandidateBundle> bundle;
    std::int64_t activation_ns{0};
  };

  mutable std::mutex mutex_;
  std::uint64_t admission_goal_epoch_{0};
  std::uint64_t admission_localization_epoch_{0};
  mutable std::uint64_t timeline_version_{0};
  mutable std::uint64_t active_lineage_version_{0};
  std::uint64_t last_transaction_id_{0};
  std::optional<navigation_world_model::WorldSnapshotIdentity> world_identity_;
  mutable ActiveRecord active_;
  mutable StagedRecord staged_;
  mutable ExecutionLifecycleState lifecycle_{};
};

// Compatibility name for code that only consumes the active-command API.
using ExecutionTimelineStore = ExecutionAuthority;
using CommittedBundleStore = ExecutionAuthority;

}  // namespace navigation_execution
