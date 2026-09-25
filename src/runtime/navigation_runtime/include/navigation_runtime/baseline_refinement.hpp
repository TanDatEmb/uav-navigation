#pragma once

#include <optional>

#include <navigation_execution/execution_authority.hpp>
#include <navigation_planning/planning_outcome.hpp>
#include "navigation_runtime/execution_lifecycle_view.hpp"
#include "navigation_runtime/planner_fsm.hpp"
#include "navigation_runtime/planning_key.hpp"

namespace navigation_runtime {

struct BaselineRefinementContext final {
  PlanningKey key{};
  const navigation_planning::CandidateBundle* active{nullptr};
  navigation_execution::ExecutionAuthoritySnapshot execution{};
  navigation_world_model::WorldSnapshotIdentity world{};
  std::uint64_t backend_generation{0U};
  std::int64_t now_ns{0};
  navigation_planning::CandidateRole sampled_role{
      navigation_planning::CandidateRole::kEmergency};
  bool pending{false};
  bool desired_matches_executing{false};
  bool exposure_allowed{false};
  bool tracking_supported{false};
};

// Serial planning-worker-owned scheduling receipt, never command authority.
// The first admitted initial baseline gets at most one later quality attempt.
// A seed successor or a stopped recovery must not create a replan train.
class BaselineRefinementOpportunity final {
 public:
  void noteAdmission(const PlanningKey& request,
                     const navigation_planning::CandidateBundle& candidate,
                     navigation_planning::CompletePlanningOutcome outcome,
                     bool initial_stopped_transition) noexcept {
    if (!request.valid() || !candidate.valid()) return;
    if (owner_ && sameGoalOwner(*owner_, request)) return;
    owner_ = request;
    generation_ = 0U;
    if (initial_stopped_transition &&
        request.start_mode == PlanningStartMode::kStoppedMeasuredState &&
        outcome == navigation_planning::CompletePlanningOutcome::kBaselineCompleteBundle &&
        candidate.kind == navigation_planning::CandidateBundleKind::kMainWithBackup &&
        candidate.localization_epoch == request.localization_epoch &&
        candidate.goal_epoch == request.goal_epoch &&
        candidate.request_id == request.request_id &&
        candidate.pinned_world_identity.generation == request.pinned_world_generation) {
      generation_ = candidate.bundle_generation;
    }
  }

  [[nodiscard]] bool ready(const PlannerRenewalDecision& ordinary,
                           const BaselineRefinementContext& c) const noexcept {
    if (!owner_ || generation_ == 0U || !c.key.valid() || !c.active ||
        ordinary.reason != PlannerRenewalReason::kRetainCertifiedMain ||
        ordinary.run_optimizer ||
        !std::isfinite(ordinary.remaining_main_horizon_s) ||
        ordinary.remaining_main_horizon_s <=
            navigation_planning::PlanningTimingContract::kUrgentBaselineThresholdS ||
        !sameGoalOwner(*owner_, c.key) ||
        owner_->route_revision != c.key.route_revision ||
        owner_->pinned_world_generation != c.key.pinned_world_generation ||
        owner_->dynamics_hash != c.key.dynamics_hash) return false;
    const auto& a = *c.active;
    const auto& execution = c.execution;
    const auto& lifecycle = execution.lifecycle;
    return a.valid() && a.kind == navigation_planning::CandidateBundleKind::kMainWithBackup &&
        a.hasTrajectoryMetadata() && a.bundle_generation == generation_ &&
        c.key.start_mode == PlanningStartMode::kCommittedFutureState &&
        c.key.committed_bundle_generation == generation_ &&
        c.backend_generation == generation_ &&
        a.localization_epoch == c.key.localization_epoch &&
        a.goal_epoch == c.key.goal_epoch && a.request_id == c.key.request_id &&
        navigation_world_model::sameWorldSnapshotIdentity(a.world_identity, c.world) &&
        c.world.generation == c.key.pinned_world_generation &&
        c.world.revision == c.key.pinned_world_revision &&
        c.now_ns >= a.valid_from_ns && c.now_ns <= a.valid_until_ns &&
        c.sampled_role == navigation_planning::CandidateRole::kMain &&
        execution.active &&
        execution.active->localization_epoch == a.localization_epoch &&
        execution.activeGoalEpoch() == a.goal_epoch &&
        execution.activeRequestId() == a.request_id &&
        execution.activeGeneration() == generation_ &&
        lifecycle.phase == ExecutionPhase::kTrackingMain &&
        lifecycle.recovery == ExecutionRecoveryState::kTrackMain &&
        execution.commandAvailable() && !execution.failed() &&
        !execution.safetySuffixActive() && !execution.restartFromRest() &&
        !c.pending && c.desired_matches_executing &&
        c.exposure_allowed && c.tracking_supported;
  }

  // Consume at backend entry, after a valid typed request and future anchor.
  // Failure, cancellation and retained outcomes never refund this receipt.
  bool consume(const PlannerRenewalDecision& ordinary,
               const BaselineRefinementContext& context) noexcept {
    if (!ready(ordinary, context)) return false;
    generation_ = 0U;
    return true;
  }

 private:
  static bool sameGoalOwner(const PlanningKey& a, const PlanningKey& b) noexcept {
    return a.localization_epoch == b.localization_epoch && a.goal_epoch == b.goal_epoch &&
        a.request_id == b.request_id;
  }
  std::optional<PlanningKey> owner_;
  std::uint64_t generation_{0U};
};

// Kept separate from forced/recovery transitions: quality may only open the
// otherwise retained healthy-MAIN branch. Every ordinary deadline is intact.
inline PlannerRenewalDecision withBaselineRefinement(
    PlannerRenewalDecision ordinary, bool ready) noexcept {
  if (ready && !ordinary.run_optimizer &&
      ordinary.reason == PlannerRenewalReason::kRetainCertifiedMain) {
    ordinary.run_optimizer = true;
    ordinary.reason = PlannerRenewalReason::kQualityRefinement;
  }
  return ordinary;
}

}  // namespace navigation_runtime
