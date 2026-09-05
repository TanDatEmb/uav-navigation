#pragma once

#include <cstdint>
#include <optional>

#include <navigation_planning/candidate_bundle.hpp>
#include <navigation_planning/planning_status.hpp>

namespace navigation_planning {

enum class CompletePlanningOutcome : std::uint8_t {
  kRefinedCompleteBundle,
  kBaselineCompleteBundle,
  kDeadlineWithCompleteBundle,
  // The active execution timeline remains authoritative; no replacement was
  // produced and the caller must continue sampling the retained bundle.
  kRetainedCommittedBundle,
  kMapEvidenceInsufficient,
  kNoCompleteBundle,
  kStaleResult,
  kCancelledByHigherPriorityRequest,
  kInvalidRequest,
};

enum class PlanningFailureStage : std::uint8_t {
  kNone,
  kInput,
  kWorldFreshness,
  kRouteWindow,
  kAStar,
  kCorridor,
  kNominalSeed,
  kNominalRefinement,
  kBackupSeed,
  kBackupRefinement,
  kDynamicCertificate,
  kFlatnessCertificate,
  kWorldCertificate,
  kCommitRecertification,
  kDeadline,
};

enum class PlanningFailureReason : std::uint8_t {
  kNone,
  kAnchorOutOfMap,
  kAnchorUnknown,
  kAnchorOccupied,
  kMainPathUnavailable,
  kMainCorridorUnavailable,
  kNominalDynamics,
  kNominalFlatness,
  kBackupKnownFreeInsufficient,
  kBackupDynamics,
  kBackupFlatness,
  kWorldChanged,
  kNoCompleteBundleAtDeadline,
  kStaleResult,
  kSuperseded,
  kInvalidInput,
  // Keep this new telemetry reason at the end: preceding enum ordinals are
  // serialized in runtime diagnostics and are therefore wire-compatible.
  kMainKnownFreeInsufficient,
};

[[nodiscard]] constexpr bool completePlanningSucceeded(
    CompletePlanningOutcome outcome) noexcept {
  return outcome == CompletePlanningOutcome::kRefinedCompleteBundle ||
         outcome == CompletePlanningOutcome::kBaselineCompleteBundle ||
         outcome == CompletePlanningOutcome::kDeadlineWithCompleteBundle;
}

[[nodiscard]] constexpr bool planningRetainedCommittedBundle(
    CompletePlanningOutcome outcome) noexcept {
  return outcome == CompletePlanningOutcome::kRetainedCommittedBundle;
}

struct PlanningTrace {
  std::uint32_t expanded_nodes{0};
  std::uint32_t optimizer_attempts{0};
  std::int64_t elapsed_steady_ns{0};
};

struct PlanningOutcome {
  CompletePlanningOutcome outcome{CompletePlanningOutcome::kInvalidRequest};
  PlanningFailureStage failure_stage{PlanningFailureStage::kInput};
  PlanningFailureReason failure_reason{PlanningFailureReason::kInvalidInput};
  std::optional<CandidateBundle> candidate;
  PlanningTrace trace;

  [[nodiscard]] bool valid() const noexcept {
    const bool success = completePlanningSucceeded(outcome);
    const bool retained = planningRetainedCommittedBundle(outcome);
    const bool failure_is_clear = (success || retained)
        ? failure_stage == PlanningFailureStage::kNone &&
              failure_reason == PlanningFailureReason::kNone
        : failure_stage != PlanningFailureStage::kNone &&
              failure_reason != PlanningFailureReason::kNone;
    return trace.elapsed_steady_ns >= 0 && failure_is_clear &&
           ((success && candidate.has_value()) ||
            (retained && !candidate.has_value()) ||
            (!success && !retained && !candidate.has_value())) &&
           (!candidate || candidate->valid());
  }
};

}  // namespace navigation_planning
