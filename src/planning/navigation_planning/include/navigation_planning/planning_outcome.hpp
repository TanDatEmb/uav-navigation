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
};

[[nodiscard]] constexpr bool completePlanningSucceeded(
    CompletePlanningOutcome outcome) noexcept {
  return outcome == CompletePlanningOutcome::kRefinedCompleteBundle ||
         outcome == CompletePlanningOutcome::kBaselineCompleteBundle ||
         outcome == CompletePlanningOutcome::kDeadlineWithCompleteBundle;
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
    const bool failure_is_clear = success
        ? failure_stage == PlanningFailureStage::kNone &&
              failure_reason == PlanningFailureReason::kNone
        : failure_stage != PlanningFailureStage::kNone &&
              failure_reason != PlanningFailureReason::kNone;
    return trace.elapsed_steady_ns >= 0 && failure_is_clear &&
           success == candidate.has_value() && (!candidate || candidate->valid());
  }
};

}  // namespace navigation_planning
