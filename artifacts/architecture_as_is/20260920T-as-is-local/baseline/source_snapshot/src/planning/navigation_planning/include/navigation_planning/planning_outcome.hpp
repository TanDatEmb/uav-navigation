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
  // Candidate construction/export failed after the planner transaction. This
  // remains distinct from a deadline failure; the internal exporter records
  // the first violated invariant.
  kCandidateExportInvalid,
  // A BACKUP world sweep failed under AllowUnknown: UNKNOWN is permitted,
  // but currently occupied/undefined/out-of-map cells remain forbidden.
  kBackupWorldBlocked,
  // A necessary physical recovery bound proves the measured boundary cannot
  // satisfy the supplied recovery envelope. Appended to preserve wire codes.
  kStopOutsideRecoveryEnvelope,
  // The bounded concrete stop-polynomial search failed without proving
  // physical infeasibility. Appended to preserve wire codes.
  kStopSynthesisFailed,
};

inline const char* planningFailureStageName(
    const PlanningFailureStage stage) noexcept {
  switch (stage) {
    case PlanningFailureStage::kNone: return "none";
    case PlanningFailureStage::kInput: return "input";
    case PlanningFailureStage::kWorldFreshness: return "world_freshness";
    case PlanningFailureStage::kRouteWindow: return "route_window";
    case PlanningFailureStage::kAStar: return "a_star";
    case PlanningFailureStage::kCorridor: return "corridor";
    case PlanningFailureStage::kNominalSeed: return "nominal_seed";
    case PlanningFailureStage::kNominalRefinement: return "nominal_refinement";
    case PlanningFailureStage::kBackupSeed: return "backup_seed";
    case PlanningFailureStage::kBackupRefinement: return "backup_refinement";
    case PlanningFailureStage::kDynamicCertificate: return "dynamic_certificate";
    case PlanningFailureStage::kFlatnessCertificate: return "flatness_certificate";
    case PlanningFailureStage::kWorldCertificate: return "world_certificate";
    case PlanningFailureStage::kCommitRecertification: return "commit_recertification";
    case PlanningFailureStage::kDeadline: return "deadline";
  }
  return "unknown";
}

inline const char* planningFailureReasonName(
    const PlanningFailureReason reason) noexcept {
  switch (reason) {
    case PlanningFailureReason::kNone: return "none";
    case PlanningFailureReason::kAnchorOutOfMap: return "anchor_out_of_map";
    case PlanningFailureReason::kAnchorUnknown: return "anchor_unknown";
    case PlanningFailureReason::kAnchorOccupied: return "anchor_occupied";
    case PlanningFailureReason::kMainPathUnavailable: return "main_path_unavailable";
    case PlanningFailureReason::kMainCorridorUnavailable:
      return "main_corridor_unavailable";
    case PlanningFailureReason::kNominalDynamics: return "nominal_dynamics";
    case PlanningFailureReason::kNominalFlatness: return "nominal_flatness";
    case PlanningFailureReason::kBackupKnownFreeInsufficient:
      return "backup_known_free_insufficient";
    case PlanningFailureReason::kBackupDynamics: return "backup_dynamics";
    case PlanningFailureReason::kBackupFlatness: return "backup_flatness";
    case PlanningFailureReason::kWorldChanged: return "world_changed";
    case PlanningFailureReason::kNoCompleteBundleAtDeadline:
      return "no_complete_bundle_at_deadline";
    case PlanningFailureReason::kStaleResult: return "stale_result";
    case PlanningFailureReason::kSuperseded: return "superseded";
    case PlanningFailureReason::kInvalidInput: return "invalid_input";
    case PlanningFailureReason::kMainKnownFreeInsufficient:
      return "main_known_free_insufficient";
    case PlanningFailureReason::kCandidateExportInvalid:
      return "candidate_export_invalid";
    case PlanningFailureReason::kBackupWorldBlocked:
      return "backup_world_blocked";
    case PlanningFailureReason::kStopOutsideRecoveryEnvelope:
      return "stop_outside_recovery_envelope";
    case PlanningFailureReason::kStopSynthesisFailed:
      return "stop_synthesis_failed";
  }
  return "unknown";
}

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
