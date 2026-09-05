#pragma once

#include <navigation_math/type_utils.hpp>
#include <navigation_planning/planning_outcome.hpp>
#include <planner_core/planner_result.hpp>

#include <utility>

namespace navigation_planning_backend {

// Only a newly solved EXP trajectory may proceed to backup generation and
// candidate authorization. NO_NEED carries historical planning metadata and
// therefore owns no new executable command.
inline bool expResultMayBuildCommandCandidate(navigation_math::RET_CODE result) noexcept {
  return result == navigation_math::SUCCESS;
}

// A backup result may reach candidate construction only when the backup
// branch is explicitly safe to represent in the executable bundle.  Any
// optimizer/deadline/cancellation failure must return before build or
// authorization so the previously committed command remains untouched.
inline bool backupResultMayBuildCommandCandidate(navigation_math::RET_CODE result) noexcept {
  return result == navigation_math::SUCCESS || result == navigation_math::NO_NEED ||
         result == navigation_math::FINISH;
}

// NO_NEED carries historical planner metadata rather than a new executable
// trajectory. Keep a still-future certified suffix for the same goal; FINISH
// is handled by constructing and authorizing its complete main-only candidate.
inline bool shouldRetainBackupCapableCommand(
    bool new_goal, bool backup_available) noexcept {
  return !new_goal && backup_available;
}

// Preserve the first actionable backup failure at the planner boundary.  The
// raw optimizer enum is still logged, but a generic BACKUP_FAILED must not hide
// a timeout, initialization error, or explicit no-path result.
inline PlannerResultCode classifyBackupResult(
    navigation_math::RET_CODE result) noexcept {
  switch (result) {
    case navigation_math::TIME_OUT:
      return PLANNER_SOLVE_TIMEOUT;
    case navigation_math::OPT_FAILED:
      return PLANNER_BACKUP_OPTIMIZATION_FAILED;
    case navigation_math::INIT_ERROR:
      return PLANNER_BACKUP_INITIALIZATION_FAILED;
    case navigation_math::NO_PATH:
      return PLANNER_BACKUP_NO_PATH;
    default:
      return PLANNER_BACKUP_FAILED;
  }
}

// Keep the typed outcome mapping in one place.  The nominal-solve status is
// only consulted for the generic EXP failure; the strict MAIN known-free gate
// has its own result code and therefore cannot be inferred from solve_stage.
inline std::pair<navigation_planning::PlanningFailureStage,
                 navigation_planning::PlanningFailureReason>
classifyPlannerFailure(const PlannerResultCode planner_result,
                       const bool nominal_solve_failed) noexcept {
  using Stage = navigation_planning::PlanningFailureStage;
  using Reason = navigation_planning::PlanningFailureReason;
  switch (planner_result) {
    case PLANNER_NO_ODOM:
      return {Stage::kInput, Reason::kInvalidInput};
    case PLANNER_NO_START_POINT:
      return {Stage::kInput, Reason::kAnchorOutOfMap};
    case PLANNER_INVALID_ROUTE:
      return {Stage::kRouteWindow, Reason::kInvalidInput};
    case PLANNER_SOLVE_TIMEOUT:
      return {Stage::kDeadline, Reason::kNoCompleteBundleAtDeadline};
    case PLANNER_SOLVE_CANCELLED:
      return {Stage::kDeadline, Reason::kSuperseded};
    case PLANNER_BACKUP_NO_PATH:
      return {Stage::kBackupSeed, Reason::kBackupKnownFreeInsufficient};
    case PLANNER_BACKUP_INITIALIZATION_FAILED:
      return {Stage::kBackupSeed, Reason::kBackupDynamics};
    case PLANNER_BACKUP_OPTIMIZATION_FAILED:
    case PLANNER_BACKUP_FAILED:
      return {Stage::kBackupRefinement, Reason::kBackupDynamics};
    case PLANNER_CANDIDATE_REJECTED:
      return {Stage::kCommitRecertification, Reason::kWorldChanged};
    case PLANNER_MAIN_KNOWN_FREE_INSUFFICIENT:
      return {Stage::kNominalSeed, Reason::kMainKnownFreeInsufficient};
    case PLANNER_EXP_FAILED:
      return {nominal_solve_failed ? Stage::kNominalSeed
                                   : Stage::kNominalRefinement,
              Reason::kNominalDynamics};
    default:
      return {Stage::kNominalRefinement, Reason::kNoCompleteBundleAtDeadline};
  }
}

}  // namespace navigation_planning_backend
