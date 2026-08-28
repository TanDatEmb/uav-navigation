#pragma once

#include <cmath>

#include <navigation_math/type_utils.hpp>
#include <planner_core/planner_result.hpp>

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

// A visible main-only replacement must not erase a still-future certified
// safety suffix.  The existing bundle remains subject to the runtime's latest
// world recertification; this predicate only decides whether the backend may
// replace that bundle at all.
inline bool shouldRetainBackupCapableCommand(
    bool new_goal, bool backup_available) noexcept {
  return !new_goal && backup_available;
}

// BACKUP is an executable braking certificate, not route guidance.  A hot
// recovery must preserve the sampled BACKUP PVAJ boundary, but retaining its
// spatial suffix as the next nominal guide recursively feeds the stopping
// curve into MAIN and prolongs the deceleration.  MAIN may retain only its
// configured continuity prefix; BACKUP recovery starts geometric search at
// the exact sampled state.
inline double retainedHotReplanGuideDistance(
    double configured_distance_m, bool sampled_state_is_backup) noexcept {
  if (!std::isfinite(configured_distance_m) || configured_distance_m <= 0.0 ||
      sampled_state_is_backup) {
    return 0.0;
  }
  return configured_distance_m;
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

}  // namespace navigation_planning_backend
