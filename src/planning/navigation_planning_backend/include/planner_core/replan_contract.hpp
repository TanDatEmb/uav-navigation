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

// Receding-horizon admission must not replace a still-executable MAIN prefix
// with one whose absolute switch-to-BACKUP time is earlier. Comparing relative
// durations is incorrect because successive candidates have different wall
// start times. Safety recertification and mission-identity checks remain the
// caller's responsibility.
inline bool currentMainHorizonDominatesCandidate(
    double current_start_wall_time_s,
    double current_duration_s,
    double current_backup_start_s,
    bool current_backup_available,
    double candidate_start_wall_time_s,
    double candidate_duration_s,
    double candidate_backup_start_s,
    bool candidate_backup_available,
    double authorization_wall_time_s) noexcept {
  const auto main_end = [](double start, double duration, double backup_start,
                           bool backup_available) {
    return start + (backup_available ? backup_start : duration);
  };
  if (!std::isfinite(current_start_wall_time_s) ||
      !std::isfinite(current_duration_s) || current_duration_s < 0.0 ||
      !std::isfinite(current_backup_start_s) || current_backup_start_s < 0.0 ||
      current_backup_start_s > current_duration_s + 1.0e-9 ||
      !std::isfinite(candidate_start_wall_time_s) ||
      !std::isfinite(candidate_duration_s) || candidate_duration_s < 0.0 ||
      !std::isfinite(candidate_backup_start_s) || candidate_backup_start_s < 0.0 ||
      candidate_backup_start_s > candidate_duration_s + 1.0e-9 ||
      !std::isfinite(authorization_wall_time_s)) {
    return false;
  }
  const double current_main_end = main_end(
      current_start_wall_time_s, current_duration_s,
      current_backup_start_s, current_backup_available);
  const double candidate_main_end = main_end(
      candidate_start_wall_time_s, candidate_duration_s,
      candidate_backup_start_s, candidate_backup_available);
  constexpr double kTimeEpsilonS = 1.0e-6;
  return current_main_end > authorization_wall_time_s + kTimeEpsilonS &&
         current_main_end > candidate_main_end + kTimeEpsilonS;
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
