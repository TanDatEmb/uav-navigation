#pragma once

#include <navigation_math/type_utils.hpp>

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
// authorization so the previously committed CmdTraj remains untouched.
inline bool backupResultMayBuildCommandCandidate(navigation_math::RET_CODE result) noexcept {
  return result == navigation_math::SUCCESS || result == navigation_math::NO_NEED ||
         result == navigation_math::FINISH;
}

}  // namespace navigation_planning_backend
