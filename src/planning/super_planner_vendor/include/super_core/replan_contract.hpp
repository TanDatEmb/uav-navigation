#pragma once

#include <super_utils/type_utils.hpp>

namespace super_planner {

// Only a newly solved EXP trajectory may proceed to backup generation and
// candidate authorization. NO_NEED carries historical planning metadata and
// therefore owns no new executable command.
inline bool expResultMayBuildCommandCandidate(super_utils::RET_CODE result) noexcept {
  return result == super_utils::SUCCESS;
}

// A backup result may reach candidate construction only when the backup
// branch is explicitly safe to represent in the executable bundle.  Any
// optimizer/deadline/cancellation failure must return before build or
// authorization so the previously committed CmdTraj remains untouched.
inline bool backupResultMayBuildCommandCandidate(super_utils::RET_CODE result) noexcept {
  return result == super_utils::SUCCESS || result == super_utils::NO_NEED ||
         result == super_utils::FINISH;
}

}  // namespace super_planner
