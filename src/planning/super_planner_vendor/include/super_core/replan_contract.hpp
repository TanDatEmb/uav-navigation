#pragma once

#include <super_utils/type_utils.hpp>

namespace super_planner {

// Only a newly solved EXP trajectory may proceed to backup generation and
// candidate authorization. NO_NEED carries historical planning metadata and
// therefore owns no new executable command.
inline bool expResultMayBuildCommandCandidate(super_utils::RET_CODE result) noexcept {
  return result == super_utils::SUCCESS;
}

}  // namespace super_planner
