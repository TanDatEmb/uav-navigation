#pragma once

// Product-facing planner include. Implementation details remain private to
// this package; callers use navigation_planning_backend names only.
#include <planner_core/planning_stage.hpp>
#include <planner_core/planner.hpp>
#include <planner_core/planner_result.hpp>
#include <planner_core/trajectory_world_validator.hpp>

namespace navigation_planning_backend {

namespace math = ::navigation_math;
using ::navigation_math::RET_CODE;
using ::navigation_math::FAILED;
using ::navigation_math::NO_NEED;
using ::navigation_math::SUCCESS;
using ::navigation_math::NEW_TRAJ;
using ::navigation_math::FINISH;
using ::navigation_math::OPT_FAILED;
using ::navigation_math::EMER;

}  // namespace navigation_planning_backend
