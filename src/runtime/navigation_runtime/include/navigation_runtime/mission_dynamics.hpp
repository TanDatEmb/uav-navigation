#pragma once

#include <filesystem>

#include <navigation_planning/planning_limits.hpp>

namespace navigation_runtime {

// Reads the same mission.planning limits consumed by Avoidance Mission. The
// planner must receive these values before planner backend constructs its optimizers.
navigation_planning::DynamicLimits loadMissionDynamicLimits(
    const std::filesystem::path& mission_file);

}  // namespace navigation_runtime
