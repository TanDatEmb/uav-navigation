#pragma once

#include <filesystem>

#include <navigation_planning_backend/planner.hpp>

namespace navigation_runtime {

// Reads the same mission.planning limits consumed by Avoidance Mission. The
// planner must receive these values before planner backend constructs its optimizers.
navigation_planning_backend::DynamicLimits loadMissionDynamicLimits(
    const std::filesystem::path& mission_file);

}  // namespace navigation_runtime
