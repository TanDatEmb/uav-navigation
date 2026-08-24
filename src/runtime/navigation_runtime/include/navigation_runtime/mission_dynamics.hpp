#pragma once

#include <filesystem>

#include <super_core/config.hpp>

namespace navigation_runtime {

// Reads the same mission.planning limits consumed by Avoidance Mission. The
// planner must receive these values before SUPER constructs its optimizers.
super_planner::DynamicLimits loadMissionDynamicLimits(
    const std::filesystem::path& mission_file);

}  // namespace navigation_runtime
