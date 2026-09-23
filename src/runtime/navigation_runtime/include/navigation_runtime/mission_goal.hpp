#pragma once

#include <optional>

#include <builtin_interfaces/msg/time.hpp>
#include <navigation_contracts/msg/navigation_goal.hpp>

#include "navigation_runtime/mission_progress.hpp"

namespace navigation_runtime {

[[nodiscard]] std::optional<navigation_contracts::msg::NavigationGoal>
makeMissionGoal(const MissionProgress& progress,
                const builtin_interfaces::msg::Time& stamp);

}  // namespace navigation_runtime
