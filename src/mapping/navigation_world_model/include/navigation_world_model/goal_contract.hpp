#pragma once

namespace navigation_world_model {

// One product-owned 3-D completion/connectivity tolerance.  Planner endpoint
// resolution and runtime mission completion must not silently drift apart.
inline constexpr double kGoalCompletionToleranceM = 0.20;

}  // namespace navigation_world_model
