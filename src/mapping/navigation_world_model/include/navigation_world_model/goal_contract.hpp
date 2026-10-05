#pragma once

#include <navigation_world_model/world_model_view.hpp>

namespace navigation_world_model {

// These are deliberately separate contracts even while they share the
// current provisional value.  Completion is a runtime mission decision;
// connection is planner endpoint metadata; the near-goal shortcut decides
// whether the planner may omit A*.  Keeping the names separate prevents a
// future change in one policy from silently changing the others.
inline constexpr double kGoalCompletionToleranceM = 0.20;
inline constexpr double kGoalConnectionToleranceM = kGoalCompletionToleranceM;
inline constexpr double kNearGoalShortcutToleranceM = kGoalCompletionToleranceM;

// A near-goal shortcut is still a main-trajectory segment.  UNKNOWN remains
// allowed for planner backend's exploratory path, while the WorldModel implementation
// must reject OCCUPIED and OUT_OF_MAP cells.  Backup safety is validated by
// its own certificate and is not weakened by this helper.
inline bool isGoalSegmentTraversable(const WorldModelView& world,
                                     const Point3& start,
                                     const Point3& goal) noexcept {
  if (!start.allFinite() || !goal.allFinite() ||
      !world.contains(start) || !world.contains(goal)) {
    return false;
  }
  return world.isSegmentTraversable(
      start, goal, GridLayer::kInflated, UnknownPolicy::kAllowUnknown);
}

}  // namespace navigation_world_model
