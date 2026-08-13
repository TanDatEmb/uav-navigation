#include "navigation_planning/planner.hpp"

#include <utility>

namespace navigation_planning {

Planner::Planner(PlannerConfig config) : config_(std::move(config)) {}

PlanResult Planner::plan(const VehicleState& state, const Goal& goal,
                         const navigation_mapping::WorldModel& world) const {
  return planModel(state, goal, world);
}

bool TimeParameterizedTrajectory::finiteAndMonotonic() const noexcept {
  if (!std::isfinite(duration_s) || duration_s < 0.0 || points.empty()) return false;
  double previous_time = -1.0;
  for (const auto& point : points) {
    if (!std::isfinite(point.time_from_start_s) ||
        (previous_time >= 0.0 && point.time_from_start_s <= previous_time) ||
        !point.position.allFinite() || !point.velocity.allFinite() ||
        !point.acceleration.allFinite()) {
      return false;
    }
    previous_time = point.time_from_start_s;
  }
  return std::abs(points.back().time_from_start_s - duration_s) <= 1e-9;
}

}  // namespace navigation_planning
