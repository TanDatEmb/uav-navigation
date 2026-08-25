#pragma once

#include <cmath>
#include <cstdint>
#include <string>

#include <Eigen/Core>

#include <navigation_planning/kinematic_state.hpp>
#include <navigation_planning/planning_budget.hpp>
#include <navigation_world_model/world_model_view.hpp>

namespace navigation_planning {

struct GoalIdentity {
  std::uint64_t localization_epoch{0};
  std::uint64_t goal_epoch{0};
  std::string mission_id;
  std::uint32_t waypoint_index{0};
  std::uint64_t request_id{0};
  Eigen::Vector3d target_world{Eigen::Vector3d::Zero()};

  [[nodiscard]] bool valid() const noexcept {
    return localization_epoch != 0 && goal_epoch != 0 && request_id != 0 &&
           target_world.allFinite();
  }
};

struct DynamicsLimits {
  double maximum_velocity_mps{0.0};
  double maximum_acceleration_mps2{0.0};
  double maximum_jerk_mps3{0.0};

  [[nodiscard]] bool valid() const noexcept {
    return std::isfinite(maximum_velocity_mps) &&
           std::isfinite(maximum_acceleration_mps2) &&
           std::isfinite(maximum_jerk_mps3) && maximum_velocity_mps > 0.0 &&
           maximum_acceleration_mps2 > 0.0 && maximum_jerk_mps3 > 0.0;
  }
};

struct PlanningHistory {
  std::uint64_t previous_bundle_generation{0};
  Eigen::Vector3d previous_velocity_world{Eigen::Vector3d::Zero()};
};

struct PlanningRequest {
  GoalIdentity goal;
  KinematicState start_state;
  PlanningHistory history;
  navigation_world_model::WorldModelViewPtr world;
  DynamicsLimits dynamics;
  PlanningBudget budget;

  [[nodiscard]] bool valid() const noexcept {
    return goal.valid() && start_state.finite() &&
           static_cast<bool>(world) && world->identity().localization_epoch ==
               goal.localization_epoch && dynamics.valid() &&
           !budget.exhausted();
  }
};

}  // namespace navigation_planning
