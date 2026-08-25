#pragma once

#include <cmath>
#include <cstdint>
#include <memory>

#include <navigation_planning_backend/planner.hpp>

namespace navigation_runtime {

struct PlannerExecutionState {
  navigation_planning_backend::math::Vec3f position{navigation_planning_backend::math::Vec3f::Zero()};
  navigation_planning_backend::math::Vec3f velocity{navigation_planning_backend::math::Vec3f::Zero()};
  navigation_planning_backend::math::Vec3f acceleration{navigation_planning_backend::math::Vec3f::Zero()};
  navigation_planning_backend::math::Vec3f jerk{navigation_planning_backend::math::Vec3f::Zero()};
  navigation_planning_backend::math::Quatf orientation{navigation_planning_backend::math::Quatf::Identity()};
  double yaw{0.0};
  double stamp_s{0.0};
  bool valid{false};

  bool finite() const {
    return position.allFinite() && velocity.allFinite() && acceleration.allFinite() &&
           jerk.allFinite() && orientation.coeffs().allFinite() &&
           orientation.norm() > 1.0e-6 && std::isfinite(yaw) &&
           std::isfinite(stamp_s) && stamp_s > 0.0;
  }

  navigation_planning_backend::math::RobotState toRobotState() const {
    navigation_planning_backend::math::RobotState result;
    result.p = position;
    result.v = velocity;
    result.a = acceleration;
    result.j = jerk;
    result.q = orientation.normalized();
    result.yaw = yaw;
    result.rcv_time = stamp_s;
    result.rcv = valid && finite();
    return result;
  }
};

}  // namespace navigation_runtime
