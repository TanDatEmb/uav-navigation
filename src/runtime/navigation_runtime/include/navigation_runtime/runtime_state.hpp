#pragma once

#include <cmath>
#include <cstdint>
#include <memory>

#include <nav_msgs/msg/odometry.hpp>
#include <rog_map/rog_map.h>

namespace navigation_runtime {

struct MappingObservation {
  std::shared_ptr<rog_map::PointCloud> cloud;
  nav_msgs::msg::Odometry corrected_odometry;
  std::int64_t stamp_ns{0};
  std::int64_t pointcloud_decode_us{0};
  std::int64_t pair_wait_us{0};
};

struct PlannerExecutionState {
  super_utils::Vec3f position{super_utils::Vec3f::Zero()};
  super_utils::Vec3f velocity{super_utils::Vec3f::Zero()};
  super_utils::Vec3f acceleration{super_utils::Vec3f::Zero()};
  super_utils::Vec3f jerk{super_utils::Vec3f::Zero()};
  super_utils::Quatf orientation{super_utils::Quatf::Identity()};
  double yaw{0.0};
  double stamp_s{0.0};
  bool valid{false};

  bool finite() const {
    return position.allFinite() && velocity.allFinite() && acceleration.allFinite() &&
           jerk.allFinite() && orientation.coeffs().allFinite() &&
           orientation.norm() > 1.0e-6 && std::isfinite(yaw) &&
           std::isfinite(stamp_s) && stamp_s > 0.0;
  }

  super_utils::RobotState toRobotState() const {
    super_utils::RobotState result;
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
