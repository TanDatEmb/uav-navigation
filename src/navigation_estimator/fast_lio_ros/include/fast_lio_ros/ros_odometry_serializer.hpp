#pragma once

#include <nav_msgs/msg/odometry.hpp>

#include "fast_lio_core/common/result.hpp"
#include "fast_lio_core/navigation/rigid_body_state.hpp"
#include "fast_lio_ros/parameter_loader.hpp"

namespace uav::nav::lio {

class RosOdometrySerializer {
 public:
  [[nodiscard]] static Result<nav_msgs::msg::Odometry> serialize(
      const RigidBodyState& state, const RosParameters& parameters);
};

}  // namespace uav::nav::lio
