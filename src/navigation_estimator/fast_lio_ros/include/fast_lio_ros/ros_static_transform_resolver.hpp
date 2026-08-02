#pragma once

#include <chrono>
#include <cstddef>
#include <string>

#include <rclcpp/node.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include "fast_lio_core/common/result.hpp"
#include "fast_lio_core/geometry/rigid_transform.hpp"

namespace uav::nav::lio {

class RosStaticTransformResolver {
 public:
  explicit RosStaticTransformResolver(rclcpp::Node& node);

  [[nodiscard]] Result<RigidTransform> resolve(
      const std::string& target_frame, const std::string& source_frame,
      std::size_t attempts = 60U,
      std::chrono::milliseconds retry_period = std::chrono::milliseconds(50));

 private:
  tf2_ros::Buffer buffer_;
  tf2_ros::TransformListener listener_;
};

}  // namespace uav::nav::lio
