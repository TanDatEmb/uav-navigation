#pragma once

#include <cstdint>
#include <optional>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <nav_msgs/msg/odometry.hpp>

namespace px4_odometry_bridge {

struct ExternalOdometryFrame {
  std::int64_t timestamp_ns{0};
  Eigen::Vector3d position_frd{Eigen::Vector3d::Zero()};
  Eigen::Quaterniond orientation_frd{Eigen::Quaterniond::Identity()};
  Eigen::Vector3d velocity_body_frd{Eigen::Vector3d::Zero()};
  Eigen::Vector3d angular_velocity_body_frd{Eigen::Vector3d::Zero()};
  Eigen::Vector3d position_variance{Eigen::Vector3d::Constant(1e-6)};
  Eigen::Vector3d orientation_variance{Eigen::Vector3d::Constant(1e-6)};
  Eigen::Vector3d velocity_variance{Eigen::Vector3d::Constant(1e-6)};
};

[[nodiscard]] std::optional<ExternalOdometryFrame> convert_ros_lio_odometry(
    const nav_msgs::msg::Odometry& message);

}  // namespace px4_odometry_bridge
