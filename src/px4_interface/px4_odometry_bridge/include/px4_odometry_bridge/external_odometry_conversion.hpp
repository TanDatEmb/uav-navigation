#pragma once

#include <array>
#include <cstddef>
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
  Eigen::Vector3d position_variance{Eigen::Vector3d::Zero()};
  Eigen::Vector3d orientation_variance{Eigen::Vector3d::Zero()};
  Eigen::Vector3d velocity_variance{Eigen::Vector3d::Zero()};
  bool frame_valid{false};
  bool covariance_valid{false};
};

// Keep these matrices separate even though both are FRD/FLU sign changes.
// The world matrix changes ROS local Z-up coordinates; the body matrix changes
// ROS body FLU coordinates. The distinction is part of the public contract.
[[nodiscard]] const Eigen::Matrix3d& C_world_frd_from_ros_local_zup() noexcept;
[[nodiscard]] const Eigen::Matrix3d& C_body_frd_from_body_flu() noexcept;

[[nodiscard]] std::optional<Eigen::Vector3d> transformed_covariance_diagonal(
    const std::array<double, 36>& covariance, std::size_t block_offset,
    const Eigen::Matrix3d& transform);

// PX4 receives float variances. A positive double is representable only if
// the converted float remains finite and strictly positive; otherwise the
// bridge must reject it instead of silently publishing zero or infinity.
[[nodiscard]] std::optional<float> positive_variance_to_px4_float(double value);

[[nodiscard]] std::optional<std::array<float, 3>>
positive_variances_to_px4_float(const Eigen::Vector3d& values);

[[nodiscard]] std::optional<ExternalOdometryFrame> convert_ros_lio_odometry(
    const nav_msgs::msg::Odometry& message);

}  // namespace px4_odometry_bridge
