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
  // PX4 VehicleOdometry input is deliberately published in NED, not in an
  // unlabeled project-local FRD frame.  This makes position, velocity, and
  // attitude use the same world basis and lets the simulator contract be
  // checked directly against Gazebo ground truth.
  Eigen::Vector3d position_ned{Eigen::Vector3d::Zero()};
  Eigen::Quaterniond orientation_ned{Eigen::Quaterniond::Identity()};
  Eigen::Vector3d velocity_ned{Eigen::Vector3d::Zero()};
  // VehicleOdometry.angular_velocity is always body-FRD, independent of the
  // pose and linear-velocity frames.
  Eigen::Vector3d angular_velocity_body_frd{Eigen::Vector3d::Zero()};
  Eigen::Vector3d position_variance{Eigen::Vector3d::Zero()};
  Eigen::Vector3d orientation_variance{Eigen::Vector3d::Zero()};
  Eigen::Vector3d velocity_variance{Eigen::Vector3d::Zero()};
  bool frame_valid{false};
  bool covariance_valid{false};
};

// LIO's public world contract is local ENU/Z-up:
//   +X east, +Y north, +Z up.
// PX4 POSE_FRAME_NED and VELOCITY_FRAME_NED are:
//   +X north, +Y east, +Z down.
// This is the one and only world basis change used for /fmu/in.  In basis
// form: e_x(ENU)->e_y(NED), e_y(ENU)->e_x(NED), e_z(ENU)->-e_z(NED).
[[nodiscard]] const Eigen::Matrix3d& C_ned_from_lio_enu() noexcept;

// ROS base_link is FLU; PX4 body fields are FRD.  This is independent of the
// world transform above and must never be reused for world position/velocity.
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
