#pragma once

#include <algorithm>
#include <cstdint>
#include <limits>
#include <optional>

#include <Eigen/Core>
#include <mars_quadrotor_msgs/msg/position_command.hpp>
#include <nav_msgs/msg/odometry.hpp>

namespace px4_navigation_external_mode {

struct RejectProvenance {
  double odometry_header_age_ms{0.0};
  double odometry_receive_age_ms{0.0};
  Eigen::Vector3d measured_position{Eigen::Vector3d::Zero()};
  Eigen::Vector3d measured_velocity{Eigen::Vector3d::Zero()};
  bool previous_valid{false};
  mars_quadrotor_msgs::msg::PositionCommand previous{};
  bool generation_changed{false};
  std::int64_t generation_delta{0};
  Eigen::Vector3d previous_position{Eigen::Vector3d::Constant(
      std::numeric_limits<double>::quiet_NaN())};
  Eigen::Vector3d previous_velocity{previous_position};
  Eigen::Vector3d previous_acceleration{previous_position};
  Eigen::Vector3d previous_jerk{previous_position};
  double command_delta_position_m{std::numeric_limits<double>::quiet_NaN()};
  double command_delta_velocity_mps{std::numeric_limits<double>::quiet_NaN()};
  double command_delta_acceleration_mps2{std::numeric_limits<double>::quiet_NaN()};
  double command_delta_jerk_mps3{std::numeric_limits<double>::quiet_NaN()};
};

inline RejectProvenance buildRejectProvenance(
    std::int64_t now_ns, std::int64_t odometry_receive_ns,
    const nav_msgs::msg::Odometry& odometry,
    const mars_quadrotor_msgs::msg::PositionCommand& command,
    const std::optional<mars_quadrotor_msgs::msg::PositionCommand>& previous) {
  RejectProvenance result;
  const std::int64_t odometry_stamp_ns =
      static_cast<std::int64_t>(odometry.header.stamp.sec) * 1000000000LL +
      static_cast<std::int64_t>(odometry.header.stamp.nanosec);
  result.odometry_header_age_ms = static_cast<double>(now_ns - odometry_stamp_ns) * 1.0e-6;
  result.odometry_receive_age_ms =
      static_cast<double>(now_ns - odometry_receive_ns) * 1.0e-6;
  result.measured_position = {odometry.pose.pose.position.x, odometry.pose.pose.position.y,
                              odometry.pose.pose.position.z};
  result.measured_velocity = {odometry.twist.twist.linear.x, odometry.twist.twist.linear.y,
                              odometry.twist.twist.linear.z};
  if (!previous) return result;

  result.previous_valid = true;
  result.previous = *previous;
  result.generation_changed = previous->trajectory_generation != command.trajectory_generation;
  if (command.trajectory_generation >= previous->trajectory_generation) {
    result.generation_delta = static_cast<std::int64_t>(std::min<std::uint64_t>(
        command.trajectory_generation - previous->trajectory_generation,
        static_cast<std::uint64_t>(std::numeric_limits<std::int64_t>::max())));
  } else {
    result.generation_delta = -static_cast<std::int64_t>(std::min<std::uint64_t>(
        previous->trajectory_generation - command.trajectory_generation,
        static_cast<std::uint64_t>(std::numeric_limits<std::int64_t>::max())));
  }
  result.previous_position = {previous->position.x, previous->position.y, previous->position.z};
  result.previous_velocity = {previous->velocity.x, previous->velocity.y, previous->velocity.z};
  result.previous_acceleration = {
      previous->acceleration.x, previous->acceleration.y, previous->acceleration.z};
  result.previous_jerk = {previous->jerk.x, previous->jerk.y, previous->jerk.z};
  const Eigen::Vector3d position{command.position.x, command.position.y, command.position.z};
  const Eigen::Vector3d velocity{command.velocity.x, command.velocity.y, command.velocity.z};
  const Eigen::Vector3d acceleration{
      command.acceleration.x, command.acceleration.y, command.acceleration.z};
  const Eigen::Vector3d jerk{command.jerk.x, command.jerk.y, command.jerk.z};
  result.command_delta_position_m = (position - result.previous_position).norm();
  result.command_delta_velocity_mps = (velocity - result.previous_velocity).norm();
  result.command_delta_acceleration_mps2 =
      (acceleration - result.previous_acceleration).norm();
  result.command_delta_jerk_mps3 = (jerk - result.previous_jerk).norm();
  return result;
}

}  // namespace px4_navigation_external_mode
