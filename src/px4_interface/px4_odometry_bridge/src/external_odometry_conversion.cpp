#include "px4_odometry_bridge/external_odometry_conversion.hpp"

#include <cmath>

#include "px4_odometry_bridge/frame_converter.hpp"

namespace px4_odometry_bridge {
namespace {

bool valid_variance(double value) {
  return std::isfinite(value) && value > 0.0;
}

}  // namespace

std::optional<ExternalOdometryFrame> convert_ros_lio_odometry(
    const nav_msgs::msg::Odometry& message) {
  const std::int64_t timestamp_ns =
      static_cast<std::int64_t>(message.header.stamp.sec) * 1'000'000'000LL +
      static_cast<std::int64_t>(message.header.stamp.nanosec);
  if (timestamp_ns <= 0) return std::nullopt;
  const Eigen::Quaterniond orientation_ros(
      message.pose.pose.orientation.w, message.pose.pose.orientation.x,
      message.pose.pose.orientation.y, message.pose.pose.orientation.z);
  if (!orientation_ros.coeffs().allFinite() || orientation_ros.norm() < 1e-9) {
    return std::nullopt;
  }
  const Eigen::Matrix3d C = FrameConverter::c_flu_frd();
  ExternalOdometryFrame result;
  result.timestamp_ns = timestamp_ns;
  result.position_frd = C * Eigen::Vector3d(
      message.pose.pose.position.x, message.pose.pose.position.y,
      message.pose.pose.position.z);
  result.orientation_frd = Eigen::Quaterniond(
      C * orientation_ros.normalized().toRotationMatrix() * C).normalized();
  result.velocity_body_frd = C * Eigen::Vector3d(
      message.twist.twist.linear.x, message.twist.twist.linear.y,
      message.twist.twist.linear.z);
  result.angular_velocity_body_frd = C * Eigen::Vector3d(
      message.twist.twist.angular.x, message.twist.twist.angular.y,
      message.twist.twist.angular.z);
  if (!result.position_frd.allFinite() || !result.velocity_body_frd.allFinite() ||
      !result.angular_velocity_body_frd.allFinite()) {
    return std::nullopt;
  }
  for (int index = 0; index < 3; ++index) {
    const double position_variance =
        message.pose.covariance[static_cast<std::size_t>(index * 6 + index)];
    const double orientation_variance =
        message.pose.covariance[static_cast<std::size_t>((index + 3) * 6 + index + 3)];
    const double velocity_variance =
        message.twist.covariance[static_cast<std::size_t>(index * 6 + index)];
    if (!valid_variance(position_variance) ||
        !valid_variance(orientation_variance) ||
        !valid_variance(velocity_variance)) {
      return std::nullopt;
    }
    result.position_variance[index] = position_variance;
    result.orientation_variance[index] = orientation_variance;
    result.velocity_variance[index] = velocity_variance;
  }
  return result;
}

}  // namespace px4_odometry_bridge
