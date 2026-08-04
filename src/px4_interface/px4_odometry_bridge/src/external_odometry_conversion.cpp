#include "px4_odometry_bridge/external_odometry_conversion.hpp"

#include <cmath>

#include <Eigen/Eigenvalues>

namespace px4_odometry_bridge {
namespace {

constexpr double kSymmetryTolerance = 1e-10;
constexpr double kPsdTolerance = 1e-10;

bool valid_variance(const double value) {
  // The bridge must not manufacture a covariance floor.  A positive,
  // finite variance remains valid even when it is smaller than 1e-6; zero,
  // negative, and non-finite values are unavailable for this contract.
  return std::isfinite(value) && value > 0.0;
}

std::optional<Eigen::Matrix3d> validated_block(
    const std::array<double, 36>& covariance, const std::size_t block_offset) {
  if (block_offset > 3) {
    return std::nullopt;
  }
  Eigen::Matrix3d block;
  for (std::size_t row = 0; row < 3; ++row) {
    for (std::size_t column = 0; column < 3; ++column) {
      block(static_cast<Eigen::Index>(row), static_cast<Eigen::Index>(column)) =
          covariance[(block_offset + row) * 6U + block_offset + column];
    }
  }
  if (!block.allFinite() ||
      (block - block.transpose()).cwiseAbs().maxCoeff() > kSymmetryTolerance) {
    return std::nullopt;
  }
  for (Eigen::Index index = 0; index < 3; ++index) {
    if (!valid_variance(block(index, index))) {
      return std::nullopt;
    }
  }
  const Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver(block);
  if (solver.info() != Eigen::Success ||
      solver.eigenvalues().minCoeff() < -kPsdTolerance) {
    return std::nullopt;
  }
  return block;
}

}  // namespace

const Eigen::Matrix3d& C_ned_from_lio_enu() noexcept {
  // Exact ENU -> NED basis conversion.  Do not replace this with a yaw-only
  // rotation: the vertical axis also changes sign.
  static const Eigen::Matrix3d matrix =
      (Eigen::Matrix3d() << 0.0, 1.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, -1.0)
          .finished();
  return matrix;
}

const Eigen::Matrix3d& C_body_frd_from_body_flu() noexcept {
  static const Eigen::Matrix3d matrix =
      (Eigen::Matrix3d() << 1.0, 0.0, 0.0, 0.0, -1.0, 0.0, 0.0, 0.0, -1.0)
          .finished();
  return matrix;
}

std::optional<Eigen::Vector3d> transformed_covariance_diagonal(
    const std::array<double, 36>& covariance, const std::size_t block_offset,
    const Eigen::Matrix3d& transform) {
  const auto block = validated_block(covariance, block_offset);
  if (!block || !transform.allFinite()) {
    return std::nullopt;
  }
  const Eigen::Matrix3d transformed = transform * *block * transform.transpose();
  if (!transformed.allFinite()) {
    return std::nullopt;
  }
  Eigen::Vector3d diagonal = transformed.diagonal();
  for (Eigen::Index index = 0; index < 3; ++index) {
    if (!valid_variance(diagonal[index])) {
      return std::nullopt;
    }
  }
  return diagonal;
}

std::optional<float> positive_variance_to_px4_float(const double value) {
  if (!valid_variance(value)) {
    return std::nullopt;
  }
  const float converted = static_cast<float>(value);
  if (!std::isfinite(converted) || converted <= 0.0F) {
    return std::nullopt;
  }
  return converted;
}

std::optional<std::array<float, 3>> positive_variances_to_px4_float(
    const Eigen::Vector3d& values) {
  std::array<float, 3> converted{};
  for (Eigen::Index index = 0; index < 3; ++index) {
    const auto value = positive_variance_to_px4_float(values[index]);
    if (!value) {
      return std::nullopt;
    }
    converted[static_cast<std::size_t>(index)] = *value;
  }
  return converted;
}

std::optional<ExternalOdometryFrame> convert_ros_lio_odometry(
    const nav_msgs::msg::Odometry& message) {
  if (message.header.frame_id != "lio_odom" ||
      message.child_frame_id != "base_link") {
    return std::nullopt;
  }
  const std::int64_t timestamp_ns =
      static_cast<std::int64_t>(message.header.stamp.sec) * 1'000'000'000LL +
      static_cast<std::int64_t>(message.header.stamp.nanosec);
  if (timestamp_ns <= 0) {
    return std::nullopt;
  }
  const Eigen::Quaterniond orientation_ros(
      message.pose.pose.orientation.w, message.pose.pose.orientation.x,
      message.pose.pose.orientation.y, message.pose.pose.orientation.z);
  if (!orientation_ros.coeffs().allFinite() || orientation_ros.norm() < 1e-9) {
    return std::nullopt;
  }

  const auto& c_ned = C_ned_from_lio_enu();
  const auto& c_body = C_body_frd_from_body_flu();
  const Eigen::Matrix3d rotation_enu_flu = orientation_ros.normalized().toRotationMatrix();
  const Eigen::Matrix3d rotation_ned_frd = c_ned * rotation_enu_flu * c_body.inverse();
  const Eigen::Matrix3d rotation_ned_from_body_flu = c_ned * rotation_enu_flu;
  const auto position_variance = transformed_covariance_diagonal(
      message.pose.covariance, 0, c_ned);
  // VehicleOdometry.orientation_variance is expressed in body coordinates.
  // ROS pose orientation covariance is expressed in the LIO world basis, so
  // rotate world -> body-FLU -> body-FRD before taking the diagonal.
  const auto orientation_variance = transformed_covariance_diagonal(
      message.pose.covariance, 3, c_body * rotation_enu_flu.transpose());
  const auto velocity_variance = transformed_covariance_diagonal(
      message.twist.covariance, 0, rotation_ned_from_body_flu);
  if (!position_variance || !orientation_variance || !velocity_variance) {
    return std::nullopt;
  }

  ExternalOdometryFrame result;
  result.timestamp_ns = timestamp_ns;
  result.position_ned = c_ned * Eigen::Vector3d(
      message.pose.pose.position.x, message.pose.pose.position.y,
      message.pose.pose.position.z);
  result.orientation_ned = Eigen::Quaterniond(rotation_ned_frd).normalized();
  // nav_msgs/Odometry states twist in child_frame_id.  Here child_frame_id is
  // base_link (FLU), therefore convert body-FLU -> LIO ENU -> PX4 NED.  The
  // previous implementation only applied the body sign matrix and published
  // a body velocity while labeling the field as BODY_FRD; the new contract is
  // an explicit NED world velocity.
  const Eigen::Vector3d velocity_body_flu(
      message.twist.twist.linear.x, message.twist.twist.linear.y,
      message.twist.twist.linear.z);
  result.velocity_ned = rotation_ned_from_body_flu * velocity_body_flu;
  result.angular_velocity_body_frd = c_body * Eigen::Vector3d(
      message.twist.twist.angular.x, message.twist.twist.angular.y,
      message.twist.twist.angular.z);
  result.position_variance = *position_variance;
  result.orientation_variance = *orientation_variance;
  result.velocity_variance = *velocity_variance;
  result.frame_valid = result.position_ned.allFinite() &&
                       result.orientation_ned.coeffs().allFinite() &&
                       result.velocity_ned.allFinite() &&
                       result.angular_velocity_body_frd.allFinite();
  result.covariance_valid = result.position_variance.allFinite() &&
                            result.orientation_variance.allFinite() &&
                            result.velocity_variance.allFinite();
  if (!result.frame_valid || !result.covariance_valid) {
    return std::nullopt;
  }
  return result;
}

}  // namespace px4_odometry_bridge
