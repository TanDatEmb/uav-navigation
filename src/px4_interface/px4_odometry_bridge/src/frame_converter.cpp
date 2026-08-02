#include "px4_odometry_bridge/frame_converter.hpp"

#include <cmath>

namespace px4_odometry_bridge {
namespace {

bool finite(const Eigen::Vector3d &v) { return v.allFinite(); }

Eigen::Vector3d rotate_variance(const Eigen::Matrix3d &rotation,
                                const Eigen::Vector3d &variance) {
  Eigen::Vector3d result = Eigen::Vector3d::Constant(-1.0);
  if (!finite(variance)) return result;
  for (int i = 0; i < 3; ++i) {
    if (variance[i] < 0.0 || !std::isfinite(variance[i])) continue;
    result[i] = (rotation.row(i).array().square() * variance.transpose().array()).sum();
  }
  return result;
}

}  // namespace

const Eigen::Matrix3d &FrameConverter::c_enu_ned() {
  static const Eigen::Matrix3d matrix = (Eigen::Matrix3d() << 0.0, 1.0, 0.0,
                                         1.0, 0.0, 0.0, 0.0, 0.0, -1.0)
                                            .finished();
  return matrix;
}

const Eigen::Matrix3d &FrameConverter::c_flu_frd() {
  static const Eigen::Matrix3d matrix = Eigen::DiagonalMatrix<double, 3>(1.0, -1.0, -1.0);
  return matrix;
}

const Eigen::Matrix3d &FrameConverter::c_zup_frd() {
  // PX4's FRD world convention differs from ROS Z-UP by the same reflection
  // used for body FRD -> FLU. Keeping it explicit prevents hidden sign fixes.
  return c_flu_frd();
}

ConversionResult FrameConverter::convert(const Px4OdometrySample &sample) {
  if (sample.timestamp_ns <= 0 || !finite(sample.position) ||
      !finite(sample.velocity) || !finite(sample.angular_velocity)) {
    return {std::nullopt, "timestamp or vector contains a non-finite/invalid value"};
  }
  if (!sample.angular_velocity_valid || sample.angular_velocity.squaredNorm() < 1e-18) {
    return {std::nullopt, "PX4 angular velocity is invalid or zero"};
  }

  const double norm = sample.orientation.norm();
  if (!std::isfinite(norm) || norm < 1e-9) {
    return {std::nullopt, "PX4 quaternion is near zero or non-finite"};
  }
  const Eigen::Quaterniond q = sample.orientation.normalized();
  Eigen::Quaterniond continuous_q = q;
  if (previous_orientation_.has_value() && previous_orientation_->dot(q) < 0.0) {
    continuous_q.coeffs() *= -1.0;
  }
  previous_orientation_ = continuous_q;

  Eigen::Matrix3d world_from_flu;
  Eigen::Vector3d position_world;
  Eigen::Vector3d velocity_world;
  switch (sample.pose_frame) {
    case PoseFrame::kNed:
      world_from_flu = c_enu_ned() * continuous_q.toRotationMatrix() * c_flu_frd();
      position_world = c_enu_ned() * sample.position;
      break;
    case PoseFrame::kFrd:
      world_from_flu = c_zup_frd() * continuous_q.toRotationMatrix() * c_flu_frd();
      position_world = c_zup_frd() * sample.position;
      break;
    default:
      return {std::nullopt, "unsupported PX4 pose frame"};
  }

  switch (sample.velocity_frame) {
    case VelocityFrame::kNed:
      velocity_world = c_enu_ned() * sample.velocity;
      break;
    case VelocityFrame::kFrd:
      velocity_world = c_zup_frd() * sample.velocity;
      break;
    case VelocityFrame::kBodyFrd:
      velocity_world = world_from_flu * c_flu_frd() * sample.velocity;
      break;
    default:
      return {std::nullopt, "unsupported PX4 velocity frame"};
  }

  ConvertedOdometry output;
  output.timestamp_ns = sample.timestamp_ns;
  output.position = position_world;
  output.orientation = Eigen::Quaterniond(world_from_flu).normalized();
  output.velocity_body = world_from_flu.transpose() * velocity_world;
  output.angular_velocity_body = c_flu_frd() * sample.angular_velocity;
  output.position_variance = rotate_variance(c_enu_ned(), sample.position_variance);
  output.velocity_variance = rotate_variance(world_from_flu.transpose(),
                                             rotate_variance(c_enu_ned(), sample.velocity_variance));
  output.orientation_variance = rotate_variance(c_flu_frd(), sample.orientation_variance);
  output.reset_counter = sample.reset_counter;
  output.angular_velocity_valid = true;
  return {output, {}};
}

}  // namespace px4_odometry_bridge
