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
  // PX4 NED -> ROS ENU coordinate conversion is its own inverse:
  //   [x_enu y_enu z_enu]^T = C_enu_ned [x_ned y_ned z_ned]^T
  //   x_enu=y_ned, y_enu=x_ned, z_enu=-z_ned.
  static const Eigen::Matrix3d matrix = (Eigen::Matrix3d() << 0.0, 1.0, 0.0,
                                         1.0, 0.0, 0.0, 0.0, 0.0, -1.0)
                                            .finished();
  return matrix;
}

const Eigen::Matrix3d &FrameConverter::c_flu_frd() {
  // ROS base_link is FLU; PX4 body fields are FRD.
  static const Eigen::Matrix3d matrix = Eigen::DiagonalMatrix<double, 3>(1.0, -1.0, -1.0);
  return matrix;
}

ConversionResult FrameConverter::convert(const Px4OdometrySample &sample) {
  if (sample.timestamp_ns <= 0 || !finite(sample.position) ||
      !finite(sample.velocity) || !finite(sample.angular_velocity)) {
    return {std::nullopt, "timestamp or vector contains a non-finite/invalid value"};
  }
  if (!sample.angular_velocity_valid) {
    return {std::nullopt, "PX4 angular velocity is invalid"};
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
      // POSE_FRAME_FRD carries an arbitrary local heading.  Without a
      // measured ^lio_odom T_px4_frd alignment, applying a guessed yaw here
      // silently mirrors/rotates every position, velocity, and yaw.  Reject
      // it instead of manufacturing a transform.
      return {std::nullopt, "PX4 FRD pose frame requires an explicit world alignment"};
    default:
      return {std::nullopt, "unsupported PX4 pose frame"};
  }

  switch (sample.velocity_frame) {
    case VelocityFrame::kNed:
      if (sample.pose_frame != PoseFrame::kNed) {
        return {std::nullopt, "NED velocity cannot be combined with a non-NED pose frame"};
      }
      velocity_world = c_enu_ned() * sample.velocity;
      break;
    case VelocityFrame::kFrd:
      return {std::nullopt, "PX4 FRD world velocity requires an explicit world alignment"};
    case VelocityFrame::kBodyFrd:
      velocity_world = world_from_flu * c_flu_frd() * sample.velocity;
      break;
    default:
      return {std::nullopt, "unsupported PX4 velocity frame"};
  }

  ConvertedOdometry output;
  output.timestamp_ns = sample.timestamp_ns;
  output.source_pose_frame = sample.pose_frame;
  output.source_velocity_frame = sample.velocity_frame;
  output.world_convention = sample.pose_frame == PoseFrame::kNed
                                ? WorldConvention::kRosEnu
                                : WorldConvention::kPx4FrdLocal;
  output.position = position_world;
  output.orientation = Eigen::Quaterniond(world_from_flu).normalized();
  output.velocity_world = velocity_world;
  output.velocity_body = world_from_flu.transpose() * velocity_world;
  output.angular_velocity_body = c_flu_frd() * sample.angular_velocity;
  const Eigen::Matrix3d position_basis =
      c_enu_ned();
  output.position_variance = rotate_variance(position_basis, sample.position_variance);
  if (sample.velocity_frame == VelocityFrame::kBodyFrd) {
    output.velocity_variance = rotate_variance(c_flu_frd(), sample.velocity_variance);
  } else {
    const Eigen::Matrix3d velocity_world_basis = c_enu_ned();
    output.velocity_variance = rotate_variance(
        world_from_flu.transpose(),
        rotate_variance(velocity_world_basis, sample.velocity_variance));
  }
  output.orientation_variance = rotate_variance(
      world_from_flu * c_flu_frd(), sample.orientation_variance);
  output.reset_counter = sample.reset_counter;
  output.position_covariance_available = output.position_variance.allFinite() &&
                                         (output.position_variance.array() >= 0.0).all();
  output.velocity_covariance_available = output.velocity_variance.allFinite() &&
                                         (output.velocity_variance.array() >= 0.0).all();
  output.orientation_covariance_available = output.orientation_variance.allFinite() &&
                                            (output.orientation_variance.array() >= 0.0).all();
  output.angular_velocity_valid = true;
  return {output, {}};
}

}  // namespace px4_odometry_bridge
