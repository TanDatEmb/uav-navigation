#include "odometry_supervisor/residual_calculator.hpp"

#include <algorithm>
#include <cmath>

namespace odometry_supervisor {

namespace {

constexpr double kHeadingProjectionMinimum = 1e-6;

std::optional<double> projected_body_x_heading(const Eigen::Matrix3d& rotation) {
  const double horizontal_norm = std::hypot(rotation(0, 0), rotation(1, 0));
  if (horizontal_norm < kHeadingProjectionMinimum) {
    return std::nullopt;
  }
  return std::atan2(rotation(1, 0), rotation(0, 0));
}

}  // namespace

bool ResidualCalculator::valid(const OdometryState& state) {
  return state.valid && state.timestamp_ns > 0 &&
         (state.frame_id == "lio_odom" || state.frame_id == "px4_odom") &&
         state.child_frame_id == "base_link" && state.position_odom.allFinite() &&
         state.velocity_base.allFinite() && state.orientation_odom_base.coeffs().allFinite() &&
         std::isfinite(state.orientation_odom_base.norm()) &&
         state.orientation_odom_base.norm() > 1e-9;
}

std::optional<Residual> ResidualCalculator::compare(
    const OdometryState& lio, const OdometryState& px4,
    const std::optional<Residual>& previous) {
  if (!valid(lio) || !valid(px4) || lio.frame_id != px4.frame_id ||
      lio.timestamp_ns != px4.timestamp_ns) {
    return std::nullopt;
  }
  const auto lio_q = lio.orientation_odom_base.normalized();
  const auto px4_q = px4.orientation_odom_base.normalized();
  const auto lio_rotation = lio_q.toRotationMatrix();
  const auto px4_rotation = px4_q.toRotationMatrix();
  const Eigen::Vector3d lio_velocity_odom = lio_q * lio.velocity_base;
  const Eigen::Vector3d px4_velocity_odom = px4_q * px4.velocity_base;
  Eigen::Quaterniond q_error = (px4_q.conjugate() * lio_q).normalized();
  if (q_error.w() < 0.0) {
    q_error.coeffs() *= -1.0;
  }
  const double scalar = std::clamp(q_error.w(), 0.0, 1.0);
  const auto lio_heading = projected_body_x_heading(lio_rotation);
  const auto px4_heading = projected_body_x_heading(px4_rotation);
  Residual result;
  result.valid = true;
  result.timestamp_ns = lio.timestamp_ns;
  result.position_error_m = (lio.position_odom - px4.position_odom).norm();
  result.velocity_error_m_s = (lio_velocity_odom - px4_velocity_odom).norm();
  result.orientation_error_rad = 2.0 * std::acos(scalar);
  result.body_z_dot = lio_rotation.col(2).dot(px4_rotation.col(2));
  result.body_x_horizontal_norm_lio = std::hypot(lio_rotation(0, 0), lio_rotation(1, 0));
  result.body_x_horizontal_norm_px4 = std::hypot(px4_rotation(0, 0), px4_rotation(1, 0));
  if (q_error.vec().norm() > 1e-9) {
    result.q_error_axis = q_error.vec().normalized();
  }
  const Eigen::Vector3d lio_euler = lio_rotation.eulerAngles(0, 1, 2);
  const Eigen::Vector3d px4_euler = px4_rotation.eulerAngles(0, 1, 2);
  result.euler_yaw_error_rad = wrap_to_pi(lio_euler.z() - px4_euler.z());
  if (!lio_heading || !px4_heading) {
    return std::nullopt;
  }
  result.heading_observable = true;
  result.robust_heading_lio_rad = *lio_heading;
  result.robust_heading_px4_rad = *px4_heading;
  result.yaw_error_rad = wrap_to_pi(*lio_heading - *px4_heading);
  if (previous && previous->valid && previous->timestamp_ns < result.timestamp_ns) {
    const double dt = static_cast<double>(result.timestamp_ns - previous->timestamp_ns) * 1e-9;
    result.position_error_growth_m_s =
        (result.position_error_m - previous->position_error_m) / dt;
  }
  if (!std::isfinite(result.position_error_m) || !std::isfinite(result.velocity_error_m_s) ||
      !std::isfinite(result.orientation_error_rad) || !std::isfinite(result.yaw_error_rad) ||
      !std::isfinite(result.euler_yaw_error_rad) || !result.q_error_axis.allFinite() ||
      !std::isfinite(result.body_z_dot) ||
      !std::isfinite(result.body_x_horizontal_norm_lio) ||
      !std::isfinite(result.body_x_horizontal_norm_px4) ||
      !std::isfinite(result.position_error_growth_m_s)) {
    return std::nullopt;
  }
  return result;
}

double ResidualCalculator::wrap_to_pi(double angle) {
  constexpr double pi = 3.14159265358979323846;
  while (angle > pi) angle -= 2.0 * pi;
  while (angle < -pi) angle += 2.0 * pi;
  return angle;
}

}  // namespace odometry_supervisor
