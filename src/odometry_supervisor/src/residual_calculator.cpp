#include "odometry_supervisor/residual_calculator.hpp"

#include <algorithm>
#include <cmath>

namespace odometry_supervisor {

bool ResidualCalculator::valid(const OdometryState& state) {
  return state.valid && state.timestamp_ns > 0 && state.frame_id == "odom" &&
         state.child_frame_id == "base_link" && state.position_odom.allFinite() &&
         state.velocity_base.allFinite() && state.orientation_odom_base.coeffs().allFinite() &&
         std::isfinite(state.orientation_odom_base.norm()) &&
         state.orientation_odom_base.norm() > 1e-9;
}

std::optional<Residual> ResidualCalculator::compare(
    const OdometryState& lio, const OdometryState& px4,
    const std::optional<Residual>& previous) {
  if (!valid(lio) || !valid(px4) || lio.timestamp_ns != px4.timestamp_ns) {
    return std::nullopt;
  }
  const auto lio_q = lio.orientation_odom_base.normalized();
  const auto px4_q = px4.orientation_odom_base.normalized();
  const Eigen::Vector3d lio_velocity_odom = lio_q * lio.velocity_base;
  const Eigen::Vector3d px4_velocity_odom = px4_q * px4.velocity_base;
  const Eigen::Quaterniond q_error = (px4_q.conjugate() * lio_q).normalized();
  const double scalar = std::clamp(std::abs(q_error.w()), 0.0, 1.0);
  const Eigen::Vector3d lio_euler = lio_q.toRotationMatrix().eulerAngles(0, 1, 2);
  const Eigen::Vector3d px4_euler = px4_q.toRotationMatrix().eulerAngles(0, 1, 2);
  Residual result;
  result.valid = true;
  result.timestamp_ns = lio.timestamp_ns;
  result.position_error_m = (lio.position_odom - px4.position_odom).norm();
  result.velocity_error_m_s = (lio_velocity_odom - px4_velocity_odom).norm();
  result.orientation_error_rad = 2.0 * std::acos(scalar);
  result.yaw_error_rad = wrap_to_pi(lio_euler.z() - px4_euler.z());
  if (previous && previous->valid && previous->timestamp_ns < result.timestamp_ns) {
    const double dt = static_cast<double>(result.timestamp_ns - previous->timestamp_ns) * 1e-9;
    result.position_error_growth_m_s =
        (result.position_error_m - previous->position_error_m) / dt;
  }
  if (!std::isfinite(result.position_error_m) || !std::isfinite(result.velocity_error_m_s) ||
      !std::isfinite(result.orientation_error_rad) || !std::isfinite(result.yaw_error_rad) ||
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
