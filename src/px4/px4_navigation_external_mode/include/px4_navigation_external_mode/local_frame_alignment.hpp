#pragma once

#include <limits>
#include <optional>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <navigation_common/frame_conventions.hpp>

namespace px4_navigation_external_mode {

[[nodiscard]] inline std::optional<double> yawFromQuaternionEnu(
    const Eigen::Quaterniond& orientation) noexcept {
  if (!orientation.coeffs().allFinite() ||
      !std::isfinite(orientation.squaredNorm()) ||
      orientation.squaredNorm() <= 1.0e-12) {
    return std::nullopt;
  }
  const Eigen::Quaterniond normalized = orientation.normalized();
  const double yaw = std::atan2(
      2.0 * (normalized.w() * normalized.z() +
              normalized.x() * normalized.y()),
      1.0 - 2.0 * (normalized.y() * normalized.y() +
                    normalized.z() * normalized.z()));
  if (!std::isfinite(yaw)) return std::nullopt;
  return yaw;
}

[[nodiscard]] inline std::optional<double> yawResidualLioToPx4(
    const double lio_yaw_enu, const double px4_heading_ned) noexcept {
  if (!std::isfinite(lio_yaw_enu) || !std::isfinite(px4_heading_ned)) {
    return std::nullopt;
  }
  constexpr double kPi = 3.141592653589793238462643383279502884;
  const double px4_yaw_enu = 0.5 * kPi - px4_heading_ned;
  const double residual = std::remainder(lio_yaw_enu - px4_yaw_enu, 2.0 * kPi);
  if (!std::isfinite(residual)) return std::nullopt;
  return residual;
}

inline std::optional<Eigen::Vector3f> checkedEnuToNed(const Eigen::Vector3d& value_enu) {
  if (!value_enu.allFinite()) return std::nullopt;
  const Eigen::Vector3d value_ned = navigation_common::enuToNed(value_enu);
  if (!value_ned.allFinite() ||
      (value_ned.cwiseAbs().array() > static_cast<double>(std::numeric_limits<float>::max()))
          .any()) {
    return std::nullopt;
  }
  return value_ned.cast<float>();
}

// Planner positions live in the LIO local ENU frame while PX4 trajectory
// setpoints live in PX4's local NED frame. The basis conversion is fixed, but
// the local origins are not guaranteed to be identical when PX4 fuses GPS and
// external vision together. Capture this translation while stationary; do
// not continuously fit it during flight because that would hide tracking
// error.
[[nodiscard]] inline std::optional<Eigen::Vector3d> localNedTranslationFromStationaryPair(
    const Eigen::Vector3d& lio_position_enu,
    const Eigen::Vector3d& px4_position_ned) noexcept {
  if (!lio_position_enu.allFinite() || !px4_position_ned.allFinite()) {
    return std::nullopt;
  }
  const Eigen::Vector3d translation =
      px4_position_ned - navigation_common::enuToNed(lio_position_enu);
  if (!translation.allFinite()) return std::nullopt;
  return translation;
}

[[nodiscard]] inline std::optional<Eigen::Vector3d> lioPositionToLocalNed(
    const Eigen::Vector3d& lio_position_enu,
    const Eigen::Vector3d& translation_ned) noexcept {
  if (!lio_position_enu.allFinite() || !translation_ned.allFinite()) {
    return std::nullopt;
  }
  const Eigen::Vector3d position_ned =
      navigation_common::enuToNed(lio_position_enu) + translation_ned;
  if (!position_ned.allFinite()) return std::nullopt;
  return position_ned;
}

}  // namespace px4_navigation_external_mode
