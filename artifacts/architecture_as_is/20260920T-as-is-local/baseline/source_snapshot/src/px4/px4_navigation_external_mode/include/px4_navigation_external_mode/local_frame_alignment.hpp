#pragma once

#include <optional>

#include <Eigen/Core>

#include <navigation_common/frame_conventions.hpp>

namespace px4_navigation_external_mode {

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
