#pragma once

#include <cmath>
#include <cstdint>
#include <string>

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace navigation_planning {

struct KinematicState {
  Eigen::Vector3d position_world{Eigen::Vector3d::Zero()};
  Eigen::Vector3d velocity_world{Eigen::Vector3d::Zero()};
  Eigen::Vector3d acceleration_world{Eigen::Vector3d::Zero()};
  Eigen::Vector3d jerk_world{Eigen::Vector3d::Zero()};
  // PropagatedOdometry does not currently carry A/J.  These flags distinguish
  // finite-difference estimates from a future genuinely measured interface.
  bool acceleration_estimated{false};
  bool jerk_estimated{false};
  Eigen::Quaterniond orientation_world_body{Eigen::Quaterniond::Identity()};
  double yaw_rad{0.0};
  std::int64_t source_stamp_ns{0};
  std::int64_t receive_stamp_ns{0};
  std::uint64_t localization_epoch{0};
  std::string world_frame_id;
  std::string body_frame_id;

  [[nodiscard]] bool finite() const noexcept {
    const double quaternion_scale = orientation_world_body.coeffs().cwiseAbs().maxCoeff();
    const auto scaled_quaternion = orientation_world_body.coeffs() / quaternion_scale;
    const long double quaternion_norm =
        static_cast<long double>(quaternion_scale) *
        std::sqrt(static_cast<long double>(scaled_quaternion.squaredNorm()));
    return position_world.allFinite() && velocity_world.allFinite() &&
           acceleration_world.allFinite() && jerk_world.allFinite() &&
           orientation_world_body.coeffs().allFinite() &&
           std::isfinite(quaternion_scale) && quaternion_scale > 1.0e-9 &&
           std::isfinite(static_cast<double>(quaternion_norm)) &&
           std::abs(quaternion_norm - 1.0L) <= 1.0e-6L &&
           source_stamp_ns > 0 &&
           receive_stamp_ns > 0 && std::isfinite(yaw_rad) && localization_epoch != 0 &&
           !world_frame_id.empty() && !body_frame_id.empty();
  }
};

}  // namespace navigation_planning
