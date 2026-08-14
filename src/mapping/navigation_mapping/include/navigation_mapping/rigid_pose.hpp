#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace navigation_mapping {

// Explicit, direction-unambiguous transform type for this package (P1 code
// quality rule: no vague "transform"/"pose"/"tf" names). Represents
// ^target T_source: applying it to a point in `source` yields the point in
// `target`.
struct T_odom_lidar {
  Eigen::Vector3d translation_odom_m{Eigen::Vector3d::Zero()};
  Eigen::Quaterniond rotation_odom_lidar{Eigen::Quaterniond::Identity()};

  [[nodiscard]] Eigen::Vector3d apply(const Eigen::Vector3d& point_lidar_m) const noexcept {
    return (rotation_odom_lidar * point_lidar_m) + translation_odom_m;
  }

  [[nodiscard]] bool allFinite() const noexcept {
    return translation_odom_m.allFinite() && rotation_odom_lidar.coeffs().allFinite() &&
           rotation_odom_lidar.coeffs().squaredNorm() > 1e-6;
  }
};

}  // namespace navigation_mapping
