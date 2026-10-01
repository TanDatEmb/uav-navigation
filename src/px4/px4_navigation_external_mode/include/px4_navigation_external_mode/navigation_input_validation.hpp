#pragma once

#include <cmath>

#include <Eigen/Geometry>

namespace px4_navigation_external_mode {

inline bool isNormalizableOdometryQuaternion(const Eigen::Quaterniond& quaternion) {
  const double squared_norm = quaternion.squaredNorm();
  return quaternion.coeffs().allFinite() && std::isfinite(squared_norm) &&
         squared_norm > 1.0e-12;
}

}  // namespace px4_navigation_external_mode
