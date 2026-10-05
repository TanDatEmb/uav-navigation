#pragma once

#include <cmath>
#include <limits>

#include <Eigen/Geometry>

namespace px4_navigation_external_mode {

inline bool floatRepresentable(const double value) {
  return std::isfinite(value) &&
         std::abs(value) <= static_cast<double>(std::numeric_limits<float>::max());
}

inline bool isNormalizableOdometryQuaternion(const Eigen::Quaterniond& quaternion) {
  const double squared_norm = quaternion.squaredNorm();
  return quaternion.coeffs().allFinite() && std::isfinite(squared_norm) &&
         squared_norm > 1.0e-12;
}

}  // namespace px4_navigation_external_mode
