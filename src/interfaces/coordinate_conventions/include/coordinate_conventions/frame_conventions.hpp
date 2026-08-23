#pragma once

#include <Eigen/Core>

namespace coordinate_conventions {

// C_enu_ned maps a vector expressed in PX4 NED coordinates into ROS ENU.
// This matrix is symmetric, so it is also the ENU-to-NED setpoint transform.
inline const Eigen::Matrix3d& c_enu_ned() {
  static const Eigen::Matrix3d matrix =
      (Eigen::Matrix3d() << 0.0, 1.0, 0.0,
                            1.0, 0.0, 0.0,
                            0.0, 0.0, -1.0)
          .finished();
  return matrix;
}

inline Eigen::Vector3d enuToNed(const Eigen::Vector3d& value_enu) {
  return c_enu_ned() * value_enu;
}

inline Eigen::Vector3f enuToNed(const Eigen::Vector3f& value_enu) {
  return c_enu_ned().cast<float>() * value_enu;
}

inline Eigen::Vector3d nedToEnu(const Eigen::Vector3d& value_ned) {
  return c_enu_ned() * value_ned;
}

}  // namespace coordinate_conventions
