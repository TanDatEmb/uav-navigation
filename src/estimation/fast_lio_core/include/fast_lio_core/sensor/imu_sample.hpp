#pragma once

#include <Eigen/Core>

#include "fast_lio_core/common/status.hpp"
#include "fast_lio_core/time/timestamp.hpp"

namespace uav::nav::lio {

struct ImuSample {
  Timestamp time;
  Eigen::Vector3d angular_velocity_imu_rad_s{Eigen::Vector3d::Zero()};
  Eigen::Vector3d linear_acceleration_imu_m_s2{Eigen::Vector3d::Zero()};

  [[nodiscard]] Status validate() const;
};

}  // namespace uav::nav::lio
