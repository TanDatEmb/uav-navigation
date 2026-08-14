#pragma once

#include <Eigen/Core>

#include "fast_lio_core/estimation/state_estimate.hpp"

namespace uav::nav::lio {

// A navigation state and the IMU angular velocity resolved at exactly the
// same epoch.  Keeping the pair together prevents downstream serializers
// from accidentally combining a state with a later "latest IMU" sample.
struct KinematicStateEstimate {
  StateEstimate estimate;
  Eigen::Vector3d angular_velocity_imu_rad_s{Eigen::Vector3d::Zero()};

  [[nodiscard]] bool allFinite() const noexcept {
    return estimate.allFinite() && angular_velocity_imu_rad_s.allFinite();
  }
};

}  // namespace uav::nav::lio
