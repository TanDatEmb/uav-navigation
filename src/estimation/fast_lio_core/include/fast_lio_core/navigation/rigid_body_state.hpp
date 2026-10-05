#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <cmath>
#include <optional>

#include "fast_lio_core/geometry/frame.hpp"
#include "fast_lio_core/time/timestamp.hpp"

namespace uav::nav::lio {

// Kinematics for one rigid body state. Position and orientation convert body
// coordinates into reference coordinates; the two linear-velocity fields make
// their expression frames explicit. This type intentionally has no
// covariance: StateEstimate::covariance remains an IMU-origin error-state
// covariance until the covariance projection contract is available.
struct RigidBodyState {
  Timestamp time;

  // Metadata retained from the source estimate and the converted output.
  FrameId source_frame;
  FrameId reference_frame;
  FrameId body_frame;

  // ^reference T_body, expressed as position of body in reference and the
  // rotation that maps body vectors into reference vectors.
  Eigen::Vector3d position_reference_body_m{Eigen::Vector3d::Zero()};
  Eigen::Quaterniond orientation_reference_body{Eigen::Quaterniond::Identity()};

  // The same physical velocity, expressed at the converted body origin in
  // the reference frame and in the body frame respectively.
  Eigen::Vector3d linear_velocity_reference_body_m_s{Eigen::Vector3d::Zero()};
  Eigen::Vector3d linear_velocity_body_m_s{Eigen::Vector3d::Zero()};

  // The runtime contract requires this input from the caller. There is no
  // zero fallback.
  std::optional<Eigen::Vector3d> angular_velocity_body_rad_s;

  [[nodiscard]] bool allFinite() const noexcept {
    return position_reference_body_m.allFinite() &&
           orientation_reference_body.coeffs().allFinite() &&
           std::isfinite(orientation_reference_body.squaredNorm()) &&
           orientation_reference_body.squaredNorm() > 1e-24 &&
           linear_velocity_reference_body_m_s.allFinite() &&
           linear_velocity_body_m_s.allFinite() &&
           (!angular_velocity_body_rad_s.has_value() ||
            angular_velocity_body_rad_s->allFinite());
  }
};

}  // namespace uav::nav::lio
