#pragma once

#include <Eigen/Core>
#include <cstddef>

#include "fast_lio_core/common/result.hpp"
#include "fast_lio_core/geometry/rigid_transform.hpp"
#include "fast_lio_core/navigation/kinematic_state_estimate.hpp"
#include "fast_lio_core/navigation/rigid_body_state.hpp"

namespace uav::nav::lio {

using Matrix6d = Eigen::Matrix<double, 6, 6>;
using Matrix6x23d =
    Eigen::Matrix<double, 6, ManifoldState::kErrorStateDimension>;

struct BaseLinkNavigationCovariance {
  Matrix6d pose_covariance_odom{Matrix6d::Zero()};
  Matrix6d twist_covariance_base{Matrix6d::Zero()};

  [[nodiscard]] bool allFinite() const noexcept {
    return pose_covariance_odom.allFinite() &&
           twist_covariance_base.allFinite();
  }
};

struct BaseLinkCovarianceProjectionDiagnostics {
  bool success{false};
  bool source_nonfinite{false};
  bool source_asymmetry{false};
  bool source_non_psd{false};
  bool source_zero{false};
  bool output_pose_nonfinite{false};
  bool output_twist_nonfinite{false};
  bool output_pose_non_psd{false};
  bool output_twist_non_psd{false};
  bool roundoff_repair{false};
  double pose_covariance_trace{0.0};
  double twist_covariance_trace{0.0};
  double pose_covariance_minimum_eigenvalue{0.0};
  double twist_covariance_minimum_eigenvalue{0.0};
};

class BaseLinkCovarianceProjector {
 public:
  static constexpr double kAsymmetryTolerance = 1e-8;
  static constexpr double kPsdToleranceScale = 1e-10;

  explicit BaseLinkCovarianceProjector(RigidTransform base_to_imu);

  [[nodiscard]] Result<BaseLinkNavigationCovariance> project(
      const KinematicStateEstimate& imu_estimate,
      const RigidBodyState& base_state,
      BaseLinkCovarianceProjectionDiagnostics* diagnostics = nullptr) const;

  [[nodiscard]] Matrix6x23d poseStateJacobian(
      const KinematicStateEstimate& imu_estimate,
      const RigidBodyState& base_state) const;

  [[nodiscard]] Matrix6x23d twistStateJacobian(
      const KinematicStateEstimate& imu_estimate,
      const RigidBodyState& base_state) const;

 private:
  static void validateBaseToImu(const RigidTransform& base_to_imu);

  RigidTransform base_to_imu_;
  Eigen::Vector3d r_base_imu_m_;
  Eigen::Matrix3d R_base_imu_;
};

}  // namespace uav::nav::lio
