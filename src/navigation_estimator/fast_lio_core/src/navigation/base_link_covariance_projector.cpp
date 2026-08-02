#include "fast_lio_core/navigation/base_link_covariance_projector.hpp"

#include <Eigen/Eigenvalues>
#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <string>

#include "fast_lio_core/geometry/frame_ids.hpp"

namespace uav::nav::lio {
namespace {

template <typename Matrix>
Eigen::Matrix<typename Matrix::Scalar, 3, 3> skew(
    const Eigen::MatrixBase<Matrix>& vector) {
  Eigen::Matrix<typename Matrix::Scalar, 3, 3> result;
  result << 0.0, -vector.z(), vector.y(), vector.z(), 0.0, -vector.x(),
      -vector.y(), vector.x(), 0.0;
  return result;
}

Status numericalFailure(std::string message) {
  return Status(StatusCode::kNumericalFailure, std::move(message));
}

template <typename Matrix>
bool repairOrReject(Matrix& covariance, bool& repaired,
                    bool& asymmetry_failure, bool& non_psd_failure,
                    const double& tolerance_scale =
                        BaseLinkCovarianceProjector::kPsdToleranceScale) {
  if (!covariance.allFinite()) {
    return false;
  }
  const double maximum_asymmetry =
      (covariance - covariance.transpose()).cwiseAbs().maxCoeff();
  if (maximum_asymmetry > BaseLinkCovarianceProjector::kAsymmetryTolerance) {
    asymmetry_failure = true;
    return false;
  }
  if (maximum_asymmetry > 0.0) {
    covariance = 0.5 * (covariance + covariance.transpose());
    repaired = true;
  }
  if (covariance.cwiseAbs().maxCoeff() == 0.0 ||
      covariance.trace() <= 0.0) {
    non_psd_failure = true;
    return false;
  }
  const double scale = std::max(1.0, covariance.diagonal().cwiseAbs().maxCoeff());
  const double tolerance = tolerance_scale * scale;
  Eigen::SelfAdjointEigenSolver<Matrix> solver(covariance);
  if (solver.info() != Eigen::Success) {
    non_psd_failure = true;
    return false;
  }
  const double minimum_eigenvalue = solver.eigenvalues().minCoeff();
  if (minimum_eigenvalue < -tolerance) {
    non_psd_failure = true;
    return false;
  }
  if (minimum_eigenvalue < 0.0) {
    auto eigenvalues = solver.eigenvalues();
    eigenvalues = eigenvalues.cwiseMax(0.0);
    covariance = solver.eigenvectors() * eigenvalues.asDiagonal() *
                 solver.eigenvectors().transpose();
    covariance = 0.5 * (covariance + covariance.transpose());
    repaired = true;
  }
  return true;
}

template <typename Matrix>
bool validateOutput(Matrix& covariance, bool& repaired, bool& nonfinite,
                    bool& non_psd) {
  if (!covariance.allFinite()) {
    nonfinite = true;
    return false;
  }
  return repairOrReject(covariance, repaired, non_psd, non_psd);
}

}  // namespace

BaseLinkCovarianceProjector::BaseLinkCovarianceProjector(
    RigidTransform base_to_imu)
    : base_to_imu_(std::move(base_to_imu)),
      r_base_imu_m_(base_to_imu_.translation()),
      R_base_imu_(base_to_imu_.rotation().toRotationMatrix()) {
  validateBaseToImu(base_to_imu_);
}

void BaseLinkCovarianceProjector::validateBaseToImu(
    const RigidTransform& base_to_imu) {
  if (base_to_imu.targetFrame() != baseFrame() ||
      base_to_imu.sourceFrame() != imuFrame() || !base_to_imu.allFinite() ||
      !base_to_imu.rotation().coeffs().allFinite() ||
      base_to_imu.rotation().norm() < 1e-12) {
    throw std::invalid_argument(
        "BaseLinkCovarianceProjector requires finite ^base_link T_livox_imu_frame");
  }
}

Matrix6x23d BaseLinkCovarianceProjector::poseStateJacobian(
    const KinematicStateEstimate& imu_estimate,
    const RigidBodyState& base_state) const {
  Matrix6x23d jacobian = Matrix6x23d::Zero();
  const Eigen::Matrix3d R_odom_base =
      base_state.orientation_reference_body.toRotationMatrix();
  const Eigen::Matrix3d R_odom_imu =
      imu_estimate.estimate.state.orientation_odom_imu().toRotationMatrix();
  jacobian.template block<3, 3>(0, ManifoldState::kPositionOffset) =
      Eigen::Matrix3d::Identity();
  jacobian.template block<3, 3>(0, ManifoldState::kOrientationOffset) =
      R_odom_base * skew(r_base_imu_m_) * R_base_imu_;
  jacobian.template block<3, 3>(3, ManifoldState::kOrientationOffset) =
      R_odom_imu;
  return jacobian;
}

Matrix6x23d BaseLinkCovarianceProjector::twistStateJacobian(
    const KinematicStateEstimate& imu_estimate,
    const RigidBodyState& base_state) const {
  Matrix6x23d jacobian = Matrix6x23d::Zero();
  const Eigen::Matrix3d R_odom_base =
      base_state.orientation_reference_body.toRotationMatrix();
  const Eigen::Matrix3d R_base_odom = R_odom_base.transpose();
  const Eigen::Vector3d a_base =
      R_base_odom * imu_estimate.estimate.state.velocity_odom_imu_m_s();
  jacobian.template block<3, 3>(0, ManifoldState::kOrientationOffset) =
      skew(a_base) * R_base_imu_;
  jacobian.template block<3, 3>(0, ManifoldState::kVelocityOffset) =
      R_base_odom;
  jacobian.template block<3, 3>(0, ManifoldState::kGyroBiasOffset) =
      -skew(r_base_imu_m_) * R_base_imu_;
  jacobian.template block<3, 3>(3, ManifoldState::kGyroBiasOffset) =
      -R_base_imu_;
  return jacobian;
}

Result<BaseLinkNavigationCovariance> BaseLinkCovarianceProjector::project(
    const KinematicStateEstimate& imu_estimate,
    const RigidBodyState& base_state,
    BaseLinkCovarianceProjectionDiagnostics* diagnostics) const {
  BaseLinkCovarianceProjectionDiagnostics local_diagnostics;
  auto& report = diagnostics == nullptr ? local_diagnostics : *diagnostics;
  report = {};
  if (!imu_estimate.estimate.state.allFinite() ||
      !imu_estimate.angular_velocity_imu_rad_s.allFinite() ||
      !base_state.allFinite() ||
      !base_state.angular_velocity_body_rad_s.has_value() ||
      base_state.reference_frame != odomFrame() ||
      base_state.body_frame != baseFrame() ||
      imu_estimate.estimate.time != base_state.time) {
    return numericalFailure(
        "Base-link covariance projection input is invalid or at a different epoch");
  }

  auto state_covariance = imu_estimate.estimate.covariance;
  bool source_repaired = false;
  if (!state_covariance.allFinite()) {
    report.source_nonfinite = true;
    return numericalFailure("State covariance is non-finite");
  }
  if (!repairOrReject(state_covariance, source_repaired,
                      report.source_asymmetry, report.source_non_psd)) {
    if (state_covariance.cwiseAbs().maxCoeff() == 0.0) {
      report.source_zero = true;
    }
    return numericalFailure("State covariance is not finite, symmetric, and PSD");
  }
  report.roundoff_repair = source_repaired;

  const Matrix6x23d pose_jacobian =
      poseStateJacobian(imu_estimate, base_state);
  const Matrix6x23d twist_jacobian =
      twistStateJacobian(imu_estimate, base_state);
  BaseLinkNavigationCovariance result;
  result.pose_covariance_odom =
      pose_jacobian * state_covariance * pose_jacobian.transpose();
  result.twist_covariance_base =
      twist_jacobian * state_covariance * twist_jacobian.transpose();
  bool pose_repaired = false;
  bool twist_repaired = false;
  const bool pose_valid = validateOutput(
      result.pose_covariance_odom, pose_repaired, report.output_pose_nonfinite,
      report.output_pose_non_psd);
  const bool twist_valid = validateOutput(
      result.twist_covariance_base, twist_repaired,
      report.output_twist_nonfinite, report.output_twist_non_psd);
  if (!pose_valid || !twist_valid) {
    return numericalFailure("Projected base-link covariance is invalid");
  }
  report.roundoff_repair = report.roundoff_repair || pose_repaired || twist_repaired;
  Eigen::SelfAdjointEigenSolver<Matrix6d> pose_solver(
      result.pose_covariance_odom, Eigen::EigenvaluesOnly);
  Eigen::SelfAdjointEigenSolver<Matrix6d> twist_solver(
      result.twist_covariance_base, Eigen::EigenvaluesOnly);
  if (pose_solver.info() != Eigen::Success ||
      twist_solver.info() != Eigen::Success) {
    return numericalFailure("Projected covariance eigenvalue validation failed");
  }
  report.pose_covariance_trace = result.pose_covariance_odom.trace();
  report.twist_covariance_trace = result.twist_covariance_base.trace();
  report.pose_covariance_minimum_eigenvalue = pose_solver.eigenvalues().minCoeff();
  report.twist_covariance_minimum_eigenvalue =
      twist_solver.eigenvalues().minCoeff();
  if (!(report.pose_covariance_trace > 0.0) ||
      !(report.twist_covariance_trace > 0.0)) {
    report.output_pose_non_psd = report.pose_covariance_trace <= 0.0;
    report.output_twist_non_psd = report.twist_covariance_trace <= 0.0;
    return numericalFailure("Projected covariance is zero or has no positive trace");
  }
  report.success = true;
  return result;
}

}  // namespace uav::nav::lio
