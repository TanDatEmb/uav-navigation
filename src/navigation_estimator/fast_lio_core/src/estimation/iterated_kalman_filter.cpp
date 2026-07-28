#include "fast_lio_core/estimation/iterated_kalman_filter.hpp"

#include <Eigen/Cholesky>
#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace uav::nav::lio {
namespace {

using Matrix23 = ManifoldState::Covariance;
using Vector23 = ManifoldState::ErrorVector;

bool covarianceIsFinite(const Matrix23& covariance) {
  return covariance.allFinite() && (covariance.diagonal().array() > 0.0).all();
}

Matrix23 symmetrizeAndFloor(Matrix23 covariance, double minimum_diagonal) {
  covariance = 0.5 * (covariance + covariance.transpose());
  for (Eigen::Index index = 0; index < covariance.rows(); ++index) {
    covariance(index, index) = std::max(covariance(index, index), minimum_diagonal);
  }
  return covariance;
}

}  // namespace

bool LinearizedMeasurement::valid() const noexcept {
  return jacobian.rows() > 0 && jacobian.cols() == ManifoldState::kErrorStateDimension &&
         residual_m.size() == jacobian.rows() && variance_m2.size() == jacobian.rows() &&
         jacobian.allFinite() && residual_m.allFinite() && variance_m2.allFinite() &&
         (variance_m2.array() > 0.0).all();
}

double LinearizedMeasurement::residualRmsM() const noexcept {
  if (residual_m.size() == 0) {
    return 0.0;
  }
  return std::sqrt(residual_m.squaredNorm() / static_cast<double>(residual_m.size()));
}

IteratedKalmanFilter::IteratedKalmanFilter(IteratedKalmanFilterConfig config) : config_(config) {
  if (!(config_.normal_equation_damping >= 0.0) ||
      !(config_.minimum_measurement_variance_m2 > 0.0) ||
      !(config_.minimum_covariance_diagonal > 0.0)) {
    throw std::invalid_argument("iterated Kalman filter numeric limits must be positive");
  }
}

CorrectionResult IteratedKalmanFilter::correct(
    const ManifoldState& predicted_state, const ManifoldState::Covariance& predicted_covariance,
    const MeasurementBuilder& build_measurement) const {
  CorrectionResult result;
  result.corrected_state = predicted_state;
  result.corrected_covariance = predicted_covariance;

  if (!predicted_state.allFinite() || !covarianceIsFinite(predicted_covariance) ||
      !build_measurement) {
    result.status = ConvergenceStatus::kNonFinite;
    result.successful = false;
    result.finite = false;
    result.reason = "INVALID_PREDICTED_STATE_OR_COVARIANCE";
    return result;
  }

  const Matrix23 covariance =
      symmetrizeAndFloor(predicted_covariance, config_.minimum_covariance_diagonal);
  const Eigen::LDLT<Matrix23> covariance_ldlt(covariance);
  if (covariance_ldlt.info() != Eigen::Success || !covariance_ldlt.isPositive()) {
    result.status = ConvergenceStatus::kNonFinite;
    result.successful = false;
    result.finite = false;
    result.reason = "PREDICTED_COVARIANCE_NOT_POSITIVE_DEFINITE";
    return result;
  }
  const Matrix23 information = covariance_ldlt.solve(Matrix23::Identity());

  ManifoldState iterate = predicted_state;
  Matrix23 final_normal = information;
  ConvergenceMonitor convergence(config_.convergence);
  LinearizedMeasurement measurement;

  while (convergence.status() == ConvergenceStatus::kIterating) {
    measurement = build_measurement(iterate);
    if (static_cast<std::size_t>(measurement.residual_m.size()) <
            config_.convergence.minimum_accepted_residuals &&
        measurement.jacobian.rows() == measurement.residual_m.size() &&
        measurement.variance_m2.size() == measurement.residual_m.size() &&
        measurement.jacobian.cols() == ManifoldState::kErrorStateDimension &&
        measurement.jacobian.allFinite() && measurement.residual_m.allFinite() &&
        measurement.variance_m2.allFinite()) {
      static_cast<void>(convergence.observe(Vector23::Zero(), measurement.residualRmsM(),
                                            static_cast<std::size_t>(measurement.residual_m.size()),
                                            measurement.rejected_residual_count));
      break;
    }
    if (!measurement.valid()) {
      Vector23 invalid_increment = Vector23::Constant(std::numeric_limits<double>::quiet_NaN());
      static_cast<void>(convergence.observe(invalid_increment,
                                            std::numeric_limits<double>::quiet_NaN(),
                                            static_cast<std::size_t>(measurement.residual_m.size()),
                                            measurement.rejected_residual_count));
      break;
    }

    const Eigen::VectorXd variances =
        measurement.variance_m2.array().max(config_.minimum_measurement_variance_m2);
    const Eigen::VectorXd weights = variances.cwiseInverse();
    const auto weighted_jacobian = measurement.jacobian.array().colwise() * weights.array();

    final_normal = information + measurement.jacobian.transpose() * weighted_jacobian.matrix();
    final_normal.diagonal().array() += config_.normal_equation_damping;

    const Vector23 displacement = iterate.boxMinus(predicted_state, config_.estimate_extrinsic);
    const Vector23 gradient = information * displacement +
                              measurement.jacobian.transpose() *
                                  (weights.array() * measurement.residual_m.array()).matrix();

    const Eigen::LDLT<Matrix23> normal_ldlt(final_normal);
    Vector23 increment = Vector23::Constant(std::numeric_limits<double>::quiet_NaN());
    if (normal_ldlt.info() == Eigen::Success && normal_ldlt.isPositive()) {
      increment = -normal_ldlt.solve(gradient);
    }

    const ConvergenceStatus status =
        convergence.observe(increment, measurement.residualRmsM(),
                            static_cast<std::size_t>(measurement.residual_m.size()),
                            measurement.rejected_residual_count);
    if (!increment.allFinite() || status == ConvergenceStatus::kCorrectionTooLarge ||
        status == ConvergenceStatus::kDiverged ||
        status == ConvergenceStatus::kInsufficientResiduals ||
        status == ConvergenceStatus::kNonFinite) {
      break;
    }

    ManifoldState candidate = iterate;
    candidate.applyErrorState(increment, config_.estimate_extrinsic);
    if (!candidate.allFinite()) {
      result.finite = false;
      break;
    }
    iterate = candidate;
  }

  result.status = convergence.status();
  result.converged = convergence.converged();
  result.successful = result.converged && iterate.allFinite();
  result.finite = result.finite && iterate.allFinite() && final_normal.allFinite();
  result.reason = convergence.reason();
  result.iterations = convergence.iterations();
  result.corrected_state = iterate;

  if (!result.iterations.empty()) {
    const auto& last = result.iterations.back();
    result.accepted_residual_count = last.accepted_residual_count;
    result.rejected_residual_count = last.rejected_residual_count;
    result.residual_rms_m = last.residual_rms_m;
  }
  const Vector23 total_correction = iterate.boxMinus(predicted_state, config_.estimate_extrinsic);
  result.correction_translation_norm_m =
      total_correction.segment<3>(ManifoldState::kPositionOffset).norm();
  result.correction_rotation_norm_rad =
      total_correction.segment<3>(ManifoldState::kOrientationOffset).norm();

  if (result.finite) {
    const Eigen::LDLT<Matrix23> posterior_ldlt(final_normal);
    if (posterior_ldlt.info() == Eigen::Success && posterior_ldlt.isPositive()) {
      result.corrected_covariance = symmetrizeAndFloor(posterior_ldlt.solve(Matrix23::Identity()),
                                                       config_.minimum_covariance_diagonal);
    } else {
      result.finite = false;
      result.successful = false;
      result.reason = "POSTERIOR_COVARIANCE_SOLVE_FAILED";
    }
  }
  return result;
}

}  // namespace uav::nav::lio
