#pragma once

#include <Eigen/Core>

#include "fast_lio_core/estimation/ikfom_state.hpp"

namespace uav::nav::lio::test_reference {

struct MeasurementUpdate {
  Eigen::MatrixXd gain;
  Eigen::Matrix<double, 23, 23> gain_times_jacobian;
  Eigen::Matrix<double, 23, 1> increment;
  IkfomState corrected_state;
  Eigen::Matrix<double, 23, 23> corrected_covariance;
  bool converged{};
  double final_increment_norm{};
};

inline MeasurementUpdate denseMeasurementUpdate(
    const Eigen::Matrix<double, 23, 23>& covariance,
    const Eigen::MatrixXd& jacobian, const Eigen::VectorXd& variance,
    const Eigen::VectorXd& innovation,
    const Eigen::Matrix<double, 23, 1>& dx_new,
    const IkfomState& initial_state, double convergence_limit) {
  MeasurementUpdate result;
  const Eigen::MatrixXd residual_covariance =
      jacobian * covariance * jacobian.transpose() +
      variance.asDiagonal().toDenseMatrix();
  result.gain =
      covariance * jacobian.transpose() * residual_covariance.inverse();
  result.gain_times_jacobian = result.gain * jacobian;
  result.increment =
      result.gain * innovation +
      (result.gain_times_jacobian -
       Eigen::Matrix<double, 23, 23>::Identity()) *
          dx_new;
  result.corrected_state = initial_state;
  result.corrected_state.boxplus(result.increment);
  result.corrected_covariance =
      (Eigen::Matrix<double, 23, 23>::Identity() -
       result.gain_times_jacobian) *
      covariance;
  result.converged =
      (result.increment.array().abs() <= convergence_limit).all();
  result.final_increment_norm = result.increment.norm();
  return result;
}

}  // namespace uav::nav::lio::test_reference
