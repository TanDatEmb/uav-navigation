#pragma once

#include <Eigen/Core>

#include <esekfom/esekfom.hpp>

#include "reference/dense_ikfom_measurement_update.hpp"

namespace uav::nav::lio::test_reference {

inline bool compactMeasurementUpdate(
    const Eigen::Matrix<double, 23, 23>& covariance,
    const Eigen::MatrixXd& jacobian, const Eigen::VectorXd& variance,
    const Eigen::VectorXd& innovation,
    const Eigen::Matrix<double, 23, 1>& dx_new,
    const IkfomState& initial_state, double convergence_limit,
    MeasurementUpdate& result) {
  Eigen::Matrix<double, 23, Eigen::Dynamic> gain;
  if (!esekfom::detail::solve_compact_normal_equations<double, 23>(
          covariance, jacobian, variance, gain)) {
    return false;
  }
  result.gain = gain;
  result.gain_times_jacobian = gain * jacobian;
  result.increment =
      gain * innovation +
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
  return result.gain.allFinite() && result.increment.allFinite() &&
         result.corrected_covariance.allFinite();
}

}  // namespace uav::nav::lio::test_reference
