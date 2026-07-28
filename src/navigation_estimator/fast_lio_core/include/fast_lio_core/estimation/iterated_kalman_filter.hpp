#pragma once

#include <Eigen/Core>
#include <cstddef>
#include <functional>
#include <string>
#include <vector>

#include "fast_lio_core/estimation/convergence_monitor.hpp"
#include "fast_lio_core/estimation/manifold_state.hpp"

namespace uav::nav::lio {

struct LinearizedMeasurement {
  Eigen::Matrix<double, Eigen::Dynamic, ManifoldState::kErrorStateDimension> jacobian;
  Eigen::VectorXd residual_m;
  Eigen::VectorXd variance_m2;
  std::size_t rejected_residual_count{0};

  [[nodiscard]] bool valid() const noexcept;
  [[nodiscard]] double residualRmsM() const noexcept;
};

struct IteratedKalmanFilterConfig {
  ConvergenceConfig convergence{};
  double normal_equation_damping{1e-9};
  double minimum_measurement_variance_m2{1e-8};
  double minimum_covariance_diagonal{1e-12};
  bool estimate_extrinsic{false};
};

struct CorrectionResult {
  ManifoldState corrected_state{};
  ManifoldState::Covariance corrected_covariance{ManifoldState::Covariance::Identity()};
  ConvergenceStatus status{ConvergenceStatus::kIterating};
  bool successful{false};
  bool converged{false};
  bool finite{true};
  std::string reason;
  std::vector<IterationDiagnostics> iterations;
  std::size_t accepted_residual_count{0};
  std::size_t rejected_residual_count{0};
  double residual_rms_m{0.0};
  double correction_translation_norm_m{0.0};
  double correction_rotation_norm_rad{0.0};
};

class IteratedKalmanFilter {
 public:
  using MeasurementBuilder = std::function<LinearizedMeasurement(const ManifoldState&)>;

  explicit IteratedKalmanFilter(IteratedKalmanFilterConfig config = {});

  [[nodiscard]] CorrectionResult correct(const ManifoldState& predicted_state,
                                         const ManifoldState::Covariance& predicted_covariance,
                                         const MeasurementBuilder& build_measurement) const;

 private:
  IteratedKalmanFilterConfig config_;
};

}  // namespace uav::nav::lio
