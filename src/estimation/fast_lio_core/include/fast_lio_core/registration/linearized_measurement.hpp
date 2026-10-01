#pragma once

#include <Eigen/Core>
#include <cstddef>

#include "fast_lio_core/estimation/manifold_state.hpp"

namespace uav::nav::lio {

struct LinearizedMeasurement {
  Eigen::Matrix<double, Eigen::Dynamic, ManifoldState::kErrorStateDimension>
      jacobian;
  Eigen::VectorXd residual_m;
  Eigen::VectorXd variance_m2;
  std::size_t rejected_residual_count{0};

  [[nodiscard]] bool valid() const noexcept;
  [[nodiscard]] double residualRmsM() const noexcept;
};

}  // namespace uav::nav::lio
