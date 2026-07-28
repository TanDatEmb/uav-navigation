#include "fast_lio_core/registration/linearized_measurement.hpp"

#include <cmath>

namespace uav::nav::lio {

bool LinearizedMeasurement::valid() const noexcept {
  return residual_m.size() > 0 && jacobian.rows() == residual_m.size() &&
         variance_m2.size() == residual_m.size() &&
         jacobian.cols() == ManifoldState::kErrorStateDimension &&
         jacobian.allFinite() && residual_m.allFinite() &&
         variance_m2.allFinite() && (variance_m2.array() > 0.0).all();
}

double LinearizedMeasurement::residualRmsM() const noexcept {
  if (residual_m.size() == 0 || !residual_m.allFinite()) {
    return 0.0;
  }
  return std::sqrt(residual_m.squaredNorm() /
                   static_cast<double>(residual_m.size()));
}

}  // namespace uav::nav::lio
