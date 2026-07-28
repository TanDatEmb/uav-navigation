#pragma once

#include <cstddef>
#include <string>

#include "fast_lio_core/estimation/iterated_kalman_filter.hpp"
#include "fast_lio_core/registration/residual_builder.hpp"

namespace uav::nav::lio {

struct RegistrationResult {
  CorrectionResult correction{};
  ResidualBuildDiagnostics residual_diagnostics{};
  bool successful{false};
  std::string reason;

  [[nodiscard]] bool converged() const noexcept { return successful && correction.converged; }
};

}  // namespace uav::nav::lio
