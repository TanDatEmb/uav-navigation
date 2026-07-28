#pragma once

#include <cstddef>
#include <string>
#include <vector>

#include "fast_lio_core/estimation/manifold_state.hpp"

namespace uav::nav::lio {

enum class ConvergenceStatus {
  kIterating,
  kConverged,
  kMaximumIterations,
  kInsufficientResiduals,
  kNonFinite,
  kCorrectionTooLarge,
  kDiverged,
};

struct ConvergenceConfig {
  std::size_t maximum_iterations{4};
  std::size_t minimum_accepted_residuals{20};
  double translation_increment_threshold_m{1e-3};
  double rotation_increment_threshold_rad{1e-4};
  double maximum_translation_correction_m{2.0};
  double maximum_rotation_correction_rad{0.5};
  double divergence_rms_ratio{2.0};
};

struct IterationDiagnostics {
  std::size_t iteration{0};
  std::size_t accepted_residual_count{0};
  std::size_t rejected_residual_count{0};
  double residual_rms_m{0.0};
  double translation_increment_norm_m{0.0};
  double rotation_increment_norm_rad{0.0};
};

class ConvergenceMonitor {
 public:
  explicit ConvergenceMonitor(ConvergenceConfig config = {});

  void reset();
  [[nodiscard]] ConvergenceStatus observe(const ManifoldState::ErrorVector& increment,
                                          double residual_rms_m,
                                          std::size_t accepted_residual_count,
                                          std::size_t rejected_residual_count);

  [[nodiscard]] ConvergenceStatus status() const noexcept;
  [[nodiscard]] bool converged() const noexcept;
  [[nodiscard]] const std::vector<IterationDiagnostics>& iterations() const noexcept;
  [[nodiscard]] std::string reason() const;

 private:
  ConvergenceConfig config_;
  ConvergenceStatus status_{ConvergenceStatus::kIterating};
  std::vector<IterationDiagnostics> iterations_;
};

}  // namespace uav::nav::lio
