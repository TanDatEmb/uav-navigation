#include "fast_lio_core/estimation/convergence_monitor.hpp"

#include <cmath>
#include <limits>
#include <stdexcept>

namespace uav::nav::lio {

ConvergenceMonitor::ConvergenceMonitor(ConvergenceConfig config) : config_(config) {
  if (config_.maximum_iterations == 0U) {
    throw std::invalid_argument("maximum_iterations must be positive");
  }
  if (config_.minimum_accepted_residuals == 0U) {
    throw std::invalid_argument("minimum_accepted_residuals must be positive");
  }
}

void ConvergenceMonitor::reset() {
  status_ = ConvergenceStatus::kIterating;
  iterations_.clear();
}

ConvergenceStatus ConvergenceMonitor::observe(const ManifoldState::ErrorVector& increment,
                                              double residual_rms_m,
                                              std::size_t accepted_residual_count,
                                              std::size_t rejected_residual_count) {
  if (status_ != ConvergenceStatus::kIterating) {
    return status_;
  }

  IterationDiagnostics diagnostics;
  diagnostics.iteration = iterations_.size() + 1U;
  diagnostics.accepted_residual_count = accepted_residual_count;
  diagnostics.rejected_residual_count = rejected_residual_count;
  diagnostics.residual_rms_m = residual_rms_m;
  diagnostics.translation_increment_norm_m =
      increment.segment<3>(ManifoldState::kPositionOffset).norm();
  diagnostics.rotation_increment_norm_rad =
      increment.segment<3>(ManifoldState::kOrientationOffset).norm();
  iterations_.push_back(diagnostics);

  if (!increment.allFinite() || !std::isfinite(residual_rms_m)) {
    status_ = ConvergenceStatus::kNonFinite;
  } else if (accepted_residual_count < config_.minimum_accepted_residuals) {
    status_ = ConvergenceStatus::kInsufficientResiduals;
  } else if (diagnostics.translation_increment_norm_m > config_.maximum_translation_correction_m ||
             diagnostics.rotation_increment_norm_rad > config_.maximum_rotation_correction_rad) {
    status_ = ConvergenceStatus::kCorrectionTooLarge;
  } else if (iterations_.size() > 1U && iterations_[iterations_.size() - 2U].residual_rms_m > 0.0 &&
             residual_rms_m > config_.divergence_rms_ratio *
                                  iterations_[iterations_.size() - 2U].residual_rms_m) {
    status_ = ConvergenceStatus::kDiverged;
  } else if (diagnostics.translation_increment_norm_m <=
                 config_.translation_increment_threshold_m &&
             diagnostics.rotation_increment_norm_rad <= config_.rotation_increment_threshold_rad) {
    status_ = ConvergenceStatus::kConverged;
  } else if (iterations_.size() >= config_.maximum_iterations) {
    status_ = ConvergenceStatus::kMaximumIterations;
  }
  return status_;
}

ConvergenceStatus ConvergenceMonitor::status() const noexcept { return status_; }

bool ConvergenceMonitor::converged() const noexcept {
  return status_ == ConvergenceStatus::kConverged;
}

const std::vector<IterationDiagnostics>& ConvergenceMonitor::iterations() const noexcept {
  return iterations_;
}

std::string ConvergenceMonitor::reason() const {
  switch (status_) {
    case ConvergenceStatus::kIterating:
      return "ITERATING";
    case ConvergenceStatus::kConverged:
      return "CONVERGED";
    case ConvergenceStatus::kMaximumIterations:
      return "MAXIMUM_ITERATIONS";
    case ConvergenceStatus::kInsufficientResiduals:
      return "INSUFFICIENT_RESIDUALS";
    case ConvergenceStatus::kNonFinite:
      return "NON_FINITE";
    case ConvergenceStatus::kCorrectionTooLarge:
      return "CORRECTION_TOO_LARGE";
    case ConvergenceStatus::kDiverged:
      return "DIVERGED";
  }
  return "UNKNOWN";
}

}  // namespace uav::nav::lio
