#include "uavnav/lio/degeneracy.hpp"

#include <algorithm>
#include <cmath>

#include <Eigen/Eigenvalues>

namespace uavnav::lio {

namespace {

constexpr DegeneracyReport kUnusable{0.0, 0.0, true, 0};

// Smallest eigenvalue of the symmetrised block, or nullopt when the solver fails or it is not finite.
// The caller has already checked that `block` is finite.
std::optional<double> min_eigenvalue(const Eigen::Matrix3d& block) {
  // 0.5*A + 0.5*Aᵀ rather than 0.5*(A + Aᵀ): each term is halved first, so two huge finite entries cannot
  // overflow in the sum and the symmetrised block of a finite block is always finite.
  const Eigen::Matrix3d sym = 0.5 * block + 0.5 * block.transpose();
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver(sym, Eigen::EigenvaluesOnly);
  if (solver.info() != Eigen::Success) return std::nullopt;
  const double lambda = solver.eigenvalues().minCoeff();  // ascending order; minCoeff is order-independent
  if (!std::isfinite(lambda)) return std::nullopt;
  return lambda;
}

bool usable_threshold(double v) { return std::isfinite(v) && v > 0.0; }

}  // namespace

DegeneracyReport evaluate_degeneracy(const std::optional<Eigen::Matrix<double, 6, 6>>& information,
                                     const DegeneracyConfig& config) noexcept {
  if (!information || !information->allFinite()) return kUnusable;

  const auto lambda_t = min_eigenvalue(information->topLeftCorner<3, 3>());
  const auto lambda_r = min_eigenvalue(information->bottomRightCorner<3, 3>());
  if (!lambda_t || !lambda_r) return kUnusable;

  if (!usable_threshold(config.translation_min_info) || !usable_threshold(config.rotation_min_info)) {
    return DegeneracyReport{*lambda_t, *lambda_r, true, 0};
  }

  const bool degenerate = *lambda_t < config.translation_min_info || *lambda_r < config.rotation_min_info;
  // A negative ratio (negative eigenvalue: numerical noise for a PSD information matrix) lands on the lower
  // clamp, i.e. quality 0. The ratio may overflow to +inf for huge eigenvalues; clamp in double before the
  // cast so the cast is always defined.
  const double ratio = std::min(*lambda_t / config.translation_min_info, *lambda_r / config.rotation_min_info);
  const double scaled = std::clamp(std::round(50.0 * ratio), 0.0, 100.0);
  // quality >= 50 <=> !degenerate: a ratio just below 1 rounds to 50, so cap it at 49 when degenerate.
  const double capped = degenerate ? std::min(scaled, 49.0) : scaled;
  return DegeneracyReport{*lambda_t, *lambda_r, degenerate, static_cast<std::uint8_t>(capped)};
}

}  // namespace uavnav::lio
