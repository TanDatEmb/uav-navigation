#pragma once

#include <algorithm>
#include <cmath>
#include <optional>

#include <Eigen/Core>
#include <Eigen/LU>

#include <data_structure/base/piece.h>

namespace navigation_planning_backend {

// Propagated odometry currently derives acceleration and jerk from finite
// differences. Those values are useful diagnostics, but they are not measured
// command-boundary constraints. Keep measured position and velocity untouched
// while preventing an estimated high-order derivative from making the next
// certified MINCO boundary physically infeasible.
inline Eigen::Vector3d boundEstimatedDerivative(
    const Eigen::Vector3d& derivative, bool estimated, double maximum_norm) noexcept {
  if (!estimated || !derivative.allFinite() || !std::isfinite(maximum_norm) ||
      maximum_norm <= 0.0) {
    return derivative;
  }
  const double norm = derivative.norm();
  if (!std::isfinite(norm) || norm <= maximum_norm) return derivative;
  return derivative * (maximum_norm / norm);
}

// PX4's external trajectory interface consumes position, velocity and
// acceleration feed-forward, but not jerk.  Preserve the complete executable
// PVA boundary and make jerk a new-segment shaping variable instead of an
// artificial C3 handoff equality.  The generated segment still has to satisfy
// the unchanged continuous jerk and flatness certificates.
inline geometry_utils::StatePVAJ px4ExecutableBoundaryState(
    const geometry_utils::StatePVAJ& source) noexcept {
  auto output = source;
  output.col(3).setZero();
  return output;
}

// Construct a seventh-order C3 connector between two complete PVAJ states.
// This is a handoff primitive only: the caller must still run the normal
// dynamic, flatness, corridor, and immutable-world certificates on the
// resulting trajectory before publication.
inline std::optional<geometry_utils::Piece> minimumSnapStateTransitionPiece(
    const geometry_utils::StatePVAJ& initial_state,
    const geometry_utils::StatePVAJ& terminal_state,
    const double duration_s) {
  if (!initial_state.allFinite() || !terminal_state.allFinite() ||
      !std::isfinite(duration_s) || duration_s <= 1.0e-6) {
    return std::nullopt;
  }

  const double t2 = duration_s * duration_s;
  const double t3 = t2 * duration_s;
  const double t4 = t3 * duration_s;
  const double t5 = t4 * duration_s;
  const double t6 = t5 * duration_s;
  const double t7 = t6 * duration_s;
  if (!std::isfinite(t2) || !std::isfinite(t3) || !std::isfinite(t4) ||
      !std::isfinite(t5) || !std::isfinite(t6) || !std::isfinite(t7)) {
    return std::nullopt;
  }

  // Solve for ascending powers a4..a7 after the initial PVAJ terms have
  // already been fixed. The returned Piece stores the same polynomial in
  // descending powers [t^7 ... 1] as the rest of the trajectory stack.
  Eigen::Matrix4d boundary_matrix;
  boundary_matrix <<
      t4,        t5,         t6,          t7,
      4.0 * t3,  5.0 * t4,   6.0 * t5,    7.0 * t6,
      12.0 * t2, 20.0 * t3, 30.0 * t4,   42.0 * t5,
      24.0 * duration_s, 60.0 * t2, 120.0 * t3, 210.0 * t4;
  if (!boundary_matrix.allFinite()) return std::nullopt;

  Eigen::Matrix<double, 4, 3> rhs;
  rhs.row(0) = terminal_state.col(0).transpose() -
      (initial_state.col(0) + duration_s * initial_state.col(1) +
       0.5 * t2 * initial_state.col(2) +
       (t3 / 6.0) * initial_state.col(3)).transpose();
  rhs.row(1) = terminal_state.col(1).transpose() -
      (initial_state.col(1) + duration_s * initial_state.col(2) +
       0.5 * t2 * initial_state.col(3)).transpose();
  rhs.row(2) = terminal_state.col(2).transpose() -
      (initial_state.col(2) + duration_s * initial_state.col(3)).transpose();
  rhs.row(3) = terminal_state.col(3).transpose() - initial_state.col(3).transpose();
  if (!rhs.allFinite()) return std::nullopt;

  const Eigen::Matrix<double, 4, 3> high_order =
      boundary_matrix.fullPivLu().solve(rhs);
  if (!high_order.allFinite() ||
      (boundary_matrix * high_order - rhs).norm() >
          1.0e-8 * std::max(1.0, rhs.norm())) {
    return std::nullopt;
  }

  Eigen::Matrix<double, 3, 8> coefficients;
  coefficients.setZero();
  coefficients.col(7) = initial_state.col(0);
  coefficients.col(6) = initial_state.col(1);
  coefficients.col(5) = 0.5 * initial_state.col(2);
  coefficients.col(4) = initial_state.col(3) / 6.0;
  coefficients.col(3) = high_order.row(0).transpose();
  coefficients.col(2) = high_order.row(1).transpose();
  coefficients.col(1) = high_order.row(2).transpose();
  coefficients.col(0) = high_order.row(3).transpose();
  return geometry_utils::Piece(duration_s, coefficients);
}

// Build the same C3 connector while enforcing scalar rate and acceleration
// limits. Yaw handoffs use a one-axis state embedded in StatePVAJ; checking
// here prevents a flatness-valid connector from reaching the final command
// certificate with excessive yaw acceleration.
inline std::optional<geometry_utils::Piece>
minimumSnapStateTransitionPieceWithinRateAccelerationLimits(
    const geometry_utils::StatePVAJ& initial_state,
    const geometry_utils::StatePVAJ& terminal_state,
    const double duration_s,
    const double maximum_rate,
    const double maximum_acceleration) {
  if (!std::isfinite(maximum_rate) || maximum_rate <= 0.0 ||
      !std::isfinite(maximum_acceleration) || maximum_acceleration <= 0.0) {
    return std::nullopt;
  }
  auto piece = minimumSnapStateTransitionPiece(
      initial_state, terminal_state, duration_s);
  if (!piece.has_value()) return std::nullopt;
  const double rate = piece->getMaxVelRate();
  const double acceleration = piece->getMaxAccRate();
  if (!std::isfinite(rate) || !std::isfinite(acceleration) ||
      rate > maximum_rate + 1.0e-6 ||
      acceleration > maximum_acceleration + 1.0e-6) {
    return std::nullopt;
  }
  return piece;
}

}  // namespace navigation_planning_backend
