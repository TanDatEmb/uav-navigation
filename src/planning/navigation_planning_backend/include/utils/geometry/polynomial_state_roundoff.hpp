#pragma once

#include <cmath>
#include <limits>

#include <data_structure/base/piece.h>
#include <utils/header/eigen_alias.hpp>

namespace navigation_planning_backend {

inline navigation_math::StatePVAJ pieceState(
    const geometry_utils::Piece& piece, const double time_s) {
  navigation_math::StatePVAJ state;
  state.col(0) = piece.getPos(time_s);
  state.col(1) = piece.getVel(time_s);
  state.col(2) = piece.getAcc(time_s);
  state.col(3) = piece.getJer(time_s);
  return state;
}

// Bound the unavoidable forward error when Piece evaluates a power-basis
// polynomial in double precision. Long pieces can have large, cancelling
// coefficients even when their physical endpoint state is exactly constrained.
// The bound follows the absolute term sum and operation count; it is derived
// from machine precision rather than a flight-tuned epsilon.
inline navigation_math::StatePVAJ pieceStateRoundoffBound(
    const geometry_utils::Piece& piece, const double time_s) {
  navigation_math::StatePVAJ bound = navigation_math::StatePVAJ::Zero();
  const auto& coefficients = piece.getCoeffMat();
  const int degree = piece.getDegree();
  if (!coefficients.allFinite() || !std::isfinite(time_s) || degree < 0) {
    bound.setConstant(std::numeric_limits<double>::infinity());
    return bound;
  }
  constexpr double kEvaluationOperationsPerTerm = 4.0;
  const double epsilon = std::numeric_limits<double>::epsilon();
  for (int derivative = 0; derivative <= 3; ++derivative) {
    const int term_count = degree - derivative + 1;
    if (term_count <= 0) continue;
    const double operation_count =
        kEvaluationOperationsPerTerm * static_cast<double>(term_count);
    const double denominator = 1.0 - operation_count * epsilon;
    if (!(denominator > 0.0)) {
      bound.col(derivative).setConstant(
          std::numeric_limits<double>::infinity());
      continue;
    }
    const double gamma = operation_count * epsilon / denominator;
    for (int axis = 0; axis < 3; ++axis) {
      long double absolute_term_sum = 0.0L;
      for (int column = 0; column < coefficients.cols(); ++column) {
        const int power = degree - column;
        if (power < derivative) continue;
        long double multiplier = 1.0L;
        for (int order = 0; order < derivative; ++order) {
          multiplier *= static_cast<long double>(power - order);
        }
        const long double time_power = std::pow(
            static_cast<long double>(std::abs(time_s)), power - derivative);
        absolute_term_sum += std::abs(
            static_cast<long double>(coefficients(axis, column)) *
            multiplier * time_power);
      }
      const long double numerical_bound =
          static_cast<long double>(gamma) * absolute_term_sum;
      bound(axis, derivative) = numerical_bound <=
              static_cast<long double>(std::numeric_limits<double>::max())
          ? static_cast<double>(numerical_bound)
          : std::numeric_limits<double>::infinity();
    }
  }
  return bound;
}

inline bool trajectoryTerminalIsRestWithinRoundoff(
    const geometry_utils::Trajectory& trajectory) {
  if (trajectory.empty() || trajectory.getPieceNum() <= 0) return false;
  const auto& terminal_piece = trajectory[trajectory.getPieceNum() - 1];
  const double terminal_time_s = terminal_piece.getDuration();
  const auto state = pieceState(terminal_piece, terminal_time_s);
  const auto roundoff = pieceStateRoundoffBound(terminal_piece, terminal_time_s);
  if (!state.allFinite() || !roundoff.allFinite()) return false;
  return (state.block<3, 3>(0, 1).cwiseAbs().array() <=
          roundoff.block<3, 3>(0, 1).array()).all();
}

}  // namespace navigation_planning_backend
