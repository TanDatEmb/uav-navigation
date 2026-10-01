#pragma once

#include <algorithm>
#include <cmath>
#include <limits>
#include <set>
#include <utility>

#include <Eigen/Core>

#include <data_structure/base/trajectory.h>

namespace navigation_planning_backend {

// Half-space rows are [normal_x, normal_y, normal_z, offset]. Normalization is
// a safety boundary: malformed planes must be rejected before any optimizer
// or geometric query can consume them.
template <typename PlaneMatrix>
bool normalizeCorridorPlanes(PlaneMatrix &planes) {
  if (planes.rows() == 0 || planes.cols() != 4 || !planes.allFinite()) return false;

  // Use a stable norm so valid positive rescalings of the same half-space do
  // not depend on intermediate squaring overflow or underflow.
  PlaneMatrix normalized = planes;
  for (Eigen::Index row = 0; row < normalized.rows(); ++row) {
    const auto normal = normalized.row(row).template head<3>();
    const auto normal_norm = normal.stableNorm();
    if (!std::isfinite(normal_norm) || normal_norm <= decltype(normal_norm){0}) return false;
    normalized.row(row) /= normal_norm;
  }

  if (!normalized.allFinite()) return false;
  planes = std::move(normalized);
  return true;
}

// Evaluate the geometric certificate in metres after the plane rows have been
// normalized.  This deliberately has no optimizer penalty or weight input:
// penalties guide the search, while this result authorizes the trajectory.
inline double maximumContinuousCorridorPlaneViolation(
    const geometry_utils::Piece &piece,
    const Eigen::MatrixXd &normalized_planes) {
  if (normalized_planes.rows() == 0 || normalized_planes.cols() != 4 ||
      !normalized_planes.allFinite()) {
    return std::numeric_limits<double>::infinity();
  }
  const double duration = piece.getDuration();
  const Eigen::MatrixXd &coefficients = piece.getCoeffMat();
  if (!std::isfinite(duration) || duration <= 0.0 || coefficients.rows() != 3 ||
      coefficients.cols() <= 0 || !coefficients.allFinite()) {
    return std::numeric_limits<double>::infinity();
  }

  const int degree = static_cast<int>(coefficients.cols()) - 1;
  constexpr double kRootTolerance = 1.0e-10;
  double maximum_violation = -std::numeric_limits<double>::infinity();
  for (Eigen::Index plane_index = 0;
       plane_index < normalized_planes.rows(); ++plane_index) {
    const Eigen::Vector3d normal =
        normalized_planes.block<1, 3>(plane_index, 0).transpose();
    const double offset = normalized_planes(plane_index, 3);
    const auto evaluate = [&](const double local_time) {
      const double value = normal.dot(piece.getPos(local_time)) + offset;
      return std::isfinite(value) ? value
                                  : std::numeric_limits<double>::infinity();
    };
    maximum_violation = std::max(maximum_violation, evaluate(0.0));
    maximum_violation = std::max(maximum_violation, evaluate(duration));

    if (degree <= 0) continue;
    Eigen::VectorXd derivative(degree);
    for (int coefficient_index = 0; coefficient_index < degree;
         ++coefficient_index) {
      const int power = degree - coefficient_index;
      derivative(coefficient_index) =
          static_cast<double>(power) *
          normal.dot(coefficients.col(coefficient_index));
    }
    if (!derivative.allFinite()) {
      return std::numeric_limits<double>::infinity();
    }
    // Exclude the endpoints from the root solver because its Sturm path
    // requires non-zero endpoint evaluations; endpoints were evaluated above.
    const double lower = std::nextafter(0.0, duration);
    const double upper = std::nextafter(duration, 0.0);
    if (lower >= upper) continue;
    const std::set<double> roots = math_utils::RootFinder::solvePolynomial(
        derivative, lower, upper, kRootTolerance);
    for (const double root : roots) {
      if (std::isfinite(root)) {
        maximum_violation = std::max(maximum_violation, evaluate(root));
      }
    }
  }
  return maximum_violation;
}

inline double maximumContinuousCorridorPlaneViolation(
    const geometry_utils::Trajectory &trajectory,
    const Eigen::MatrixXd &normalized_planes) {
  if (trajectory.empty()) return std::numeric_limits<double>::infinity();
  double maximum_violation = -std::numeric_limits<double>::infinity();
  for (int piece_index = 0; piece_index < trajectory.getPieceNum(); ++piece_index) {
    maximum_violation = std::max(
        maximum_violation,
        maximumContinuousCorridorPlaneViolation(trajectory[piece_index], normalized_planes));
  }
  return maximum_violation;
}

}  // namespace navigation_planning_backend
