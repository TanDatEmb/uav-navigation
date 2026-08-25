#pragma once

#include <cmath>
#include <utility>

#include <Eigen/Core>

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

}  // namespace navigation_planning_backend
