#pragma once

#include <algorithm>
#include <cmath>
#include <limits>

#include <navigation_world_model/world_model_view.hpp>

namespace navigation_world_model {

// CIRI certifies a seed line against the observed occupied points, using the
// same robot-radius tube as the planner.  The grid query remains necessary for
// unknown/out-of-map semantics, but it is not sufficient at voxel corners or
// when a coarse inflated cell under-approximates the continuous tube.
//
// Keep the 1 cm tolerance identical to CIRI's seed-clearance tolerance.  A
// point at or below radius-tolerance is therefore rejected by both contracts.
[[nodiscard]] inline bool observedOccupiedTubeIsClear(
    const WorldModelView& world,
    const Point3& start,
    const Point3& end,
    const double radius_m) noexcept {
  constexpr double kClearanceToleranceM = 0.01;
  if (!start.allFinite() || !end.allFinite() ||
      !std::isfinite(radius_m) || radius_m < 0.0) {
    return false;
  }

  const double query_radius = radius_m + kClearanceToleranceM;
  if (!std::isfinite(query_radius)) return false;
  try {
    const AxisAlignedBox query_box{
        start.cwiseMin(end).array() - query_radius,
        start.cwiseMax(end).array() + query_radius};
    const auto occupied = world.observedOccupiedPoints(query_box);
    const Point3 segment = end - start;
    const double segment_squared_norm = segment.squaredNorm();
    for (const Point3& point : occupied) {
      if (!point.allFinite()) return false;
      double projection = 0.0;
      if (segment_squared_norm > std::numeric_limits<double>::epsilon()) {
        projection = (point - start).dot(segment) / segment_squared_norm;
        projection = std::clamp(projection, 0.0, 1.0);
      }
      const Point3 closest = start + projection * segment;
      const double distance = (point - closest).norm();
      if (!std::isfinite(distance) ||
          distance < radius_m - kClearanceToleranceM) {
        return false;
      }
    }
  } catch (...) {
    // A failed point-cloud query must never turn into an optimistic planning
    // result.  The caller will remain fail-closed and retain its safe suffix.
    return false;
  }
  return true;
}

}  // namespace navigation_world_model
