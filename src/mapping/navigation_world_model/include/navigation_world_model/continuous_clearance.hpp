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

  const long double query_radius = static_cast<long double>(radius_m) +
                                   static_cast<long double>(kClearanceToleranceM);
  if (!std::isfinite(query_radius) ||
      query_radius > static_cast<long double>(std::numeric_limits<double>::max())) {
    return false;
  }
  try {
    AxisAlignedBox query_box;
    for (int axis = 0; axis < 3; ++axis) {
      const long double lower = std::min(static_cast<long double>(start(axis)),
                                         static_cast<long double>(end(axis))) - query_radius;
      const long double upper = std::max(static_cast<long double>(start(axis)),
                                         static_cast<long double>(end(axis))) + query_radius;
      if (!std::isfinite(lower) || !std::isfinite(upper) ||
          lower < static_cast<long double>(std::numeric_limits<double>::lowest()) ||
          upper > static_cast<long double>(std::numeric_limits<double>::max())) {
        return false;
      }
      query_box.minimum(axis) = static_cast<double>(lower);
      query_box.maximum(axis) = static_cast<double>(upper);
    }
    if (!query_box.valid()) return false;
    const auto occupied = world.observedOccupiedPoints(query_box);
    long double segment_squared_norm = 0.0L;
    for (int axis = 0; axis < 3; ++axis) {
      const long double delta = static_cast<long double>(end(axis)) -
                                static_cast<long double>(start(axis));
      if (!std::isfinite(delta)) return false;
      segment_squared_norm += delta * delta;
    }
    if (!std::isfinite(segment_squared_norm)) return false;
    for (const Point3& point : occupied) {
      if (!point.allFinite()) return false;
      long double projection = 0.0L;
      if (segment_squared_norm > static_cast<long double>(std::numeric_limits<double>::epsilon())) {
        long double numerator = 0.0L;
        for (int axis = 0; axis < 3; ++axis) {
          const long double delta = static_cast<long double>(end(axis)) -
                                    static_cast<long double>(start(axis));
          numerator += (static_cast<long double>(point(axis)) -
                        static_cast<long double>(start(axis))) * delta;
        }
        if (!std::isfinite(numerator)) return false;
        projection = std::clamp(numerator / segment_squared_norm, 0.0L, 1.0L);
      }
      long double distance_squared = 0.0L;
      for (int axis = 0; axis < 3; ++axis) {
        const long double delta = static_cast<long double>(point(axis)) -
                                  (static_cast<long double>(start(axis)) + projection *
                                   (static_cast<long double>(end(axis)) -
                                    static_cast<long double>(start(axis))));
        distance_squared += delta * delta;
      }
      const long double distance = std::sqrt(distance_squared);
      if (!std::isfinite(distance) ||
          distance < static_cast<long double>(radius_m - kClearanceToleranceM)) {
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
