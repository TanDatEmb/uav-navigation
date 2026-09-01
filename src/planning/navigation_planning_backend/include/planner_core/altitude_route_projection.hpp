#pragma once

#include <cmath>

#include <utils/header/type_utils.hpp>

namespace navigation_planning_backend {

// Build a constant-altitude route only when every newly introduced edge is
// certified by the caller.  The first edge is an explicit vertical recovery
// from the measured start; subsequent edges retain the A* horizontal route.
// This is a geometric preference, never a collision-validation bypass.
template <typename SegmentTraversable>
bool buildConstantAltitudeRoute(
    const navigation_math::vec_Vec3f& source_path,
    const double target_altitude_m,
    const double altitude_tolerance_m,
    SegmentTraversable&& segment_traversable,
    navigation_math::vec_Vec3f& projected_path) {
  projected_path.clear();
  if (source_path.size() < 2U || !std::isfinite(target_altitude_m) ||
      !std::isfinite(altitude_tolerance_m) || altitude_tolerance_m < 0.0) {
    return false;
  }
  for (const auto& point : source_path) {
    if (!point.allFinite()) return false;
  }

  projected_path.reserve(source_path.size() + 1U);
  projected_path.push_back(source_path.front());
  const double altitude_delta = target_altitude_m - source_path.front().z();
  if (std::abs(altitude_delta) > altitude_tolerance_m) {
    navigation_math::Vec3f climb_point = source_path.front();
    climb_point.z() = static_cast<float>(target_altitude_m);
    if (!segment_traversable(source_path.front(), climb_point)) {
      projected_path.clear();
      return false;
    }
    projected_path.push_back(climb_point);
  }

  for (std::size_t index = 1U; index < source_path.size(); ++index) {
    navigation_math::Vec3f projected = source_path[index];
    projected.z() = static_cast<float>(target_altitude_m);
    if (!segment_traversable(projected_path.back(), projected)) {
      projected_path.clear();
      return false;
    }
    projected_path.push_back(projected);
  }
  return projected_path.size() >= 2U;
}

}  // namespace navigation_planning_backend
