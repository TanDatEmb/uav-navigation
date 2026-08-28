#pragma once

#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>

#include <data_structure/base/polytope.h>
#include <utils/header/type_utils.hpp>

namespace navigation_planning_backend {

struct GuideVerticalEnvelope {
  bool valid{false};
  double lower_z_m{std::numeric_limits<double>::quiet_NaN()};
  double upper_z_m{std::numeric_limits<double>::quiet_NaN()};
  double slack_m{std::numeric_limits<double>::quiet_NaN()};
};

// The collision-checked guide owns intentional climb/descent. Give the
// continuous optimizer one inflated-map voxel of vertical smoothing freedom,
// rather than the full height of a broad obstacle-free CIRI cell.
inline GuideVerticalEnvelope deriveGuideVerticalEnvelope(
    const navigation_math::vec_Vec3f& guide,
    const double inflated_resolution_m) noexcept {
  GuideVerticalEnvelope envelope;
  if (guide.empty() || !std::isfinite(inflated_resolution_m) ||
      inflated_resolution_m <= 0.0) {
    return envelope;
  }
  double minimum_z = std::numeric_limits<double>::infinity();
  double maximum_z = -std::numeric_limits<double>::infinity();
  for (const auto& point : guide) {
    if (!point.allFinite()) return envelope;
    minimum_z = std::min(minimum_z, point.z());
    maximum_z = std::max(maximum_z, point.z());
  }
  envelope.lower_z_m = minimum_z - inflated_resolution_m;
  envelope.upper_z_m = maximum_z + inflated_resolution_m;
  envelope.slack_m = inflated_resolution_m;
  envelope.valid = std::isfinite(envelope.lower_z_m) &&
      std::isfinite(envelope.upper_z_m) &&
      envelope.lower_z_m < envelope.upper_z_m;
  return envelope;
}

inline bool applyGuideVerticalEnvelope(
    geometry_utils::PolytopeVec& corridor,
    const GuideVerticalEnvelope& envelope) {
  if (corridor.empty() || !envelope.valid) return false;
  for (auto& polytope : corridor) {
    const auto planes = polytope.GetPlanes();
    if (planes.cols() != 4 || planes.rows() <= 0 || !planes.allFinite() ||
        !polytope.HaveSeedLine() ||
        !polytope.seed_line.first.allFinite() ||
        !polytope.seed_line.second.allFinite()) {
      return false;
    }
    const double local_lower_z = std::min(
        polytope.seed_line.first.z(), polytope.seed_line.second.z()) -
        envelope.slack_m;
    const double local_upper_z = std::max(
        polytope.seed_line.first.z(), polytope.seed_line.second.z()) +
        envelope.slack_m;
    navigation_math::MatD4f bounded(planes.rows() + 2, 4);
    bounded.topRows(planes.rows()) = planes;
    bounded.row(planes.rows()) << 0.0, 0.0, 1.0, -local_upper_z;
    bounded.row(planes.rows() + 1) << 0.0, 0.0, -1.0, local_lower_z;
    polytope.SetPlanes(std::move(bounded));
    if (!polytope.PointIsInside(polytope.seed_line.first, 1.0e-9) ||
        !polytope.PointIsInside(polytope.seed_line.second, 1.0e-9)) {
      return false;
    }
  }
  // The local envelopes intentionally differ on a vertical detour. Recompute
  // the corridor-junction certificate so a future caller cannot consume stale
  // overlap metadata from the unbounded CIRI cells.
  for (std::size_t index = 1; index < corridor.size(); ++index) {
    const auto overlap = corridor[index - 1].CrossWith(corridor[index]);
    navigation_math::Vec3f interior;
    const double depth =
        geometry_utils::findInteriorDist(overlap.GetPlanes(), interior);
    if (!std::isfinite(depth) || depth <= 0.0 || !interior.allFinite()) {
      return false;
    }
    corridor[index].overlap_depth_with_last_one = depth;
    corridor[index].interior_pt_with_last_one = interior;
  }
  return true;
}

}  // namespace navigation_planning_backend
