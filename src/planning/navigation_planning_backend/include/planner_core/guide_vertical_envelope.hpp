#pragma once

#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>
#include <vector>

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

  std::vector<navigation_math::MatD4f> source_planes;
  source_planes.reserve(corridor.size());

  const auto set_vertical_bounds =
      [](geometry_utils::Polytope& polytope,
         const navigation_math::MatD4f& planes, const double lower_z,
         const double upper_z) {
        if (planes.cols() != 4 || planes.rows() <= 0 || !planes.allFinite() ||
            !std::isfinite(lower_z) || !std::isfinite(upper_z) ||
            lower_z >= upper_z || !polytope.HaveSeedLine() ||
            !polytope.seed_line.first.allFinite() ||
            !polytope.seed_line.second.allFinite()) {
          return false;
        }
        const auto seed_line = polytope.seed_line;
        const double robot_radius = polytope.robot_r;
        const bool known_free = polytope.IsKnownFree();
        const bool route_boundary_gate = polytope.IsRouteBoundaryGate();
        const auto route_boundary_point = polytope.GetRouteBoundaryPoint();
        const double route_boundary_radius =
            polytope.GetRouteBoundaryRadius();
        navigation_math::MatD4f bounded(planes.rows() + 2, 4);
        bounded.topRows(planes.rows()) = planes;
        bounded.row(planes.rows()) << 0.0, 0.0, 1.0, -upper_z;
        bounded.row(planes.rows() + 1) << 0.0, 0.0, -1.0, lower_z;
        polytope.SetPlanes(std::move(bounded));
        polytope.SetSeedLine(seed_line, robot_radius);
        polytope.SetKnownFree(known_free);
        if (route_boundary_gate) {
          polytope.SetRouteBoundaryContract(
              route_boundary_point, route_boundary_radius);
        }
        return polytope.PointIsInside(polytope.seed_line.first, 1.0e-9) &&
            polytope.PointIsInside(polytope.seed_line.second, 1.0e-9);
      };

  for (auto& polytope : corridor) {
    const auto planes = polytope.GetPlanes();
    source_planes.push_back(planes);
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
    if (!set_vertical_bounds(
            polytope, planes, local_lower_z, local_upper_z)) {
      return false;
    }
  }

  const auto overlap_depth_floor =
      [](const navigation_math::MatD4f& planes) {
        double coordinate_scale = 1.0;
        for (Eigen::Index row = 0; row < planes.rows(); ++row) {
          const double normal_norm = planes.block<1, 3>(row, 0).norm();
          if (!std::isfinite(normal_norm) || normal_norm <= 0.0) {
            return std::numeric_limits<double>::quiet_NaN();
          }
          coordinate_scale = std::max(
              coordinate_scale,
              std::abs(planes(row, 3)) / normal_norm);
        }
        return 64.0 * std::numeric_limits<double>::epsilon() *
            coordinate_scale;
      };

  // The local envelopes intentionally differ on a vertical detour. Recompute
  // the corridor-junction certificate so a future caller cannot consume stale
  // overlap metadata from the unbounded CIRI cells.
  const auto has_usable_overlap = [](const geometry_utils::Polytope& first,
                                     const geometry_utils::Polytope& second,
                                     const double depth_floor,
                                     double& depth,
                                     navigation_math::Vec3f& interior) {
    const auto overlap = first.CrossWith(second);
    depth = geometry_utils::findInteriorDist(overlap.GetPlanes(), interior);
    if (!std::isfinite(depth) || !interior.allFinite() ||
        !std::isfinite(depth_floor) || depth <= depth_floor) {
      return false;
    }
    // findInteriorDist() can report a tiny positive Chebyshev radius for a
    // lower-dimensional intersection. MINCO does not consume that scalar: it
    // needs a finite 3-D vertex hull for the overlap polytope. Validate the
    // same downstream operation here so a numerical seam cannot be advertised
    // as a usable corridor junction.
    Eigen::Matrix3Xd vertices;
    geometry_utils::enumerateVs(overlap.GetPlanes(), interior, vertices);
    return vertices.cols() > 0 && vertices.allFinite();
  };

  for (std::size_t index = 1; index < corridor.size(); ++index) {
    const auto overlap = corridor[index - 1].CrossWith(corridor[index]);
    navigation_math::Vec3f interior;
    const double depth =
        geometry_utils::findInteriorDist(overlap.GetPlanes(), interior);
    const double floor = overlap_depth_floor(overlap.GetPlanes());
    if (!std::isfinite(depth) || !std::isfinite(floor) ||
        !interior.allFinite()) {
      return false;
    }
    Eigen::Matrix3Xd vertices;
    const bool usable_overlap = depth > floor;
    if (usable_overlap) {
      geometry_utils::enumerateVs(overlap.GetPlanes(), interior, vertices);
    }
    const bool full_dimensional_overlap = usable_overlap &&
        vertices.cols() > 0 && vertices.allFinite();
    if (!full_dimensional_overlap) {
      // A local per-segment Z envelope can leave two otherwise overlapping
      // CIRI cells at a numerical seam after a retained seed line is
      // replaced. Retry exactly this junction with the still-bounded
      // guide-wide envelope. This never changes the original CIRI
      // half-spaces, and the final full-dimensional vertex check remains
      // mandatory.
      if (!set_vertical_bounds(corridor[index - 1], source_planes[index - 1],
                               envelope.lower_z_m, envelope.upper_z_m) ||
          !set_vertical_bounds(corridor[index], source_planes[index],
                               envelope.lower_z_m, envelope.upper_z_m)) {
        return false;
      }
    }
  }

  // Recompute every junction after any pair-local fallback. A positive result
  // smaller than the numerical floor is not a usable 3-D corridor and must
  // not reach vertex enumeration.
  for (std::size_t index = 1; index < corridor.size(); ++index) {
    const auto overlap = corridor[index - 1].CrossWith(corridor[index]);
    navigation_math::Vec3f interior;
    const double floor = overlap_depth_floor(overlap.GetPlanes());
    double depth = std::numeric_limits<double>::quiet_NaN();
    if (!has_usable_overlap(corridor[index - 1], corridor[index], floor,
                            depth, interior)) {
      return false;
    }
    corridor[index].overlap_depth_with_last_one = depth;
    corridor[index].interior_pt_with_last_one = interior;
  }
  return true;
}

}  // namespace navigation_planning_backend
