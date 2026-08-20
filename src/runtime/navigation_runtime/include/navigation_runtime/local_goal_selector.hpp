#pragma once

#include <algorithm>
#include <cmath>
#include <limits>
#include <optional>

#include "navigation_mapping/world_model.hpp"

namespace navigation_runtime {

enum class LocalGoalSelectionStatus {
  Direct,
  LocalSubGoal,
  InvalidState,
  StartOutsideBounds,
  NoUsableSubGoal,
};

struct LocalGoalSelection {
  LocalGoalSelectionStatus status{LocalGoalSelectionStatus::InvalidState};
  navigation_mapping::Vec3 goal{navigation_mapping::Vec3::Zero()};
  // Direction of the locally selected corridor at the horizon endpoint.  The
  // legacy selector only returned a point; rolling-horizon planning also needs
  // the tangent so the trajectory endpoint does not inherit a mission-ray or
  // next-waypoint direction that points behind the vehicle.
  navigation_mapping::Vec3 tangent{navigation_mapping::Vec3::Zero()};
  double forward_projection_m{0.0};

  [[nodiscard]] bool success() const noexcept {
    return status == LocalGoalSelectionStatus::Direct ||
           status == LocalGoalSelectionStatus::LocalSubGoal;
  }

  [[nodiscard]] bool usesSubGoal() const noexcept {
    return status == LocalGoalSelectionStatus::LocalSubGoal;
  }
};

namespace detail {

inline bool inside(const navigation_mapping::Vec3& point,
                   const navigation_mapping::Vec3& lower,
                   const navigation_mapping::Vec3& upper) noexcept {
  return (point.array() >= lower.array()).all() && (point.array() <= upper.array()).all();
}

inline navigation_mapping::Vec3 componentwiseMin(const navigation_mapping::Vec3& lhs,
                                                  const navigation_mapping::Vec3& rhs) noexcept {
  return navigation_mapping::Vec3{std::min(lhs.x(), rhs.x()), std::min(lhs.y(), rhs.y()),
                                  std::min(lhs.z(), rhs.z())};
}

inline navigation_mapping::Vec3 componentwiseMax(const navigation_mapping::Vec3& lhs,
                                                  const navigation_mapping::Vec3& rhs) noexcept {
  return navigation_mapping::Vec3{std::max(lhs.x(), rhs.x()), std::max(lhs.y(), rhs.y()),
                                  std::max(lhs.z(), rhs.z())};
}

}  // namespace detail

// Selects a bounded, known-free planning target for a rolling local map. The
// requested mission waypoint remains the terminal target; this helper supplies
// a temporary receding-horizon target while the waypoint is outside the map or
// has not yet been observed as known free.
template <typename Model>
[[nodiscard]] LocalGoalSelection selectLocalGoal(const Model& world,
                                                  const navigation_mapping::Vec3& start,
                                                  const navigation_mapping::Vec3& requested,
                                                  double boundary_margin_m,
                                                  double max_distance_m = 5.0,
                                                  const std::optional<navigation_mapping::Vec3>&
                                                      preferred_goal = std::nullopt,
                                                  const std::optional<navigation_mapping::Vec3>&
                                                      forward_direction = std::nullopt,
                                                  bool reuse_preferred = true) {
  LocalGoalSelection result;
  if (!start.allFinite() || !requested.allFinite() || !std::isfinite(boundary_margin_m) ||
      boundary_margin_m < 0.0 || !std::isfinite(max_distance_m) || max_distance_m <= 0.0) {
    result.status = LocalGoalSelectionStatus::InvalidState;
    return result;
  }
  if constexpr (requires { world.isReady(); }) {
    if (!world.isReady()) {
      result.status = LocalGoalSelectionStatus::InvalidState;
      return result;
    }
  }

  constexpr auto layer = navigation_mapping::WorldLayer::Inflated;
  const auto bounds = world.bounds(layer);
  const auto lower_grid = world.gridToWorld(layer, bounds.min);
  const auto upper_grid = world.gridToWorld(layer, bounds.max);
  const auto lower = detail::componentwiseMin(lower_grid, upper_grid);
  const auto upper = detail::componentwiseMax(lower_grid, upper_grid);
  if (!lower.allFinite() || !upper.allFinite() ||
      (upper.array() <= lower.array()).any() || !detail::inside(start, lower, upper)) {
    result.status = LocalGoalSelectionStatus::StartOutsideBounds;
    return result;
  }

  const auto resolution = world.resolution(layer);
  if (!std::isfinite(resolution) || resolution <= 0.0) {
    result.status = LocalGoalSelectionStatus::InvalidState;
    return result;
  }

  const auto inset_lower = lower.array() + boundary_margin_m;
  const auto inset_upper = upper.array() - boundary_margin_m;
  if ((inset_upper <= inset_lower).any()) {
    result.status = LocalGoalSelectionStatus::NoUsableSubGoal;
    return result;
  }

  const auto inside_inset = [&](const navigation_mapping::Vec3& point) {
    return detail::inside(point, inset_lower.matrix(), inset_upper.matrix());
  };
  const auto known_free = [&](const navigation_mapping::Vec3& point) {
    if (!point.allFinite() || !inside_inset(point)) return false;
    const auto index = world.worldToGrid(layer, point);
    if (!bounds.contains(index)) return false;
    return world.cellState(layer, index) == navigation_mapping::CellState::KnownFree;
  };
  const auto horizon_known_free = [&](const navigation_mapping::Vec3& point) {
    if (!known_free(point)) return false;
    // A single KnownFree cell at the edge of a newly revealed scan is not a
    // stable horizon endpoint.  Its first spline sample can land in the
    // neighbouring Unknown cell, so the planner may reject exactly the point
    // that the selector just returned. Require a small horizontal support
    // stencil for local anchors; the mission waypoint itself keeps the less
    // conservative direct check above.
    const auto center = world.worldToGrid(layer, point);
    for (int dx = -1; dx <= 1; ++dx) {
      for (int dy = -1; dy <= 1; ++dy) {
        const navigation_mapping::GridIndex3 neighbour{center.x + dx, center.y + dy,
                                                        center.z};
        if (!bounds.contains(neighbour) ||
            world.cellState(layer, neighbour) != navigation_mapping::CellState::KnownFree) {
          return false;
        }
      }
    }
    return true;
  };

  const auto mission_direction = requested - start;
  const double mission_direction_norm = mission_direction.norm();
  if (!std::isfinite(mission_direction_norm) || mission_direction_norm <= 1e-9) {
    result.status = LocalGoalSelectionStatus::NoUsableSubGoal;
    return result;
  }

  const auto direction = [&]() -> navigation_mapping::Vec3 {
    if (forward_direction.has_value() && forward_direction->allFinite() &&
        forward_direction->norm() > 1e-6) {
      return *forward_direction;
    }
    return mission_direction;
  }();
  const double direction_norm = direction.norm();
  if (!std::isfinite(direction_norm) || direction_norm <= 1e-9) {
    result.status = LocalGoalSelectionStatus::NoUsableSubGoal;
    return result;
  }

  // Once the mission waypoint is inside the current, stably observed map
  // window it is the only endpoint that may be treated as Direct, regardless
  // of its distance from the vehicle.  The old distance gate made a waypoint
  // that had just become visible look like another local goal and forced the
  // runtime to walk through a chain of artificial endpoints.  Require the
  // same local support stencil used for horizon anchors so a one-voxel scan
  // island at the map edge cannot trigger a nominal trajectory that is
  // invalidated on the next map update.
  const auto unit_direction = direction / direction_norm;
  const double requested_forward_projection = (requested - start).dot(unit_direction);
  if (inside_inset(requested) && horizon_known_free(requested) &&
      requested_forward_projection >= -0.5 * resolution) {
    result.status = LocalGoalSelectionStatus::Direct;
    result.goal = requested;
    result.tangent = direction / direction_norm;
    result.forward_projection_m = (requested - start).dot(result.tangent);
    return result;
  }

  const auto mission_unit_direction = mission_direction / mission_direction_norm;
  const double search_distance = std::min(max_distance_m, direction_norm);
  // A preferred endpoint is now a continuity cost only for rolling planning.
  // Returning it verbatim made every 5 Hz replan look like a subgoal
  // completion loop.  The old API keeps the compatibility behaviour for
  // existing callers/tests; selectPlanningHorizon() disables that reuse.
  if (reuse_preferred && preferred_goal.has_value() && preferred_goal->allFinite() &&
      horizon_known_free(*preferred_goal)) {
    const double preferred_projection = (*preferred_goal - start).dot(unit_direction);
    if (preferred_projection > 0.5 * resolution &&
        preferred_projection <= search_distance + resolution) {
      result.status = LocalGoalSelectionStatus::LocalSubGoal;
      result.goal = *preferred_goal;
      const auto delta = result.goal - start;
      result.tangent = delta.norm() > 1e-6 ? delta.normalized() : unit_direction;
      result.forward_projection_m = delta.dot(unit_direction);
      return result;
    }
  }
  const int longitudinal_steps = std::max(1, static_cast<int>(std::ceil(search_distance / resolution)));
  // Search a thin horizontal stencil around the mission ray. This keeps the
  // selector bounded (and cheap at 5 Hz) while allowing A* to choose a side
  // of a newly observed obstacle. Vertical motion remains on the requested
  // ray; a different known-free mission altitude is handled directly once it
  // enters the map window.
  // Keep the receding-horizon target inside the observed corridor even when
  // the direct mission ray enters an unknown/occluded pocket.  Two cells were
  // too narrow for the occlusion detour: the selector repeatedly returned
  // NoUsableSubGoal and the mission watchdog interpreted that transient map
  // condition as a planner failure.  The inflated layer still performs the
  // collision and unknown checks for every candidate.
  const int lateral_cells = 6;
  const int vertical_cells = 2;

  // If an occupied cell is already visible on the mission ray, a target that
  // merely advances along that ray is not a detour: it drives the vehicle
  // toward the obstacle until the current voxel becomes OCCUPIED. Select a
  // short lateral side-step from a bounded shell instead. This lets the next
  // planning cycle build a genuinely long corridor around a wall, including
  // the common case where the first safe motion is slightly backward.
  bool occupied_on_ray = false;
  double first_occupied_distance = std::numeric_limits<double>::infinity();
  navigation_mapping::GridIndex3 first_occupied_index{};
  for (int step = 1; step <= longitudinal_steps; ++step) {
    const double distance = std::min(search_distance, step * resolution);
    const auto ray_point = start + distance * unit_direction;
    const auto index = world.worldToGrid(layer, ray_point);
    if (!bounds.contains(index)) break;
    if (world.cellState(layer, index) == navigation_mapping::CellState::Occupied) {
      occupied_on_ray = true;
      first_occupied_distance = distance;
      first_occupied_index = index;
      break;
    }
  }
  if (occupied_on_ray) {
    // Search around the first observed obstacle, not only around the current
    // vehicle voxel.  A start-centred shell can return a point just in front
    // of a pillar; the vehicle then reaches that point and waits for another
    // goal.  Anchoring the stencil at the obstacle lets the selector choose a
    // known-free point beside or beyond the obstacle whenever the current map
    // already exposes that side corridor.
    const int longitudinal_shell_cells = std::max(
        1, static_cast<int>(std::ceil(std::min(search_distance,
                                               first_occupied_distance + 3.0) /
                                      resolution)));
    // Do not jump to the far end of a long local window when the direct ray
    // is blocked. The side corridor beyond the first obstacle may still be
    // Unknown even though one endpoint voxel is KnownFree; A* would then
    // reject the entire spline and the runtime would brake. Keep the rolling
    // horizon continuous, but bound a detour endpoint to the first obstacle
    // plus a small post-obstacle buffer. This is not a completion gate: the
    // next map tick immediately extends/replaces the horizon.
    const double detour_max_distance = std::min(
        search_distance, first_occupied_distance + std::max(2.0, 2.0 * resolution));
    const int lateral_shell_cells = std::max(
        lateral_cells, static_cast<int>(std::ceil(3.0 / resolution)));
    bool detour_found = false;
    double best_score = -std::numeric_limits<double>::infinity();
    navigation_mapping::Vec3 detour_goal = navigation_mapping::Vec3::Zero();
    for (int dx = -longitudinal_shell_cells; dx <= longitudinal_shell_cells; ++dx) {
      for (int dy = -lateral_shell_cells; dy <= lateral_shell_cells; ++dy) {
        for (int dz = -vertical_cells; dz <= vertical_cells; ++dz) {
          const navigation_mapping::GridIndex3 index{first_occupied_index.x + dx,
                                                      first_occupied_index.y + dy,
                                                      first_occupied_index.z + dz};
          if (!bounds.contains(index)) continue;
          const auto candidate = world.gridToWorld(layer, index);
          if (!horizon_known_free(candidate)) continue;
          const auto delta = candidate - start;
          const double distance = delta.norm();
          if (!std::isfinite(distance) || distance <= 0.5 * resolution ||
              distance > detour_max_distance + resolution) {
            continue;
          }
          const double projection = delta.dot(unit_direction);
          if (!std::isfinite(projection) || projection < -2.0 * resolution) continue;
          const double lateral = std::sqrt(std::max(
              0.0, delta.squaredNorm() - projection * projection));
          if (lateral <= 0.5 * resolution) continue;
          const double goal_distance = (candidate - requested).norm();
          const double continuity_distance =
              preferred_goal.has_value() && preferred_goal->allFinite()
                  ? (candidate - *preferred_goal).norm()
                  : 0.0;
          // Preserve the mission altitude whenever the map offers a safe
          // lateral detour.  A rolling voxel stencil can otherwise pick the
          // first known-free cell below the requested flight level (typically
          // one or two z cells above the ground).  That creates a slow,
          // cumulative descent on long missions even though the mission
          // waypoint is level.  Keep the lateral escape term dominant, but
          // make vertical displacement an explicit cost and tie breaker.
          const double vertical_error = std::abs(candidate.z() - requested.z());
          const double mission_progress = (candidate - start).dot(mission_unit_direction);
          const double score = 1.5 * std::min(lateral, 2.5) -
                               0.35 * goal_distance + 0.35 * projection +
                               0.10 * mission_progress -
                               1.25 * vertical_error - 0.75 * continuity_distance;
          if (!detour_found || score > best_score + 1e-9) {
            detour_found = true;
            best_score = score;
            detour_goal = candidate;
          }
        }
      }
    }
    if (detour_found) {
      result.status = LocalGoalSelectionStatus::LocalSubGoal;
      result.goal = detour_goal;
      const auto delta = result.goal - start;
      result.tangent = delta.norm() > 1e-6 ? delta.normalized() : unit_direction;
      result.forward_projection_m = delta.dot(unit_direction);
      return result;
    }
  }

  bool found = false;
  // Prefer the requested flight level over a marginally farther voxel.  The
  // rolling map is quantised in z; choosing purely by projection makes the
  // target alternate between e.g. 2.7 and 3.1 m and produces a real vertical
  // oscillation.  A few metres of projected progress are worth one metre of
  // altitude error, while the selector remains bounded by max_distance_m.
  constexpr double kAltitudeContinuityWeight = 4.0;
  double best_score = -std::numeric_limits<double>::infinity();
  double best_projection = -std::numeric_limits<double>::infinity();
  double best_lateral_error = std::numeric_limits<double>::infinity();
  navigation_mapping::Vec3 best_goal = navigation_mapping::Vec3::Zero();
  for (int step = longitudinal_steps; step >= 1; --step) {
    const double distance = std::min(search_distance, step * resolution);
    const auto ray_point = start + distance * unit_direction;
    const auto ray_index = world.worldToGrid(layer, ray_point);
    for (int dx = -lateral_cells; dx <= lateral_cells; ++dx) {
      for (int dy = -lateral_cells; dy <= lateral_cells; ++dy) {
        for (int dz = -vertical_cells; dz <= vertical_cells; ++dz) {
          const navigation_mapping::GridIndex3 index{ray_index.x + dx, ray_index.y + dy,
                                                      ray_index.z + dz};
          if (!bounds.contains(index)) continue;
          const auto candidate = world.gridToWorld(layer, index);
          if (!horizon_known_free(candidate)) continue;
          const double projection = (candidate - start).dot(unit_direction);
          const double lateral_error = (candidate - (start + projection * unit_direction)).squaredNorm();
          if (projection <= 0.5 * resolution || projection > search_distance + resolution) {
            continue;
          }
          const double vertical_error = std::abs(candidate.z() - requested.z());
          const double mission_progress = (candidate - start).dot(mission_unit_direction);
          const double preferred_distance =
              preferred_goal.has_value() && preferred_goal->allFinite()
                  ? (candidate - *preferred_goal).norm()
                  : 0.0;
          const double score = projection + 0.10 * mission_progress -
                               0.35 * lateral_error -
                               kAltitudeContinuityWeight * vertical_error -
                               (reuse_preferred ? 0.0 : 0.05 * preferred_distance);
          if (!found || score > best_score + 1e-9 ||
              (std::abs(score - best_score) <= 1e-9 &&
               (projection > best_projection + 1e-9 ||
                (std::abs(projection - best_projection) <= 1e-9 &&
                 lateral_error < best_lateral_error)))) {
            found = true;
            best_score = score;
            best_projection = projection;
            best_lateral_error = lateral_error;
            best_goal = candidate;
          }
        }
      }
    }
  }
  if (!found) {
    result.status = LocalGoalSelectionStatus::NoUsableSubGoal;
    return result;
  }
  result.status = LocalGoalSelectionStatus::LocalSubGoal;
  result.goal = best_goal;
  const auto delta = result.goal - start;
  result.tangent = delta.norm() > 1e-6 ? delta.normalized() : unit_direction;
  result.forward_projection_m = delta.dot(unit_direction);
  return result;
}

// Compatibility-shaped horizon API. The returned point is an A* corridor
// anchor only; it is never a mission waypoint and must be represented by a
// non-terminal B-spline suffix in the runtime. Keeping this API separate from
// the old selector name makes that distinction explicit at the planning
// boundary while existing selector unit tests remain source compatible.
template <typename Model>
[[nodiscard]] LocalGoalSelection selectPlanningHorizon(
    const Model& world, const navigation_mapping::Vec3& start,
    const navigation_mapping::Vec3& requested, double boundary_margin_m,
    double max_distance_m = 5.0,
    const std::optional<navigation_mapping::Vec3>& preferred_endpoint = std::nullopt,
    const std::optional<navigation_mapping::Vec3>& forward_direction = std::nullopt) {
  return selectLocalGoal(world, start, requested, boundary_margin_m, max_distance_m,
                         preferred_endpoint, forward_direction, false);
}

}  // namespace navigation_runtime
