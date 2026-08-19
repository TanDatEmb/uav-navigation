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
                                                      preferred_goal = std::nullopt) {
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

  if (inside_inset(requested) && known_free(requested)) {
    result.status = LocalGoalSelectionStatus::Direct;
    result.goal = requested;
    return result;
  }

  const auto direction = requested - start;
  const double direction_norm = direction.norm();
  if (!std::isfinite(direction_norm) || direction_norm <= 1e-9) {
    result.status = LocalGoalSelectionStatus::NoUsableSubGoal;
    return result;
  }

  const auto unit_direction = direction / direction_norm;
  const double search_distance = std::min(max_distance_m, direction_norm);
  // Keep a previously committed receding-horizon target while it is still
  // ahead of the vehicle and known free.  Without this hysteresis, each lidar
  // update can expose a different voxel on the same ray; the planner then
  // alternates left/right short splines instead of following one long route.
  if (preferred_goal.has_value() && preferred_goal->allFinite() &&
      known_free(*preferred_goal)) {
    const double preferred_projection = (*preferred_goal - start).dot(unit_direction);
    if (preferred_projection > 0.5 * resolution &&
        preferred_projection <= search_distance + resolution) {
      result.status = LocalGoalSelectionStatus::LocalSubGoal;
      result.goal = *preferred_goal;
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
  for (int step = 1; step <= longitudinal_steps; ++step) {
    const double distance = std::min(search_distance, step * resolution);
    const auto ray_point = start + distance * unit_direction;
    const auto index = world.worldToGrid(layer, ray_point);
    if (!bounds.contains(index)) break;
    if (world.cellState(layer, index) == navigation_mapping::CellState::Occupied) {
      occupied_on_ray = true;
      break;
    }
  }
  if (occupied_on_ray) {
    const int shell_cells = std::max(
        1, static_cast<int>(std::ceil(std::min(search_distance, 3.0) / resolution)));
    bool detour_found = false;
    double best_score = -std::numeric_limits<double>::infinity();
    navigation_mapping::Vec3 detour_goal = navigation_mapping::Vec3::Zero();
    const auto start_index = world.worldToGrid(layer, start);
    for (int dx = -shell_cells; dx <= shell_cells; ++dx) {
      for (int dy = -shell_cells; dy <= shell_cells; ++dy) {
        for (int dz = -vertical_cells; dz <= vertical_cells; ++dz) {
          const navigation_mapping::GridIndex3 index{start_index.x + dx,
                                                      start_index.y + dy,
                                                      start_index.z + dz};
          if (!bounds.contains(index)) continue;
          const auto candidate = world.gridToWorld(layer, index);
          if (!known_free(candidate)) continue;
          const auto delta = candidate - start;
          const double distance = delta.norm();
          if (!std::isfinite(distance) || distance <= 0.5 * resolution ||
              distance > search_distance + resolution) {
            continue;
          }
          const double projection = delta.dot(unit_direction);
          if (!std::isfinite(projection) || projection < -2.0 * resolution) continue;
          const double lateral = std::sqrt(std::max(
              0.0, delta.squaredNorm() - projection * projection));
          if (lateral <= 0.5 * resolution) continue;
          const double goal_distance = (candidate - requested).norm();
          const double score = 1.5 * std::min(lateral, 2.5) -
                               0.35 * goal_distance + 0.1 * projection;
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
      return result;
    }
  }

  bool found = false;
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
          if (!known_free(candidate)) continue;
          const double projection = (candidate - start).dot(unit_direction);
          const double lateral_error = (candidate - (start + projection * unit_direction)).squaredNorm();
          if (projection <= 0.5 * resolution || projection > search_distance + resolution) {
            continue;
          }
          if (!found || projection > best_projection + 1e-9 ||
              (std::abs(projection - best_projection) <= 1e-9 &&
               lateral_error < best_lateral_error)) {
            found = true;
            best_projection = projection;
            best_lateral_error = lateral_error;
            best_goal = candidate;
          }
        }
      }
    }
    if (found && best_projection >= distance - resolution) break;
  }
  if (!found) {
    result.status = LocalGoalSelectionStatus::NoUsableSubGoal;
    return result;
  }
  result.status = LocalGoalSelectionStatus::LocalSubGoal;
  result.goal = best_goal;
  return result;
}

}  // namespace navigation_runtime
