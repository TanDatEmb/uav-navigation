/**
 * This file is part of ROG-Map
 *
 * Copyright 2024 Yunfan REN, MaRS Lab, University of Hong Kong, <mars.hku.hk>
 * Developed by Yunfan REN <renyf at connect dot hku dot hk>
 * for more information see <https://github.com/hku-mars/ROG-Map>.
 * If you use this code, please cite the respective publications as
 * listed on the above website.
 *
 * ROG-Map is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * ROG-Map is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with ROG-Map. If not, see <http://www.gnu.org/licenses/>.
 */

#include "rog_map/rog_map.h"

#include <algorithm>
#include <cstdint>
#include <limits>

// uav-navigation local modification: pcl::io::loadPCDFile (used only by the
// optional disabled load_pcd_en path below) was previously pulled in
// transitively via pcl_conversions.h, which this vendor package no longer
// includes (see UPSTREAM.md). Include it directly instead.
#include <pcl/io/pcd_io.h>

using namespace rog_map;
using namespace navigation_math;

PlanningGridExport ROGMap::exportPlanningGrid() const {
  PlanningGridExport output;
  output.base_layout.resolution_m = sc_.resolution;
  output.base_layout.global_min_index = local_map_origin_i_ - sc_.half_map_size_i;
  output.base_layout.dimensions = sc_.map_size_i;
  output.base_layout.local_center_m = local_map_origin_d_;
  output.base_layout.local_size_m = sc_.map_size_i.cast<double>() * sc_.resolution;
  output.base_state.resize(static_cast<std::size_t>(sc_.map_vox_num));

  const auto logical_coordinate = [this](int global, int axis) {
    int local = global % sc_.map_size_i(axis);
    if (local > sc_.half_map_size_i(axis)) local -= sc_.map_size_i(axis);
    if (local < -sc_.half_map_size_i(axis)) local += sc_.map_size_i(axis);
    return local + sc_.half_map_size_i(axis);
  };
  std::vector<int> hash_x(static_cast<std::size_t>(sc_.map_size_i.x()));
  std::vector<int> hash_y(static_cast<std::size_t>(sc_.map_size_i.y()));
  for (int i = 0; i < sc_.map_size_i.x(); ++i) {
    hash_x[static_cast<std::size_t>(i)] =
        logical_coordinate(output.base_layout.global_min_index.x() + i, 0) *
        sc_.map_size_i.y() * sc_.map_size_i.z();
  }
  for (int i = 0; i < sc_.map_size_i.y(); ++i) {
    hash_y[static_cast<std::size_t>(i)] =
        logical_coordinate(output.base_layout.global_min_index.y() + i, 1) * sc_.map_size_i.z();
  }
  std::size_t logical_offset = 0;
  const Vec3i global_max = output.base_layout.global_min_index + sc_.map_size_i;
  const int logical_z_start = logical_coordinate(output.base_layout.global_min_index.z(), 2);
  const std::size_t first_z_count = static_cast<std::size_t>(
      std::min(sc_.map_size_i.z(), sc_.map_size_i.z() - logical_z_start));
  const std::size_t second_z_count =
      static_cast<std::size_t>(sc_.map_size_i.z()) - first_z_count;
  for (int x = output.base_layout.global_min_index.x(), xi = 0; x < global_max.x(); ++x, ++xi) {
    for (int y = output.base_layout.global_min_index.y(), yi = 0; y < global_max.y(); ++y, ++yi) {
      const int hash_xy = hash_x[static_cast<std::size_t>(xi)] +
                          hash_y[static_cast<std::size_t>(yi)];
      const auto emit_z_segment = [&](const int source_z_start,
                                      const std::size_t count,
                                      const int global_z_start) {
        if (!cfg_.virtual_ground_ceiling_en) {
          const auto source_begin = occupancy_buffer_.begin() + hash_xy + source_z_start;
          auto destination_begin = output.base_state.begin() +
              static_cast<std::ptrdiff_t>(logical_offset);
          std::transform(source_begin, source_begin +
                             static_cast<std::ptrdiff_t>(count), destination_begin,
                         [this](const float probability) {
                           const auto state = isKnownFree(probability)
                               ? GridType::KNOWN_FREE
                               : (isOccupied(probability) ? GridType::OCCUPIED
                                                           : GridType::UNKNOWN);
                           return static_cast<std::uint8_t>(state);
                         });
          logical_offset += count;
          return;
        }
        for (std::size_t index = 0; index < count; ++index) {
          // Export the public planning semantics, including the index-domain
          // virtual-plane policy, rather than only the underlying probability byte.
          GridType state = GridType::OCCUPIED;
          const int global_z = global_z_start + static_cast<int>(index);
          if (!cfg_.virtual_ground_ceiling_en ||
              (global_z > sc_.virtual_ground_height_id_g &&
               global_z < sc_.virtual_ceil_height_id_g - sc_.safe_margin_i)) {
            const float probability = occupancy_buffer_[
                hash_xy + source_z_start + static_cast<int>(index)];
            state = isKnownFree(probability)
                        ? GridType::KNOWN_FREE
                        : (isOccupied(probability) ? GridType::OCCUPIED : GridType::UNKNOWN);
          }
          output.base_state[logical_offset++] = static_cast<std::uint8_t>(state);
        }
      };
      emit_z_segment(logical_z_start, first_z_count, output.base_layout.global_min_index.z());
      if (second_z_count != 0U) {
        emit_z_segment(0, second_z_count,
                       output.base_layout.global_min_index.z() +
                           static_cast<int>(first_z_count));
      }
    }
  }

  output.inflated = inf_map_->exportPlanningGrid();
  output.nearest_offsets = planning_nearest_offsets_;
  output.unknown_inflation_enabled = cfg_.unk_inflation_en;
  output.virtual_ground_ceiling_enabled = cfg_.virtual_ground_ceiling_en;
  if (cfg_.virtual_ground_ceiling_en) {
    output.virtual_ground_m = cfg_.virtual_ground_height;
    output.virtual_ceiling_m = cfg_.virtual_ceil_height;
    output.inflated_virtual_ground_m =
        cfg_.virtual_ground_height + cfg_.inflation_resolution * (1 + cfg_.inflation_step);
    output.inflated_virtual_ceiling_m =
        cfg_.virtual_ceil_height - cfg_.inflation_resolution * (1 + cfg_.inflation_step);
  } else {
    // Preserve a finite geometry contract for corridor clamping while making
    // it relative to the immutable snapshot's sliding-map window, not to an
    // arbitrary absolute Z origin.
    output.virtual_ground_m =
        static_cast<double>(output.base_layout.global_min_index.z()) *
        output.base_layout.resolution_m;
    output.virtual_ceiling_m =
        static_cast<double>(output.base_layout.global_min_index.z() +
                            output.base_layout.dimensions.z()) *
        output.base_layout.resolution_m;
    output.inflated_virtual_ground_m = output.virtual_ground_m;
    output.inflated_virtual_ceiling_m = output.virtual_ceiling_m;
  }
  output.occupied_inflation_radius_m =
      cfg_.inflation_resolution * static_cast<double>(cfg_.inflation_step);
  return output;
}

PlanningGridPatchExport ROGMap::exportPlanningGridRegion(
    const Vec3f& region_min, const Vec3f& region_max) const {
  PlanningGridPatchExport output;
  output.base_layout.resolution_m = sc_.resolution;
  output.base_layout.local_center_m = local_map_origin_d_;
  output.base_layout.local_size_m = sc_.map_size_i.cast<double>() * sc_.resolution;
  if (!region_min.allFinite() || !region_max.allFinite() ||
      (region_max.array() < region_min.array()).any()) {
    return output;
  }

  Vec3i requested_min;
  Vec3i requested_max;
  posToGlobalIndex(region_min, requested_min);
  posToGlobalIndex(region_max, requested_max);
  const Vec3i map_min = local_map_origin_i_ - sc_.half_map_size_i;
  const Vec3i map_max = map_min + sc_.map_size_i - Vec3i::Ones();
  const Vec3i patch_min = requested_min.cwiseMax(map_min);
  const Vec3i patch_max = requested_max.cwiseMin(map_max);
  if ((patch_max.array() < patch_min.array()).any()) return output;

  output.base_layout.global_min_index = patch_min;
  output.base_layout.dimensions = patch_max - patch_min + Vec3i::Ones();
  std::size_t count = 1U;
  for (int axis = 0; axis < 3; ++axis) {
    const auto dimension = static_cast<std::size_t>(output.base_layout.dimensions[axis]);
    if (count > std::numeric_limits<std::size_t>::max() / dimension) return {};
    count *= dimension;
  }
  output.base_state.resize(count);

  // Match the full export's logical ordering, but compute circular-buffer
  // coordinates once per X/Y row instead of once per voxel. Typical LiDAR
  // changed-region unions span millions of Z-contiguous cells; calling
  // getGridType() for every cell repeated three modulo/index conversions and
  // made a patch materially slower than a full export.
  const auto logical_coordinate = [this](int global, int axis) {
    int local = global % sc_.map_size_i(axis);
    if (local > sc_.half_map_size_i(axis)) local -= sc_.map_size_i(axis);
    if (local < -sc_.half_map_size_i(axis)) local += sc_.map_size_i(axis);
    return local + sc_.half_map_size_i(axis);
  };
  std::vector<int> hash_x(static_cast<std::size_t>(output.base_layout.dimensions.x()));
  std::vector<int> hash_y(static_cast<std::size_t>(output.base_layout.dimensions.y()));
  for (int i = 0; i < output.base_layout.dimensions.x(); ++i) {
    hash_x[static_cast<std::size_t>(i)] =
        logical_coordinate(patch_min.x() + i, 0) *
        sc_.map_size_i.y() * sc_.map_size_i.z();
  }
  for (int i = 0; i < output.base_layout.dimensions.y(); ++i) {
    hash_y[static_cast<std::size_t>(i)] =
        logical_coordinate(patch_min.y() + i, 1) * sc_.map_size_i.z();
  }

  std::size_t offset = 0U;
  for (int x = patch_min.x(), xi = 0; x <= patch_max.x(); ++x, ++xi) {
    for (int y = patch_min.y(), yi = 0; y <= patch_max.y(); ++y, ++yi) {
      const int hash_xy = hash_x[static_cast<std::size_t>(xi)] +
                          hash_y[static_cast<std::size_t>(yi)];
      for (int z = patch_min.z(); z <= patch_max.z(); ++z) {
        GridType state = GridType::OCCUPIED;
        if (!cfg_.virtual_ground_ceiling_en ||
            (z > sc_.virtual_ground_height_id_g &&
             z < sc_.virtual_ceil_height_id_g - sc_.safe_margin_i)) {
          const float probability = occupancy_buffer_[
              hash_xy + logical_coordinate(z, 2)];
          state = isKnownFree(probability)
              ? GridType::KNOWN_FREE
              : (isOccupied(probability) ? GridType::OCCUPIED : GridType::UNKNOWN);
        }
        output.base_state[offset++] = static_cast<std::uint8_t>(state);
      }
    }
  }
  output.inflated = inf_map_->exportPlanningGridRegion(region_min, region_max);
  return output;
}

PlanningGridRegionSizeEstimate ROGMap::estimatePlanningGridRegionSize(
    const Vec3f& region_min, const Vec3f& region_max) const {
  PlanningGridRegionSizeEstimate estimate;
  if (!region_min.allFinite() || !region_max.allFinite() ||
      (region_max.array() < region_min.array()).any()) return estimate;
  Vec3i requested_min;
  Vec3i requested_max;
  posToGlobalIndex(region_min, requested_min);
  posToGlobalIndex(region_max, requested_max);
  const Vec3i map_min = local_map_origin_i_ - sc_.half_map_size_i;
  const Vec3i map_max = map_min + sc_.map_size_i - Vec3i::Ones();
  const Vec3i patch_min = requested_min.cwiseMax(map_min);
  const Vec3i patch_max = requested_max.cwiseMin(map_max);
  if ((patch_max.array() < patch_min.array()).any()) return estimate;
  const Vec3i dimensions = patch_max - patch_min + Vec3i::Ones();
  std::size_t base_count = 1U;
  for (int axis = 0; axis < 3; ++axis) {
    const auto dimension = static_cast<std::size_t>(dimensions[axis]);
    if (dimension == 0U ||
        base_count > std::numeric_limits<std::size_t>::max() / dimension) return estimate;
    base_count *= dimension;
  }
  const std::size_t inflated_count =
      inf_map_->estimatePlanningGridRegionCellCount(region_min, region_max);
  if (inflated_count == 0U) return estimate;
  const std::size_t inflated_layers = cfg_.unk_inflation_en ? 2U : 1U;
  if (inflated_count >
      (std::numeric_limits<std::size_t>::max() - base_count) / inflated_layers) {
    return estimate;
  }
  estimate.base_cell_count = base_count;
  estimate.inflated_cell_count = inflated_count;
  estimate.owned_byte_count = base_count + inflated_layers * inflated_count;
  estimate.valid = true;
  return estimate;
}

void ROGMap::init() {
  initProbMap();
  planning_nearest_offsets_ =
      std::make_shared<const std::vector<Vec3i>>(cfg_.spherical_neighbor);

  map_info_log_file_.open(NAVIGATION_MAP_DEBUG_FILE_DIR("rm_info_log.csv"), std::ios::out | std::ios::trunc);

  robot_state_.p = cfg_.fix_map_origin;

  if (cfg_.map_sliding_en) {
    mapSliding(Vec3f(0, 0, 0));
    inf_map_->mapSliding(Vec3f(0, 0, 0));
  } else {
    /// if disable map sliding, fix map origin to (0,0,0)
    /// update the local map bound as
    local_map_bound_min_d_ = -cfg_.half_map_size_d + cfg_.fix_map_origin;
    local_map_bound_max_d_ = cfg_.half_map_size_d + cfg_.fix_map_origin;
    mapSliding(cfg_.fix_map_origin);
    inf_map_->mapSliding(cfg_.fix_map_origin);
  }

  writeMapInfoToLog(map_info_log_file_);
  map_info_log_file_.close();
  if (cfg_.load_pcd_en) {
    string pcd_path = cfg_.pcd_name;
    PointCloud::Ptr pcd_map(new PointCloud);
    if (pcl::io::loadPCDFile(pcd_path, *pcd_map) == -1) {
      cout << YELLOW << "Load pcd file at: [" << cfg_.pcd_name << "] failed!" << RESET << endl;
      exit(-1);
    }
    Pose cur_pose;
    cur_pose.first = Vec3f(0, 0, 0);
    updateOccPointCloud(*pcd_map);
    if (cfg_.esdf_en) {
      esdf_map_->updateESDF3D(robot_state_.p);
    }
    cout << BLUE << " -- [ROGMap]Load pcd file success with " << pcd_map->size() << " pts." << RESET
         << endl;
    map_empty_ = false;
  }
}

bool ROGMap::findNearestCellThat(const bool& is, const GridType& target_type,
                                 const Vec3f& start_pos, Vec3f& nearest_pt,
                                 const double& max_dis) const {
  Vec3i start_id;
  posToGlobalIndex(start_pos, start_id);
  nearest_pt.setConstant(NAN);

  for (const auto& nei_id : cfg_.spherical_neighbor) {
    const Vec3i q_id = start_id + nei_id;
    Vec3f q_pos;
    globalIndexToPos(q_id, q_pos);
    if ((q_pos - start_pos).norm() > max_dis) {
      return false;
    }

    if ((getGridType(q_pos) == target_type) == is) {
      nearest_pt = q_pos;
      return true;
    }
  }

  return false;
}

bool ROGMap::findNearestInfCellThat(const bool& is, const GridType& target_type,
                                    const Vec3f& start_pos, Vec3f& nearest_pt,
                                    const double& max_dis) const {
  Vec3i start_id;
  posToGlobalIndex(start_pos, start_id);
  nearest_pt.setConstant(NAN);

  for (const auto& nei_id : cfg_.spherical_neighbor) {
    const Vec3i q_id = start_id + nei_id;
    Vec3f q_pos;
    globalIndexToPos(q_id, q_pos);
    if ((q_pos - start_pos).norm() > max_dis) {
      return false;
    }

    if ((getInfGridType(q_pos) == target_type) == is) {
      nearest_pt = q_pos;
      return true;
    }
  }
  fmt::print(fg(fmt::color::yellow),
             " -- [ROGMap] findNearestInfCellThat failed to find all {} neighbors at start_pos: "
             "{}, target_type: {}, is: {}\n",
             cfg_.spherical_neighbor.size(), start_pos.transpose(), target_type, is);
  return false;
}

bool ROGMap::isLineFree(const rog_map::Vec3f& start_pt, const rog_map::Vec3f& end_pt,
                        const bool& use_inf_map, const bool& use_unk_as_occ) const {
  if (start_pt.array().isNaN().any() || end_pt.array().isNaN().any()) {
    cout << YELLOW << " -- [ROGMap] Call isLineFree with NaN in start or end pt, return false."
         << RESET << endl;
    return false;
  }
  if (!insideLocalMap(start_pt) || !insideLocalMap(end_pt)) return false;
  raycaster::RayCaster raycaster;
  if (use_inf_map) {
    raycaster.setResolution(cfg_.inflation_resolution);
  } else {
    raycaster.setResolution(cfg_.resolution);
  }
  const auto point_is_traversable = [this, use_inf_map, use_unk_as_occ](const Vec3f& point) {
    if (!insideLocalMap(point)) return false;
    if (!use_unk_as_occ) {
      // allow both unk and free
      if (use_inf_map) {
        if (isOccupiedInflate(point)) {
          return false;
        }
      } else {
        if (isOccupied(point)) {
          return false;
        }
      }
    } else {
      // only allow known free
      if (use_inf_map) {
        if (!cfg_.unk_inflation_en) {
          if (!isKnownFree(point) || isOccupiedInflate(point)) return false;
        } else if (isUnknownInflate(point) || isOccupiedInflate(point)) {
          return false;
        }
      } else {
        if (!isKnownFree(point)) {
          return false;
        }
      }
    }
    return true;
  };
  if (!point_is_traversable(start_pt)) return false;
  raycaster.setInput(start_pt, end_pt);
  Vec3f ray_pt;
  while (raycaster.step(ray_pt)) {
    if (!point_is_traversable(ray_pt)) return false;
  }
  // RayCaster enumerates cells crossed before the endpoint cell and returns
  // false when its cursor reaches the endpoint. Always classify both metric
  // endpoints so same-voxel segments and endpoint occupancy share one rule.
  if (!point_is_traversable(end_pt)) return false;
  return true;
}

bool ROGMap::isLineKnownFree(const Vec3f& start_pt, const Vec3f& end_pt,
                             const bool& use_inf_map) const {
  if (!use_inf_map) {
    return isLineFree(start_pt, end_pt, false, true);
  }

  // ROG-Map can intentionally disable unknown inflation for the planner's
  // endpoint-only visibility model. A known-free query still has to combine
  // the two independent facts we care about: the probabilistic cell was
  // observed free, and the robot-radius inflated layer contains no obstacle.
  // Falling back to the base layer alone drops the vehicle envelope; calling
  // isUnknownInflate() when that layer is disabled throws by contract.
  const auto point_index_representable = [this](const Vec3f& point) {
    if (!std::isfinite(cfg_.resolution) || cfg_.resolution <= 0.0) {
      return false;
    }
    for (int axis = 0; axis < 3; ++axis) {
      const long double index =
          static_cast<long double>(point(axis)) /
          static_cast<long double>(cfg_.resolution);
      if (!std::isfinite(index) ||
          index < static_cast<long double>(std::numeric_limits<int>::min()) ||
          index > static_cast<long double>(std::numeric_limits<int>::max())) {
        return false;
      }
    }
    return true;
  };
  const auto point_is_known_free = [this, &point_index_representable](
                                       const Vec3f& point) {
    if (!point.allFinite() || !point_index_representable(point) ||
        !insideLocalMap(point) ||
        !isKnownFree(point) || isOccupiedInflate(point)) {
      return false;
    }
    return !cfg_.unk_inflation_en || !isUnknownInflate(point);
  };
  if (!point_is_known_free(start_pt) || !point_is_known_free(end_pt)) {
    return false;
  }
  raycaster::RayCaster raycaster;
  raycaster.setResolution(std::min(cfg_.resolution, cfg_.inflation_resolution));
  Vec3f ray_pt;
  if (raycaster.setInput(start_pt, end_pt)) {
    while (raycaster.step(ray_pt)) {
      if (!point_is_known_free(ray_pt)) {
        return false;
      }
    }
  }
  return true;
}

bool ROGMap::isLineFree(const Vec3f& start_pt, const Vec3f& end_pt, const double& max_dis,
                        const vec_Vec3i& neighbor_list) const {
  if (!start_pt.allFinite() || !end_pt.allFinite() ||
      !std::isfinite(max_dis) || max_dis < 0.0 ||
      !insideLocalMap(start_pt) || !insideLocalMap(end_pt)) {
    return false;
  }
  const auto point_is_clear = [this, &neighbor_list](const Vec3f& point) {
    if (!point.allFinite() || !insideLocalMap(point)) return false;
    if (neighbor_list.empty()) return !isOccupied(point);
    Vec3i point_id;
    posToGlobalIndex(point, point_id);
    for (const auto& neighbor : neighbor_list) {
      Vec3i shifted = point_id;
      for (int axis = 0; axis < 3; ++axis) {
        const std::int64_t value = static_cast<std::int64_t>(point_id(axis)) +
                                   static_cast<std::int64_t>(neighbor(axis));
        if (value < std::numeric_limits<int>::min() ||
            value > std::numeric_limits<int>::max()) return false;
        shifted(axis) = static_cast<int>(value);
      }
      if (isOccupied(shifted)) return false;
    }
    return true;
  };
  if (!point_is_clear(start_pt)) return false;
  raycaster::RayCaster raycaster;
  raycaster.setResolution(cfg_.resolution);
  Vec3f ray_pt;
  if (!raycaster.setInput(start_pt, end_pt)) return point_is_clear(end_pt);
  while (raycaster.step(ray_pt)) {
    if (max_dis > 0 && (ray_pt - start_pt).norm() > max_dis) {
      return false;
    }

    if (!point_is_clear(ray_pt)) return false;
  }
  return point_is_clear(end_pt);
}

bool ROGMap::isLineFree(const Vec3f& start_pt, const Vec3f& end_pt, Vec3f& free_local_goal,
                        const double& max_dis, const vec_Vec3i& neighbor_list) const {
  free_local_goal = start_pt;
  if (!start_pt.allFinite() || !end_pt.allFinite() ||
      !std::isfinite(max_dis) || max_dis < 0.0 ||
      !insideLocalMap(start_pt) || !insideLocalMap(end_pt)) {
    return false;
  }
  const auto point_is_clear = [this, &neighbor_list](const Vec3f& point) {
    if (!point.allFinite() || !insideLocalMap(point)) return false;
    if (neighbor_list.empty()) return !isOccupied(point);
    Vec3i point_id;
    posToGlobalIndex(point, point_id);
    for (const auto& neighbor : neighbor_list) {
      Vec3i shifted = point_id;
      for (int axis = 0; axis < 3; ++axis) {
        const std::int64_t value = static_cast<std::int64_t>(point_id(axis)) +
                                   static_cast<std::int64_t>(neighbor(axis));
        if (value < std::numeric_limits<int>::min() ||
            value > std::numeric_limits<int>::max()) return false;
        shifted(axis) = static_cast<int>(value);
      }
      if (isOccupied(shifted)) return false;
    }
    return true;
  };
  if (!point_is_clear(start_pt)) return false;
  raycaster::RayCaster raycaster;
  raycaster.setResolution(cfg_.resolution);
  Vec3f ray_pt;
  if (!raycaster.setInput(start_pt, end_pt)) {
    if (!point_is_clear(end_pt)) return false;
    free_local_goal = end_pt;
    return true;
  }
  while (raycaster.step(ray_pt)) {
    free_local_goal = ray_pt;
    if (max_dis > 0 && (ray_pt - start_pt).norm() > max_dis) {
      return false;
    }

    if (!point_is_clear(ray_pt)) return false;
  }
  if (!point_is_clear(end_pt)) return false;
  free_local_goal = end_pt;
  return true;
}

MapUpdateOutcome ROGMap::updateMap(const PointCloud& cloud, const Pose& pose) {
  static const PointCloud no_return_endpoints;
  return updateMap(cloud, no_return_endpoints, pose);
}

MapUpdateOutcome ROGMap::updateMap(
    const PointCloud& cloud, const PointCloud& free_space_endpoints,
    const Pose& pose) {
  // Product runtime keeps timing in MappingDiagnostics; upstream's
  // per-update console timer is research instrumentation.
  TimeConsuming ssss("updateMap", false);
  last_diagnostics_ = ProbMap::RaycastDiagnostics{};
  last_diagnostics_.endpoint_count = cloud.size();
  last_diagnostics_.free_space_endpoint_count = free_space_endpoints.size();
  last_diagnostics_.allocated_voxel_count = static_cast<std::uint64_t>(sc_.map_vox_num);
  if (cfg_.ros_callback_en) {
    std::cout << YELLOW << "ROS callback is enabled, can not insert map from updateMap API."
              << RESET << std::endl;
    last_diagnostics_.update_outcome = MapUpdateOutcome::CALLBACK_OWNED;
    return last_diagnostics_.update_outcome;
  }

  if (cloud.empty() && free_space_endpoints.empty()) {
    static int local_cnt = 0;
    if (local_cnt++ > 100) {
      cout << YELLOW << "No cloud input, please check the input topic." << RESET << endl;
      local_cnt = 0;
    }
    last_diagnostics_.update_outcome = MapUpdateOutcome::EMPTY_CLOUD;
    return last_diagnostics_.update_outcome;
  }

  updateRobotState(pose);
  return updateProbMap(cloud, free_space_endpoints, pose);
}

RobotState ROGMap::getRobotState() const { return robot_state_; }

void ROGMap::updateRobotState(const Pose& pose) {
  robot_state_.p = pose.first;
  robot_state_.q = pose.second;
  robot_state_.rcv_time = getSystemWalltimeNow();
  robot_state_.rcv = true;
  robot_state_.yaw = get_yaw_from_quaternion<double>(pose.second);
  updateLocalBox(pose.first);
}
