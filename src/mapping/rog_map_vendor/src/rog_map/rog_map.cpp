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

// uav-navigation local modification: pcl::io::loadPCDFile (used only by the
// optional, P1-disabled load_pcd_en path below) was previously pulled in
// transitively via pcl_conversions.h, which this vendor package no longer
// includes (see UPSTREAM.md). Include it directly instead.
#include <pcl/io/pcd_io.h>

using namespace rog_map;
using namespace super_utils;
void ROGMap::init() {
  initProbMap();

  map_info_log_file_.open(DEBUG_FILE_DIR("rm_info_log.csv"), std::ios::out | std::ios::trunc);

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
  Vec3f ray_pt;
  raycaster.setInput(start_pt, end_pt);
  while (raycaster.step(ray_pt)) {
    if (!insideLocalMap(ray_pt)) return false;
    if (!use_unk_as_occ) {
      // allow both unk and free
      if (use_inf_map) {
        if (isOccupiedInflate(ray_pt)) {
          return false;
        }
      } else {
        if (isOccupied(ray_pt)) {
          return false;
        }
      }
    } else {
      // only allow known free
      if (use_inf_map) {
        if (!cfg_.unk_inflation_en) {
          if (!isKnownFree(ray_pt) || isOccupiedInflate(ray_pt)) return false;
        } else if (isUnknownInflate(ray_pt) || isOccupiedInflate(ray_pt)) {
          return false;
        }
      } else {
        if (!isKnownFree(ray_pt)) {
          return false;
        }
      }
    }
  }
  return true;
}

bool ROGMap::isLineKnownFree(const Vec3f& start_pt, const Vec3f& end_pt,
                             const bool& use_inf_map) const {
  if (!use_inf_map) {
    return isLineFree(start_pt, end_pt, false, true);
  }

  // ROG-Map can intentionally disable unknown inflation for SUPER's
  // endpoint-only visibility model.  A known-free query still has to combine
  // the two independent facts we care about: the probabilistic cell was
  // observed free, and the robot-radius inflated layer contains no obstacle.
  // Falling back to the base layer alone drops the vehicle envelope; calling
  // isUnknownInflate() when that layer is disabled throws by contract.
  if (start_pt.array().isNaN().any() || end_pt.array().isNaN().any()) {
    return false;
  }
  raycaster::RayCaster raycaster;
  raycaster.setResolution(std::min(cfg_.resolution, cfg_.inflation_resolution));
  Vec3f ray_pt;
  raycaster.setInput(start_pt, end_pt);
  while (raycaster.step(ray_pt)) {
    if (!isKnownFree(ray_pt) || isOccupiedInflate(ray_pt)) {
      return false;
    }
  }
  return true;
}

bool ROGMap::isLineFree(const Vec3f& start_pt, const Vec3f& end_pt, const double& max_dis,
                        const vec_Vec3i& neighbor_list) const {
  raycaster::RayCaster raycaster;
  raycaster.setResolution(cfg_.resolution);
  Vec3f ray_pt;
  raycaster.setInput(start_pt, end_pt);
  while (raycaster.step(ray_pt)) {
    if (max_dis > 0 && (ray_pt - start_pt).norm() > max_dis) {
      return false;
    }

    if (neighbor_list.empty()) {
      if (isOccupied(ray_pt)) {
        return false;
      }
    } else {
      Vec3i ray_pt_id_g;
      posToGlobalIndex(ray_pt, ray_pt_id_g);
      for (const auto& nei : neighbor_list) {
        Vec3i shift_tmp = ray_pt_id_g + nei;
        if (isOccupied(shift_tmp)) {
          return false;
        }
      }
    }
  }
  return true;
}

bool ROGMap::isLineFree(const Vec3f& start_pt, const Vec3f& end_pt, Vec3f& free_local_goal,
                        const double& max_dis, const vec_Vec3i& neighbor_list) const {
  raycaster::RayCaster raycaster;
  raycaster.setResolution(cfg_.resolution);
  Vec3f ray_pt;
  raycaster.setInput(start_pt, end_pt);
  free_local_goal = start_pt;
  while (raycaster.step(ray_pt)) {
    free_local_goal = ray_pt;
    if (max_dis > 0 && (ray_pt - start_pt).norm() > max_dis) {
      return false;
    }

    if (neighbor_list.empty()) {
      if (isOccupied(ray_pt)) {
        return false;
      }
    } else {
      Vec3i ray_pt_id_g;
      posToGlobalIndex(ray_pt, ray_pt_id_g);
      for (const auto& nei : neighbor_list) {
        Vec3i shift_tmp = ray_pt_id_g + nei;
        if (isOccupied(shift_tmp)) {
          return false;
        }
      }
    }
  }
  free_local_goal = end_pt;
  return true;
}

void ROGMap::updateMap(const PointCloud& cloud, const Pose& pose) {
  // Product runtime keeps timing in MappingDiagnostics; upstream's
  // per-update console timer is research instrumentation.
  TimeConsuming ssss("updateMap", false);
  last_diagnostics_ = ProbMap::RaycastDiagnostics{};
  last_diagnostics_.endpoint_count = cloud.size();
  last_diagnostics_.allocated_voxel_count = static_cast<std::uint64_t>(sc_.map_vox_num);
  if (cfg_.ros_callback_en) {
    std::cout << YELLOW << "ROS callback is enabled, can not insert map from updateMap API."
              << RESET << std::endl;
    return;
  }

  if (cloud.empty()) {
    static int local_cnt = 0;
    if (local_cnt++ > 100) {
      cout << YELLOW << "No cloud input, please check the input topic." << RESET << endl;
      local_cnt = 0;
    }
    return;
  }

  updateRobotState(pose);
  updateProbMap(cloud, pose);
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
