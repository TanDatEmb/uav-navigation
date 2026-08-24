/**
* This file is part of SUPER
*
* Copyright 2025 Yunfan REN, MaRS Lab, University of Hong Kong, <mars.hku.hk>
* Developed by Yunfan REN <renyf at connect dot hku dot hk>
* for more information see <https://github.com/hku-mars/SUPER>.
* If you use this code, please cite the respective publications as
* listed on the above website.
*
* SUPER is free software: you can redistribute it and/or modify
* it under the terms of the GNU Lesser General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* SUPER is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU Lesser General Public License
* along with SUPER. If not, see <http://www.gnu.org/licenses/>.
*/

#include <path_search/astar.h>
#include <fmt/color.h>
#include <rog_map/rog_map_core/common_lib.hpp>

using namespace color_text;
using namespace super_utils;

namespace path_search {
    using namespace rog_map;
    using navigation_world_model::CellState;
    using navigation_world_model::GridLayer;
    using navigation_world_model::UnknownPolicy;
    constexpr CellState OCCUPIED_CELL = CellState::kOccupied;
    constexpr CellState KNOWN_FREE_CELL = CellState::kKnownFree;
    constexpr CellState UNKNOWN_CELL = CellState::kUnknown;
    constexpr CellState UNDEFINED_CELL = CellState::kUndefined;


    Astar::Astar(const std::string &cfg_path,
                 const ros_interface::RosInterface::Ptr &ros_ptr,
                 navigation_world_model::WorldModelViewPtr rm)
        : map_ptr_(std::move(rm)), ros_ptr_(ros_ptr) {
        cfg_ = PathSearchConfig(cfg_path);
        cout << GREEN << " -- [RM] Init Astar-map." << RESET << endl;
        int map_buffer_size = cfg_.map_voxel_num(0) * cfg_.map_voxel_num(1) * cfg_.map_voxel_num(2);
        grid_node_buffer_.resize(map_buffer_size);
        for (auto &i: grid_node_buffer_) {
            i = new GridNode;
            i->rounds = 0;
        }
        cout << BLUE << "\tmap index size: " << cfg_.map_size_i.transpose() << RESET << endl;
        cout << BLUE << "\tmap vox_num: " << cfg_.map_voxel_num.transpose() << RESET << endl;
    }

    RET_CODE
    Astar::setup(const Vec3f &start_pt, const Vec3f &goal_pt, const int &flag, const double &searching_horizon) {
        md_.start_pt = start_pt;
        md_.goal_pt = goal_pt;
        md_.mission_rcv_WT = ros_ptr_->getSimTime();
        md_.searching_horizon = searching_horizon;
        md_.use_inf_map = flag & ON_INF_MAP;
        md_.use_prob_map = flag & ON_PROB_MAP;
        md_.unknown_as_occ = flag & UNKNOWN_AS_OCCUPIED;
        md_.unknown_as_free = flag & UNKNOWN_AS_FREE;
        md_.use_inf_neighbor = flag & USE_INF_NEIGHBOR;
        if (flag & DONT_USE_INF_NEIGHBOR) {
            md_.use_inf_neighbor = false;
        }
        if ((md_.use_inf_map && md_.use_prob_map) ||
            (!md_.use_inf_map && !md_.use_prob_map)) {
            cout << YELLOW << " -- [A*] " << RET_CODE_STR[INIT_ERROR]
                 << ": cannot use both inf map and prob map." << RESET << endl;
            return INIT_ERROR;
        }
        if (md_.unknown_as_occ && md_.unknown_as_free) {
            cout << YELLOW << " -- [A*] " << RET_CODE_STR[INIT_ERROR]
                 << ": cannot use both unknown_as_occupied and unknown_as_free." << RESET << endl;
            return INIT_ERROR;
        }
        if (md_.use_prob_map) {
            md_.resolution = map_ptr_->geometry().evidence_resolution_m;
        } else {
            md_.resolution = map_ptr_->geometry().inflated_resolution_m;
        }
        if (searching_horizon > 0) {
            md_.local_map_center_d = start_pt;
        } else {
            md_.local_map_center_d = (start_pt + goal_pt) / 2;
        }

        posToGlobalIndex(md_.local_map_center_d, md_.local_map_center_id_g);
        md_.local_map_min_d = md_.local_map_center_d - md_.resolution * cfg_.map_size_i.cast<double>();
        md_.local_map_max_d = md_.local_map_center_d + md_.resolution * cfg_.map_size_i.cast<double>();;
        if (cfg_.visual_process||cfg_.debug_visualization_en) {
            ros_ptr_->vizAstarBoundingBox(md_.local_map_min_d, md_.local_map_max_d);
        }

        return SUCCESS;
    }

    double Astar::getHeu(GridNodePtr node1, GridNodePtr node2, int type) const {
        switch (type) {
            case DIAG: {
                double dx = std::abs(node1->id_g(0) - node2->id_g(0));
                double dy = std::abs(node1->id_g(1) - node2->id_g(1));
                double dz = std::abs(node1->id_g(2) - node2->id_g(2));

                double h = 0.0;
                int diag = std::min(std::min(dx, dy), dz);
                dx -= diag;
                dy -= diag;
                dz -= diag;

                if (dx == 0) {
                    h = 1.0 * sqrt(3.0) * diag + sqrt(2.0) * std::min(dy, dz) + 1.0 * std::abs(dy - dz);
                }
                if (dy == 0) {
                    h = 1.0 * sqrt(3.0) * diag + sqrt(2.0) * std::min(dx, dz) + 1.0 * std::abs(dx - dz);
                }
                if (dz == 0) {
                    h = 1.0 * sqrt(3.0) * diag + sqrt(2.0) * std::min(dx, dy) + 1.0 * std::abs(dx - dy);
                }
                return cfg_.heuristic_weight * h;
            }
            case MANH: {
                double dx = std::abs(node1->id_g(0) - node2->id_g(0));
                double dy = std::abs(node1->id_g(1) - node2->id_g(1));
                double dz = std::abs(node1->id_g(2) - node2->id_g(2));

                return cfg_.heuristic_weight * (dx + dy + dz);
            }
            case EUCL: {
                return cfg_.heuristic_weight * (node2->id_g - node1->id_g).norm();
            }
            default: {
                fmt::print(fg(fmt::color::indian_red), " -- [A*] Wrong hue type.\n");
                return 0;
            }
        }
    }

    int Astar::getLocalIndexHash(const Vec3i &id_in) const {
        rog_map::Vec3i id = id_in - md_.local_map_center_id_g + cfg_.map_size_i;
        return id(0) * cfg_.map_voxel_num(1) * cfg_.map_voxel_num(2) +
               id(1) * cfg_.map_voxel_num(2) +
               id(2);
    }

    void Astar::posToGlobalIndex(const rog_map::Vec3f &pos, rog_map::Vec3i &id_g) const {
        if (md_.use_inf_map) {
            id_g = map_ptr_->positionToIndex(pos, GridLayer::kInflated);
        } else if (md_.use_prob_map) {
            id_g = map_ptr_->positionToIndex(pos, GridLayer::kEvidence);
        } else {
            throw std::runtime_error(" -- [A*] Map type not defined.");
        }
    }

    void Astar::globalIndexToPos(const rog_map::Vec3i &id_g, rog_map::Vec3f &pos) const {
        if (md_.use_inf_map) {
            pos = map_ptr_->indexToPosition(id_g, GridLayer::kInflated);
        } else if (md_.use_prob_map) {
            pos = map_ptr_->indexToPosition(id_g, GridLayer::kEvidence);
        } else {
            throw std::runtime_error(" -- [A*] Map type not defined.");
        }
    }

    bool Astar::insideLocalMap(const rog_map::Vec3f &pos) const {
        rog_map::Vec3i id_g;
        posToGlobalIndex(pos, id_g);
        return insideLocalMap(id_g);
    }

    bool Astar::insideLocalMap(const rog_map::Vec3i &id_g) const {
        rog_map::Vec3i delta = id_g - md_.local_map_center_id_g;
        if (fabs(delta.x()) > cfg_.map_size_i.x() ||
            fabs(delta.y()) > cfg_.map_size_i.y() ||
            fabs(delta.z()) > cfg_.map_size_i.z()) {
            return false;
        }
        return true;
    }

    void Astar::setVisualProcessEn(const bool &en) {
        cfg_.visual_process = en;
    }

    void Astar::retrievePath(GridNodePtr current, vector<GridNodePtr> &path) {
        path.push_back(current);
        while (current->father_ptr != NULL) {
            current = current->father_ptr;
            path.push_back(current);
        }
    }

    void Astar::ConvertNodePathToPointPath(const vector<GridNodePtr> &node_path, rog_map::vec_Vec3f &point_path) {
        point_path.clear();
        for (auto ptr: node_path) {
            rog_map::Vec3f pos;
            globalIndexToPos(ptr->id_g, pos);
            point_path.push_back(pos);
        }
        reverse(point_path.begin(), point_path.end());
    }

    void Astar::setFineInfNeighbors(const int &neighbor_step) {
        neighbor_list.clear();
        for (int i = -neighbor_step; i <= neighbor_step; i++) {
            for (int j = -neighbor_step; j <= neighbor_step; j++) {
                for (int k = -neighbor_step; k <= neighbor_step; k++) {
                    if (i == 0 && j == 0 && k == 0) {
                        continue;
                    }
                    if (i * i + j * j + k * k > neighbor_step * neighbor_step) {
                        continue;
                    }
                    neighbor_list.emplace_back(i, j, k);
                }
            }
        }
    }


    RET_CODE Astar::pointToPointPathSearch(const rog_map::Vec3f &start_pt, const rog_map::Vec3f &end_pt,
                                           const int &flag, const double &searching_horizon,
                                           rog_map::vec_Vec3f &out_path, const double &time_out,
                                           const bool prefer_start_goal_altitude) {
        const double effective_time_out = time_out > 0.0 ? time_out : cfg_.search_time_limit_s;
        RET_CODE setup_ret = setup(start_pt, end_pt, flag, searching_horizon);
        if (setup_ret != SUCCESS) {
            return setup_ret;
        }
        out_path.clear();
        double time_1 = ros_ptr_->getSimTime();
        ++rounds_;
        /// 2) Switch both start and end point to local map

        rog_map::Vec3f hit_pt;
        rog_map::Vec3f local_start_pt, local_end_pt;
        bool start_pt_out_local_map = false;
        bool end_pt_out_local_map = false;

        local_start_pt = start_pt;
        local_end_pt = end_pt;

        if (!insideLocalMap(start_pt)) {
            ros_ptr_->warn(" -- [A*] Start point [{}] is out of local map, find a waypoint to the map edge.",
                           start_pt.transpose());
            if (rog_map::lineIntersectBox(start_pt, md_.local_map_center_d, md_.local_map_min_d,
                                          md_.local_map_max_d, hit_pt)) {
                rog_map::Vec3f dir = (hit_pt - start_pt).normalized();
                double dis = (hit_pt - start_pt).norm();
                local_start_pt = start_pt + dir * (dis + md_.resolution * 2);
                start_pt_out_local_map = true;
                const auto nearest = map_ptr_->nearestNotOccupied(
                        local_start_pt, GridLayer::kInflated, 3.0);
                if (!nearest) {
                    if (cfg_.visual_process || cfg_.debug_visualization_en) {
                        ros_ptr_->vizAstarPoints(local_start_pt, Color::Orange(),
                                                 "local_start_pt",
                                                 0.3, 1);
                    }
                    cout << RED <<
                         " -- [A*] " << RET_CODE_STR[INIT_ERROR]
                         << " : start point deeply occupied, cannot find feasible path.\n" << RESET << endl;
                    return INIT_ERROR;
                }
                local_start_pt = *nearest;
            }
        }

        if (!insideLocalMap(end_pt)) {
            rog_map::Vec3f seed_pt = start_pt_out_local_map ? md_.local_map_center_d : start_pt;
            if (rog_map::lineIntersectBox(end_pt, seed_pt, md_.local_map_min_d, md_.local_map_max_d,
                                          hit_pt)) {
                rog_map::Vec3f dir = (hit_pt - end_pt).normalized();
                double dis = (hit_pt - end_pt).norm();
                local_end_pt = end_pt + dir * (dis + 2.5);
                end_pt_out_local_map = true;

                const auto nearest = map_ptr_->nearestNotOccupied(
                        local_end_pt, GridLayer::kInflated, 2.0);
                if (!nearest) {
                    ros_ptr_->error(
                            " -- [A*] Error with: {}, Goal point [{}] deeply occupied, cannot find feasible path.",
                            RET_CODE_STR[INIT_ERROR],
                            local_end_pt.transpose());
                    if (cfg_.visual_process || cfg_.debug_visualization_en) {
                        ros_ptr_->vizAstarPoints(local_end_pt, Color::Red(), "local_end_pt",
                                                 0.5,
                                                 1);
                    }
                    return INIT_ERROR;
                }
                local_end_pt = *nearest;
            }
        }

        // The A* scratch grid is centred on every search start and can extend
        // beyond the currently allocated ROG map.  Projecting only to that
        // scratch-grid boundary lets the direct-line fast path return points
        // that mapping/corridor generation cannot represent.  Intersect the
        // endpoint with the actual ROG map as the authoritative bound.
        if (!map_ptr_->contains(local_end_pt)) {
            const auto geometry = map_ptr_->geometry();
            const Vec3f map_center = geometry.local_center_m;
            const Vec3f map_half_size = 0.5 * geometry.local_size_m;
            const Vec3f map_margin = Vec3f::Constant(2.0 * md_.resolution);
            const Vec3f map_min = map_center - map_half_size + map_margin;
            const Vec3f map_max = map_center + map_half_size - map_margin;
            if (!rog_map::lineIntersectBox(local_end_pt, local_start_pt,
                                           map_min, map_max, hit_pt)) {
                ros_ptr_->error(
                        " -- [A*] Cannot project endpoint {} into ROG map [{}, {}]",
                        local_end_pt.transpose(), map_min.transpose(), map_max.transpose());
                return INIT_ERROR;
            }
            const Vec3f inward = (local_start_pt - hit_pt).normalized();
            local_end_pt = hit_pt + inward * (2.0 * md_.resolution);
            end_pt_out_local_map = true;
        }

        if (cfg_.visual_process) {
            ros_ptr_->vizAstarPoints(local_start_pt, Color::Orange(), "local_start_pt",
                                     0.3,
                                     1);
            ros_ptr_->vizAstarPoints(local_end_pt, Color::Green(), "local_end_pt", 0.3,
                                     1);
        }
        rog_map::Vec3i start_idx, end_idx;
        posToGlobalIndex(local_start_pt, start_idx);
        posToGlobalIndex(local_end_pt, end_idx);
        if (cfg_.visual_process) {
            ros_ptr_->vizAstarPoints(local_start_pt, Color::Orange(), "local_start_pt", 0.3, 1);
            ros_ptr_->vizAstarPoints(local_end_pt, Color::Green(), "local_end_pt", 0.3, 1);
        }
        if (!insideLocalMap(start_idx) || !insideLocalMap(end_idx)) {
            cout << RED << " -- [RM] Start or end point is out of local map, which should not happen." <<
                 RESET
                 << endl;
            ros_ptr_->error(" -- [RM] Start [{}] or end point [{}] is out of local map, which should not happen.",
                            local_start_pt.transpose(),
                            local_end_pt.transpose()
                            );
            if(cfg_.visual_process || cfg_.debug_visualization_en) {
                ros_ptr_->vizAstarPoints(local_start_pt, Color::Orange(), "local_start_pt", 0.3, 1);
                ros_ptr_->vizAstarPoints(local_end_pt, Color::Green(), "local_end_pt", 0.3, 1);
            }
            return INIT_ERROR;
        }

        // UAV missions normally prefer a lateral detour over diving toward
        // unseen ground.  First search the start/goal altitude slab, expanded
        // by one map cell for discretization.  The caller retries unrestricted
        // 3-D A* when this preferred search cannot find a route, so vertical
        // avoidance remains available where it is genuinely required.
        const double preferred_min_z =
                std::min(local_start_pt.z(), local_end_pt.z()) - md_.resolution;
        const double preferred_max_z =
                std::max(local_start_pt.z(), local_end_pt.z()) + md_.resolution;

        // A large open local segment does not need a 3-D graph expansion.
        // ROG-Map checks the complete segment against the same inflated map
        // and unknown-space policy used below. Preserve a resolution-dense
        // path so corridor generation still validates and bounds every seed
        // segment instead of receiving one long unchecked edge.
        if (map_ptr_->contains(local_start_pt) &&
            map_ptr_->contains(local_end_pt) &&
            map_ptr_->isSegmentTraversable(
                    local_start_pt, local_end_pt,
                    md_.use_inf_map ? GridLayer::kInflated : GridLayer::kEvidence,
                    md_.unknown_as_occ ? UnknownPolicy::kRequireKnownFree
                                       : UnknownPolicy::kAllowUnknown)) {
            const Vec3f delta = local_end_pt - local_start_pt;
            const int sample_count = std::max(
                    1, static_cast<int>(std::ceil(delta.norm() / md_.resolution)));
            out_path.reserve(static_cast<std::size_t>(sample_count + 1));
            for (int sample = 0; sample <= sample_count; ++sample) {
                const double ratio = static_cast<double>(sample) /
                                     static_cast<double>(sample_count);
                out_path.emplace_back(local_start_pt + ratio * delta);
            }
            return end_pt_out_local_map ? REACH_HORIZON : REACH_GOAL;
        }


        GridNodePtr startPtr = grid_node_buffer_[getLocalIndexHash(start_idx)];
        GridNodePtr endPtr = grid_node_buffer_[getLocalIndexHash(end_idx)];
        endPtr->id_g = end_idx;

        std::priority_queue<GridNodePtr, std::vector<GridNodePtr>, NodeComparator> open_set;
        std::priority_queue<GridNodePtr, std::vector<GridNodePtr>, FrontierComparator> frontier_queue;


        GridNodePtr neighborPtr = NULL;
        GridNodePtr current = NULL;

        startPtr->id_g = start_idx;
        startPtr->rounds = rounds_;
        startPtr->distance_score = 0;
        startPtr->total_score = getHeu(startPtr, endPtr, cfg_.heu_type);
        startPtr->state = GridNode::OPENSET; //put start node in open set
        startPtr->father_ptr = NULL;
        open_set.push(startPtr); //put start in open set
        int num_iter = 0;
        vector<GridNodePtr> node_path;

        if (cfg_.visual_process) {
            ros_ptr_->vizAstarPoints(
                    start_pt,
                    Color::Green(),
                    "start_pt",
                    0.3, 1);
            ros_ptr_->vizAstarPoints(
                    end_pt,
                    Color::Blue(),
                    "goal_pt",
                    0.3, 1);
        }

        while (!open_set.empty()) {
            num_iter++;
            current = open_set.top();
            open_set.pop();
            if (cfg_.visual_process) {
                rog_map::Vec3f local_pt;
                globalIndexToPos(current->id_g, local_pt);
                ros_ptr_->vizAstarPoints(
                        local_pt,
                        Color(Color::Pink(), 0.5),
                        "astar_process",
                        0.1);
                usleep(1000);
            }
            if (current->id_g(0) == endPtr->id_g(0) &&
                current->id_g(1) == endPtr->id_g(1) &&
                current->id_g(2) == endPtr->id_g(2)) {
                retrievePath(current, node_path);
                if (start_pt_out_local_map) {
                    rog_map::Vec3i start_idx_g;
                    posToGlobalIndex(start_pt, start_idx_g);
                    GridNodePtr temp_ptr(new GridNode);
                    temp_ptr->id_g = start_idx_g;
                    node_path.push_back(temp_ptr);
                }
                ConvertNodePathToPointPath(node_path, out_path);
                // Reaching a goal projected onto the local-map boundary is not
                // equivalent to reaching the mission goal.  Reporting
                // REACH_GOAL makes PathSearch append the remote mission goal to
                // this dense local path, creating one unvalidated corridor seed
                // line from the map boundary to the remote goal.
                return end_pt_out_local_map ? REACH_HORIZON : REACH_GOAL;
            }

            // Distance terminate condition
            if (searching_horizon > 0 && current->distance_score > searching_horizon / md_.resolution) {
                GridNodePtr local_goal = current;
                if (md_.unknown_as_occ && !frontier_queue.empty()) {
                    local_goal = frontier_queue.top()->father_ptr;
                    if (local_goal->distance_to_goal > current->distance_to_goal) {
                        local_goal = current;
                    }
                }
                retrievePath(local_goal, node_path);
                if (start_pt_out_local_map) {
                    node_path.push_back(startPtr);
                }
                ConvertNodePathToPointPath(node_path, out_path);
                return REACH_HORIZON;
            }


            current->state = GridNode::CLOSEDSET; //move current node from open set to closed set.

            for (int dx = -1; dx <= 1; dx++)
                for (int dy = -1; dy <= 1; dy++)
                    for (int dz = -1; dz <= 1; dz++) {
                        if (dx == 0 && dy == 0 && dz == 0) {
                            continue;
                        }
                        if (!cfg_.allow_diag &&
                            (std::abs(dx) + std::abs(dy) + std::abs(dz) > 1)) {
                            continue;
                        }

                        rog_map::Vec3i neighborIdx;
                        rog_map::Vec3f neighborPos;
                        neighborIdx(0) = (current->id_g)(0) + dx;
                        neighborIdx(1) = (current->id_g)(1) + dy;
                        neighborIdx(2) = (current->id_g)(2) + dz;
                        globalIndexToPos(neighborIdx, neighborPos);

                        if (!insideLocalMap(neighborIdx)) {
                            continue;
                        }
                        if (prefer_start_goal_altitude &&
                            (neighborPos.z() < preferred_min_z ||
                             neighborPos.z() > preferred_max_z)) {
                            continue;
                        }

                        // A free diagonal endpoint does not imply a free
                        // continuous edge: the segment can cut through the
                        // corner of an inflated occupied voxel. Corridor
                        // generation and CIRI operate on continuous seed
                        // lines, so A* must validate the same geometry.
                        if (std::abs(dx) + std::abs(dy) + std::abs(dz) > 1) {
                            rog_map::Vec3f current_pos;
                            globalIndexToPos(current->id_g, current_pos);
                            if (!map_ptr_->isSegmentTraversable(
                                    current_pos, neighborPos,
                                    md_.use_inf_map ? GridLayer::kInflated : GridLayer::kEvidence,
                                    md_.unknown_as_occ ? UnknownPolicy::kRequireKnownFree
                                                       : UnknownPolicy::kAllowUnknown)) {
                                continue;
                            }
                        }

                        CellState neighbor_type;

                        if (md_.use_inf_map) {
                            neighbor_type = map_ptr_->classify(neighborPos, GridLayer::kInflated);
                        } else {
                            if (!md_.use_inf_neighbor) {
                                neighbor_type = map_ptr_->classify(neighborPos, GridLayer::kEvidence);
                            } else {
                                // use prob map, but query all neighbors of the current node
                                // if there is one neighbor is occupied, then the neighbor is occupied.
                                neighbor_type = neighborHaveOne(OCCUPIED_CELL, neighborIdx) ? OCCUPIED_CELL : UNDEFINED_CELL;
                                // if there is one known free neighbor, then the neighbor is known free.
                                if (md_.unknown_as_occ && neighbor_type != OCCUPIED_CELL) {
                                    neighbor_type = neighborHaveOne(KNOWN_FREE_CELL, neighborIdx) ? KNOWN_FREE_CELL : UNKNOWN_CELL;
                                }
                            }
                        }

                        if (neighbor_type == CellState::kOccupied ||
                            neighbor_type == CellState::kOutOfMap) {
                            continue;
                        }

                        neighborPtr = grid_node_buffer_[getLocalIndexHash(neighborIdx)];
                        if (neighborPtr == nullptr) {
                            cout << RED << " -- [RM] neighborPtr is null, which should not happen." <<
                                 RESET
                                 << endl;
                            continue;
                        }
                        neighborPtr->id_g = neighborIdx;

                        bool flag_explored = neighborPtr->rounds == rounds_;

                        if (flag_explored && neighborPtr->state == GridNode::CLOSEDSET) {
                            continue; //in closed set.
                        }

                        if (md_.unknown_as_occ && neighbor_type == CellState::kUnknown && neighborPtr) {
                            // the frontier is recorded but not expand.
                            neighborPtr->father_ptr = current;
                            rog_map::Vec3f pos;
                            globalIndexToPos(neighborIdx, pos);
                            neighborPtr->distance_to_goal = getHeu(neighborPtr, endPtr, cfg_.heu_type);
                            frontier_queue.push(neighborPtr);
                            continue;
                        }

                        neighborPtr->rounds = rounds_;
                        double distance_score = sqrt(dx * dx + dy * dy + dz * dz);
                        distance_score = current->distance_score + distance_score;
                        rog_map::Vec3f pos;
                        globalIndexToPos(neighborIdx, pos);
                        double heu_score = getHeu(neighborPtr, endPtr, cfg_.heu_type);

                        if (!flag_explored) {
                            //discover a new node
                            neighborPtr->state = GridNode::OPENSET;
                            neighborPtr->father_ptr = current;
                            neighborPtr->distance_score = distance_score;
                            neighborPtr->distance_to_goal = heu_score;
                            neighborPtr->total_score = distance_score + heu_score;
                            open_set.push(neighborPtr); //put neighbor in open set and record it.
                        } else if (distance_score < neighborPtr->distance_score) {
                            neighborPtr->father_ptr = current;
                            neighborPtr->distance_score = distance_score;
                            neighborPtr->distance_to_goal = heu_score;
                            neighborPtr->total_score = distance_score + heu_score;
                        }
                    }
            double time_2 = ros_ptr_->getSimTime();
            if (!cfg_.visual_process && (time_2 - time_1) > effective_time_out) {
                fmt::print(fg(fmt::color::indian_red),
                           "Failed in A star path searching after {} iterations: "
                           "{} seconds time limit exceeded.\n",
                           num_iter, effective_time_out);
                return TIME_OUT;
            }
        }
        double time_2 = ros_ptr_->getSimTime();
        if ((time_2 - time_1) > effective_time_out) {
            fmt::print(fg(fmt::color::indian_red), "Time consume in A star path finding is {} s, iter={}.\n",
                       (time_2 - time_1),
                       num_iter);
            return NO_PATH;
        }

        if (md_.unknown_as_occ && !frontier_queue.empty()) {
            GridNodePtr local_goal{nullptr};
            while (!frontier_queue.empty()) {
                GridNodePtr candidate = frontier_queue.top();
                frontier_queue.pop();
                rog_map::Vec3f pos;
                globalIndexToPos(candidate->id_g, pos);
                if ((pos - start_pt).norm() < 1.0) {
                    continue;
                }
                local_goal = candidate;
                break;
            }
            if (local_goal == nullptr) {
                cout << RED << " -- [A*] No frontier farther than 1 m, return."
                     << RESET << endl;
                return NO_PATH;
            }
            retrievePath(local_goal, node_path);
            if (start_pt_out_local_map) {
                node_path.push_back(startPtr);
            }
            ConvertNodePathToPointPath(node_path, out_path);
            cout << BLUE << "Frontier queue: " << frontier_queue.size() << endl;
            return REACH_HORIZON;
        }
        ros_ptr_->error(" -- [A*] Point to point path cannot find path with iter num: {}, return.", num_iter);
        return NO_PATH;
    }

    RET_CODE Astar::escapePathSearch(const rog_map::Vec3f &start_pt, const int flag,
                                     rog_map::vec_Vec3f &out_path,
                                     const bool prefer_start_altitude) {
        // setup() records the goal even though escape search only uses the
        // start-centred horizon. Avoid propagating an indeterminate Eigen
        // vector into mission state.
        Vec3f tmp = start_pt;
        RET_CODE setup_ret = setup(start_pt, tmp, flag, 999);
        if (setup_ret != SUCCESS) {
            return setup_ret;
        }

        double time_1 = ros_ptr_->getSimTime();
        ++rounds_;

        posToGlobalIndex(md_.local_map_center_d, md_.local_map_center_id_g);
        /// 2) Check start point

        if (!insideLocalMap(start_pt) ||
            !map_ptr_->contains(start_pt)) {
            fmt::print(fg(fmt::color::indian_red), " -- [A*] {}: escape start point is not inside local map.\n",
                       RET_CODE_STR[INIT_ERROR].c_str());
            return INIT_ERROR;
        }
        rog_map::Vec3f local_start_pt = start_pt;
        const CellState exact_start_type = map_ptr_->classify(
                local_start_pt, md_.use_inf_map ? GridLayer::kInflated : GridLayer::kEvidence);
        // UNKNOWN_AS_FREE means an exact non-occupied start already satisfies
        // the escape contract. Snapping it to a voxel centre can otherwise
        // introduce an artificial altitude step on every planning cycle.
        if (md_.unknown_as_free && exact_start_type != CellState::kOccupied &&
            exact_start_type != CellState::kOutOfMap) {
            out_path.clear();
            return NO_NEED;
        }
        const auto nearest_start = map_ptr_->nearestNotOccupied(
                start_pt, GridLayer::kEvidence, 3.0);
        if (!nearest_start) {
            cout << RED <<
                 " -- [A*] " << RET_CODE_STR[INIT_ERROR]
                 << " : escape start point deeply occupied, cannot find feasible path.\n" << RESET << endl;
            return INIT_ERROR;
        }
        local_start_pt = *nearest_start;

        rog_map::Vec3i start_idx;
        posToGlobalIndex(local_start_pt, start_idx);

        GridNodePtr startPtr = grid_node_buffer_[getLocalIndexHash(start_idx)];
        std::priority_queue<GridNodePtr, std::vector<GridNodePtr>, NodeComparator> open_set;
        GridNodePtr neighborPtr = NULL;
        GridNodePtr current = NULL;

        startPtr->id_g = start_idx;
        startPtr->rounds = rounds_;
        startPtr->distance_score = 0;
        startPtr->total_score = 0;
        startPtr->state = GridNode::OPENSET; //put start node in open set
        startPtr->father_ptr = NULL;
        open_set.push(startPtr); //put start in open set
        int num_iter = 0;

        vector<GridNodePtr> node_path;

        if (cfg_.visual_process) {
            ros_ptr_->vizAstarPoints(local_start_pt, Color::Orange(), "local_start_pt",
                                     0.05,
                                     1);
        }
        while (!open_set.empty()) {
            num_iter++;
            current = open_set.top();
            open_set.pop();
            if (cfg_.visual_process) {
                rog_map::Vec3f local_pt;
                globalIndexToPos(current->id_g, local_pt);
                ros_ptr_->vizAstarPoints(local_pt,
                                         Color::Pink(),
                                         "astar_process",
                                         0.05);
                usleep(1000);
            }
            rog_map::Vec3f cur_pos;
            globalIndexToPos(current->id_g, cur_pos);
            const CellState cur_inf_type = map_ptr_->classify(cur_pos, GridLayer::kInflated);
            if (md_.unknown_as_occ && cur_inf_type != CellState::kOccupied &&
                cur_inf_type != CellState::kUnknown) {
                retrievePath(current, node_path);
                ConvertNodePathToPointPath(node_path, out_path);
//                double time_2 = ros_ptr_->getSimTime();
                //                    printf("\033[34m Escape: A star iter:%d, time:%.3f ms\033[0m\n", num_iter, (time_2 - time_1).toSec() * 1000);
                return REACH_HORIZON;
            }

            if (md_.unknown_as_free && cur_inf_type != CellState::kOccupied) {
                retrievePath(current, node_path);
                ConvertNodePathToPointPath(node_path, out_path);
//                double time_2 = ros_ptr_->getSimTime();
                //                    printf("\033[34m Escape: A star iter:%d, time:%.3f ms\033[0m\n", num_iter, (time_2 - time_1).toSec() * 1000);
                return REACH_HORIZON;
            }

            current->state = GridNode::CLOSEDSET; //move current node from open set to closed set.

            for (int dx = -1; dx <= 1; dx++)
                for (int dy = -1; dy <= 1; dy++)
                    for (int dz = -1; dz <= 1; dz++) {
                        if (dx == 0 && dy == 0 && dz == 0) {
                            continue;
                        }
                        rog_map::Vec3i neighborIdx;
                        rog_map::Vec3f neighborPos;
                        neighborIdx(0) = (current->id_g)(0) + dx;
                        neighborIdx(1) = (current->id_g)(1) + dy;
                        neighborIdx(2) = (current->id_g)(2) + dz;
                        globalIndexToPos(neighborIdx, neighborPos);
                        if (!map_ptr_->contains(neighborPos) ||
                            !insideLocalMap(neighborIdx)) {
                            continue;
                        }
                        if (prefer_start_altitude &&
                            std::abs(neighborPos.z() - local_start_pt.z()) > md_.resolution) {
                            continue;
                        }

                        CellState neighbor_type;
                        if (md_.use_inf_map) {
                            neighbor_type = map_ptr_->classify(neighborPos, GridLayer::kInflated);
                        } else {
                            neighbor_type = map_ptr_->classify(neighborPos, GridLayer::kEvidence);
                        }

                        if (neighbor_type == CellState::kOccupied ||
                            neighbor_type == CellState::kOutOfMap) {
                            continue;
                        }

                        if (md_.unknown_as_occ && neighbor_type == CellState::kUnknown) {
                            continue;
                        }

                        neighborPtr = grid_node_buffer_[getLocalIndexHash(neighborIdx)];
                        if (neighborPtr == nullptr) {
                            cout << RED << " -- [RM] neighborPtr is null, which should not happen" <<
                                 RESET << endl;
                            continue;
                        }
                        neighborPtr->id_g = neighborIdx;

                        bool flag_explored = neighborPtr->rounds == rounds_;

                        if (flag_explored && neighborPtr->state == GridNode::CLOSEDSET) {
                            continue; //in closed set.
                        }

                        neighborPtr->rounds = rounds_;
                        double distance_score = sqrt(dx * dx + dy * dy + dz * dz);
                        distance_score = current->distance_score + distance_score;
                        rog_map::Vec3f pos;
                        globalIndexToPos(neighborIdx, pos);
                        double heu_score = 0;
                        if (!flag_explored) {
                            //discover a new node
                            neighborPtr->state = GridNode::OPENSET;
                            neighborPtr->father_ptr = current;
                            neighborPtr->distance_score = distance_score;
                            neighborPtr->total_score = distance_score + heu_score;
                            open_set.push(neighborPtr); //put neighbor in open set and record it.
                        } else if (distance_score < neighborPtr->distance_score) {
                            neighborPtr->father_ptr = current;
                            neighborPtr->distance_score = distance_score;
                            neighborPtr->total_score = distance_score + heu_score;
                        }
                    }
            double time_2 = ros_ptr_->getSimTime();
            if (!cfg_.visual_process && (time_2 - time_1) > 0.2) {
                fmt::print(fg(fmt::color::indian_red),
                           "Failed in A star path searching !!! 0.2 seconds time limit exceeded.\n");
                return TIME_OUT;
            }
        }
        double time_2 = ros_ptr_->getSimTime();
        if ((time_2 - time_1) > 0.1) {
            fmt::print(fg(fmt::color::indian_red), "Time consume in A star path finding is {} s, iter={}.\n",
                       (time_2 - time_1),
                       num_iter);
        }
        cout << RED << " -- [A*] Escape path searcher, cannot find path, return." << RESET << endl;
        return NO_PATH;
    }

    bool Astar::neighborHaveOne(const CellState& type, const rog_map::Vec3i& src_id) {
        for (const auto& nei : neighbor_list) {
            rog_map::Vec3i nei_id = src_id + nei;
            if (!insideLocalMap(nei_id)) {
                continue;
            }
            rog_map::Vec3f nei_pos;
            globalIndexToPos(nei_id, nei_pos);
            CellState nei_type;
            if (md_.use_inf_map) {
                nei_type = map_ptr_->classify(nei_pos, GridLayer::kInflated);
            }
            else {
                nei_type = map_ptr_->classify(nei_pos, GridLayer::kEvidence);
            }
            if (nei_type == type) {
                return true;
            }
        }
        return false;
    }
}
