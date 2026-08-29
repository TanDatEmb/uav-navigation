/*
 * Product-owned navigation implementation.
 * Algorithmic provenance and external attributions are documented in the
 * package documentation; they are not part of the runtime API or behaviour.
 */

#include <path_search/astar.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <fmt/color.h>
#include <rog_map/rog_map_core/common_lib.hpp>

using namespace color_text;
using namespace navigation_math;

namespace path_search {
    using namespace rog_map;
    using navigation_world_model::CellState;
    using navigation_world_model::GridLayer;
    using navigation_world_model::UnknownPolicy;
    using navigation_world_model::isCellTraversable;
    constexpr CellState OCCUPIED_CELL = CellState::kOccupied;
    constexpr CellState KNOWN_FREE_CELL = CellState::kKnownFree;
    constexpr CellState UNKNOWN_CELL = CellState::kUnknown;

    bool checkedIndexStep(const int base, const int delta, int& result) {
        if ((delta > 0 && base > std::numeric_limits<int>::max() - delta) ||
            (delta < 0 && base < std::numeric_limits<int>::min() - delta)) {
            return false;
        }
        result = base + delta;
        return true;
    }


    Astar::Astar(const PathSearchConfig& config,
                 const navigation_planner_context::PlannerRuntimeContext::Ptr &planner_context,
                 navigation_world_model::WorldModelViewPtr rm,
                 const double search_time_limit_s)
        : map_ptr_(std::move(rm)), planner_context_(planner_context),
          cfg_(config), search_time_limit_s_(search_time_limit_s) {
        if (!std::isfinite(search_time_limit_s_) || search_time_limit_s_ <= 0.0) {
            throw std::invalid_argument(
                "planner A* attempt budget must be finite and positive");
        }
        if (!planner_context_) {
            throw std::invalid_argument("planner A* runtime context must not be null");
        }
        if (cfg_.heu_type < 0 || cfg_.heu_type > 2 ||
            !std::isfinite(cfg_.heuristic_weight) ||
            cfg_.heuristic_weight < 1.0 || cfg_.heuristic_weight > 5.0) {
            throw std::invalid_argument("planner A* heuristic configuration is invalid");
        }
        cout << GREEN << " -- [A*] Initialize sparse search workspace." << RESET << endl;
    }

    Astar::~Astar() = default;

    RET_CODE
    Astar::setup(const Vec3f &start_pt, const Vec3f &goal_pt, const int &flag, const double &searching_horizon) {
        if (!start_pt.allFinite() || !goal_pt.allFinite() ||
            !std::isfinite(searching_horizon)) {
            return INIT_ERROR;
        }
        visited_nodes_.clear();
        frontier_sequence_ = 0U;
        md_.start_pt = start_pt;
        md_.goal_pt = goal_pt;
        md_.mission_rcv_WT = planner_context_->getSimTime();
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
        if (!map_ptr_) {
            if (planner_context_) {
                planner_context_->error(" -- [A*] World model is unavailable.");
            }
            return INIT_ERROR;
        }
        const auto geometry = map_ptr_->geometry();
        md_.resolution = md_.use_prob_map
                ? geometry.evidence_resolution_m
                : geometry.inflated_resolution_m;
        const auto bounds = md_.use_prob_map
                ? geometry.evidence_bounds
                : geometry.inflated_bounds;
        if (!std::isfinite(md_.resolution) || md_.resolution <= 0.0 ||
            !bounds.valid()) {
            if (planner_context_) {
                planner_context_->error(
                        " -- [A*] World geometry has no finite grid resolution or exact layer bounds.");
            }
            return INIT_ERROR;
        }
        for (int axis = 0; axis < 3; ++axis) {
            const int dimension = bounds.dimensions(axis);
            const std::int64_t minimum_index = bounds.global_min_index(axis);
            if (dimension <= 0 ||
                minimum_index > std::numeric_limits<std::int64_t>::max() -
                    (static_cast<std::int64_t>(dimension) - 1)) {
                if (planner_context_) {
                    planner_context_->error(
                            " -- [A*] World geometry axis {} cannot define a grid window.", axis);
                }
                return INIT_ERROR;
            }
            const std::int64_t maximum_index = minimum_index +
                    static_cast<std::int64_t>(dimension) - 1;
            if (maximum_index > std::numeric_limits<int>::max() ||
                maximum_index < std::numeric_limits<int>::min()) {
                if (planner_context_) {
                    planner_context_->error(
                            " -- [A*] World geometry axis {} cannot define a grid window.", axis);
                }
                return INIT_ERROR;
            }
            md_.local_global_min_index(axis) = bounds.global_min_index(axis);
            md_.local_global_max_index(axis) = static_cast<int>(maximum_index);
            md_.local_voxel_count(axis) = dimension;
            md_.local_lower_extent_i(axis) = dimension / 2;
            md_.local_upper_extent_i(axis) = dimension - 1 - dimension / 2;
        }
        const auto maximum_index = std::numeric_limits<std::size_t>::max();
        const auto y_count = static_cast<std::size_t>(md_.local_voxel_count.y());
        const auto z_count = static_cast<std::size_t>(md_.local_voxel_count.z());
        const auto x_count = static_cast<std::size_t>(md_.local_voxel_count.x());
        if (z_count == 0U || y_count > maximum_index / z_count ||
            x_count > maximum_index / (y_count * z_count)) {
            if (planner_context_) {
                planner_context_->error(" -- [A*] World geometry index range overflows local addressing.");
            }
            return INIT_ERROR;
        }
        if (searching_horizon > 0) {
            md_.local_map_center_d = start_pt;
        } else {
            md_.local_map_center_d = (start_pt.cast<double>() + goal_pt.cast<double>()) / 2.0;
        }

        posToGlobalIndex(md_.local_map_center_d, md_.local_map_center_id_g);
        globalIndexToPos(md_.local_global_min_index, md_.local_map_min_d);
        globalIndexToPos(md_.local_global_max_index, md_.local_map_max_d);
        if (cfg_.visual_process||cfg_.debug_visualization_en) {
            planner_context_->vizAstarBoundingBox(md_.local_map_min_d, md_.local_map_max_d);
        }

        return SUCCESS;
    }

    double Astar::getHeu(GridNodePtr node1, GridNodePtr node2, int type) const {
        switch (type) {
            case DIAG: {
                double dx = std::abs(static_cast<double>(node1->id_g(0)) -
                                     static_cast<double>(node2->id_g(0)));
                double dy = std::abs(static_cast<double>(node1->id_g(1)) -
                                     static_cast<double>(node2->id_g(1)));
                double dz = std::abs(static_cast<double>(node1->id_g(2)) -
                                     static_cast<double>(node2->id_g(2)));

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
                double dx = std::abs(static_cast<double>(node1->id_g(0)) -
                                     static_cast<double>(node2->id_g(0)));
                double dy = std::abs(static_cast<double>(node1->id_g(1)) -
                                     static_cast<double>(node2->id_g(1)));
                double dz = std::abs(static_cast<double>(node1->id_g(2)) -
                                     static_cast<double>(node2->id_g(2)));

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

    std::size_t Astar::getLocalIndexHash(const Vec3i &id_in) const {
        const std::int64_t local_x = static_cast<std::int64_t>(id_in.x()) -
                static_cast<std::int64_t>(md_.local_global_min_index.x());
        const std::int64_t local_y = static_cast<std::int64_t>(id_in.y()) -
                static_cast<std::int64_t>(md_.local_global_min_index.y());
        const std::int64_t local_z = static_cast<std::int64_t>(id_in.z()) -
                static_cast<std::int64_t>(md_.local_global_min_index.z());
        return static_cast<std::size_t>(local_x) *
                   static_cast<std::size_t>(md_.local_voxel_count(1)) *
                   static_cast<std::size_t>(md_.local_voxel_count(2)) +
               static_cast<std::size_t>(local_y) *
                   static_cast<std::size_t>(md_.local_voxel_count(2)) +
               static_cast<std::size_t>(local_z);
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
        return (id_g.array() >= md_.local_global_min_index.array()).all() &&
               (id_g.array() <= md_.local_global_max_index.array()).all();
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
        constexpr int kMaximumNeighborStep = 64;
        if (neighbor_step <= 0 || neighbor_step > kMaximumNeighborStep) return;
        for (int i = -neighbor_step; i <= neighbor_step; i++) {
            for (int j = -neighbor_step; j <= neighbor_step; j++) {
                for (int k = -neighbor_step; k <= neighbor_step; k++) {
                    if (i == 0 && j == 0 && k == 0) {
                        continue;
                    }
                    if (static_cast<std::int64_t>(i) * i +
                        static_cast<std::int64_t>(j) * j +
                        static_cast<std::int64_t>(k) * k >
                        static_cast<std::int64_t>(neighbor_step) * neighbor_step) {
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
        if (std::isnan(time_out) ||
            (std::isinf(time_out) && time_out > 0.0)) {
            return INIT_ERROR;
        }
        const double effective_time_out = time_out >= 0.0 ? time_out : search_time_limit_s_;
        const auto steady_deadline = std::chrono::steady_clock::now() +
            std::chrono::duration_cast<std::chrono::steady_clock::duration>(
                std::chrono::duration<double>(effective_time_out));
        RET_CODE setup_ret = setup(start_pt, end_pt, flag, searching_horizon);
        if (setup_ret != SUCCESS) {
            return setup_ret;
        }
        out_path.clear();
        if (std::chrono::steady_clock::now() >= steady_deadline) {
            return TIME_OUT;
        }
        ++rounds_;
        /// 2) Switch both start and end point to local map

        rog_map::Vec3f hit_pt;
        rog_map::Vec3f local_start_pt, local_end_pt;
        bool start_pt_out_local_map = false;
        bool end_pt_out_local_map = false;
        // Keep the endpoint inside the same two-cell AABB margin used by the
        // planner route-support contract. This is derived from the active
        // grid resolution; a fixed metre offset can erase most of a narrow
        // lateral window or create inconsistent support accounting.
        const double endpoint_inset_m = 2.0 * md_.resolution;

        local_start_pt = start_pt;
        local_end_pt = end_pt;

        if (!insideLocalMap(start_pt)) {
            planner_context_->warn(" -- [A*] Start point [{}] is out of local map, find a waypoint to the map edge.",
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
                        planner_context_->vizAstarPoints(local_start_pt, Color::Orange(),
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
                local_end_pt = end_pt + dir * (dis + endpoint_inset_m);
                end_pt_out_local_map = true;

                const auto nearest = map_ptr_->nearestNotOccupied(
                        local_end_pt, GridLayer::kInflated, 2.0);
                if (!nearest) {
                    planner_context_->error(
                            " -- [A*] Error with: {}, Goal point [{}] deeply occupied, cannot find feasible path.",
                            RET_CODE_STR[INIT_ERROR],
                            local_end_pt.transpose());
                    if (cfg_.visual_process || cfg_.debug_visualization_en) {
                        planner_context_->vizAstarPoints(local_end_pt, Color::Red(), "local_end_pt",
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
                planner_context_->error(
                        " -- [A*] Cannot project endpoint {} into ROG map [{}, {}]",
                        local_end_pt.transpose(), map_min.transpose(), map_max.transpose());
                return INIT_ERROR;
            }
            const Vec3f inward = (local_start_pt - hit_pt).normalized();
            local_end_pt = hit_pt + inward * endpoint_inset_m;
            end_pt_out_local_map = true;
        }

        if (cfg_.visual_process) {
            planner_context_->vizAstarPoints(local_start_pt, Color::Orange(), "local_start_pt",
                                     0.3,
                                     1);
            planner_context_->vizAstarPoints(local_end_pt, Color::Green(), "local_end_pt", 0.3,
                                     1);
        }
        rog_map::Vec3i start_idx, end_idx;
        posToGlobalIndex(local_start_pt, start_idx);
        posToGlobalIndex(local_end_pt, end_idx);
        if (cfg_.visual_process) {
            planner_context_->vizAstarPoints(local_start_pt, Color::Orange(), "local_start_pt", 0.3, 1);
            planner_context_->vizAstarPoints(local_end_pt, Color::Green(), "local_end_pt", 0.3, 1);
        }
        if (!insideLocalMap(start_idx) || !insideLocalMap(end_idx)) {
            cout << RED << " -- [RM] Start or end point is out of local map, which should not happen." <<
                 RESET
                 << endl;
            planner_context_->error(" -- [RM] Start [{}] or end point [{}] is out of local map, which should not happen.",
                            local_start_pt.transpose(),
                            local_end_pt.transpose()
                            );
            if(cfg_.visual_process || cfg_.debug_visualization_en) {
                planner_context_->vizAstarPoints(local_start_pt, Color::Orange(), "local_start_pt", 0.3, 1);
                planner_context_->vizAstarPoints(local_end_pt, Color::Green(), "local_end_pt", 0.3, 1);
            }
            return INIT_ERROR;
        }

        // The graph nodes are voxel centres, while the planner start is the
        // measured continuous pose (or a point on the already certified guide
        // prefix).  Starting the graph unconditionally at the containing
        // voxel centre can therefore create an unvalidated first edge: the
        // centre may be on the far side of an inflated obstacle even though
        // the measured pose itself is free.  That edge is only discovered by
        // PathSearch/CIRI after A* has returned, which strands hot replanning
        // at exactly the point where the vehicle needs a detour.
        //
        // Use a bounded virtual-start projection.  It never changes the
        // command start: PathSearch still prepends local_start_pt and checks
        // the continuous segment.  It only chooses the first graph node from
        // a small neighbourhood whose connection to the real pose has already
        // passed the same inflated-map/unknown-space oracle used by the
        // search.  If no such node exists, report NO_PATH and remain
        // fail-closed rather than accepting an unverified jump.
        const auto search_layer = md_.use_inf_map
                ? GridLayer::kInflated : GridLayer::kEvidence;
        const auto search_policy = md_.unknown_as_occ
                ? UnknownPolicy::kRequireKnownFree
                : UnknownPolicy::kAllowUnknown;
        rog_map::Vec3f graph_start_pt;
        globalIndexToPos(start_idx, graph_start_pt);
        if (!map_ptr_->isSegmentTraversable(
                    local_start_pt, graph_start_pt, search_layer, search_policy)) {
            struct StartCandidate {
                rog_map::Vec3i index;
                rog_map::Vec3f position;
                double score{std::numeric_limits<double>::infinity()};
            };
            std::optional<StartCandidate> best_candidate;
            constexpr int kVirtualStartSearchRadiusCells = 3;
            for (int radius = 1; radius <= kVirtualStartSearchRadiusCells; ++radius) {
                for (int dx = -radius; dx <= radius; ++dx) {
                    for (int dy = -radius; dy <= radius; ++dy) {
                        for (int dz = -radius; dz <= radius; ++dz) {
                            if (std::max({std::abs(dx), std::abs(dy), std::abs(dz)}) != radius) {
                                continue;
                            }
                            rog_map::Vec3i candidate_idx = start_idx;
                            candidate_idx += rog_map::Vec3i(dx, dy, dz);
                            if (!insideLocalMap(candidate_idx)) continue;
                            rog_map::Vec3f candidate_pos;
                            globalIndexToPos(candidate_idx, candidate_pos);
                            if (!map_ptr_->contains(candidate_pos) ||
                                (prefer_start_goal_altitude &&
                                 std::abs(candidate_pos.z() - local_start_pt.z()) > md_.resolution)) {
                                continue;
                            }
                            const CellState candidate_type = map_ptr_->classify(
                                    candidate_pos, search_layer);
                            if (!isCellTraversable(candidate_type,
                                                   UnknownPolicy::kAllowUnknown) ||
                                (md_.unknown_as_occ &&
                                 (candidate_type == CellState::kUnknown ||
                                  candidate_type == CellState::kFrontier))) {
                                continue;
                            }
                            if (!map_ptr_->isSegmentTraversable(
                                        local_start_pt, candidate_pos,
                                        search_layer, search_policy)) {
                                continue;
                            }
                            const double distance_from_pose =
                                    (candidate_pos - local_start_pt).squaredNorm();
                            const double distance_to_goal =
                                    (candidate_pos - local_end_pt).squaredNorm();
                            const double score = distance_from_pose + 0.05 * distance_to_goal;
                            if (!best_candidate || score < best_candidate->score) {
                                best_candidate = StartCandidate{
                                        candidate_idx, candidate_pos, score};
                            }
                        }
                    }
                }
                // Prefer the nearest reachable shell. A farther shell is
                // considered only when every closer shell is geometrically
                // blocked.
                if (best_candidate) break;
            }
            if (!best_candidate) {
                planner_context_->warn(
                        " -- [A*] no graph voxel is continuously reachable from start {}; "
                        "force return",
                        local_start_pt.transpose());
                return NO_PATH;
            }
            planner_context_->warn(
                    " -- [A*] virtual start shifted graph seed from {} to {} "
                    "while preserving measured start {}",
                    graph_start_pt.transpose(), best_candidate->position.transpose(),
                    local_start_pt.transpose());
            start_idx = best_candidate->index;
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
            const double distance = delta.cast<double>().norm();
            const double sample_ratio = distance / md_.resolution;
            if (!std::isfinite(distance) || !std::isfinite(sample_ratio) ||
                sample_ratio > static_cast<double>(std::numeric_limits<int>::max() - 1)) {
                return INIT_ERROR;
            }
            const int sample_count = std::max(
                    1, static_cast<int>(std::ceil(sample_ratio)));
            out_path.reserve(static_cast<std::size_t>(sample_count + 1));
            for (int sample = 0; sample <= sample_count; ++sample) {
                const double ratio = static_cast<double>(sample) /
                                     static_cast<double>(sample_count);
                out_path.emplace_back(local_start_pt + ratio * delta);
            }
            return end_pt_out_local_map ? REACH_HORIZON : REACH_GOAL;
        }

        GridNodePtr startPtr = nodeAt(getLocalIndexHash(start_idx));
        GridNodePtr endPtr = nodeAt(getLocalIndexHash(end_idx));
        endPtr->id_g = end_idx;

        using OpenSet = std::priority_queue<OpenSetEntry,
                                            std::vector<OpenSetEntry>,
                                            OpenSetComparator>;
        OpenSet open_set;
        std::priority_queue<FrontierEntry, std::vector<FrontierEntry>, FrontierComparator> frontier_queue;
        const auto discard_stale_frontier_entries = [&frontier_queue]() {
            while (!frontier_queue.empty()) {
                const FrontierEntry& entry = frontier_queue.top();
                if (entry.node != nullptr && entry.node->frontier_sequence == entry.sequence) {
                    return;
                }
                frontier_queue.pop();
            }
        };


        GridNodePtr neighborPtr = NULL;
        GridNodePtr current = NULL;

        startPtr->id_g = start_idx;
        startPtr->rounds = rounds_;
        startPtr->distance_score = 0;
        startPtr->total_score = getHeu(startPtr, endPtr, cfg_.heu_type);
        startPtr->state = GridNode::OPENSET; //put start node in open set
        startPtr->father_ptr = NULL;
        open_set.push({startPtr, startPtr->total_score, rounds_}); // put start in open set
        int num_iter = 0;
        vector<GridNodePtr> node_path;

        if (cfg_.visual_process) {
            planner_context_->vizAstarPoints(
                    start_pt,
                    Color::Green(),
                    "start_pt",
                    0.3, 1);
            planner_context_->vizAstarPoints(
                    end_pt,
                    Color::Blue(),
                    "goal_pt",
                    0.3, 1);
        }

        while (!open_set.empty()) {
            if (std::chrono::steady_clock::now() >= steady_deadline) {
                fmt::print(fg(fmt::color::indian_red),
                           "Failed in A star path searching after {} iterations: "
                           "steady-clock time limit exceeded.\n",
                           num_iter);
                return TIME_OUT;
            }
            num_iter++;
            const OpenSetEntry current_entry = open_set.top();
            open_set.pop();
            current = current_entry.node;
            if (current_entry.rounds != current->rounds ||
                current_entry.total_score != current->total_score ||
                current->state == GridNode::CLOSEDSET) {
                continue;
            }
            if (cfg_.visual_process) {
                rog_map::Vec3f local_pt;
                globalIndexToPos(current->id_g, local_pt);
                planner_context_->vizAstarPoints(
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
                    GridNode temporary_start;
                    temporary_start.id_g = start_idx_g;
                    node_path.push_back(&temporary_start);
                    ConvertNodePathToPointPath(node_path, out_path);
                    return end_pt_out_local_map ? REACH_HORIZON : REACH_GOAL;
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
                discard_stale_frontier_entries();
                if (md_.unknown_as_occ && !frontier_queue.empty()) {
                    const GridNodePtr frontier_parent = frontier_queue.top().node->father_ptr;
                    if (frontier_parent != nullptr &&
                        frontier_parent->distance_to_goal > current->distance_to_goal) {
                        local_goal = frontier_parent;
                    } else if (frontier_parent == nullptr) {
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
                        if (!checkedIndexStep(current->id_g(0), dx, neighborIdx(0)) ||
                            !checkedIndexStep(current->id_g(1), dy, neighborIdx(1)) ||
                            !checkedIndexStep(current->id_g(2), dz, neighborIdx(2))) {
                            continue;
                        }
                        globalIndexToPos(neighborIdx, neighborPos);

                        if (!insideLocalMap(neighborIdx)) {
                            continue;
                        }
                        if (prefer_start_goal_altitude &&
                            (neighborPos.z() < preferred_min_z ||
                             neighborPos.z() > preferred_max_z)) {
                            continue;
                        }

                        // Resolve graph state before invoking the continuous
                        // edge oracle. A CLOSED neighbour has already been
                        // expanded on this immutable world, so the reverse
                        // edge cannot improve it. The previous ordering paid
                        // for that ray query and discarded the result below.
                        // This changes no admissibility decision and avoids
                        // certifying most undirected edges twice.
                        neighborPtr = nodeAt(getLocalIndexHash(neighborIdx));
                        if (neighborPtr == nullptr) {
                            cout << RED << " -- [RM] neighborPtr is null, which should not happen." <<
                                 RESET
                                 << endl;
                            continue;
                        }
                        neighborPtr->id_g = neighborIdx;
                        const bool flag_explored = neighborPtr->rounds == rounds_;
                        if (flag_explored && neighborPtr->state == GridNode::CLOSEDSET) {
                            continue;
                        }

                        CellState neighbor_type;
                        if (md_.use_inf_map) {
                            neighbor_type = map_ptr_->classify(
                                    neighborPos, GridLayer::kInflated);
                        } else if (!md_.use_inf_neighbor) {
                            neighbor_type = map_ptr_->classify(
                                    neighborPos, GridLayer::kEvidence);
                        } else {
                            // Use the probability map, but query all
                            // neighbours of the candidate. If any is occupied,
                            // the candidate is occupied.
                            neighbor_type = neighborHaveOne(OCCUPIED_CELL, neighborIdx)
                                ? OCCUPIED_CELL : UNKNOWN_CELL;
                            // If one neighbour is known free, retain that
                            // evidence for the frontier policy. A missing
                            // neighbour is UNKNOWN, never an implicit
                            // traversable UNDEFINED value.
                            if (md_.unknown_as_occ &&
                                neighbor_type != OCCUPIED_CELL) {
                                neighbor_type = neighborHaveOne(
                                        KNOWN_FREE_CELL, neighborIdx)
                                    ? KNOWN_FREE_CELL : UNKNOWN_CELL;
                            }
                        }
                        if (!isCellTraversable(
                                    neighbor_type,
                                    UnknownPolicy::kAllowUnknown)) {
                            continue;
                        }

                        // A free endpoint does not imply a free continuous
                        // edge: the segment can cut through the corner of an
                        // inflated occupied voxel. This applies to axial
                        // edges as well as diagonals. Validate every graph
                        // edge under the selected search layer and, because
                        // the returned path is consumed by the inflated
                        // corridor/execution gates, under the authoritative
                        // inflated layer too. The latter is essential for the
                        // probability-map fallback; evidence-free is not
                        // equivalent to execution-safe.
                        rog_map::Vec3f current_pos;
                        globalIndexToPos(current->id_g, current_pos);
                        const auto search_layer = md_.use_inf_map
                                ? GridLayer::kInflated : GridLayer::kEvidence;
                        const auto search_policy = md_.unknown_as_occ
                                ? UnknownPolicy::kRequireKnownFree
                                : UnknownPolicy::kAllowUnknown;
                        if (!map_ptr_->isSegmentTraversable(
                                    current_pos, neighborPos,
                                    search_layer, search_policy)) {
                            continue;
                        }
                        // When A* already searches the inflated layer, the
                        // first query is the authoritative execution-clearance
                        // certificate for this exact edge. Repeating the same
                        // immutable ray query doubles the dominant inner-loop
                        // work without adding evidence. Probability-map search
                        // still receives the separate inflated-layer check.
                        if (search_layer != GridLayer::kInflated &&
                            !map_ptr_->isSegmentTraversable(
                                    current_pos, neighborPos,
                                    GridLayer::kInflated, search_policy)) {
                            continue;
                        }

                        if (md_.unknown_as_occ && neighbor_type == CellState::kUnknown && neighborPtr) {
                            // the frontier is recorded but not expand.
                            neighborPtr->father_ptr = current;
                            rog_map::Vec3f pos;
                            globalIndexToPos(neighborIdx, pos);
                            neighborPtr->distance_to_goal = getHeu(neighborPtr, endPtr, cfg_.heu_type);
                            neighborPtr->frontier_sequence = ++frontier_sequence_;
                            frontier_queue.push({neighborPtr,
                                                 neighborPtr->distance_to_goal,
                                                 neighborPtr->frontier_sequence});
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
                            open_set.push({neighborPtr, neighborPtr->total_score, rounds_});
                        } else if (distance_score < neighborPtr->distance_score) {
                            neighborPtr->father_ptr = current;
                            neighborPtr->distance_score = distance_score;
                            neighborPtr->distance_to_goal = heu_score;
                            neighborPtr->total_score = distance_score + heu_score;
                            // priority_queue has no decrease-key operation.
                            // Store the score in the entry; the old entry is
                            // discarded when its score no longer matches.
                            open_set.push({neighborPtr, neighborPtr->total_score, rounds_});
                        }
                    }
            if (std::chrono::steady_clock::now() >= steady_deadline) {
                fmt::print(fg(fmt::color::indian_red),
                           "Failed in A star path searching after {} iterations: "
                           "steady-clock time limit exceeded.\n",
                           num_iter);
                return TIME_OUT;
            }
        }
        if (std::chrono::steady_clock::now() >= steady_deadline) {
            return TIME_OUT;
        }

        discard_stale_frontier_entries();
        if (md_.unknown_as_occ && !frontier_queue.empty()) {
            GridNodePtr local_goal{nullptr};
            while (!frontier_queue.empty()) {
                const FrontierEntry entry = frontier_queue.top();
                frontier_queue.pop();
                if (entry.node == nullptr || entry.node->frontier_sequence != entry.sequence) {
                    continue;
                }
                GridNodePtr candidate = entry.node;
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
        planner_context_->error(" -- [A*] Point to point path cannot find path with iter num: {}, return.", num_iter);
        return NO_PATH;
    }

    RET_CODE Astar::escapePathSearch(const rog_map::Vec3f &start_pt, const int flag,
                                     rog_map::vec_Vec3f &out_path,
                                     const bool prefer_start_altitude,
                                     const double time_out) {
        const double effective_time_out = time_out >= 0.0 ? time_out : search_time_limit_s_;
        const auto steady_deadline = std::chrono::steady_clock::now() +
            std::chrono::duration_cast<std::chrono::steady_clock::duration>(
                std::chrono::duration<double>(effective_time_out));
        // setup() records the goal even though escape search only uses the
        // start-centred horizon. Avoid propagating an indeterminate Eigen
        // vector into mission state.
        Vec3f tmp = start_pt;
        RET_CODE setup_ret = setup(start_pt, tmp, flag, 999);
        if (setup_ret != SUCCESS) {
            return setup_ret;
        }

        if (std::chrono::steady_clock::now() >= steady_deadline) {
            return TIME_OUT;
        }
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
        const auto escape_layer = md_.use_inf_map
                ? GridLayer::kInflated : GridLayer::kEvidence;
        const auto escape_policy = md_.unknown_as_occ
                ? UnknownPolicy::kRequireKnownFree : UnknownPolicy::kAllowUnknown;
        const CellState exact_start_type = map_ptr_->classify(
                local_start_pt, escape_layer);
        // UNKNOWN_AS_FREE means an exact non-occupied start already satisfies
        // the escape contract. Snapping it to a voxel centre can otherwise
        // introduce an artificial altitude step on every planning cycle.
        if (md_.unknown_as_free &&
            isCellTraversable(exact_start_type, UnknownPolicy::kAllowUnknown)) {
            out_path.clear();
            return NO_NEED;
        }
        const auto nearest_start = map_ptr_->nearestNotOccupied(
                start_pt, escape_layer, 3.0);
        if (!nearest_start) {
            cout << RED <<
                 " -- [A*] " << RET_CODE_STR[INIT_ERROR]
                 << " : escape start point deeply occupied, cannot find feasible path.\n" << RESET << endl;
            return INIT_ERROR;
        }
        local_start_pt = *nearest_start;

        rog_map::Vec3i start_idx;
        posToGlobalIndex(local_start_pt, start_idx);

        GridNodePtr startPtr = nodeAt(getLocalIndexHash(start_idx));
        using OpenSet = std::priority_queue<OpenSetEntry,
                                            std::vector<OpenSetEntry>,
                                            OpenSetComparator>;
        OpenSet open_set;
        GridNodePtr neighborPtr = NULL;
        GridNodePtr current = NULL;

        startPtr->id_g = start_idx;
        startPtr->rounds = rounds_;
        startPtr->distance_score = 0;
        startPtr->total_score = 0;
        startPtr->state = GridNode::OPENSET; //put start node in open set
        startPtr->father_ptr = NULL;
        open_set.push({startPtr, startPtr->total_score, rounds_}); // put start in open set
        int num_iter = 0;

        vector<GridNodePtr> node_path;

        if (cfg_.visual_process) {
            planner_context_->vizAstarPoints(local_start_pt, Color::Orange(), "local_start_pt",
                                     0.05,
                                     1);
        }
        while (!open_set.empty()) {
            if (std::chrono::steady_clock::now() >= steady_deadline) {
                return TIME_OUT;
            }
            num_iter++;
            const OpenSetEntry current_entry = open_set.top();
            open_set.pop();
            current = current_entry.node;
            if (current_entry.rounds != current->rounds ||
                current_entry.total_score != current->total_score ||
                current->state == GridNode::CLOSEDSET) {
                continue;
            }
            if (cfg_.visual_process) {
                rog_map::Vec3f local_pt;
                globalIndexToPos(current->id_g, local_pt);
                planner_context_->vizAstarPoints(local_pt,
                                         Color::Pink(),
                                         "astar_process",
                                         0.05);
                usleep(1000);
            }
            rog_map::Vec3f cur_pos;
            globalIndexToPos(current->id_g, cur_pos);
            const CellState cur_type = map_ptr_->classify(cur_pos, escape_layer);
            if (md_.unknown_as_occ && cur_type != CellState::kOccupied &&
                cur_type != CellState::kUnknown) {
                retrievePath(current, node_path);
                ConvertNodePathToPointPath(node_path, out_path);
                return REACH_HORIZON;
            }

            if (md_.unknown_as_free &&
                isCellTraversable(cur_type, escape_policy)) {
                retrievePath(current, node_path);
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
                        rog_map::Vec3i neighborIdx;
                        rog_map::Vec3f neighborPos;
                        if (!checkedIndexStep(current->id_g(0), dx, neighborIdx(0)) ||
                            !checkedIndexStep(current->id_g(1), dy, neighborIdx(1)) ||
                            !checkedIndexStep(current->id_g(2), dz, neighborIdx(2))) {
                            continue;
                        }
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

                        if (!isCellTraversable(neighbor_type, UnknownPolicy::kAllowUnknown)) {
                            continue;
                        }

                        if (md_.unknown_as_occ && neighbor_type == CellState::kUnknown) {
                            continue;
                        }

                        neighborPtr = nodeAt(getLocalIndexHash(neighborIdx));
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
                            open_set.push({neighborPtr, neighborPtr->total_score, rounds_});
                        } else if (distance_score < neighborPtr->distance_score) {
                            neighborPtr->father_ptr = current;
                            neighborPtr->distance_score = distance_score;
                            neighborPtr->total_score = distance_score + heu_score;
                            open_set.push({neighborPtr, neighborPtr->total_score, rounds_});
                        }
                    }
            if (std::chrono::steady_clock::now() >= steady_deadline) {
                return TIME_OUT;
            }
        }
        if (std::chrono::steady_clock::now() >= steady_deadline) {
            return TIME_OUT;
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
