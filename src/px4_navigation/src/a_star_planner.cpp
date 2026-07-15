#include <px4_nav_common/math/grid.hpp>
#include <px4_navigation/a_star_planner.hpp>

#include <algorithm>
#include <cmath>
#include <limits>

namespace px4_navigation {

AStarPlanner::PlanResult AStarPlanner::Plan(const LocalPlanGrid& grid,
                                            const px4_nav_common::DroneStateNed& start,
                                            const px4_nav_common::WaypointNed& goal) {
    PlanResult result;
    std::ostringstream debug;

    // Convert start and goal positions to grid indices
    Eigen::Vector3i start_idx = WorldToGrid(grid, start.position);
    Eigen::Vector3i goal_idx = WorldToGrid(grid, goal.position);

    debug << "A*3D: start(" << start_idx.transpose() << ") -> goal(" << goal_idx.transpose() << ")";

    // Adjust start and goal positions if they're blocked
    if (!AdjustStartIfBlocked(grid, start_idx, debug)) {
        debug << " | FAILED: No free start";
        result.debug_info = debug.str();
        return result;
    }

    if (!AdjustGoalIfBlocked(grid, start_idx, goal_idx, debug)) {
        debug << " | FAILED: No free goal";
        result.debug_info = debug.str();
        return result;
    }

    // Pre-allocate node pool. Each expanded node can push up to 26 neighbors,
    // so reserve enough capacity for the worst-case branch factor over the
    // iteration budget plus a small margin.
    node_pool_.resize(26 * kMaxIterations + 100);
    pool_idx_ = 0;

    // Initialize open set priority queue and best_g map
    std::priority_queue<Node*, std::vector<Node*>, NodeComparator> open;
    best_g_.clear();

    // Create start node and add to open set
    Node* start_node = CreateNode(start_idx, 0.0, nullptr, goal_idx);
    if (!start_node) {
        result.debug_info = "Pool exhausted at start";
        return result;
    }
    open.push(start_node);
    best_g_[start_idx] = 0.0;

    Node* final_node = nullptr;

    // Calculate planning parameters
    const double voxel_res = grid.Resolution();
    const double inv_res = 1.0 / voxel_res;
    const double horizon_voxels = kPlanningHorizonM * inv_res;
    const double horizon_voxels_sq = horizon_voxels * horizon_voxels;

    // Set wall-clock budget deadline
    const auto plan_start = std::chrono::steady_clock::now();
    const auto plan_deadline = plan_start + std::chrono::milliseconds(kWallClockBudgetMs);

    // Main A* search loop
    while (!open.empty() && result.iterations < kMaxIterations) {
        result.iterations++;

        // Check wall-clock budget periodically
        if (result.iterations % kBudgetCheckInterval == 0 &&
            std::chrono::steady_clock::now() > plan_deadline) {
            result.timed_out = true;
            debug << " | TIMEOUT after " << result.iterations << " iter";
            break;
        }

        // Get node with lowest f-score
        Node* current = open.top();
        open.pop();

        // Skip if we've already found a strictly better path to this node.
        // Stale queue entries from sub-optimal pushes are dropped here.
        auto bg = best_g_.find(current->idx);
        if (bg != best_g_.end() && current->g > bg->second + 1e-9) {
            continue;
        }

        // Check if we've reached the goal
        if (current->idx == goal_idx) {
            final_node = current;
            break;
        }

        // Expand neighbors in 26-connectivity
        for (int dz = -1; dz <= 1; ++dz) {
            for (int dy = -1; dy <= 1; ++dy) {
                for (int dx = -1; dx <= 1; ++dx) {
                    // Skip self
                    if (dx == 0 && dy == 0 && dz == 0) {
                        continue;
                    }

                    Eigen::Vector3i neighbor_idx = current->idx + Eigen::Vector3i(dx, dy, dz);

                    // Check if neighbor is within planning horizon
                    Eigen::Vector3i diff = neighbor_idx - start_idx;
                    double dist_sq = diff.cast<double>().squaredNorm();
                    if (dist_sq > horizon_voxels_sq) {
                        continue;
                    }

                    // Check if neighbor is blocked
                    if (IsBlocked(grid, neighbor_idx)) {
                        continue;
                    }

                    // Calculate movement cost
                    double base_cost = std::sqrt(static_cast<double>(dx * dx + dy * dy + dz * dz));
                    double z_cost = kZPenalty * std::abs(dz);

                    double dir_cost = 0.0;
                    if (current->parent) {
                        Eigen::Vector3i prev_dir = current->idx - current->parent->idx;
                        Eigen::Vector3i curr_dir(dx, dy, dz);
                        if (prev_dir.dot(curr_dir) < 0) {
                            dir_cost = kDirChangePenalty;
                        }
                    }

                    double new_g = current->g + base_cost + z_cost + dir_cost;

                    // Check if this is a better path to the neighbor
                    auto existing = best_g_.find(neighbor_idx);
                    if (existing != best_g_.end() && new_g >= existing->second) {
                        continue;
                    }

                    // Update best g-value and create new node
                    best_g_[neighbor_idx] = new_g;
                    Node* neighbor_node = CreateNode(neighbor_idx, new_g, current, goal_idx);
                    if (!neighbor_node) {
                        result.debug_info = debug.str() + " | Pool exhausted at iteration " +
                                            std::to_string(result.iterations);
                        result.timed_out = true;
                        return result;
                    }
                    open.push(neighbor_node);
                }
            }
        }
    }

    debug << " | Iterations: " << result.iterations;

    // Reconstruct path if goal was reached
    if (final_node) {
        Node* current = final_node;
        while (current) {
            result.path.push_back(GridToWorld(grid, current->idx));
            current = current->parent;
        }
        std::reverse(result.path.begin(), result.path.end());

        // Apply path smoothing
        SmoothPath(result.path);

        result.success = true;
        debug << " | Path found: " << result.path.size() << " points";
    } else {
        debug << " | FAILED: No path found";
    }

    result.debug_info = debug.str();
    return result;
}

Eigen::Vector3i AStarPlanner::WorldToGrid(const LocalPlanGrid& grid,
                                          const Eigen::Vector3d& world) const {
    return px4_nav_common::math::WorldToIndex(world, grid.Origin(), grid.Resolution());
}

Eigen::Vector3d AStarPlanner::GridToWorld(const LocalPlanGrid& grid,
                                          const Eigen::Vector3i& idx) const {
    return px4_nav_common::math::IndexToWorld(idx, grid.Origin(), grid.Resolution());
}

bool AStarPlanner::IsBlocked(const LocalPlanGrid& grid, const Eigen::Vector3i& idx) const {
    const Eigen::Vector3d world_pos =
        px4_nav_common::math::IndexToWorld(idx, grid.Origin(), grid.Resolution());
    return !grid.IsFree(world_pos.x(), world_pos.y(), world_pos.z());
}

bool AStarPlanner::AdjustStartIfBlocked(const LocalPlanGrid& grid, Eigen::Vector3i& start_idx,
                                        std::ostringstream& debug) const {
    // Check if start position is blocked
    if (IsBlocked(grid, start_idx)) {
        // Collect all nearby free positions and choose the closest one to the
        // original start so that the adjusted start minimizes position error.
        std::vector<std::pair<Eigen::Vector3i, int>> candidates;
        constexpr int kMaxSearchRadius = 15;
        for (int dz = -kMaxSearchRadius; dz <= kMaxSearchRadius; ++dz) {
            for (int dy = -kMaxSearchRadius; dy <= kMaxSearchRadius; ++dy) {
                for (int dx = -kMaxSearchRadius; dx <= kMaxSearchRadius; ++dx) {
                    if (dx == 0 && dy == 0 && dz == 0) {
                        continue;
                    }
                    const Eigen::Vector3i nbr = start_idx + Eigen::Vector3i(dx, dy, dz);
                    if (!IsBlocked(grid, nbr)) {
                        const int dist_sq = dx * dx + dy * dy + dz * dz;
                        candidates.emplace_back(nbr, dist_sq);
                    }
                }
            }
        }

        if (!candidates.empty()) {
            std::sort(candidates.begin(), candidates.end(), [](const auto& a, const auto& b) {
                return a.second < b.second;
            });
            start_idx = candidates.front().first;
            debug << " | Start adjusted";
            return true;
        }
        return false;
    }
    return true;
}

bool AStarPlanner::AdjustGoalIfBlocked(const LocalPlanGrid& grid, const Eigen::Vector3i& start_idx,
                                       Eigen::Vector3i& goal_idx, std::ostringstream& debug) const {
    // Check if goal position is blocked
    if (IsBlocked(grid, goal_idx)) {
        // Calculate direction from start to goal
        Eigen::Vector3d dir_to_goal = (goal_idx - start_idx).cast<double>();
        double dir_norm = dir_to_goal.norm();
        if (dir_norm > 0.1) {
            dir_to_goal /= dir_norm;
        }

        // Pull back along approach direction
        const double voxel_res = grid.Resolution();
        int margin_voxels = static_cast<int>(kGoalSafetyMarginM / voxel_res);
        bool found = false;

        // Try pulling back along the approach direction in voxel steps
        for (int step = 1; step <= margin_voxels * 2; ++step) {
            Eigen::Vector3i pulled = goal_idx;
            for (int component = 0; component < 3; ++component) {
                pulled(component) -= static_cast<int>(std::round(dir_to_goal(component) * step));
            }

            if (!IsBlocked(grid, pulled)) {
                goal_idx = pulled;
                debug << " | Goal pulled back " << step << " voxels";
                found = true;
                break;
            }
        }

        // Fallback: directional spiral search
        if (!found) {
            for (int r = 1; r <= 20 && !found; ++r) {
                for (int dz = -r; dz <= r && !found; ++dz) {
                    for (int dy = -r; dy <= r && !found; ++dy) {
                        for (int dx = -r; dx <= r && !found; ++dx) {
                            Eigen::Vector3i nbr = goal_idx + Eigen::Vector3i(dx, dy, dz);
                            if (!IsBlocked(grid, nbr)) {
                                Eigen::Vector3d offset(dx, dy, dz);
                                if (offset.normalized().dot(dir_to_goal) < -0.3) {
                                    continue;
                                }
                                goal_idx = nbr;
                                found = true;
                                debug << " | Goal adjusted";
                            }
                        }
                    }
                }
            }
        }

        return found;
    }
    return true;
}

AStarPlanner::Node* AStarPlanner::CreateNode(const Eigen::Vector3i& idx, double g, Node* parent,
                                             const Eigen::Vector3i& goal_idx) {
    if (pool_idx_ >= node_pool_.size()) {
        return nullptr;
    }
    Node* node = &node_pool_[pool_idx_++];
    node->idx = idx;
    node->g = g;
    node->parent = parent;
    // Calculate heuristic as Euclidean distance to goal
    node->h = (idx - goal_idx).cast<double>().norm();
    return node;
}

void AStarPlanner::SmoothPath(std::vector<Eigen::Vector3d>& path) const {
    // Apply 3-pass moving average smoothing
    for (int pass = 0; pass < 3; ++pass) {
        if (path.size() < 3) {
            break;
        }
        std::vector<Eigen::Vector3d> smoothed = path;
        for (size_t i = 1; i < path.size() - 1; ++i) {
            smoothed[i] = (path[i - 1] + path[i] + path[i + 1]) / 3.0;
        }
        path = smoothed;
    }
}

}  // namespace px4_navigation