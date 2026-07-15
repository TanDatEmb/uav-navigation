#ifndef PX4_NAVIGATION_A_STAR_PLANNER_HPP_
#define PX4_NAVIGATION_A_STAR_PLANNER_HPP_

#include <Eigen/Dense>
#include <chrono>
#include <queue>
#include <string>
#include <unordered_map>
#include <vector>

#include <px4_nav_common/types.hpp>
#include <px4_navigation/local_plan_grid.hpp>

namespace px4_navigation {

/**
 * @brief 3D A* path planner for local navigation in PX4 NED frame.
 *
 * This class implements a 3D A* path planner that operates on a LocalPlanGrid
 * to find collision-free paths between start and goal positions. The planner
 * uses 26-connectivity for neighbor expansion and includes several penalties
 * to encourage smooth, efficient paths:
 *
 * - Z-axis penalty discourages altitude changes
 * - Direction change penalty discourages erratic paths
 * - Planning horizon caps search radius for real-time replanning
 * - Goal safety margin adjusts goal position when blocked
 *
 * Coordinate Frame: NED (North-East-Down)
 * - X axis: North (positive) / South (negative)
 * - Y axis: East (positive) / West (negative)
 * - Z axis: Down (positive) / Up (negative)
 */
class AStarPlanner {
   public:
    /// Default constructor
    AStarPlanner() = default;

    /// Destructor
    ~AStarPlanner() = default;

    /**
     * @brief Plan result containing the computed path and metadata.
     */
    struct PlanResult {
        /// Path as a vector of waypoints in NED frame (metres)
        std::vector<Eigen::Vector3d> path;

        /// Number of iterations performed during search
        int iterations{0};

        /// Whether a valid path was found
        bool success{false};

        /// Whether the search timed out before finding a solution
        bool timed_out{false};

        /// Debug information about the planning process
        std::string debug_info;
    };

    /**
     * @brief Compute a path from start to goal using 3D A* search.
     *
     * @param grid LocalPlanGrid containing occupancy information
     * @param start Start state in NED frame
     * @param goal Goal position in NED frame
     * @return PlanResult containing the computed path or failure information
     */
    PlanResult Plan(const LocalPlanGrid& grid, const px4_nav_common::DroneStateNed& start,
                    const px4_nav_common::WaypointNed& goal);

    /// Maximum search distance from start position (metres)
    inline static constexpr double kPlanningHorizonM = 20.0;

    /// Safety margin to pull goal back when blocked (metres)
    inline static constexpr double kGoalSafetyMarginM = 1.5;

    /// Maximum number of search iterations before giving up
    inline static constexpr int kMaxIterations = 30000;

    /// Wall-clock time budget for planning (milliseconds)
    inline static constexpr int kWallClockBudgetMs = 50;

    /// Check wall-clock budget every N iterations
    inline static constexpr int kBudgetCheckInterval = 1000;

   private:
    /**
     * @brief Single A* search node, allocated from pre-allocated pool.
     */
    struct Node {
        /// Grid index of this node
        Eigen::Vector3i idx;

        /// Cost from start to this node
        double g{0.0};

        /// Heuristic cost from this node to goal
        double h{0.0};

        /// Parent node in the path
        Node* parent{nullptr};

        /// Total cost (g + h)
        double f() const {
            return g + h;
        }
    };

    /**
     * @brief Ordering comparator for A* open set priority queue.
     */
    struct NodeComparator {
        bool operator()(const Node* lhs, const Node* rhs) const {
            return lhs->f() > rhs->f();
        }
    };

    /// Pre-allocated node pool to avoid heap allocations during search
    std::vector<Node> node_pool_;

    /// Best g-value for each visited node to avoid redundant expansions
    std::unordered_map<Eigen::Vector3i, double, px4_nav_common::VoxelHash> best_g_;

    /// Current index in node pool
    size_t pool_idx_{0};

    /// Penalty for Z-axis movement to discourage altitude changes
    inline static constexpr double kZPenalty = 0.5;

    /// Penalty for direction changes to encourage smooth paths
    inline static constexpr double kDirChangePenalty = 0.3;

    /**
     * @brief Convert world coordinates to grid indices.
     *
     * @param grid LocalPlanGrid to use for conversion
     * @param world World position in NED frame (metres)
     * @return Grid index corresponding to world position
     */
    Eigen::Vector3i WorldToGrid(const LocalPlanGrid& grid, const Eigen::Vector3d& world) const;

    /**
     * @brief Convert grid indices to world coordinates.
     *
     * @param grid LocalPlanGrid to use for conversion
     * @param idx Grid index
     * @return World position in NED frame (metres)
     */
    Eigen::Vector3d GridToWorld(const LocalPlanGrid& grid, const Eigen::Vector3i& idx) const;

    /**
     * @brief Check if a grid cell is blocked.
     *
     * @param grid LocalPlanGrid to query
     * @param idx Grid index to check
     * @return true if cell is blocked or out of bounds, false if free
     */
    bool IsBlocked(const LocalPlanGrid& grid, const Eigen::Vector3i& idx) const;

    /**
     * @brief Adjust start position if it's inside an obstacle.
     *
     * @param grid LocalPlanGrid to query
     * @param start_idx Reference to start grid index (may be modified)
     * @param debug Debug string stream to append information
     * @return true if a valid start position was found, false otherwise
     */
    bool AdjustStartIfBlocked(const LocalPlanGrid& grid, Eigen::Vector3i& start_idx,
                              std::ostringstream& debug) const;

    /**
     * @brief Adjust goal position if it's inside an obstacle.
     *
     * @param grid LocalPlanGrid to query
     * @param start_idx Start grid index for direction calculation
     * @param goal_idx Reference to goal grid index (may be modified)
     * @param debug Debug string stream to append information
     * @return true if a valid goal position was found, false otherwise
     */
    bool AdjustGoalIfBlocked(const LocalPlanGrid& grid, const Eigen::Vector3i& start_idx,
                             Eigen::Vector3i& goal_idx, std::ostringstream& debug) const;

    /**
     * @brief Create a new node from pool with specified parameters.
     *
     * @param idx Grid index for the node
     * @param g Cost from start to this node
     * @param parent Parent node in the path
     * @param goal_idx Goal grid index for heuristic calculation
     * @return Pointer to created node, or nullptr if pool exhausted
     */
    Node* CreateNode(const Eigen::Vector3i& idx, double g, Node* parent,
                     const Eigen::Vector3i& goal_idx);

    /**
     * @brief Apply smoothing to the computed path.
     *
     * @param path Reference to path vector to smooth (modified in place)
     */
    void SmoothPath(std::vector<Eigen::Vector3d>& path) const;
};

}  // namespace px4_navigation

#endif  // PX4_NAVIGATION_A_STAR_PLANNER_HPP_