#include <gtest/gtest.h>
#include <Eigen/Dense>
#include <cmath>
#include <px4_navigation/a_star_planner.hpp>
#include <px4_navigation/local_plan_grid.hpp>
#include <vector>

using namespace px4_navigation;

class AStarPlannerTest : public ::testing::Test {
   protected:
    void SetUp() override {
        // Initialize grid with 1m resolution
        grid_.Reset(Eigen::Vector3i(20, 20, 10), Eigen::Vector3d(-10.0, -10.0, -5.0), 1.0);
        // Inflate obstacles with radius of 1 voxel
        grid_.InflateObstacles(1);

        // Initialize start and goal positions
        start_.position = Eigen::Vector3d(0.0, 0.0, 0.0);
        start_.yaw = 0.0;
        start_.velocity = Eigen::Vector3d::Zero();
        start_.valid = true;

        goal_.position = Eigen::Vector3d(8.0, 8.0, 0.0);
        goal_.lat = 0.0;
        goal_.lon = 0.0;
        goal_.alt = 0.0;
        goal_.valid = true;
    }

    LocalPlanGrid grid_;
    px4_common::DroneStateNed start_;
    px4_common::WaypointNed goal_;
    AStarPlanner planner_;
};

// Test that planner finds a straight path in an empty grid
TEST_F(AStarPlannerTest, EmptyGridStraightPath) {
    // Plan path in empty grid
    auto result = planner_.Plan(grid_, start_, goal_);

    // Should succeed and find a path
    EXPECT_TRUE(result.success);
    EXPECT_FALSE(result.path.empty());
    EXPECT_FALSE(result.timed_out);

    // Path should have reasonable length (at least start and goal)
    EXPECT_GE(result.path.size(), 2);

    // Start and goal should be close to requested positions
    EXPECT_NEAR(result.path.front().x(), 0.0, 1.0);  // Within 1m of start
    EXPECT_NEAR(result.path.front().y(), 0.0, 1.0);
    EXPECT_NEAR(result.path.back().x(), 8.0, 1.0);  // Within 1m of goal
    EXPECT_NEAR(result.path.back().y(), 8.0, 1.0);

    // Path should be roughly diagonal (since no obstacles)
    EXPECT_LT(std::abs(result.path.back().x() - result.path.front().x() - 8.0), 2.0);
    EXPECT_LT(std::abs(result.path.back().y() - result.path.front().y() - 8.0), 2.0);
}

// Test that planner returns empty path for fully blocked grid
TEST_F(AStarPlannerTest, FullyBlockedGrid) {
    // Block the entire grid
    for (int x = -9; x <= 9; x++) {
        for (int y = -9; y <= 9; y++) {
            for (int z = -4; z <= 4; z++) {
                grid_.MarkOccupied(Eigen::Vector3d(x, y, z));
            }
        }
    }
    grid_.InflateObstacles(1);  // Inflate obstacles

    // Plan path in fully blocked grid
    auto result = planner_.Plan(grid_, start_, goal_);

    // Should not succeed but also not time out
    EXPECT_FALSE(result.success);
    EXPECT_FALSE(result.timed_out);
    // Path should be empty
    EXPECT_TRUE(result.path.empty());
}

// Test that planner finds a detour around a single obstacle
TEST_F(AStarPlannerTest, SingleObstacleDetour) {
    // Add a single obstacle in the direct path
    grid_.MarkOccupied(Eigen::Vector3d(4.0, 4.0, 0.0));
    grid_.InflateObstacles(1);  // Inflate obstacles

    // Plan path
    auto result = planner_.Plan(grid_, start_, goal_);

    // Should succeed
    EXPECT_TRUE(result.success);
    EXPECT_FALSE(result.path.empty());

    // Path should be longer than direct distance due to detour
    double direct_distance = std::sqrt(8.0 * 8.0 + 8.0 * 8.0);  // ~11.3m
    double path_distance = 0.0;
    for (size_t i = 1; i < result.path.size(); ++i) {
        path_distance += (result.path[i] - result.path[i - 1]).norm();
    }

    // Path should be longer than direct distance (detour)
    EXPECT_GT(path_distance, direct_distance);

    // But not excessively long (should still be reasonable)
    EXPECT_LT(path_distance, direct_distance * 3.0);
}

// Test that path stays within grid bounds
TEST_F(AStarPlannerTest, PathStaysInBounds) {
    // Plan path in empty grid
    auto result = planner_.Plan(grid_, start_, goal_);

    // Should succeed
    EXPECT_TRUE(result.success);
    EXPECT_FALSE(result.path.empty());

    // All path points should be within grid bounds
    const Eigen::Vector3d& origin = grid_.Origin();
    const Eigen::Vector3i& size = grid_.Size();
    const double resolution = grid_.Resolution();

    double min_x = origin.x();
    double max_x = origin.x() + size.x() * resolution;
    double min_y = origin.y();
    double max_y = origin.y() + size.y() * resolution;
    double min_z = origin.z();
    double max_z = origin.z() + size.z() * resolution;

    for (const auto& point : result.path) {
        EXPECT_GE(point.x(), min_x - 0.5);  // Allow slight margin
        EXPECT_LE(point.x(), max_x + 0.5);
        EXPECT_GE(point.y(), min_y - 0.5);
        EXPECT_LE(point.y(), max_y + 0.5);
        EXPECT_GE(point.z(), min_z - 0.5);
        EXPECT_LE(point.z(), max_z + 0.5);
    }
}

// Test that planner handles start position inside obstacle
TEST_F(AStarPlannerTest, StartInObstacle) {
    // Block start position
    grid_.MarkOccupied(start_.position);
    grid_.InflateObstacles(1);  // Inflate obstacles

    // Plan path
    auto result = planner_.Plan(grid_, start_, goal_);

    // Should succeed (planner should adjust start position)
    EXPECT_TRUE(result.success);
    EXPECT_FALSE(result.path.empty());

    // Start of path should be different from original start position
    // (since original was blocked)
    EXPECT_FALSE((result.path.front() - start_.position).isZero(0.1));
}

// Test that planner handles goal position inside obstacle
TEST_F(AStarPlannerTest, GoalInObstacle) {
    // Block goal position
    grid_.MarkOccupied(goal_.position);
    grid_.InflateObstacles(1);  // Inflate obstacles

    // Plan path
    auto result = planner_.Plan(grid_, start_, goal_);

    // Should succeed (planner should adjust goal position)
    EXPECT_TRUE(result.success);
    EXPECT_FALSE(result.path.empty());

    // End of path should be different from original goal position
    // (since original was blocked)
    EXPECT_FALSE((result.path.back() - goal_.position).isZero(0.1));
}
