#include <px4_navigation/local_plan_grid.hpp>

#include <gtest/gtest.h>
#include <Eigen/Dense>

namespace {

using px4_navigation::LocalPlanGrid;

// Test fixture for LocalPlanGrid tests
class LocalPlanGridTest : public ::testing::Test {
   protected:
    void SetUp() override {
        // Set up a small grid for testing
        grid_size_ = Eigen::Vector3i(10, 10, 5);           // 10x10x5 voxels
        grid_origin_ = Eigen::Vector3d(-5.0, -5.0, -2.0);  // Centered around origin
        grid_resolution_ = 1.0;                            // 1m per voxel

        // Reset the grid
        grid_.reset(grid_size_, grid_origin_, grid_resolution_);
    }

    LocalPlanGrid grid_;
    Eigen::Vector3i grid_size_;
    Eigen::Vector3d grid_origin_;
    double grid_resolution_;
};

// Test grid reset functionality
TEST_F(LocalPlanGridTest, GridReset) {
    // Check that grid is properly initialized
    EXPECT_EQ(grid_.size().x(), grid_size_.x());
    EXPECT_EQ(grid_.size().y(), grid_size_.y());
    EXPECT_EQ(grid_.size().z(), grid_size_.z());
    EXPECT_EQ(grid_.origin().x(), grid_origin_.x());
    EXPECT_EQ(grid_.origin().y(), grid_origin_.y());
    EXPECT_EQ(grid_.origin().z(), grid_origin_.z());
    EXPECT_EQ(grid_.resolution(), grid_resolution_);
}

// Test marking occupied voxels
TEST_F(LocalPlanGridTest, MarkOccupied) {
    // Mark a point at the center of the grid
    const Eigen::Vector3d test_point(0.0, 0.0, 0.0);
    grid_.markOccupied(test_point);

    // Check that the point is marked as occupied
    EXPECT_TRUE(grid_.isOccupied(0.0, 0.0, 0.0));
    EXPECT_FALSE(grid_.isFree(0.0, 0.0, 0.0));
}

// Test marking free voxels
TEST_F(LocalPlanGridTest, MarkFree) {
    // Mark a point as occupied first
    const Eigen::Vector3d test_point(0.0, 0.0, 0.0);
    grid_.markOccupied(test_point);
    EXPECT_TRUE(grid_.isOccupied(0.0, 0.0, 0.0));

    // Then mark it as free
    grid_.markFree(test_point);
    EXPECT_TRUE(grid_.isFree(0.0, 0.0, 0.0));
    EXPECT_FALSE(grid_.isOccupied(0.0, 0.0, 0.0));
}

// Test obstacle inflation
TEST_F(LocalPlanGridTest, InflateObstacles) {
    // Mark a point at the center
    const Eigen::Vector3d center_point(0.0, 0.0, 0.0);
    grid_.markOccupied(center_point);

    // Inflate with radius of 1 voxel
    grid_.inflateObstacles(1);

    // Check that the center point is occupied
    EXPECT_TRUE(grid_.isOccupied(0.0, 0.0, 0.0));

    // Check that neighboring points in XY plane are also occupied (inflated)
    EXPECT_TRUE(grid_.isOccupied(1.0, 0.0, 0.0));   // Right
    EXPECT_TRUE(grid_.isOccupied(-1.0, 0.0, 0.0));  // Left
    EXPECT_TRUE(grid_.isOccupied(0.0, 1.0, 0.0));   // Forward
    EXPECT_TRUE(grid_.isOccupied(0.0, -1.0, 0.0));  // Backward
    // With square inflation, diagonal points should also be occupied with radius 1
    EXPECT_TRUE(grid_.isOccupied(1.0, 1.0, 0.0));    // Diagonal
    EXPECT_TRUE(grid_.isOccupied(-1.0, 1.0, 0.0));   // Diagonal
    EXPECT_TRUE(grid_.isOccupied(1.0, -1.0, 0.0));   // Diagonal
    EXPECT_TRUE(grid_.isOccupied(-1.0, -1.0, 0.0));  // Diagonal
}

// Test world to index round trip conversion
TEST_F(LocalPlanGridTest, WorldToIndexRoundTrip) {
    // Test a point at the boundary of the grid
    const Eigen::Vector3d boundary_point(4.5, 4.5, 2.5);  // Near upper bounds

    // Convert to index
    const Eigen::Vector3i index =
        px4_common::math::WorldToIndex(boundary_point, grid_.origin(), grid_.resolution());

    // Convert back to world position
    const Eigen::Vector3d converted_position =
        px4_common::math::IndexToWorld(index, grid_.origin(), grid_.resolution());

    // Check that the conversion is consistent (within half voxel resolution)
    const double tolerance = grid_.resolution() / 2.0;
    EXPECT_NEAR(converted_position.x(), boundary_point.x(), tolerance);
    EXPECT_NEAR(converted_position.y(), boundary_point.y(), tolerance);
    EXPECT_NEAR(converted_position.z(), boundary_point.z(), tolerance);
}

// Test out of bounds behavior
TEST_F(LocalPlanGridTest, OutOfBounds) {
    // Test a point outside the grid
    const Eigen::Vector3d out_of_bounds_point(10.0, 10.0, 10.0);

    // Out of bounds should be considered occupied for safety
    EXPECT_TRUE(grid_.isOccupied(10.0, 10.0, 10.0));
    EXPECT_FALSE(grid_.isFree(10.0, 10.0, 10.0));
}

}  // namespace

int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}