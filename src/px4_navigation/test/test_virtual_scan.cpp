#include <gtest/gtest.h>
#include <px4_nav_common/types.hpp>
#include <px4_navigation/virtual_scan.hpp>

using namespace px4_navigation;
using namespace px4_nav_common;

class VirtualScanTest : public ::testing::Test {
   protected:
    void SetUp() override {
        virtual_scan_ = std::make_unique<VirtualScan>();
        drone_state_.position = Eigen::Vector3d(0.0, 0.0, -5.0);  // 5m above ground
        drone_state_.yaw = 0.0;                                   // Facing north
        drone_state_.velocity = Eigen::Vector3d::Zero();
        drone_state_.valid = true;
    }

    std::unique_ptr<VirtualScan> virtual_scan_;
    DroneStateNed drone_state_;
};

// Test that empty input returns clear scan
TEST_F(VirtualScanTest, EmptyInputReturnsClearScan) {
    std::vector<PointLivox> empty_points;

    virtual_scan_->Update(empty_points, drone_state_);

    const auto& distances = virtual_scan_->ObstacleDistances();

    // All distances should be at max range
    for (const auto& distance : distances) {
        EXPECT_FLOAT_EQ(distance, VirtualScan::kDefaultMaxRange);
    }
}

// Test that single obstacle at known bearing produces expected distance
TEST_F(VirtualScanTest, SingleObstacleAtKnownBearing) {
    std::vector<PointLivox> points;
    PointLivox point;
    point.x = 5.0;  // 5m north of drone
    point.y = 0.0;
    point.z = -5.0;  // Same altitude as drone
    points.push_back(point);

    virtual_scan_->Update(points, drone_state_);

    const auto& distances = virtual_scan_->ObstacleDistances();

    // Find the bin corresponding to 0 degrees (north)
    int north_bin = static_cast<int>((0.0 + M_PI) / virtual_scan_->AngleIncrement());

    // The distance should be 5m (minus vehicle radius for safety)
    EXPECT_NEAR(distances[north_bin], 5.0, 0.1);
}

// Test max range truncation
TEST_F(VirtualScanTest, MaxRangeTruncation) {
    virtual_scan_->Reset(M_PI / 180.0, 10.0, 0.5);  // 10m max range

    std::vector<PointLivox> points;
    PointLivox point;
    point.x = 15.0;  // 15m north of drone (beyond max range)
    point.y = 0.0;
    point.z = -5.0;
    points.push_back(point);

    virtual_scan_->Update(points, drone_state_);

    const auto& distances = virtual_scan_->ObstacleDistances();

    // All distances should be at max range since the point is beyond range
    for (const auto& distance : distances) {
        EXPECT_FLOAT_EQ(distance, 10.0);
    }
}

// Test vehicle radius filtering
TEST_F(VirtualScanTest, VehicleRadiusFiltering) {
    virtual_scan_->Reset(M_PI / 180.0, 15.0, 1.0);  // 1m vehicle radius

    std::vector<PointLivox> points;
    PointLivox point;
    point.x = 0.5;  // 0.5m from drone (within vehicle radius)
    point.y = 0.0;
    point.z = -5.0;
    points.push_back(point);

    virtual_scan_->Update(points, drone_state_);

    const auto& distances = virtual_scan_->ObstacleDistances();

    // All distances should be at max range since the point is within vehicle radius
    for (const auto& distance : distances) {
        EXPECT_FLOAT_EQ(distance, 15.0);
    }
}

// Test height filtering
TEST_F(VirtualScanTest, HeightFiltering) {
    std::vector<PointLivox> points;

    // Point above the drone's height band
    PointLivox point_above;
    point_above.x = 5.0;
    point_above.y = 0.0;
    point_above.z = -8.0;  // 3m above drone (beyond height_above=2.0)
    points.push_back(point_above);

    // Point below the drone's height band
    PointLivox point_below;
    point_below.x = 5.0;
    point_below.y = 0.0;
    point_below.z = -3.5;  // 1.5m below drone (beyond height_below=1.0)
    points.push_back(point_below);

    virtual_scan_->Update(points, drone_state_, 2.0, 1.0);  // height_above=2.0, height_below=1.0

    const auto& distances = virtual_scan_->ObstacleDistances();

    // All distances should be at max range since both points are outside height band
    for (const auto& distance : distances) {
        EXPECT_FLOAT_EQ(distance, VirtualScan::kDefaultMaxRange);
    }
}

// Test angle binning
TEST_F(VirtualScanTest, AngleBinning) {
    std::vector<PointLivox> points;

    // Add points at different angles
    PointLivox point_north;
    point_north.x = 5.0;  // North
    point_north.y = 0.0;
    point_north.z = -5.0;
    points.push_back(point_north);

    PointLivox point_east;
    point_east.x = 0.0;
    point_east.y = 5.0;  // East
    point_east.z = -5.0;
    points.push_back(point_east);

    PointLivox point_south;
    point_south.x = -5.0;  // South
    point_south.y = 0.0;
    point_south.z = -5.0;
    points.push_back(point_south);

    PointLivox point_west;
    point_west.x = 0.0;
    point_west.y = -5.0;  // West
    point_west.z = -5.0;
    points.push_back(point_west);

    virtual_scan_->Update(points, drone_state_);

    const auto& distances = virtual_scan_->ObstacleDistances();
    double angle_increment = virtual_scan_->AngleIncrement();

    // Check bins for cardinal directions
    int north_bin = static_cast<int>((0.0 + M_PI) / angle_increment);
    int east_bin = static_cast<int>((M_PI / 2.0 + M_PI) / angle_increment);
    int south_bin = static_cast<int>((M_PI + M_PI) / angle_increment);
    if (south_bin >= VirtualScan::kNumBins)
        south_bin = VirtualScan::kNumBins - 1;
    int west_bin = static_cast<int>((-M_PI / 2.0 + M_PI) / angle_increment);

    // All should be approximately 5m
    EXPECT_NEAR(distances[north_bin], 5.0, 0.1);
    EXPECT_NEAR(distances[east_bin], 5.0, 0.1);
    EXPECT_NEAR(distances[south_bin], 5.0, 0.1);
    EXPECT_NEAR(distances[west_bin], 5.0, 0.1);
}
