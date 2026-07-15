#include <gtest/gtest.h>

#include <array>
#include <cmath>
#include <limits>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include <px4_navigation/obstacle_perception.hpp>

namespace px4_navigation {

class ObstaclePerceptionTestAccess {
   public:
    static void ConfigureSensorFrame(ObstaclePerception& node) {
        node.cloud_frame_ = "sensor";
        node.filter_ground_points_ = false;
    }

    static std::array<uint16_t, 72> Process(ObstaclePerception& node,
                                            const std::vector<Eigen::Vector3f>& points) {
        node.BuildSphericalGrid(points);
        std::array<uint16_t, 72> distances{};
        node.ComputeMinDistances(distances);
        return distances;
    }

    static uint16_t ClearDistanceCm(const ObstaclePerception& node) {
        return node.ClearDistanceCm();
    }

    static int YawBins(const ObstaclePerception& node) {
        return node.yaw_bins_;
    }
};

class ObstaclePerceptionTest : public ::testing::Test {
   protected:
    static void SetUpTestSuite() {
        if (!rclcpp::ok()) {
            int argc = 0;
            char** argv = nullptr;
            rclcpp::init(argc, argv);
        }
    }

    static void TearDownTestSuite() {
        if (rclcpp::ok()) {
            rclcpp::shutdown();
        }
    }

    static rclcpp::NodeOptions Options() {
        rclcpp::NodeOptions options;
        options.append_parameter_override("use_sim_time", false);
        options.append_parameter_override("publish_local_virtual_scan", false);
        options.append_parameter_override("cloud_frame", "sensor");
        options.append_parameter_override("filter_ground_points", false);
        return options;
    }

    static Eigen::Vector3f PointAt(float range_m, float yaw_deg) {
        constexpr float kDegToRad = 3.14159265358979323846f / 180.0f;
        const float yaw_rad = yaw_deg * kDegToRad;
        return {range_m * std::cos(yaw_rad), range_m * std::sin(yaw_rad), 0.0f};
    }
};

TEST_F(ObstaclePerceptionTest, FreshObservedBinsWithoutReturnsAreMeasuredClear) {
    ObstaclePerception node(Options());
    ObstaclePerceptionTestAccess::ConfigureSensorFrame(node);

    const float nan = std::numeric_limits<float>::quiet_NaN();
    const auto distances =
        ObstaclePerceptionTestAccess::Process(node, {Eigen::Vector3f(nan, nan, nan)});

    EXPECT_EQ(ObstaclePerceptionTestAccess::ClearDistanceCm(node), 4001U);
    for (const uint16_t distance : distances) {
        EXPECT_EQ(distance, 4001U);
    }
    EXPECT_EQ(ObstaclePerception::kNoObstacle, std::numeric_limits<uint16_t>::max());
}

TEST_F(ObstaclePerceptionTest, ForwardPointUsesBinZeroAndLeavesOtherBinsClear) {
    ObstaclePerception node(Options());
    ObstaclePerceptionTestAccess::ConfigureSensorFrame(node);

    const auto distances =
        ObstaclePerceptionTestAccess::Process(node, {Eigen::Vector3f(1.0f, 0.0f, 0.0f)});

    EXPECT_EQ(distances[0], 100U);
    for (size_t i = 1; i < distances.size(); ++i) {
        EXPECT_EQ(distances[i], 4001U);
    }
}

TEST_F(ObstaclePerceptionTest, YawBinningUsesNearestPx4BinCenter) {
    ObstaclePerception node(Options());
    ObstaclePerceptionTestAccess::ConfigureSensorFrame(node);

    const auto below_boundary = ObstaclePerceptionTestAccess::Process(node, {PointAt(1.0f, 2.49f)});
    EXPECT_EQ(below_boundary[0], 100U);
    EXPECT_EQ(below_boundary[1], 4001U);

    const auto above_boundary = ObstaclePerceptionTestAccess::Process(node, {PointAt(1.0f, 2.51f)});
    EXPECT_EQ(above_boundary[0], 4001U);
    EXPECT_EQ(above_boundary[1], 100U);

    const auto wrap_boundary = ObstaclePerceptionTestAccess::Process(node, {PointAt(1.0f, -2.51f)});
    EXPECT_EQ(wrap_boundary[71], 100U);
}

TEST_F(ObstaclePerceptionTest, InclusiveBodyExclusionRemovesSyntheticRotorReturns) {
    ObstaclePerception node(Options());
    ObstaclePerceptionTestAccess::ConfigureSensorFrame(node);

    const std::vector<Eigen::Vector3f> rotor_returns{PointAt(0.37f, 60.0f), PointAt(0.37f, 115.0f),
                                                     PointAt(0.37f, 240.0f), PointAt(0.37f, 295.0f),
                                                     PointAt(0.50f, 0.0f)};
    const auto excluded = ObstaclePerceptionTestAccess::Process(node, rotor_returns);
    for (const uint16_t distance : excluded) {
        EXPECT_EQ(distance, 4001U);
    }

    const auto outside_boundary =
        ObstaclePerceptionTestAccess::Process(node, {PointAt(0.51f, 60.0f)});
    EXPECT_EQ(outside_boundary[12], 51U);
}

TEST_F(ObstaclePerceptionTest, InvalidYawBinCountFallsBackToPx4Contract) {
    auto options = Options();
    options.append_parameter_override("yaw_bins", 36);
    ObstaclePerception node(options);

    EXPECT_EQ(ObstaclePerceptionTestAccess::YawBins(node), 72);
}

}  // namespace px4_navigation
