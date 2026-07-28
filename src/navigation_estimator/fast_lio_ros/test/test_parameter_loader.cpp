#include <gtest/gtest.h>

#include <rclcpp/rclcpp.hpp>

#include "fast_lio_ros/parameter_loader.hpp"

namespace uav::nav::lio {

class ParameterLoaderTest : public ::testing::Test {
 protected:
  static void SetUpTestSuite() {
    if (!rclcpp::ok()) {
      int argc = 0;
      rclcpp::init(argc, nullptr);
    }
  }
  static void TearDownTestSuite() { rclcpp::shutdown(); }
};

TEST_F(ParameterLoaderTest, LoadsAndValidatesDefaultProductionSchema) {
  rclcpp::Node node{"parameter_loader_test"};
  const auto parameters = ParameterLoader::declareAndLoad(node);
  EXPECT_EQ(parameters.odom_frame, "odom");
  EXPECT_EQ(parameters.lidar_timing_mode, "simultaneous_scan");
  EXPECT_EQ(parameters.input_clock_domain, "ros_time");
  EXPECT_EQ(parameters.livox_timestamp_policy, "require_header_match");
  EXPECT_FALSE(parameters.estimate_extrinsic_online);
  EXPECT_EQ(parameters.minimum_imu_samples, 200);
}

TEST_F(ParameterLoaderTest, RejectsAutoTimingForProductionNode) {
  rclcpp::Node node{"parameter_loader_auto_test"};
  auto parameters = ParameterLoader::declareAndLoad(node);
  parameters.lidar_timing_mode = "auto";
  EXPECT_THROW(ParameterLoader::validate(parameters), std::invalid_argument);
}

TEST_F(ParameterLoaderTest, AcceptsPinnedLivoxCustomBoundary) {
  rclcpp::Node node{"parameter_loader_livox_test"};
  auto parameters = ParameterLoader::declareAndLoad(node);
  parameters.lidar_topic = "/livox/lidar";
  parameters.imu_topic = "/livox/imu";
  parameters.lidar_message_type = "livox_custom";
  parameters.lidar_timing_mode = "per_point";
  parameters.input_clock_domain = "sensor_time";
  parameters.livox_timestamp_policy = "require_header_match";
  EXPECT_NO_THROW(ParameterLoader::validate(parameters));
}

TEST_F(ParameterLoaderTest, RejectsLivoxCustomWithoutPerPointTiming) {
  rclcpp::Node node{"parameter_loader_bad_livox_test"};
  auto parameters = ParameterLoader::declareAndLoad(node);
  parameters.lidar_message_type = "livox_custom";
  parameters.lidar_timing_mode = "simultaneous_scan";
  EXPECT_THROW(ParameterLoader::validate(parameters), std::invalid_argument);
}

}  // namespace uav::nav::lio
