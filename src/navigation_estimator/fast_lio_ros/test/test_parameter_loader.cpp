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
  EXPECT_FALSE(parameters.estimate_extrinsic_online);
  EXPECT_EQ(parameters.minimum_imu_samples, 200);
}

TEST_F(ParameterLoaderTest, RejectsAutoTimingForProductionNode) {
  RosParameters parameters;
  parameters.odom_frame = "odom";
  parameters.base_frame = "base_link";
  parameters.imu_frame = "imu_link";
  parameters.lidar_frame = "lidar_link";
  parameters.lidar_message_type = "pointcloud2";
  parameters.lidar_timing_mode = "auto";
  EXPECT_THROW(ParameterLoader::validate(parameters), std::invalid_argument);
}

}  // namespace uav::nav::lio
