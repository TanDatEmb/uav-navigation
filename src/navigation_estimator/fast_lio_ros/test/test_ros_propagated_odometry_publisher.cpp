#include <gtest/gtest.h>

#include <rclcpp/rclcpp.hpp>

#include "fast_lio_ros/ros_propagated_odometry_publisher.hpp"

namespace uav::nav::lio {

class RosPropagatedOdometryPublisherTest : public ::testing::Test {
 protected:
  static void SetUpTestSuite() {
    if (!rclcpp::ok()) {
      int argc = 0;
      rclcpp::init(argc, nullptr);
    }
  }
  static void TearDownTestSuite() { rclcpp::shutdown(); }
};

StateEstimate estimateAt(std::int64_t time_ns) {
  StateEstimate estimate;
  estimate.time = Timestamp(time_ns);
  return estimate;
}

TEST_F(RosPropagatedOdometryPublisherTest, ConvertsAndPublishesOwnedEstimate) {
  rclcpp::Node node("propagated_publisher_test");
  RosParameters parameters;
  parameters.odom_frame = "odom";
  parameters.imu_frame = "livox_imu_frame";
  RosPropagatedOdometryPublisher publisher(node, parameters);
  EXPECT_NO_THROW(publisher.publish(estimateAt(20'000'000)));
}

}  // namespace uav::nav::lio
