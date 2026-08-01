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

TEST_F(RosPropagatedOdometryPublisherTest,
       UsesActualImuTimestampsAndSkipsMissedDeadlines) {
  rclcpp::Node node("propagated_publisher_test");
  RosParameters parameters;
  parameters.odom_frame = "odom";
  parameters.imu_frame = "imu_link";
  parameters.propagated_odometry_publish_rate_hz = 50.0;
  RosPropagatedOdometryPublisher publisher(node, parameters);

  publisher.onImuEstimate(estimateAt(1'000'000));
  publisher.onImuEstimate(estimateAt(10'000'000));
  publisher.onImuEstimate(estimateAt(21'000'000));
  publisher.onImuEstimate(estimateAt(101'000'000));

  EXPECT_EQ(publisher.publicationCount(), 3U);
  EXPECT_EQ(publisher.publicationSkipCount(), 1U);
  ASSERT_TRUE(publisher.lastPublishedTime().has_value());
  EXPECT_EQ(publisher.lastPublishedTime()->nanoseconds(), 101'000'000);
  ASSERT_TRUE(publisher.nextPublishDeadline().has_value());
  EXPECT_EQ(publisher.nextPublishDeadline()->nanoseconds(), 121'000'000);
}

TEST_F(RosPropagatedOdometryPublisherTest, RejectsDuplicateOutputTimestamp) {
  rclcpp::Node node("propagated_duplicate_test");
  RosParameters parameters;
  parameters.odom_frame = "odom";
  parameters.imu_frame = "imu_link";
  RosPropagatedOdometryPublisher publisher(node, parameters);
  publisher.onImuEstimate(estimateAt(20'000'000));
  publisher.onImuEstimate(estimateAt(20'000'000));
  EXPECT_EQ(publisher.publicationCount(), 1U);
  EXPECT_EQ(publisher.publicationSkipCount(), 1U);
}

}  // namespace uav::nav::lio
