#include <gtest/gtest.h>

#include "fast_lio_ros/ros_odometry_serializer.hpp"

namespace uav::nav::lio {
namespace {

TEST(RosOdometrySerializerTest, UsesBaseLinkBodyContractAndExactTimestamp) {
  RosParameters parameters;
  parameters.odom_frame = "odom";
  parameters.base_frame = "base_link";
  RigidBodyState state;
  state.time = Timestamp(1'234'567'890);
  state.reference_frame = odomFrame();
  state.body_frame = baseFrame();
  state.position_reference_body_m = {1.0, 2.0, 3.0};
  state.linear_velocity_body_m_s = {4.0, 5.0, 6.0};
  state.angular_velocity_body_rad_s = Eigen::Vector3d(0.1, 0.2, 0.3);
  BaseLinkNavigationCovariance covariance;
  covariance.pose_covariance_odom = Matrix6d::Identity();
  covariance.twist_covariance_base = 2.0 * Matrix6d::Identity();
  covariance.pose_covariance_odom(0, 5) = 0.25;
  covariance.pose_covariance_odom(5, 0) = 0.25;
  covariance.twist_covariance_base(1, 4) = -0.5;
  covariance.twist_covariance_base(4, 1) = -0.5;

  const auto message =
      RosOdometrySerializer::serialize(state, covariance, parameters);
  ASSERT_TRUE(message.ok()) << message.status().message();
  EXPECT_EQ(message.value().header.frame_id, "odom");
  EXPECT_EQ(message.value().child_frame_id, "base_link");
  EXPECT_EQ(message.value().header.stamp.nanosec, 234'567'890U);
  EXPECT_DOUBLE_EQ(message.value().twist.twist.linear.x, 4.0);
  EXPECT_DOUBLE_EQ(message.value().twist.twist.angular.z, 0.3);
  EXPECT_DOUBLE_EQ(message.value().pose.covariance[0], 1.0);
  EXPECT_DOUBLE_EQ(message.value().pose.covariance[5], 0.25);
  EXPECT_DOUBLE_EQ(message.value().twist.covariance[1 * 6 + 4], -0.5);
  EXPECT_DOUBLE_EQ(message.value().twist.covariance[5 * 6 + 5], 2.0);
}

TEST(RosOdometrySerializerTest, RejectsUnavailableAngularVelocity) {
  RosParameters parameters;
  parameters.odom_frame = "odom";
  parameters.base_frame = "base_link";
  RigidBodyState state;
  state.reference_frame = odomFrame();
  state.body_frame = baseFrame();
  EXPECT_FALSE(RosOdometrySerializer::serialize(
                             state, BaseLinkNavigationCovariance{}, parameters)
                   .ok());
}

}  // namespace
}  // namespace uav::nav::lio
