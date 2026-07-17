// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2026, LeTanDat

#include <cmath>

#include <gtest/gtest.h>

#include <Eigen/Geometry>

#include "px4_mapping/lio_px4_bridge.hpp"

namespace {

nav_msgs::msg::Odometry MakeLioOdometry() {
    nav_msgs::msg::Odometry msg;
    msg.pose.pose.position.x = 1.0;
    msg.pose.pose.position.y = 2.0;
    msg.pose.pose.position.z = 3.0;
    msg.pose.pose.orientation.w = 1.0;
    msg.pose.pose.orientation.x = 0.0;
    msg.pose.pose.orientation.y = 0.0;
    msg.pose.pose.orientation.z = 0.0;

    msg.pose.covariance[0] = 1.0;
    msg.pose.covariance[7] = 4.0;
    msg.pose.covariance[14] = 9.0;
    msg.pose.covariance[21] = 0.1;
    msg.pose.covariance[28] = 0.2;
    msg.pose.covariance[35] = 0.3;
    return msg;
}

TEST(LioPx4BridgeConversionTest, ConvertsEnuFluPoseToNedFrd) {
    const auto output =
        px4_mapping::ConvertLioOdometryToPx4(MakeLioOdometry(), 2'000'000, 1'900'000, 100);

    EXPECT_EQ(output.timestamp, 2'000'000U);
    EXPECT_EQ(output.timestamp_sample, 1'900'000U);
    EXPECT_EQ(output.pose_frame, px4_msgs::msg::VehicleOdometry::POSE_FRAME_NED);
    EXPECT_FLOAT_EQ(output.position[0], 2.0F);
    EXPECT_FLOAT_EQ(output.position[1], 1.0F);
    EXPECT_FLOAT_EQ(output.position[2], -3.0F);

    const Eigen::Quaterniond actual(output.q[0], output.q[1], output.q[2], output.q[3]);
    const Eigen::Quaterniond expected(std::sqrt(0.5), 0.0, 0.0, std::sqrt(0.5));
    EXPECT_NEAR(std::abs(actual.dot(expected)), 1.0, 1e-6);
}

TEST(LioPx4BridgeConversionTest, PreservesTimestampQualityAndMapsVariances) {
    const auto output =
        px4_mapping::ConvertLioOdometryToPx4(MakeLioOdometry(), 2'000'000, 1'900'000, 87);

    EXPECT_FLOAT_EQ(output.position_variance[0], 4.0F);
    EXPECT_FLOAT_EQ(output.position_variance[1], 1.0F);
    EXPECT_FLOAT_EQ(output.position_variance[2], 9.0F);
    EXPECT_FLOAT_EQ(output.orientation_variance[0], 0.1F);
    EXPECT_FLOAT_EQ(output.orientation_variance[1], 0.2F);
    EXPECT_FLOAT_EQ(output.orientation_variance[2], 0.3F);
    EXPECT_EQ(output.quality, 87);
    EXPECT_EQ(output.reset_counter, 0U);
}

TEST(LioPx4BridgeConversionTest, MarksUnavailableTwistAndInvalidVarianceAsNan) {
    auto input = MakeLioOdometry();
    input.pose.covariance[0] = -1.0;

    const auto output = px4_mapping::ConvertLioOdometryToPx4(input, 2'000'000, 1'900'000, 100);

    EXPECT_EQ(output.velocity_frame, px4_msgs::msg::VehicleOdometry::VELOCITY_FRAME_UNKNOWN);
    for (int axis = 0; axis < 3; ++axis) {
        EXPECT_TRUE(std::isnan(output.velocity[axis]));
        EXPECT_TRUE(std::isnan(output.angular_velocity[axis]));
        EXPECT_TRUE(std::isnan(output.velocity_variance[axis]));
    }
    EXPECT_TRUE(std::isnan(output.position_variance[1]));
}

}  // namespace
