// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2026, LeTanDat

#include <cmath>

#include <gtest/gtest.h>

#include <Eigen/Geometry>

#include "px4_mapping/lio_px4_bridge.hpp"

namespace {

nav_msgs::msg::Odometry MakeLioIdentity() {
    nav_msgs::msg::Odometry msg;
    msg.header.frame_id = "lio_world";
    msg.pose.pose.position.x = 0.0;
    msg.pose.pose.position.y = 0.0;
    msg.pose.pose.position.z = 0.0;
    msg.pose.pose.orientation.w = 1.0;
    msg.pose.pose.orientation.x = 0.0;
    msg.pose.pose.orientation.y = 0.0;
    msg.pose.pose.orientation.z = 0.0;

    // Valid covariance
    msg.pose.covariance[0] = 0.01;
    msg.pose.covariance[7] = 0.01;
    msg.pose.covariance[14] = 0.01;
    msg.pose.covariance[21] = 0.01;
    msg.pose.covariance[28] = 0.01;
    msg.pose.covariance[35] = 0.01;
    return msg;
}

nav_msgs::msg::Odometry MakeLioTranslation(double x, double y, double z) {
    auto msg = MakeLioIdentity();
    msg.pose.pose.position.x = x;
    msg.pose.pose.position.y = y;
    msg.pose.pose.position.z = z;
    return msg;
}

nav_msgs::msg::Odometry MakeLioRotation(double roll, double pitch, double yaw) {
    auto msg = MakeLioIdentity();
    Eigen::Quaterniond q =
        Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()) *
        Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) *
        Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX());
    msg.pose.pose.orientation.w = q.w();
    msg.pose.pose.orientation.x = q.x();
    msg.pose.pose.orientation.y = q.y();
    msg.pose.pose.orientation.z = q.z();
    return msg;
}

Eigen::Vector3d ExtractPosition(const px4_msgs::msg::VehicleOdometry& msg) {
    return Eigen::Vector3d(msg.position[0], msg.position[1], msg.position[2]);
}

Eigen::Quaterniond ExtractOrientation(const px4_msgs::msg::VehicleOdometry& msg) {
    return Eigen::Quaterniond(msg.q[0], msg.q[1], msg.q[2], msg.q[3]);
}

// C = diag(1, -1, -1) for FLU -> FRD
Eigen::Vector3d ExpectedFrdPosition(const Eigen::Vector3d& p_flu) {
    return Eigen::Vector3d(p_flu.x(), -p_flu.y(), -p_flu.z());
}

}  // namespace

// Frame conversion tests: Local-FRD
TEST(LioPx4BridgeFrdTest, IdentityPoseProducesIdentityFrd) {
    const auto output = px4_mapping::ConvertLioToPx4Frd(
        MakeLioIdentity(), 2'000'000, 1'900'000, 100);

    EXPECT_EQ(output.pose_frame, px4_msgs::msg::VehicleOdometry::POSE_FRAME_FRD);

    const auto pos = ExtractPosition(output);
    EXPECT_NEAR(pos.x(), 0.0, 1e-6);
    EXPECT_NEAR(pos.y(), 0.0, 1e-6);
    EXPECT_NEAR(pos.z(), 0.0, 1e-6);

    const auto quat = ExtractOrientation(output);
    EXPECT_NEAR(quat.w(), 1.0, 1e-6);
    EXPECT_NEAR(quat.x(), 0.0, 1e-6);
    EXPECT_NEAR(quat.y(), 0.0, 1e-6);
    EXPECT_NEAR(quat.z(), 0.0, 1e-6);
}

TEST(LioPx4BridgeFrdTest, TranslationXBecomesX) {
    const auto output = px4_mapping::ConvertLioToPx4Frd(
        MakeLioTranslation(2.0, 0.0, 0.0), 2'000'000, 1'900'000, 100);

    const auto pos = ExtractPosition(output);
    EXPECT_NEAR(pos.x(), 2.0, 1e-6);
    EXPECT_NEAR(pos.y(), 0.0, 1e-6);
    EXPECT_NEAR(pos.z(), 0.0, 1e-6);
}

TEST(LioPx4BridgeFrdTest, TranslationYFluBecomesNegativeYFrd) {
    const auto output = px4_mapping::ConvertLioToPx4Frd(
        MakeLioTranslation(0.0, 3.0, 0.0), 2'000'000, 1'900'000, 100);

    const auto pos = ExtractPosition(output);
    EXPECT_NEAR(pos.x(), 0.0, 1e-6);
    EXPECT_NEAR(pos.y(), -3.0, 1e-6);  // Y_FLU -> -Y_FRD
    EXPECT_NEAR(pos.z(), 0.0, 1e-6);
}

TEST(LioPx4BridgeFrdTest, TranslationZFluBecomesNegativeZFrd) {
    const auto output = px4_mapping::ConvertLioToPx4Frd(
        MakeLioTranslation(0.0, 0.0, 1.5), 2'000'000, 1'900'000, 100);

    const auto pos = ExtractPosition(output);
    EXPECT_NEAR(pos.x(), 0.0, 1e-6);
    EXPECT_NEAR(pos.y(), 0.0, 1e-6);
    EXPECT_NEAR(pos.z(), -1.5, 1e-6);  // Z_FLU -> -Z_FRD
}

TEST(LioPx4BridgeFrdTest, Yaw90Degrees) {
    const auto output = px4_mapping::ConvertLioToPx4Frd(
        MakeLioRotation(0.0, 0.0, M_PI / 2.0), 2'000'000, 1'900'000, 100);

    const auto quat = ExtractOrientation(output);
    // Yaw rotation around Z should be preserved in FRD (same axis)
    EXPECT_NEAR(std::abs(quat.w()), std::sqrt(0.5), 1e-6);
    EXPECT_NEAR(std::abs(quat.z()), std::sqrt(0.5), 1e-6);
}

TEST(LioPx4BridgeFrdTest, Roll20Degrees) {
    const auto output = px4_mapping::ConvertLioToPx4Frd(
        MakeLioRotation(20.0 * M_PI / 180.0, 0.0, 0.0), 2'000'000, 1'900'000, 100);

    const auto quat = ExtractOrientation(output);
    EXPECT_TRUE(quat.coeffs().allFinite());
    EXPECT_NEAR(quat.norm(), 1.0, 1e-6);
}

TEST(LioPx4BridgeFrdTest, Pitch20Degrees) {
    const auto output = px4_mapping::ConvertLioToPx4Frd(
        MakeLioRotation(0.0, 20.0 * M_PI / 180.0, 0.0), 2'000'000, 1'900'000, 100);

    const auto quat = ExtractOrientation(output);
    EXPECT_TRUE(quat.coeffs().allFinite());
    EXPECT_NEAR(quat.norm(), 1.0, 1e-6);
}

TEST(LioPx4BridgeFrdTest, VelocityIsUnknown) {
    const auto output = px4_mapping::ConvertLioToPx4Frd(
        MakeLioIdentity(), 2'000'000, 1'900'000, 100);

    EXPECT_EQ(output.velocity_frame, px4_msgs::msg::VehicleOdometry::VELOCITY_FRAME_UNKNOWN);
    for (int i = 0; i < 3; ++i) {
        EXPECT_TRUE(std::isnan(output.velocity[i]));
        EXPECT_TRUE(std::isnan(output.angular_velocity[i]));
        EXPECT_TRUE(std::isnan(output.velocity_variance[i]));
    }
}

TEST(LioPx4BridgeFrdTest, CovariancePassedThrough) {
    auto input = MakeLioIdentity();
    input.pose.covariance[0] = 0.1;
    input.pose.covariance[7] = 0.2;
    input.pose.covariance[14] = 0.3;
    input.pose.covariance[21] = 0.01;
    input.pose.covariance[28] = 0.02;
    input.pose.covariance[35] = 0.03;

    const auto output = px4_mapping::ConvertLioToPx4Frd(
        input, 2'000'000, 1'900'000, 100);

    // Position variance
    EXPECT_FLOAT_EQ(output.position_variance[0], 0.1f);
    EXPECT_FLOAT_EQ(output.position_variance[1], 0.2f);
    EXPECT_FLOAT_EQ(output.position_variance[2], 0.3f);
    // Orientation variance
    EXPECT_FLOAT_EQ(output.orientation_variance[0], 0.01f);
    EXPECT_FLOAT_EQ(output.orientation_variance[1], 0.02f);
    EXPECT_FLOAT_EQ(output.orientation_variance[2], 0.03f);
}

TEST(LioPx4BridgeFrdTest, InvalidCovarianceBecomesNan) {
    auto input = MakeLioIdentity();
    input.pose.covariance[0] = -1.0;  // Invalid

    const auto output = px4_mapping::ConvertLioToPx4Frd(
        input, 2'000'000, 1'900'000, 100);

    EXPECT_TRUE(std::isnan(output.position_variance[0]));
}

TEST(LioPx4BridgeFrdTest, ResetCounterZeroInPhase1) {
    const auto output = px4_mapping::ConvertLioToPx4Frd(
        MakeLioIdentity(), 2'000'000, 1'900'000, 100);

    // Phase 1: reset_counter not connected to estimator reset events
    EXPECT_EQ(output.reset_counter, 0u);
}

// Non-normalized quaternion should be normalized
TEST(LioPx4BridgeFrdTest, NonNormalizedQuaternionIsHandled) {
    auto input = MakeLioIdentity();
    input.pose.pose.orientation.w = 2.0;
    input.pose.pose.orientation.x = 0.0;
    input.pose.pose.orientation.y = 0.0;
    input.pose.pose.orientation.z = 0.0;

    const auto output = px4_mapping::ConvertLioToPx4Frd(
        input, 2'000'000, 1'900'000, 100);

    const auto quat = ExtractOrientation(output);
    EXPECT_NEAR(quat.norm(), 1.0, 1e-6);
}
