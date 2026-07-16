// Copyright (c) 2026, LeTanDat
// SPDX-License-Identifier: BSD-3-Clause

#include <gtest/gtest.h>

#include <Eigen/Geometry>
#include <cmath>
#include <vector>

#include "fast_lio/commons.hpp"
#include "fast_lio/ieskf.hpp"

namespace fast_lio {
namespace {

// Helper to build an IMU sample from body-frame accel and gyro.
IMUData makeImu(const Eigen::Vector3d& acc_body, const Eigen::Vector3d& gyro_body,
                double time_s) {
    return IMUData(acc_body, gyro_body, time_s);
}

TEST(ImuPropagationTest, ConstantAccelerationNoRotation) {
    IESKF filter;
    filter.reset();

    // Body frame aligned with world at start.
    State15 state;
    state.R_wb = SO3d::exp(Eigen::Vector3d::Zero());
    state.p_w = Eigen::Vector3d::Zero();
    state.v_w = Eigen::Vector3d::Zero();
    state.b_a = Eigen::Vector3d::Zero();
    state.b_w = Eigen::Vector3d::Zero();
    filter.setState(state);

    const Eigen::Vector3d accel_body(1.0, 0.0, 0.0);  // 1 m/s^2 along body x
    const Eigen::Vector3d gyro_body = Eigen::Vector3d::Zero();
    const double dt = 0.005;  // 200 Hz
    const int steps = 200;    // 1 second

    for (int i = 0; i < steps; ++i) {
        filter.predict(makeImu(accel_body, gyro_body, (i + 1) * dt), dt);
    }

    const double t = steps * dt;
    // Since R stays identity, world accel = body accel + gravity.
    const Eigen::Vector3d expected_p =
        0.5 * (accel_body + Eigen::Vector3d(0, 0, -9.80665)) * t * t;
    const Eigen::Vector3d actual_p = filter.getState().p_w;
    const Eigen::Vector3d expected_v = (accel_body + Eigen::Vector3d(0, 0, -9.80665)) * t;
    const Eigen::Vector3d actual_v = filter.getState().v_w;

    EXPECT_NEAR(actual_p.x(), expected_p.x(), 1e-6);
    EXPECT_NEAR(actual_p.y(), expected_p.y(), 1e-6);
    EXPECT_NEAR(actual_p.z(), expected_p.z(), 1e-6);
    EXPECT_NEAR(actual_v.x(), expected_v.x(), 1e-6);
    EXPECT_NEAR(actual_v.y(), expected_v.y(), 1e-6);
    EXPECT_NEAR(actual_v.z(), expected_v.z(), 1e-6);

    // Rotation should stay identity.
    EXPECT_TRUE(filter.getState().R_wb.matrix().isApprox(Eigen::Matrix3d::Identity(), 1e-9));
}

TEST(ImuPropagationTest, ConstantYawRate) {
    IESKF filter;
    filter.reset();

    State15 state;
    state.R_wb = SO3d::exp(Eigen::Vector3d::Zero());
    state.p_w = Eigen::Vector3d::Zero();
    state.v_w = Eigen::Vector3d::Zero();
    state.b_a = Eigen::Vector3d::Zero();
    state.b_w = Eigen::Vector3d::Zero();
    filter.setState(state);

    // Hover: accel body z measures gravity.
    const Eigen::Vector3d accel_body(0.0, 0.0, 9.80665);
    const Eigen::Vector3d gyro_body(0.0, 0.0, 0.5);  // 0.5 rad/s yaw
    const double dt = 0.005;
    const int steps = 200;

    for (int i = 0; i < steps; ++i) {
        filter.predict(makeImu(accel_body, gyro_body, (i + 1) * dt), dt);
    }

    const double t = steps * dt;
    const double expected_yaw = gyro_body.z() * t;

    // Extract yaw from final rotation (ZYX convention, assuming small roll/pitch).
    const Eigen::Matrix3d R = filter.getState().R_wb.matrix();
    const double yaw = std::atan2(R(1, 0), R(0, 0));

    EXPECT_NEAR(yaw, expected_yaw, 1e-6);

    // Hovering: velocity and position should remain near zero.
    EXPECT_LT(filter.getState().p_w.norm(), 1e-5);
    EXPECT_LT(filter.getState().v_w.norm(), 1e-5);
}

TEST(ImuPropagationTest, MidpointRotationWithAcceleration) {
    IESKF filter;
    filter.reset();

    State15 state;
    state.R_wb = SO3d::exp(Eigen::Vector3d::Zero());
    state.p_w = Eigen::Vector3d::Zero();
    state.v_w = Eigen::Vector3d::Zero();
    state.b_a = Eigen::Vector3d::Zero();
    state.b_w = Eigen::Vector3d::Zero();
    filter.setState(state);

    // Constant forward accel plus constant yaw rate.
    const Eigen::Vector3d accel_body(0.2, 0.0, 9.80665);
    const Eigen::Vector3d gyro_body(0.0, 0.0, 0.5);
    const double dt = 0.005;
    const int steps = 200;

    for (int i = 0; i < steps; ++i) {
        filter.predict(makeImu(accel_body, gyro_body, (i + 1) * dt), dt);
    }

    // The key sanity check: with true midpoint integration, the trajectory should
    // curve smoothly; z should stay close to zero because accel_body.z() cancels
    // gravity on average and we have no z-direction motion.
    EXPECT_LT(std::abs(filter.getState().p_w.z()), 0.05);
    EXPECT_LT(std::abs(filter.getState().v_w.z()), 0.05);

    // Yaw should match the integrated gyro.
    const Eigen::Matrix3d R = filter.getState().R_wb.matrix();
    const double yaw = std::atan2(R(1, 0), R(0, 0));
    EXPECT_NEAR(yaw, gyro_body.z() * steps * dt, 1e-6);
}

}  // namespace
}  // namespace fast_lio
