// Copyright (c) 2026, LeTanDat
// SPDX-License-Identifier: BSD-3-Clause

#include <gtest/gtest.h>

#include <cmath>

#include "fast_lio/commons.hpp"
#include "fast_lio/imu_processor.hpp"
#include "fast_lio/imu_trajectory.hpp"
#include "fast_lio/ieskf.hpp"

namespace fast_lio {
namespace {

// Helper: Create IMU data for stationary case (specific force = -gravity in IMU frame)
IMUData makeStationaryImu(double time_s) {
    // Accelerometer measures specific force: f = -R^T * g
    // For IMU at rest with gravity [0, 0, -9.80665] in world:
    // f = [0, 0, +9.80665] in IMU frame
    return IMUData(Eigen::Vector3d(0.0, 0.0, 9.80665), Eigen::Vector3d::Zero(), time_s);
}

// Helper: Create IMU data for constant acceleration
IMUData makeAcceleratingImu(const Eigen::Vector3d& accel_body, const Eigen::Vector3d& gyro_body, double time_s) {
    return IMUData(accel_body, gyro_body, time_s);
}

// T1: Trajectory end matches predicted filter state at scan_end
TEST(IMUTrajectoryTest, TrajectoryEndMatchesFilterStateAtScanEnd) {
    Config config;
    config.imu_init_num = 1;
    config.imu_init_gyro_rms_max = 2.0;
    IMUProcessor processor(config);
    auto filter = std::make_shared<IESKF>();

    // Initialize with stationary IMU
    const std::deque<IMUData> initialization{makeStationaryImu(0.0)};
    ASSERT_TRUE(processor.initialize(initialization));
    filter->initWithGravity(processor.getMeanAcc());

    // First scan: establish epoch
    processor.propagate(filter, initialization, 0.0, 0.0);

    // Second scan with motion: constant velocity 1 m/s in X
    const double scan_start = 1.0;
    const double scan_end = 1.1;
    const Eigen::Vector3d accel_body(0.0, 0.0, 9.80665);  // Stationary w.r.t. gravity
    const Eigen::Vector3d gyro_body(0.0, 0.0, 0.0);
    
    std::deque<IMUData> imus;
    imus.emplace_back(accel_body, gyro_body, 1.02);
    imus.emplace_back(accel_body, gyro_body, 1.05);
    imus.emplace_back(accel_body, gyro_body, 1.08);

    // Propagate and capture trajectory
    ImuTrajectory trajectory = processor.propagate(filter, imus, scan_start, scan_end);

    // Get final filter state
    const State15 final_state = filter->getState();

    // ASSERTIONS: Trajectory end must match filter predicted state
    ASSERT_GE(trajectory.size(), 2u);
    const auto& last_knot = trajectory.back();

    // T1.1: Timestamp must match scan_end
    EXPECT_NEAR(last_knot.timestamp_s, scan_end, 1e-9)
        << "Trajectory end timestamp must match scan_end_time";

    // T1.2: Rotation must match
    EXPECT_TRUE(last_knot.R_W_I.matrix().isApprox(final_state.R_wb.matrix(), 1e-6))
        << "Trajectory end rotation must match filter predicted rotation";

    // T1.3: Position must match
    EXPECT_TRUE(last_knot.p_W_I.isApprox(final_state.p_w, 1e-6))
        << "Trajectory end position must match filter predicted position";

    // T1.4: Velocity must match
    EXPECT_TRUE(last_knot.v_W_I.isApprox(final_state.v_w, 1e-6))
        << "Trajectory end velocity must match filter predicted velocity";
}

// T2: Stationary propagation - velocity remains near zero, position constant
TEST(IMUTrajectoryTest, StationaryPropagationMaintainsZeroVelocity) {
    Config config;
    config.imu_init_num = 1;
    config.imu_init_gyro_rms_max = 2.0;
    IMUProcessor processor(config);
    auto filter = std::make_shared<IESKF>();

    // Initialize
    const std::deque<IMUData> initialization{makeStationaryImu(0.0)};
    ASSERT_TRUE(processor.initialize(initialization));
    filter->initWithGravity(processor.getMeanAcc());
    const Eigen::Vector3d initial_position = filter->getState().p_w;

    // First scan
    processor.propagate(filter, initialization, 0.0, 0.0);

    // Multiple stationary scans
    for (int scan = 0; scan < 5; ++scan) {
        const double scan_start = 1.0 + scan * 0.1;
        const double scan_end = scan_start + 0.1;
        
        std::deque<IMUData> imus;
        imus.emplace_back(makeStationaryImu(scan_start + 0.02));
        imus.emplace_back(makeStationaryImu(scan_start + 0.05));
        imus.emplace_back(makeStationaryImu(scan_start + 0.08));

        ImuTrajectory trajectory = processor.propagate(filter, imus, scan_start, scan_end);

        ASSERT_GE(trajectory.size(), 2u);
        const auto& last_knot = trajectory.back();

        // T2.1: Velocity must remain near zero
        EXPECT_NEAR(last_knot.v_W_I.norm(), 0.0, 1e-6)
            << "Stationary propagation: velocity must remain near zero at scan " << scan;

        // T2.2: Position must not drift
        EXPECT_NEAR((last_knot.p_W_I - initial_position).norm(), 0.0, 1e-6)
            << "Stationary propagation: position must not drift at scan " << scan;

        // T2.3: World-frame acceleration should cancel gravity
        // a_W = R * (f - b_a) + g_W = R * (R^T * g) - g = 0 (approximately)
        EXPECT_NEAR(last_knot.a_W.norm(), 0.0, 1e-3)
            << "Stationary propagation: world acceleration should be near zero";
    }
}

// T3: Covariance propagation - check basic properties
TEST(IMUTrajectoryTest, CovariancePropagationProperties) {
    Config config;
    config.imu_init_num = 1;
    config.imu_init_gyro_rms_max = 2.0;
    config.na = 0.01;   // Enable process noise
    config.ng = 0.001;
    config.nba = 0.0001;
    config.nbg = 0.00001;
    
    IMUProcessor processor(config);
    auto filter = std::make_shared<IESKF>();
    filter->configure(config);

    // Initialize
    const std::deque<IMUData> initialization{makeStationaryImu(0.0)};
    ASSERT_TRUE(processor.initialize(initialization));
    filter->initWithGravity(processor.getMeanAcc());

    // Capture initial covariance
    const Eigen::Matrix<double, 15, 15> initial_cov = filter->getCovariance();

    // First scan
    processor.propagate(filter, initialization, 0.0, 0.0);

    // Propagate with motion
    const double scan_start = 1.0;
    const double scan_end = 1.1;
    
    std::deque<IMUData> imus;
    imus.emplace_back(makeStationaryImu(1.02));
    imus.emplace_back(makeStationaryImu(1.05));
    imus.emplace_back(makeStationaryImu(1.08));

    processor.propagate(filter, imus, scan_start, scan_end);

    const Eigen::Matrix<double, 15, 15> final_cov = filter->getCovariance();

    // T3.1: Covariance must be finite (no NaN/Inf)
    EXPECT_TRUE(final_cov.allFinite())
        << "Covariance must not contain NaN or Inf after propagation";

    // T3.2: Covariance must be symmetric
    EXPECT_TRUE(final_cov.isApprox(final_cov.transpose(), 1e-12))
        << "Covariance must be symmetric";

    // T3.3: Check positive semi-definite via Cholesky
    Eigen::LLT<Eigen::Matrix<double, 15, 15>> llt(final_cov);
    EXPECT_EQ(llt.info(), Eigen::Success)
        << "Covariance must be positive semi-definite";

    // T3.4: With process noise, covariance should not shrink
    EXPECT_GE(final_cov.trace(), initial_cov.trace())
        << "With process noise, covariance should not decrease";
}

// T4: Trajectory covers expected time range
TEST(IMUTrajectoryTest, TrajectoryCoversScanTimeRange) {
    Config config;
    config.imu_init_num = 1;
    IMUProcessor processor(config);
    auto filter = std::make_shared<IESKF>();

    const std::deque<IMUData> initialization{makeStationaryImu(0.0)};
    ASSERT_TRUE(processor.initialize(initialization));
    filter->initWithGravity(processor.getMeanAcc());
    processor.propagate(filter, initialization, 0.0, 0.0);

    const double scan_start = 2.0;
    const double scan_end = 2.1;
    
    std::deque<IMUData> imus;
    imus.emplace_back(makeStationaryImu(2.02));
    imus.emplace_back(makeStationaryImu(2.05));
    imus.emplace_back(makeStationaryImu(2.08));

    ImuTrajectory trajectory = processor.propagate(filter, imus, scan_start, scan_end);

    // T4.1: Trajectory must cover scan start
    EXPECT_LE(trajectory.front().timestamp_s, scan_start + 1e-6)
        << "Trajectory must start at or before scan_start";

    // T4.2: Trajectory must cover scan end
    EXPECT_GE(trajectory.back().timestamp_s, scan_end - 1e-6)
        << "Trajectory must end at or after scan_end";

    // T4.3: covers() method must return true for all points in range
    EXPECT_TRUE(trajectory.covers(scan_start));
    EXPECT_TRUE(trajectory.covers(scan_end));
    EXPECT_TRUE(trajectory.covers((scan_start + scan_end) / 2.0));
}

// T5: Interpolation accuracy for query points within knots
TEST(IMUTrajectoryTest, InterpolationAccuracyWithinKnots) {
    ImuTrajectory traj;
    
    // Create simple trajectory: constant velocity 1 m/s in X
    ImuTrajectoryKnot k0;
    k0.timestamp_s = 0.0;
    k0.R_W_I = SO3d();
    k0.p_W_I = Eigen::Vector3d::Zero();
    k0.v_W_I = Eigen::Vector3d(1.0, 0.0, 0.0);  // 1 m/s in X
    k0.omega_I = Eigen::Vector3d::Zero();
    k0.a_W = Eigen::Vector3d::Zero();
    
    ImuTrajectoryKnot k1;
    k1.timestamp_s = 0.1;
    k1.R_W_I = SO3d();
    k1.p_W_I = Eigen::Vector3d(0.1, 0.0, 0.0);  // 1 m/s * 0.1 s
    k1.v_W_I = Eigen::Vector3d(1.0, 0.0, 0.0);
    k1.omega_I = Eigen::Vector3d::Zero();
    k1.a_W = Eigen::Vector3d::Zero();
    
    traj.append(k0);
    traj.append(k1);

    // Interpolate at mid-point
    const double query_time = 0.05;
    SE3d T_W_I_mid = traj.interpolateImuPose(query_time);

    // Expected: position = v * t = 1.0 * 0.05 = 0.05
    EXPECT_NEAR(T_W_I_mid.translation().x(), 0.05, 1e-6);
    EXPECT_NEAR(T_W_I_mid.translation().y(), 0.0, 1e-6);
    EXPECT_NEAR(T_W_I_mid.translation().z(), 0.0, 1e-6);
}

// T6: Interpolation handles edge cases correctly
TEST(IMUTrajectoryTest, InterpolationHandlesEdgeCases) {
    ImuTrajectory traj;
    
    // Single knot
    ImuTrajectoryKnot k0;
    k0.timestamp_s = 1.0;
    k0.R_W_I = SO3d::exp(Eigen::Vector3d(0.0, 0.0, M_PI/4));  // 45° Z rotation
    k0.p_W_I = Eigen::Vector3d(1.0, 2.0, 3.0);
    k0.v_W_I = Eigen::Vector3d::Zero();
    k0.omega_I = Eigen::Vector3d::Zero();
    k0.a_W = Eigen::Vector3d::Zero();
    traj.append(k0);

    // Query before first knot
    SE3d T_before = traj.interpolateImuPose(0.5);
    EXPECT_TRUE(T_before.rotation().matrix().isApprox(k0.R_W_I.matrix()));
    EXPECT_TRUE(T_before.translation().isApprox(k0.p_W_I));

    // Query at exact knot time
    SE3d T_at = traj.interpolateImuPose(1.0);
    EXPECT_TRUE(T_at.rotation().matrix().isApprox(k0.R_W_I.matrix()));
    EXPECT_TRUE(T_at.translation().isApprox(k0.p_W_I));

    // Query after last knot
    SE3d T_after = traj.interpolateImuPose(2.0);
    EXPECT_TRUE(T_after.rotation().matrix().isApprox(k0.R_W_I.matrix()));
    EXPECT_TRUE(T_after.translation().isApprox(k0.p_W_I));
}

}  // namespace
}  // namespace fast_lio
