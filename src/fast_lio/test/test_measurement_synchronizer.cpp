// Copyright (c) 2026, LeTanDat
// SPDX-License-Identifier: BSD-3-Clause

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <deque>
#include <limits>

#include "fast_lio/commons.hpp"
#include "fast_lio/lidar_scan.hpp"
#include "fast_lio/measurement_synchronizer.hpp"

namespace fast_lio {
namespace {

// ============================================================
// Helpers
// ============================================================

NormalizedLidarScan makeScan(double start_time, double end_time, bool has_per_point_time,
                              int num_points = 3) {
    NormalizedLidarScan scan;
    scan.scan_start_time_s = start_time;
    scan.scan_end_time_s = end_time;
    scan.has_per_point_time = has_per_point_time;
    scan.lidar_frame = "mid360_lidar";
    scan.cloud->width = num_points;
    scan.cloud->height = 1;
    scan.cloud->points.resize(num_points);
    for (int i = 0; i < num_points; ++i) {
        scan.cloud->points[i].x = static_cast<float>(i + 1);
        scan.cloud->points[i].y = 0.0f;
        scan.cloud->points[i].z = 0.0f;
        scan.cloud->points[i].intensity = 0.0f;
        scan.cloud->points[i].curvature =
            has_per_point_time
                ? static_cast<float>((end_time - start_time) * i / std::max(1, num_points - 1))
                : 0.0f;
    }
    return scan;
}

IMUData makeImu(double time, double acc_z = 9.81) {
    return IMUData(Eigen::Vector3d(0, 0, acc_z), Eigen::Vector3d::Zero(), time);
}

SynchronizerConfig defaultConfig() {
    SynchronizerConfig config;
    config.max_imu_samples = 1000;
    config.max_lidar_scans = 5;
    config.max_imu_gap_s = 0.02;
    config.require_imu_before_scan_start = true;
    config.require_imu_after_scan_end = true;
    return config;
}

// ============================================================
// Tests
// ============================================================

// Test 1: REAL — full coverage produces Ready package
TEST(MeasurementSynchronizerTest, RealFullCoverageProducesReady) {
    auto config = defaultConfig();
    MeasurementSynchronizer sync(config);

    // Scan [1.00, 1.10]
    sync.pushLidar(makeScan(1.00, 1.10, true));

    // IMU bracketing scan
    sync.pushImu(makeImu(0.99));
    sync.pushImu(makeImu(1.00));
    for (double t = 1.005; t <= 1.10; t += 0.005) sync.pushImu(makeImu(t));
    sync.pushImu(makeImu(1.11));

    SyncResult result = sync.tryPop();

    EXPECT_EQ(result.status, SyncStatus::kReady);
    EXPECT_NEAR(result.package.cloud_start_time, 1.00, 1e-9);
    EXPECT_NEAR(result.package.cloud_end_time, 1.10, 1e-9);
    EXPECT_TRUE(result.package.has_per_point_time);
    ASSERT_TRUE(result.package.imu_before_scan.has_value());
    EXPECT_NEAR(result.package.imu_before_scan->time, 0.99, 1e-9);
    ASSERT_TRUE(result.package.imu_after_scan.has_value());
    EXPECT_NEAR(result.package.imu_after_scan->time, 1.11, 1e-9);
    // IMUs within [1.00, 1.10]
    EXPECT_GE(result.package.imus.size(), 4u);
}

// Test 2: IMU hasn't reached scan_end → WaitingForImu
TEST(MeasurementSynchronizerTest, ImuNotReachedScanEndWaits) {
    auto config = defaultConfig();
    MeasurementSynchronizer sync(config);

    sync.pushLidar(makeScan(1.00, 1.10, true));
    sync.pushImu(makeImu(0.99));
    sync.pushImu(makeImu(1.00));
    sync.pushImu(makeImu(1.05));  // last IMU < 1.10

    SyncResult result = sync.tryPop();

    EXPECT_EQ(result.status, SyncStatus::kWaitingForImu);
}

// Test 3: No IMU before scan_start → DropNoStartCoverage
TEST(MeasurementSynchronizerTest, NoImuBeforeStartDropsScan) {
    auto config = defaultConfig();
    config.require_imu_before_scan_start = true;
    MeasurementSynchronizer sync(config);

    // Scan with duration
    sync.pushLidar(makeScan(1.00, 1.10, true));

    // First IMU at 1.05 (after scan_start)
    sync.pushImu(makeImu(1.05));
    sync.pushImu(makeImu(1.08));
    sync.pushImu(makeImu(1.10));
    sync.pushImu(makeImu(1.11));

    SyncResult result = sync.tryPop();

    EXPECT_EQ(result.status, SyncStatus::kDropNoStartCoverage);
}

// Test 4: SIM snapshot — start == end, bracketing still works
TEST(MeasurementSynchronizerTest, SimSnapshotProducesReady) {
    auto config = defaultConfig();
    // For SIM, don't require start coverage (scan_start == scan_end)
    config.require_imu_before_scan_start = true;
    MeasurementSynchronizer sync(config);

    // SIM snapshot: start == end
    sync.pushLidar(makeScan(1.00, 1.00, false));

    // IMU around t=1.00
    sync.pushImu(makeImu(0.995));
    sync.pushImu(makeImu(1.005));

    SyncResult result = sync.tryPop();

    EXPECT_EQ(result.status, SyncStatus::kReady);
    EXPECT_FALSE(result.package.has_per_point_time);
    EXPECT_NEAR(result.package.cloud_start_time, 1.00, 1e-9);
    EXPECT_NEAR(result.package.cloud_end_time, 1.00, 1e-9);
}

// Test 5: IMU gap exceeds max → DropImuGap
TEST(MeasurementSynchronizerTest, LargeImuGapDropsScan) {
    auto config = defaultConfig();
    config.max_imu_gap_s = 0.015;  // 15ms max
    MeasurementSynchronizer sync(config);

    sync.pushLidar(makeScan(1.00, 1.10, true));

    sync.pushImu(makeImu(0.99));
    sync.pushImu(makeImu(1.00));
    sync.pushImu(makeImu(1.02));
    // Gap: 1.02 → 1.08 = 60ms > 15ms
    sync.pushImu(makeImu(1.08));
    sync.pushImu(makeImu(1.10));
    sync.pushImu(makeImu(1.11));

    SyncResult result = sync.tryPop();

    EXPECT_EQ(result.status, SyncStatus::kDropImuGap);
}

// Test 6: Boundary sample reuse — consecutive scans share IMU at boundary
TEST(MeasurementSynchronizerTest, BoundarySampleReusedBetweenScans) {
    auto config = defaultConfig();
    MeasurementSynchronizer sync(config);

    // Scan A: [1.00, 1.10]
    sync.pushLidar(makeScan(1.00, 1.10, true));
    sync.pushImu(makeImu(0.99));
    sync.pushImu(makeImu(1.00));
    for (double t = 1.005; t <= 1.10; t += 0.005) sync.pushImu(makeImu(t));
    sync.pushImu(makeImu(1.11));  // bracket after A

    SyncResult result_a = sync.tryPop();
    ASSERT_EQ(result_a.status, SyncStatus::kReady);
    ASSERT_TRUE(result_a.package.imu_after_scan.has_value());
    EXPECT_NEAR(result_a.package.imu_after_scan->time, 1.11, 1e-9);

    // Scan B: [1.10, 1.20]
    sync.pushLidar(makeScan(1.10, 1.20, true));
    for (double t = 1.12; t <= 1.20; t += 0.005) sync.pushImu(makeImu(t));
    sync.pushImu(makeImu(1.21));  // bracket after B

    SyncResult result_b = sync.tryPop();
    ASSERT_EQ(result_b.status, SyncStatus::kReady);
    // Boundary sample at scan_end A (1.10) should be reused as imu_before_scan for B
    ASSERT_TRUE(result_b.package.imu_before_scan.has_value());
    EXPECT_NEAR(result_b.package.imu_before_scan->time, 1.10, 1e-9);
}

// Test 7: LiDAR arrives before IMU → buffer and wait
TEST(MeasurementSynchronizerTest, LidarBeforeImuBuffersAndWaits) {
    auto config = defaultConfig();
    MeasurementSynchronizer sync(config);

    sync.pushLidar(makeScan(1.00, 1.10, true));
    // No IMU yet

    SyncResult result = sync.tryPop();
    EXPECT_EQ(result.status, SyncStatus::kWaitingForImu);

    // IMU arrives
    sync.pushImu(makeImu(0.99));
    sync.pushImu(makeImu(1.00));
    for (double t = 1.005; t <= 1.10; t += 0.005) sync.pushImu(makeImu(t));
    sync.pushImu(makeImu(1.11));

    SyncResult result2 = sync.tryPop();
    EXPECT_EQ(result2.status, SyncStatus::kReady);
}

// Test 8: IMU arrives before LiDAR → emit when scan arrives
TEST(MeasurementSynchronizerTest, ImuBeforeLidarEmitsOnLidarArrival) {
    auto config = defaultConfig();
    MeasurementSynchronizer sync(config);

    // IMU first
    sync.pushImu(makeImu(0.99));
    sync.pushImu(makeImu(1.00));
    for (double t = 1.005; t <= 1.10; t += 0.005) sync.pushImu(makeImu(t));
    sync.pushImu(makeImu(1.11));

    SyncResult result = sync.tryPop();
    EXPECT_EQ(result.status, SyncStatus::kWaitingForLidar);

    // LiDAR arrives
    sync.pushLidar(makeScan(1.00, 1.10, true));

    SyncResult result2 = sync.tryPop();
    EXPECT_EQ(result2.status, SyncStatus::kReady);
}

// Test 9: Duplicate IMU timestamp → drop, don't clear buffer
TEST(MeasurementSynchronizerTest, DuplicateImuTimestampDropped) {
    auto config = defaultConfig();
    MeasurementSynchronizer sync(config);

    sync.pushImu(makeImu(0.99));  // bracket before scan_start
    sync.pushImu(makeImu(1.00));  // at scan_start (will be in package)
    sync.pushImu(makeImu(1.00));  // duplicate, will be dropped
    for (double t = 1.005; t <= 1.10; t += 0.005) sync.pushImu(makeImu(t));
    sync.pushImu(makeImu(1.11));

    sync.pushLidar(makeScan(1.00, 1.10, true));

    SyncResult result = sync.tryPop();
    EXPECT_EQ(result.status, SyncStatus::kReady);
    // 1.005..1.10 within scan, plus 1.00 at start = ~20 IMU samples
    EXPECT_GE(result.package.imus.size(), 15u);
}

// Test 10: Large clock reset → clear buffers
TEST(MeasurementSynchronizerTest, ClockResetClearsBuffers) {
    auto config = defaultConfig();
    MeasurementSynchronizer sync(config);

    sync.pushImu(makeImu(100.0));
    sync.pushImu(makeImu(100.05));
    sync.pushLidar(makeScan(100.0, 100.1, true));

    // Clock jumps back to ~0 — buffers cleared
    sync.pushImu(makeImu(0.01));

    // No LiDAR after reset
    SyncResult result = sync.tryPop();
    EXPECT_EQ(result.status, SyncStatus::kWaitingForLidar);
}

// Test 11: Buffer overflow trims IMU
TEST(MeasurementSynchronizerTest, BufferOverflowTrimsImu) {
    auto config = defaultConfig();
    config.max_imu_samples = 5;
    MeasurementSynchronizer sync(config);

    // Push 20 IMU samples
    for (int i = 0; i < 20; ++i) {
        sync.pushImu(makeImu(0.01 * i));
    }

    // Buffer should be trimmed to 5
    auto diag = sync.diagnostics();
    EXPECT_LE(diag.imu_buffer_size, 5u);
}

// Test 12: No LiDAR → WaitingForLidar
TEST(MeasurementSynchronizerTest, NoLidarReturnsWaiting) {
    auto config = defaultConfig();
    MeasurementSynchronizer sync(config);

    sync.pushImu(makeImu(1.0));
    sync.pushImu(makeImu(1.05));

    SyncResult result = sync.tryPop();
    EXPECT_EQ(result.status, SyncStatus::kWaitingForLidar);
}

// Test 13: SIM snapshot with require_imu_before_scan_start=false
TEST(MeasurementSynchronizerTest, SimSnapshotNoStartCoverageRequired) {
    auto config = defaultConfig();
    config.require_imu_before_scan_start = false;
    MeasurementSynchronizer sync(config);

    sync.pushLidar(makeScan(1.00, 1.00, false));
    // IMU only after scan time
    sync.pushImu(makeImu(1.005));

    SyncResult result = sync.tryPop();
    EXPECT_EQ(result.status, SyncStatus::kReady);
}

// Test 14: Non-finite IMU rejected
TEST(MeasurementSynchronizerTest, NonFiniteImuRejected) {
    auto config = defaultConfig();
    MeasurementSynchronizer sync(config);

    const double nan = std::numeric_limits<double>::quiet_NaN();
    sync.pushImu(IMUData(Eigen::Vector3d(nan, 0, 0), Eigen::Vector3d::Zero(), 1.0));
    sync.pushImu(IMUData(Eigen::Vector3d(0, 0, 9.81), Eigen::Vector3d::Zero(), 1.0));

    auto diag = sync.diagnostics();
    EXPECT_EQ(diag.imu_received, 2u);  // both received
    // But only 1 stored (NaN rejected)
    // Verify by trying to produce a package
    sync.pushLidar(makeScan(1.0, 1.0, false));
    sync.pushImu(makeImu(1.01));
    SyncResult result = sync.tryPop();
    // Should work with the single valid IMU
    EXPECT_EQ(result.status, SyncStatus::kReady);
}

}  // namespace
}  // namespace fast_lio