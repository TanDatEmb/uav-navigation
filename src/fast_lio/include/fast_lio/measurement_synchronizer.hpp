// Copyright (c) 2026, LeTanDat
// SPDX-License-Identifier: BSD-3-Clause

#ifndef FAST_LIO_MEASUREMENT_SYNCHRONIZER_HPP_
#define FAST_LIO_MEASUREMENT_SYNCHRONIZER_HPP_

#include <cstdint>
#include <deque>
#include <mutex>
#include <optional>

#include "fast_lio/commons.hpp"  // IMUData, SyncPackage
#include "fast_lio/lidar_scan.hpp"  // NormalizedLidarScan

namespace fast_lio {

/// @brief Synchronization status codes for debugging.
enum class SyncStatus : int {
    kReady = 0,
    kWaitingForImu,        // IMU hasn't reached scan_end yet
    kWaitingForLidar,      // No LiDAR scan in buffer
    kDropNoStartCoverage,  // First IMU > scan_start (no coverage at beginning)
    kDropImuGap,           // IMU gap exceeds max_imu_gap_s
    kTimestampRegression,  // Timestamp went backwards (large reset)
};

/// @brief Configuration for the MeasurementSynchronizer.
struct SynchronizerConfig {
    /// Maximum IMU samples retained in buffer.
    std::size_t max_imu_samples = 4000;

    /// Maximum LiDAR scans retained in buffer.
    std::size_t max_lidar_scans = 5;

    /// Maximum allowed gap between consecutive IMU samples within a scan.
    double max_imu_gap_s = 0.02;

    /// Epsilon for timestamp comparison.
    double timestamp_epsilon_s = 1e-6;

    /// If true, reject scans when first IMU > scan_start (no start coverage).
    bool require_imu_before_scan_start = true;

    /// If true, retain first IMU >= scan_end as bracketing sample.
    bool require_imu_after_scan_end = true;
};

/// @brief Result of a tryPop() call.
struct SyncResult {
    SyncStatus status = SyncStatus::kWaitingForLidar;
    SyncPackage package;

    bool ready() const { return status == SyncStatus::kReady; }
};

/// @brief Synchronizes LiDAR scans with IMU streams.
///
/// Ensures each MeasurementPackage has IMU samples bracketing both
/// scan_start and scan_end, enabling interpolation instead of extrapolation
/// in the IMU processor.
///
/// Contract:
///   - imu_before_scan.time <= scan_start (if require_imu_before_scan_start)
///   - imu_after_scan.time >= scan_end (if require_imu_after_scan_end)
///   - All IMU samples in [scan_start, scan_end] are included
///   - Boundary sample (imu_after_scan) is retained for next package
///   - Points are NOT sorted here (done in decoder)
///   - Scan times come from NormalizedLidarScan (not curvature)
class MeasurementSynchronizer {
   public:
    explicit MeasurementSynchronizer(SynchronizerConfig config);

    /// Push an IMU sample. Thread-safe.
    void pushImu(const IMUData& sample);

    /// Push a normalized LiDAR scan. Thread-safe.
    void pushLidar(const NormalizedLidarScan& scan);

    /// Try to pop a synchronized package. Thread-safe.
    SyncResult tryPop();

    /// Access current diagnostics (not thread-safe, call from main thread).
    struct Diagnostics {
        std::uint64_t imu_received = 0;
        std::uint64_t lidar_received = 0;
        std::uint64_t packages_emitted = 0;
        std::uint64_t imu_out_of_order = 0;
        std::uint64_t lidar_out_of_order = 0;
        std::uint64_t dropped_no_start_coverage = 0;
        std::uint64_t dropped_imu_gap = 0;
        std::size_t imu_buffer_size = 0;
        std::size_t lidar_buffer_size = 0;
        double last_scan_duration_s = 0.0;
        double last_max_imu_gap_s = 0.0;
    };

    Diagnostics diagnostics() const;

   private:
    SynchronizerConfig config_;
    std::mutex mutex_;

    std::deque<IMUData> imu_buffer_;
    std::deque<NormalizedLidarScan> lidar_buffer_;

    /// Retained boundary IMU sample from previous package (for continuity).
    std::optional<IMUData> retained_boundary_imu_;

    double last_imu_time_ = -1e18;
    double last_lidar_start_time_ = -1e18;

    bool lidar_pushed_ = false;
    double cached_start_time_ = 0.0;
    double cached_end_time_ = 0.0;
    bool cached_has_per_point_time_ = false;

    /// End time of the previously emitted scan, used to collect IMU history
    /// for zero-duration (snapshot) scans.
    double last_scan_end_time_ = -1e18;

    Diagnostics diagnostics_;

    /// Trim IMU buffer to max size, preserving samples >= scan_start if possible.
    void trimImuBuffer();

    /// Trim LiDAR buffer to max size.
    void trimLidarBuffer();
};

}  // namespace fast_lio

#endif  // FAST_LIO_MEASUREMENT_SYNCHRONIZER_HPP_