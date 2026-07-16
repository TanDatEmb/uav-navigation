// Copyright (c) 2026, LeTanDat
// SPDX-License-Identifier: BSD-3-Clause

#ifndef FAST_LIO_IMU_PROCESSOR_HPP_
#define FAST_LIO_IMU_PROCESSOR_HPP_

#include <deque>
#include <memory>
#include <optional>
#include <vector>

#include "fast_lio/commons.hpp"
#include "fast_lio/ieskf.hpp"
#include "fast_lio/imu_trajectory.hpp"

namespace fast_lio {

/// @brief Diagnostics for IMU initialization progress.
struct ImuInitializationDiagnostics {
    std::size_t collected_samples{0};
    std::size_t required_samples{0};
    V3D mean_acceleration{V3D::Zero()};
    V3D mean_gyro{V3D::Zero()};
    double acceleration_norm{0.0};
    double gyro_norm{0.0};
    double accel_std{0.0};
    double gyro_rms{0.0};
    double gravity_error{0.0};
    bool stationary{false};
    bool initialized{false};
};

/// @brief IMU processing: initialization, forward propagation, trajectory generation.
class IMUProcessor {
   public:
    explicit IMUProcessor(const Config& config);

    /// Initialize IMU with static measurements (gravity, bias).
    bool initialize(const std::deque<IMUData>& imu_buffer);

    /// @brief Current initialization diagnostics (for logging/debugging).
    ImuInitializationDiagnostics initializationDiagnostics() const;

    bool isInitialized() const { return initialized_; }

    /// @brief Forward propagate IESKF through IMU samples, building trajectory.
    ///
    /// Uses imu_before_scan and imu_after_scan brackets from the synchronizer
    /// to ensure continuous integration from scan_start to scan_end.
    ///
    /// @param kf Shared IESKF instance
    /// @param imus IMU samples within (scan_start, scan_end)
    /// @param scan_start_time Scan start time (seconds)
    /// @param scan_end_time Scan end time (seconds)
    /// @param imu_before Optional bracket sample at or before scan_start
    /// @param imu_after Optional bracket sample at or after scan_end
    /// @return ImuTrajectory spanning [scan_start, scan_end]
    ImuTrajectory propagate(std::shared_ptr<IESKF> kf,
                              const std::deque<IMUData>& imus,
                              double scan_start_time,
                              double scan_end_time,
                              const std::optional<IMUData>& imu_before = std::nullopt,
                              const std::optional<IMUData>& imu_after = std::nullopt);

    V3D getMeanAcc() const { return mean_acc_; }
    V3D getMeanGyro() const { return mean_gyro_; }
    double getAccelScale() const { return accel_scale_; }

   private:
    Config config_;
    bool initialized_;

    V3D mean_acc_;
    V3D mean_gyro_;
    std::deque<IMUData> init_window_;
    double accel_scale_{1.0};

    bool integration_initialized_{false};
    double integration_time_{0.0};
    IMUData last_imu_;

    /// Build a continuous, deduplicated IMU sequence from package parts.
    std::vector<IMUData> buildContinuousSequence(
        const std::deque<IMUData>& imus,
        const std::optional<IMUData>& imu_before,
        const std::optional<IMUData>& imu_after) const;

    /// Interpolate IMU measurement at a specific time.
    IMUData interpolateImu(const IMUData& a, const IMUData& b, double time) const;
};

}  // namespace fast_lio

#endif  // FAST_LIO_IMU_PROCESSOR_HPP_