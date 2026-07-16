// Copyright (c) 2026, LeTanDat
// SPDX-License-Identifier: BSD-3-Clause

#ifndef FAST_LIO_LIDAR_DESKEWER_HPP_
#define FAST_LIO_LIDAR_DESKEWER_HPP_

#include <cstddef>
#include <memory>

#include "fast_lio/commons.hpp"      // CloudType, PointType, SE3d
#include "fast_lio/imu_trajectory.hpp"  // ImuTrajectory
#include "fast_lio/lidar_scan.hpp"   // NormalizedLidarScan

namespace fast_lio {

/// @brief Status of deskew operation.
enum class DeskewStatus {
    kSuccess = 0,          ///< All points compensated successfully
    kPartial = 1,            ///< Some points rejected (outside trajectory)
    kInvalidTrajectory = 2,  ///< Trajectory doesn't cover scan time range
    kEmptyInput = 3          ///< Empty input cloud
};

/// @brief Result of a deskew operation.
struct DeskewResult {
    /// Undistorted point cloud in LiDAR frame at scan_end.
    /// Empty if status is kInvalidTrajectory or kEmptyInput.
    CloudType::Ptr cloud;

    /// Number of points successfully compensated.
    std::size_t compensated_points = 0;

    /// Number of points rejected (outside trajectory coverage).
    std::size_t rejected_points = 0;

    /// Scan end time used as reference.
    double reference_time_s = 0.0;

    /// Operation status.
    DeskewStatus status = DeskewStatus::kSuccess;

    DeskewResult() : cloud(new CloudType()) {}
};

/// @brief Deskews a LiDAR scan by compensating point motion distortion.
///
/// Uses the forward-propagated IMU trajectory to transform each point
/// from its capture time to the scan_end frame using SE(3) absolute
/// transforms:
///
///   P_L_end = (T_W_L_end)⁻¹ * T_W_L_i * P_L_i
///
/// where:
///   T_W_L_i = T_W_I_i * T_I_L  (LiDAR pose in world at point time i)
///   T_W_L_end = T_W_I_end * T_I_L  (LiDAR pose in world at scan_end)
///
/// Points are iterated in ascending time order (forward, not backward)
/// with a monotonic interval index for O(N+M) complexity.
class LidarDeskewer {
   public:
    /// @brief Deskew a scan to scan_end frame.
    ///
    /// @param scan Normalized LiDAR scan (points sorted by time, curvature = seconds)
    /// @param trajectory Forward-propagated IMU trajectory covering [scan_start, scan_end]
    /// @param T_I_L LiDAR-to-IMU extrinsic
    /// @param state_end Predicted state at scan_end (before LiDAR correction)
    /// @return DeskewResult with compensated cloud
    DeskewResult deskewToScanEnd(const NormalizedLidarScan& scan,
                                  const ImuTrajectory& trajectory,
                                  const SE3d& T_I_L,
                                  const SE3d& T_W_I_end) const;

    /// @brief Check if deskew is needed for this scan.
    ///
    /// Deskew is needed when:
    ///   - has_per_point_time is true
    ///   - scan duration > 0
    ///   - more than 1 point
    static bool needsDeskew(const NormalizedLidarScan& scan, double time_epsilon = 1e-9);
};

}  // namespace fast_lio

#endif  // FAST_LIO_LIDAR_DESKEWER_HPP_