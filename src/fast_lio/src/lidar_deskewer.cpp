// Copyright (c) 2026, LeTanDat
// SPDX-License-Identifier: BSD-3-Clause

#include "fast_lio/lidar_deskewer.hpp"

#include <cmath>

namespace fast_lio {

bool LidarDeskewer::needsDeskew(const NormalizedLidarScan& scan, double time_epsilon) {
    const double start_s = static_cast<double>(scan.scan_start_time_ns) * 1e-9;
    const double end_s = static_cast<double>(scan.scan_end_time_ns) * 1e-9;
    return scan.has_per_point_time &&
           end_s > start_s + time_epsilon &&
           scan.cloud->points.size() > 1;
}

DeskewResult LidarDeskewer::deskewToScanEnd(const NormalizedLidarScan& scan,
                                             const ImuTrajectory& trajectory,
                                             const SE3d& T_I_L,
                                             const SE3d& T_W_I_end) const {
    DeskewResult result;
    result.reference_time_s = static_cast<double>(scan.scan_end_time_ns) * 1e-9;

    const double scan_start_s = static_cast<double>(scan.scan_start_time_ns) * 1e-9;
    const double scan_end_s = static_cast<double>(scan.scan_end_time_ns) * 1e-9;

    // Validate trajectory covers scan time range
    if (!trajectory.covers(scan_start_s) ||
        !trajectory.covers(scan_end_s)) {
        result.status = DeskewStatus::kInvalidTrajectory;
        result.cloud->points.clear();
        return result;
    }

    const auto& src_cloud = *scan.cloud;
    if (src_cloud.points.empty()) {
        result.status = DeskewStatus::kEmptyInput;
        return result;
    }

    // Pre-allocate output cloud
    result.cloud->points.reserve(src_cloud.points.size());
    result.cloud->width = static_cast<uint32_t>(src_cloud.points.size());
    result.cloud->height = 1;
    result.cloud->is_dense = src_cloud.is_dense;

    // Precompute transforms
    // T_W_L_end = T_W_I_end * T_I_L
    const SE3d T_W_L_end = T_W_I_end * T_I_L;
    const SE3d T_L_end_W = T_W_L_end.inverse();

    // First pass: validate all points are within trajectory coverage
    // This ensures we don't produce a hybrid deskewed/raw cloud
    for (const auto& src_pt : src_cloud.points) {
        const double point_time_s = scan_start_s + static_cast<double>(src_pt.curvature);
        if (!trajectory.covers(point_time_s)) {
            result.status = DeskewStatus::kInvalidTrajectory;
            result.cloud->points.clear();
            return result;
        }
    }

    // Second pass: deskew all points
    // Monotonic interval index for O(N+M) traversal
    std::size_t knot_index = 0;

    for (const auto& src_pt : src_cloud.points) {
        PointType dst_pt;

        // Copy non-spatial fields
        dst_pt.intensity = src_pt.intensity;
        dst_pt.curvature = src_pt.curvature;
        dst_pt.normal_x = src_pt.normal_x;
        dst_pt.normal_y = src_pt.normal_y;
        dst_pt.normal_z = src_pt.normal_z;

        // Compute absolute point time
        const double point_time_s = scan_start_s + static_cast<double>(src_pt.curvature);

        // Advance knot_index to the interval containing point_time
        while (knot_index + 1 < trajectory.size() &&
               trajectory[knot_index + 1].timestamp_s < point_time_s) {
            ++knot_index;
        }

        // IMU pose at point time: T_W_I_i = trajectory.interpolateImuPose(point_time_s)
        const SE3d T_W_I_i = trajectory.interpolateImuPose(point_time_s);

        // LiDAR pose at point time: T_W_L_i = T_W_I_i * T_I_L
        const SE3d T_W_L_i = T_W_I_i * T_I_L;

        // Deskew: P_L_end = (T_W_L_end)^-1 * T_W_L_i * P_L_i
        const Eigen::Vector3d P_L_i(src_pt.x, src_pt.y, src_pt.z);
        const Eigen::Vector3d P_L_end = T_L_end_W * (T_W_L_i * P_L_i);

        dst_pt.x = static_cast<float>(P_L_end.x());
        dst_pt.y = static_cast<float>(P_L_end.y());
        dst_pt.z = static_cast<float>(P_L_end.z());
        result.cloud->points.push_back(dst_pt);
        ++result.compensated_points;
    }

    result.status = DeskewStatus::kSuccess;
    return result;
}

}  // namespace fast_lio
