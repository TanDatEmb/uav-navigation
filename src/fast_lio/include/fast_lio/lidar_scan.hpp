// Copyright (c) 2026, LeTanDat
// SPDX-License-Identifier: BSD-3-Clause

#ifndef FAST_LIO_LIDAR_SCAN_HPP_
#define FAST_LIO_LIDAR_SCAN_HPP_

#include <cstdint>
#include <string>
#include <vector>

#include <Eigen/Core>
#include <Eigen/StdVector>

#include "fast_lio/commons.hpp"  // CloudType, PointType

namespace fast_lio {

// ============================================================
// Time unit enumeration
// ============================================================

/// @brief Time unit for per-point timestamp fields in PointCloud2.
enum class TimeUnit : int {
    kSeconds = 0,
    kMilliseconds = 1,
    kMicroseconds = 2,
    kNanoseconds = 3,
};

/// @brief Convert a raw time value in the given unit to seconds.
inline double timeToSeconds(double raw, TimeUnit unit) {
    switch (unit) {
        case TimeUnit::kSeconds:
            return raw;
        case TimeUnit::kMilliseconds:
            return raw * 1e-3;
        case TimeUnit::kMicroseconds:
            return raw * 1e-6;
        case TimeUnit::kNanoseconds:
            return raw * 1e-9;
        default:
            return raw;
    }
}

// ============================================================
// Input profile
// ============================================================

/// @brief LiDAR input profile. Determines default decoding behavior.
enum class LidarInputProfile : int {
    /// Gazebo GPU-LiDAR snapshot: XYZI only, no per-point time.
    kSimXyziSnapshot = 0,
    /// Livox MID-360 via PointCloud2: XYZ + intensity + per-point time + tag/line.
    kMid360PointCloud2 = 1,
};

// ============================================================
// Decoder configuration
// ============================================================

/// @brief Configuration for the PointCloud2 decoder.
struct PointCloudDecoderConfig {
    /// Input profile (drives defaults).
    LidarInputProfile profile = LidarInputProfile::kSimXyziSnapshot;

    /// Expected LiDAR frame_id. If non-empty, mismatched frames emit a warning.
    std::string lidar_frame;

    // --- Timestamp configuration ---

    /// Name of the per-point time field in PointCloud2. Empty = no per-point time.
    std::string time_field;

    /// Time unit of the per-point time field.
    TimeUnit time_unit = TimeUnit::kNanoseconds;

    /// Whether header.stamp is scan start (true) or scan end (false).
    bool header_stamp_is_scan_start = true;

    /// If true, reject scans that lack per-point time when the profile requires it.
    bool require_per_point_time = false;

    /// Maximum allowed scan duration in seconds. Longer scans are rejected.
    double max_scan_duration_s = 0.2;

    // --- Field name overrides (for non-standard drivers) ---

    std::string intensity_field = "intensity";
    std::string line_field = "line";
    std::string tag_field = "tag";

    // --- Filtering ---

    /// Minimum range in meters. Points closer than this are rejected.
    double min_range_m = 0.5;

    /// Maximum range in meters. Points farther than this are rejected.
    double max_range_m = 100.0;

    /// Keep every N-th point (1 = keep all). Reduces decode load.
    int point_stride = 1;

    /// If true, filter Livox tags: keep only (tag & 0x30) == 0x10 or 0x00.
    bool filter_livox_tags = false;
};

// ============================================================
// Normalized LiDAR scan
// ============================================================

/// @brief Normalized LiDAR scan output by the decoder.
///
/// Points are in the LiDAR sensor frame (no transform applied).
/// Per-point time is stored in `PointType::curvature` as **seconds** relative
/// to scan_start_time_s. Points are sorted by ascending relative time.
///
/// Contract:
///   - scan_start_time_s <= scan_end_time_s
///   - If has_per_point_time: curvature >= 0 for all points
///   - If !has_per_point_time: curvature == 0 for all points
///   - All points have finite XYZ and valid range
struct NormalizedLidarScan {
    /// Point cloud in LiDAR frame. curvature = relative_time_s (seconds).
    CloudType::Ptr cloud;

    /// Scan start time in seconds (epoch/sim time).
    double scan_start_time_s = 0.0;

    /// Scan end time in seconds (epoch/sim time).
    double scan_end_time_s = 0.0;

    /// Whether per-point timestamps were present and decoded.
    bool has_per_point_time = false;

    /// Frame_id from the original PointCloud2 header.
    std::string lidar_frame;

    NormalizedLidarScan() : cloud(new CloudType()) {}
};

// ============================================================
// Decode result
// ============================================================

/// @brief Error codes for decoding failures.
enum class DecodeError : int {
    kOk = 0,
    kEmptyCloud,
    kMissingXyzField,
    kMissingTimeField,
    kInvalidTimeDatatype,
    kInvalidTimestamp,
    kInvalidScanDuration,
    kAllPointsInvalid,
    kSchemaMismatch,
};

/// @brief Result of decoding a PointCloud2 message.
struct DecodeResult {
    DecodeError error = DecodeError::kOk;
    NormalizedLidarScan scan;

    bool ok() const { return error == DecodeError::kOk; }

    /// Human-readable error message.
    std::string errorMessage() const;
};

// ============================================================
// Diagnostics
// ============================================================

/// @brief Input diagnostics for monitoring decode health.
struct LidarInputDiagnostics {
    std::uint64_t received_scans = 0;
    std::uint64_t accepted_scans = 0;
    std::uint64_t rejected_scans = 0;

    std::uint64_t input_points = 0;
    std::uint64_t valid_points = 0;

    std::uint64_t nonfinite_points = 0;
    std::uint64_t near_range_rejected = 0;
    std::uint64_t far_range_rejected = 0;
    std::uint64_t invalid_tag_rejected = 0;

    double last_scan_duration_s = 0.0;
    double last_point_time_min_s = 0.0;
    double last_point_time_max_s = 0.0;
};

}  // namespace fast_lio

#endif  // FAST_LIO_LIDAR_SCAN_HPP_