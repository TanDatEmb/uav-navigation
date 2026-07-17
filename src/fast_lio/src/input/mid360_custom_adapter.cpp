// Copyright (c) 2026, LeTanDat
// SPDX-License-Identifier: BSD-3-Clause

#include "fast_lio/input/mid360_custom_adapter.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

#include <rclcpp/rclcpp.hpp>

#include "fast_lio/commons.hpp"

namespace fast_lio {

#ifdef LIVOX_ROS2_FOUND

Mid360CustomAdapter::Mid360CustomAdapter() = default;

DecodeResult Mid360CustomAdapter::decode(const sensor_msgs::msg::PointCloud2& /*msg*/) {
    DecodeResult result;
    result.error = DecodeError::kSchemaMismatch;
    return result;
}

DecodeResult Mid360CustomAdapter::decode(const livox_ros_driver2::msg::CustomMsg& msg) {
    ++diagnostics_.received_scans;
    DecodeResult result;

    if (msg.point_num == 0) {
        result.error = DecodeError::kEmptyCloud;
        ++diagnostics_.rejected_scans;
        return result;
    }

    const double timebase_s = static_cast<double>(msg.timebase) * 1e-9;
    if (!std::isfinite(timebase_s) || timebase_s <= 0.0) {
        result.error = DecodeError::kInvalidTimestamp;
        ++diagnostics_.rejected_scans;
        return result;
    }

    NormalizedLidarScan& scan = result.scan;
    scan.lidar_frame = msg.header.frame_id;
    scan.has_per_point_time = true;
    scan.cloud->clear();
    scan.cloud->is_dense = false;
    scan.cloud->height = 1;
    scan.cloud->points.reserve(msg.point_num);

    double min_rel_s = std::numeric_limits<double>::max();
    double max_rel_s = std::numeric_limits<double>::lowest();
    std::uint64_t valid_count = 0;

    constexpr double kMinRangeM = 0.5;
    constexpr double kMaxRangeM = 100.0;

    for (const auto& p : msg.points) {
        const double x = static_cast<double>(p.x);
        const double y = static_cast<double>(p.y);
        const double z = static_cast<double>(p.z);

        if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) {
            ++diagnostics_.nonfinite_points;
            continue;
        }

        const double range = std::sqrt(x * x + y * y + z * z);
        if (range < kMinRangeM) {
            ++diagnostics_.near_range_rejected;
            continue;
        }
        if (range > kMaxRangeM) {
            ++diagnostics_.far_range_rejected;
            continue;
        }

        // Livox tag filter: keep only valid points (tag & 0x30) == 0x10 or 0x00.
        const std::uint8_t tag = p.tag;
        if ((tag & 0x30) != 0x10 && (tag & 0x30) != 0x00) {
            ++diagnostics_.invalid_tag_rejected;
            continue;
        }

        const double rel_s = static_cast<double>(p.offset_time) * 1e-9;
        if (!std::isfinite(rel_s) || rel_s < -1e-6) {
            ++diagnostics_.nonfinite_points;
            continue;
        }

        PointType pt;
        pt.x = p.x;
        pt.y = p.y;
        pt.z = p.z;
        pt.intensity = static_cast<float>(p.reflectivity);
        pt.normal_x = 0.0f;
        pt.normal_y = 0.0f;
        pt.normal_z = 0.0f;
        pt.curvature = static_cast<float>(rel_s);
        scan.cloud->points.push_back(pt);
        ++valid_count;

        if (rel_s < min_rel_s) min_rel_s = rel_s;
        if (rel_s > max_rel_s) max_rel_s = rel_s;
    }

    diagnostics_.input_points += msg.point_num;
    diagnostics_.valid_points += valid_count;

    if (scan.cloud->points.empty()) {
        result.error = DecodeError::kAllPointsInvalid;
        ++diagnostics_.rejected_scans;
        return result;
    }

    // Normalize relative times to start at zero.
    if (min_rel_s > 0.0 && std::isfinite(min_rel_s)) {
        for (auto& pt : scan.cloud->points) {
            pt.curvature -= static_cast<float>(min_rel_s);
        }
        max_rel_s -= min_rel_s;
        min_rel_s = 0.0;
    }

    std::sort(scan.cloud->points.begin(), scan.cloud->points.end(),
              [](const PointType& a, const PointType& b) { return a.curvature < b.curvature; });

    scan.scan_start_time_s = timebase_s + min_rel_s;
    scan.scan_end_time_s = timebase_s + max_rel_s;
    scan.cloud->width = static_cast<std::uint32_t>(scan.cloud->points.size());
    scan.cloud->is_dense = true;

    diagnostics_.last_point_time_min_s = min_rel_s;
    diagnostics_.last_point_time_max_s = max_rel_s;
    diagnostics_.last_scan_duration_s = max_rel_s - min_rel_s;
    ++diagnostics_.accepted_scans;

    return result;
}

std::string Mid360CustomAdapter::name() const {
    return "mid360_custom";
}

const LidarInputDiagnostics& Mid360CustomAdapter::diagnostics() const {
    return diagnostics_;
}

#endif  // LIVOX_ROS2_FOUND

std::unique_ptr<LidarInputAdapter> makeMid360CustomAdapter() {
#ifdef LIVOX_ROS2_FOUND
    return std::make_unique<Mid360CustomAdapter>();
#else
    throw std::runtime_error(
        "mid360_custom adapter requires livox_ros_driver2. "
        "Install the driver and rebuild, or use a PointCloud2-based adapter.");
#endif
}

}  // namespace fast_lio
