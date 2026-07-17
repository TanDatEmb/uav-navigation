// Copyright (c) 2026, LeTanDat
// SPDX-License-Identifier: BSD-3-Clause

#include "fast_lio/input/pointcloud2_adapter.hpp"

namespace fast_lio {

PointCloud2Adapter::PointCloud2Adapter(PointCloudDecoderConfig config, std::string name)
    : decoder_(std::move(config)), name_(std::move(name)) {}

DecodeResult PointCloud2Adapter::decode(const sensor_msgs::msg::PointCloud2& msg) {
    return decoder_.decode(msg);
}

std::string PointCloud2Adapter::name() const {
    return name_;
}

const LidarInputDiagnostics& PointCloud2Adapter::diagnostics() const {
    return decoder_.diagnostics();
}

std::unique_ptr<LidarInputAdapter> makeSimSnapshotAdapter() {
    PointCloudDecoderConfig config;
    config.profile = LidarInputProfile::kSimXyziSnapshot;
    config.time_field = "";
    config.require_per_point_time = false;
    config.header_stamp_is_scan_start = true;
    config.min_range_m = 0.5;
    config.max_range_m = 100.0;
    config.point_stride = 1;
    config.filter_livox_tags = false;

    return std::make_unique<PointCloud2Adapter>(config, "sim_snapshot");
}

std::unique_ptr<LidarInputAdapter> makeMid360PointCloud2Adapter() {
    PointCloudDecoderConfig config;
    config.profile = LidarInputProfile::kMid360PointCloud2;
    // Livox ROS Driver 2 PointCloud2 format (xfer_format=0) publishes a FLOAT64
    // `timestamp` field with absolute seconds since epoch for each point.
    config.time_field = "timestamp";
    config.time_unit = TimeUnit::kSeconds;
    config.require_per_point_time = true;
    // header.stamp is the scan end time; per-point timestamps are absolute and
    // decrease toward the start of the scan.
    config.header_stamp_is_scan_start = false;
    config.min_range_m = 0.5;
    config.max_range_m = 100.0;
    config.point_stride = 1;
    config.filter_livox_tags = true;

    return std::make_unique<PointCloud2Adapter>(config, "mid360_pointcloud2");
}

}  // namespace fast_lio
