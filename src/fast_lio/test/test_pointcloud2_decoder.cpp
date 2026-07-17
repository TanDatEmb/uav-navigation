// Copyright (c) 2026, LeTanDat
// SPDX-License-Identifier: BSD-3-Clause

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <limits>
#include <vector>

#include "fast_lio/lidar_scan.hpp"
#include "fast_lio/pointcloud2_decoder.hpp"

#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_field.hpp>

namespace fast_lio {
namespace {

// ============================================================
// Helpers to build PointCloud2 messages
// ============================================================

/// Build a PointCloud2 message with x, y, z, intensity fields (SIM snapshot).
sensor_msgs::msg::PointCloud2 makeSimXyziCloud(
    const std::vector<std::tuple<float, float, float, float>>& points,
    double header_time_sec = 10.0,
    const std::string& frame_id = "mid360_lidar") {
    sensor_msgs::msg::PointCloud2 msg;
    msg.header.stamp.sec = static_cast<int32_t>(header_time_sec);
    msg.header.stamp.nanosec = static_cast<uint32_t>(
        (header_time_sec - static_cast<int32_t>(header_time_sec)) * 1e9);
    msg.header.frame_id = frame_id;

    // Fields: x, y, z, intensity (all FLOAT32)
    std::vector<sensor_msgs::msg::PointField> fields(4);
    fields[0].name = "x";
    fields[0].offset = 0;
    fields[0].datatype = sensor_msgs::msg::PointField::FLOAT32;
    fields[0].count = 1;
    fields[1].name = "y";
    fields[1].offset = 4;
    fields[1].datatype = sensor_msgs::msg::PointField::FLOAT32;
    fields[1].count = 1;
    fields[2].name = "z";
    fields[2].offset = 8;
    fields[2].datatype = sensor_msgs::msg::PointField::FLOAT32;
    fields[2].count = 1;
    fields[3].name = "intensity";
    fields[3].offset = 12;
    fields[3].datatype = sensor_msgs::msg::PointField::FLOAT32;
    fields[3].count = 1;

    msg.fields = fields;
    msg.point_step = 16;  // 4 floats × 4 bytes
    msg.width = static_cast<uint32_t>(points.size());
    msg.height = 1;
    msg.row_step = msg.point_step * msg.width;
    msg.is_dense = true;
    msg.data.resize(points.size() * msg.point_step);

    for (size_t i = 0; i < points.size(); ++i) {
        auto [x, y, z, intensity] = points[i];
        std::uint8_t* ptr = msg.data.data() + i * msg.point_step;
        std::memcpy(ptr + 0, &x, sizeof(float));
        std::memcpy(ptr + 4, &y, sizeof(float));
        std::memcpy(ptr + 8, &z, sizeof(float));
        std::memcpy(ptr + 12, &intensity, sizeof(float));
    }

    return msg;
}

/// Build a PointCloud2 message with x, y, z, intensity, offset_time, line, tag
/// fields (REAL MID-360 style).
sensor_msgs::msg::PointCloud2 makeMid360Cloud(
    const std::vector<std::tuple<float, float, float, float, uint32_t, uint8_t, uint8_t>>& points,
    double header_time_sec = 10.0,
    const std::string& frame_id = "mid360_lidar") {
    sensor_msgs::msg::PointCloud2 msg;
    msg.header.stamp.sec = static_cast<int32_t>(header_time_sec);
    msg.header.stamp.nanosec = static_cast<uint32_t>(
        (header_time_sec - static_cast<int32_t>(header_time_sec)) * 1e9);
    msg.header.frame_id = frame_id;

    // Fields: x(F32), y(F32), z(F32), intensity(F32), offset_time(U32 ns), line(U8), tag(U8)
    std::vector<sensor_msgs::msg::PointField> fields(7);
    fields[0].name = "x";
    fields[0].offset = 0;
    fields[0].datatype = sensor_msgs::msg::PointField::FLOAT32;
    fields[0].count = 1;
    fields[1].name = "y";
    fields[1].offset = 4;
    fields[1].datatype = sensor_msgs::msg::PointField::FLOAT32;
    fields[1].count = 1;
    fields[2].name = "z";
    fields[2].offset = 8;
    fields[2].datatype = sensor_msgs::msg::PointField::FLOAT32;
    fields[2].count = 1;
    fields[3].name = "reflectivity";
    fields[3].offset = 12;
    fields[3].datatype = sensor_msgs::msg::PointField::FLOAT32;
    fields[3].count = 1;
    fields[4].name = "offset_time";
    fields[4].offset = 16;
    fields[4].datatype = sensor_msgs::msg::PointField::UINT32;
    fields[4].count = 1;
    fields[5].name = "line";
    fields[5].offset = 20;
    fields[5].datatype = sensor_msgs::msg::PointField::UINT8;
    fields[5].count = 1;
    fields[6].name = "tag";
    fields[6].offset = 21;
    fields[6].datatype = sensor_msgs::msg::PointField::UINT8;
    fields[6].count = 1;

    msg.fields = fields;
    msg.point_step = 24;  // pad to 24 for alignment
    msg.width = static_cast<uint32_t>(points.size());
    msg.height = 1;
    msg.row_step = msg.point_step * msg.width;
    msg.is_dense = true;
    msg.data.resize(points.size() * msg.point_step);

    for (size_t i = 0; i < points.size(); ++i) {
        auto [x, y, z, intensity, offset_ns, line, tag] = points[i];
        std::uint8_t* ptr = msg.data.data() + i * msg.point_step;
        std::memcpy(ptr + 0, &x, sizeof(float));
        std::memcpy(ptr + 4, &y, sizeof(float));
        std::memcpy(ptr + 8, &z, sizeof(float));
        std::memcpy(ptr + 12, &intensity, sizeof(float));
        std::memcpy(ptr + 16, &offset_ns, sizeof(uint32_t));
        std::memcpy(ptr + 20, &line, sizeof(uint8_t));
        std::memcpy(ptr + 21, &tag, sizeof(uint8_t));
    }

    return msg;
}

// ============================================================
// SIM tests
// ============================================================

TEST(PointCloud2DecoderSimTest, AcceptsValidXyziSnapshot) {
    PointCloudDecoderConfig config;
    config.profile = LidarInputProfile::kSimXyziSnapshot;
    config.time_field = "";
    config.lidar_frame = "mid360_lidar";
    config.min_range_m = 0.5;
    config.max_range_m = 100.0;

    PointCloud2Decoder decoder(config);

    auto cloud = makeSimXyziCloud({
        {1.0f, 0.0f, 0.0f, 10.0f},
        {0.0f, 2.0f, 0.0f, 20.0f},
        {0.0f, 0.0f, 3.0f, 30.0f},
    });

    DecodeResult result = decoder.decode(cloud);

    ASSERT_TRUE(result.ok()) << result.errorMessage();
    EXPECT_EQ(result.scan.cloud->points.size(), 3u);
    EXPECT_FALSE(result.scan.has_per_point_time);
    EXPECT_NEAR(static_cast<double>(result.scan.scan_start_time_ns) * 1e-9, 10.0, 1e-9);
    EXPECT_NEAR(static_cast<double>(result.scan.scan_end_time_ns) * 1e-9, 10.0, 1e-9);

    for (const auto& pt : result.scan.cloud->points) {
        EXPECT_FLOAT_EQ(pt.curvature, 0.0f);
        EXPECT_TRUE(std::isfinite(pt.x));
        EXPECT_TRUE(std::isfinite(pt.y));
        EXPECT_TRUE(std::isfinite(pt.z));
    }
}

TEST(PointCloud2DecoderSimTest, RejectsNaNAndInfPoints) {
    PointCloudDecoderConfig config;
    config.profile = LidarInputProfile::kSimXyziSnapshot;
    config.min_range_m = 0.5;
    config.max_range_m = 100.0;

    PointCloud2Decoder decoder(config);

    const float nan_val = std::numeric_limits<float>::quiet_NaN();
    const float inf_val = std::numeric_limits<float>::infinity();

    auto cloud = makeSimXyziCloud({
        {nan_val, 0.0f, 0.0f, 10.0f},   // NaN x
        {1.0f, 0.0f, 0.0f, 20.0f},       // valid
        {0.0f, 0.0f, inf_val, 30.0f},   // Inf z
        {0.0f, 5.0f, 0.0f, 40.0f},       // valid
    });

    DecodeResult result = decoder.decode(cloud);

    ASSERT_TRUE(result.ok());
    EXPECT_EQ(result.scan.cloud->points.size(), 2u);
}

TEST(PointCloud2DecoderSimTest, RangeFilterWorks) {
    PointCloudDecoderConfig config;
    config.profile = LidarInputProfile::kSimXyziSnapshot;
    config.min_range_m = 1.0;
    config.max_range_m = 5.0;

    PointCloud2Decoder decoder(config);

    auto cloud = makeSimXyziCloud({
        {0.1f, 0.0f, 0.0f, 10.0f},  // range 0.1 < 1.0 → reject
        {2.0f, 0.0f, 0.0f, 20.0f},  // range 2.0 → accept
        {10.0f, 0.0f, 0.0f, 30.0f}, // range 10.0 > 5.0 → reject
    });

    DecodeResult result = decoder.decode(cloud);

    ASSERT_TRUE(result.ok());
    EXPECT_EQ(result.scan.cloud->points.size(), 1u);
    EXPECT_FLOAT_EQ(result.scan.cloud->points[0].x, 2.0f);
}

TEST(PointCloud2DecoderSimTest, RejectsEmptyCloud) {
    PointCloudDecoderConfig config;
    config.profile = LidarInputProfile::kSimXyziSnapshot;

    PointCloud2Decoder decoder(config);

    sensor_msgs::msg::PointCloud2 empty_msg;
    empty_msg.width = 0;
    empty_msg.height = 0;

    DecodeResult result = decoder.decode(empty_msg);

    EXPECT_FALSE(result.ok());
    EXPECT_EQ(result.error, DecodeError::kEmptyCloud);
}

TEST(PointCloud2DecoderSimTest, RejectsMissingXyzField) {
    PointCloudDecoderConfig config;
    config.profile = LidarInputProfile::kSimXyziSnapshot;

    PointCloud2Decoder decoder(config);

    sensor_msgs::msg::PointCloud2 msg;
    msg.width = 1;
    msg.height = 1;
    msg.point_step = 4;
    msg.data.resize(4);
    // No fields defined

    DecodeResult result = decoder.decode(msg);

    EXPECT_FALSE(result.ok());
    EXPECT_EQ(result.error, DecodeError::kMissingXyzField);
}

// ============================================================
// REAL MID-360 tests
// ============================================================

TEST(PointCloud2DecoderMid360Test, ParsesOffsetTimeNanoseconds) {
    PointCloudDecoderConfig config;
    config.profile = LidarInputProfile::kMid360PointCloud2;
    config.time_field = "offset_time";
    config.time_unit = TimeUnit::kNanoseconds;
    config.header_stamp_is_scan_start = true;
    config.require_per_point_time = true;
    config.intensity_field = "reflectivity";
    config.lidar_frame = "mid360_lidar";
    config.min_range_m = 0.5;
    config.max_range_m = 100.0;
    config.filter_livox_tags = false;

    PointCloud2Decoder decoder(config);

    auto cloud = makeMid360Cloud({
        {1.0f, 0.0f, 0.0f, 10.0f, 0u,         0, 0x10},
        {0.0f, 2.0f, 0.0f, 20.0f, 10'000'000u, 1, 0x10},
        {0.0f, 0.0f, 3.0f, 30.0f, 20'000'000u, 2, 0x10},
    });

    DecodeResult result = decoder.decode(cloud);

    ASSERT_TRUE(result.ok()) << result.errorMessage();
    EXPECT_TRUE(result.scan.has_per_point_time);
    EXPECT_EQ(result.scan.cloud->points.size(), 3u);

    // Per-point time should be in seconds, sorted ascending
    EXPECT_NEAR(result.scan.cloud->points[0].curvature, 0.0, 1e-6);
    EXPECT_NEAR(result.scan.cloud->points[1].curvature, 0.01, 1e-6);
    EXPECT_NEAR(result.scan.cloud->points[2].curvature, 0.02, 1e-6);

    // Scan bounds: header=10.0 (start), max_rel_time=0.02s
    EXPECT_NEAR(static_cast<double>(result.scan.scan_start_time_ns) * 1e-9, 10.0, 1e-9);
    EXPECT_NEAR(static_cast<double>(result.scan.scan_end_time_ns) * 1e-9, 10.02, 1e-9);
}

TEST(PointCloud2DecoderMid360Test, SortsUnorderedTime) {
    PointCloudDecoderConfig config;
    config.profile = LidarInputProfile::kMid360PointCloud2;
    config.time_field = "offset_time";
    config.time_unit = TimeUnit::kNanoseconds;
    config.require_per_point_time = true;
    config.intensity_field = "reflectivity";
    config.min_range_m = 0.5;
    config.max_range_m = 100.0;

    PointCloud2Decoder decoder(config);

    // Points with out-of-order time
    auto cloud = makeMid360Cloud({
        {0.0f, 0.0f, 3.0f, 30.0f, 20'000'000u, 2, 0x10},  // t=0.02s
        {1.0f, 0.0f, 0.0f, 10.0f, 0u,         0, 0x10},  // t=0.00s
        {0.0f, 2.0f, 0.0f, 20.0f, 10'000'000u, 1, 0x10},  // t=0.01s
    });

    DecodeResult result = decoder.decode(cloud);

    ASSERT_TRUE(result.ok());
    EXPECT_TRUE(result.scan.has_per_point_time);
    EXPECT_EQ(result.scan.cloud->points.size(), 3u);

    // Points should be sorted by time
    EXPECT_LE(result.scan.cloud->points[0].curvature,
              result.scan.cloud->points[1].curvature);
    EXPECT_LE(result.scan.cloud->points[1].curvature,
              result.scan.cloud->points[2].curvature);

    // Positions should follow their time ordering
    EXPECT_FLOAT_EQ(result.scan.cloud->points[0].x, 1.0f);  // t=0
    EXPECT_FLOAT_EQ(result.scan.cloud->points[1].y, 2.0f);  // t=0.01
    EXPECT_FLOAT_EQ(result.scan.cloud->points[2].z, 3.0f);  // t=0.02
}

TEST(PointCloud2DecoderMid360Test, RejectsMissingTimeFieldWhenRequired) {
    PointCloudDecoderConfig config;
    config.profile = LidarInputProfile::kMid360PointCloud2;
    config.time_field = "offset_time";
    config.require_per_point_time = true;
    config.intensity_field = "reflectivity";

    PointCloud2Decoder decoder(config);

    // Build a cloud without offset_time field
    auto cloud = makeSimXyziCloud({
        {1.0f, 0.0f, 0.0f, 10.0f},
    });

    DecodeResult result = decoder.decode(cloud);

    EXPECT_FALSE(result.ok());
    EXPECT_EQ(result.error, DecodeError::kMissingTimeField);
}

TEST(PointCloud2DecoderMid360Test, AcceptsMissingTimeFieldWhenNotRequired) {
    PointCloudDecoderConfig config;
    config.profile = LidarInputProfile::kMid360PointCloud2;
    config.time_field = "offset_time";
    config.require_per_point_time = false;
    config.intensity_field = "intensity";

    PointCloud2Decoder decoder(config);

    auto cloud = makeSimXyziCloud({
        {1.0f, 0.0f, 0.0f, 10.0f},
    });

    DecodeResult result = decoder.decode(cloud);

    ASSERT_TRUE(result.ok());
    EXPECT_FALSE(result.scan.has_per_point_time);
    EXPECT_FLOAT_EQ(result.scan.cloud->points[0].curvature, 0.0f);
}

TEST(PointCloud2DecoderMid360Test, FiltersLivoxTags) {
    PointCloudDecoderConfig config;
    config.profile = LidarInputProfile::kMid360PointCloud2;
    config.time_field = "offset_time";
    config.time_unit = TimeUnit::kNanoseconds;
    config.intensity_field = "reflectivity";
    config.filter_livox_tags = true;
    config.min_range_m = 0.5;
    config.max_range_m = 100.0;

    PointCloud2Decoder decoder(config);

    auto cloud = makeMid360Cloud({
        {1.0f, 0.0f, 0.0f, 10.0f, 0u, 0, 0x10},  // valid tag
        {0.0f, 2.0f, 0.0f, 20.0f, 0u, 1, 0x20},  // invalid tag (0x20)
        {0.0f, 0.0f, 3.0f, 30.0f, 0u, 2, 0x00},  // valid tag
    });

    DecodeResult result = decoder.decode(cloud);

    ASSERT_TRUE(result.ok());
    EXPECT_EQ(result.scan.cloud->points.size(), 2u);  // 0x20 rejected
}

TEST(PointCloud2DecoderMid360Test, RejectsExcessiveScanDuration) {
    PointCloudDecoderConfig config;
    config.profile = LidarInputProfile::kMid360PointCloud2;
    config.time_field = "offset_time";
    config.time_unit = TimeUnit::kNanoseconds;
    config.require_per_point_time = true;
    config.intensity_field = "reflectivity";
    config.max_scan_duration_s = 0.1;  // 100ms max

    PointCloud2Decoder decoder(config);

    // Time spans 200ms > 100ms max
    auto cloud = makeMid360Cloud({
        {1.0f, 0.0f, 0.0f, 10.0f, 0u,          0, 0x10},
        {0.0f, 2.0f, 0.0f, 20.0f, 200'000'000u, 1, 0x10},
    });

    DecodeResult result = decoder.decode(cloud);

    EXPECT_FALSE(result.ok());
    EXPECT_EQ(result.error, DecodeError::kInvalidScanDuration);
}

TEST(PointCloud2DecoderMid360Test, RejectsAllInvalidPoints) {
    PointCloudDecoderConfig config;
    config.profile = LidarInputProfile::kMid360PointCloud2;
    config.time_field = "offset_time";
    config.time_unit = TimeUnit::kNanoseconds;
    config.intensity_field = "reflectivity";
    config.min_range_m = 10.0;  // All points closer than 10m → rejected

    PointCloud2Decoder decoder(config);

    auto cloud = makeMid360Cloud({
        {1.0f, 0.0f, 0.0f, 10.0f, 0u, 0, 0x10},
        {0.0f, 2.0f, 0.0f, 20.0f, 0u, 1, 0x10},
    });

    DecodeResult result = decoder.decode(cloud);

    EXPECT_FALSE(result.ok());
    EXPECT_EQ(result.error, DecodeError::kAllPointsInvalid);
}

TEST(PointCloud2DecoderMid360Test, FrameMismatchWarnsButAccepts) {
    PointCloudDecoderConfig config;
    config.profile = LidarInputProfile::kMid360PointCloud2;
    config.lidar_frame = "expected_frame";
    config.min_range_m = 0.5;
    config.max_range_m = 100.0;

    PointCloud2Decoder decoder(config);

    auto cloud = makeSimXyziCloud({{1.0f, 0.0f, 0.0f, 10.0f}}, 10.0, "wrong_frame");

    DecodeResult result = decoder.decode(cloud);

    // Should still accept (warn only)
    ASSERT_TRUE(result.ok());
    EXPECT_EQ(result.scan.lidar_frame, "wrong_frame");
}

// ============================================================
// Time unit conversion tests
// ============================================================

TEST(TimeUnitConversionTest, ConvertsAllUnits) {
    EXPECT_DOUBLE_EQ(timeToSeconds(1.0, TimeUnit::kSeconds), 1.0);
    EXPECT_NEAR(timeToSeconds(1.0, TimeUnit::kMilliseconds), 0.001, 1e-12);
    EXPECT_NEAR(timeToSeconds(1.0, TimeUnit::kMicroseconds), 1e-6, 1e-12);
    EXPECT_NEAR(timeToSeconds(1.0, TimeUnit::kNanoseconds), 1e-9, 1e-12);
}

// ============================================================
// NormalizedLidarScan contract tests
// ============================================================

TEST(NormalizedLidarScanTest, DefaultConstructsEmpty) {
    NormalizedLidarScan scan;
    EXPECT_TRUE(scan.cloud->empty());
    EXPECT_DOUBLE_EQ(static_cast<double>(scan.scan_start_time_ns) * 1e-9, 0.0);
    EXPECT_DOUBLE_EQ(static_cast<double>(scan.scan_end_time_ns) * 1e-9, 0.0);
    EXPECT_FALSE(scan.has_per_point_time);
    EXPECT_TRUE(scan.lidar_frame.empty());
}

// ============================================================
// Diagnostics tests
// ============================================================

TEST(PointCloud2DecoderDiagnosticsTest, TracksAcceptedAndRejected) {
    PointCloudDecoderConfig config;
    config.profile = LidarInputProfile::kSimXyziSnapshot;
    config.min_range_m = 0.5;
    config.max_range_m = 100.0;

    PointCloud2Decoder decoder(config);

    // Accept one valid scan
    auto good_cloud = makeSimXyziCloud({{1.0f, 0.0f, 0.0f, 10.0f}});
    decoder.decode(good_cloud);

    // Reject one empty scan
    sensor_msgs::msg::PointCloud2 empty_msg;
    empty_msg.width = 0;
    empty_msg.height = 0;
    decoder.decode(empty_msg);

    const auto& diag = decoder.diagnostics();
    EXPECT_EQ(diag.received_scans, 2u);
    EXPECT_EQ(diag.accepted_scans, 1u);
    EXPECT_EQ(diag.rejected_scans, 1u);
    EXPECT_GT(diag.valid_points, 0u);
}

}  // namespace
}  // namespace fast_lio