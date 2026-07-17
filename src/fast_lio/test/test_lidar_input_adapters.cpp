// Copyright (c) 2026, LeTanDat
// SPDX-License-Identifier: BSD-3-Clause

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <limits>
#include <tuple>
#include <vector>

#include "fast_lio/input/lidar_input_adapter.hpp"
#include "fast_lio/input/mid360_custom_adapter.hpp"
#include "fast_lio/input/pointcloud2_adapter.hpp"
#include "fast_lio/lidar_scan.hpp"

#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_field.hpp>

namespace fast_lio {
namespace {

sensor_msgs::msg::PointCloud2 makeXyziCloud(
    const std::vector<std::tuple<float, float, float, float>>& points,
    double header_time_sec = 10.0,
    const std::string& frame_id = "mid360_lidar") {
    sensor_msgs::msg::PointCloud2 msg;
    msg.header.stamp.sec = static_cast<int32_t>(header_time_sec);
    msg.header.stamp.nanosec = static_cast<uint32_t>(
        (header_time_sec - static_cast<int32_t>(header_time_sec)) * 1e9);
    msg.header.frame_id = frame_id;

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
    msg.point_step = 16;
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

sensor_msgs::msg::PointCloud2 makeMid360Cloud(
    const std::vector<std::tuple<float, float, float, float, uint32_t, uint8_t, uint8_t>>& points,
    double header_time_sec = 10.0,
    const std::string& frame_id = "mid360_lidar") {
    sensor_msgs::msg::PointCloud2 msg;
    msg.header.stamp.sec = static_cast<int32_t>(header_time_sec);
    msg.header.stamp.nanosec = static_cast<uint32_t>(
        (header_time_sec - static_cast<int32_t>(header_time_sec)) * 1e9);
    msg.header.frame_id = frame_id;

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
    msg.point_step = 24;
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

TEST(LidarInputAdapterTest, SimSnapshotAdapterDecodesXyzi) {
    auto adapter = makeSimSnapshotAdapter();
    ASSERT_NE(adapter, nullptr);
    EXPECT_EQ(adapter->name(), "sim_snapshot");

    auto cloud = makeXyziCloud({
        {1.0f, 0.0f, 0.0f, 10.0f},
        {0.0f, 2.0f, 0.0f, 20.0f},
        {0.0f, 0.0f, 3.0f, 30.0f},
    });

    DecodeResult result = adapter->decode(cloud);
    ASSERT_TRUE(result.ok()) << result.errorMessage();
    EXPECT_EQ(result.scan.cloud->points.size(), 3u);
    EXPECT_FALSE(result.scan.has_per_point_time);
    EXPECT_NEAR(result.scan.scan_start_time_s, 10.0, 1e-6);
}

TEST(LidarInputAdapterTest, Mid360PointCloud2AdapterRequiresOffsetTime) {
    auto adapter = makeMid360PointCloud2Adapter();
    ASSERT_NE(adapter, nullptr);
    EXPECT_EQ(adapter->name(), "mid360_pointcloud2");

    // XYZI cloud without offset_time must be rejected by this adapter.
    auto cloud = makeXyziCloud({{1.0f, 0.0f, 0.0f, 10.0f}});
    DecodeResult result = adapter->decode(cloud);
    EXPECT_FALSE(result.ok());
}

TEST(LidarInputAdapterTest, Mid360PointCloud2AdapterDecodesTimedCloud) {
    auto adapter = makeMid360PointCloud2Adapter();

    auto cloud = makeMid360Cloud({
        {1.0f, 0.0f, 0.0f, 10.0f, 0u, 0u, 0x10u},
        {0.0f, 2.0f, 0.0f, 20.0f, 50000000u, 1u, 0x10u},
        {0.0f, 0.0f, 3.0f, 30.0f, 100000000u, 2u, 0x10u},
    });

    DecodeResult result = adapter->decode(cloud);
    ASSERT_TRUE(result.ok()) << result.errorMessage();
    EXPECT_EQ(result.scan.cloud->points.size(), 3u);
    EXPECT_TRUE(result.scan.has_per_point_time);
    EXPECT_NEAR(result.scan.scan_start_time_s, 10.0, 1e-6);
    EXPECT_NEAR(result.scan.scan_end_time_s, 10.1, 1e-6);

    // Points are sorted by ascending relative time.
    for (size_t i = 1; i < result.scan.cloud->points.size(); ++i) {
        EXPECT_GE(result.scan.cloud->points[i].curvature,
                  result.scan.cloud->points[i - 1].curvature);
    }
}

TEST(LidarInputAdapterTest, Mid360CustomAdapterThrowsWithoutLivox) {
    // In this test environment livox_ros_driver2 is not present, so the
    // factory must throw before the node attempts to subscribe.
    EXPECT_THROW(makeMid360CustomAdapter(), std::runtime_error);
}

}  // namespace
}  // namespace fast_lio
