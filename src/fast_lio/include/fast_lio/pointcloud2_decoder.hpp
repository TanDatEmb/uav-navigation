// Copyright (c) 2026, LeTanDat
// SPDX-License-Identifier: BSD-3-Clause

#ifndef FAST_LIO_POINTCLOUD2_DECODER_HPP_
#define FAST_LIO_POINTCLOUD2_DECODER_HPP_

#include <memory>
#include <string>

#include <sensor_msgs/msg/point_cloud2.hpp>

#include "fast_lio/lidar_scan.hpp"

namespace fast_lio {

/// @brief Decodes sensor_msgs::PointCloud2 into NormalizedLidarScan.
///
/// Single class, config-driven. Handles both SIM snapshot and REAL MID-360
/// profiles by configuration, not by inheritance.
///
/// Pipeline:
///   1. Schema inspection (validate required fields, datatypes)
///   2. Per-point decode (XYZ, intensity, time, tag, line)
///   3. Time unit conversion to seconds
///   4. Finite / range / tag filtering
///   5. Sort by relative time (if has_per_point_time)
///   6. Compute scan_start/end_time
///
/// Output points remain in the LiDAR sensor frame. No transform, no deskew.
class PointCloud2Decoder {
   public:
    explicit PointCloud2Decoder(PointCloudDecoderConfig config);

    /// Decode a PointCloud2 message into a NormalizedLidarScan.
    DecodeResult decode(const sensor_msgs::msg::PointCloud2& msg);

    /// Access the current diagnostics.
    const LidarInputDiagnostics& diagnostics() const { return diagnostics_; }

    /// Access the current configuration.
    const PointCloudDecoderConfig& config() const { return config_; }

   private:
    PointCloudDecoderConfig config_;
    LidarInputDiagnostics diagnostics_;
    bool schema_logged_ = false;

    // Schema inspection helpers
    bool findField(const sensor_msgs::msg::PointCloud2& msg, const std::string& name,
                   sensor_msgs::msg::PointField& out) const;

    /// Read a scalar value from a point's raw data at a given offset.
    /// Supports FLOAT32, FLOAT64, UINT8, UINT16, UINT32, UINT64.
    double readFieldValue(const std::uint8_t* data_ptr,
                          const sensor_msgs::msg::PointField& field) const;

    /// Compute scan time bounds from header and per-point times.
    void computeScanTimeBounds(NormalizedLidarScan& scan,
                              double header_time, double max_rel_time) const;

    /// Log schema summary once on first accepted scan.
    void logSchemaOnce(const sensor_msgs::msg::PointCloud2& msg);
};

}  // namespace fast_lio

#endif  // FAST_LIO_POINTCLOUD2_DECODER_HPP_