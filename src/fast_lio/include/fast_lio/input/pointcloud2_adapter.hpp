// Copyright (c) 2026, LeTanDat
// SPDX-License-Identifier: BSD-3-Clause

#ifndef FAST_LIO_INPUT_POINTCLOUD2_ADAPTER_HPP_
#define FAST_LIO_INPUT_POINTCLOUD2_ADAPTER_HPP_

#include <memory>
#include <string>

#include "fast_lio/input/lidar_input_adapter.hpp"
#include "fast_lio/lidar_scan.hpp"
#include "fast_lio/pointcloud2_decoder.hpp"

namespace fast_lio {

/// @brief Adapter for sensor_msgs::PointCloud2 inputs.
///
/// Wraps PointCloud2Decoder and exposes it through the LidarInputAdapter
/// interface. Two factory presets are provided for the supported SITL and
/// hardware-replay profiles:
///   - SimSnapshotAdapter: Gazebo GPU-LiDAR XYZI snapshot, no per-point time.
///   - Mid360PointCloud2Adapter: MID-360 PointCloud2 with offset_time + tag/line.
class PointCloud2Adapter : public LidarInputAdapter {
   public:
    PointCloud2Adapter(PointCloudDecoderConfig config, std::string name);

    DecodeResult decode(const sensor_msgs::msg::PointCloud2& msg) override;

    std::string name() const override;

    const LidarInputDiagnostics& diagnostics() const override;

    const PointCloudDecoderConfig& config() const { return decoder_.config(); }

   private:
    PointCloud2Decoder decoder_;
    std::string name_;
};

/// @brief Factory for the Gazebo simulation snapshot adapter.
std::unique_ptr<LidarInputAdapter> makeSimSnapshotAdapter();

/// @brief Factory for the MID-360 PointCloud2 adapter.
std::unique_ptr<LidarInputAdapter> makeMid360PointCloud2Adapter();

}  // namespace fast_lio

#endif  // FAST_LIO_INPUT_POINTCLOUD2_ADAPTER_HPP_
