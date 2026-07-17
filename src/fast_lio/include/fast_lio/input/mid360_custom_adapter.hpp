// Copyright (c) 2026, LeTanDat
// SPDX-License-Identifier: BSD-3-Clause

#ifndef FAST_LIO_INPUT_MID360_CUSTOM_ADAPTER_HPP_
#define FAST_LIO_INPUT_MID360_CUSTOM_ADAPTER_HPP_

#include <memory>
#include <string>

#include "fast_lio/input/lidar_input_adapter.hpp"
#include "fast_lio/lidar_scan.hpp"

namespace fast_lio {

#ifdef LIVOX_ROS2_FOUND

/// @brief Adapter for Livox ROS Driver 2 CustomMsg (MID-360 hardware).
///
/// The Livox MID-360 hardware driver publishes livox_ros_driver2::msg::CustomMsg,
/// which carries a message-level `timebase` (nanoseconds) plus per-point
/// `offset_time` (nanoseconds), `tag`, `line`, and `reflectivity`.
///
/// This adapter converts the message into a NormalizedLidarScan. Points are
/// filtered by range and Livox tag; relative timestamps are written to
/// PointType::curvature as seconds from scan start.
class Mid360CustomAdapter : public LidarInputAdapter {
   public:
    Mid360CustomAdapter();

    DecodeResult decode(const sensor_msgs::msg::PointCloud2& msg) override;
    DecodeResult decode(const livox_ros_driver2::msg::CustomMsg& msg) override;

    std::string name() const override;

    const LidarInputDiagnostics& diagnostics() const override;

   private:
    LidarInputDiagnostics diagnostics_;
};

#endif  // LIVOX_ROS2_FOUND

/// @brief Factory for the MID-360 Livox CustomMsg adapter.
///
/// Throws std::runtime_error if the workspace was not built with
/// LIVOX_ROS2_FOUND.
std::unique_ptr<LidarInputAdapter> makeMid360CustomAdapter();

}  // namespace fast_lio

#endif  // FAST_LIO_INPUT_MID360_CUSTOM_ADAPTER_HPP_
