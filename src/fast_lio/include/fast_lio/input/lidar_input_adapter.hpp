// Copyright (c) 2026, LeTanDat
// SPDX-License-Identifier: BSD-3-Clause

#ifndef FAST_LIO_INPUT_LIDAR_INPUT_ADAPTER_HPP_
#define FAST_LIO_INPUT_LIDAR_INPUT_ADAPTER_HPP_

#include <memory>
#include <string>

#include <sensor_msgs/msg/point_cloud2.hpp>

#include "fast_lio/lidar_scan.hpp"

namespace fast_lio {

/// @brief Abstract interface for LiDAR input adapters.
///
/// An adapter owns the sensor-specific decoding logic and exposes a normalized
/// NormalizedLidarScan. The FAST-LIO node is decoupled from the concrete message
/// format (PointCloud2, Livox CustomMsg, etc.).
class LidarInputAdapter {
   public:
    virtual ~LidarInputAdapter() = default;

    /// @brief Decode a sensor_msgs::PointCloud2 scan.
    ///
    /// Subclasses that consume a different ROS message type may implement a
    /// separate decode overload; this interface covers the PointCloud2-based
    /// adapters used in both simulation and MID-360 PointCloud2 mode.
    virtual DecodeResult decode(const sensor_msgs::msg::PointCloud2& msg) = 0;

    /// @brief Human-readable adapter name for diagnostics.
    virtual std::string name() const = 0;

    /// @brief Current input diagnostics.
    virtual const LidarInputDiagnostics& diagnostics() const = 0;
};

}  // namespace fast_lio

#endif  // FAST_LIO_INPUT_LIDAR_INPUT_ADAPTER_HPP_
