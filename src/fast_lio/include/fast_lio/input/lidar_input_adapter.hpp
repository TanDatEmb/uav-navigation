// Copyright (c) 2026, LeTanDat
// SPDX-License-Identifier: BSD-3-Clause

#ifndef FAST_LIO_INPUT_LIDAR_INPUT_ADAPTER_HPP_
#define FAST_LIO_INPUT_LIDAR_INPUT_ADAPTER_HPP_

#include <memory>
#include <string>

#include <sensor_msgs/msg/point_cloud2.hpp>

#include "fast_lio/lidar_scan.hpp"

#ifdef LIVOX_ROS2_FOUND
#include <livox_ros_driver2/msg/custom_msg.hpp>
#endif

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
    virtual DecodeResult decode(const sensor_msgs::msg::PointCloud2& msg) = 0;

#ifdef LIVOX_ROS2_FOUND
    /// @brief Decode a Livox ROS Driver 2 CustomMsg scan.
    ///
    /// Only adapters that consume livox_ros_driver2::msg::CustomMsg need to
    /// implement this overload. PointCloud2-based adapters inherit the default
    /// error implementation.
    virtual DecodeResult decode(const livox_ros_driver2::msg::CustomMsg& msg) {
        DecodeResult result;
        result.error = DecodeError::kSchemaMismatch;
        return result;
    }
#endif

    /// @brief Human-readable adapter name for diagnostics.
    virtual std::string name() const = 0;

    /// @brief Current input diagnostics.
    virtual const LidarInputDiagnostics& diagnostics() const = 0;
};

}  // namespace fast_lio

#endif  // FAST_LIO_INPUT_LIDAR_INPUT_ADAPTER_HPP_
