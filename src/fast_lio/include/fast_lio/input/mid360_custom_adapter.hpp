// Copyright (c) 2026, LeTanDat
// SPDX-License-Identifier: BSD-3-Clause

#ifndef FAST_LIO_INPUT_MID360_CUSTOM_ADAPTER_HPP_
#define FAST_LIO_INPUT_MID360_CUSTOM_ADAPTER_HPP_

#include <memory>
#include <stdexcept>
#include <string>

#include "fast_lio/input/lidar_input_adapter.hpp"

namespace fast_lio {

/// @brief Factory for the MID-360 Livox CustomMsg adapter.
///
/// The Livox MID-360 hardware driver publishes livox_ros_driver2::msg::CustomMsg,
/// which carries per-point timestamp, line, and tag information in a single
/// message. This adapter converts that message into a NormalizedLidarScan.
///
/// Because livox_ros_driver2 is an optional dependency, the implementation is
/// only available when the workspace is built with LIVOX_ROS2_FOUND. When the
/// driver is absent, the factory throws std::runtime_error with a clear message
/// explaining that the package is required for hardware input.
///
/// To wire this adapter, the FAST-LIO node must subscribe to
/// livox_ros_driver2::msg::CustomMsg and call decode(msg) on the returned
/// adapter. The current PointCloud2 subscription path is used for SITL and for
/// MID-360 PointCloud2 replay.
std::unique_ptr<LidarInputAdapter> makeMid360CustomAdapter();

}  // namespace fast_lio

#endif  // FAST_LIO_INPUT_MID360_CUSTOM_ADAPTER_HPP_
