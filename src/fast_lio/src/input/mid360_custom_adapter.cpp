// Copyright (c) 2026, LeTanDat
// SPDX-License-Identifier: BSD-3-Clause

#include "fast_lio/input/mid360_custom_adapter.hpp"

namespace fast_lio {

std::unique_ptr<LidarInputAdapter> makeMid360CustomAdapter() {
    throw std::runtime_error(
        "Mid360CustomMsgAdapter requires livox_ros_driver2. "
        "Install the driver and rebuild with LIVOX_ROS2_FOUND, "
        "or use the mid360_pointcloud2 / sim_snapshot adapter instead.");
}

}  // namespace fast_lio
