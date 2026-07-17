// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2026, LeTanDat

#include <memory>

#include <rclcpp/rclcpp.hpp>

#include "px4_mapping/lio_px4_bridge.hpp"

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<px4_mapping::LioPx4Bridge>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
