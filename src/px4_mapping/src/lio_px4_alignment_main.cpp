// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2026, LeTanDat

#include <memory>

#include <rclcpp/rclcpp.hpp>

#include "px4_mapping/lio_px4_alignment.hpp"

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<px4_mapping::LioPx4Alignment>());
    rclcpp::shutdown();
    return 0;
}
