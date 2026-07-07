// Copyright 2026 CTUAV. All rights reserved.
//
// Main entry point for the Livox MID-360 processor node.

#include "px4_navigation/livox_mid360_processor.hpp"

#include <rclcpp/rclcpp.hpp>

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);

    rclcpp::NodeOptions options;
    auto node = std::make_shared<px4_navigation::LivoxMid360Processor>(options);

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
