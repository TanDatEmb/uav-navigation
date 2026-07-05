// Copyright 2026 CTUAV. All rights reserved.
//
// Main function for the NED transform node.

#include <memory>

#include "px4_mapping/ned_transform_node.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<px4_mapping::NedTransformNode>());
    rclcpp::shutdown();
    return 0;
}