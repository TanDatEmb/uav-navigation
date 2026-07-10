// Copyright 2026 CTUAV. All rights reserved.
//
// Main function for the voxel map manager node.

#include <memory>

#include "px4_mapping/global_mapper.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<px4_mapping::GlobalMapper>());
    rclcpp::shutdown();
    return 0;
}