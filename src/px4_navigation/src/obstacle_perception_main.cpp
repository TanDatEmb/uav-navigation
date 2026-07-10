// Copyright 2026 TanDatEmb.
//
// Main entry point for the Livox MID-360 processor node.

#include "px4_navigation/obstacle_perception.hpp"

#include <rclcpp/rclcpp.hpp>

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);

    rclcpp::NodeOptions options;
    auto node = std::make_shared<px4_navigation::ObstaclePerception>(options);

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
