// Copyright 2026 CTUAV. All rights reserved.
//
// Main entry point for the obstacle distance publisher node.

#include <memory>

#include "px4_navigation/obstacle_distance_publisher.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<px4_navigation::ObstacleDistancePublisher>());
    rclcpp::shutdown();
    return 0;
}