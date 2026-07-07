// Copyright 2026 TanDatEmb.
//
// Main entry point for the obstacle distance visualizer node.

#include <memory>

#include "px4_navigation/obstacle_distance_visualizer.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<px4_navigation::ObstacleDistanceVisualizer>());
    rclcpp::shutdown();
    return 0;
}