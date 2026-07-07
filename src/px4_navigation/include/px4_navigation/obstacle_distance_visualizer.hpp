// Copyright 2026 CTUAV. All rights reserved.
//
// ObstacleDistance visualizer — converts /fmu/in/obstacle_distance into
// visualization_msgs/MarkerArray for RViz.
//
// Publishes 72 arrow markers (one per bin) around the UAV in body FRD
// frame. Arrow length = reported distance, color = red (close) → green (far).

#ifndef PX4_NAVIGATION_OBSTACLE_DISTANCE_VISUALIZER_HPP_
#define PX4_NAVIGATION_OBSTACLE_DISTANCE_VISUALIZER_HPP_

#include <array>
#include <cstdint>
#include <memory>
#include <string>

#include <px4_msgs/msg/obstacle_distance.hpp>
#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

namespace px4_navigation {

class ObstacleDistanceVisualizer : public rclcpp::Node {
   public:
    static constexpr int kNumBins = 72;
    static constexpr double kIncrementDeg = 5.0;
    static constexpr uint16_t kNoObstacle = 65535;

    explicit ObstacleDistanceVisualizer(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

   private:
    void ObstacleDistanceCallback(const px4_msgs::msg::ObstacleDistance::SharedPtr msg);
    void PublishMarkers();

    rclcpp::Subscription<px4_msgs::msg::ObstacleDistance>::SharedPtr sub_obstacle_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_markers_;
    rclcpp::TimerBase::SharedPtr timer_;

    std::array<uint16_t, kNumBins> distances_{};
    bool obstacle_valid_{false};

    uint64_t sequence_{0};
};

}  // namespace px4_navigation

#endif  // PX4_NAVIGATION_OBSTACLE_DISTANCE_VISUALIZER_HPP_