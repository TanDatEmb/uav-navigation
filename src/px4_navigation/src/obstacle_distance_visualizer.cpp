// Copyright 2026 CTUAV. All rights reserved.

#include "px4_navigation/obstacle_distance_visualizer.hpp"

#include <array>
#include <cmath>
#include <limits>

#include <geometry_msgs/msg/point.hpp>
#include <px4_common/math/transforms.hpp>
#include <px4_msgs/msg/obstacle_distance.hpp>
#include <std_msgs/msg/color_rgba.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

namespace px4_navigation {

namespace {

// Distance -> danger color (RGBA). Three clear levels; no smooth gradient.
std::array<float, 4> ColorFromDistance(float dist_m) {
    if (dist_m < 3.0f) {
        return {1.0f, 0.0f, 0.0f, 0.9f};  // red: danger
    }
    if (dist_m < 8.0f) {
        return {1.0f, 0.5f, 0.0f, 0.8f};  // orange: warning
    }
    return {0.0f, 0.5f, 1.0f, 0.7f};      // light blue: safe
}

}  // namespace

ObstacleDistanceVisualizer::ObstacleDistanceVisualizer(const rclcpp::NodeOptions& options)
    : rclcpp::Node("obstacle_distance_visualizer", options) {
    const auto px4_qos = rclcpp::QoS(20).best_effort();

    sub_obstacle_ = this->create_subscription<px4_msgs::msg::ObstacleDistance>(
        "/fmu/in/obstacle_distance", px4_qos,
        std::bind(&ObstacleDistanceVisualizer::ObstacleDistanceCallback, this,
                  std::placeholders::_1));

    pub_markers_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
        "/obstacle_distance/markers", rclcpp::QoS(20).reliable());

    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(100),
        std::bind(&ObstacleDistanceVisualizer::PublishMarkers, this));

    RCLCPP_INFO(this->get_logger(),
                "obstacle_distance_visualizer started: publishing %d bounded line markers",
                kNumBins);
}

void ObstacleDistanceVisualizer::ObstacleDistanceCallback(
    const px4_msgs::msg::ObstacleDistance::SharedPtr msg) {
    for (int i = 0; i < kNumBins; ++i) {
        distances_[i] = msg->distances[i];
    }
    obstacle_valid_ = true;
}

void ObstacleDistanceVisualizer::PublishMarkers() {
    if (!obstacle_valid_) {
        return;
    }

    auto msg = visualization_msgs::msg::MarkerArray();
    msg.markers.reserve(2);

    // Delete previous markers.
    {
        auto delete_all = visualization_msgs::msg::Marker();
        delete_all.header.frame_id = "lidar_sensor_link";
        delete_all.header.stamp = this->now();
        delete_all.action = visualization_msgs::msg::Marker::DELETEALL;
        delete_all.ns = "obstacle_distance";
        msg.markers.push_back(delete_all);
    }

    // Bounded line-list: one thin line per bin from UAV to obstacle.
    auto lines = visualization_msgs::msg::Marker();
    lines.header.frame_id = "lidar_sensor_link";
    lines.header.stamp = this->now();
    lines.ns = "obstacle_distance";
    lines.id = 1;
    lines.type = visualization_msgs::msg::Marker::LINE_LIST;
    lines.action = visualization_msgs::msg::Marker::ADD;
    lines.pose.orientation.w = 1.0;
    lines.scale.x = 0.04;  // line width

    constexpr float kMaxDrawDistance = 5.0f;

    for (int i = 0; i < kNumBins; ++i) {
        const uint16_t dist_cm = distances_[i];
        if (dist_cm == 0 || dist_cm >= kNoObstacle) {
            continue;
        }

        const float dist_m = static_cast<float>(dist_cm) / 100.0f;
        const float draw_dist = std::min(dist_m, kMaxDrawDistance);

        // PX4 ObstacleDistance convention:
        //   - bin 0 = forward
        //   - positive increment = clockwise (right)
        // In lidar_sensor_link (FLU x=FWD, y=LEFT, z=UP), clockwise from forward
        // maps to x = +cos(theta), y = -sin(theta).
        const double theta_deg = static_cast<double>(i) * kIncrementDeg;
        const double theta_rad = px4_common::math::Deg2Rad(theta_deg);
        const double dx = static_cast<double>(draw_dist) * std::cos(theta_rad);
        const double dy = -static_cast<double>(draw_dist) * std::sin(theta_rad);

        geometry_msgs::msg::Point p0;
        p0.x = 0.0f;
        p0.y = 0.0f;
        p0.z = 0.0f;

        geometry_msgs::msg::Point p1;
        p1.x = static_cast<float>(dx);
        p1.y = static_cast<float>(dy);
        p1.z = 0.0f;

        const auto rgba = ColorFromDistance(dist_m);
        std_msgs::msg::ColorRGBA color;
        color.r = rgba[0];
        color.g = rgba[1];
        color.b = rgba[2];
        color.a = rgba[3];

        lines.points.push_back(p0);
        lines.points.push_back(p1);
        lines.colors.push_back(color);
        lines.colors.push_back(color);
    }

    msg.markers.push_back(lines);
    pub_markers_->publish(msg);
    ++sequence_;
}

}  // namespace px4_navigation
