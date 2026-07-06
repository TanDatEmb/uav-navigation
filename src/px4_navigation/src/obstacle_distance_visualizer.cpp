// Copyright 2026 CTUAV. All rights reserved.

#include "px4_navigation/obstacle_distance_visualizer.hpp"

#include <cmath>

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <px4_common/math/transforms.hpp>
#include <px4_msgs/msg/obstacle_distance.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

namespace px4_navigation {

ObstacleDistanceVisualizer::ObstacleDistanceVisualizer(const rclcpp::NodeOptions& options)
    : rclcpp::Node("obstacle_distance_visualizer", options) {
    const auto px4_qos = rclcpp::QoS(20).best_effort();

    sub_obstacle_ = this->create_subscription<px4_msgs::msg::ObstacleDistance>(
        "/fmu/in/obstacle_distance", px4_qos,
        std::bind(&ObstacleDistanceVisualizer::ObstacleDistanceCallback, this,
                  std::placeholders::_1));

    sub_odom_ = this->create_subscription<px4_msgs::msg::VehicleOdometry>(
        "/fmu/out/vehicle_odometry", px4_qos,
        std::bind(&ObstacleDistanceVisualizer::VehicleOdomCallback, this,
                  std::placeholders::_1));

    pub_markers_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
        "/obstacle_distance/markers", rclcpp::QoS(20).reliable());

    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(100),
        std::bind(&ObstacleDistanceVisualizer::PublishMarkers, this));

    RCLCPP_INFO(this->get_logger(),
                "obstacle_distance_visualizer started: publishing %d arrow markers",
                kNumBins);
}

void ObstacleDistanceVisualizer::ObstacleDistanceCallback(
    const px4_msgs::msg::ObstacleDistance::SharedPtr msg) {
    for (int i = 0; i < kNumBins; ++i) {
        distances_[i] = msg->distances[i];
    }
    obstacle_valid_ = true;
}

void ObstacleDistanceVisualizer::VehicleOdomCallback(
    const px4_msgs::msg::VehicleOdometry::SharedPtr msg) {
    const Eigen::Quaterniond q(msg->q[0], msg->q[1], msg->q[2], msg->q[3]);
    vehicle_yaw_ = px4_common::math::QuaternionGetYaw(q);
    odom_valid_ = true;
}

void ObstacleDistanceVisualizer::PublishMarkers() {
    if (!obstacle_valid_ || !odom_valid_) {
        return;
    }

    auto msg = visualization_msgs::msg::MarkerArray();
    msg.markers.reserve(kNumBins + 1);

    // Delete previous markers.
    {
        auto delete_all = visualization_msgs::msg::Marker();
        delete_all.header.frame_id = "map_ned";
        delete_all.header.stamp = this->now();
        delete_all.action = visualization_msgs::msg::Marker::DELETEALL;
        delete_all.ns = "obstacle_distance";
        msg.markers.push_back(delete_all);
    }

    const double yaw_cos = std::cos(vehicle_yaw_);
    const double yaw_sin = std::sin(vehicle_yaw_);

    for (int i = 0; i < kNumBins; ++i) {
        const uint16_t dist_cm = distances_[i];
        if (dist_cm == 0 || dist_cm >= kNoObstacle) {
            continue;  // no obstacle / unknown
        }

        const double dist_m = static_cast<double>(dist_cm) / 100.0;

        // Bin angle in body FRD: 0 = forward, positive = clockwise (right).
        // i=0 → -180° (behind), i=36 → 0° (forward), i=54 → +90° (right).
        const double angle_rad =
            px4_common::math::Deg2Rad(static_cast<double>(i) * kIncrementDeg);

        // Direction in body FRD.
        const double bx = std::cos(angle_rad);
        const double by = std::sin(angle_rad);

        // Rotate body FRD → NED world.
        const double dx = yaw_cos * bx - yaw_sin * by;
        const double dy = yaw_sin * bx + yaw_cos * by;

        auto marker = visualization_msgs::msg::Marker();
        marker.header.frame_id = "map_ned";
        marker.header.stamp = this->now();
        marker.ns = "obstacle_distance";
        marker.id = i;
        marker.type = visualization_msgs::msg::Marker::ARROW;
        marker.action = visualization_msgs::msg::Marker::ADD;

        // Start at drone origin (we don't have exact drone position; use origin).
        marker.pose.position.x = 0.0;
        marker.pose.position.y = 0.0;
        marker.pose.position.z = 0.0;

        // Orientation: arrow points in direction (dx, dy, 0).
        const double half_angle = std::atan2(dy, dx) / 2.0;
        marker.pose.orientation.w = std::cos(half_angle);
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = std::sin(half_angle);

        // Arrow length = distance.
        marker.scale.x = dist_m;  // shaft length
        marker.scale.y = 0.05;    // shaft width
        marker.scale.z = 0.05;    // head width

        // Color by distance: red (close) → green (far), clamped at 5 m.
        const float ratio = static_cast<float>(
            std::min(dist_m, 5.0) / 5.0);
        marker.color.r = 1.0f - ratio;
        marker.color.g = ratio;
        marker.color.b = 0.0f;
        marker.color.a = 0.8f;

        marker.lifetime = rclcpp::Duration::from_seconds(0.5);
        msg.markers.push_back(marker);
    }

    pub_markers_->publish(msg);
    ++sequence_;
}

}  // namespace px4_navigation