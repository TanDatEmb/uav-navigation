#pragma once

#include "rclcpp/qos.hpp"

// The product QoS profiles (SYSTEM_DESIGN §6.5, D33/O21). Every publisher and subscription of a product
// topic takes its profile from here, so a depth or reliability change is made, and tested, in one place.
// All profiles are keep-last with volatile durability.
namespace uavnav::rosutil::qos {

/// Sensor inputs (/livox/lidar, /livox/imu): SensorDataQoS, best effort, depth 5.
inline rclcpp::QoS sensor_input() { return rclcpp::SensorDataQoS().keep_last(5); }

/// /lio/state: best effort, depth 10 (100 Hz stream; a late sample is worthless).
inline rclcpp::QoS state_output() { return rclcpp::QoS(rclcpp::KeepLast(10)).best_effort().durability_volatile(); }

/// /lio/odometry, /lio/health, /alignment: reliable, depth 10.
inline rclcpp::QoS reliable_output() { return rclcpp::QoS(rclcpp::KeepLast(10)).reliable().durability_volatile(); }

/// /events: reliable, depth 1000 (a burst between two recorder drains must not be dropped by the middleware).
inline rclcpp::QoS events() { return rclcpp::QoS(rclcpp::KeepLast(1000)).reliable().durability_volatile(); }

/// Topics exchanged with PX4 (/fmu/in/*, /fmu/out/*): best effort, depth 5.
inline rclcpp::QoS px4() { return rclcpp::QoS(rclcpp::KeepLast(5)).best_effort().durability_volatile(); }

}  // namespace uavnav::rosutil::qos
