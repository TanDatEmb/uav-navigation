// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2026, LeTanDat

#ifndef PX4_MAPPING_LIO_PX4_ALIGNMENT_HPP_
#define PX4_MAPPING_LIO_PX4_ALIGNMENT_HPP_

#include <cstdint>
#include <memory>
#include <string>

#include <nav_msgs/msg/odometry.hpp>
#include <px4_msgs/msg/timesync_status.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <px4_ros2_utils/time/timesync.hpp>
#include <rclcpp/rclcpp.hpp>

namespace px4_mapping {

/**
 * @brief Convert an ENU-world/FLU-body representation into PX4 NED/FRD fields.
 *
 * This is a coordinate-basis conversion only; it does not estimate origin or
 * yaw alignment. Timestamp conversion is handled by the caller so this
 * function remains deterministic and directly testable.
 */
px4_msgs::msg::VehicleOdometry ConvertLioOdometryToPx4(const nav_msgs::msg::Odometry& lio_msg,
                                                       std::uint64_t publish_timestamp_us,
                                                       std::uint64_t sample_timestamp_us,
                                                       std::int8_t quality);

/**
 * @brief Bridge FAST-LIO odometry into PX4 external-odometry semantics.
 *
 * The input must already satisfy ENU world semantics with a FLU body. Output is
 * represented as NED world + FRD body. A gravity-aligned LIO frame with arbitrary
 * yaw still requires a separately validated origin/yaw alignment before it is
 * semantically north-aligned. ROS and PX4 timestamps are converted only through
 * px4_ros2_utils::time::Timesync.
 */
class LioPx4Alignment : public rclcpp::Node {
   public:
    explicit LioPx4Alignment(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

   private:
    void LoadParameters();
    void LioCallback(const nav_msgs::msg::Odometry::SharedPtr msg);

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr lio_sub_;
    rclcpp::Subscription<px4_msgs::msg::TimesyncStatus>::SharedPtr timesync_sub_;
    rclcpp::Publisher<px4_msgs::msg::VehicleOdometry>::SharedPtr px4_pub_;

    std::unique_ptr<px4_ros2_utils::time::Timesync> timesync_;

    std::string lio_topic_;
    std::string px4_topic_;
    int visual_odom_quality_{100};
};

}  // namespace px4_mapping

#endif  // PX4_MAPPING_LIO_PX4_ALIGNMENT_HPP_
