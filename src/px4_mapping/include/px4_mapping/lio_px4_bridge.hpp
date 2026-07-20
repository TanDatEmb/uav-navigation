// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2026, LeTanDat

#ifndef PX4_MAPPING_LIO_PX4_BRIDGE_HPP_
#define PX4_MAPPING_LIO_PX4_BRIDGE_HPP_

#include <array>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>

#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <px4_msgs/msg/timesync_status.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <px4_ros2_utils/time/timesync.hpp>
#include <rclcpp/rclcpp.hpp>

namespace px4_mapping {

/**
 * @brief Convert LIO FLU pose to PX4 FRD external vision.
 *
 * Frame conversion: FLU (LIO) → FRD (PX4)
 * Using basis transform: C = diag(1, -1, -1)
 *   p_frd = C * p_flu
 *   R_frd = C * R_flu * C
 *
 * No alignment estimation. The pose is published as-is in POSE_FRAME_FRD.
 */
px4_msgs::msg::VehicleOdometry ConvertLioToPx4Frd(const nav_msgs::msg::Odometry& lio_msg,
                                                   std::uint64_t publish_timestamp_us,
                                                   std::uint64_t sample_timestamp_us,
                                                   std::int8_t quality);

/**
 * @brief Bridge FAST-LIO odometry into PX4 external-vision semantics.
 *
 * Subscribes:
 * - /lio/odometry (nav_msgs/Odometry, ENU world / FLU body)
 *
 * Publishes:
 * - /fmu/in/vehicle_visual_odometry (px4_msgs/VehicleOdometry, FRD)
 *
 * Uses POSE_FRAME_FRD: local FRD with arbitrary initial yaw.
 * No dependency on PX4 local position or armed state.
 */
class LioPx4Bridge : public rclcpp::Node {
   public:
    explicit LioPx4Bridge(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

    LioPx4Bridge(const LioPx4Bridge&) = delete;
    LioPx4Bridge& operator=(const LioPx4Bridge&) = delete;
    LioPx4Bridge(LioPx4Bridge&&) = delete;
    LioPx4Bridge& operator=(LioPx4Bridge&&) = delete;

    ~LioPx4Bridge() override = default;

   private:
    void LoadParameters();
    void LioCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
    void PublishVisualOdometry(const nav_msgs::msg::Odometry& lio_msg);
    void PublishDiagnostics();

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr lio_sub_;
    rclcpp::Subscription<px4_msgs::msg::TimesyncStatus>::SharedPtr timesync_sub_;
    rclcpp::Publisher<px4_msgs::msg::VehicleOdometry>::SharedPtr px4_pub_;
    rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostic_pub_;

    std::unique_ptr<px4_ros2_utils::time::Timesync> timesync_;

    std::string lio_topic_;
    std::string px4_topic_;

    int visual_odom_quality_{100};
    std::array<double, 3> position_variance_{0.04, 0.04, 0.09};
    std::array<double, 3> orientation_variance_{0.25, 0.25, 0.05};

    // Frame FRD: publish enabled immediately, no alignment needed
    bool frd_ready_{true};
    uint8_t reset_counter_{0};

    // Protects diagnostics data copy
    std::mutex diag_mutex_;
    uint64_t messages_received_{0};
    uint64_t messages_published_{0};
};

}  // namespace px4_mapping

#endif  // PX4_MAPPING_LIO_PX4_BRIDGE_HPP_
