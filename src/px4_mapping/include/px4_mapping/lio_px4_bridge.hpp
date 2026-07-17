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
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>
#include <px4_ros2_utils/time/timesync.hpp>
#include <rclcpp/rclcpp.hpp>

#include <px4_mapping/time/pose_buffer.hpp>

namespace px4_mapping {

/**
 * @brief Convert an ENU-world/FLU-body LIO pose into PX4 NED/FRD fields.
 *
 * This is a coordinate-basis conversion only. It does not estimate origin or
 * yaw alignment. Timestamp conversion is handled by the caller so this
 * function remains deterministic and directly testable.
 */
px4_msgs::msg::VehicleOdometry ConvertLioOdometryToPx4(const nav_msgs::msg::Odometry& lio_msg,
                                                       std::uint64_t publish_timestamp_us,
                                                       std::uint64_t sample_timestamp_us,
                                                       std::int8_t quality);

/**
 * @brief Timestamped pose pair used for PX4/LIO alignment.
 */
struct AlignmentPair {
    std::int64_t t_ns{0};
    Eigen::Vector3d lio_position{Eigen::Vector3d::Zero()};
    Eigen::Vector3d px4_position{Eigen::Vector3d::Zero()};
    Eigen::Quaterniond lio_orientation{Eigen::Quaterniond::Identity()};
    Eigen::Quaterniond px4_orientation{Eigen::Quaterniond::Identity()};
};

/**
 * @brief Result of a windowed yaw+translation alignment.
 */
struct AlignmentResult {
    bool ready{false};
    Eigen::Quaterniond rotation{Eigen::Quaterniond::Identity()};
    Eigen::Vector3d translation{Eigen::Vector3d::Zero()};
    double yaw_offset_rad{0.0};
    std::size_t samples_used{0};
};

/**
 * @brief Compute a robust yaw+translation alignment from synchronized pairs.
 *
 * Implements the FAST-LIO2/PX4 alignment baseline:
 *   1. Per-pair yaw difference.
 *   2. Circular mean yaw.
 *   3. Reject yaw outliers.
 *   4. Per-pair translation under the mean yaw.
 *   5. Component-wise median translation.
 */
AlignmentResult ComputeYawTranslationAlignment(const std::vector<AlignmentPair>& pairs,
                                               double yaw_outlier_threshold_rad = 0.1);

/**
 * @brief Bridge FAST-LIO odometry into PX4 external-odometry semantics.
 *
 * Subscribes to:
 * - /lio/odometry (nav_msgs/Odometry, ENU world / FLU body)
 * - /fmu/out/vehicle_odometry (px4_msgs/VehicleOdometry, NED world / FRD body)
 * - /fmu/out/timesync_status (optional, for hardware timesync)
 *
 * Publishes:
 * - /fmu/in/vehicle_visual_odometry (px4_msgs/VehicleOdometry, NED/FRD, PX4 boot time)
 *
 * The node estimates a fixed T_map_ned_lio_world transform by comparing the LIO
 * pose with PX4 odometry at initialization. After capture, the same transform
 * is applied to every subsequent LIO pose. No per-scan point-cloud transform is
 * performed here; the occupancy map remains in lio_world and frame conversions
 * happen at planning/PX4 boundaries.
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
    void Px4OdomCallback(const px4_msgs::msg::VehicleOdometry::SharedPtr msg);
    void Px4LocalPositionCallback(const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg);
    void VehicleStatusCallback(const px4_msgs::msg::VehicleStatus::SharedPtr msg);
    void PublishVisualOdometry(const px4_mapping::time::PoseSample& lio_sample);
    void PublishAlignmentDiagnostics();
    bool AlignmentGateOpen(double lio_position_variance);
    void ResetAlignment();

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr lio_sub_;
    rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr px4_odom_sub_;
    rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr px4_local_pos_sub_;
    rclcpp::Subscription<px4_msgs::msg::VehicleStatus>::SharedPtr px4_status_sub_;
    rclcpp::Subscription<px4_msgs::msg::TimesyncStatus>::SharedPtr timesync_sub_;
    rclcpp::Publisher<px4_msgs::msg::VehicleOdometry>::SharedPtr px4_pub_;
    rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostic_pub_;

    std::unique_ptr<px4_ros2_utils::time::Timesync> timesync_;

    px4_mapping::time::PoseBuffer lio_pose_buffer_;
    px4_mapping::time::PoseBuffer px4_pose_buffer_;

    std::string lio_topic_;
    std::string px4_odom_topic_;
    std::string px4_topic_;

    bool align_to_px4_{true};
    std::string alignment_mode_{"yaw_translation"};
    int visual_odom_quality_{100};
    std::array<double, 3> position_variance_{0.04, 0.04, 0.09};
    std::array<double, 3> orientation_variance_{0.25, 0.25, 0.05};

    // Alignment gate parameters.
    double alignment_window_seconds_{2.0};
    int alignment_minimum_samples_{20};
    double alignment_max_speed_mps_{0.5};
    double alignment_max_lio_position_variance_{0.1};
    double alignment_yaw_outlier_threshold_rad_{0.1};

    std::mutex alignment_mutex_;
    Eigen::Quaterniond align_rotation_{Eigen::Quaterniond::Identity()};
    Eigen::Vector3d align_translation_{Eigen::Vector3d::Zero()};
    bool alignment_ready_{false};
    int alignment_reset_counter_{0};
    std::vector<AlignmentPair> alignment_pairs_;
    std::int64_t last_lio_t_ns_{0};
    double latest_lio_position_variance_{std::numeric_limits<double>::infinity()};

    // Latest PX4 local-position gate inputs.
    std::mutex px4_local_pos_mutex_;
    px4_msgs::msg::VehicleLocalPosition latest_px4_local_pos_;
    bool px4_local_pos_valid_{false};
    bool px4_yaw_valid_{false};
    double px4_speed_mps_{0.0};

    // Latest vehicle status.
    std::mutex px4_status_mutex_;
    bool px4_armed_{false};

    std::uint64_t px4_pose_lookup_miss_{0};
};

}  // namespace px4_mapping

#endif  // PX4_MAPPING_LIO_PX4_BRIDGE_HPP_
