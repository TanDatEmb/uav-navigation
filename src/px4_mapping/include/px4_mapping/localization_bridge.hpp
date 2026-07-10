// Copyright 2026 CTUAV. All rights reserved.
//
// Localization bridge node for converting camera_init point clouds to map_ned
// and bridging visual odometry into PX4.

#ifndef PX4_MAPPING_LOCALIZATION_BRIDGE_HPP_
#define PX4_MAPPING_LOCALIZATION_BRIDGE_HPP_

#include <memory>
#include <mutex>
#include <string>

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <nav_msgs/msg/odometry.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <px4_common/time/pose_buffer.hpp>
#include <px4_ros_com/time_sync.hpp>

namespace px4_mapping {

/**
 * @brief Node for transforming localization point clouds from camera_init frame to PX4 map_ned frame.
 *
 * Subscribes to:
 * - /localization/cloud (sensor_msgs/PointCloud2, camera_init/ENU frame)
 * - /localization/odometry (nav_msgs/Odometry, camera_init frame)
 * - /fmu/out/vehicle_odometry (px4_msgs/VehicleOdometry, NED frame)
 *
 * Publishes:
 * - /world/cloud (sensor_msgs/PointCloud2, map_ned frame)
 * - /fmu/in/vehicle_visual_odometry (px4_msgs/VehicleOdometry, optional)
 */
class LocalizationBridge : public rclcpp::Node {
   public:
    /**
     * @brief Constructor.
     * @param options Node options.
     */
    explicit LocalizationBridge(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

    /// Deleted copy constructor.
    LocalizationBridge(const LocalizationBridge&) = delete;

    /// Deleted copy assignment operator.
    LocalizationBridge& operator=(const LocalizationBridge&) = delete;

    /// Deleted move constructor.
    LocalizationBridge(LocalizationBridge&&) = delete;

    /// Deleted move assignment operator.
    LocalizationBridge& operator=(LocalizationBridge&&) = delete;

    /// Destructor.
    ~LocalizationBridge() override = default;

   private:
    // Callback functions
    void LocalizationCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
    void LioOdomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
    void Px4OdomCallback(const px4_msgs::msg::VehicleOdometry::SharedPtr msg);

    // Helper functions
    bool TransformPointCloud(const sensor_msgs::msg::PointCloud2::SharedPtr& input_cloud,
                             sensor_msgs::msg::PointCloud2& output_cloud);

    void PublishVisualOdometry(const px4_common::time::PoseSample& lio_sample);

    // Parameter loading
    void LoadParameters();

    // Publishers and subscribers
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_localization_cloud_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_lio_odom_;
    rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr sub_px4_odom_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_world_cloud_;
    rclcpp::Publisher<px4_msgs::msg::VehicleOdometry>::SharedPtr pub_visual_odom_;

    // Callback groups
    rclcpp::CallbackGroup::SharedPtr io_callback_group_;
    rclcpp::CallbackGroup::SharedPtr compute_callback_group_;

    // Pose buffers
    px4_common::time::PoseBuffer lio_pose_buffer_;
    px4_common::time::PoseBuffer px4_pose_buffer_;

    // Parameters
    std::string input_cloud_topic_;
    std::string output_cloud_topic_;
    std::string lio_odom_topic_;
    std::string px4_odom_topic_;
    std::string visual_odom_topic_;
    bool use_px4_odom_;
    bool publish_visual_odometry_to_px4_;
    bool visual_odom_align_to_px4_;
    bool visual_odom_align_full_6dof_;
    std::string visual_odom_alignment_mode_;
    int visual_odom_quality_;
    std::array<double, 3> visual_odom_position_variance_;
    std::array<double, 3> visual_odom_orientation_variance_;

    // Visual odometry alignment
    std::mutex visual_odom_mutex_;
    Eigen::Vector3d visual_align_translation_;
    Eigen::Quaterniond visual_align_rotation_;
    bool visual_alignment_ready_;

    // Diagnostic counters for runtime stability monitoring.
    uint64_t lio_pose_lookup_miss_{0};
    uint64_t px4_pose_lookup_miss_{0};

    // Statistics
    uint64_t frame_count_;

    // Shared timestamp-domain adapter for PX4->ROS conversion.
    px4_ros_com::time::Px4TimestampDomainAdapter px4_timestamp_adapter_;
};

}  // namespace px4_mapping

#endif  // PX4_MAPPING_LOCALIZATION_BRIDGE_HPP_