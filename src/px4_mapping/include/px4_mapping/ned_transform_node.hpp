// Copyright 2026 CTUAV. All rights reserved.
//
// NED transform node for converting FAST-LIO2 point clouds from camera_init
// frame to map_ned frame for PX4 integration.

#ifndef PX4_MAPPING_NED_TRANSFORM_NODE_HPP_
#define PX4_MAPPING_NED_TRANSFORM_NODE_HPP_

#include <atomic>
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

namespace px4_mapping {

/**
 * @brief Node for transforming point clouds from FAST-LIO2 camera_init frame to PX4 map_ned frame.
 *
 * Subscribes to:
 * - /livox_processed (sensor_msgs/PointCloud2, camera_init/ENU frame)
 * - /odometry (nav_msgs/Odometry, camera_init frame)
 * - /fmu/out/vehicle_odometry (px4_msgs/VehicleOdometry, NED frame)
 *
 * Publishes:
 * - /livox_processed_ned (sensor_msgs/PointCloud2, map_ned frame)
 * - /fmu/in/vehicle_visual_odometry (px4_msgs/VehicleOdometry, optional)
 */
class NedTransformNode : public rclcpp::Node {
   public:
    /**
     * @brief Constructor.
     * @param options Node options.
     */
    explicit NedTransformNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

    /// Deleted copy constructor.
    NedTransformNode(const NedTransformNode&) = delete;

    /// Deleted copy assignment operator.
    NedTransformNode& operator=(const NedTransformNode&) = delete;

    /// Deleted move constructor.
    NedTransformNode(NedTransformNode&&) = delete;

    /// Deleted move assignment operator.
    NedTransformNode& operator=(NedTransformNode&&) = delete;

    /// Destructor.
    ~NedTransformNode() override = default;

   private:
    // Callback functions
    void LivoxCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
    void LioOdomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
    void Px4OdomCallback(const px4_msgs::msg::VehicleOdometry::SharedPtr msg);

    // Helper functions
    bool TransformPointCloud(const sensor_msgs::msg::PointCloud2::SharedPtr& input_cloud,
                             sensor_msgs::msg::PointCloud2& output_cloud);

    void PublishVisualOdometry(const px4_common::time::PoseSample& lio_sample);

    // Parameter loading
    void LoadParameters();

    // Publishers and subscribers
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_livox_cloud_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_lio_odom_;
    rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr sub_px4_odom_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_livox_ned_;
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
    int visual_odom_quality_;
    std::array<double, 3> visual_odom_position_variance_;
    std::array<double, 3> visual_odom_orientation_variance_;

    // Visual odometry alignment
    std::mutex visual_odom_mutex_;
    Eigen::Vector3d visual_align_translation_;
    Eigen::Quaterniond visual_align_rotation_;
    bool visual_alignment_ready_;

    // Statistics
    uint64_t frame_count_;
};

}  // namespace px4_mapping

#endif  // PX4_MAPPING_NED_TRANSFORM_NODE_HPP_