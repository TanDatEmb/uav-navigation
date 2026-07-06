// Copyright 2026 CTUAV. All rights reserved.
//
// Obstacle distance publisher node for PX4 Collision Prevention.
//
// Subscribes to a PointCloud2 topic and vehicle odometry from PX4, converts
// the point cloud into a 72-bin horizontal obstacle distance array in the
// aircraft body frame (FRD), and publishes px4_msgs/ObstacleDistance to PX4
// via the uXRCE-DDS bridge.
//
// This node enables PX4's built-in Collision Prevention in Position Mode
// without requiring a custom flight mode.

#ifndef PX4_NAVIGATION_OBSTACLE_DISTANCE_PUBLISHER_HPP_
#define PX4_NAVIGATION_OBSTACLE_DISTANCE_PUBLISHER_HPP_

#include <atomic>
#include <chrono>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <px4_msgs/msg/obstacle_distance.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

namespace px4_navigation {

/**
 * @brief Publishes obstacle distance data to PX4 for Collision Prevention.
 *
 * Converts a 3D point cloud into a 72-bin horizontal scan (5° per bin) in
 * the aircraft body frame (FRD: Forward-Right-Down) and publishes the result
 * as px4_msgs::msg::ObstacleDistance on /fmu/in/obstacle_distance.
 *
 * The node runs at a configurable rate (default 10 Hz). If the input point
 * cloud is stale beyond a threshold, all bins are set to UINT16_MAX to clear
 * the obstacle map inside PX4 (safe behaviour: drone stops moving).
 */
class ObstacleDistancePublisher : public rclcpp::Node {
   public:
    /// Number of angular bins in the PX4 ObstacleDistance message (fixed by PX4).
    static constexpr int kNumBins = 72;

    /// Angular width of each bin in degrees (360 / 72 = 5).
    static constexpr double kIncrementDeg = 5.0;

    /// Default publish rate in Hz.
    static constexpr double kDefaultRateHz = 10.0;

    /// Default minimum reporting distance in centimetres (0.3 m).
    static constexpr int kDefaultMinDistanceCm = 30;

    /// Default maximum reporting distance in centimetres (15 m).
    static constexpr int kDefaultMaxDistanceCm = 1500;

    /// Default height band in metres (include points within +/- this value of drone altitude).
    static constexpr double kDefaultHeightBandM = 2.0;

    /// Default stale timeout in milliseconds before clearing the obstacle map.
    static constexpr int kDefaultStaleTimeoutMs = 400;

    /// MAV_FRAME_BODY_FRD constant from the ObstacleDistance message.
    static constexpr uint8_t kFrameBodyFrd = 12;

    /// MAV_DISTANCE_SENSOR_LASER constant.
    static constexpr uint8_t kSensorTypeLaser = 0;

    /// Sentinel value indicating "no obstacle / unknown" in the distance array.
    static constexpr uint16_t kNoObstacle = 65535;  // UINT16_MAX

    /**
     * @brief Constructor.
     * @param options Node options.
     */
    explicit ObstacleDistancePublisher(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

    /// Deleted copy constructor.
    ObstacleDistancePublisher(const ObstacleDistancePublisher&) = delete;

    /// Deleted copy assignment.
    ObstacleDistancePublisher& operator=(const ObstacleDistancePublisher&) = delete;

    /// Deleted move constructor.
    ObstacleDistancePublisher(ObstacleDistancePublisher&&) = delete;

    /// Deleted move assignment.
    ObstacleDistancePublisher& operator=(ObstacleDistancePublisher&&) = delete;

    /// Destructor.
    ~ObstacleDistancePublisher() override = default;

   private:
    /// Called when a new PointCloud2 message arrives.
    void PointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);

    /// Called when PX4 vehicle odometry arrives.
    void VehicleOdomCallback(const px4_msgs::msg::VehicleOdometry::SharedPtr msg);

    /// Timer callback: builds and publishes the ObstacleDistance message.
    void PublishTimerCallback();

    /// Declare and load all ROS 2 parameters.
    void LoadParameters();

    /// Convert the latest cached point cloud into 72-bin distances (cm, body FRD).
    void ComputeBinDistances(std::array<uint16_t, kNumBins>& distances);

    // ── Subscriptions ──────────────────────────────────────────────
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_cloud_;
    rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr sub_odom_;

    // ── Publisher ──────────────────────────────────────────────────
    rclcpp::Publisher<px4_msgs::msg::ObstacleDistance>::SharedPtr pub_obstacle_distance_;

    // ── Timer ─────────────────────────────────────────────────────
    rclcpp::TimerBase::SharedPtr publish_timer_;

    // ── Parameters ─────────────────────────────────────────────────
    double publish_rate_hz_;
    int min_distance_cm_;
    int max_distance_cm_;
    double height_band_m_;
    int stale_timeout_ms_;
    std::string input_cloud_topic_;
    std::string vehicle_odom_topic_;
    std::string obstacle_distance_topic_;
    uint8_t frame_;

    // ── State (protected by mutex) ─────────────────────────────────
    mutable std::mutex state_mutex_;

    /// Latest cached point cloud in NED frame (from ned_transform_node).
    std::vector<Eigen::Vector3f> cloud_body_frd_;

    /// Timestamp of the most recent point cloud (ROS clock).
    rclcpp::Time last_cloud_time_;

    /// Latest vehicle position in NED frame (x=North, y=East, z=Down).
    Eigen::Vector3d vehicle_position_ned_{Eigen::Vector3d::Zero()};

    /// Latest vehicle yaw in NED frame (radians, clockwise positive).
    double vehicle_yaw_ned_{0.0};

    /// True once we have received at least one odometry message.
    std::atomic<bool> odom_received_{false};

    /// True once we have received at least one point cloud.
    std::atomic<bool> cloud_received_{false};

    // ── Statistics ─────────────────────────────────────────────────
    uint64_t messages_published_{0};
    uint64_t clouds_received_{0};
    uint64_t stale_clears_{0};
};

}  // namespace px4_navigation

#endif  // PX4_NAVIGATION_OBSTACLE_DISTANCE_PUBLISHER_HPP_