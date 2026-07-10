// Copyright 2026 TanDatEmb.
//
// Livox MID-360 processor node for PX4 Collision Prevention.
//
// Converts a 2.5D point cloud into a spherical yaw x pitch grid, applies
// a body-exclusion filter, and produces /fmu/in/obstacle_distance:
// a 72-bin horizontal obstacle distance message for PX4 Collision Prevention.

#ifndef PX4_NAVIGATION_LIVOX_MID360_PROCESSOR_HPP_
#define PX4_NAVIGATION_LIVOX_MID360_PROCESSOR_HPP_

#include <array>
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
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

namespace px4_navigation {

class LivoxMid360Processor : public rclcpp::Node {
   public:
    static constexpr int kDefaultYawBins = 72;
    static constexpr int kDefaultPitchBins = 28;
    static constexpr double kDefaultMinPitchDeg = -7.0;
    static constexpr double kDefaultMaxPitchDeg = 52.0;
    static constexpr double kDefaultMinDistanceM = 0.1;
    static constexpr double kDefaultMaxDistanceM = 40.0;
    static constexpr double kDefaultBodyExclusionRadiusM = 0.3;
    static constexpr double kDefaultPublishRateHz = 20.0;
    static constexpr int kDefaultStaleTimeoutMs = 400;
    static constexpr uint8_t kFrameBodyFrd = 12;
    static constexpr uint8_t kSensorTypeLaser = 0;
    static constexpr uint16_t kNoObstacle = 65535;  // UINT16_MAX

    explicit LivoxMid360Processor(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

   private:
    struct GridCell {
        uint16_t min_distance_cm = kNoObstacle;
    };

    void LoadParameters();
    void CloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
    void OdomCallback(const px4_msgs::msg::VehicleOdometry::SharedPtr msg);
    void TimerCallback();

    void BuildSphericalGrid(const std::vector<Eigen::Vector3f>& cloud);
    void ComputeMinDistances(std::array<uint16_t, 72>& min_distances) const;

    // Subscriptions
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_cloud_;
    rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr sub_odom_;

    // Publishers
    rclcpp::Publisher<px4_msgs::msg::ObstacleDistance>::SharedPtr pub_obstacle_distance_;
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr pub_local_virtual_scan_;

    // Timer
    rclcpp::TimerBase::SharedPtr publish_timer_;

    // State (protected by mutex)
    mutable std::mutex state_mutex_;
    std::vector<Eigen::Vector3f> cloud_points_;
    rclcpp::Time last_cloud_time_{0, 0, RCL_ROS_TIME};
    std::chrono::steady_clock::time_point last_cloud_arrival_time_;
    bool cloud_received_ = false;
    bool odom_received_ = false;
    double vehicle_yaw_ = 0.0;
    Eigen::Vector3d vehicle_position_ned_ = Eigen::Vector3d::Zero();

    // Runtime spherical grid: [yaw_bin][pitch_bin]
    std::vector<std::vector<GridCell>> grid_;

    // Parameters
    int yaw_bins_ = kDefaultYawBins;
    int pitch_bins_ = kDefaultPitchBins;
    double min_pitch_rad_ = 0.0;
    double max_pitch_rad_ = 0.0;
    double min_distance_m_ = kDefaultMinDistanceM;
    double max_distance_m_ = kDefaultMaxDistanceM;
    double body_exclusion_radius_m_ = kDefaultBodyExclusionRadiusM;
    double publish_rate_hz_ = kDefaultPublishRateHz;
    int stale_timeout_ms_ = kDefaultStaleTimeoutMs;
    std::string input_cloud_topic_;
    std::string vehicle_odom_topic_;
    std::string obstacle_distance_topic_;
    bool publish_local_virtual_scan_ = true;
    std::string local_virtual_scan_topic_;
    std::string local_virtual_scan_frame_id_;
    // Coordinate frame of the input point cloud.
    //   "sensor"     : already in PX4 body FRD (x=FWD, y=RIGHT, z=DOWN).
    //   "sensor_flu" : ROS/Gazebo FLU (x=FWD, y=LEFT, z=UP); converted to FRD.
    //   "ned"        : map_ned world frame; rotated to body FRD using yaw.
    std::string cloud_frame_;
    bool filter_ground_points_ = true;

    // Statistics
    uint64_t clouds_received_ = 0;
    uint64_t messages_published_ = 0;
    uint64_t stale_clears_ = 0;
};

}  // namespace px4_navigation

#endif  // PX4_NAVIGATION_LIVOX_MID360_PROCESSOR_HPP_
