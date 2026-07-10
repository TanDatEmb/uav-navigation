#ifndef PX4_MAPPING_FAST_LIO2_NODE_HPP_
#define PX4_MAPPING_FAST_LIO2_NODE_HPP_

#include <mutex>
#include <string>
#include <vector>

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <nav_msgs/msg/odometry.hpp>
#include <px4_msgs/msg/vehicle_imu.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <px4_ros_com/time_sync.hpp>

namespace px4_mapping {

/**
 * @brief FAST-LIO2 style adapter for mapping pipeline integration.
 *
 * This node extracts the I/O contract used by FAST-LIO2:
 *   - Input: LiDAR cloud in sensor FLU + PX4 odometry in map_ned.
 *   - Output: /livox/l1/cloud (camera_init ENU-like frame) and
 *             /livox/l1/odometry (nav_msgs/Odometry in camera_init).
 *
 * It keeps the relevant preprocessing behavior from the reference project
 * (point decimation + blind-zone filtering), while remaining dependency-light
 * for incremental integration.
 */
class FastLio2Node : public rclcpp::Node {
   public:
     explicit FastLio2Node(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());

   private:
    struct OdomSample {
        rclcpp::Time stamp{0, 0, RCL_ROS_TIME};
        Eigen::Vector3d position_ned{Eigen::Vector3d::Zero()};
        Eigen::Quaterniond orientation_ned{Eigen::Quaterniond::Identity()};
        Eigen::Vector3d velocity_ned{Eigen::Vector3d::Zero()};
        bool valid{false};
    };

    void LoadParameters();
    void Px4OdomCallback(const px4_msgs::msg::VehicleOdometry::SharedPtr msg);
    void VehicleImuCallback(const px4_msgs::msg::VehicleImu::SharedPtr msg);
    void CloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);

    bool BuildProcessedCloud(const sensor_msgs::msg::PointCloud2 &input,
                             sensor_msgs::msg::PointCloud2 &output,
                             const OdomSample &odom_sample);
    void PublishLioOdometry(const OdomSample &odom_sample, const rclcpp::Time &stamp);

    // Subscriptions and publishers
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_cloud_;
    rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr sub_px4_odom_;
    rclcpp::Subscription<px4_msgs::msg::VehicleImu>::SharedPtr sub_vehicle_imu_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_processed_cloud_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_lio_odom_;

    // Parameters
    std::string input_cloud_topic_;
    std::string px4_odom_topic_;
    std::string vehicle_imu_topic_;
    std::string output_cloud_topic_;
    std::string output_odom_topic_;
    std::string output_frame_id_;
    std::string output_child_frame_id_;
    int point_filter_num_{3};
    double blind_m_{0.5};
    bool use_imu_fusion_{true};

    // LIO pose/pose-covariance published to the configured L1 odometry topic.
    // The covariance is used
    // by downstream consumers (RViz, navigation planner) to weight observations.
    // These values represent a healthy LIO pose in our SITL setup and are the
    // same defaults used by the legacy code path.
    std::array<double, 36> pose_covariance_{};
    std::array<double, 36> twist_covariance_{};
    double max_velocity_for_estimate_mps_{2.0};

    // Runtime state
    mutable std::mutex state_mutex_;
    OdomSample latest_odom_;
    OdomSample fused_odom_;
    bool fused_odom_valid_{false};
    bool origin_initialized_{false};
    Eigen::Vector3d origin_position_ned_{Eigen::Vector3d::Zero()};
    Eigen::Quaterniond origin_orientation_enu_{Eigen::Quaterniond::Identity()};

    // Last published pose, used to compute a finite-difference twist when the
    // upstream velocity is unavailable.
    rclcpp::Time last_publish_stamp_{0, 0, RCL_ROS_TIME};
    Eigen::Vector3d last_publish_position_ned_{Eigen::Vector3d::Zero()};
    bool last_publish_valid_{false};
    px4_ros_com::time::Px4TimestampDomainAdapter px4_timestamp_adapter_;
};

  using CloudPreprocessorNode = FastLio2Node;

}  // namespace px4_mapping

#endif  // PX4_MAPPING_FAST_LIO2_NODE_HPP_
