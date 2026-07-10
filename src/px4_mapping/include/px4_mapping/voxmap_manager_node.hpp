/* =========================================================================
    voxmap_manager_node.hpp, Layer 2 ROS 2 node wrapping VoxelHashMap

    1. VoxMapManagerNode
       - Subscribes raw lidar /livox/lidar and PX4 VehicleOdometry
       - Transforms raw points from sensor FLU to world NED before raycasting
       - Implements IVoxMapManager so Layer 3 can query resolution
       - Runs distance based eviction timer to bound map memory
    - Publishes global map topic for RViz and Layer 3 ring buffer

    2. Factory
       - get_voxmap_node returns Node and IVoxMapManager interface
       - Composed pipeline uses single instance for both roles
   ========================================================================= */

#ifndef PX4_MAPPING_VOXMAP_MANAGER_NODE_HPP_
#define PX4_MAPPING_VOXMAP_MANAGER_NODE_HPP_

#include <atomic>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <px4_common/frame_constants.hpp>
#include <px4_common/mapping/voxel_map_interface.hpp>
#include <px4_common/time/pose_buffer.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>
#include <px4_ros_com/frame_transforms.hpp>
#include <px4_ros_com/time_sync.hpp>
#include <px4_ros_com/topic_helpers.hpp>

#include <px4_mapping/voxel_hash_map.hpp>

namespace px4_mapping {

class VoxMapManagerNode : public rclcpp::Node, public px4_common::mapping::IVoxMapManager {
   public:
    explicit VoxMapManagerNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

    ~VoxMapManagerNode() override;

    // IVoxMapManager interface implementation
    double GetResolution() const override;
    void GetOccupiedPointsInRadius(const Eigen::Vector3d& center, double radius,
                                   std::vector<Eigen::Vector3d>& out) override;
    bool IsReady() const noexcept override;
    std::uint64_t FramesDropped() const noexcept override;
    Eigen::Vector3d GetExtrinsicTranslation() const noexcept override;

   private:
    // Core map
    VoxelHashMap voxel_map_;

    // ROS 2 interfaces
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_cloud_;
    rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr sub_odom_;
    rclcpp::Subscription<px4_msgs::msg::VehicleStatus>::SharedPtr sub_status_;
    rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr sub_local_pos_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_lio_odom_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_map_;
    rclcpp::TimerBase::SharedPtr timeout_timer_;
    rclcpp::TimerBase::SharedPtr eviction_timer_;

    // Callback groups
    rclcpp::CallbackGroup::SharedPtr io_cb_group_;
    rclcpp::CallbackGroup::SharedPtr compute_cb_group_;

    // Reusable scratch buffer
    std::vector<px4_common::PointLivox> changed_buf_;

    // Parameters
    bool publish_local_map_{true};
    int log_interval_{1};
    double timeout_seconds_{3600.0};
    std::string log_path_;
    std::string cloud_topic_{"/livox/world/cloud"};
    std::string map_topic_{"/livox/map/global"};
    std::string input_source_{"px4_full"};
    bool deskewed_input_{false};
    bool full_pose_input_{false};
    bool lio_world_input_{false};

    // Activity tracking
    rclcpp::Time last_data_time_;
    std::string last_frame_id_;
    bool map_cleared_{true};

    // Drone state (PX4 NED frame)
    Eigen::Vector3d drone_pos_{Eigen::Vector3d::Zero()};
    double drone_yaw_{0.0};
    double cos_yaw_{1.0};
    double sin_yaw_{0.0};
    Eigen::Quaterniond drone_q_{Eigen::Quaterniond::Identity()};  // PX4 q_ned_frd
    bool have_drone_q_{false};

    // LIO pose in camera_init frame
    Eigen::Quaterniond lio_q_camera_init_{Eigen::Quaterniond::Identity()};
    bool have_lio_q_{false};

    mutable std::mutex odom_mutex_;

    // Lidar-to-IMU extrinsic
    Eigen::Vector3d T_lidar_in_imu_{Eigen::Vector3d::Zero()};

    // Layer 3 health signals (see IsReady() for the boolean expression)
    // ready_consecutive_frames_ is reset on timeout/fault and incremented while
    // coverage and data freshness hold.
    std::atomic<std::uint64_t> frames_dropped_{0};
    std::uint64_t frames_dropped_at_last_log_{0};

    // Coverage-based readiness gate configuration and runtime state
    int ready_min_frames_{5};
    int ready_min_occupied_{1000};
    std::atomic<int> ready_consecutive_frames_{0};
    std::atomic<bool> data_fresh_{false};
    std::atomic<bool> coverage_ok_{false};

    // Fatal fault flag set on timestamp-domain contamination or other
    // unrecoverable conditions. Once true IsReady() stays false.
    std::atomic<bool> fatal_fault_{false};

    // LIO subscription monitor
    std::string lio_odom_topic_{"/livox/l1/odometry"};
    std::size_t lio_samples_received_{0};
    rclcpp::Time lio_first_sample_time_;

    // Timestamped pose buffers for scan-time raycast origin lookup
    px4_common::time::PoseBuffer lio_buf_;
    px4_common::time::PoseBuffer px4_buf_;

    // Rollback knob
    bool use_lio_buffer_{true};

    // Shared timestamp-domain adapter for PX4->ROS conversion.
    px4_ros_com::time::Px4TimestampDomainAdapter px4_timestamp_adapter_;

    // Alignment gate: when enabled, the node waits for the drone to be armed,
    // nearly stationary, EKF position valid, and LIO covariance small before
    // marking the map as ready. The gate is disabled by default for backwards
    // compatibility with existing SITL pipelines; set require_alignment_gate=true
    // to enforce it in production flights.
    bool require_alignment_gate_{false};
    double aligned_min_seconds_{5.0};
    double aligned_max_velocity_{0.05};
    double aligned_lio_covariance_max_{0.01};
    double aligned_max_seconds_to_capture_{30.0};
    std::string aligned_timeout_action_{"hold_indefinitely"};

    // Alignment gate runtime state
    std::atomic<bool> alignment_captured_{false};
    std::atomic<bool> alignment_degraded_{false};
    std::atomic<bool> armed_{false};
    std::atomic<double> drone_speed_{0.0};
    std::atomic<bool> ekf_pose_valid_{false};

    std::atomic<double> lio_covariance_trace_{1e9};
    rclcpp::Time alignment_start_time_;
    rclcpp::Time aligned_streak_start_;
    bool alignment_warned_{false};
    rclcpp::TimerBase::SharedPtr alignment_timer_;

    // PointCloud2 field cache
    uint32_t cached_point_step_{0};
    uint32_t off_x_{0};
    uint32_t off_y_{4};
    uint32_t off_z_{8};
    uint32_t off_intensity_{0};

    // Persistent point buffer
    std::vector<px4_common::PointLivox> input_points_;

    // Static FRD to FLU rotation matrix
    const Eigen::Matrix3d C_FRD_FLU_ =
        (Eigen::Matrix3d() << 1.0, 0.0, 0.0, 0.0, -1.0, 0.0, 0.0, 0.0, -1.0).finished();

    // Logging
    double total_process_time_ms_{0.0};
    double total_update_time_ms_{0.0};
    double total_publish_time_ms_{0.0};
    unsigned long frame_count_{0};
    double min_time_ms_{std::numeric_limits<double>::max()};
    double max_time_ms_{0.0};

    // Callbacks
    void cloudCallback(sensor_msgs::msg::PointCloud2::UniquePtr msg);
    void odomCallback(const px4_msgs::msg::VehicleOdometry::SharedPtr msg);
    void vehicleStatusCallback(const px4_msgs::msg::VehicleStatus::SharedPtr msg);
    void vehicleLocalPositionCallback(const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg);
    void lioOdomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
    void timeoutCallback();
    void alignmentTick();

    // Helper functions
    bool hasField(const sensor_msgs::msg::PointCloud2& msg, const std::string& name);
    void saveGlobalMap();
    void initLogging();
    static void mkdirRecursive(const std::string& path);
    void writeSummary();
    void publishMap();
};

using VoxelMapNode = VoxMapManagerNode;

// Factory for composed pipeline
std::shared_ptr<rclcpp::Node> get_voxmap_node(
    const rclcpp::NodeOptions& options,
    std::shared_ptr<px4_common::mapping::IVoxMapManager>& out_iface);

}  // namespace px4_mapping

#endif  // PX4_MAPPING_VOXMAP_MANAGER_NODE_HPP_