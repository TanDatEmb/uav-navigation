/* =========================================================================
    global_mapper.cpp, Layer 2 ROS 2 node wrapping VoxelHashMap

    1. GlobalMapper
    - Subscribes world cloud and PX4 VehicleOdometry
       - Transforms raw points from sensor FLU to world NED before raycasting
       - Implements IVoxMapManager so Layer 3 can query resolution
       - Supports opt-in distance eviction for memory-constrained deployments
    - Publishes accumulated global occupancy and a radius-bounded local view

    2. Factory
       - get_global_mapper_node returns Node and IVoxMapManager interface
       - Composed pipeline uses single instance for both roles
   ========================================================================= */

#include "px4_mapping/global_mapper.hpp"

#include <sys/stat.h>
#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <ctime>
#include <deque>
#include <fstream>
#include <iomanip>
#include <limits>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

#include <px4_mapping/time/pose_buffer.hpp>
#include <px4_msgs/msg/timesync_status.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>
#include <px4_nav_common/frame_constants.hpp>
#include <px4_nav_common/mapping/voxel_map_interface.hpp>
#include <px4_nav_common/mapping/voxel_types.hpp>
#include <px4_nav_common/math/grid.hpp>
#include <px4_nav_common/types.hpp>
#include <px4_ros2_utils/frame/transform.hpp>
#include <px4_ros2_utils/px4/topic.hpp>
#include <px4_ros2_utils/qos/sensor.hpp>

#include <px4_mapping/voxel_hash_map.hpp>

using std::placeholders::_1;

namespace px4_mapping {

namespace {

sensor_msgs::msg::PointCloud2 MakeGlobalMapCloud(
    const std::vector<px4_nav_common::PointLivox>& points,
    const builtin_interfaces::msg::Time& stamp, const std::string& frame_id) {
    sensor_msgs::msg::PointCloud2 msg;
    msg.header.stamp = stamp;
    msg.header.frame_id = frame_id;
    msg.height = 1;
    msg.width = static_cast<std::uint32_t>(points.size());
    msg.is_dense = true;
    msg.is_bigendian = false;

    sensor_msgs::PointCloud2Modifier modifier(msg);
    modifier.setPointCloud2Fields(4, "x", 1, sensor_msgs::msg::PointField::FLOAT32, "y", 1,
                                  sensor_msgs::msg::PointField::FLOAT32, "z", 1,
                                  sensor_msgs::msg::PointField::FLOAT32, "intensity", 1,
                                  sensor_msgs::msg::PointField::FLOAT32);
    modifier.resize(points.size());

    sensor_msgs::PointCloud2Iterator<float> iter_x(msg, "x");
    sensor_msgs::PointCloud2Iterator<float> iter_y(msg, "y");
    sensor_msgs::PointCloud2Iterator<float> iter_z(msg, "z");
    sensor_msgs::PointCloud2Iterator<float> iter_intensity(msg, "intensity");
    for (const auto& point : points) {
        *iter_x = static_cast<float>(point.x);
        *iter_y = static_cast<float>(point.y);
        *iter_z = static_cast<float>(point.z);
        *iter_intensity = point.intensity;
        ++iter_x;
        ++iter_y;
        ++iter_z;
        ++iter_intensity;
    }
    return msg;
}

sensor_msgs::msg::PointCloud2 MakeLocalMapCloud(const std::vector<Eigen::Vector3d>& points,
                                                const builtin_interfaces::msg::Time& stamp,
                                                const std::string& frame_id) {
    sensor_msgs::msg::PointCloud2 msg;
    msg.header.stamp = stamp;
    msg.header.frame_id = frame_id;
    msg.height = 1;
    msg.width = static_cast<std::uint32_t>(points.size());
    msg.is_dense = true;
    msg.is_bigendian = false;

    sensor_msgs::PointCloud2Modifier modifier(msg);
    modifier.setPointCloud2Fields(3, "x", 1, sensor_msgs::msg::PointField::FLOAT32, "y", 1,
                                  sensor_msgs::msg::PointField::FLOAT32, "z", 1,
                                  sensor_msgs::msg::PointField::FLOAT32);
    modifier.resize(points.size());

    sensor_msgs::PointCloud2Iterator<float> iter_x(msg, "x");
    sensor_msgs::PointCloud2Iterator<float> iter_y(msg, "y");
    sensor_msgs::PointCloud2Iterator<float> iter_z(msg, "z");
    for (const auto& point : points) {
        *iter_x = static_cast<float>(point.x());
        *iter_y = static_cast<float>(point.y());
        *iter_z = static_cast<float>(point.z());
        ++iter_x;
        ++iter_y;
        ++iter_z;
    }
    return msg;
}

}  // namespace

GlobalMapper::GlobalMapper(const rclcpp::NodeOptions& options)
    : Node("global_mapper", options),
      voxel_map_(),
      lio_buf_(px4_mapping::time::PoseBuffer::kDefaultMaxSamples,
               px4_mapping::time::PoseBuffer::kDefaultWindow),
      px4_buf_(px4_mapping::time::PoseBuffer::kDefaultMaxSamples,
               px4_mapping::time::PoseBuffer::kDefaultWindow),
      timesync_(this->get_clock()) {
    // Declare and read parameters
    this->declare_parameter<bool>("publish_global_map", true);
    this->declare_parameter<bool>("publish_local_map", true);
    this->declare_parameter<bool>("enable_distance_eviction", false);
    this->declare_parameter<int>("global_map_publish_interval", 1);
    this->declare_parameter<int>("log_interval", 1);
    this->declare_parameter<double>("timeout_seconds", 3600.0);
    this->declare_parameter<std::string>("log_path", "");
    this->declare_parameter<std::string>("cloud_topic", "/lio/cloud_registered");
    this->declare_parameter<std::string>("map_topic", "/mapping/global");
    this->declare_parameter<std::string>("local_map_topic", "/mapping/local");
    this->declare_parameter<double>("local_map_radius_m", 30.0);
    this->declare_parameter<std::string>("input_source", "lio_world");
    this->declare_parameter<std::vector<double>>("extrinsic_T",
                                                 std::vector<double>{-0.011, -0.02329, 0.04412});
    this->declare_parameter<int>("ready_min_frames", 5);
    this->declare_parameter<int>("ready_min_occupied", 1000);
    this->declare_parameter<std::string>("lio_odom_topic", "/lio/odometry");
    this->declare_parameter<bool>("use_lio_buffer", true);
    this->declare_parameter<bool>("require_alignment_gate", false);
    this->declare_parameter<double>("aligned_min_seconds", 5.0);
    this->declare_parameter<double>("aligned_max_velocity", 0.05);
    this->declare_parameter<double>("aligned_lio_covariance_max", 0.01);
    this->declare_parameter<double>("aligned_max_seconds_to_capture", 30.0);
    this->declare_parameter<std::string>("aligned_timeout_action", "hold_indefinitely");

    publish_global_map_ = this->get_parameter("publish_global_map").as_bool();
    publish_local_map_ = this->get_parameter("publish_local_map").as_bool();
    enable_distance_eviction_ = this->get_parameter("enable_distance_eviction").as_bool();
    global_map_publish_interval_ = this->get_parameter("global_map_publish_interval").as_int();
    log_interval_ = this->get_parameter("log_interval").as_int();
    timeout_seconds_ = this->get_parameter("timeout_seconds").as_double();
    log_path_ = this->get_parameter("log_path").as_string();
    cloud_topic_ = this->get_parameter("cloud_topic").as_string();
    map_topic_ = this->get_parameter("map_topic").as_string();
    local_map_topic_ = this->get_parameter("local_map_topic").as_string();
    local_map_radius_m_ = this->get_parameter("local_map_radius_m").as_double();
    input_source_ = this->get_parameter("input_source").as_string();

    if (!std::isfinite(local_map_radius_m_) || local_map_radius_m_ <= 0.0) {
        RCLCPP_FATAL(this->get_logger(), "local_map_radius_m must be finite and > 0, got %.3f",
                     local_map_radius_m_);
        throw std::runtime_error("Invalid local_map_radius_m parameter");
    }
    if (global_map_publish_interval_ <= 0) {
        RCLCPP_FATAL(this->get_logger(), "global_map_publish_interval must be > 0, got %d",
                     global_map_publish_interval_);
        throw std::runtime_error("Invalid global_map_publish_interval parameter");
    }

    auto extrinsic_T_vec = this->get_parameter("extrinsic_T").as_double_array();
    if (extrinsic_T_vec.size() != 3) {
        RCLCPP_FATAL(this->get_logger(), "extrinsic_T must have 3 elements, got %zu",
                     extrinsic_T_vec.size());
        throw std::runtime_error("Invalid extrinsic_T parameter");
    }
    T_lidar_in_imu_ = Eigen::Vector3d(extrinsic_T_vec[0], extrinsic_T_vec[1], extrinsic_T_vec[2]);
    RCLCPP_INFO(this->get_logger(), "Extrinsic T_lidar_in_imu loaded: [%.4f, %.4f, %.4f]",
                T_lidar_in_imu_.x(), T_lidar_in_imu_.y(), T_lidar_in_imu_.z());

    ready_min_frames_ = this->get_parameter("ready_min_frames").as_int();
    ready_min_occupied_ = this->get_parameter("ready_min_occupied").as_int();
    lio_odom_topic_ = this->get_parameter("lio_odom_topic").as_string();
    use_lio_buffer_ = this->get_parameter("use_lio_buffer").as_bool();
    RCLCPP_INFO(this->get_logger(), "Path B Option gamma chain: %s",
                use_lio_buffer_ ? "ENABLED" : "DISABLED (rollback to latest-cache)");

    require_alignment_gate_ = this->get_parameter("require_alignment_gate").as_bool();
    aligned_min_seconds_ = this->get_parameter("aligned_min_seconds").as_double();
    aligned_max_velocity_ = this->get_parameter("aligned_max_velocity").as_double();
    aligned_lio_covariance_max_ = this->get_parameter("aligned_lio_covariance_max").as_double();
    aligned_max_seconds_to_capture_ =
        this->get_parameter("aligned_max_seconds_to_capture").as_double();
    aligned_timeout_action_ = this->get_parameter("aligned_timeout_action").as_string();

    if (require_alignment_gate_) {
        RCLCPP_INFO(this->get_logger(), "Alignment gate: ENABLED (timeout_action=%s)",
                    aligned_timeout_action_.c_str());
    } else {
        RCLCPP_INFO(this->get_logger(), "Alignment gate: DISABLED");
    }
    RCLCPP_INFO(this->get_logger(),
                "Readiness gate, requires %d consecutive frames AND %d occupied voxels",
                ready_min_frames_, ready_min_occupied_);

    deskewed_input_ = (input_source_ == "localization_deskew");
    full_pose_input_ = (input_source_ == "px4_full");
    lio_world_input_ = (input_source_ == "lio_world");

    if (!deskewed_input_ && !full_pose_input_ && !lio_world_input_ && input_source_ != "px4_only") {
        RCLCPP_FATAL(this->get_logger(),
                     "Unknown input_source: '%s'. Valid: px4_only, px4_full, "
                     "localization_deskew, lio_world",
                     input_source_.c_str());
        throw std::runtime_error("Invalid input_source parameter");
    }

    RCLCPP_INFO(this->get_logger(), "Input source: %s (cloud_topic=%s map_topic=%s)",
                input_source_.c_str(), cloud_topic_.c_str(), map_topic_.c_str());
    RCLCPP_INFO(this->get_logger(), "Log Interval: every %d frames", log_interval_);
    RCLCPP_INFO(this->get_logger(), "Timeout: %.1f seconds", timeout_seconds_);

    initLogging();

    // Callback groups keep pose updates from waiting on cloud raycast compute
    io_cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    compute_cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    rclcpp::SubscriptionOptions io_opts;
    io_opts.callback_group = io_cb_group_;
    rclcpp::SubscriptionOptions compute_opts;
    compute_opts.callback_group = compute_cb_group_;

    // === Subscribers ===
    const auto qos_sensor = px4_ros2_utils::qos::sensor_qos(50);

    // Subscribe point cloud input according to the selected pipeline mode.
    sub_cloud_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        cloud_topic_, qos_sensor, std::bind(&GlobalMapper::cloudCallback, this, _1), compute_opts);

    // Sensor origin source depends on input_source
    sub_lio_odom_ = this->create_subscription<nav_msgs::msg::Odometry>(
        lio_odom_topic_, rclcpp::QoS(20), std::bind(&GlobalMapper::lioOdomCallback, this, _1),
        io_opts);

    if (!lio_world_input_) {
        // Subscribe PX4 timesync status for accurate PX4 <-> ROS time mapping.
        sub_timesync_status_ = this->create_subscription<px4_msgs::msg::TimesyncStatus>(
            px4_ros2_utils::px4::topic::topic_name<px4_msgs::msg::TimesyncStatus>(
                "/fmu/out/timesync_status"),
            px4_ros2_utils::qos::sensor_qos(5),
            [this](px4_msgs::msg::TimesyncStatus::SharedPtr msg) {
                timesync_.update(msg);
            },
            io_opts);

        // Subscribe PX4 /fmu/out/vehicle_odometry for pose information
        sub_odom_ = this->create_subscription<px4_msgs::msg::VehicleOdometry>(
            px4_ros2_utils::px4::topic::topic_name<px4_msgs::msg::VehicleOdometry>(
                "/fmu/out/vehicle_odometry"),
            px4_ros2_utils::qos::sensor_qos(20),
            [this](px4_msgs::msg::VehicleOdometry::SharedPtr msg) {
                const auto sample_stamp = timesync_.toROS(msg->timestamp_sample);

                if (!sample_stamp) {
                    RCLCPP_WARN_THROTTLE(
                        this->get_logger(), *this->get_clock(), 5000,
                        "[VoxMap] PX4 odometry timestamp_sample invalid or conversion failed "
                        "(px4_us=%lu). Verify UXRCE_DDS_SYNCT and /clock setup.",
                        static_cast<unsigned long>(msg->timestamp_sample));
                    return;
                }
                const int64_t sample_t_ns = sample_stamp->nanoseconds();

                px4_mapping::time::PoseSample sample;
                sample.t_ns = sample_t_ns;
                sample.position =
                    Eigen::Vector3d(msg->position[0], msg->position[1], msg->position[2]);
                sample.orientation =
                    Eigen::Quaterniond(msg->q[0], msg->q[1], msg->q[2], msg->q[3]).normalized();

                {
                    std::lock_guard<std::mutex> lock(odom_mutex_);
                    drone_pos_ = sample.position;
                    drone_q_ = sample.orientation;
                    have_drone_q_ = true;
                }
                // Push for SLERP lookup, internal monotonic guard handles drops
                px4_buf_.Push(sample);
            },
            io_opts);

        // Alignment gate inputs: arming state and EKF position/velocity validity
        sub_status_ = this->create_subscription<px4_msgs::msg::VehicleStatus>(
            px4_ros2_utils::px4::topic::topic_name<px4_msgs::msg::VehicleStatus>(
                "/fmu/out/vehicle_status"),
            px4_ros2_utils::qos::sensor_qos(5),
            std::bind(&GlobalMapper::vehicleStatusCallback, this, _1), io_opts);

        sub_local_pos_ = this->create_subscription<px4_msgs::msg::VehicleLocalPosition>(
            px4_ros2_utils::px4::topic::topic_name<px4_msgs::msg::VehicleLocalPosition>(
                "/fmu/out/vehicle_local_position"),
            px4_ros2_utils::qos::sensor_qos(5),
            std::bind(&GlobalMapper::vehicleLocalPositionCallback, this, _1), io_opts);
    }

    // === Publishers ===
    pub_global_map_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(map_topic_, 20);
    pub_local_map_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(local_map_topic_, 20);
    RCLCPP_INFO(this->get_logger(),
                "Map outputs: global=%s (%s, every %d frame%s), "
                "local=%s (%s, radius=%.1fm)",
                map_topic_.c_str(), publish_global_map_ ? "enabled" : "disabled",
                global_map_publish_interval_, global_map_publish_interval_ == 1 ? "" : "s",
                local_map_topic_.c_str(), publish_local_map_ ? "enabled" : "disabled",
                local_map_radius_m_);

    // === Timers ===
    last_data_time_ = this->now();

    timeout_timer_ =
        this->create_wall_timer(std::chrono::seconds(1),
                                std::bind(&GlobalMapper::timeoutCallback, this), compute_cb_group_);

    // Alignment monitor runs at 10 Hz on the IO callback group so that
    // long-running cloud raycasting in the compute group cannot starve it.
    if (require_alignment_gate_ && !lio_world_input_) {
        alignment_start_time_ = this->now();
        alignment_timer_ =
            this->create_wall_timer(std::chrono::milliseconds(100),
                                    std::bind(&GlobalMapper::alignmentTick, this), io_cb_group_);
    } else {
        // Gate disabled or lio_world mode
        alignment_captured_.store(true, std::memory_order_release);
    }

    // Optional distance eviction is independent of the input pipeline. It is
    // disabled by default so /mapping/global retains history in every mode;
    // /mapping/local remains bounded by its query radius either way.
    if (enable_distance_eviction_) {
        eviction_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(px4_nav_common::mapping::kEvictIntervalMs),
            [this]() {
                Eigen::Vector3d pos;
                {
                    std::lock_guard<std::mutex> lock(odom_mutex_);
                    pos = drone_pos_;
                }
                const size_t evicted = voxel_map_.EvictDistant(pos);
                if (evicted > 0) {
                    RCLCPP_DEBUG(this->get_logger(), "Evicted %zu voxels (>%.0fm)", evicted,
                                 px4_nav_common::mapping::kEvictRadiusM);
                }
            },
            compute_cb_group_);
    }

    RCLCPP_INFO(this->get_logger(),
                "Voxel Map Manager (Layer 2) initialized. Retention: %s + age/capacity bounds",
                enable_distance_eviction_ ? "distance-bounded" : "global");

    if (!enable_distance_eviction_) {
        RCLCPP_WARN(this->get_logger(),
                    "Distance eviction disabled: /mapping/global retains history across the "
                    "flight and is bounded by kMaxVoxels=%zu and kMaxFrameAge=%d.",
                    static_cast<size_t>(px4_nav_common::mapping::kMaxVoxels),
                    px4_nav_common::mapping::kMaxFrameAge);
    }

    RCLCPP_INFO(this->get_logger(), "GlobalMapper initialized");
}

GlobalMapper::~GlobalMapper() {
    writeSummary();
    saveGlobalMap();
    printf("\n[VoxMap] Node shutting down\n");
}

// === IVoxMapManager interface ===
double GlobalMapper::GetResolution() const {
    return voxel_map_.GetResolution();
}

void GlobalMapper::GetOccupiedPointsInRadius(const Eigen::Vector3d& center, double radius,
                                             std::vector<Eigen::Vector3d>& out) {
    voxel_map_.GetOccupiedPointsInRadius(center, radius, out);
}

bool GlobalMapper::IsReady() const noexcept {
    // Explicit boolean expression: fresh data + coverage + sustained frames +
    // no fatal fault + alignment captured. This replaces the previous one-way
    // ready_ latch so that readiness drops when data goes stale or the map
    // becomes sparse (e.g. flying into open space).
    const bool fresh = data_fresh_.load(std::memory_order_acquire);
    const bool coverage = coverage_ok_.load(std::memory_order_acquire);
    const bool sustained =
        ready_consecutive_frames_.load(std::memory_order_acquire) >= ready_min_frames_;
    const bool no_fault = !fatal_fault_.load(std::memory_order_acquire);
    const bool aligned = alignment_captured_.load(std::memory_order_acquire);
    return fresh && coverage && sustained && no_fault && aligned;
}

std::uint64_t GlobalMapper::FramesDropped() const noexcept {
    return frames_dropped_.load(std::memory_order_relaxed);
}

Eigen::Vector3d GlobalMapper::GetExtrinsicTranslation() const noexcept {
    return T_lidar_in_imu_;
}

/* =====================================================================
    saveGlobalMap, dump entire occupied map to PCD and PLY on shutdown
   ===================================================================== */
void GlobalMapper::saveGlobalMap() {
    // Simplified implementation without PCL
    printf("\n[VoxMap] Global map saving not implemented (PCL not available)\n");
}

/* =====================================================================
    initLogging, open CSV timing log file in log_path_
   ===================================================================== */
/* Helper: create directory tree recursively (like mkdir -p) */
void GlobalMapper::mkdirRecursive(const std::string& path) {
    std::string acc;
    for (size_t i = 0; i < path.size(); ++i) {
        acc += path[i];
        if (path[i] == '/' || i == path.size() - 1) {
            mkdir(acc.c_str(), 0755);
        }
    }
}

void GlobalMapper::initLogging() {
    if (log_path_.empty()) {
        log_path_ = "./src/Log/mapping";
    }

    mkdirRecursive(log_path_);

    auto now = std::chrono::system_clock::now();
    auto time_t = std::chrono::system_clock::to_time_t(now);
    std::tm tm_buf{};
    localtime_r(&time_t, &tm_buf);

    std::ostringstream oss;
    oss << log_path_ << "/global_mapper_" << std::put_time(&tm_buf, "%Y%m%d_%H%M%S") << ".log";
    // Not storing log_filename_ as member since we're not using it in this implementation

    // For simplicity, we're not implementing full logging as in the original
    // In a real implementation, you would open and manage the log file here
}

/* =====================================================================
    writeSummary, append aggregate timing stats to log file on shutdown
   ===================================================================== */
void GlobalMapper::writeSummary() {
    const auto occupied_points = voxel_map_.GetPointCloud();

    printf("\n============ VOXMAP SUMMARY ============\n");
    printf("Total Frames:      %lu\n", frame_count_);
    printf("Frames Dropped:    %lu\n", frames_dropped_.load(std::memory_order_relaxed));
    printf("Allocated Voxels:  %zu\n", voxel_map_.Size());
    printf("Occupied Voxels:   %zu\n", occupied_points.size());
    printf("Voxel Resolution:  %.3f m\n", voxel_map_.GetResolution());
    if (!occupied_points.empty()) {
        double min_x = occupied_points.front().x;
        double min_y = occupied_points.front().y;
        double min_z = occupied_points.front().z;
        double max_x = min_x;
        double max_y = min_y;
        double max_z = min_z;
        for (const auto& point : occupied_points) {
            min_x = std::min(min_x, point.x);
            min_y = std::min(min_y, point.y);
            min_z = std::min(min_z, point.z);
            max_x = std::max(max_x, point.x);
            max_y = std::max(max_y, point.y);
            max_z = std::max(max_z, point.z);
        }
        printf("Occupied Bounds:  X[%.2f, %.2f] Y[%.2f, %.2f] Z[%.2f, %.2f] m\n", min_x, max_x,
               min_y, max_y, min_z, max_z);
    }
    printf("Final Ready State: %s\n", IsReady() ? "true" : "false");
    if (fatal_fault_.load(std::memory_order_relaxed)) {
        printf("FATAL FAULT: timestamp-domain contamination or unrecoverable error occurred\n");
    }
    if (frame_count_ > 0) {
        double mean_total = total_process_time_ms_ / frame_count_;
        double mean_update = total_update_time_ms_ / frame_count_;
        double mean_publish = total_publish_time_ms_ / frame_count_;
        printf("Mean Update Time:  %.3f ms\n", mean_update);
        printf("Mean Publish Time: %.3f ms\n", mean_publish);
        printf("Mean Total Time:   %.3f ms\n", mean_total);
        printf("Min: %.3f ms | Max: %.3f ms\n", min_time_ms_, max_time_ms_);
    }
    printf("========================================\n");
}

/* =====================================================================
    timeoutCallback, clear map if no lidar data arrived recently
   ===================================================================== */
void GlobalMapper::timeoutCallback() {
    // Timeout state and PointCloud2 headers stay in the node-owned ROS domain.
    const rclcpp::Time ros_now = this->now();
    const double elapsed = (ros_now - last_data_time_).seconds();
    const bool fresh = elapsed < timeout_seconds_;
    data_fresh_.store(fresh, std::memory_order_release);

    if (elapsed > timeout_seconds_ && !map_cleared_) {
        printf("[VoxMap] TIMEOUT: Clearing map after %.1fs\n", elapsed);
        fflush(stdout);

        voxel_map_.Clear();

        // Publish empty clouds so RViz/downstream rolling views do not retain stale points.
        const std::string frame_id =
            (lio_world_input_ && !last_frame_id_.empty()) ? last_frame_id_ : "map_ned";
        if (publish_global_map_) {
            pub_global_map_->publish(MakeGlobalMapCloud({}, ros_now, frame_id));
        }
        if (publish_local_map_) {
            pub_local_map_->publish(MakeLocalMapCloud({}, ros_now, frame_id));
        }

        map_cleared_ = true;
        coverage_ok_.store(false, std::memory_order_release);
        ready_consecutive_frames_.store(0, std::memory_order_release);
    }
}

/* =====================================================================
    alignmentTick, 10 Hz check of the alignment gate conditions
   ===================================================================== */
void GlobalMapper::alignmentTick() {
    if (alignment_captured_.load(std::memory_order_acquire)) {
        return;
    }

    const bool c1 = armed_.load(std::memory_order_acquire);
    const bool c2 = drone_speed_.load(std::memory_order_acquire) < aligned_max_velocity_;
    const bool c3 = ekf_pose_valid_.load(std::memory_order_acquire);
    const bool c4 =
        lio_covariance_trace_.load(std::memory_order_acquire) < aligned_lio_covariance_max_;

    const rclcpp::Time ros_now = this->now();
    if (c1 && c2 && c3 && c4) {
        if (aligned_streak_start_.nanoseconds() == 0) {
            aligned_streak_start_ = ros_now;
        }
        const double streak_s = (ros_now - aligned_streak_start_).seconds();
        if (streak_s >= aligned_min_seconds_) {
            alignment_captured_.store(true, std::memory_order_release);
            RCLCPP_INFO(this->get_logger(),
                        "Alignment captured after %.1fs sustained 4-condition gate", streak_s);
            return;
        }
    } else {
        aligned_streak_start_ = rclcpp::Time(0, 0, this->get_clock()->get_clock_type());
    }

    // Apply timeout fallback
    const double since_start = (ros_now - alignment_start_time_).seconds();
    if (since_start > aligned_max_seconds_to_capture_ && !alignment_warned_) {
        alignment_warned_ = true;
        RCLCPP_ERROR(this->get_logger(),
                     "Alignment not captured after %.0fs, conditions C1=%d C2=%d "
                     "C3=%d C4=%d, action=%s",
                     since_start, c1, c2, c3, c4, aligned_timeout_action_.c_str());
        if (aligned_timeout_action_ == "warn_and_proceed") {
            alignment_degraded_.store(true, std::memory_order_release);
            alignment_captured_.store(true, std::memory_order_release);
            RCLCPP_WARN(this->get_logger(),
                        "Proceeding with degraded alignment, L3 should refuse "
                        "aggressive maneuver");
        }
    }
}

/* =====================================================================
    vehicleStatusCallback, arming state for the alignment gate
   ===================================================================== */
void GlobalMapper::vehicleStatusCallback(const px4_msgs::msg::VehicleStatus::SharedPtr msg) {
    armed_.store(msg->arming_state == px4_msgs::msg::VehicleStatus::ARMING_STATE_ARMED,
                 std::memory_order_release);
}

/* =====================================================================
    vehicleLocalPositionCallback, EKF validity + speed for the alignment gate
   ===================================================================== */
void GlobalMapper::vehicleLocalPositionCallback(
    const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg) {
    const bool ekf_ok = msg->xy_valid && msg->z_valid;
    ekf_pose_valid_.store(ekf_ok, std::memory_order_release);

    const double vx = static_cast<double>(msg->vx);
    const double vy = static_cast<double>(msg->vy);
    const double vz = static_cast<double>(msg->vz);
    drone_speed_.store(std::sqrt(vx * vx + vy * vy + vz * vz), std::memory_order_release);
}

/* =====================================================================
    lioOdomCallback, always-on LIO pose handler
   ===================================================================== */
void GlobalMapper::lioOdomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    if (lio_samples_received_ == 0) {
        lio_first_sample_time_ = this->now();
    }
    lio_samples_received_++;

    // Wall-clock contamination guard. Only active when the node is explicitly
    // using simulation time and a /clock source is present (clock not stuck at 0).
    if (this->get_parameter("use_sim_time").as_bool() && lio_samples_received_ > 20) {
        const rclcpp::Time clock_now = this->now();
        if (clock_now.nanoseconds() > 1'000'000'000LL) {
            const rclcpp::Time stamp(msg->header.stamp);
            const double age_ms = (clock_now - stamp).seconds() * 1000.0;
            if (std::abs(age_ms) > 5000.0) {
                RCLCPP_FATAL(this->get_logger(),
                             "LIO /odometry stamp domain mismatch, stamp=%d.%09u, "
                             "clock_now=%.3f, age=%.1fms, likely wall-clock contamination "
                             "under use_sim_time=true",
                             msg->header.stamp.sec, msg->header.stamp.nanosec, clock_now.seconds(),
                             age_ms);
                fatal_fault_.store(true, std::memory_order_release);
                RCLCPP_FATAL(this->get_logger(),
                             "Fatal fault set: LIO /odometry stamp domain mismatch. "
                             "Node will report IsReady()=false until restart.");
                return;
            }
        }
    }

    // Build the timestamped sample
    px4_mapping::time::PoseSample sample;
    sample.t_ns = static_cast<int64_t>(msg->header.stamp.sec) * 1'000'000'000LL +
                  static_cast<int64_t>(msg->header.stamp.nanosec);
    sample.position = Eigen::Vector3d(msg->pose.pose.position.x, msg->pose.pose.position.y,
                                      msg->pose.pose.position.z);
    sample.orientation =
        Eigen::Quaterniond(msg->pose.pose.orientation.w, msg->pose.pose.orientation.x,
                           msg->pose.pose.orientation.y, msg->pose.pose.orientation.z)
            .normalized();

    {
        std::lock_guard<std::mutex> lock(odom_mutex_);
        if (lio_world_input_) {
            // Mode 12, position is the raycast origin in camera_init
            drone_pos_ = sample.position;
            // yaw unused in lio_world mode
            drone_yaw_ = 0.0;
            cos_yaw_ = 1.0;
            sin_yaw_ = 0.0;
        }
        // Cache LIO quaternion for raycast origin lookup in PX4-driven modes
        lio_q_camera_init_ = sample.orientation;
        have_lio_q_ = true;
    }

    // Push to timestamped buffer for SLERP lookup
    lio_buf_.Push(sample);

    // Diagonal trace of position covariance
    const double cov_trace =
        msg->pose.covariance[0] + msg->pose.covariance[7] + msg->pose.covariance[14];
    lio_covariance_trace_.store(cov_trace, std::memory_order_release);

    const double elapsed = (this->now() - lio_first_sample_time_).seconds();
    if (elapsed < 10.0) {
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                             "[LIO] received %zu samples, latest stamp=%d.%09u, rate=%.1f Hz, "
                             "buf_size=%zu, non_mono=%lu, overflow=%lu",
                             lio_samples_received_, msg->header.stamp.sec,
                             msg->header.stamp.nanosec,
                             lio_samples_received_ / std::max(elapsed, 0.1), lio_buf_.Size(),
                             static_cast<unsigned long>(lio_buf_.NonMonotonicCount()),
                             static_cast<unsigned long>(lio_buf_.OverflowCount()));
    }
}

/* =====================================================================
    hasField, helper to check PointCloud2 for a named field
   ===================================================================== */
bool GlobalMapper::hasField(const sensor_msgs::msg::PointCloud2& msg, const std::string& name) {
    for (const auto& field : msg.fields) {
        if (field.name == name)
            return true;
    }
    return false;
}

/* =====================================================================
    cloudCallback, main raycasting pipeline
   ===================================================================== */
void GlobalMapper::cloudCallback(sensor_msgs::msg::PointCloud2::UniquePtr msg) {
    last_data_time_ = this->now();
    last_frame_id_ = msg->header.frame_id;

    if (map_cleared_) {
        printf("[VoxMap] Data received. Starting...\n");

        printf("[VoxMap] Fields: ");
        for (const auto& f : msg->fields) {
            printf("%s(off=%d,type=%d) ", f.name.c_str(), f.offset, f.datatype);
        }
        printf("\n");

        // Cache field offsets
        cached_point_step_ = msg->point_step;
        off_x_ = 0;
        off_y_ = 4;
        off_z_ = 8;
        off_intensity_ = 0;
        for (const auto& f : msg->fields) {
            if (f.name == "x")
                off_x_ = f.offset;
            else if (f.name == "y")
                off_y_ = f.offset;
            else if (f.name == "z")
                off_z_ = f.offset;
            else if (f.name == "intensity")
                off_intensity_ = f.offset;
        }

        map_cleared_ = false;
    }

    size_t num_points = msg->width * msg->height;
    if (num_points == 0) {
        frames_dropped_.fetch_add(1, std::memory_order_relaxed);
        return;
    }

    // Extract points with raw pointer stride
    const uint8_t* data_ptr = msg->data.data();
    const uint32_t step = cached_point_step_;

    input_points_.clear();
    input_points_.reserve(num_points);

    for (size_t i = 0; i < num_points; ++i) {
        const uint8_t* p = data_ptr + i * step;
        float x = *reinterpret_cast<const float*>(p + off_x_);
        float y = *reinterpret_cast<const float*>(p + off_y_);
        float z = *reinterpret_cast<const float*>(p + off_z_);
        float intensity = *reinterpret_cast<const float*>(p + off_intensity_);

        if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z))
            continue;

        px4_nav_common::PointLivox pt;
        pt.x = static_cast<double>(x);
        pt.y = static_cast<double>(y);
        pt.z = static_cast<double>(z);
        pt.intensity = intensity;
        input_points_.push_back(pt);
    }

    if (input_points_.empty()) {
        frames_dropped_.fetch_add(1, std::memory_order_relaxed);
        return;
    }

    // Transform FLU sensor frame to NED world frame
    Eigen::Vector3d pos;
    double cy, sy;
    Eigen::Quaterniond q_ned_frd;
    bool have_q;
    {
        std::lock_guard<std::mutex> lock(odom_mutex_);
        pos = drone_pos_;
        cy = cos_yaw_;
        sy = sin_yaw_;
        q_ned_frd = drone_q_;
        have_q = have_drone_q_;
    }

    // Path B Option gamma chain attempt
    const int64_t cloud_t_ns = static_cast<int64_t>(msg->header.stamp.sec) * 1'000'000'000LL +
                               static_cast<int64_t>(msg->header.stamp.nanosec);
    px4_mapping::time::PoseSample px4_at_cloud, lio_at_cloud;
    bool chain_ok = false;
    bool lio_world_pose_ok = false;
    if (lio_world_input_) {
        lio_world_pose_ok = lio_buf_.Lookup(cloud_t_ns, lio_at_cloud);
    } else if (use_lio_buffer_) {
        const bool px4_hit = px4_buf_.Lookup(cloud_t_ns, px4_at_cloud);
        const bool lio_hit = lio_buf_.Lookup(cloud_t_ns, lio_at_cloud);
        chain_ok = px4_hit && lio_hit;
    }

    Eigen::Vector3d sensor_world;
    if (chain_ok) {
        // Path B Option gamma, scan-time pose chain
        const Eigen::Vector3d t_ned = px4_at_cloud.orientation.toRotationMatrix() *
                                      px4_ros2_utils::frame::R_FLU_TO_FRD *
                                      lio_at_cloud.orientation.toRotationMatrix() * T_lidar_in_imu_;
        sensor_world = px4_at_cloud.position + t_ned;
    } else if (lio_world_input_) {
        // The registered cloud is already in lio_world, but raycasting still
        // needs the synchronized LiDAR origin. Derive it from the LIO IMU pose
        // and rotate the calibrated IMU-to-LiDAR translation.
        if (!lio_world_pose_ok) {
            frames_dropped_.fetch_add(1, std::memory_order_relaxed);
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                                 "lio_world mode: waiting for synchronized LIO odometry");
            return;
        }
        sensor_world =
            lio_at_cloud.position + lio_at_cloud.orientation.toRotationMatrix() * T_lidar_in_imu_;
    } else if (full_pose_input_ && have_q) {
        // px4_full fallback
        const Eigen::Vector3d t_ned =
            q_ned_frd.toRotationMatrix() * px4_ros2_utils::frame::R_FLU_TO_FRD * T_lidar_in_imu_;
        sensor_world = pos + t_ned;
    } else {
        // Yaw-only chain
        const Eigen::Vector3d t_frd = px4_ros2_utils::frame::flu_to_frd(T_lidar_in_imu_);
        sensor_world =
            Eigen::Vector3d(pos.x() + cy * t_frd.x() - sy * t_frd.y(),
                            pos.y() + sy * t_frd.x() + cy * t_frd.y(), pos.z() + t_frd.z());
    }

    // Save first raw point for periodic transform debug log
    double dbg_raw_x = 0, dbg_raw_y = 0, dbg_raw_z = 0;
    if (!input_points_.empty()) {
        dbg_raw_x = input_points_[0].x;
        dbg_raw_y = input_points_[0].y;
        dbg_raw_z = input_points_[0].z;
    }

    if (full_pose_input_) {
        // px4_full mode
        if (!have_q) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                                 "px4_full mode: waiting for /fmu/out/vehicle_odometry");
            return;
        }
        const Eigen::Matrix3d R_ned_flu =
            q_ned_frd.toRotationMatrix() * px4_ros2_utils::frame::R_FLU_TO_FRD;
        for (auto& pt : input_points_) {
            Eigen::Vector3d p_flu(pt.x, pt.y, pt.z);
            Eigen::Vector3d p_ned = R_ned_flu * p_flu + pos;
            pt.x = p_ned.x();
            pt.y = p_ned.y();
            pt.z = p_ned.z();
        }
    } else if (lio_world_input_) {
        // lio_world mode, points already in L1 camera_init world frame
    } else if (!deskewed_input_) {
        // px4_only mode: legacy yaw-only transform
        const double offset_bz = -T_lidar_in_imu_.z();
        for (auto& pt : input_points_) {
            // FLU to body NED, then add sensor offset
            double bx = pt.x;
            double by = -pt.y;
            double bz = -pt.z + offset_bz;

            // Yaw rotation, then translate to world NED
            pt.x = cy * bx - sy * by + pos.x();
            pt.y = sy * bx + cy * by + pos.y();
            pt.z = bz + pos.z();
        }
    }

    // Periodic transform debug
    if (frame_count_ % 100 == 0 && !input_points_.empty()) {
        RCLCPP_INFO(this->get_logger(),
                    "[TF_DBG] F%lu yaw=%.1f° pos=(%.1f,%.1f,%.1f) | "
                    "raw=(%.2f,%.2f,%.2f) -> world=(%.2f,%.2f,%.2f)",
                    frame_count_ + 1, std::atan2(sy, cy) * 180.0 / M_PI, pos.x(), pos.y(), pos.z(),
                    dbg_raw_x, dbg_raw_y, dbg_raw_z, input_points_[0].x, input_points_[0].y,
                    input_points_[0].z);
    }

    // Update voxel map with raycasting from sensor world position
    auto t0 = std::chrono::high_resolution_clock::now();
    voxel_map_.Update(input_points_, sensor_world);

    auto t1 = std::chrono::high_resolution_clock::now();

    // Coverage-based readiness gate: explicit condition update every frame.
    // readiness is recomputed on demand in IsReady() as the conjunction of
    // data_fresh_, coverage_ok_, sustained consecutive frames, no fatal fault,
    // and alignment_captured_. This avoids the previous one-way latch bug where
    // the node stayed ready after flying into open space.
    data_fresh_.store(true, std::memory_order_release);
    const size_t occ = voxel_map_.OccupiedCount();
    const bool coverage = occ >= static_cast<size_t>(ready_min_occupied_);
    coverage_ok_.store(coverage, std::memory_order_release);

    if (coverage) {
        const int prev = ready_consecutive_frames_.fetch_add(1, std::memory_order_acq_rel);
        if (prev + 1 == ready_min_frames_) {
            RCLCPP_INFO(this->get_logger(), "VoxMap ready, %d frames with %zu occupied voxels",
                        prev + 1, occ);
        }
    } else {
        ready_consecutive_frames_.store(0, std::memory_order_release);
    }

    const std::string map_frame_id =
        (lio_world_input_ && !last_frame_id_.empty()) ? last_frame_id_ : "map_ned";

    // Publish complete snapshots at the configured frame interval. RViz PointCloud2
    // displays replace the previous cloud, so publishing per-frame deltas would make
    // accumulated surfaces disappear. Frame 1 is always published immediately.
    if (publish_global_map_ &&
        ((frame_count_ - 1U) % static_cast<std::uint64_t>(global_map_publish_interval_) == 0U)) {
        const auto map_points = voxel_map_.GetPointCloud();
        pub_global_map_->publish(MakeGlobalMapCloud(map_points, msg->header.stamp, map_frame_id));
    }

    // The local map is a stateless rolling view derived from global occupancy.
    // Rebuilding it every frame makes points outside the planning radius disappear
    // immediately without deleting them from the global map.
    if (publish_local_map_) {
        Eigen::Vector3d local_map_center = pos;
        if (chain_ok) {
            local_map_center = px4_at_cloud.position;
        } else if (lio_world_input_ && lio_world_pose_ok) {
            local_map_center = lio_at_cloud.position;
        }
        voxel_map_.GetOccupiedPointsInRadius(local_map_center, local_map_radius_m_,
                                             local_map_points_);
        pub_local_map_->publish(
            MakeLocalMapCloud(local_map_points_, msg->header.stamp, map_frame_id));
    }

    auto t2 = std::chrono::high_resolution_clock::now();

    // === Timing and Logging ===
    std::chrono::duration<double, std::milli> update_elapsed = t1 - t0;
    std::chrono::duration<double, std::milli> publish_elapsed = t2 - t1;
    std::chrono::duration<double, std::milli> total_elapsed = t2 - t0;

    double update_ms = update_elapsed.count();
    double publish_ms = publish_elapsed.count();
    double total_ms = total_elapsed.count();

    total_process_time_ms_ += total_ms;
    total_update_time_ms_ += update_ms;
    total_publish_time_ms_ += publish_ms;
    if (total_ms < min_time_ms_)
        min_time_ms_ = total_ms;
    if (total_ms > max_time_ms_)
        max_time_ms_ = total_ms;

    frame_count_++;

    if (log_interval_ <= 1 || frame_count_ % log_interval_ == 0) {
        const uint64_t fd = frames_dropped_.load(std::memory_order_relaxed);
        const uint64_t fd_delta = fd - frames_dropped_at_last_log_;
        frames_dropped_at_last_log_ = fd;
        if (voxel_map_.WasTimedOut()) {
            RCLCPP_WARN(this->get_logger(),
                        "[VoxMap] F%lu: %zu/%zu pts (TIMEOUT) | upd %.1fms pub %.1fms = %.1fms | "
                        "%zu vox (%zu new %zu upd) | drop %lu (+%lu) | Mean: %.1fms",
                        frame_count_, voxel_map_.PointsProcessed(), input_points_.size(), update_ms,
                        publish_ms, total_ms, voxel_map_.Size(), voxel_map_.NewCount(),
                        voxel_map_.UpdatedCount(), fd, fd_delta,
                        total_process_time_ms_ / frame_count_);
        } else {
            RCLCPP_INFO(this->get_logger(),
                        "[VoxMap] F%lu: %zu pts | upd %.1fms pub %.1fms = %.1fms | "
                        "%zu vox (%zu new %zu upd) | drop %lu (+%lu)",
                        frame_count_, input_points_.size(), update_ms, publish_ms, total_ms,
                        voxel_map_.Size(), voxel_map_.NewCount(), voxel_map_.UpdatedCount(), fd,
                        fd_delta);
        }
    }
}

// Factory function for composed pipeline
std::shared_ptr<rclcpp::Node> get_global_mapper_node(
    const rclcpp::NodeOptions& options,
    std::shared_ptr<px4_nav_common::mapping::IVoxMapManager>& out_iface) {
    auto mgr = std::make_shared<GlobalMapper>(options);
    out_iface = mgr;
    return mgr;
}

}  // namespace px4_mapping
