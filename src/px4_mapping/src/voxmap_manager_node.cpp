/* =========================================================================
    voxmap_manager_node.cpp, Layer 2 ROS 2 node wrapping VoxelHashMap

    1. VoxMapManagerNode
       - Subscribes raw lidar /livox/lidar and PX4 VehicleOdometry
       - Transforms raw points from sensor FLU to world NED before raycasting
       - Implements IVoxMapManager so Layer 3 can query resolution
       - Runs distance based eviction timer to bound map memory
       - Publishes /livox_map for RViz and Layer 3 ring buffer (intra-process)

    2. Factory
       - get_voxmap_node returns Node and IVoxMapManager interface
       - Composed pipeline uses single instance for both roles
   ========================================================================= */

#include "px4_mapping/voxmap_manager_node.hpp"

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

#include <px4_common/frame_constants.hpp>
#include <px4_common/mapping/voxel_map_interface.hpp>
#include <px4_common/mapping/voxel_types.hpp>
#include <px4_common/math/grid.hpp>
#include <px4_common/math/transforms.hpp>
#include <px4_common/time/pose_buffer.hpp>
#include <px4_common/types.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>
#include <px4_ros_com/frame_transforms.hpp>
#include <px4_ros_com/topic_helpers.hpp>

#include <px4_mapping/voxel_hash_map.hpp>

using std::placeholders::_1;

namespace px4_mapping {

VoxMapManagerNode::VoxMapManagerNode(const rclcpp::NodeOptions& options)
    : Node("vox_map_manager", options),
      voxel_map_(),
      lio_buf_(px4_common::time::PoseBuffer::kDefaultMaxSamples,
               px4_common::time::PoseBuffer::kDefaultWindow),
      px4_buf_(px4_common::time::PoseBuffer::kDefaultMaxSamples,
               px4_common::time::PoseBuffer::kDefaultWindow) {
    // Declare and read parameters
    this->declare_parameter<bool>("publish_local_map", true);
    this->declare_parameter<int>("log_interval", 1);
    this->declare_parameter<double>("timeout_seconds", 3600.0);
    this->declare_parameter<std::string>("log_path", "");
    this->declare_parameter<std::string>("cloud_topic", "/livox/lidar");
    this->declare_parameter<std::string>("input_source", "px4_only");
    this->declare_parameter<std::vector<double>>("extrinsic_T",
                                                 std::vector<double>{-0.011, -0.02329, 0.04412});
    this->declare_parameter<int>("ready_min_frames", 5);
    this->declare_parameter<int>("ready_min_occupied", 1000);
    this->declare_parameter<std::string>("lio_odom_topic", "/odometry");
    this->declare_parameter<bool>("use_lio_buffer", true);
    this->declare_parameter<bool>("require_alignment_gate", true);
    this->declare_parameter<double>("aligned_min_seconds", 5.0);
    this->declare_parameter<double>("aligned_max_velocity", 0.05);
    this->declare_parameter<double>("aligned_lio_covariance_max", 0.01);
    this->declare_parameter<double>("aligned_max_seconds_to_capture", 30.0);
    this->declare_parameter<std::string>("aligned_timeout_action", "hold_indefinitely");

    publish_local_map_ = this->get_parameter("publish_local_map").as_bool();
    log_interval_ = this->get_parameter("log_interval").as_int();
    timeout_seconds_ = this->get_parameter("timeout_seconds").as_double();
    log_path_ = this->get_parameter("log_path").as_string();
    cloud_topic_ = this->get_parameter("cloud_topic").as_string();
    input_source_ = this->get_parameter("input_source").as_string();

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
    RCLCPP_INFO(this->get_logger(),
                "Alignment gate: %s, min=%.1fs, vel<%.2fm/s, cov<%.3fm^2, capture_timeout=%.0fs, "
                "timeout_action=%s",
                require_alignment_gate_ ? "ENABLED" : "DISABLED", aligned_min_seconds_,
                aligned_max_velocity_, aligned_lio_covariance_max_, aligned_max_seconds_to_capture_,
                aligned_timeout_action_.c_str());
    RCLCPP_INFO(this->get_logger(),
                "Readiness gate, requires %d consecutive frames AND %d occupied voxels",
                ready_min_frames_, ready_min_occupied_);

    deskewed_input_ = (input_source_ == "fast_lio2_deskew");
    full_pose_input_ = (input_source_ == "px4_full");
    lio_world_input_ = (input_source_ == "lio_world");

    if (!deskewed_input_ && !full_pose_input_ && !lio_world_input_ && input_source_ != "px4_only") {
        RCLCPP_FATAL(this->get_logger(),
                     "Unknown input_source: '%s'. Valid: px4_only, px4_full, "
                     "fast_lio2_deskew, lio_world",
                     input_source_.c_str());
        throw std::runtime_error("Invalid input_source parameter");
    }

    RCLCPP_INFO(this->get_logger(), "Input source: %s (topic=%s)", input_source_.c_str(),
                cloud_topic_.c_str());
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
    auto qos_sensor = rclcpp::SensorDataQoS();
    qos_sensor.keep_last(50);

    // Subscribe lidar cloud (raw /livox/lidar or Layer 1 deskewed NED)
    sub_cloud_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        cloud_topic_, qos_sensor, std::bind(&VoxMapManagerNode::cloudCallback, this, _1),
        compute_opts);

    // Sensor origin source depends on input_source
    sub_lio_odom_ = this->create_subscription<nav_msgs::msg::Odometry>(
        lio_odom_topic_, rclcpp::QoS(20), std::bind(&VoxMapManagerNode::lioOdomCallback, this, _1),
        io_opts);

    if (!lio_world_input_) {
        // Subscribe PX4 /fmu/out/vehicle_odometry for pose information
        sub_odom_ = this->create_subscription<px4_msgs::msg::VehicleOdometry>(
            px4_ros_com::topic::Px4TopicName<px4_msgs::msg::VehicleOdometry>(
                "/fmu/out/vehicle_odometry"),
            rclcpp::QoS(20).best_effort(),
            [this](px4_msgs::msg::VehicleOdometry::SharedPtr msg) {
                // Translate PX4 timestamp_sample (wall-clock via MicroXRCEAgent
                // CLOCK_REALTIME after UXRCE_DDS_SYNCT) into the ROS time domain
                const int64_t px4_wall_ns = static_cast<int64_t>(msg->timestamp_sample) * 1000LL;
                if (!px4_offset_initialized_.load(std::memory_order_acquire)) {
                    const int64_t now_ns = this->now().nanoseconds();
                    // /clock topic may not have reached this node yet
                    if (now_ns < 1'000'000'000LL)
                        return;
                    px4_to_ros_offset_ns_.store(now_ns - px4_wall_ns, std::memory_order_release);
                    px4_offset_initialized_.store(true, std::memory_order_release);
                    RCLCPP_INFO(this->get_logger(),
                                "PX4 timestamp offset captured at sim_time=%.3f: %.3fs "
                                "(wall-clock to ROS time)",
                                now_ns * 1e-9, (now_ns - px4_wall_ns) * 1e-9);
                }

                px4_common::time::PoseSample sample;
                sample.t_ns = px4_wall_ns + px4_to_ros_offset_ns_.load(std::memory_order_acquire);
                sample.position =
                    Eigen::Vector3d(msg->position[0], msg->position[1], msg->position[2]);
                sample.orientation =
                    Eigen::Quaterniond(msg->q[0], msg->q[1], msg->q[2], msg->q[3]).normalized();

                {
                    std::lock_guard<std::mutex> lock(odom_mutex_);
                    drone_q_ = sample.orientation;
                    have_drone_q_ = true;
                }
                // Push for SLERP lookup, internal monotonic guard handles drops
                px4_buf_.Push(sample);
            },
            io_opts);
    }

    // === Publishers ===
    pub_map_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/livox_map", 20);

    // === Timers ===
    last_data_time_ = this->now();

    timeout_timer_ = this->create_wall_timer(std::chrono::seconds(1),
                                             std::bind(&VoxMapManagerNode::timeoutCallback, this),
                                             compute_cb_group_);

    // Alignment monitor runs at 10 Hz
    if (require_alignment_gate_ && !lio_world_input_) {
        alignment_start_time_ = this->now();
        alignment_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100), std::bind(&VoxMapManagerNode::alignmentTick, this),
            compute_cb_group_);
    } else {
        // Gate disabled or lio_world mode
        alignment_captured_.store(true, std::memory_order_release);
    }

    // Distance-based eviction sweeps voxels beyond EVICT_RADIUS around drone_pos_
    if (!lio_world_input_) {
        eviction_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(px4_common::mapping::kEvictIntervalMs),
            [this]() {
                Eigen::Vector3d pos;
                {
                    std::lock_guard<std::mutex> lock(odom_mutex_);
                    pos = drone_pos_;
                }
                size_t evicted = voxel_map_.EvictDistant(pos);
                if (evicted > 0)
                    RCLCPP_DEBUG(this->get_logger(), "Evicted %zu voxels (>%.0fm)", evicted,
                                 px4_common::mapping::kEvictRadiusM);
            },
            compute_cb_group_);
    }

    RCLCPP_INFO(
        this->get_logger(), "Voxel Map Manager (Layer 2) initialized. Eviction: %s",
        lio_world_input_ ? "age-only (lio_world global map)" : "distance + age (local map)");

    if (lio_world_input_) {
        RCLCPP_WARN(this->get_logger(),
                    "lio_world mode: distance eviction disabled; voxel count bounded "
                    "only by kMaxVoxels=%zu and kMaxFrameAge=%d. Dev/debug only — "
                    "use input_source=px4_full for production flights.",
                    static_cast<size_t>(px4_common::mapping::kMaxVoxels),
                    px4_common::mapping::kMaxFrameAge);
    }

    RCLCPP_INFO(this->get_logger(), "VoxMapManagerNode initialized");
}

VoxMapManagerNode::~VoxMapManagerNode() {
    writeSummary();
    saveGlobalMap();
    printf("\n[VoxMap] Node shutting down\n");
}

// === IVoxMapManager interface ===
double VoxMapManagerNode::GetResolution() const {
    return voxel_map_.GetResolution();
}

void VoxMapManagerNode::GetOccupiedPointsInRadius(const Eigen::Vector3d& center, double radius,
                                                  std::vector<Eigen::Vector3d>& out) {
    voxel_map_.GetOccupiedPointsInRadius(center, radius, out);
}

bool VoxMapManagerNode::IsReady() const noexcept {
    // Require both map coverage and sustained pose-source alignment
    return ready_.load(std::memory_order_acquire) &&
           alignment_captured_.load(std::memory_order_acquire);
}

std::uint64_t VoxMapManagerNode::FramesDropped() const noexcept {
    return frames_dropped_.load(std::memory_order_relaxed);
}

Eigen::Vector3d VoxMapManagerNode::GetExtrinsicTranslation() const noexcept {
    return T_lidar_in_imu_;
}

/* =====================================================================
    saveGlobalMap, dump entire occupied map to PCD and PLY on shutdown
   ===================================================================== */
void VoxMapManagerNode::saveGlobalMap() {
    // Simplified implementation without PCL
    printf("\n[VoxMap] Global map saving not implemented (PCL not available)\n");
}

/* =====================================================================
    initLogging, open CSV timing log file in log_path_
   ===================================================================== */
/* Helper: create directory tree recursively (like mkdir -p) */
void VoxMapManagerNode::mkdirRecursive(const std::string& path) {
    std::string acc;
    for (size_t i = 0; i < path.size(); ++i) {
        acc += path[i];
        if (path[i] == '/' || i == path.size() - 1) {
            mkdir(acc.c_str(), 0755);
        }
    }
}

void VoxMapManagerNode::initLogging() {
    if (log_path_.empty()) {
        log_path_ = "./src/Log/mapping";
    }

    mkdirRecursive(log_path_);

    auto now = std::chrono::system_clock::now();
    auto time_t = std::chrono::system_clock::to_time_t(now);
    std::tm tm_buf{};
    localtime_r(&time_t, &tm_buf);

    std::ostringstream oss;
    oss << log_path_ << "/voxmap_" << std::put_time(&tm_buf, "%Y%m%d_%H%M%S") << ".log";
    // Not storing log_filename_ as member since we're not using it in this implementation

    // For simplicity, we're not implementing full logging as in the original
    // In a real implementation, you would open and manage the log file here
}

/* =====================================================================
    writeSummary, append aggregate timing stats to log file on shutdown
   ===================================================================== */
void VoxMapManagerNode::writeSummary() {
    // Simplified summary - in a real implementation you would write to a log file
    printf("\n============ VOXMAP SUMMARY ============\n");
    printf("Total Frames:      %lu\n", frame_count_);
    printf("Frames Dropped:    %lu\n", frames_dropped_.load(std::memory_order_relaxed));
    printf("Final Ready State: %s\n", ready_.load(std::memory_order_relaxed) ? "true" : "false");
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
void VoxMapManagerNode::timeoutCallback() {
    double elapsed = (this->now() - last_data_time_).seconds();

    if (elapsed > timeout_seconds_ && !map_cleared_) {
        printf("[VoxMap] TIMEOUT: Clearing map after %.1fs\n", elapsed);
        fflush(stdout);

        voxel_map_.Clear();

        // Publish empty cloud to clear RViz display
        sensor_msgs::msg::PointCloud2 empty_msg;
        empty_msg.header.frame_id = "map_ned";
        empty_msg.header.stamp = this->now();
        empty_msg.height = 0;
        empty_msg.width = 0;
        empty_msg.is_dense = true;
        empty_msg.is_bigendian = false;
        empty_msg.point_step = 16;  // x(4) + y(4) + z(4) + intensity(4)
        empty_msg.row_step = 0;
        pub_map_->publish(empty_msg);

        map_cleared_ = true;
        ready_.store(false, std::memory_order_release);
        ready_consecutive_frames_ = 0;
    }
}

/* =====================================================================
    alignmentTick, 10 Hz check of the alignment gate conditions
   ===================================================================== */
void VoxMapManagerNode::alignmentTick() {
    if (alignment_captured_.load(std::memory_order_acquire)) {
        return;
    }

    const bool c1 = armed_.load(std::memory_order_acquire);
    const bool c2 = drone_speed_.load(std::memory_order_acquire) < aligned_max_velocity_;
    const bool c3 = ekf_pose_valid_.load(std::memory_order_acquire);
    const bool c4 =
        lio_covariance_trace_.load(std::memory_order_acquire) < aligned_lio_covariance_max_;

    const auto now = this->now();
    if (c1 && c2 && c3 && c4) {
        if (aligned_streak_start_.nanoseconds() == 0) {
            aligned_streak_start_ = now;
        }
        const double streak_s = (now - aligned_streak_start_).seconds();
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
    const double since_start = (now - alignment_start_time_).seconds();
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
    lioOdomCallback, always-on LIO pose handler
   ===================================================================== */
void VoxMapManagerNode::lioOdomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    if (lio_samples_received_ == 0) {
        lio_first_sample_time_ = this->now();
    }
    lio_samples_received_++;

    // Wall-clock contamination guard
    if (this->get_parameter("use_sim_time").as_bool() && lio_samples_received_ > 20) {
        const rclcpp::Time stamp(msg->header.stamp);
        const rclcpp::Time clock_now = this->now();
        const double age_ms = (clock_now - stamp).seconds() * 1000.0;
        if (std::abs(age_ms) > 5000.0) {
            RCLCPP_FATAL(this->get_logger(),
                         "LIO /odometry stamp domain mismatch, stamp=%d.%09u, "
                         "clock_now=%.3f, age=%.1fms, likely wall-clock contamination "
                         "under use_sim_time=true",
                         msg->header.stamp.sec, msg->header.stamp.nanosec, clock_now.seconds(),
                         age_ms);
            rclcpp::shutdown();
            return;
        }
    }

    // Build the timestamped sample
    px4_common::time::PoseSample sample;
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
bool VoxMapManagerNode::hasField(const sensor_msgs::msg::PointCloud2& msg,
                                 const std::string& name) {
    for (const auto& field : msg.fields) {
        if (field.name == name)
            return true;
    }
    return false;
}

/* =====================================================================
    cloudCallback, main raycasting pipeline
   ===================================================================== */
void VoxMapManagerNode::cloudCallback(sensor_msgs::msg::PointCloud2::UniquePtr msg) {
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

        px4_common::PointLivox pt;
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
    px4_common::time::PoseSample px4_at_cloud, lio_at_cloud;
    bool chain_ok = false;
    if (use_lio_buffer_ && !lio_world_input_) {
        const bool px4_hit = px4_buf_.Lookup(cloud_t_ns, px4_at_cloud);
        const bool lio_hit = lio_buf_.Lookup(cloud_t_ns, lio_at_cloud);
        chain_ok = px4_hit && lio_hit;
    }

    Eigen::Vector3d sensor_world;
    if (chain_ok) {
        // Path B Option gamma, scan-time pose chain
        const Eigen::Vector3d t_ned = px4_at_cloud.orientation.toRotationMatrix() * C_FRD_FLU_ *
                                      lio_at_cloud.orientation.toRotationMatrix() * T_lidar_in_imu_;
        sensor_world = px4_at_cloud.position + t_ned;
    } else if (lio_world_input_) {
        // lio_world mode
        const Eigen::Vector3d t_world =
            have_q ? (q_ned_frd.toRotationMatrix() * T_lidar_in_imu_) : T_lidar_in_imu_;
        sensor_world = pos + t_world;
    } else if (full_pose_input_ && have_q) {
        // px4_full fallback
        const Eigen::Vector3d t_ned = q_ned_frd.toRotationMatrix() * C_FRD_FLU_ * T_lidar_in_imu_;
        sensor_world = pos + t_ned;
    } else {
        // Yaw-only chain
        const double bx = T_lidar_in_imu_.x();
        const double by = -T_lidar_in_imu_.y();
        const double bz = -T_lidar_in_imu_.z();
        sensor_world =
            Eigen::Vector3d(pos.x() + cy * bx - sy * by, pos.y() + sy * bx + cy * by, pos.z() + bz);
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
        const Eigen::Matrix3d R_ned_flu = q_ned_frd.toRotationMatrix() * C_FRD_FLU_;
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

    // Coverage-based readiness gate
    if (!ready_.load(std::memory_order_relaxed)) {
        const size_t occ = voxel_map_.OccupiedCount();
        if (occ >= static_cast<size_t>(ready_min_occupied_)) {
            ready_consecutive_frames_++;
            if (ready_consecutive_frames_ >= ready_min_frames_) {
                ready_.store(true, std::memory_order_release);
                RCLCPP_INFO(this->get_logger(), "VoxMap ready, %d frames with %zu occupied voxels",
                            ready_consecutive_frames_, occ);
            }
        } else {
            ready_consecutive_frames_ = 0;
        }
    }

    // Publish changed voxels
    if (publish_local_map_) {
        voxel_map_.GetChangedPoints(changed_buf_);
        if (!changed_buf_.empty()) {
            // Create a simple point structure for output
            struct PointXYZI {
                float x, y, z, intensity;
            };
            std::vector<PointXYZI> pcl_out;
            pcl_out.reserve(changed_buf_.size());

            for (const auto& p : changed_buf_) {
                PointXYZI pt;
                pt.x = static_cast<float>(p.x);
                pt.y = static_cast<float>(p.y);
                pt.z = static_cast<float>(p.z);
                pt.intensity = p.intensity;
                pcl_out.push_back(pt);
            }

            // Create PointCloud2 message manually
            sensor_msgs::msg::PointCloud2 ros_msg;
            ros_msg.header.stamp = msg->header.stamp;
            ros_msg.header.frame_id = lio_world_input_ ? "camera_init" : "map_ned";
            ros_msg.height = 1;
            ros_msg.width = pcl_out.size();
            ros_msg.is_dense = true;
            ros_msg.is_bigendian = false;

            // Define fields
            sensor_msgs::msg::PointField field_x, field_y, field_z, field_intensity;
            field_x.name = "x";
            field_x.offset = 0;
            field_x.datatype = sensor_msgs::msg::PointField::FLOAT32;
            field_x.count = 1;
            field_y.name = "y";
            field_y.offset = 4;
            field_y.datatype = sensor_msgs::msg::PointField::FLOAT32;
            field_y.count = 1;
            field_z.name = "z";
            field_z.offset = 8;
            field_z.datatype = sensor_msgs::msg::PointField::FLOAT32;
            field_z.count = 1;
            field_intensity.name = "intensity";
            field_intensity.offset = 12;
            field_intensity.datatype = sensor_msgs::msg::PointField::FLOAT32;
            field_intensity.count = 1;

            ros_msg.fields = {field_x, field_y, field_z, field_intensity};
            ros_msg.point_step = 16;  // 4 floats
            ros_msg.row_step = ros_msg.point_step * ros_msg.width;

            // Fill data
            ros_msg.data.resize(ros_msg.row_step);
            uint8_t* data_ptr = ros_msg.data.data();
            for (size_t i = 0; i < pcl_out.size(); ++i) {
                float* point_data = reinterpret_cast<float*>(data_ptr + i * 16);
                point_data[0] = pcl_out[i].x;
                point_data[1] = pcl_out[i].y;
                point_data[2] = pcl_out[i].z;
                point_data[3] = pcl_out[i].intensity;
            }

            pub_map_->publish(ros_msg);
        }
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
std::shared_ptr<rclcpp::Node> get_voxmap_node(
    const rclcpp::NodeOptions& options,
    std::shared_ptr<px4_common::mapping::IVoxMapManager>& out_iface) {
    auto mgr = std::make_shared<VoxMapManagerNode>(options);
    out_iface = mgr;
    return mgr;
}

}  // namespace px4_mapping
