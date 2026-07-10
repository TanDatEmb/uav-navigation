// Copyright 2026 CTUAV. All rights reserved.
//
// Implementation of NED transform node for converting FAST-LIO2 point clouds
// from camera_init frame to map_ned frame for PX4 integration.

#include "px4_mapping/ned_transform_node.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <nav_msgs/msg/odometry.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <rcl_interfaces/msg/parameter_descriptor.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

#include <px4_common/frame_constants.hpp>
#include <px4_common/math/transforms.hpp>
#include <px4_common/time/pose_buffer.hpp>
#include <px4_ros_com/time_sync.hpp>
#include <px4_ros_com/topic_helpers.hpp>

namespace px4_mapping {

namespace {

Eigen::Matrix3d LioWorldToNedMatrix() {
    return (Eigen::Matrix3d() <<
            0.0, 1.0, 0.0,
            1.0, 0.0, 0.0,
            0.0, 0.0, -1.0)
        .finished();
}

Eigen::Vector3d LioPositionToNed(const Eigen::Vector3d& p_lio) {
    return LioWorldToNedMatrix() * p_lio;
}

Eigen::Quaterniond LioOrientationToNed(const Eigen::Quaterniond& q_lio) {
    static const Eigen::Matrix3d kCFrdFlu =
        (Eigen::Matrix3d() << 1.0, 0.0, 0.0, 0.0, -1.0, 0.0, 0.0, 0.0, -1.0).finished();

    const Eigen::Matrix3d r_ned_frd =
        LioWorldToNedMatrix() * q_lio.normalized().toRotationMatrix() * kCFrdFlu;
    return Eigen::Quaterniond(r_ned_frd).normalized();
}

bool IsValidAlignmentMode(const std::string& mode) {
    return mode == "translation_only" || mode == "yaw_translation" || mode == "full_6dof";
}

}  // namespace

NedTransformNode::NedTransformNode(const rclcpp::NodeOptions& options)
    : rclcpp::Node("world_bridge_node", options),
      lio_pose_buffer_(500, std::chrono::nanoseconds(5'000'000'000LL)),
      px4_pose_buffer_(500, std::chrono::nanoseconds(5'000'000'000LL)),
      visual_align_translation_(Eigen::Vector3d::Zero()),
      visual_align_rotation_(Eigen::Quaterniond::Identity()),
      visual_alignment_ready_(false),
      frame_count_(0) {
    // Load parameters
    LoadParameters();

    // Create callback groups
    io_callback_group_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    compute_callback_group_ =
        this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

    // Subscription options
    rclcpp::SubscriptionOptions io_subscription_options;
    io_subscription_options.callback_group = io_callback_group_;

    rclcpp::SubscriptionOptions compute_subscription_options;
    compute_subscription_options.callback_group = compute_callback_group_;

    // Create subscriptions
    // === PX4 odometry (NED, 50-100 Hz) ===
    auto px4_qos = rclcpp::QoS(20).best_effort();
    sub_px4_odom_ = this->create_subscription<px4_msgs::msg::VehicleOdometry>(
        px4_odom_topic_, px4_qos,
        [this](const px4_msgs::msg::VehicleOdometry::SharedPtr msg) {
            this->Px4OdomCallback(msg);
        },
        io_subscription_options);

    // === L1 odometry (camera_init, 10 Hz) ===
    sub_lio_odom_ = this->create_subscription<nav_msgs::msg::Odometry>(
        lio_odom_topic_, rclcpp::QoS(20),
        [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
            this->LioOdomCallback(msg);
        },
        io_subscription_options);

    // === L1 world-frame cloud (camera_init, 10 Hz) ===
    auto cloud_qos = rclcpp::SensorDataQoS();
    cloud_qos.keep_last(50);
    sub_livox_cloud_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        input_cloud_topic_, cloud_qos,
        [this](const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
            this->LivoxCloudCallback(msg);
        },
        compute_subscription_options);

    // Create publishers.
    // /livox/world/cloud is an internal pipeline topic consumed by the
    // voxmap_manager and recorded by rosbag2 (which subscribes with the
    // default reliable QoS). Use reliable to stay consistent with the
    // fast_lio2 publisher pattern and avoid QoS incompatibility.
    const auto ned_cloud_qos = rclcpp::QoS(20).reliable();
    pub_livox_ned_ =
        this->create_publisher<sensor_msgs::msg::PointCloud2>(output_cloud_topic_, ned_cloud_qos);

    if (publish_visual_odometry_to_px4_) {
        // /fmu/in/* topics are best_effort in PX4 (per px4_ros_com convention).
        pub_visual_odom_ =
            this->create_publisher<px4_msgs::msg::VehicleOdometry>(visual_odom_topic_, px4_qos);
    }

    RCLCPP_INFO(this->get_logger(), "NED Transform Node initialized: %s + %s + %s → %s",
                input_cloud_topic_.c_str(), lio_odom_topic_.c_str(), px4_odom_topic_.c_str(),
                output_cloud_topic_.c_str());

    if (publish_visual_odometry_to_px4_) {
        RCLCPP_INFO(this->get_logger(), "[NED] FAST-LIO2 external vision publisher enabled: %s",
                    visual_odom_topic_.c_str());
    }
}

void NedTransformNode::LoadParameters() {
    rcl_interfaces::msg::ParameterDescriptor use_px4_odom_desc;
    use_px4_odom_desc.read_only = true;
    use_px4_odom_desc.description =
        "Require PX4 odometry for camera_init->map_ned SE(3) composition.";
    use_px4_odom_desc.additional_constraints =
        "Must be true. Legacy static shortcut was removed.";

    rcl_interfaces::msg::ParameterDescriptor alignment_mode_desc;
    alignment_mode_desc.description =
        "Alignment mode for EV publishing when visual_odom_align_to_px4=true.";
    alignment_mode_desc.additional_constraints =
        "One of: translation_only, yaw_translation, full_6dof.";

    // Declare and load parameters
    this->declare_parameter<std::string>("input_cloud_topic", "/livox/l1/cloud");
    this->declare_parameter<std::string>("output_cloud_topic", "/livox/world/cloud");
    this->declare_parameter<std::string>("lio_odom_topic", "/livox/l1/odometry");
    this->declare_parameter<std::string>("px4_odom_topic", "/fmu/out/vehicle_odometry");
    this->declare_parameter<bool>("use_px4_odom", true, use_px4_odom_desc);
    this->declare_parameter<bool>("publish_visual_odometry_to_px4", false);
    this->declare_parameter<std::string>("visual_odom_topic", "/fmu/in/vehicle_visual_odometry");
    this->declare_parameter<bool>("visual_odom_align_to_px4", true);
    this->declare_parameter<std::string>("visual_odom_alignment_mode", "translation_only",
                                         alignment_mode_desc);
    this->declare_parameter<bool>("visual_odom_align_full_6dof", false);
    this->declare_parameter<std::vector<double>>("visual_odom_position_variance",
                                                 {0.04, 0.04, 0.09});
    this->declare_parameter<std::vector<double>>("visual_odom_orientation_variance",
                                                 {0.25, 0.25, 0.05});
    this->declare_parameter<int>("visual_odom_quality", 100);

    input_cloud_topic_ = this->get_parameter("input_cloud_topic").as_string();
    output_cloud_topic_ = this->get_parameter("output_cloud_topic").as_string();
    lio_odom_topic_ = this->get_parameter("lio_odom_topic").as_string();
    px4_odom_topic_ = this->get_parameter("px4_odom_topic").as_string();
    use_px4_odom_ = this->get_parameter("use_px4_odom").as_bool();
    if (!use_px4_odom_) {
        RCLCPP_FATAL(this->get_logger(),
                     "use_px4_odom=false is not supported. The static (x,y,z)->(y,x,-z) "
                     "shortcut assumes camera_init is ENU and ignores the SE(3) alignment "
                     "between LIO world and PX4 NED. Provide PX4 odometry or implement a "
                     "calibrated T_map_ned_camera_init transform instead.");
        throw std::runtime_error("use_px4_odom=false is not supported");
    }
    publish_visual_odometry_to_px4_ =
        this->get_parameter("publish_visual_odometry_to_px4").as_bool();
    visual_odom_topic_ = this->get_parameter("visual_odom_topic").as_string();
    visual_odom_align_to_px4_ = this->get_parameter("visual_odom_align_to_px4").as_bool();
    visual_odom_alignment_mode_ = this->get_parameter("visual_odom_alignment_mode").as_string();
    visual_odom_align_full_6dof_ = this->get_parameter("visual_odom_align_full_6dof").as_bool();

    if (!IsValidAlignmentMode(visual_odom_alignment_mode_)) {
        RCLCPP_WARN(this->get_logger(),
                    "Invalid visual_odom_alignment_mode='%s', fallback to translation_only",
                    visual_odom_alignment_mode_.c_str());
        visual_odom_alignment_mode_ = "translation_only";
    }
    if (visual_odom_align_full_6dof_ && visual_odom_alignment_mode_ != "full_6dof") {
        RCLCPP_WARN(this->get_logger(),
                    "visual_odom_align_full_6dof=true overrides visual_odom_alignment_mode='%s'",
                    visual_odom_alignment_mode_.c_str());
        visual_odom_alignment_mode_ = "full_6dof";
    }

    // Load variance parameters
    auto position_variance = this->get_parameter("visual_odom_position_variance").as_double_array();
    auto orientation_variance =
        this->get_parameter("visual_odom_orientation_variance").as_double_array();

    for (size_t i = 0; i < 3 && i < position_variance.size(); ++i) {
        visual_odom_position_variance_[i] = std::max(position_variance[i], 1e-6);
    }

    for (size_t i = 0; i < 3 && i < orientation_variance.size(); ++i) {
        visual_odom_orientation_variance_[i] = std::max(orientation_variance[i], 1e-6);
    }

    visual_odom_quality_ =
        static_cast<int>(std::clamp(this->get_parameter("visual_odom_quality").as_int(),
                                    static_cast<int64_t>(-1), static_cast<int64_t>(100)));
}

void NedTransformNode::LivoxCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
    const size_t n = msg->width * msg->height;
    if (n == 0) {
        return;
    }

    const auto t_start = std::chrono::steady_clock::now();

    // Transform the point cloud
    sensor_msgs::msg::PointCloud2 output_cloud;
    if (!TransformPointCloud(msg, output_cloud)) {
        return;
    }

    const auto t_after_transform = std::chrono::steady_clock::now();

    // Publish the transformed cloud
    pub_livox_ned_->publish(output_cloud);

    const auto t_after_publish = std::chrono::steady_clock::now();

    // Log stage timings every 100 frames
    frame_count_++;
    if (frame_count_ % 100 == 0) {
        using ms_t = std::chrono::duration<double, std::milli>;
        double transform_ms = ms_t(t_after_transform - t_start).count();
        double publish_ms = ms_t(t_after_publish - t_after_transform).count();
        RCLCPP_INFO(this->get_logger(), "[NED] F%lu: %zu pts | tf %.2fms pub %.2fms", frame_count_,
                    n, transform_ms, publish_ms);
    }
}

void NedTransformNode::LioOdomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    // Convert timestamp to nanoseconds
    const int64_t t_ns = static_cast<int64_t>(msg->header.stamp.sec) * 1'000'000'000LL +
                         static_cast<int64_t>(msg->header.stamp.nanosec);

    // Create pose sample
    px4_common::time::PoseSample sample;
    sample.t_ns = t_ns;
    sample.position = Eigen::Vector3d(msg->pose.pose.position.x, msg->pose.pose.position.y,
                                      msg->pose.pose.position.z);
    sample.orientation =
        Eigen::Quaterniond(msg->pose.pose.orientation.w, msg->pose.pose.orientation.x,
                           msg->pose.pose.orientation.y, msg->pose.pose.orientation.z);

    // Publish visual odometry if enabled
    PublishVisualOdometry(sample);

    // Add to buffer
    lio_pose_buffer_.Push(sample);
}

void NedTransformNode::Px4OdomCallback(const px4_msgs::msg::VehicleOdometry::SharedPtr msg) {
    // MicroXRCE-DDS time sync is the single boundary policy in this repo:
    // timestamp_sample is already in ROS clock domain (microseconds).
    const int64_t now_ns = this->now().nanoseconds();
    const int64_t t_ns = px4_timestamp_adapter_.ToRosNanoseconds(msg->timestamp_sample, now_ns);

    if (now_ns > 1'000'000'000LL &&
        !px4_ros_com::time::IsWithinTimestampDomainGuard(t_ns, now_ns)) {
        RCLCPP_ERROR_THROTTLE(
            this->get_logger(), *this->get_clock(), 5000,
            "[NED] PX4 /fmu/out/vehicle_odometry timestamp_sample is outside ROS domain "
            "guard (stamp_ns=%ld, now_ns=%ld). Verify UXRCE_DDS_SYNCT and /clock setup.",
            t_ns, now_ns);
        return;
    }

    // Create pose sample
    px4_common::time::PoseSample sample;
    sample.t_ns = t_ns;
    sample.position = Eigen::Vector3d(static_cast<double>(msg->position[0]),
                                      static_cast<double>(msg->position[1]),
                                      static_cast<double>(msg->position[2]));
    sample.orientation =
        Eigen::Quaterniond(static_cast<double>(msg->q[0]), static_cast<double>(msg->q[1]),
                           static_cast<double>(msg->q[2]), static_cast<double>(msg->q[3]))
            .normalized();

    // Add to buffer
    px4_pose_buffer_.Push(sample);
}

bool NedTransformNode::TransformPointCloud(
    const sensor_msgs::msg::PointCloud2::SharedPtr& input_cloud,
    sensor_msgs::msg::PointCloud2& output_cloud) {
    const size_t n = input_cloud->width * input_cloud->height;
    if (n == 0) {
        return false;
    }

    // Convert timestamp to nanoseconds
    const int64_t scan_t_ns =
        static_cast<int64_t>(input_cloud->header.stamp.sec) * 1'000'000'000LL +
        static_cast<int64_t>(input_cloud->header.stamp.nanosec);

    // Field offsets from input message
    uint32_t off_x = 0, off_y = 4, off_z = 8, off_i = 16;
    bool has_intensity = false;
    for (const auto& f : input_cloud->fields) {
        if (f.name == "x")
            off_x = f.offset;
        else if (f.name == "y")
            off_y = f.offset;
        else if (f.name == "z")
            off_z = f.offset;
        else if (f.name == "intensity") {
            off_i = f.offset;
            has_intensity = true;
        }
    }

    // Build output cloud
    output_cloud.header = input_cloud->header;
    output_cloud.header.frame_id = std::string(px4_common::frame::kMapNed);
    output_cloud.height = 1;
    output_cloud.width = static_cast<uint32_t>(n);

    sensor_msgs::PointCloud2Modifier mod(output_cloud);
    mod.setPointCloud2Fields(4, "x", 1, sensor_msgs::msg::PointField::FLOAT32, "y", 1,
                             sensor_msgs::msg::PointField::FLOAT32, "z", 1,
                             sensor_msgs::msg::PointField::FLOAT32, "intensity", 1,
                             sensor_msgs::msg::PointField::FLOAT32);
    mod.resize(n);

    sensor_msgs::PointCloud2Iterator<float> it_ox(output_cloud, "x");
    sensor_msgs::PointCloud2Iterator<float> it_oy(output_cloud, "y");
    sensor_msgs::PointCloud2Iterator<float> it_oz(output_cloud, "z");
    sensor_msgs::PointCloud2Iterator<float> it_oi(output_cloud, "intensity");

    const uint8_t* in_data = input_cloud->data.data();
    const uint32_t step = input_cloud->point_step;

    // use_px4_odom_ must be true (enforced in LoadParameters). We need PX4
    // odometry to know the SE(3) pose of the baselink in map_ned so that the
    // camera_init -> map_ned transform can be composed per-scan.
    px4_common::time::PoseSample lio_sample, px4_sample;
    if (!lio_pose_buffer_.Lookup(scan_t_ns, lio_sample)) {
        ++lio_pose_lookup_miss_;
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                             "LIO pose interpolation failed for scan at time %ld ns "
                             "(miss=%lu)",
                             scan_t_ns,
                             static_cast<unsigned long>(lio_pose_lookup_miss_));
        return false;
    }

    if (!px4_pose_buffer_.Lookup(scan_t_ns, px4_sample)) {
        ++px4_pose_lookup_miss_;
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                             "PX4 pose interpolation failed for scan at time %ld ns "
                             "(miss=%lu)",
                             scan_t_ns,
                             static_cast<unsigned long>(px4_pose_lookup_miss_));
        return false;
    }

    // Compose the combined transform.
    //
    // Coordinate frames:
    //   camera_init : FAST-LIO2 world frame at power-on (ENU-like, ROS convention).
    //   body_lio      : IMU/sensor body frame used by FAST-LIO2 (Forward-Left-Up).
    //   body_px4      : PX4 body frame (Forward-Right-Down).
    //   map_ned       : PX4 local world frame (North-East-Down).
    //
    // Inputs:
    //   p_camera_init : point from /livox/l1/cloud (already in camera_init world).
    //   T_camera_init_body_lio = (R_lio, t_lio) from FAST-LIO2 odometry.
    //   T_map_ned_body_px4     = (R_px4, t_px4) from PX4 VehicleOdometry.
    //
    // Chain:
    //   p_body_lio  = R_lio^T * (p_camera_init - t_lio)
    //   p_body_px4  = C_FRD_FLU * p_body_lio       (FLU -> FRD reflection)
    //   p_map_ned   = R_px4 * p_body_px4 + t_px4
    //
    // The FRD_FLU reflection comes from Gazebo/sim convention: the processed
    // Livox cloud reaches this node in a ROS body-FLU-like representation, while
    // PX4 expects body-FRD. If the upstream cloud frame ever changes, this
    // constant must be revisited.
    static const Eigen::Matrix3d C_FRD_FLU =
        (Eigen::Matrix3d() << 1.0, 0.0, 0.0, 0.0, -1.0, 0.0, 0.0, 0.0, -1.0).finished();

    const Eigen::Matrix3d R_lio_T = lio_sample.orientation.toRotationMatrix().transpose();
    const Eigen::Matrix3d R_total =
        px4_sample.orientation.toRotationMatrix() * C_FRD_FLU * R_lio_T;

    for (size_t i = 0; i < n; ++i, ++it_ox, ++it_oy, ++it_oz, ++it_oi) {
        const uint8_t* p = in_data + i * step;
        float x = *reinterpret_cast<const float*>(p + off_x);
        float y = *reinterpret_cast<const float*>(p + off_y);
        float z = *reinterpret_cast<const float*>(p + off_z);

        Eigen::Vector3d p_ci(x, y, z);
        Eigen::Vector3d p_rel = p_ci - lio_sample.position;
        Eigen::Vector3d p_ned = R_total * p_rel + px4_sample.position;

        *it_ox = static_cast<float>(p_ned.x());
        *it_oy = static_cast<float>(p_ned.y());
        *it_oz = static_cast<float>(p_ned.z());
        *it_oi = has_intensity ? *reinterpret_cast<const float*>(p + off_i) : 0.0f;
    }

    return true;
}

void NedTransformNode::PublishVisualOdometry(const px4_common::time::PoseSample& lio_sample) {
    if (!publish_visual_odometry_to_px4_ || !pub_visual_odom_) {
        return;
    }

    std::lock_guard<std::mutex> lock(visual_odom_mutex_);

    // Convert LIO pose from camera_init world + FLU body convention into
    // PX4 map_ned + FRD body convention.
    px4_common::time::PoseSample lio_in_ned;
    lio_in_ned.t_ns = lio_sample.t_ns;
    lio_in_ned.position = LioPositionToNed(lio_sample.position);
    lio_in_ned.orientation = LioOrientationToNed(lio_sample.orientation);

    if (visual_odom_align_to_px4_ && !visual_alignment_ready_) {
        // Get interpolated PX4 pose for alignment
        px4_common::time::PoseSample px4_sample;
        if (!px4_pose_buffer_.Lookup(lio_in_ned.t_ns, px4_sample)) {
            return;
        }

        if (visual_odom_alignment_mode_ == "full_6dof") {
            // Full SE(3) alignment: T_map_ned_camera_init = T_map_ned_body *
            // T_body_camera_init. T_body_camera_init is the inverse of the LIO
            // pose expressed in NED (p_raw/q_raw).
            visual_align_rotation_ = px4_sample.orientation *
                                     lio_in_ned.orientation.inverse();
            visual_align_translation_ =
                px4_sample.position - visual_align_rotation_ * lio_in_ned.position;
        } else if (visual_odom_alignment_mode_ == "yaw_translation") {
            // Yaw + translation alignment for world frames where roll/pitch are
            // already gravity-aligned but heading differs.
            const double yaw_delta =
                px4_common::math::QuaternionGetYaw(px4_sample.orientation) -
                px4_common::math::QuaternionGetYaw(lio_in_ned.orientation);

            visual_align_rotation_ =
                Eigen::Quaterniond(Eigen::AngleAxisd(yaw_delta, Eigen::Vector3d::UnitZ()));
            visual_align_translation_ =
                px4_sample.position - visual_align_rotation_ * lio_in_ned.position;
        } else {
            // Translation-only alignment for stacks where the LIO world already
            // matches PX4 axes but uses a local origin.
            visual_align_rotation_ = Eigen::Quaterniond::Identity();
            visual_align_translation_ = px4_sample.position - lio_in_ned.position;
        }

        visual_alignment_ready_ = true;

        RCLCPP_INFO(this->get_logger(),
                    "[NED] FAST-LIO2 EV alignment captured (%s): offset=(%.3f, %.3f, %.3f)",
                    visual_odom_alignment_mode_.c_str(),
                    visual_align_translation_.x(), visual_align_translation_.y(),
                    visual_align_translation_.z());
    }

    if (visual_odom_align_to_px4_ && !visual_alignment_ready_) {
        return;
    }

    // Apply alignment if needed
    const Eigen::Vector3d p_ev = visual_alignment_ready_
                                     ? visual_align_rotation_ * lio_in_ned.position +
                                           visual_align_translation_
                                     : lio_in_ned.position;
    const Eigen::Quaterniond q_ev =
        (visual_alignment_ready_ ? visual_align_rotation_ * lio_in_ned.orientation
                                 : lio_in_ned.orientation)
            .normalized();

    // Publish timestamps directly from ROS clock domain. PX4 boundary sync is
    // handled by MicroXRCE-DDS according to the repo timestamp policy.
    const uint64_t sample_px4_us = px4_ros_com::time::RosNanosecondsToPx4Microseconds(
        lio_sample.t_ns);
    const uint64_t now_px4_us = px4_ros_com::time::RosTimeToPx4Microseconds(this->now());
    if (sample_px4_us == 0 || now_px4_us == 0) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                             "[NED] Skip EV publish due to invalid ROS timestamp");
        return;
    }

    // Create and populate the message
    px4_msgs::msg::VehicleOdometry ev_msg;
    ev_msg.timestamp = now_px4_us;
    ev_msg.timestamp_sample = sample_px4_us;
    ev_msg.pose_frame = px4_msgs::msg::VehicleOdometry::POSE_FRAME_NED;
    ev_msg.position[0] = static_cast<float>(p_ev.x());
    ev_msg.position[1] = static_cast<float>(p_ev.y());
    ev_msg.position[2] = static_cast<float>(p_ev.z());
    ev_msg.q[0] = static_cast<float>(q_ev.w());
    ev_msg.q[1] = static_cast<float>(q_ev.x());
    ev_msg.q[2] = static_cast<float>(q_ev.y());
    ev_msg.q[3] = static_cast<float>(q_ev.z());

    ev_msg.velocity_frame = px4_msgs::msg::VehicleOdometry::VELOCITY_FRAME_UNKNOWN;

    const float nan = std::numeric_limits<float>::quiet_NaN();
    for (int i = 0; i < 3; ++i) {
        ev_msg.velocity[i] = nan;
        ev_msg.angular_velocity[i] = nan;
        ev_msg.velocity_variance[i] = nan;
    }

    for (int i = 0; i < 3; ++i) {
        ev_msg.position_variance[i] = static_cast<float>(visual_odom_position_variance_[i]);
        ev_msg.orientation_variance[i] = static_cast<float>(visual_odom_orientation_variance_[i]);
    }

    ev_msg.reset_counter = 0;
    ev_msg.quality = static_cast<int8_t>(visual_odom_quality_);

    pub_visual_odom_->publish(ev_msg);
}

}  // namespace px4_mapping