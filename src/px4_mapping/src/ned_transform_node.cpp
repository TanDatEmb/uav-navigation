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
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

#include <px4_common/frame_constants.hpp>
#include <px4_common/math/transforms.hpp>
#include <px4_common/time/pose_buffer.hpp>
#include <px4_ros_com/frame_transforms.hpp>
#include <px4_ros_com/topic_helpers.hpp>

namespace px4_mapping {

NedTransformNode::NedTransformNode(const rclcpp::NodeOptions& options)
    : rclcpp::Node("ned_transform_node", options),
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

    // Create publishers
    pub_livox_ned_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(output_cloud_topic_, 20);

    if (publish_visual_odometry_to_px4_) {
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
    // Declare and load parameters
    this->declare_parameter<std::string>("input_cloud_topic", "/livox_processed");
    this->declare_parameter<std::string>("output_cloud_topic", "/livox_processed_ned");
    this->declare_parameter<std::string>("lio_odom_topic", "/odometry");
    this->declare_parameter<std::string>("px4_odom_topic", "/fmu/out/vehicle_odometry");
    this->declare_parameter<bool>("use_px4_odom", true);
    this->declare_parameter<bool>("publish_visual_odometry_to_px4", false);
    this->declare_parameter<std::string>("visual_odom_topic", "/fmu/in/vehicle_visual_odometry");
    this->declare_parameter<bool>("visual_odom_align_to_px4", true);
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
    publish_visual_odometry_to_px4_ =
        this->get_parameter("publish_visual_odometry_to_px4").as_bool();
    visual_odom_topic_ = this->get_parameter("visual_odom_topic").as_string();
    visual_odom_align_to_px4_ = this->get_parameter("visual_odom_align_to_px4").as_bool();

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
    // Convert timestamp to nanoseconds
    const int64_t t_ns = static_cast<int64_t>(msg->timestamp_sample);

    // Create pose sample
    px4_common::time::PoseSample sample;
    sample.t_ns = t_ns;
    sample.position = Eigen::Vector3d(static_cast<double>(msg->position[0]),
                                      static_cast<double>(msg->position[1]),
                                      static_cast<double>(msg->position[2]));
    sample.orientation =
        Eigen::Quaterniond(static_cast<double>(msg->q[0]), static_cast<double>(msg->q[1]),
                           static_cast<double>(msg->q[2]), static_cast<double>(msg->q[3]));

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

    if (!use_px4_odom_) {
        // Just apply static ENU -> NED transform: (x, y, z) -> (y, x, -z)
        for (size_t i = 0; i < n; ++i, ++it_ox, ++it_oy, ++it_oz, ++it_oi) {
            const uint8_t* p = in_data + i * step;
            float x = *reinterpret_cast<const float*>(p + off_x);
            float y = *reinterpret_cast<const float*>(p + off_y);
            float z = *reinterpret_cast<const float*>(p + off_z);

            *it_ox = y;
            *it_oy = x;
            *it_oz = -z;
            *it_oi = has_intensity ? *reinterpret_cast<const float*>(p + off_i) : 0.0f;
        }
    } else {
        // Get interpolated poses
        px4_common::time::PoseSample lio_sample, px4_sample;
        if (!lio_pose_buffer_.Lookup(scan_t_ns, lio_sample)) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                                 "LIO pose interpolation failed for scan at time %ld ns",
                                 scan_t_ns);
            return false;
        }

        if (!px4_pose_buffer_.Lookup(scan_t_ns, px4_sample)) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                                 "PX4 pose interpolation failed for scan at time %ld ns",
                                 scan_t_ns);
            return false;
        }

        // Compose the combined transform
        // p_ned = R_ned_frd * C * R_lio^T * (p_camera_init - t_lio) + t_px4
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
    }

    return true;
}

void NedTransformNode::PublishVisualOdometry(const px4_common::time::PoseSample& lio_sample) {
    if (!publish_visual_odometry_to_px4_ || !pub_visual_odom_) {
        return;
    }

    std::lock_guard<std::mutex> lock(visual_odom_mutex_);

    // Convert LIO position and orientation to NED
    Eigen::Vector3d p_raw = px4_common::math::EnuToNed(lio_sample.position);
    Eigen::Quaterniond q_raw =
        px4_ros_com::frame_transforms::QuaternionEnuToNed(lio_sample.orientation);

    if (visual_odom_align_to_px4_ && !visual_alignment_ready_) {
        // Get interpolated PX4 pose for alignment
        px4_common::time::PoseSample px4_sample;
        if (!px4_pose_buffer_.Lookup(lio_sample.t_ns, px4_sample)) {
            return;
        }

        // Calculate alignment transform
        const double yaw_delta = px4_common::math::QuaternionGetYaw(px4_sample.orientation) -
                                 px4_common::math::QuaternionGetYaw(q_raw);

        visual_align_rotation_ =
            Eigen::Quaterniond(Eigen::AngleAxisd(yaw_delta, Eigen::Vector3d::UnitZ()));
        visual_align_translation_ = px4_sample.position - visual_align_rotation_ * p_raw;
        visual_alignment_ready_ = true;

        RCLCPP_INFO(
            this->get_logger(),
            "[NED] FAST-LIO2 EV alignment captured: yaw_delta=%.3f rad, offset=(%.3f, %.3f, %.3f)",
            yaw_delta, visual_align_translation_.x(), visual_align_translation_.y(),
            visual_align_translation_.z());
    }

    if (visual_odom_align_to_px4_ && !visual_alignment_ready_) {
        return;
    }

    // Apply alignment if needed
    const Eigen::Vector3d p_ev = visual_alignment_ready_
                                     ? visual_align_rotation_ * p_raw + visual_align_translation_
                                     : p_raw;
    const Eigen::Quaterniond q_ev =
        (visual_alignment_ready_ ? visual_align_rotation_ * q_raw : q_raw).normalized();

    // Convert timestamp to microseconds for PX4
    const uint64_t sample_px4_us =
        static_cast<uint64_t>(static_cast<double>(lio_sample.t_ns) / 1000.0);
    const uint64_t now_px4_us = static_cast<uint64_t>(this->now().nanoseconds() / 1000.0);

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
        ev_msg.position_variance[i] = static_cast<float>(visual_odom_position_variance_[i]);
        ev_msg.orientation_variance[i] = static_cast<float>(visual_odom_orientation_variance_[i]);
    }

    ev_msg.reset_counter = 0;
    ev_msg.quality = static_cast<int8_t>(visual_odom_quality_);

    pub_visual_odom_->publish(ev_msg);
}

}  // namespace px4_mapping