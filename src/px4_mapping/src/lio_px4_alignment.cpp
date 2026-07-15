// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2026, LeTanDat

#include "px4_mapping/lio_px4_alignment.hpp"

#include <cmath>
#include <functional>
#include <limits>
#include <stdexcept>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <px4_ros2_utils/frame/covariance.hpp>
#include <px4_ros2_utils/frame/transform.hpp>
#include <px4_ros2_utils/parameter/param_utils.hpp>
#include <px4_ros2_utils/px4/topic.hpp>
#include <px4_ros2_utils/qos/sensor.hpp>

namespace px4_mapping {

namespace {

float VarianceOrNaN(double value) {
    return std::isfinite(value) && value >= 0.0 ? static_cast<float>(value)
                                                : std::numeric_limits<float>::quiet_NaN();
}

bool HasFinitePose(const nav_msgs::msg::Odometry& odometry) {
    const auto& position = odometry.pose.pose.position;
    const auto& orientation = odometry.pose.pose.orientation;
    const Eigen::Quaterniond q_enu_flu(orientation.w, orientation.x, orientation.y, orientation.z);
    return std::isfinite(position.x) && std::isfinite(position.y) && std::isfinite(position.z) &&
           q_enu_flu.coeffs().allFinite() && q_enu_flu.norm() > 1e-12;
}

}  // namespace

px4_msgs::msg::VehicleOdometry ConvertLioOdometryToPx4(const nav_msgs::msg::Odometry& lio_msg,
                                                       std::uint64_t publish_timestamp_us,
                                                       std::uint64_t sample_timestamp_us,
                                                       std::int8_t quality) {
    const Eigen::Vector3d position_enu(lio_msg.pose.pose.position.x, lio_msg.pose.pose.position.y,
                                       lio_msg.pose.pose.position.z);
    const Eigen::Quaterniond q_enu_flu(
        lio_msg.pose.pose.orientation.w, lio_msg.pose.pose.orientation.x,
        lio_msg.pose.pose.orientation.y, lio_msg.pose.pose.orientation.z);

    Eigen::Vector3d position_ned;
    Eigen::Quaterniond q_ned_frd;
    px4_ros2_utils::frame::enu_to_ned_pose(position_enu, q_enu_flu.normalized(), position_ned,
                                           q_ned_frd);

    px4_msgs::msg::VehicleOdometry px4_msg;
    px4_msg.timestamp = publish_timestamp_us;
    px4_msg.timestamp_sample = sample_timestamp_us;
    px4_msg.pose_frame = px4_msgs::msg::VehicleOdometry::POSE_FRAME_NED;
    px4_msg.position[0] = static_cast<float>(position_ned.x());
    px4_msg.position[1] = static_cast<float>(position_ned.y());
    px4_msg.position[2] = static_cast<float>(position_ned.z());
    px4_msg.q[0] = static_cast<float>(q_ned_frd.w());
    px4_msg.q[1] = static_cast<float>(q_ned_frd.x());
    px4_msg.q[2] = static_cast<float>(q_ned_frd.y());
    px4_msg.q[3] = static_cast<float>(q_ned_frd.z());

    // FAST-LIO currently does not populate nav_msgs/Odometry.twist. Publishing
    // default zeroes as measured velocity would over-constrain EKF2, so mark
    // velocity and angular velocity unavailable explicitly.
    px4_msg.velocity_frame = px4_msgs::msg::VehicleOdometry::VELOCITY_FRAME_UNKNOWN;
    const float nan = std::numeric_limits<float>::quiet_NaN();
    for (int axis = 0; axis < 3; ++axis) {
        px4_msg.velocity[axis] = nan;
        px4_msg.angular_velocity[axis] = nan;
        px4_msg.velocity_variance[axis] = nan;
    }

    const Eigen::Vector3d position_variance_enu(
        lio_msg.pose.covariance[0], lio_msg.pose.covariance[7], lio_msg.pose.covariance[14]);
    const Eigen::Vector3d position_variance_ned =
        px4_ros2_utils::frame::enu_var_to_ned(position_variance_enu);
    const Eigen::Vector3d orientation_variance_flu(
        lio_msg.pose.covariance[21], lio_msg.pose.covariance[28], lio_msg.pose.covariance[35]);
    const Eigen::Vector3d orientation_variance_frd =
        px4_ros2_utils::frame::flu_var_to_frd(orientation_variance_flu);

    for (int axis = 0; axis < 3; ++axis) {
        px4_msg.position_variance[axis] = VarianceOrNaN(position_variance_ned[axis]);
        px4_msg.orientation_variance[axis] = VarianceOrNaN(orientation_variance_frd[axis]);
    }

    px4_msg.reset_counter = 0;
    px4_msg.quality = quality;
    return px4_msg;
}

LioPx4Alignment::LioPx4Alignment(const rclcpp::NodeOptions& options)
    : rclcpp::Node("lio_px4_alignment", options) {
    LoadParameters();

    bool use_sim_time = false;
    this->get_parameter("use_sim_time", use_sim_time);
    const auto timesync_mode = use_sim_time ? px4_ros2_utils::time::Timesync::Mode::Simulation
                                            : px4_ros2_utils::time::Timesync::Mode::External;
    timesync_ = std::make_unique<px4_ros2_utils::time::Timesync>(this->get_clock(), timesync_mode);

    lio_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        lio_topic_, px4_ros2_utils::qos::telemetry_qos(20),
        std::bind(&LioPx4Alignment::LioCallback, this, std::placeholders::_1));
    px4_pub_ = this->create_publisher<px4_msgs::msg::VehicleOdometry>(
        px4_topic_, px4_ros2_utils::qos::sensor_qos(20));

    if (!use_sim_time) {
        timesync_sub_ = this->create_subscription<px4_msgs::msg::TimesyncStatus>(
            px4_ros2_utils::px4::topic::topic_name<px4_msgs::msg::TimesyncStatus>(
                "/fmu/out/timesync_status"),
            px4_ros2_utils::qos::sensor_qos(5),
            [this](px4_msgs::msg::TimesyncStatus::SharedPtr msg) {
                timesync_->update(msg);
            });
    }

    RCLCPP_INFO(this->get_logger(),
                "LIO-PX4 basis converter started: %s (ENU/FLU required) -> %s (NED/FRD), "
                "quality=%d; origin/yaw alignment is not estimated",
                lio_topic_.c_str(), px4_topic_.c_str(), visual_odom_quality_);
}

void LioPx4Alignment::LoadParameters() {
    const auto load_parameter = [this]<typename T>(const std::string& name, const T& default_value,
                                                   const std::string& description) {
        return px4_ros2_utils::parameter::declare_and_get(
            *this, name, default_value, px4_ros2_utils::parameter::descriptor(description));
    };

    lio_topic_ = load_parameter(
        "lio_topic", std::string("/lio/odometry"),
        "Odometry input topic; pose must already satisfy ENU-world/FLU-body semantics.");
    px4_topic_ = load_parameter("px4_topic", std::string("/fmu/in/vehicle_visual_odometry"),
                                "PX4 external-odometry output topic (NED world, FRD body).");
    visual_odom_quality_ =
        load_parameter("visual_odom_quality", 100, "PX4 VehicleOdometry quality in [1, 100].");

    if (lio_topic_.empty() || px4_topic_.empty()) {
        throw std::invalid_argument("lio_topic and px4_topic must not be empty");
    }
    if (visual_odom_quality_ < 1 || visual_odom_quality_ > 100) {
        throw std::invalid_argument("visual_odom_quality must be in [1, 100]");
    }
}

void LioPx4Alignment::LioCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    if (!msg || !HasFinitePose(*msg)) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                             "Dropping LIO odometry with invalid pose");
        return;
    }

    const rclcpp::Time sample_time(msg->header.stamp, this->get_clock()->get_clock_type());
    const auto sample_px4_us = timesync_->toPX4(sample_time);
    const auto publish_px4_us = timesync_->toPX4(this->now());
    if (!sample_px4_us || !publish_px4_us || *sample_px4_us == 0 || *publish_px4_us == 0) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                             "Dropping LIO odometry until PX4/ROS timesync is valid");
        return;
    }

    px4_pub_->publish(ConvertLioOdometryToPx4(*msg, *publish_px4_us, *sample_px4_us,
                                              static_cast<std::int8_t>(visual_odom_quality_)));
}

}  // namespace px4_mapping
