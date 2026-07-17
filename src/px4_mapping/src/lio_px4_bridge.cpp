// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2026, LeTanDat

#include "px4_mapping/lio_px4_bridge.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <functional>
#include <limits>
#include <stdexcept>
#include <string>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <nav_msgs/msg/odometry.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <px4_ros2_utils/frame/covariance.hpp>
#include <px4_ros2_utils/frame/transform.hpp>
#include <px4_ros2_utils/math/quaternion.hpp>
#include <px4_ros2_utils/parameter/param_utils.hpp>
#include <px4_ros2_utils/px4/topic.hpp>
#include <px4_ros2_utils/qos/sensor.hpp>
#include <rclcpp/rclcpp.hpp>

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

bool IsValidAlignmentMode(const std::string& mode) {
    return mode == "translation_only" || mode == "yaw_translation" || mode == "full_6dof";
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

LioPx4Bridge::LioPx4Bridge(const rclcpp::NodeOptions& options)
    : rclcpp::Node("lio_px4_bridge", options),
      lio_pose_buffer_(500, std::chrono::nanoseconds(5'000'000'000LL)),
      px4_pose_buffer_(500, std::chrono::nanoseconds(5'000'000'000LL)) {
    LoadParameters();

    bool use_sim_time = false;
    this->get_parameter("use_sim_time", use_sim_time);
    const auto timesync_mode = use_sim_time ? px4_ros2_utils::time::Timesync::Mode::Simulation
                                            : px4_ros2_utils::time::Timesync::Mode::External;
    timesync_ = std::make_unique<px4_ros2_utils::time::Timesync>(this->get_clock(), timesync_mode);

    const auto px4_qos = px4_ros2_utils::qos::sensor_qos(20);

    lio_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        lio_topic_, px4_ros2_utils::qos::telemetry_qos(20),
        std::bind(&LioPx4Bridge::LioCallback, this, std::placeholders::_1));

    px4_odom_sub_ = this->create_subscription<px4_msgs::msg::VehicleOdometry>(
        px4_odom_topic_, px4_qos,
        std::bind(&LioPx4Bridge::Px4OdomCallback, this, std::placeholders::_1));

    px4_pub_ = this->create_publisher<px4_msgs::msg::VehicleOdometry>(px4_topic_, px4_qos);

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
                "LIO-PX4 bridge started: %s (ENU/FLU) + %s -> %s (NED/FRD), "
                "alignment=%s, quality=%d",
                lio_topic_.c_str(), px4_odom_topic_.c_str(), px4_topic_.c_str(),
                align_to_px4_ ? alignment_mode_.c_str() : "none", visual_odom_quality_);
}

void LioPx4Bridge::LoadParameters() {
    const auto load_parameter = [this]<typename T>(const std::string& name, const T& default_value,
                                                   const std::string& description) {
        return px4_ros2_utils::parameter::declare_and_get(
            *this, name, default_value, px4_ros2_utils::parameter::descriptor(description));
    };

    lio_topic_ = load_parameter("lio_topic", std::string("/lio/odometry"),
                                "Odometry input topic; pose must satisfy ENU-world/FLU-body "
                                "semantics (e.g. FAST-LIO output).");
    px4_odom_topic_ = load_parameter("px4_odom_topic", std::string("/fmu/out/vehicle_odometry"),
                                     "PX4 vehicle odometry topic used for alignment capture.");
    px4_topic_ = load_parameter("px4_topic", std::string("/fmu/in/vehicle_visual_odometry"),
                                "PX4 external-odometry output topic (NED world, FRD body).");

    align_to_px4_ = load_parameter(
        "align_to_px4", true,
        "Estimate and apply a fixed T_map_ned_lio_world from PX4 odometry at startup.");
    alignment_mode_ = load_parameter(
        "alignment_mode", std::string("translation_only"),
        "Alignment mode: translation_only, yaw_translation, or full_6dof.");
    visual_odom_quality_ = load_parameter("visual_odom_quality", 100,
                                          "PX4 VehicleOdometry quality in [1, 100].");

    auto position_variance =
        load_parameter("position_variance", std::vector<double>{0.04, 0.04, 0.09},
                       "External vision position variance in m^2.");
    auto orientation_variance =
        load_parameter("orientation_variance", std::vector<double>{0.25, 0.25, 0.05},
                       "External vision orientation variance in rad^2.");

    if (lio_topic_.empty() || px4_odom_topic_.empty() || px4_topic_.empty()) {
        throw std::invalid_argument("lio_topic, px4_odom_topic and px4_topic must not be empty");
    }
    if (visual_odom_quality_ < 1 || visual_odom_quality_ > 100) {
        throw std::invalid_argument("visual_odom_quality must be in [1, 100]");
    }
    if (!IsValidAlignmentMode(alignment_mode_)) {
        RCLCPP_WARN(this->get_logger(),
                    "Invalid alignment_mode='%s', fallback to translation_only",
                    alignment_mode_.c_str());
        alignment_mode_ = "translation_only";
    }

    for (size_t i = 0; i < 3 && i < position_variance.size(); ++i) {
        position_variance_[i] = std::max(position_variance[i], 1e-6);
    }
    for (size_t i = 0; i < 3 && i < orientation_variance.size(); ++i) {
        orientation_variance_[i] = std::max(orientation_variance[i], 1e-6);
    }
}

void LioPx4Bridge::LioCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    if (!msg || !HasFinitePose(*msg)) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                             "Dropping LIO odometry with invalid pose");
        return;
    }

    const rclcpp::Time sample_time(msg->header.stamp, this->get_clock()->get_clock_type());
    const int64_t t_ns = sample_time.nanoseconds();

    px4_mapping::time::PoseSample sample;
    sample.t_ns = t_ns;
    sample.position = Eigen::Vector3d(msg->pose.pose.position.x, msg->pose.pose.position.y,
                                      msg->pose.pose.position.z);
    sample.orientation =
        Eigen::Quaterniond(msg->pose.pose.orientation.w, msg->pose.pose.orientation.x,
                           msg->pose.pose.orientation.y, msg->pose.pose.orientation.z)
            .normalized();

    lio_pose_buffer_.Push(sample);

    PublishVisualOdometry(sample);
}

void LioPx4Bridge::Px4OdomCallback(const px4_msgs::msg::VehicleOdometry::SharedPtr msg) {
    const auto sample_stamp = timesync_->toROS(msg->timestamp_sample);
    if (!sample_stamp) {
        RCLCPP_WARN_THROTTLE(
            this->get_logger(), *this->get_clock(), 5000,
            "PX4 /fmu/out/vehicle_odometry timestamp_sample invalid or conversion failed "
            "(px4_us=%lu). Verify UXRCE_DDS_SYNCT and /clock setup.",
            static_cast<unsigned long>(msg->timestamp_sample));
        return;
    }

    px4_mapping::time::PoseSample sample;
    sample.t_ns = sample_stamp->nanoseconds();
    sample.position = Eigen::Vector3d(static_cast<double>(msg->position[0]),
                                      static_cast<double>(msg->position[1]),
                                      static_cast<double>(msg->position[2]));
    sample.orientation =
        Eigen::Quaterniond(static_cast<double>(msg->q[0]), static_cast<double>(msg->q[1]),
                           static_cast<double>(msg->q[2]), static_cast<double>(msg->q[3]))
            .normalized();

    px4_pose_buffer_.Push(sample);
}

void LioPx4Bridge::PublishVisualOdometry(const px4_mapping::time::PoseSample& lio_sample) {
    std::lock_guard<std::mutex> lock(alignment_mutex_);

    // Convert LIO pose from ENU world / FLU body to NED world / FRD body.
    px4_mapping::time::PoseSample lio_in_ned;
    lio_in_ned.t_ns = lio_sample.t_ns;
    px4_ros2_utils::frame::enu_to_ned_pose(lio_sample.position, lio_sample.orientation,
                                           lio_in_ned.position, lio_in_ned.orientation);

    if (align_to_px4_ && !alignment_ready_) {
        px4_mapping::time::PoseSample px4_sample;
        if (!px4_pose_buffer_.Lookup(lio_in_ned.t_ns, px4_sample)) {
            ++px4_pose_lookup_miss_;
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                                 "Deferring EV alignment until PX4 odometry is available "
                                 "(miss=%lu)",
                                 static_cast<unsigned long>(px4_pose_lookup_miss_));
            return;
        }

        if (alignment_mode_ == "full_6dof") {
            align_rotation_ = px4_sample.orientation * lio_in_ned.orientation.inverse();
            align_translation_ = px4_sample.position - align_rotation_ * lio_in_ned.position;
        } else if (alignment_mode_ == "yaw_translation") {
            const double yaw_delta =
                px4_ros2_utils::math::quaternion_to_rpy(px4_sample.orientation).z() -
                px4_ros2_utils::math::quaternion_to_rpy(lio_in_ned.orientation).z();
            align_rotation_ =
                Eigen::Quaterniond(Eigen::AngleAxisd(yaw_delta, Eigen::Vector3d::UnitZ()));
            align_translation_ = px4_sample.position - align_rotation_ * lio_in_ned.position;
        } else {
            align_rotation_ = Eigen::Quaterniond::Identity();
            align_translation_ = px4_sample.position - lio_in_ned.position;
        }

        alignment_ready_ = true;
        RCLCPP_INFO(this->get_logger(),
                    "LIO-PX4 alignment captured (%s): offset=(%.3f, %.3f, %.3f)",
                    alignment_mode_.c_str(), align_translation_.x(), align_translation_.y(),
                    align_translation_.z());
    }

    if (align_to_px4_ && !alignment_ready_) {
        return;
    }

    const Eigen::Vector3d p_ev = alignment_ready_
                                     ? align_rotation_ * lio_in_ned.position + align_translation_
                                     : lio_in_ned.position;
    const Eigen::Quaterniond q_ev =
        (alignment_ready_ ? align_rotation_ * lio_in_ned.orientation : lio_in_ned.orientation)
            .normalized();

    const auto sample_px4_us_opt =
        timesync_->toPX4(rclcpp::Time(lio_sample.t_ns, this->get_clock()->get_clock_type()));
    const auto now_px4_us_opt = timesync_->toPX4(this->now());
    if (!sample_px4_us_opt || !now_px4_us_opt || *sample_px4_us_opt == 0 || *now_px4_us_opt == 0) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                             "Skip EV publish until PX4/ROS timesync yields valid non-zero "
                             "timestamps");
        return;
    }

    px4_msgs::msg::VehicleOdometry ev_msg;
    ev_msg.timestamp = *now_px4_us_opt;
    ev_msg.timestamp_sample = *sample_px4_us_opt;
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
        ev_msg.position_variance[i] = static_cast<float>(position_variance_[i]);
        ev_msg.orientation_variance[i] = static_cast<float>(orientation_variance_[i]);
    }

    ev_msg.reset_counter = 0;
    ev_msg.quality = static_cast<int8_t>(visual_odom_quality_);

    px4_pub_->publish(ev_msg);
}

}  // namespace px4_mapping
