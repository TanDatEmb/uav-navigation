// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2026, LeTanDat

#include "px4_mapping/lio_px4_bridge.hpp"

#include <array>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <string>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <px4_ros2_utils/px4/topic.hpp>
#include <px4_ros2_utils/qos/sensor.hpp>
#include <px4_ros2_utils/parameter/param_utils.hpp>
#include <rclcpp/rclcpp.hpp>

namespace px4_mapping {

namespace {

/**
 * @brief Basis transform matrix: FLU → FRD
 * C = diag(1, -1, -1)
 * This converts between FLU (Forward-Left-Up) and FRD (Forward-Right-Down) conventions.
 */
const Eigen::Matrix3d kFluToFrd = (Eigen::Matrix3d() << 1.0, 0.0, 0.0,
                                                          0.0, -1.0, 0.0,
                                                          0.0, 0.0, -1.0).finished();

/**
 * @brief Convert FLU position to FRD.
 * p_frd = C * p_flu
 */
Eigen::Vector3d FluToFrdPosition(const Eigen::Vector3d& p_flu) {
    return kFluToFrd * p_flu;
}

/**
 * @brief Convert FLU orientation to FRD.
 * R_frd = C * R_flu * C
 * (converts both world and body frames)
 */
Eigen::Quaterniond FluToFrdOrientation(const Eigen::Quaterniond& q_flu) {
    // R_frd = C * R_flu * C
    // For quaternion: if R = q, then C*R*C corresponds to:
    // Construct rotation matrix, apply C on both sides, convert back to quaternion
    Eigen::Matrix3d R_flu = q_flu.toRotationMatrix();
    Eigen::Matrix3d R_frd = kFluToFrd * R_flu * kFluToFrd;
    return Eigen::Quaterniond(R_frd).normalized();
}

float VarianceOrNaN(double value) {
    return std::isfinite(value) && value >= 0.0 ? static_cast<float>(value)
                                                : std::numeric_limits<float>::quiet_NaN();
}

bool HasFinitePose(const nav_msgs::msg::Odometry& odometry) {
    const auto& p = odometry.pose.pose.position;
    const auto& o = odometry.pose.pose.orientation;
    const Eigen::Quaterniond q(o.w, o.x, o.y, o.z);
    return std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.z) &&
           q.coeffs().allFinite() && q.norm() > 1e-12;
}

}  // namespace

px4_msgs::msg::VehicleOdometry ConvertLioToPx4Frd(const nav_msgs::msg::Odometry& lio_msg,
                                                   std::uint64_t publish_timestamp_us,
                                                   std::uint64_t sample_timestamp_us,
                                                   std::int8_t quality) {
    // Extract LIO pose (FLU body in ENU world)
    const Eigen::Vector3d p_flu(lio_msg.pose.pose.position.x,
                                lio_msg.pose.pose.position.y,
                                lio_msg.pose.pose.position.z);
    const Eigen::Quaterniond q_flu(lio_msg.pose.pose.orientation.w,
                                   lio_msg.pose.pose.orientation.x,
                                   lio_msg.pose.pose.orientation.y,
                                   lio_msg.pose.pose.orientation.z);

    // Convert to FRD
    const Eigen::Vector3d p_frd = FluToFrdPosition(p_flu);
    const Eigen::Quaterniond q_frd = FluToFrdOrientation(q_flu.normalized());

    px4_msgs::msg::VehicleOdometry px4_msg;
    px4_msg.timestamp = publish_timestamp_us;
    px4_msg.timestamp_sample = sample_timestamp_us;
    px4_msg.pose_frame = px4_msgs::msg::VehicleOdometry::POSE_FRAME_FRD;

    px4_msg.position[0] = static_cast<float>(p_frd.x());
    px4_msg.position[1] = static_cast<float>(p_frd.y());
    px4_msg.position[2] = static_cast<float>(p_frd.z());

    px4_msg.q[0] = static_cast<float>(q_frd.w());
    px4_msg.q[1] = static_cast<float>(q_frd.x());
    px4_msg.q[2] = static_cast<float>(q_frd.y());
    px4_msg.q[3] = static_cast<float>(q_frd.z());

    // Velocity not provided by LIO - mark as unknown
    px4_msg.velocity_frame = px4_msgs::msg::VehicleOdometry::VELOCITY_FRAME_UNKNOWN;
    const float nan = std::numeric_limits<float>::quiet_NaN();
    for (int i = 0; i < 3; ++i) {
        px4_msg.velocity[i] = nan;
        px4_msg.angular_velocity[i] = nan;
        px4_msg.velocity_variance[i] = nan;
    }

    // Variance: transform position variance via basis transform
    // For C = diag(1,-1,-1): variance transforms as diag(same, same, same)
    // (squaring removes sign)
    px4_msg.position_variance[0] = VarianceOrNaN(lio_msg.pose.covariance[0]);
    px4_msg.position_variance[1] = VarianceOrNaN(lio_msg.pose.covariance[7]);
    px4_msg.position_variance[2] = VarianceOrNaN(lio_msg.pose.covariance[14]);

    // Orientation variance similarly (for small angles)
    px4_msg.orientation_variance[0] = VarianceOrNaN(lio_msg.pose.covariance[21]);
    px4_msg.orientation_variance[1] = VarianceOrNaN(lio_msg.pose.covariance[28]);
    px4_msg.orientation_variance[2] = VarianceOrNaN(lio_msg.pose.covariance[35]);

    px4_msg.reset_counter = 0;  // Caller sets this
    px4_msg.quality = quality;

    return px4_msg;
}

LioPx4Bridge::LioPx4Bridge(const rclcpp::NodeOptions& options)
    : rclcpp::Node("lio_px4_bridge", options) {
    LoadParameters();

    bool use_sim_time = false;
    this->get_parameter("use_sim_time", use_sim_time);
    const auto timesync_mode = use_sim_time
        ? px4_ros2_utils::time::Timesync::Mode::Simulation
        : px4_ros2_utils::time::Timesync::Mode::External;
    timesync_ = std::make_unique<px4_ros2_utils::time::Timesync>(
        this->get_clock(), timesync_mode);

    const auto px4_qos = px4_ros2_utils::qos::sensor_qos(20);

    lio_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        lio_topic_, px4_ros2_utils::qos::telemetry_qos(20),
        std::bind(&LioPx4Bridge::LioCallback, this, std::placeholders::_1));

    px4_pub_ = this->create_publisher<px4_msgs::msg::VehicleOdometry>(
        px4_topic_, px4_qos);

    diagnostic_pub_ = this->create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
        "/lio/diagnostics", 1);

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
                "LIO-PX4 bridge started: %s -> %s (FRD frame, no alignment)",
                lio_topic_.c_str(), px4_topic_.c_str());
}

void LioPx4Bridge::LoadParameters() {
    const auto load_parameter = [this]<typename T>(const std::string& name,
                                               const T& default_value,
                                               const std::string& description) {
        return px4_ros2_utils::parameter::declare_and_get(
            *this, name, default_value,
            px4_ros2_utils::parameter::descriptor(description));
    };

    lio_topic_ = load_parameter("lio_topic", std::string("/lio/odometry"),
                              "FAST-LIO odometry input topic (FLU body, ENU world)");
    px4_topic_ = load_parameter("px4_topic",
                                std::string("/fmu/in/vehicle_visual_odometry"),
                                "PX4 external vision output topic");

    visual_odom_quality_ = load_parameter("visual_odom_quality", 100,
                                         "External vision quality [1, 100]");

    auto position_variance = load_parameter(
        "position_variance", std::vector<double>{0.04, 0.04, 0.09},
        "Position variance in m^2");
    auto orientation_variance = load_parameter(
        "orientation_variance", std::vector<double>{0.25, 0.25, 0.05},
        "Orientation variance in rad^2");

    if (lio_topic_.empty() || px4_topic_.empty()) {
        throw std::invalid_argument("lio_topic and px4_topic must not be empty");
    }
    if (visual_odom_quality_ < 1 || visual_odom_quality_ > 100) {
        throw std::invalid_argument("visual_odom_quality must be in [1, 100]");
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

    {
        std::lock_guard<std::mutex> lock(diag_mutex_);
        ++messages_received_;
    }

    PublishVisualOdometry(*msg);
}

void LioPx4Bridge::PublishVisualOdometry(const nav_msgs::msg::Odometry& lio_msg) {
    const auto sample_px4_us_opt = timesync_->toPX4(
        rclcpp::Time(lio_msg.header.stamp, this->get_clock()->get_clock_type()));
    const auto now_px4_us_opt = timesync_->toPX4(this->now());

    if (!sample_px4_us_opt || !now_px4_us_opt ||
        *sample_px4_us_opt == 0 || *now_px4_us_opt == 0) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                             "Skip EV publish: timesync not ready");
        return;
    }

    // Build EV message
    auto ev_msg = ConvertLioToPx4Frd(lio_msg, *now_px4_us_opt, *sample_px4_us_opt,
                                       visual_odom_quality_);
    ev_msg.reset_counter = reset_counter_;

    // Override variances from config
    for (int i = 0; i < 3; ++i) {
        ev_msg.position_variance[i] = static_cast<float>(position_variance_[i]);
        ev_msg.orientation_variance[i] = static_cast<float>(orientation_variance_[i]);
    }

    px4_pub_->publish(ev_msg);

    {
        std::lock_guard<std::mutex> lock(diag_mutex_);
        ++messages_published_;
    }

    PublishDiagnostics();
}

void LioPx4Bridge::PublishDiagnostics() {
    if (!diagnostic_pub_ || diagnostic_pub_->get_subscription_count() == 0) {
        return;
    }

    // Copy data under lock, then publish without lock
    uint64_t received, published;
    {
        std::lock_guard<std::mutex> lock(diag_mutex_);
        received = messages_received_;
        published = messages_published_;
    }

    diagnostic_msgs::msg::DiagnosticArray diag_array;
    diag_array.header.stamp = this->now();

    diagnostic_msgs::msg::DiagnosticStatus status;
    status.name = "lio_px4_bridge: frd";
    status.hardware_id = "lio_px4_bridge";
    status.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
    status.message = "publishing FRD";

    auto add_kv = [&status](const std::string& key, const std::string& value) {
        diagnostic_msgs::msg::KeyValue kv;
        kv.key = key;
        kv.value = value;
        status.values.push_back(kv);
    };

    add_kv("frame", "FRD");
    add_kv("received", std::to_string(received));
    add_kv("published", std::to_string(published));
    add_kv("quality", std::to_string(visual_odom_quality_));

    diag_array.status.push_back(status);
    diagnostic_pub_->publish(diag_array);
}

}  // namespace px4_mapping
