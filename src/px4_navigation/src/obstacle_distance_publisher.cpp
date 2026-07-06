// Copyright 2026 CTUAV. All rights reserved.
//
// Implementation of the obstacle distance publisher node.
//
// Pipeline per timer tick:
//   1. Lock state mutex — snapshot the latest cached point cloud + yaw.
//   2. If cloud is stale or empty → publish UINT16_MAX to all bins (clear).
//   3. Otherwise, iterate points in body FRD:
//        - Filter by height band (|z| < height_band_m).
//        - Compute horizontal angle = atan2(y, x)  (FRD: x=Fwd, y=Right).
//        - Map to bin: bin = round((angle + pi) / increment_rad) mod 72.
//        - Keep minimum distance per bin, clamp to [min, max] cm.
//   4. Fill px4_msgs::ObstacleDistance and publish.

#include "px4_navigation/obstacle_distance_publisher.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <mutex>
#include <string>
#include <vector>

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <px4_common/math/transforms.hpp>
#include <px4_msgs/msg/obstacle_distance.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

namespace px4_navigation {

ObstacleDistancePublisher::ObstacleDistancePublisher(const rclcpp::NodeOptions& options)
    : rclcpp::Node("obstacle_distance_publisher", options),
      last_cloud_time_(this->now()) {
    LoadParameters();

    // QoS for PX4 odometry: best-effort, keep last 20 (matches PX4 DDS bridge).
    const auto px4_qos = rclcpp::QoS(20).best_effort();

    // Subscription for incoming point cloud.
    sub_cloud_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        input_cloud_topic_, rclcpp::SensorDataQoS(),
        std::bind(&ObstacleDistancePublisher::PointCloudCallback, this,
                  std::placeholders::_1));

    // Subscription for PX4 vehicle odometry (NED frame).
    sub_odom_ = this->create_subscription<px4_msgs::msg::VehicleOdometry>(
        vehicle_odom_topic_, px4_qos,
        std::bind(&ObstacleDistancePublisher::VehicleOdomCallback, this,
                  std::placeholders::_1));

    // Publisher for obstacle distance to PX4. Use reliable QoS to match
    // PX4 uXRCE-DDS subscription expectations.
    pub_obstacle_distance_ = this->create_publisher<px4_msgs::msg::ObstacleDistance>(
        obstacle_distance_topic_, rclcpp::QoS(20).reliable());

    // Publish timer.
    const auto period_ns = static_cast<int64_t>(1e9 / publish_rate_hz_);
    publish_timer_ = this->create_wall_timer(
        std::chrono::nanoseconds(period_ns),
        std::bind(&ObstacleDistancePublisher::PublishTimerCallback, this));

    RCLCPP_INFO(this->get_logger(),
                "obstacle_distance_publisher started: rate=%.1f Hz, bins=%d, "
                "range=[%d, %d] cm, height_band=%.1f m, cloud_topic='%s'",
                publish_rate_hz_, kNumBins, min_distance_cm_, max_distance_cm_,
                height_band_m_, input_cloud_topic_.c_str());
}

void ObstacleDistancePublisher::LoadParameters() {
    // Declare with descriptors and sensible defaults.
    publish_rate_hz_ = this->declare_parameter("publish_rate_hz", kDefaultRateHz);
    min_distance_cm_ = this->declare_parameter("min_distance_cm", kDefaultMinDistanceCm);
    max_distance_cm_ = this->declare_parameter("max_distance_cm", kDefaultMaxDistanceCm);
    height_band_m_ = this->declare_parameter("height_band_m", kDefaultHeightBandM);
    stale_timeout_ms_ = this->declare_parameter("stale_timeout_ms", kDefaultStaleTimeoutMs);
    input_cloud_topic_ = this->declare_parameter("input_cloud_topic", std::string("/livox_processed_ned"));
    vehicle_odom_topic_ = this->declare_parameter("vehicle_odom_topic", std::string("/fmu/out/vehicle_odometry"));
    obstacle_distance_topic_ = this->declare_parameter("obstacle_distance_topic", std::string("/fmu/in/obstacle_distance"));

    int frame_int = this->declare_parameter("frame", static_cast<int>(kFrameBodyFrd));
    frame_ = static_cast<uint8_t>(frame_int);

    // Basic validation.
    if (publish_rate_hz_ <= 0.0 || publish_rate_hz_ > 100.0) {
        RCLCPP_WARN(this->get_logger(), "publish_rate_hz=%.1f out of range, clamping to [1, 100]",
                    publish_rate_hz_);
        publish_rate_hz_ = std::clamp(publish_rate_hz_, 1.0, 100.0);
    }
    if (min_distance_cm_ < 0) {
        RCLCPP_WARN(this->get_logger(), "min_distance_cm=%d < 0, clamping to 0", min_distance_cm_);
        min_distance_cm_ = 0;
    }
    if (max_distance_cm_ <= min_distance_cm_) {
        RCLCPP_WARN(this->get_logger(), "max_distance_cm=%d <= min_distance_cm=%d, using defaults",
                    max_distance_cm_, min_distance_cm_);
        min_distance_cm_ = kDefaultMinDistanceCm;
        max_distance_cm_ = kDefaultMaxDistanceCm;
    }
}

void ObstacleDistancePublisher::PointCloudCallback(
    const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
    // Parse the point cloud into Eigen vectors.
    // We expect fields: x, y, z (float32) in the NED frame (from ned_transform_node).
    std::vector<Eigen::Vector3f> points;
    points.reserve(msg->width * msg->height);

    sensor_msgs::PointCloud2ConstIterator<float> iter_x(*msg, "x");
    sensor_msgs::PointCloud2ConstIterator<float> iter_y(*msg, "y");
    sensor_msgs::PointCloud2ConstIterator<float> iter_z(*msg, "z");

    for (size_t i = 0; i < msg->width * msg->height; ++i, ++iter_x, ++iter_y, ++iter_z) {
        points.emplace_back(*iter_x, *iter_y, *iter_z);
    }

    // Lock and store.
    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        cloud_body_frd_ = std::move(points);
        last_cloud_time_ = rclcpp::Time(msg->header.stamp, this->get_clock()->get_clock_type());
        cloud_received_ = true;
    }
    ++clouds_received_;
}

void ObstacleDistancePublisher::VehicleOdomCallback(
    const px4_msgs::msg::VehicleOdometry::SharedPtr msg) {
    // Extract yaw from quaternion (PX4 stores [w, x, y, z]).
    const Eigen::Quaterniond q(msg->q[0], msg->q[1], msg->q[2], msg->q[3]);
    const double yaw = px4_common::math::QuaternionGetYaw(q);

    std::lock_guard<std::mutex> lock(state_mutex_);
    vehicle_yaw_ned_ = yaw;
    vehicle_position_ned_ = Eigen::Vector3d(msg->position[0], msg->position[1], msg->position[2]);
    odom_received_ = true;
}

void ObstacleDistancePublisher::ComputeBinDistances(
    std::array<uint16_t, kNumBins>& distances) {
    // Initialise all bins to max_distance_cm (means "no obstacle detected
    // within range"). PX4 CP treats values > max_distance as "no obstacle".
    std::fill(distances.begin(), distances.end(),
              static_cast<uint16_t>(max_distance_cm_));

    // Snapshot under lock.
    std::vector<Eigen::Vector3f> cloud;
    double yaw_ned = 0.0;
    Eigen::Vector3d drone_pos_ned = Eigen::Vector3d::Zero();
    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        cloud = cloud_body_frd_;  // copy
        yaw_ned = vehicle_yaw_ned_;
        drone_pos_ned = vehicle_position_ned_;
    }

    if (cloud.empty()) {
        return;
    }

    // The input cloud is assumed to be in NED frame (from ned_transform_node).
    // We need to rotate points into the body FRD frame using the vehicle yaw.
    //
    // NED → body FRD rotation (yaw only):
    //   body_x =  cos(yaw) * ned_x + sin(yaw) * ned_y
    //   body_y = -sin(yaw) * ned_x + cos(yaw) * ned_y
    //   body_z =  ned_z
    //
    // This gives x = forward, y = right, z = down (FRD).
    const double cos_yaw = std::cos(yaw_ned);
    const double sin_yaw = std::sin(yaw_ned);

    // Angular increment in radians.
    constexpr double kIncrementRad = px4_common::math::Deg2Rad(kIncrementDeg);

    const float max_dist_m = static_cast<float>(max_distance_cm_) / 100.0f;
    const float min_dist_m = static_cast<float>(min_distance_cm_) / 100.0f;
    const float height_band = static_cast<float>(height_band_m_);

    for (const auto& pt_ned : cloud) {
        // Skip NaN/inf points.
        if (!pt_ned.allFinite()) {
            continue;
        }

        // Compute offset from drone position (both in NED world frame).
        const double dx = pt_ned.x() - drone_pos_ned.x();
        const double dy = pt_ned.y() - drone_pos_ned.y();
        const double dz = pt_ned.z() - drone_pos_ned.z();

        // Height band filter: only keep points within ±height_band of drone Z.
        if (std::abs(dz) > height_band) {
            continue;
        }

        // Rotate offset into body FRD (yaw only, valid for near-level flight).
        const float bx = static_cast<float>(cos_yaw * dx + sin_yaw * dy);
        const float by = static_cast<float>(-sin_yaw * dx + cos_yaw * dy);

        // Horizontal distance from drone to point.
        const float dist_m = std::sqrt(bx * bx + by * by);
        if (dist_m < min_dist_m || dist_m > max_dist_m) {
            continue;
        }

        // Angle in body FRD: 0 = forward, positive = clockwise (right).
        // atan2(y, x) with y=right, x=forward gives positive for right.
        const float angle = std::atan2(by, bx);

        // Map angle [-pi, pi] to bin [0, 71].
        // With angle_offset = 0 and BODY_FRD frame:
        //   bin 0  = -180° (directly behind)
        //   bin 36 =    0° (forward)
        //   bin 18 =  -90° (left)
        //   bin 54 =   90° (right)
        //
        // PX4 CP with BODY_FRD: bin 0 = forward, increments clockwise.
        // So we need: bin = round(angle / increment) mod 72, where angle is
        // in [0, 2π) with 0 = forward, positive clockwise.
        //
        // atan2 returns [-π, π] with 0 = forward, positive = right (clockwise).
        // Convert to [0, 2π): if angle < 0, add 2π.
        float angle_normalized = angle;
        if (angle_normalized < 0.0f) {
            angle_normalized += static_cast<float>(2.0 * px4_common::math::kPi);
        }

        int bin = static_cast<int>(std::round(angle_normalized / static_cast<float>(kIncrementRad)));
        bin = ((bin % kNumBins) + kNumBins) % kNumBins;  // wrap to [0, 71]

        // Convert to centimetres.
        const uint16_t dist_cm = static_cast<uint16_t>(std::round(dist_m * 100.0f));

        // Keep minimum distance per bin.
        if (dist_cm < distances[bin]) {
            distances[bin] = dist_cm;
        }
    }
}

void ObstacleDistancePublisher::PublishTimerCallback() {
    auto msg = px4_msgs::msg::ObstacleDistance();
    msg.timestamp = this->now().nanoseconds() / 1000;  // microseconds
    msg.frame = frame_;
    msg.sensor_type = kSensorTypeLaser;
    msg.increment = static_cast<float>(kIncrementDeg);
    msg.min_distance = static_cast<uint16_t>(min_distance_cm_);
    msg.max_distance = static_cast<uint16_t>(max_distance_cm_);
    msg.angle_offset = 0.0f;  // bin 0 = forward (BODY_FRD)

    // Check staleness.
    const auto now = this->now();
    bool is_stale = false;

    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        const auto age = now - last_cloud_time_;
        if (cloud_received_ && age > std::chrono::milliseconds(stale_timeout_ms_)) {
            is_stale = true;
        }
    }

    if (is_stale || !cloud_received_ || !odom_received_) {
        // Clear all bins: PX4 treats UINT16_MAX as "unknown" → stops movement.
        std::fill(msg.distances.begin(), msg.distances.end(), kNoObstacle);
        if (is_stale) {
            ++stale_clears_;
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                                 "Point cloud stale > %d ms, clearing obstacle map",
                                 stale_timeout_ms_);
        }
    } else {
        // Compute and fill distances.
        std::array<uint16_t, kNumBins> distances{};
        ComputeBinDistances(distances);
        std::copy(distances.begin(), distances.end(), msg.distances.begin());
    }

    pub_obstacle_distance_->publish(msg);
    ++messages_published_;

    // Periodic health log.
    RCLCPP_DEBUG_THROTTLE(this->get_logger(), *this->get_clock(), 10000,
                          "published=%lu clouds=%lu stale_clears=%lu",
                          static_cast<unsigned long>(messages_published_),
                          static_cast<unsigned long>(clouds_received_),
                          static_cast<unsigned long>(stale_clears_));
}

}  // namespace px4_navigation