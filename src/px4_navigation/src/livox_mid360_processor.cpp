// Copyright 2026 TanDatEmb.
//
// Implementation of the Livox MID-360 processor node.
//
// Pipeline per timer tick:
//   1. Lock state mutex and snapshot the latest cached point cloud + pose.
//   2. If the cloud is stale or no data is available, publish UINT16_MAX to
//      all bins (clear).
//   3. Otherwise, for each point:
//        - Skip points inside the body-exclusion sphere.
//        - Compute yaw and pitch angles in body FRD.
//        - Map to a yaw x pitch spherical grid cell.
//        - Keep the minimum horizontal distance per cell.
//   4. Per yaw column, take the minimum distance across all pitch rows.
//   5. Publish px4_msgs::ObstacleDistance to /fmu/in/obstacle_distance.

#include "px4_navigation/livox_mid360_processor.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <limits>
#include <string>

#include <px4_common/math/transforms.hpp>
#include <px4_ros_com/time_sync.hpp>
#include <rclcpp/rclcpp.hpp>

namespace px4_navigation {

LivoxMid360Processor::LivoxMid360Processor(const rclcpp::NodeOptions& options)
    : rclcpp::Node("obstacle_perception_node", options) {
    LoadParameters();

    // Allocate spherical grid.
    grid_.resize(static_cast<size_t>(yaw_bins_),
                 std::vector<GridCell>(static_cast<size_t>(pitch_bins_)));

    // QoS matching PX4 uXRCE-DDS bridge expectations.
    const auto px4_qos = rclcpp::QoS(20).best_effort();

    sub_cloud_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        input_cloud_topic_, rclcpp::SensorDataQoS(),
        std::bind(&LivoxMid360Processor::CloudCallback, this, std::placeholders::_1));

    sub_odom_ = this->create_subscription<px4_msgs::msg::VehicleOdometry>(
        vehicle_odom_topic_, px4_qos,
        std::bind(&LivoxMid360Processor::OdomCallback, this, std::placeholders::_1));

    pub_obstacle_distance_ = this->create_publisher<px4_msgs::msg::ObstacleDistance>(
        obstacle_distance_topic_, rclcpp::QoS(20).reliable());
    if (publish_local_virtual_scan_) {
        pub_local_virtual_scan_ =
            this->create_publisher<sensor_msgs::msg::LaserScan>(local_virtual_scan_topic_,
                                                                 rclcpp::QoS(20).reliable());
    }

    const auto period_ns = static_cast<int64_t>(1e9 / publish_rate_hz_);
    publish_timer_ = this->create_wall_timer(std::chrono::nanoseconds(period_ns),
                                             std::bind(&LivoxMid360Processor::TimerCallback, this));

    RCLCPP_INFO(this->get_logger(),
                "obstacle_perception started: cloud_frame='%s' yaw_bins=%d "
                "pitch_bins=%d pitch=[%.1f, %.1f] deg range=[%.2f, %.2f] m "
                "body_exclusion=%.2f m rate=%.1f Hz local_scan=%s",
                cloud_frame_.c_str(), yaw_bins_, pitch_bins_,
                px4_common::math::Rad2Deg(min_pitch_rad_),
                px4_common::math::Rad2Deg(max_pitch_rad_), min_distance_m_, max_distance_m_,
                body_exclusion_radius_m_, publish_rate_hz_,
                publish_local_virtual_scan_ ? local_virtual_scan_topic_.c_str() : "disabled");
}

void LivoxMid360Processor::LoadParameters() {
    yaw_bins_ = this->declare_parameter("yaw_bins", kDefaultYawBins);
    pitch_bins_ = this->declare_parameter("pitch_bins", kDefaultPitchBins);
    const double min_pitch_deg = this->declare_parameter("min_pitch_deg", kDefaultMinPitchDeg);
    const double max_pitch_deg = this->declare_parameter("max_pitch_deg", kDefaultMaxPitchDeg);
    min_pitch_rad_ = px4_common::math::Deg2Rad(min_pitch_deg);
    max_pitch_rad_ = px4_common::math::Deg2Rad(max_pitch_deg);

    min_distance_m_ = this->declare_parameter("min_distance_m", kDefaultMinDistanceM);
    max_distance_m_ = this->declare_parameter("max_distance_m", kDefaultMaxDistanceM);
    body_exclusion_radius_m_ =
        this->declare_parameter("body_exclusion_radius_m", kDefaultBodyExclusionRadiusM);
    publish_rate_hz_ = this->declare_parameter("publish_rate_hz", kDefaultPublishRateHz);
    stale_timeout_ms_ = this->declare_parameter("stale_timeout_ms", kDefaultStaleTimeoutMs);

    input_cloud_topic_ =
        this->declare_parameter("input_cloud_topic", std::string("/lidar_360/points"));
    vehicle_odom_topic_ =
        this->declare_parameter("vehicle_odom_topic", std::string("/fmu/out/vehicle_odometry"));
    obstacle_distance_topic_ = this->declare_parameter("obstacle_distance_topic",
                                                       std::string("/fmu/in/obstacle_distance"));
    this->declare_parameter("publish_local_virtual_scan", true);
    this->get_parameter("publish_local_virtual_scan", publish_local_virtual_scan_);
    this->declare_parameter("local_virtual_scan_topic", std::string("/livox/perception/scan_1d"));
    this->get_parameter("local_virtual_scan_topic", local_virtual_scan_topic_);
    this->declare_parameter("local_virtual_scan_frame_id", std::string("aircraft"));
    this->get_parameter("local_virtual_scan_frame_id", local_virtual_scan_frame_id_);

    cloud_frame_ = this->declare_parameter("cloud_frame", std::string("sensor_flu"));
    filter_ground_points_ = this->declare_parameter("filter_ground_points", true);

    // Validation.
    if (yaw_bins_ <= 0 || yaw_bins_ > 360) {
        RCLCPP_WARN(this->get_logger(), "yaw_bins=%d out of range, using default %d", yaw_bins_,
                    kDefaultYawBins);
        yaw_bins_ = kDefaultYawBins;
    }
    if (pitch_bins_ <= 0 || pitch_bins_ > 180) {
        RCLCPP_WARN(this->get_logger(), "pitch_bins=%d out of range, using default %d", pitch_bins_,
                    kDefaultPitchBins);
        pitch_bins_ = kDefaultPitchBins;
    }
    if (max_pitch_rad_ <= min_pitch_rad_) {
        RCLCPP_WARN(this->get_logger(),
                    "max_pitch_deg <= min_pitch_deg, using defaults [%.1f, %.1f]",
                    kDefaultMinPitchDeg, kDefaultMaxPitchDeg);
        min_pitch_rad_ = px4_common::math::Deg2Rad(kDefaultMinPitchDeg);
        max_pitch_rad_ = px4_common::math::Deg2Rad(kDefaultMaxPitchDeg);
    }
    if (max_distance_m_ <= min_distance_m_) {
        RCLCPP_WARN(this->get_logger(),
                    "max_distance_m=%.2f <= min_distance_m=%.2f, using defaults", max_distance_m_,
                    min_distance_m_);
        min_distance_m_ = kDefaultMinDistanceM;
        max_distance_m_ = kDefaultMaxDistanceM;
    }
    if (cloud_frame_ != "sensor" && cloud_frame_ != "sensor_flu" && cloud_frame_ != "ned") {
        RCLCPP_WARN(this->get_logger(),
                    "cloud_frame='%s' invalid, must be 'sensor', 'sensor_flu' or 'ned', defaulting "
                    "to 'sensor_flu'",
                    cloud_frame_.c_str());
        cloud_frame_ = "sensor_flu";
    }
}

void LivoxMid360Processor::CloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
    std::vector<Eigen::Vector3f> points;
    points.reserve(static_cast<size_t>(msg->width * msg->height));

    sensor_msgs::PointCloud2ConstIterator<float> iter_x(*msg, "x");
    sensor_msgs::PointCloud2ConstIterator<float> iter_y(*msg, "y");
    sensor_msgs::PointCloud2ConstIterator<float> iter_z(*msg, "z");

    const size_t n = static_cast<size_t>(msg->width * msg->height);
    for (size_t i = 0; i < n; ++i, ++iter_x, ++iter_y, ++iter_z) {
        points.emplace_back(*iter_x, *iter_y, *iter_z);
    }

    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        cloud_points_ = std::move(points);
        last_cloud_time_ = rclcpp::Time(msg->header.stamp, this->get_clock()->get_clock_type());
        last_cloud_arrival_time_ = std::chrono::steady_clock::now();
        ++latest_cloud_seq_;
        cloud_received_ = true;
    }
    ++clouds_received_;
}

void LivoxMid360Processor::OdomCallback(const px4_msgs::msg::VehicleOdometry::SharedPtr msg) {
    const Eigen::Quaterniond q(static_cast<double>(msg->q[0]), static_cast<double>(msg->q[1]),
                               static_cast<double>(msg->q[2]), static_cast<double>(msg->q[3]));
    const double yaw = px4_common::math::QuaternionGetYaw(q);

    std::lock_guard<std::mutex> lock(state_mutex_);
    vehicle_yaw_ = yaw;
    vehicle_position_ned_ = Eigen::Vector3d(static_cast<double>(msg->position[0]),
                                            static_cast<double>(msg->position[1]),
                                            static_cast<double>(msg->position[2]));
    odom_received_ = true;
}

void LivoxMid360Processor::TimerCallback() {
    std::vector<Eigen::Vector3f> cloud;
    bool have_cloud = false;
    bool have_odom = false;
    bool reuse_last_scan = false;
    uint64_t cloud_seq = 0;
    std::chrono::steady_clock::time_point cloud_arrival;

    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        have_cloud = cloud_received_;
        have_odom = odom_received_;
        cloud_seq = latest_cloud_seq_;
        reuse_last_scan = (cloud_frame_ != "ned") && last_min_distances_valid_ &&
                          (cloud_seq == last_processed_cloud_seq_);
        if (!reuse_last_scan) {
            cloud = cloud_points_;
        }
        cloud_arrival = last_cloud_arrival_time_;
    }

    bool is_stale = false;
    if (have_cloud) {
        const auto age_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::steady_clock::now() - cloud_arrival);
        is_stale = age_ms > std::chrono::milliseconds(stale_timeout_ms_);
    }

    auto obstacle_msg = px4_msgs::msg::ObstacleDistance();
    obstacle_msg.timestamp = px4_ros_com::time::RosTimeToPx4Microseconds(this->now());
    obstacle_msg.frame = kFrameBodyFrd;
    obstacle_msg.sensor_type = kSensorTypeLaser;
    obstacle_msg.increment = 360.0f / static_cast<float>(yaw_bins_);
    obstacle_msg.min_distance = static_cast<uint16_t>(std::round(min_distance_m_ * 100.0));
    obstacle_msg.max_distance = static_cast<uint16_t>(std::round(max_distance_m_ * 100.0));
    obstacle_msg.angle_offset = 0.0f;  // bin 0 = forward for BODY_FRD

    std::array<uint16_t, 72> min_distances{};
    std::fill(min_distances.begin(), min_distances.end(), kNoObstacle);

    if (is_stale || !have_cloud || !have_odom) {
        if (is_stale) {
            std::lock_guard<std::mutex> lock(state_mutex_);
            last_min_distances_valid_ = false;
        }
        if (is_stale) {
            ++stale_clears_;
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                                 "Point cloud stale > %d ms, clearing obstacle map",
                                 stale_timeout_ms_);
        }
    } else if (reuse_last_scan) {
        std::lock_guard<std::mutex> lock(state_mutex_);
        min_distances = last_min_distances_;
    } else {
        BuildSphericalGrid(cloud);
        ComputeMinDistances(min_distances);

        if (cloud_frame_ != "ned") {
            std::lock_guard<std::mutex> lock(state_mutex_);
            last_min_distances_ = min_distances;
            last_min_distances_valid_ = true;
            last_processed_cloud_seq_ = cloud_seq;
        }
    }

    std::copy(min_distances.begin(), min_distances.end(), obstacle_msg.distances.begin());

    pub_obstacle_distance_->publish(obstacle_msg);

    if (publish_local_virtual_scan_ && pub_local_virtual_scan_) {
        sensor_msgs::msg::LaserScan scan_msg;
        scan_msg.header.stamp = this->now();
        scan_msg.header.frame_id = local_virtual_scan_frame_id_;
        scan_msg.angle_min = static_cast<float>(-px4_common::math::kPi);
        scan_msg.angle_max = static_cast<float>(px4_common::math::kPi);
        scan_msg.angle_increment =
            static_cast<float>(2.0 * px4_common::math::kPi / static_cast<double>(min_distances.size()));
        scan_msg.time_increment = 0.0f;
        scan_msg.scan_time = static_cast<float>(1.0 / publish_rate_hz_);
        scan_msg.range_min = static_cast<float>(min_distance_m_);
        scan_msg.range_max = static_cast<float>(max_distance_m_);
        scan_msg.ranges.resize(min_distances.size());

        const float no_obstacle = std::numeric_limits<float>::infinity();
        for (size_t i = 0; i < min_distances.size(); ++i) {
            const uint16_t dist_cm = min_distances[i];
            scan_msg.ranges[i] = (dist_cm == kNoObstacle)
                                     ? no_obstacle
                                     : static_cast<float>(dist_cm) * 0.01f;
        }

        pub_local_virtual_scan_->publish(scan_msg);
    }

    ++messages_published_;

    RCLCPP_DEBUG_THROTTLE(
        this->get_logger(), *this->get_clock(), 10000, "published=%lu clouds=%lu stale_clears=%lu",
        static_cast<unsigned long>(messages_published_),
        static_cast<unsigned long>(clouds_received_), static_cast<unsigned long>(stale_clears_));
}

void LivoxMid360Processor::BuildSphericalGrid(const std::vector<Eigen::Vector3f>& cloud) {
    // Reset grid.
    for (auto& row : grid_) {
        for (auto& cell : row) {
            cell.min_distance_cm = kNoObstacle;
        }
    }

    if (cloud.empty()) {
        return;
    }

    const double yaw_bin_size_rad = 2.0 * px4_common::math::kPi / yaw_bins_;
    const double pitch_bin_size_rad = (max_pitch_rad_ - min_pitch_rad_) / pitch_bins_;

    const float min_dist_m = static_cast<float>(min_distance_m_);
    const float max_dist_m = static_cast<float>(max_distance_m_);
    const float body_excl_m = static_cast<float>(body_exclusion_radius_m_);

    // Precompute NED -> body FRD rotation for the current yaw if needed.
    double cos_yaw = 1.0;
    double sin_yaw = 0.0;
    Eigen::Vector3d drone_pos_ned = Eigen::Vector3d::Zero();
    if (cloud_frame_ == "ned") {
        std::lock_guard<std::mutex> lock(state_mutex_);
        cos_yaw = std::cos(vehicle_yaw_);
        sin_yaw = std::sin(vehicle_yaw_);
        drone_pos_ned = vehicle_position_ned_;
    }

    for (const auto& pt_in : cloud) {
        if (!pt_in.allFinite()) {
            continue;
        }

        Eigen::Vector3f pt_body;
        if (cloud_frame_ == "sensor") {
            // Cloud is already in the PX4 body FRD frame (x=FWD, y=RIGHT, z=DOWN).
            pt_body = pt_in;
        } else if (cloud_frame_ == "sensor_flu") {
            // Gazebo / ROS conventional sensor frame: x=FWD, y=LEFT, z=UP.
            // Convert to body FRD before spherical mapping.
            pt_body = Eigen::Vector3f(pt_in.x(), -pt_in.y(), -pt_in.z());
        } else {
            // Cloud is in NED world frame: transform to body FRD.
            // PX4 NED yaw is positive clockwise (right-hand rule around the
            // down-axis gives a CCW angle). QuaternionGetYaw follows Eigen's
            // right-hand convention, so we must rotate by -yaw to go from NED
            // to the body FRD frame where yaw/heading is measured clockwise.
            const Eigen::Vector3d p_ned(static_cast<double>(pt_in.x()),
                                        static_cast<double>(pt_in.y()),
                                        static_cast<double>(pt_in.z()));
            const Eigen::Vector3d p_rel = p_ned - drone_pos_ned;

            const double c = cos_yaw;
            const double s = sin_yaw;
            const double bx = c * p_rel.x() - s * p_rel.y();
            const double by = s * p_rel.x() + c * p_rel.y();
            const double bz = p_rel.z();
            pt_body = Eigen::Vector3f(static_cast<float>(bx), static_cast<float>(by),
                                      static_cast<float>(bz));
        }

        // Body exclusion: points too close to the UAV frame.
        const float dist_3d_m = pt_body.norm();
        if (dist_3d_m < body_excl_m) {
            continue;
        }

        // Ground-plane filter: in body FRD, positive z points downward.
        // Points below the horizontal plane through the sensor are assumed
        // to be ground returns in a flat-world SITL. This prevents the
        // ground plane from filling every yaw bin at ~2.4 m.
        if (filter_ground_points_ && pt_body.z() > 0.0f) {
            continue;
        }

        // Horizontal distance defines the obstacle distance reported to PX4.
        const float dist_horiz_m = std::sqrt(pt_body.x() * pt_body.x() + pt_body.y() * pt_body.y());
        if (dist_horiz_m < min_dist_m || dist_horiz_m > max_dist_m) {
            continue;
        }

        // Yaw angle in body FRD: 0 = forward, positive = clockwise (right).
        float yaw_angle = std::atan2(pt_body.y(), pt_body.x());
        if (yaw_angle < 0.0f) {
            yaw_angle += static_cast<float>(2.0 * px4_common::math::kPi);
        }

        // Pitch angle from horizontal plane.
        const float pitch_angle = std::atan2(pt_body.z(), dist_horiz_m);
        if (pitch_angle < static_cast<float>(min_pitch_rad_) ||
            pitch_angle > static_cast<float>(max_pitch_rad_)) {
            continue;
        }

        int yaw_bin = static_cast<int>(std::floor(yaw_angle / yaw_bin_size_rad));
        yaw_bin = ((yaw_bin % yaw_bins_) + yaw_bins_) % yaw_bins_;

        int pitch_bin =
            static_cast<int>(std::floor((pitch_angle - min_pitch_rad_) / pitch_bin_size_rad));
        pitch_bin = std::clamp(pitch_bin, 0, pitch_bins_ - 1);

        const uint16_t dist_cm = static_cast<uint16_t>(std::round(dist_horiz_m * 100.0f));

        auto& cell = grid_[static_cast<size_t>(yaw_bin)][static_cast<size_t>(pitch_bin)];
        if (dist_cm < cell.min_distance_cm) {
            cell.min_distance_cm = dist_cm;
        }
    }
}

void LivoxMid360Processor::ComputeMinDistances(std::array<uint16_t, 72>& min_distances) const {
    std::fill(min_distances.begin(), min_distances.end(), kNoObstacle);

    const int yaw_limit = std::min(yaw_bins_, 72);
    for (int yaw_bin = 0; yaw_bin < yaw_limit; ++yaw_bin) {
        uint16_t best = kNoObstacle;
        for (int pitch_bin = 0; pitch_bin < pitch_bins_; ++pitch_bin) {
            const uint16_t d =
                grid_[static_cast<size_t>(yaw_bin)][static_cast<size_t>(pitch_bin)].min_distance_cm;
            if (d < best) {
                best = d;
            }
        }
        min_distances[static_cast<size_t>(yaw_bin)] = best;
    }
}

}  // namespace px4_navigation
