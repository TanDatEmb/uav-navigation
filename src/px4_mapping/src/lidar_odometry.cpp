#include <px4_mapping/lidar_odometry.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <vector>

#include <px4_ros2_utils/frame/transform.hpp>
#include <px4_ros2_utils/parameter/param_utils.hpp>
#include <px4_ros2_utils/px4/topic.hpp>
#include <px4_ros2_utils/qos/sensor.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

namespace px4_mapping {

namespace {

inline Eigen::Quaterniond SmallAngleQuaternion(const Eigen::Vector3d &delta_angle_rad) {
    const double angle = delta_angle_rad.norm();
    if (angle < 1e-12) {
        return Eigen::Quaterniond::Identity();
    }
    return Eigen::Quaterniond(Eigen::AngleAxisd(angle, delta_angle_rad / angle));
}

}  // namespace

LidarOdometry::LidarOdometry(const rclcpp::NodeOptions &options)
    : rclcpp::Node("lidar_odometry", options), timesync_(this->get_clock()) {
    LoadParameters();

    const auto sensor_qos = px4_ros2_utils::qos::sensor_qos();
    const auto px4_qos = px4_ros2_utils::qos::sensor_qos(20);
    const auto lio_pub_qos = rclcpp::QoS(20).reliable();

    sub_cloud_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        input_cloud_topic_, sensor_qos,
        std::bind(&LidarOdometry::CloudCallback, this, std::placeholders::_1));

    sub_px4_odom_ = this->create_subscription<px4_msgs::msg::VehicleOdometry>(
        px4_odom_topic_, px4_qos,
        std::bind(&LidarOdometry::Px4OdomCallback, this, std::placeholders::_1));

    sub_vehicle_imu_ = this->create_subscription<px4_msgs::msg::VehicleImu>(
        vehicle_imu_topic_, px4_qos,
        std::bind(&LidarOdometry::VehicleImuCallback, this, std::placeholders::_1));

    sub_timesync_status_ = this->create_subscription<px4_msgs::msg::TimesyncStatus>(
        px4_ros2_utils::px4::topic::topic_name<px4_msgs::msg::TimesyncStatus>(
            "/fmu/out/timesync_status"),
        px4_ros2_utils::qos::sensor_qos(5), [this](px4_msgs::msg::TimesyncStatus::SharedPtr msg) {
            timesync_.update(msg);
        });

    pub_processed_cloud_ =
        this->create_publisher<sensor_msgs::msg::PointCloud2>(output_cloud_topic_, lio_pub_qos);
    pub_lio_odom_ =
        this->create_publisher<nav_msgs::msg::Odometry>(output_odom_topic_, lio_pub_qos);

    RCLCPP_INFO(this->get_logger(),
                "lidar_odometry started: in_cloud=%s px4_odom=%s imu=%s out_cloud=%s out_odom=%s "
                "point_filter_num=%d blind=%.2f",
                input_cloud_topic_.c_str(), px4_odom_topic_.c_str(), vehicle_imu_topic_.c_str(),
                output_cloud_topic_.c_str(), output_odom_topic_.c_str(), point_filter_num_,
                blind_m_);
}

void LidarOdometry::LoadParameters() {
    const auto load_parameter = [this]<typename T>(const std::string &name, const T &default_value,
                                                   const std::string &description) {
        return px4_ros2_utils::parameter::declare_and_get(
            *this, name, default_value, px4_ros2_utils::parameter::descriptor(description));
    };

    input_cloud_topic_ = load_parameter("input_cloud_topic", std::string("/lidar_360/points"),
                                        "Input LiDAR point cloud topic in sensor FLU frame.");
    px4_odom_topic_ = load_parameter("px4_odom_topic", std::string("/fmu/out/vehicle_odometry"),
                                     "Input PX4 vehicle odometry in map_ned frame.");
    vehicle_imu_topic_ = load_parameter("vehicle_imu_topic", std::string("/fmu/out/vehicle_imu"),
                                        "Input PX4 vehicle IMU topic for incremental propagation.");
    output_cloud_topic_ = load_parameter("output_cloud_topic", std::string("/localization/cloud"),
                                         "Output processed cloud in camera_init frame.");
    output_odom_topic_ = load_parameter("output_odom_topic", std::string("/localization/odometry"),
                                        "Output odometry in camera_init frame.");
    output_frame_id_ = load_parameter("output_frame_id", std::string("camera_init"),
                                      "Output frame id for processed cloud and odometry.");
    output_child_frame_id_ = load_parameter("output_child_frame_id", std::string("base_link"),
                                            "Output child frame id for odometry.");

    point_filter_num_ = load_parameter("point_filter_num", 3,
                                       "Take one out of N points for lightweight preprocessing.");
    blind_m_ = load_parameter("blind_m", 0.5,
                              "Ignore points within this radius from sensor origin (metres).");
    use_imu_fusion_ = load_parameter("use_imu_fusion", true,
                                     "Use PX4 VehicleImu delta-angle/delta-velocity to propagate "
                                     "odometry between PX4 odom updates.");

    // Default pose/twist covariance for /odometry. Until localization
    // a real covariance, expose sensible values so downstream consumers
    // (RViz, navigation planner) do not treat LIO as "infinite trust" and so
    // that EKF2 can weight the EV pose against other sensors.
    // - pose: ~2 cm position, ~0.3 deg orientation diagonal.
    // - twist: ~5 cm/s linear, ~0.5 deg/s angular diagonal.
    pose_covariance_.fill(0.0);
    pose_covariance_[0] = 0.04;   // x
    pose_covariance_[7] = 0.04;   // y
    pose_covariance_[14] = 0.09;  // z
    pose_covariance_[21] = 0.0025;
    pose_covariance_[28] = 0.0025;
    pose_covariance_[35] = 0.0025;

    twist_covariance_.fill(0.0);
    twist_covariance_[0] = 0.05;
    twist_covariance_[7] = 0.05;
    twist_covariance_[14] = 0.10;
    twist_covariance_[21] = 0.0008;
    twist_covariance_[28] = 0.0008;
    twist_covariance_[35] = 0.0008;

    max_velocity_for_estimate_mps_ =
        load_parameter("max_velocity_for_estimate_mps", 2.0,
                       "Upper bound for finite-difference velocity (m/s). "
                       "Velocities above this are clamped to avoid spikes from stalled topics.");

    if (point_filter_num_ <= 0) {
        RCLCPP_WARN(this->get_logger(), "point_filter_num=%d invalid, using 1", point_filter_num_);
        point_filter_num_ = 1;
    }
    if (blind_m_ < 0.0) {
        RCLCPP_WARN(this->get_logger(), "blind_m=%.3f invalid, using 0.0", blind_m_);
        blind_m_ = 0.0;
    }
}

void LidarOdometry::Px4OdomCallback(const px4_msgs::msg::VehicleOdometry::SharedPtr msg) {
    OdomSample sample;
    const auto sample_stamp = timesync_.toROS(msg->timestamp_sample);
    if (!sample_stamp) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                             "[lidar_odometry] Dropping PX4 odometry until TimesyncStatus is valid "
                             "(timestamp_sample=%lu)",
                             static_cast<unsigned long>(msg->timestamp_sample));
        return;
    }
    sample.stamp = *sample_stamp;
    sample.position_ned = Eigen::Vector3d(static_cast<double>(msg->position[0]),
                                          static_cast<double>(msg->position[1]),
                                          static_cast<double>(msg->position[2]));
    sample.orientation_ned =
        Eigen::Quaterniond(static_cast<double>(msg->q[0]), static_cast<double>(msg->q[1]),
                           static_cast<double>(msg->q[2]), static_cast<double>(msg->q[3]))
            .normalized();
    sample.velocity_ned = Eigen::Vector3d(static_cast<double>(msg->velocity[0]),
                                          static_cast<double>(msg->velocity[1]),
                                          static_cast<double>(msg->velocity[2]));
    sample.valid = true;

    std::lock_guard<std::mutex> lock(state_mutex_);
    latest_odom_ = sample;
    fused_odom_ = sample;
    fused_odom_valid_ = true;

    if (!origin_initialized_) {
        origin_position_ned_ = sample.position_ned;
        origin_orientation_enu_ =
            px4_ros2_utils::frame::orientation_ned_to_enu(sample.orientation_ned);
        origin_initialized_ = true;
        RCLCPP_INFO(this->get_logger(),
                    "lidar_odometry origin initialized at NED (%.3f, %.3f, %.3f)",
                    origin_position_ned_.x(), origin_position_ned_.y(), origin_position_ned_.z());
    }
}

void LidarOdometry::VehicleImuCallback(const px4_msgs::msg::VehicleImu::SharedPtr msg) {
    if (!use_imu_fusion_) {
        return;
    }

    const auto imu_stamp = timesync_.toROS(msg->timestamp_sample);
    if (!imu_stamp) {
        RCLCPP_WARN_THROTTLE(
            this->get_logger(), *this->get_clock(), 5000,
            "[lidar_odometry] Ignoring IMU timestamp until TimesyncStatus is valid "
            "(timestamp_sample=%lu)",
            static_cast<unsigned long>(msg->timestamp_sample));
        return;
    }

    std::lock_guard<std::mutex> lock(state_mutex_);
    if (!fused_odom_valid_) {
        return;
    }

    const double dt_s = static_cast<double>(msg->delta_angle_dt) * 1e-6;
    if (dt_s <= 0.0 || dt_s > 0.1) {
        return;
    }

    const Eigen::Vector3d delta_angle_body(static_cast<double>(msg->delta_angle[0]),
                                           static_cast<double>(msg->delta_angle[1]),
                                           static_cast<double>(msg->delta_angle[2]));
    const Eigen::Quaterniond dq = SmallAngleQuaternion(delta_angle_body);
    fused_odom_.orientation_ned = (fused_odom_.orientation_ned * dq).normalized();

    const Eigen::Vector3d delta_velocity_body(static_cast<double>(msg->delta_velocity[0]),
                                              static_cast<double>(msg->delta_velocity[1]),
                                              static_cast<double>(msg->delta_velocity[2]));
    const Eigen::Vector3d delta_velocity_ned = fused_odom_.orientation_ned * delta_velocity_body;
    fused_odom_.velocity_ned += delta_velocity_ned;
    fused_odom_.position_ned += fused_odom_.velocity_ned * dt_s;
    fused_odom_.stamp = *imu_stamp;
    fused_odom_.valid = true;
}

void LidarOdometry::CloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
    OdomSample odom_sample;
    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        if (!origin_initialized_ || !latest_odom_.valid) {
            return;
        }
        if (use_imu_fusion_ && fused_odom_valid_ && fused_odom_.valid) {
            odom_sample = fused_odom_;
        } else {
            odom_sample = latest_odom_;
        }
    }

    sensor_msgs::msg::PointCloud2 out_cloud;
    if (!BuildProcessedCloud(*msg, out_cloud, odom_sample)) {
        return;
    }

    pub_processed_cloud_->publish(out_cloud);
    PublishLioOdometry(odom_sample, out_cloud.header.stamp);
}

bool LidarOdometry::BuildProcessedCloud(const sensor_msgs::msg::PointCloud2 &input,
                                        sensor_msgs::msg::PointCloud2 &output,
                                        const OdomSample &odom_sample) {
    const double blind_sq = blind_m_ * blind_m_;

    const Eigen::Quaterniond q_enu =
        px4_ros2_utils::frame::orientation_ned_to_enu(odom_sample.orientation_ned);
    const Eigen::Quaterniond q_rel_enu = origin_orientation_enu_.conjugate() * q_enu;

    std::vector<Eigen::Vector3f> accepted_points;
    accepted_points.reserve(static_cast<size_t>(input.width * input.height) /
                            static_cast<size_t>(point_filter_num_));

    sensor_msgs::PointCloud2ConstIterator<float> iter_x(input, "x");
    sensor_msgs::PointCloud2ConstIterator<float> iter_y(input, "y");
    sensor_msgs::PointCloud2ConstIterator<float> iter_z(input, "z");

    const size_t n = static_cast<size_t>(input.width * input.height);
    for (size_t i = 0; i < n; ++i, ++iter_x, ++iter_y, ++iter_z) {
        if ((i % static_cast<size_t>(point_filter_num_)) != 0U) {
            continue;
        }

        const Eigen::Vector3d p_flu(static_cast<double>(*iter_x), static_cast<double>(*iter_y),
                                    static_cast<double>(*iter_z));
        if (!p_flu.allFinite()) {
            continue;
        }

        const Eigen::Vector3d p_frd = px4_ros2_utils::frame::flu_to_frd(p_flu);
        if (p_frd.squaredNorm() <= blind_sq) {
            continue;
        }

        const Eigen::Vector3d p_world_ned =
            odom_sample.position_ned + odom_sample.orientation_ned * p_frd;
        const Eigen::Vector3d p_rel_ned = p_world_ned - origin_position_ned_;
        const Eigen::Vector3d p_rel_enu = px4_ros2_utils::frame::ned_to_enu(p_rel_ned);

        accepted_points.emplace_back(static_cast<float>(p_rel_enu.x()),
                                     static_cast<float>(p_rel_enu.y()),
                                     static_cast<float>(p_rel_enu.z()));
    }

    if (accepted_points.empty()) {
        return false;
    }

    output.header = input.header;
    output.header.frame_id = output_frame_id_;
    output.height = 1;
    output.width = static_cast<uint32_t>(accepted_points.size());

    sensor_msgs::PointCloud2Modifier modifier(output);
    modifier.setPointCloud2FieldsByString(1, "xyz");
    modifier.resize(accepted_points.size());

    sensor_msgs::PointCloud2Iterator<float> out_x(output, "x");
    sensor_msgs::PointCloud2Iterator<float> out_y(output, "y");
    sensor_msgs::PointCloud2Iterator<float> out_z(output, "z");

    for (const auto &p : accepted_points) {
        *out_x = p.x();
        *out_y = p.y();
        *out_z = p.z();
        ++out_x;
        ++out_y;
        ++out_z;
    }

    (void)q_rel_enu;
    return true;
}

void LidarOdometry::PublishLioOdometry(const OdomSample &odom_sample, const rclcpp::Time &stamp) {
    nav_msgs::msg::Odometry msg;
    msg.header.stamp = stamp;
    msg.header.frame_id = output_frame_id_;
    msg.child_frame_id = output_child_frame_id_;

    const Eigen::Vector3d p_rel_ned = odom_sample.position_ned - origin_position_ned_;
    const Eigen::Vector3d p_rel_enu = px4_ros2_utils::frame::ned_to_enu(p_rel_ned);

    const Eigen::Quaterniond q_enu =
        px4_ros2_utils::frame::orientation_ned_to_enu(odom_sample.orientation_ned);
    const Eigen::Quaterniond q_rel_enu = origin_orientation_enu_.conjugate() * q_enu;

    // Twist selection priority:
    //   1) Use PX4-derived velocity when available (it is the only LIO input
    //      that reflects real-time body motion in our adapter).
    //   2) Fall back to finite-difference on the LIO pose when velocity
    //      was zero or unavailable. This keeps /odometry consumers from
    //      seeing a literal zero twist during hover.
    Eigen::Vector3d v_enu;
    if (odom_sample.velocity_ned.norm() > 1e-3) {
        v_enu = px4_ros2_utils::frame::ned_to_enu(odom_sample.velocity_ned);
    } else {
        Eigen::Vector3d v_finite = Eigen::Vector3d::Zero();
        {
            std::lock_guard<std::mutex> lock(state_mutex_);
            const double dt_s = (stamp - last_publish_stamp_).seconds();
            if (last_publish_valid_ && dt_s > 1e-3) {
                v_finite = (p_rel_ned - last_publish_position_ned_) / dt_s;
                // Clamp to avoid spikes when the topic stalled.
                const double speed = v_finite.norm();
                if (speed > max_velocity_for_estimate_mps_) {
                    v_finite *= max_velocity_for_estimate_mps_ / speed;
                }
            }
            last_publish_position_ned_ = p_rel_ned;
            last_publish_stamp_ = stamp;
            last_publish_valid_ = true;
        }
        v_enu = px4_ros2_utils::frame::ned_to_enu(v_finite);
    }

    msg.pose.pose.position.x = p_rel_enu.x();
    msg.pose.pose.position.y = p_rel_enu.y();
    msg.pose.pose.position.z = p_rel_enu.z();

    msg.pose.pose.orientation.w = q_rel_enu.w();
    msg.pose.pose.orientation.x = q_rel_enu.x();
    msg.pose.pose.orientation.y = q_rel_enu.y();
    msg.pose.pose.orientation.z = q_rel_enu.z();

    // nav_msgs/Odometry requires twist in child_frame_id (base_link). Convert
    // world ENU linear velocity into body FLU using the inverse body attitude.
    const Eigen::Vector3d v_body_flu = q_rel_enu.conjugate() * v_enu;

    msg.twist.twist.linear.x = v_body_flu.x();
    msg.twist.twist.linear.y = v_body_flu.y();
    msg.twist.twist.linear.z = v_body_flu.z();

    // Same orientation in LIO-derived odometry; angular velocity in body frame
    // would require an extra finite-difference on q_rel_enu, intentionally
    // left zero here to keep this node dependency-light.
    msg.twist.twist.angular.x = 0.0;
    msg.twist.twist.angular.y = 0.0;
    msg.twist.twist.angular.z = 0.0;

    for (size_t i = 0; i < pose_covariance_.size(); ++i) {
        msg.pose.covariance[i] = pose_covariance_[i];
        msg.twist.covariance[i] = twist_covariance_[i];
    }

    pub_lio_odom_->publish(msg);
}

}  // namespace px4_mapping
