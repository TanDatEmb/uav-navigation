// lio_px4_alignment.cpp
// Bridge FAST-LIO2 odometry to PX4 external odometry

#include "px4_mapping/lio_px4_alignment.hpp"

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

namespace px4_mapping {

LioPx4Alignment::LioPx4Alignment()
    : Node("lio_px4_alignment") {

    RCLCPP_INFO(this->get_logger(), "LIO-PX4 Alignment Node Starting...");

    loadParameters();
    computeStaticTransform();

    // TF buffer
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // Subscriber
    lio_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        lio_topic_, 10,
        std::bind(&LioPx4Alignment::lioCallback, this, std::placeholders::_1));

    // Publisher
    px4_pub_ = this->create_publisher<px4_msgs::msg::VehicleOdometry>(
        px4_topic_, 10);

    RCLCPP_INFO(this->get_logger(), "Subscribing to: %s", lio_topic_.c_str());
    RCLCPP_INFO(this->get_logger(), "Publishing to: %s", px4_topic_.c_str());
}

void LioPx4Alignment::loadParameters() {
    this->declare_parameter<std::string>("lio_topic", "/lio/odometry");
    this->declare_parameter<std::string>("px4_topic", "/fmu/in/vehicle_visual_odometry");
    this->declare_parameter<std::string>("lio_frame_id", "lio_world");
    this->declare_parameter<std::string>("px4_frame_id", "map");
    this->declare_parameter<bool>("use_tf_lookup", false);

    this->get_parameter("lio_topic", lio_topic_);
    this->get_parameter("px4_topic", px4_topic_);
    this->get_parameter("lio_frame_id", lio_frame_id_);
    this->get_parameter("px4_frame_id", px4_frame_id_);
    this->get_parameter("use_tf_lookup", use_tf_lookup_);
}

void LioPx4Alignment::computeStaticTransform() {
    // ENU to NED transformation:
    // NED = T * ENU
    // where T = Rx(180°) followed by coordinate swap
    //
    // Result: [x_ned, y_ned, z_ned] = [x_enu, -y_enu, -z_enu]
    //
    // Rotation matrix for ENU → NED:
    // R = [1  0  0
    //      0 -1  0
    //      0  0 -1]
    //
    // This is equivalent to 180° rotation around X axis

    R_lio_px4_ = Eigen::Matrix3d::Identity();
    R_lio_px4_(1, 1) = -1.0;  // Flip Y
    R_lio_px4_(2, 2) = -1.0;  // Flip Z

    // Translation is typically zero (same origin)
    t_lio_px4_ = Eigen::Vector3d::Zero();

    // Jacobian for covariance transformation (6x6)
    // Position part: R_lio_px4
    // Rotation part: R_lio_px4 (for Euler angles, approximately)
    J_pose_transform_.setZero();
    J_pose_transform_.block<3, 3>(0, 0) = R_lio_px4_;
    J_pose_transform_.block<3, 3>(3, 3) = R_lio_px4_;

    RCLCPP_INFO(this->get_logger(),
        "Static transform computed: ENU → NED");
    RCLCPP_INFO(this->get_logger(),
        "  Position: [x, -y, -z]");
    RCLCPP_INFO(this->get_logger(),
        "  Velocity: [vx, -vy, -vz]");
}

px4_msgs::msg::VehicleOdometry LioPx4Alignment::transformToPX4(
    const nav_msgs::msg::Odometry& lio_msg) {

    px4_msgs::msg::VehicleOdometry px4_msg;

    // Timestamp
    px4_msg.timestamp = this->now().nanoseconds() / 1000;  // microseconds

    // Frame IDs
    // PX4 frame convention:
    //   0: NED earth-fixed frame
    //   1: FRD body-fixed frame
    px4_msg.pose_frame = px4_msgs::msg::VehicleOdometry::POSE_FRAME_NED;
    px4_msg.velocity_frame = px4_msgs::msg::VehicleOdometry::VELOCITY_FRAME_NED;

    // Position: ENU → NED [x, -y, -z]
    Eigen::Vector3d p_lio(lio_msg.pose.pose.position.x,
                          lio_msg.pose.pose.position.y,
                          lio_msg.pose.pose.position.z);

    Eigen::Vector3d p_px4 = R_lio_px4_ * p_lio + t_lio_px4_;

    px4_msg.position[0] = static_cast<float>(p_px4(0));  // North
    px4_msg.position[1] = static_cast<float>(p_px4(1));  // East
    px4_msg.position[2] = static_cast<float>(p_px4(2));  // Down

    // Orientation: ENU → NED
    // Convert quaternion to rotation matrix, transform, convert back
    tf2::Quaternion q_lio(
        lio_msg.pose.pose.orientation.x,
        lio_msg.pose.pose.orientation.y,
        lio_msg.pose.pose.orientation.z,
        lio_msg.pose.pose.orientation.w);

    tf2::Matrix3x3 R_lio(q_lio);

    // Apply ENU→NED rotation: R_px4 = R_lio_px4 * R_lio * R_lio_px4^T
    // Actually for orientation: R_ned = R_enu_to_ned * R_enu
    double roll_lio, pitch_lio, yaw_lio;
    R_lio.getRPY(roll_lio, pitch_lio, yaw_lio);

    // Transform Euler angles: roll same, pitch and yaw negated
    double r_px4_angle = roll_lio;
    double p_px4_angle = -pitch_lio;
    double y_px4_angle = -yaw_lio;

    // Convert back to quaternion
    tf2::Quaternion q_px4;
    q_px4.setRPY(r_px4_angle, p_px4_angle, y_px4_angle);

    px4_msg.q[0] = static_cast<float>(q_px4.w());
    px4_msg.q[1] = static_cast<float>(q_px4.x());
    px4_msg.q[2] = static_cast<float>(q_px4.y());
    px4_msg.q[3] = static_cast<float>(q_px4.z());

    // Velocity: ENU → NED [vx, -vy, -vz]
    Eigen::Vector3d v_lio(lio_msg.twist.twist.linear.x,
                          lio_msg.twist.twist.linear.y,
                          lio_msg.twist.twist.linear.z);

    Eigen::Vector3d v_px4 = R_lio_px4_ * v_lio;

    px4_msg.velocity[0] = static_cast<float>(v_px4(0));
    px4_msg.velocity[1] = static_cast<float>(v_px4(1));
    px4_msg.velocity[2] = static_cast<float>(v_px4(2));

    // Angular velocity (body frame, should already be correct)
    px4_msg.angular_velocity[0] = static_cast<float>(lio_msg.twist.twist.angular.x);
    px4_msg.angular_velocity[1] = static_cast<float>(-lio_msg.twist.twist.angular.y);
    px4_msg.angular_velocity[2] = static_cast<float>(-lio_msg.twist.twist.angular.z);

    // Position covariance: transform from LIO to PX4
    // Extract 3x3 position covariance
    Eigen::Matrix3d P_pos_lio;
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            P_pos_lio(i, j) = lio_msg.pose.covariance[i * 6 + j];
        }
    }

    // Transform: P_px4 = R * P_lio * R^T
    Eigen::Matrix3d P_pos_px4 = R_lio_px4_ * P_pos_lio * R_lio_px4_.transpose();

    // Set position covariance in PX4 message
    // PX4 uses row-major order for upper triangle
    px4_msg.position_variance[0] = static_cast<float>(P_pos_px4(0, 0));
    px4_msg.position_variance[1] = static_cast<float>(P_pos_px4(1, 1));
    px4_msg.position_variance[2] = static_cast<float>(P_pos_px4(2, 2));

    // Orientation variance (simplified)
    px4_msg.orientation_variance[0] = static_cast<float>(lio_msg.pose.covariance[21]);  // roll
    px4_msg.orientation_variance[1] = static_cast<float>(lio_msg.pose.covariance[28]);  // pitch
    px4_msg.orientation_variance[2] = static_cast<float>(lio_msg.pose.covariance[35]);  // yaw

    // Velocity covariance
    Eigen::Matrix3d P_vel_lio;
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            P_vel_lio(i, j) = lio_msg.twist.covariance[i * 6 + j];
        }
    }

    Eigen::Matrix3d P_vel_px4 = R_lio_px4_ * P_vel_lio * R_lio_px4_.transpose();

    px4_msg.velocity_variance[0] = static_cast<float>(P_vel_px4(0, 0));
    px4_msg.velocity_variance[1] = static_cast<float>(P_vel_px4(1, 1));
    px4_msg.velocity_variance[2] = static_cast<float>(P_vel_px4(2, 2));

    return px4_msg;
}

void LioPx4Alignment::lioCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    // Transform LIO message to PX4 format
    auto px4_msg = transformToPX4(*msg);

    // Publish to PX4
    px4_pub_->publish(px4_msg);

    // Debug logging (throttled)
    static int count = 0;
    if (++count % 50 == 0) {
        RCLCPP_INFO(this->get_logger(),
            "Published odometry [%d]: pos=[%.2f, %.2f, %.2f]",
            count,
            px4_msg.position[0],
            px4_msg.position[1],
            px4_msg.position[2]);
    }
}

}  // namespace px4_mapping

#include <rclcpp/rclcpp.hpp>

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<px4_mapping::LioPx4Alignment>());
    rclcpp::shutdown();
    return 0;
}
