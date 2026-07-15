#ifndef FAST_LIO_UTILS_HPP_
#define FAST_LIO_UTILS_HPP_

#include "fast_lio/commons.hpp"

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <std_msgs/msg/header.hpp>

#include <cstddef>
#include <deque>

// Livox optional
#ifdef LIVOX_ROS2_FOUND
#include <livox_ros_driver2/msg/custom_msg.hpp>
#endif

namespace fast_lio {
namespace utils {

// Convert Livox CustomMsg to PCL point cloud (if available)
#ifdef LIVOX_ROS2_FOUND
CloudType::Ptr livoxToPCL(const livox_ros_driver2::msg::CustomMsg::SharedPtr msg, int filter_num,
                          double min_range, double max_range);
#endif

// Convert ROS IMU to IMUData
IMUData rosToIMU(const sensor_msgs::msg::Imu::SharedPtr msg);

// Get timestamp in seconds from ROS message header
double getSec(const std_msgs::msg::Header& header);

// Convert seconds to ROS time
builtin_interfaces::msg::Time getTime(double sec);

// Convert SE3 to geometry_msgs::Transform
geometry_msgs::msg::Transform SE3ToTransform(const SE3d& T);

// Convert SE3 to nav_msgs::Odometry
void SE3ToOdometry(const SE3d& T, const std::string& frame_id, const std::string& child_frame_id,
                   nav_msgs::msg::Odometry& odom);

// Map [rotation, position] blocks from the 15-state covariance into ROS pose order
// [position, orientation].
void setOdometryPoseCovariance(const Eigen::Matrix<double, 15, 15>& state_covariance,
                               nav_msgs::msg::Odometry& odom);

// Append a pose while bounding both retained history and serialized Path size.
void appendBoundedPose(nav_msgs::msg::Path& path, const geometry_msgs::msg::PoseStamped& pose,
                       std::size_t max_poses);

// Keep the newest entries in a sensor queue and return how many were dropped.
template <typename T>
std::size_t trimDequeFront(std::deque<T>& buffer, std::size_t max_size) {
    if (buffer.size() <= max_size) {
        return 0;
    }

    const std::size_t dropped = buffer.size() - max_size;
    buffer.erase(buffer.begin(), buffer.begin() + static_cast<std::ptrdiff_t>(dropped));
    return dropped;
}

}  // namespace utils
}  // namespace fast_lio

#endif  // FAST_LIO_UTILS_HPP_
