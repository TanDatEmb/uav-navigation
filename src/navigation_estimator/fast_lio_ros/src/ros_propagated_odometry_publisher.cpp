#include "fast_lio_ros/ros_propagated_odometry_publisher.hpp"

#include "fast_lio_ros/qos_profiles.hpp"
#include "fast_lio_ros/ros_time_converter.hpp"

namespace uav::nav::lio {

RosPropagatedOdometryPublisher::RosPropagatedOdometryPublisher(
    rclcpp::Node& node, const RosParameters& parameters)
    : parameters_(parameters),
      publisher_(node.create_publisher<nav_msgs::msg::Odometry>(
          "/lio/odometry_propagated", QosProfiles::estimatorOutput())) {}

void RosPropagatedOdometryPublisher::publish(const StateEstimate& estimate) {
  nav_msgs::msg::Odometry odometry;
  odometry.header.stamp = RosTimeConverter::toRos(estimate.time);
  odometry.header.frame_id = parameters_.odom_frame;
  odometry.child_frame_id = parameters_.imu_frame;
  const auto& state = estimate.state;
  const auto& position = state.position_odom_imu_m();
  const auto& orientation = state.orientation_odom_imu();
  odometry.pose.pose.position.x = position.x();
  odometry.pose.pose.position.y = position.y();
  odometry.pose.pose.position.z = position.z();
  odometry.pose.pose.orientation.x = orientation.x();
  odometry.pose.pose.orientation.y = orientation.y();
  odometry.pose.pose.orientation.z = orientation.z();
  odometry.pose.pose.orientation.w = orientation.w();
  const Eigen::Vector3d velocity_imu =
      orientation.conjugate() * state.velocity_odom_imu_m_s();
  odometry.twist.twist.linear.x = velocity_imu.x();
  odometry.twist.twist.linear.y = velocity_imu.y();
  odometry.twist.twist.linear.z = velocity_imu.z();
  odometry.pose.covariance[0] = -1.0;
  odometry.twist.covariance[0] = -1.0;
  publisher_->publish(odometry);
}

}  // namespace uav::nav::lio
