#include "fast_lio_ros/ros_propagated_odometry_publisher.hpp"

#include <cmath>

#include "fast_lio_ros/qos_profiles.hpp"
#include "fast_lio_ros/ros_time_converter.hpp"

namespace uav::nav::lio {

RosPropagatedOdometryPublisher::RosPropagatedOdometryPublisher(
    rclcpp::Node& node, const RosParameters& parameters)
    : parameters_(parameters),
      publisher_(node.create_publisher<nav_msgs::msg::Odometry>(
          "/lio/odometry_propagated", QosProfiles::estimatorOutput())),
      publish_period_ns_(static_cast<std::int64_t>(
          std::llround(1e9 / parameters.propagated_odometry_publish_rate_hz))) {}

void RosPropagatedOdometryPublisher::onImuEstimate(
    const std::optional<StateEstimate>& estimate) {
  if (!estimate.has_value()) {
    ++publication_skip_count_;
    return;
  }
  if (last_published_time_.has_value() &&
      estimate->time.nanoseconds() <= last_published_time_->nanoseconds()) {
    ++publication_skip_count_;
    return;
  }
  if (!next_publish_deadline_.has_value()) {
    next_publish_deadline_ = estimate->time;
  }
  if (estimate->time.nanoseconds() < next_publish_deadline_->nanoseconds()) {
    ++publication_skip_count_;
    return;
  }

  nav_msgs::msg::Odometry odometry;
  odometry.header.stamp = RosTimeConverter::toRos(estimate->time);
  odometry.header.frame_id = parameters_.odom_frame;
  odometry.child_frame_id = parameters_.imu_frame;
  const auto& state = estimate->state;
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
  last_published_time_ = estimate->time;
  ++publication_count_;
  do {
    next_publish_deadline_ = Timestamp(
        next_publish_deadline_->nanoseconds() + publish_period_ns_,
        next_publish_deadline_->clock_domain());
  } while (next_publish_deadline_->nanoseconds() <=
           estimate->time.nanoseconds());
}

std::uint64_t RosPropagatedOdometryPublisher::publicationCount() const noexcept {
  return publication_count_;
}
std::uint64_t RosPropagatedOdometryPublisher::publicationSkipCount() const noexcept {
  return publication_skip_count_;
}
std::optional<Timestamp>
RosPropagatedOdometryPublisher::lastPublishedTime() const noexcept {
  return last_published_time_;
}
std::optional<Timestamp>
RosPropagatedOdometryPublisher::nextPublishDeadline() const noexcept {
  return next_publish_deadline_;
}

}  // namespace uav::nav::lio
