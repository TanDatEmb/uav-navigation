#include "fast_lio_ros/ros_odometry_serializer.hpp"

#include "fast_lio_ros/ros_time_converter.hpp"

namespace uav::nav::lio {

Result<nav_msgs::msg::Odometry> RosOdometrySerializer::serialize(
    const RigidBodyState& state, const RosParameters& parameters) {
  if (!state.allFinite() || !state.angular_velocity_body_rad_s.has_value()) {
    return Status(StatusCode::kNumericalFailure,
                  "base-link odometry requires a finite angular velocity");
  }
  if (state.reference_frame != FrameId(parameters.odom_frame) ||
      state.body_frame != FrameId(parameters.base_frame)) {
    return Status(StatusCode::kFrameMismatch,
                  "converted odometry frames do not match ROS parameters");
  }

  nav_msgs::msg::Odometry odometry;
  odometry.header.stamp = RosTimeConverter::toRos(state.time);
  odometry.header.frame_id = parameters.odom_frame;
  odometry.child_frame_id = parameters.base_frame;
  odometry.pose.pose.position.x = state.position_reference_body_m.x();
  odometry.pose.pose.position.y = state.position_reference_body_m.y();
  odometry.pose.pose.position.z = state.position_reference_body_m.z();
  odometry.pose.pose.orientation.x = state.orientation_reference_body.x();
  odometry.pose.pose.orientation.y = state.orientation_reference_body.y();
  odometry.pose.pose.orientation.z = state.orientation_reference_body.z();
  odometry.pose.pose.orientation.w = state.orientation_reference_body.w();
  odometry.twist.twist.linear.x = state.linear_velocity_body_m_s.x();
  odometry.twist.twist.linear.y = state.linear_velocity_body_m_s.y();
  odometry.twist.twist.linear.z = state.linear_velocity_body_m_s.z();
  odometry.twist.twist.angular.x = state.angular_velocity_body_rad_s->x();
  odometry.twist.twist.angular.y = state.angular_velocity_body_rad_s->y();
  odometry.twist.twist.angular.z = state.angular_velocity_body_rad_s->z();
  // Covariance projection is intentionally not part of P0.3.  The zero
  // arrays are a documented transitional "unavailable" representation.
  return odometry;
}

}  // namespace uav::nav::lio
