#include "fast_lio_ros/ros_transform_publisher.hpp"

#include <geometry_msgs/msg/transform_stamped.hpp>

#include "fast_lio_ros/ros_time_converter.hpp"

namespace uav::nav::lio {

RosTransformPublisher::RosTransformPublisher(rclcpp::Node& node, RosParameters parameters)
    : parameters_(std::move(parameters)), broadcaster_(node) {}

void RosTransformPublisher::publish(const ProcessResult& result) {
  if (!result.hasCorrectedOutput()) {
    return;
  }
  const StateEstimate& corrected = *result.corrected_estimate;
  geometry_msgs::msg::TransformStamped transform;
  transform.header.stamp = RosTimeConverter::toRos(corrected.time);
  transform.header.frame_id = parameters_.odom_frame;
  transform.child_frame_id = parameters_.imu_frame;
  const auto& state = corrected.state;
  const auto& position = state.position_odom_imu_m();
  const auto& orientation = state.orientation_odom_imu();
  transform.transform.translation.x = position.x();
  transform.transform.translation.y = position.y();
  transform.transform.translation.z = position.z();
  transform.transform.rotation.x = orientation.x();
  transform.transform.rotation.y = orientation.y();
  transform.transform.rotation.z = orientation.z();
  transform.transform.rotation.w = orientation.w();
  broadcaster_.sendTransform(transform);
}

}  // namespace uav::nav::lio
