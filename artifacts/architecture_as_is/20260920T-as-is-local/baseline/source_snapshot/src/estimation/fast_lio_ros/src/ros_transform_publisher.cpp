#include "fast_lio_ros/ros_transform_publisher.hpp"

#include <geometry_msgs/msg/transform_stamped.hpp>

#include <string>

#include "fast_lio_ros/ros_time_converter.hpp"

namespace uav::nav::lio {

RosTransformPublisher::RosTransformPublisher(rclcpp::Node& node)
    : broadcaster_(node) {}

void RosTransformPublisher::setBaseLinkConverter(
    std::shared_ptr<const BaseLinkStateConverter> converter) {
  std::lock_guard lock(mutex_);
  base_link_converter_ = std::move(converter);
}

namespace {

void publishConverted(tf2_ros::TransformBroadcaster& broadcaster,
                      const RigidBodyState& state) {
  geometry_msgs::msg::TransformStamped transform;
  transform.header.stamp = RosTimeConverter::toRos(state.time);
  transform.header.frame_id = std::string(state.reference_frame.name());
  transform.child_frame_id = std::string(state.body_frame.name());
  transform.transform.translation.x = state.position_reference_body_m.x();
  transform.transform.translation.y = state.position_reference_body_m.y();
  transform.transform.translation.z = state.position_reference_body_m.z();
  transform.transform.rotation.x = state.orientation_reference_body.x();
  transform.transform.rotation.y = state.orientation_reference_body.y();
  transform.transform.rotation.z = state.orientation_reference_body.z();
  transform.transform.rotation.w = state.orientation_reference_body.w();
  broadcaster.sendTransform(transform);
}

}  // namespace

void RosTransformPublisher::publishPropagated(
    const KinematicStateEstimate& estimate) {
  publishKinematic(estimate);
}

void RosTransformPublisher::publishKinematic(
    const KinematicStateEstimate& estimate) {
  std::shared_ptr<const BaseLinkStateConverter> converter;
  {
    std::lock_guard lock(mutex_);
    converter = base_link_converter_;
  }
  if (!converter) {
    return;
  }
  const auto converted = converter->convert(
      estimate.estimate, estimate.angular_velocity_imu_rad_s);
  if (!converted.ok()) {
    std::lock_guard lock(mutex_);
    ++diagnostics_.conversion_failure_count;
    return;
  }
  std::lock_guard lock(mutex_);
  if (last_published_time_.has_value() &&
      converted.value().time.nanoseconds() <=
          last_published_time_->nanoseconds()) {
    ++diagnostics_.timestamp_suppressed_count;
    return;
  }
  publishConverted(broadcaster_, converted.value());
  last_published_time_ = converted.value().time;
  ++diagnostics_.publication_count;
}

RosTransformPublisher::Diagnostics RosTransformPublisher::diagnostics() const {
  std::lock_guard lock(mutex_);
  return diagnostics_;
}

}  // namespace uav::nav::lio
