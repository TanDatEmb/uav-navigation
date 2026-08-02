#include "fast_lio_ros/ros_propagated_odometry_publisher.hpp"

#include "fast_lio_ros/qos_profiles.hpp"
#include "fast_lio_ros/ros_odometry_serializer.hpp"
#include "fast_lio_ros/ros_time_converter.hpp"

namespace uav::nav::lio {

RosPropagatedOdometryPublisher::RosPropagatedOdometryPublisher(
    rclcpp::Node& node, const RosParameters& parameters)
    : parameters_(parameters),
      publisher_(node.create_publisher<nav_msgs::msg::Odometry>(
          "/lio/odometry_propagated", QosProfiles::estimatorOutput())) {}

void RosPropagatedOdometryPublisher::setBaseLinkConverter(
    std::shared_ptr<const BaseLinkStateConverter> converter) {
  base_link_converter_ = std::move(converter);
}

void RosPropagatedOdometryPublisher::publish(
    const KinematicStateEstimate& estimate) {
  if (!base_link_converter_) {
    return;
  }
  const auto converted = base_link_converter_->convert(
      estimate.estimate, estimate.angular_velocity_imu_rad_s);
  if (!converted.ok()) {
    return;
  }
  const auto odometry = RosOdometrySerializer::serialize(converted.value(), parameters_);
  if (odometry.ok()) {
    publisher_->publish(odometry.value());
  }
}

}  // namespace uav::nav::lio
