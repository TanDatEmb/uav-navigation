#include "fast_lio_ros/ros_propagated_odometry_publisher.hpp"

#include <chrono>
#include <limits>

#include "fast_lio_ros/qos_profiles.hpp"
#include "fast_lio_ros/ros_odometry_serializer.hpp"
#include "fast_lio_ros/ros_time_converter.hpp"

namespace uav::nav::lio {

RosPropagatedOdometryPublisher::RosPropagatedOdometryPublisher(
    rclcpp::Node& node, const RosParameters& parameters,
    std::shared_ptr<CovarianceProjectionRuntime> covariance_runtime,
    std::shared_ptr<LioPublicFrameGeneration> public_frame_generation)
    : parameters_(parameters),
      publisher_(node.create_publisher<navigation_contracts::msg::PropagatedOdometry>(
          "/lio/odometry_propagated", QosProfiles::estimatorOutput())),
      covariance_runtime_(std::move(covariance_runtime)),
      public_frame_generation_(std::move(public_frame_generation)) {}

void RosPropagatedOdometryPublisher::setBaseLinkConverter(
    std::shared_ptr<const BaseLinkStateConverter> converter) {
  if (converter) {
    covariance_projector_.emplace(converter->baseToImu());
  }
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
  if (!covariance_projector_.has_value() || !covariance_runtime_) {
    return;
  }
  BaseLinkCovarianceProjectionDiagnostics projection_diagnostics;
  const auto projection_started = std::chrono::steady_clock::now();
  const auto covariance = covariance_projector_->project(
      estimate, converted.value(), &projection_diagnostics);
  const auto projection_elapsed = std::chrono::duration_cast<std::chrono::microseconds>(
      std::chrono::steady_clock::now() - projection_started).count();
  covariance_runtime_->record(projection_diagnostics, projection_elapsed);
  if (!covariance.ok()) {
    return;
  }
  const auto odometry = RosOdometrySerializer::serialize(
      converted.value(), covariance.value(), parameters_);
  if (!odometry.ok() || !public_frame_generation_) return;
  const auto public_frame = public_frame_generation_->snapshot();
  if (!public_frame.valid || public_frame.generation == 0U ||
      publication_sequence_ == std::numeric_limits<std::uint64_t>::max()) {
    return;
  }
  navigation_contracts::msg::PropagatedOdometry message;
  message.odometry = odometry.value();
  message.localization_epoch = public_frame.generation;
  message.sequence = ++publication_sequence_;
  publisher_->publish(std::move(message));
}

}  // namespace uav::nav::lio
