#include "fast_lio_ros/ros_propagated_odometry_publisher.hpp"

#include <chrono>
#include <limits>
#include <stdexcept>

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
      public_frame_generation_(std::move(public_frame_generation)) {
  // FastLioNode declares this before its strict override audit. Standalone
  // publisher component tests construct this class without ParameterLoader.
  const bool trace_enabled = node.has_parameter(
      "diagnostics.state_transport_trace_enabled")
      ? node.get_parameter("diagnostics.state_transport_trace_enabled").as_bool()
      : node.declare_parameter<bool>("diagnostics.state_transport_trace_enabled", false);
  if (trace_enabled) {
    if (!node.get_parameter("use_sim_time").as_bool()) {
      throw std::invalid_argument("state transport trace is SITL/test only");
    }
    timing_publisher_ = node.create_publisher<
        navigation_contracts::msg::OdometryTransportTrace>(
        "/lio/odometry_transport_trace", rclcpp::QoS(256).best_effort());
  }
}

void RosPropagatedOdometryPublisher::setBaseLinkConverter(
    std::shared_ptr<const BaseLinkStateConverter> converter) {
  if (converter) {
    covariance_projector_.emplace(converter->baseToImu());
  }
  base_link_converter_ = std::move(converter);
}

void RosPropagatedOdometryPublisher::publish(
    const KinematicStateEstimate& estimate,
    const WorkerPublicationWitness& witness) {
  const auto publisher_enter_steady_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::steady_clock::now().time_since_epoch()).count();
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
  const auto publication_sequence = ++publication_sequence_;
  message.sequence = publication_sequence;
  const auto source_stamp_ros_ns = static_cast<std::int64_t>(
      message.odometry.header.stamp.sec) * 1'000'000'000LL +
      static_cast<std::int64_t>(message.odometry.header.stamp.nanosec);
  const auto publish_call_steady_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::steady_clock::now().time_since_epoch()).count();
  publisher_->publish(std::move(message));
  const auto publisher_exit_steady_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::steady_clock::now().time_since_epoch()).count();
  if (timing_publisher_) {
    navigation_contracts::msg::OdometryTransportTrace trace;
    trace.phase = trace.PRODUCER_PUBLISHED;
    trace.disposition = trace.NOT_APPLICABLE;
    trace.localization_epoch = public_frame.generation;
    trace.sequence = publication_sequence;
    trace.source_stamp_ros_ns = source_stamp_ros_ns;
    trace.expected_publish_source_ns = witness.expected_publish_source_ns;
    trace.last_published_source_ns = witness.last_published_source_ns;
    trace.worker_estimate_ready_steady_ns = witness.estimate_ready_steady_ns;
    trace.publisher_enter_steady_ns = publisher_enter_steady_ns;
    trace.publisher_publish_call_steady_ns = publish_call_steady_ns;
    trace.publisher_exit_steady_ns = publisher_exit_steady_ns;
    try {
      timing_publisher_->publish(std::move(trace));
    } catch (...) {
      // Diagnostic transport cannot suspend the estimator publication path.
    }
  }
}

}  // namespace uav::nav::lio
