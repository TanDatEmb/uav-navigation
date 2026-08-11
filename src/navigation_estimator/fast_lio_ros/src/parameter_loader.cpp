#include "fast_lio_ros/parameter_loader.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <filesystem>
#include <stdexcept>
#include <vector>

namespace uav::nav::lio {
namespace {

LivoxTimestampPolicy parseTimestampPolicy(std::string_view value) {
  if (value == "require_header_match") {
    return LivoxTimestampPolicy::kRequireHeaderMatchesTimebase;
  }
  if (value == "timebase_authoritative") {
    return LivoxTimestampPolicy::kTimebaseAuthoritative;
  }
  throw std::invalid_argument("unsupported timing.livox_timestamp_policy");
}

InitialStatePriorSource parsePriorSource(std::string_view value) {
  if (value == "zero") return InitialStatePriorSource::kZero;
  if (value == "fixed") return InitialStatePriorSource::kFixed;
  if (value == "topic") return InitialStatePriorSource::kTopic;
  throw std::invalid_argument("unsupported initial_prior.source");
}

InitialStatePriorContext parsePriorContext(std::string_view value) {
  if (value == "ground_startup") return InitialStatePriorContext::kGroundStartup;
  if (value == "in_flight_reinitialization") {
    return InitialStatePriorContext::kInFlightReinitialization;
  }
  throw std::invalid_argument("unsupported initial_prior.context");
}

PriorAttitudeMode parsePriorAttitude(std::string_view value) {
  if (value == "none") return PriorAttitudeMode::kNone;
  if (value == "yaw_only") return PriorAttitudeMode::kYawOnly;
  if (value == "full") return PriorAttitudeMode::kFull;
  throw std::invalid_argument("unsupported initial_prior.components.attitude");
}

InitialPriorFallback parsePriorFallback(std::string_view value) {
  if (value == "reject") return InitialPriorFallback::kReject;
  if (value == "zero") return InitialPriorFallback::kZero;
  if (value == "fixed") return InitialPriorFallback::kFixed;
  throw std::invalid_argument("unsupported initial_prior.ground_fallback");
}

void requireCanonicalFields(rclcpp::Node& node) {
  static constexpr std::array<std::string_view, 33> kRequired{
      "frames.odom",
      "frames.base",
      "frames.imu",
      "frames.lidar",
      "input.lidar_topic",
      "input.imu_topic",
      "input.lidar_frame",
      "input.imu_frame",
      "input.lidar_message_type",
      "timing.lidar_mode",
      "timing.clock_domain",
      "timing.livox_timestamp_policy",
      "initial_prior.source",
      "initial_prior.context",
      "initial_prior.source_frame",
      "initial_prior.source_frame_transform",
      "initial_prior.components.position",
      "initial_prior.components.velocity",
      "initial_prior.components.attitude",
      "timing.max_imu_gap_ns",
      "timing.reject_timestamp_regression",
      "tracking.maximum_recoverable_imu_gap_ns",
      "tracking.recovery_confirmation_updates",
      "tracking.discontinuity_covariance_inflation",
      "extrinsic.estimate_online",
      "extrinsic.translation_imu_lidar",
      "extrinsic.rotation_imu_lidar_xyzw",
      "initialization.minimum_imu_samples",
      "initialization.require_stationary",
      "preprocessing.minimum_range_m",
      "preprocessing.maximum_range_m",
      "preprocessing.scan_voxel_size_m",
      "mapping.registration_map.voxel_size_m",
  };
  const auto& overrides =
      node.get_node_parameters_interface()->get_parameter_overrides();
  for (const auto name : kRequired) {
    if (!overrides.contains(std::string{name})) {
      throw std::invalid_argument("canonical estimator YAML missing required field: " +
                                  std::string{name});
    }
  }
  if (!overrides.contains("registration.maximum_iterations")) {
    throw std::invalid_argument(
        "canonical estimator YAML missing required field: "
        "registration.maximum_iterations");
  }
}

}  // namespace

ClockDomain parseClockDomain(std::string_view value) {
  if (value == "ros_time") {
    return ClockDomain::kRosTime;
  }
  if (value == "simulation_time") {
    return ClockDomain::kSimulationTime;
  }
  if (value == "sensor_time") {
    return ClockDomain::kSensorTime;
  }
  if (value == "system_time") {
    return ClockDomain::kSystemTime;
  }
  if (value == "steady_time") {
    return ClockDomain::kSteadyTime;
  }
  throw std::invalid_argument("unsupported timing.clock_domain");
}

RosParameters ParameterLoader::declareAndLoad(rclcpp::Node& node) {
  RosParameters result;
  result.odom_frame = node.declare_parameter("frames.odom", "lio_odom");
  result.base_frame = node.declare_parameter("frames.base", "base_link");
  result.imu_frame = node.declare_parameter("frames.imu", "livox_imu_frame");
  result.lidar_frame = node.declare_parameter("frames.lidar", "livox_frame");
  result.lidar_topic = node.declare_parameter("input.lidar_topic", "/lidar/points");
  result.imu_topic = node.declare_parameter("input.imu_topic", "/lidar/imu");
  result.lidar_input_frame =
      node.declare_parameter("input.lidar_frame", result.lidar_frame);
  result.imu_input_frame =
      node.declare_parameter("input.imu_frame", result.imu_frame);
  result.lidar_message_type = node.declare_parameter("input.lidar_message_type", "pointcloud2");
  result.lidar_timing_mode = node.declare_parameter("timing.lidar_mode", "simultaneous_scan");
  result.point_time_field =
      node.declare_parameter("input.point_time.field", "time");
  result.point_time_encoding = node.declare_parameter(
      "input.point_time.encoding", "uint32_relative_nanoseconds");
  result.point_time_scan_reference =
      node.declare_parameter("input.point_time.scan_reference", "header_stamp");
  result.maximum_scan_duration_ns = node.declare_parameter<std::int64_t>(
      "input.point_time.maximum_scan_duration_ns", 200'000'000);
  result.maximum_header_offset_ns = node.declare_parameter<std::int64_t>(
      "input.point_time.maximum_header_offset_ns", 200'000'000);
  result.maximum_boundary_overlap_ns = node.declare_parameter<std::int64_t>(
      "input.point_time.maximum_boundary_overlap_ns", 0);
  result.minimum_points_after_overlap_trim = node.declare_parameter<std::int64_t>(
      "input.point_time.minimum_points_after_overlap_trim", 1);
  result.input_clock_domain =
      node.declare_parameter("timing.clock_domain", "ros_time");
  result.initial_prior_source =
      node.declare_parameter("initial_prior.source", "zero");
  result.initial_prior_context =
      node.declare_parameter("initial_prior.context", "ground_startup");
  result.initial_prior_source_frame = node.declare_parameter(
      "initial_prior.source_frame", result.odom_frame);
  result.initial_prior_source_frame_transform = node.declare_parameter(
      "initial_prior.source_frame_transform", "same_frame");
  result.initial_prior_position_enabled = node.declare_parameter(
      "initial_prior.components.position", true);
  result.initial_prior_velocity_enabled = node.declare_parameter(
      "initial_prior.components.velocity", true);
  result.initial_prior_attitude =
      node.declare_parameter("initial_prior.components.attitude", "yaw_only");
  result.initial_prior_topic = node.declare_parameter(
      "initial_prior.topic", "/initial_state_prior");
  result.initial_prior_wait_timeout_ns = node.declare_parameter<std::int64_t>(
      "initial_prior.topic_wait_timeout_ns", 2'000'000'000);
  result.initial_prior_maximum_age_ns = node.declare_parameter<std::int64_t>(
      "initial_prior.maximum_topic_prior_age_ns", 500'000'000);
  result.initial_prior_fallback = node.declare_parameter(
      "initial_prior.ground_fallback", "zero");
  result.initial_prior_maximum_full_tilt_disagreement_rad = node.declare_parameter(
      "initial_prior.maximum_full_attitude_tilt_disagreement_rad",
      0.17453292519943295);
  const auto fixed_position = node.declare_parameter<std::vector<double>>(
      "initial_prior.fixed.position_m", {0.0, 0.0, 0.0});
  const auto fixed_orientation = node.declare_parameter<std::vector<double>>(
      "initial_prior.fixed.orientation_xyzw", {0.0, 0.0, 0.0, 1.0});
  const auto fixed_linear_velocity = node.declare_parameter<std::vector<double>>(
      "initial_prior.fixed.linear_velocity_m_s", {0.0, 0.0, 0.0});
  const auto fixed_angular_velocity = node.declare_parameter<std::vector<double>>(
      "initial_prior.fixed.angular_velocity_rad_s", {0.0, 0.0, 0.0});
  if (fixed_position.size() != 3U || fixed_orientation.size() != 4U ||
      fixed_linear_velocity.size() != 3U || fixed_angular_velocity.size() != 3U) {
    throw std::invalid_argument("initial_prior fixed arrays have invalid sizes");
  }
  std::copy(fixed_position.begin(), fixed_position.end(), result.initial_prior_fixed_position_m.begin());
  std::copy(fixed_orientation.begin(), fixed_orientation.end(), result.initial_prior_fixed_orientation_xyzw.begin());
  std::copy(fixed_linear_velocity.begin(), fixed_linear_velocity.end(), result.initial_prior_fixed_linear_velocity_m_s.begin());
  std::copy(fixed_angular_velocity.begin(), fixed_angular_velocity.end(), result.initial_prior_fixed_angular_velocity_rad_s.begin());
  result.livox_timestamp_policy = node.declare_parameter(
      "timing.livox_timestamp_policy", "require_header_match");
  result.maximum_imu_gap_ns =
      node.declare_parameter<std::int64_t>("timing.max_imu_gap_ns", 20'000'000);
  result.maximum_recoverable_imu_gap_ns =
      node.declare_parameter<std::int64_t>(
          "tracking.maximum_recoverable_imu_gap_ns", 50'000'000);
  result.recovery_confirmation_updates =
      node.declare_parameter<std::int64_t>(
          "tracking.recovery_confirmation_updates", 3);
  result.discontinuity_covariance_inflation =
      node.declare_parameter<double>(
          "tracking.discontinuity_covariance_inflation", 10.0);
  result.reject_timestamp_regression =
      node.declare_parameter("timing.reject_timestamp_regression", true);
  result.estimate_extrinsic_online = node.declare_parameter("extrinsic.estimate_online", false);
  const auto translation = node.declare_parameter<std::vector<double>>(
      "extrinsic.translation_imu_lidar", {0.0, 0.0, 0.0});
  const auto rotation = node.declare_parameter<std::vector<double>>(
      "extrinsic.rotation_imu_lidar_xyzw", {0.0, 0.0, 0.0, 1.0});
  if (translation.size() != 3U || rotation.size() != 4U) {
    throw std::invalid_argument("extrinsic arrays must have sizes 3 and 4");
  }
  std::copy(translation.begin(), translation.end(), result.translation_imu_lidar_m.begin());
  std::copy(rotation.begin(), rotation.end(), result.rotation_imu_lidar_xyzw.begin());
  result.minimum_imu_samples =
      node.declare_parameter<std::int64_t>("initialization.minimum_imu_samples", 200);
  result.require_stationary = node.declare_parameter("initialization.require_stationary", true);
  result.minimum_range_m = node.declare_parameter("preprocessing.minimum_range_m", 0.1);
  result.maximum_range_m = node.declare_parameter("preprocessing.maximum_range_m", 40.0);
  result.scan_voxel_size_m =
      node.declare_parameter("preprocessing.scan_voxel_size_m", 0.2);
  result.registration_map_voxel_size_m =
      node.declare_parameter("mapping.registration_map.voxel_size_m", 0.2);
  const auto local_half_extent = node.declare_parameter<std::vector<double>>(
      "mapping.local_map.half_extent_m", {50.0, 50.0, 25.0});
  if (local_half_extent.size() != 3U) {
    throw std::invalid_argument("mapping.local_map.half_extent_m must have size 3");
  }
  std::copy(local_half_extent.begin(), local_half_extent.end(),
            result.local_map_half_extent_m.begin());
  result.local_map_crop_trigger_distance_m = node.declare_parameter(
      "mapping.local_map.crop_trigger_distance_m", 5.0);
  result.local_map_absolute_point_guard = node.declare_parameter<std::int64_t>(
      "mapping.local_map.absolute_map_point_guard", 250000);
  result.dynamic_filter_enabled =
      node.declare_parameter("mapping.dynamic_filter.enabled", false);
  result.maximum_registration_iterations =
      node.declare_parameter<std::int64_t>("registration.maximum_iterations", 4);
  result.correspondence_parallel_threads =
      node.declare_parameter<std::int64_t>(
          "registration.correspondence_parallel_threads", 3);
  result.publish_registered_points =
      node.declare_parameter("output.publish_registered_points", false);
  result.imu_queue_capacity =
      node.declare_parameter<std::int64_t>("runtime.imu_queue_capacity", 4096);
  result.lidar_queue_capacity =
      node.declare_parameter<std::int64_t>("runtime.lidar_queue_capacity", 8);
  result.maximum_processing_lag_ms = node.declare_parameter<std::int64_t>(
      "runtime.maximum_processing_lag_ms", 200);
  result.overload_policy =
      node.declare_parameter("runtime.overload_policy", "fail");
  result.input_qos_reliability =
      node.declare_parameter("input.qos_reliability", "best_effort");
  result.propagated_odometry_enabled =
      node.declare_parameter("propagated_odometry.enabled", true);
  result.propagated_odometry_publish_rate_hz =
      node.declare_parameter("propagated_odometry.publish_rate_hz", 50.0);
  result.propagated_odometry_imu_ingress_capacity = node.declare_parameter<std::int64_t>(
      "propagated_odometry.imu_ingress_capacity", 4096);
  result.propagated_odometry_imu_history_duration_ns =
      node.declare_parameter<std::int64_t>(
          "propagated_odometry.imu_history_duration_ns", 1'000'000'000);
  result.propagated_odometry_maximum_correction_age_ns =
      node.declare_parameter<std::int64_t>(
          "propagated_odometry.maximum_correction_age_ns", 250'000'000);
  const auto& overrides =
      node.get_node_parameters_interface()->get_parameter_overrides();
  for (const auto& [name, unused_value] : overrides) {
    static_cast<void>(unused_value);
    if (name != "use_sim_time" && !node.has_parameter(name)) {
      throw std::invalid_argument("unknown estimator parameter override: " +
                                  name);
    }
  }
  validate(result);
  return result;
}

EstimatorProfile makeEstimatorProfile(const RosParameters& parameters) {
  ParameterLoader::validate(parameters);
  EstimatorProfile profile;
  profile.lidar_topic = parameters.lidar_topic;
  profile.imu_topic = parameters.imu_topic;
  profile.lidar_input_frame = parameters.lidar_input_frame;
  profile.imu_input_frame = parameters.imu_input_frame;
  profile.lidar_message_type = parameters.lidar_message_type;
  profile.lidar_timing_mode = parameters.lidar_timing_mode;
  profile.point_time.field = parameters.point_time_field;
  profile.point_time.encoding =
      parameters.point_time_encoding == "float64_absolute_nanoseconds"
          ? PointTimeEncoding::kFloat64AbsoluteNanoseconds
          : PointTimeEncoding::kUint32RelativeNanoseconds;
  profile.point_time.scan_reference =
      parameters.point_time_scan_reference == "minimum_point_time"
          ? ScanReference::kMinimumPointTime
          : ScanReference::kHeaderStamp;
  profile.point_time.maximum_scan_duration_ns =
      parameters.maximum_scan_duration_ns;
  profile.point_time.maximum_header_offset_ns =
      parameters.maximum_header_offset_ns;
  profile.point_time.maximum_boundary_overlap_ns =
      parameters.maximum_boundary_overlap_ns;
  profile.point_time.minimum_points_after_overlap_trim =
      static_cast<std::size_t>(parameters.minimum_points_after_overlap_trim);
  profile.point_time.reject_scan_timestamp_regression =
      parameters.reject_timestamp_regression;
  profile.clock_domain = parseClockDomain(parameters.input_clock_domain);
  profile.timestamp_policy =
      parseTimestampPolicy(parameters.livox_timestamp_policy);
  auto& config = profile.estimator;
  config.synchronization.maximum_imu_gap_ns = parameters.maximum_imu_gap_ns;
  config.tracking.maximum_recoverable_imu_gap_ns =
      parameters.maximum_recoverable_imu_gap_ns;
  config.tracking.recovery_confirmation_updates =
      static_cast<std::size_t>(parameters.recovery_confirmation_updates);
  config.tracking.discontinuity_covariance_inflation =
      parameters.discontinuity_covariance_inflation;
  config.deskew.mode = parameters.lidar_timing_mode == "per_point"
                           ? DeskewMode::kPerPoint
                           : DeskewMode::kSimultaneousScan;
  config.preprocessing.point_filter.minimum_range_m =
      parameters.minimum_range_m;
  config.preprocessing.point_filter.maximum_range_m =
      parameters.maximum_range_m;
  config.preprocessing.voxel_filter.voxel_size_m =
      parameters.scan_voxel_size_m;
  config.registration_map.voxel_size_m =
      parameters.registration_map_voxel_size_m;
  config.local_map.half_extent_m = {
      parameters.local_map_half_extent_m[0],
      parameters.local_map_half_extent_m[1],
      parameters.local_map_half_extent_m[2]};
  config.local_map.crop_trigger_distance_m =
      parameters.local_map_crop_trigger_distance_m;
  config.local_map.absolute_map_point_guard = static_cast<std::size_t>(
      parameters.local_map_absolute_point_guard);
  config.dynamic_filter.enabled = parameters.dynamic_filter_enabled;
  config.initialization.minimum_imu_samples =
      static_cast<std::size_t>(parameters.minimum_imu_samples);
  config.initialization.require_stationary = parameters.require_stationary;
  config.initial_prior.source = parsePriorSource(parameters.initial_prior_source);
  config.initial_prior.context = parsePriorContext(parameters.initial_prior_context);
  config.initial_prior.mask.position = parameters.initial_prior_position_enabled;
  config.initial_prior.mask.velocity = parameters.initial_prior_velocity_enabled;
  config.initial_prior.mask.attitude = parsePriorAttitude(parameters.initial_prior_attitude);
  config.initial_prior.topic_wait_timeout_ns = parameters.initial_prior_wait_timeout_ns;
  config.initial_prior.maximum_topic_prior_age_ns = parameters.initial_prior_maximum_age_ns;
  config.initial_prior.ground_fallback = parsePriorFallback(parameters.initial_prior_fallback);
  config.initial_prior.maximum_full_attitude_tilt_disagreement_rad =
      parameters.initial_prior_maximum_full_tilt_disagreement_rad;
  config.initial_prior.fixed_prior.source = InitialStatePriorSource::kFixed;
  config.initial_prior.fixed_prior.context = config.initial_prior.context;
  config.initial_prior.fixed_prior.mask = config.initial_prior.mask;
  config.initial_prior.fixed_prior.reference_frame = FrameId(parameters.odom_frame);
  config.initial_prior.fixed_prior.body_frame = FrameId(parameters.base_frame);
  config.initial_prior.fixed_prior.position_odom_base_m = {
      parameters.initial_prior_fixed_position_m[0], parameters.initial_prior_fixed_position_m[1],
      parameters.initial_prior_fixed_position_m[2]};
  const auto& fixed_orientation = parameters.initial_prior_fixed_orientation_xyzw;
  config.initial_prior.fixed_prior.orientation_odom_base =
      Eigen::Quaterniond{fixed_orientation[3], fixed_orientation[0], fixed_orientation[1],
                         fixed_orientation[2]};
  config.initial_prior.fixed_prior.linear_velocity_base_m_s = Eigen::Vector3d{
      parameters.initial_prior_fixed_linear_velocity_m_s[0],
      parameters.initial_prior_fixed_linear_velocity_m_s[1],
      parameters.initial_prior_fixed_linear_velocity_m_s[2]};
  config.initial_prior.fixed_prior.angular_velocity_base_rad_s = Eigen::Vector3d{
      parameters.initial_prior_fixed_angular_velocity_rad_s[0],
      parameters.initial_prior_fixed_angular_velocity_rad_s[1],
      parameters.initial_prior_fixed_angular_velocity_rad_s[2]};
  config.ikfom.maximum_iterations =
      static_cast<std::size_t>(parameters.maximum_registration_iterations);
  config.residual_builder.parallel_thread_count = static_cast<std::size_t>(
      parameters.correspondence_parallel_threads);
  config.extrinsic.estimate_online = parameters.estimate_extrinsic_online;
  config.extrinsic.translation_imu_lidar_m = {
      parameters.translation_imu_lidar_m[0],
      parameters.translation_imu_lidar_m[1],
      parameters.translation_imu_lidar_m[2]};
  const auto& quaternion = parameters.rotation_imu_lidar_xyzw;
  config.extrinsic.rotation_imu_lidar =
      Eigen::Quaterniond{quaternion[3], quaternion[0], quaternion[1],
                         quaternion[2]}
          .normalized();
  return profile;
}

EstimatorProfile loadCanonicalEstimatorProfile(const std::string& config_path) {
  if (!std::filesystem::is_regular_file(config_path)) {
    throw std::invalid_argument("canonical estimator YAML does not exist: " +
                                config_path);
  }
  try {
    rclcpp::NodeOptions options;
    options.arguments(
        {"--ros-args", "--params-file",
         std::filesystem::absolute(config_path).lexically_normal().string()});
    rclcpp::Node node("fast_lio", options);
    requireCanonicalFields(node);
    auto parameters = ParameterLoader::declareAndLoad(node);
    ParameterLoader::validateCanonicalFrameContract(parameters);
    return makeEstimatorProfile(parameters);
  } catch (const std::invalid_argument&) {
    throw;
  } catch (const std::exception& error) {
    throw std::invalid_argument("invalid canonical estimator YAML " +
                                config_path + ": " + error.what());
  }
}

void ParameterLoader::validate(const RosParameters& p) {
  if (p.odom_frame.empty() || p.base_frame.empty() || p.imu_frame.empty() ||
      p.lidar_frame.empty() || p.lidar_input_frame.empty() ||
      p.imu_input_frame.empty()) {
    throw std::invalid_argument("frame names must not be empty");
  }
  if (p.lidar_topic.empty() || p.imu_topic.empty()) {
    throw std::invalid_argument("input topic names must not be empty");
  }
  if (p.odom_frame == p.base_frame || p.imu_frame == p.lidar_frame) {
    throw std::invalid_argument("configured frames must identify distinct frames");
  }
  if (p.initial_prior_source_frame.empty() ||
      (p.initial_prior_source_frame != p.odom_frame &&
       p.initial_prior_source_frame_transform != "startup_coincident") ||
      (p.initial_prior_source_frame == p.odom_frame &&
       p.initial_prior_source_frame_transform != "same_frame") ||
      (p.initial_prior_source_frame != p.odom_frame &&
       p.initial_prior_context != "ground_startup")) {
    throw std::invalid_argument(
        "initial prior source frame needs an explicit same_frame or ground-startup "
        "startup_coincident transform");
  }
  if (p.lidar_message_type != "pointcloud2" && p.lidar_message_type != "livox_custom") {
    throw std::invalid_argument("unsupported input.lidar_message_type");
  }
  if (p.lidar_timing_mode != "simultaneous_scan" && p.lidar_timing_mode != "per_point") {
    throw std::invalid_argument("production lidar timing must be simultaneous_scan or per_point");
  }
  if (p.point_time_field.empty() ||
      (p.point_time_encoding != "uint32_relative_nanoseconds" &&
       p.point_time_encoding != "float64_absolute_nanoseconds") ||
      (p.point_time_scan_reference != "header_stamp" &&
       p.point_time_scan_reference != "minimum_point_time") ||
      p.maximum_scan_duration_ns <= 0 || p.maximum_header_offset_ns < 0 ||
      p.maximum_boundary_overlap_ns < 0 ||
      p.minimum_points_after_overlap_trim <= 0) {
    throw std::invalid_argument("invalid input.point_time configuration");
  }
  static_cast<void>(parseClockDomain(p.input_clock_domain));
  if (p.livox_timestamp_policy != "require_header_match" &&
      p.livox_timestamp_policy != "timebase_authoritative") {
    throw std::invalid_argument(
        "unsupported timing.livox_timestamp_policy");
  }
  if (p.lidar_message_type == "livox_custom" &&
      p.lidar_timing_mode != "per_point") {
    throw std::invalid_argument(
        "livox_custom requires timing.lidar_mode=per_point");
  }
  const double quaternion_norm =
      std::sqrt(p.rotation_imu_lidar_xyzw[0] * p.rotation_imu_lidar_xyzw[0] +
                p.rotation_imu_lidar_xyzw[1] * p.rotation_imu_lidar_xyzw[1] +
                p.rotation_imu_lidar_xyzw[2] * p.rotation_imu_lidar_xyzw[2] +
                p.rotation_imu_lidar_xyzw[3] * p.rotation_imu_lidar_xyzw[3]);
  const bool finite_translation =
      std::all_of(p.translation_imu_lidar_m.begin(),
                  p.translation_imu_lidar_m.end(),
                  [](double value) { return std::isfinite(value); });
  if (!finite_translation || !std::isfinite(quaternion_norm) ||
      std::abs(quaternion_norm - 1.0) > 1e-6) {
    throw std::invalid_argument("extrinsic rotation quaternion is invalid");
  }
  if (p.estimate_extrinsic_online) {
    throw std::invalid_argument("runtime configurations require extrinsic.estimate_online=false");
  }
  if (!(p.scan_voxel_size_m > 0.0) ||
      !std::isfinite(p.scan_voxel_size_m)) {
    throw std::invalid_argument(
        "preprocessing.scan_voxel_size_m must be finite and positive");
  }
  if (!(p.registration_map_voxel_size_m > 0.0) ||
      !std::isfinite(p.registration_map_voxel_size_m)) {
    throw std::invalid_argument(
        "mapping.registration_map.voxel_size_m must be finite and positive");
  }
  if (p.maximum_imu_gap_ns <= 0 ||
      p.maximum_recoverable_imu_gap_ns <= 0 ||
      p.recovery_confirmation_updates <= 0 || p.minimum_imu_samples <= 0 ||
      p.maximum_registration_iterations <= 0 ||
      p.correspondence_parallel_threads <= 0 ||
      p.correspondence_parallel_threads > 32 || p.minimum_range_m < 0.0 ||
      p.maximum_range_m <= p.minimum_range_m ||
      !std::isfinite(p.minimum_range_m) ||
      !std::isfinite(p.maximum_range_m)) {
    throw std::invalid_argument("numeric estimator parameter is out of range");
  }
  if (p.maximum_recoverable_imu_gap_ns < p.maximum_imu_gap_ns) {
    throw std::invalid_argument(
        "tracking.maximum_recoverable_imu_gap_ns must be greater than or "
        "equal to timing.max_imu_gap_ns");
  }
  if (!std::isfinite(p.discontinuity_covariance_inflation) ||
      p.discontinuity_covariance_inflation < 1.0 ||
      p.discontinuity_covariance_inflation > 1000.0) {
    throw std::invalid_argument(
        "tracking.discontinuity_covariance_inflation must be finite and in "
        "[1, 1000]");
  }
  if (p.imu_queue_capacity <= 0 || p.lidar_queue_capacity <= 0 ||
      p.maximum_processing_lag_ms <= 0 || p.overload_policy != "fail") {
    throw std::invalid_argument(
        "runtime queues must be positive and overload_policy must be fail");
  }
  if (p.input_qos_reliability != "best_effort" &&
      p.input_qos_reliability != "reliable") {
    throw std::invalid_argument(
        "input.qos_reliability must be best_effort or reliable");
  }
  if (!p.propagated_odometry_enabled ||
      !(p.propagated_odometry_publish_rate_hz > 0.0) ||
      !std::isfinite(p.propagated_odometry_publish_rate_hz) ||
      p.propagated_odometry_imu_ingress_capacity <= 0 ||
      p.propagated_odometry_imu_history_duration_ns <= 0 ||
      p.propagated_odometry_maximum_correction_age_ns <= 0 ||
      p.propagated_odometry_imu_history_duration_ns <=
          p.propagated_odometry_maximum_correction_age_ns) {
    throw std::invalid_argument(
        "propagated_odometry.enabled is mandatory and its configuration is invalid");
  }
  const Eigen::Map<const Eigen::Vector3d> local_half_extent(
      p.local_map_half_extent_m.data());
  if (!local_half_extent.allFinite() ||
      (local_half_extent.array() <= 0.0).any() ||
      !(p.local_map_crop_trigger_distance_m > 0.0) ||
      !std::isfinite(p.local_map_crop_trigger_distance_m) ||
      p.local_map_absolute_point_guard <= 0) {
    throw std::invalid_argument("invalid mapping.local_map configuration");
  }
}

void ParameterLoader::validateCanonicalFrameContract(const RosParameters& p) {
  if (p.lidar_frame != "livox_frame" ||
      p.imu_frame != "livox_imu_frame" ||
      p.lidar_input_frame != "livox_frame" ||
      p.imu_input_frame != "livox_imu_frame") {
    throw std::invalid_argument(
        "runtime configurations require livox_frame and livox_imu_frame at both "
        "the estimator and input boundaries");
  }
  if (p.imu_frame == p.lidar_frame ||
      p.imu_input_frame == p.lidar_input_frame) {
    throw std::invalid_argument(
        "runtime configurations require distinct LiDAR and IMU frames at both "
        "the estimator and input boundaries");
  }
  if (p.imu_frame != p.imu_input_frame || p.lidar_frame != p.lidar_input_frame) {
    throw std::invalid_argument(
        "runtime configurations require input frames to match their estimator "
        "sensor frames");
  }
}

}  // namespace uav::nav::lio
