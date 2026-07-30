#include "fast_lio_ros/parameter_loader.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <memory>
#include <openssl/evp.h>
#include <sstream>
#include <stdexcept>
#include <vector>

namespace uav::nav::lio {
namespace {

std::string parameterFileFromArguments(const std::vector<std::string>& arguments) {
  for (std::size_t index = 0; index + 1 < arguments.size(); ++index) {
    if (arguments[index] == "--params-file") {
      return std::filesystem::absolute(arguments[index + 1]).lexically_normal();
    }
  }
  return {};
}

LivoxTimestampPolicy parseTimestampPolicy(std::string_view value) {
  if (value == "require_header_match") {
    return LivoxTimestampPolicy::kRequireHeaderMatchesTimebase;
  }
  if (value == "timebase_authoritative") {
    return LivoxTimestampPolicy::kTimebaseAuthoritative;
  }
  throw std::invalid_argument("unsupported timing.livox_timestamp_policy");
}

void requireCanonicalFields(rclcpp::Node& node) {
  static constexpr std::array<std::string_view, 22> kRequired{
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
      "timing.max_imu_gap_ns",
      "timing.reject_timestamp_regression",
      "extrinsic.estimate_online",
      "extrinsic.translation_imu_lidar",
      "extrinsic.rotation_imu_lidar_xyzw",
      "initialization.minimum_imu_samples",
      "initialization.require_stationary",
      "preprocessing.minimum_range_m",
      "preprocessing.maximum_range_m",
      "preprocessing.voxel_size_m",
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

std::string sha256File(const std::string& path) {
  std::ifstream stream(path, std::ios::binary);
  if (!stream) {
    throw std::invalid_argument("unable to open estimator config: " + path);
  }
  EVP_MD_CTX* raw_context = EVP_MD_CTX_new();
  if (raw_context == nullptr) {
    throw std::runtime_error("unable to allocate SHA-256 context");
  }
  const std::unique_ptr<EVP_MD_CTX, decltype(&EVP_MD_CTX_free)> context(
      raw_context, EVP_MD_CTX_free);
  if (EVP_DigestInit_ex(context.get(), EVP_sha256(), nullptr) != 1) {
    throw std::runtime_error("unable to initialize SHA-256");
  }
  std::array<char, 8192> buffer{};
  while (stream) {
    stream.read(buffer.data(), buffer.size());
    const auto count = stream.gcount();
    if (count > 0 &&
        EVP_DigestUpdate(context.get(), buffer.data(),
                         static_cast<std::size_t>(count)) != 1) {
      throw std::runtime_error("unable to update SHA-256");
    }
  }
  std::array<unsigned char, EVP_MAX_MD_SIZE> digest{};
  unsigned int digest_size = 0;
  if (EVP_DigestFinal_ex(context.get(), digest.data(), &digest_size) != 1) {
    throw std::runtime_error("unable to finalize SHA-256");
  }
  std::ostringstream output;
  output << std::hex << std::setfill('0');
  for (unsigned int index = 0; index < digest_size; ++index) {
    output << std::setw(2) << static_cast<unsigned int>(digest[index]);
  }
  return output.str();
}

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
  result.config_path =
      parameterFileFromArguments(node.get_node_options().arguments());
  if (!result.config_path.empty()) {
    result.config_sha256 = sha256File(result.config_path);
  }
  result.odom_frame = node.declare_parameter("frames.odom", "odom");
  result.base_frame = node.declare_parameter("frames.base", "base_link");
  result.imu_frame = node.declare_parameter("frames.imu", "imu_link");
  result.lidar_frame = node.declare_parameter("frames.lidar", "lidar_link");
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
  result.livox_timestamp_policy = node.declare_parameter(
      "timing.livox_timestamp_policy", "require_header_match");
  result.maximum_imu_gap_ns =
      node.declare_parameter<std::int64_t>("timing.max_imu_gap_ns", 20'000'000);
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
  result.voxel_size_m = node.declare_parameter("preprocessing.voxel_size_m", 0.2);
  const auto local_half_extent = node.declare_parameter<std::vector<double>>(
      "mapping.local_map.half_extent_m", {50.0, 50.0, 25.0});
  if (local_half_extent.size() != 3U) {
    throw std::invalid_argument("mapping.local_map.half_extent_m must have size 3");
  }
  std::copy(local_half_extent.begin(), local_half_extent.end(),
            result.local_map_half_extent_m.begin());
  result.local_map_crop_trigger_distance_m = node.declare_parameter(
      "mapping.local_map.crop_trigger_distance_m", 5.0);
  result.local_map_soft_point_limit = node.declare_parameter<std::int64_t>(
      "mapping.local_map.soft_point_limit", 450000);
  result.local_map_hard_point_limit = node.declare_parameter<std::int64_t>(
      "mapping.local_map.hard_point_limit", 500000);
  result.local_map_target_point_count_after_prune =
      node.declare_parameter<std::int64_t>(
          "mapping.local_map.target_point_count_after_prune", 420000);
  result.local_map_distance_shell_size_m = node.declare_parameter(
      "mapping.local_map.distance_shell_size_m", 5.0);
  result.dynamic_filter_enabled =
      node.declare_parameter("mapping.dynamic_filter.enabled", false);
  result.maximum_registration_iterations =
      node.declare_parameter<std::int64_t>("registration.maximum_iterations", 4);
  result.publish_registered_points =
      node.declare_parameter("output.publish_registered_points", true);
  result.publish_local_map = node.declare_parameter("output.publish_local_map", true);
  result.local_map_rate_hz = node.declare_parameter("output.local_map_rate_hz", 1.0);
  result.imu_queue_capacity =
      node.declare_parameter<std::int64_t>("runtime.imu_queue_capacity", 4096);
  result.lidar_queue_capacity =
      node.declare_parameter<std::int64_t>("runtime.lidar_queue_capacity", 8);
  result.maximum_processing_lag_ms = node.declare_parameter<std::int64_t>(
      "runtime.maximum_processing_lag_ms", 500);
  result.overload_policy =
      node.declare_parameter("runtime.overload_policy", "fail");
  result.input_qos_reliability =
      node.declare_parameter("input.qos_reliability", "best_effort");
  validate(result);
  return result;
}

EstimatorProfile makeEstimatorProfile(const RosParameters& parameters) {
  ParameterLoader::validate(parameters);
  EstimatorProfile profile;
  profile.config_path = parameters.config_path;
  profile.config_sha256 = parameters.config_sha256;
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
  config.deskew.mode = parameters.lidar_timing_mode == "per_point"
                           ? DeskewMode::kPerPoint
                           : DeskewMode::kSimultaneousScan;
  config.preprocessing.point_filter.minimum_range_m =
      parameters.minimum_range_m;
  config.preprocessing.point_filter.maximum_range_m =
      parameters.maximum_range_m;
  config.preprocessing.voxel_filter.voxel_size_m = parameters.voxel_size_m;
  config.local_map.half_extent_m = {
      parameters.local_map_half_extent_m[0],
      parameters.local_map_half_extent_m[1],
      parameters.local_map_half_extent_m[2]};
  config.local_map.crop_trigger_distance_m =
      parameters.local_map_crop_trigger_distance_m;
  config.local_map.soft_point_limit =
      static_cast<std::size_t>(parameters.local_map_soft_point_limit);
  config.local_map.hard_point_limit =
      static_cast<std::size_t>(parameters.local_map_hard_point_limit);
  config.local_map.target_point_count_after_prune = static_cast<std::size_t>(
      parameters.local_map_target_point_count_after_prune);
  config.local_map.distance_shell_size_m =
      parameters.local_map_distance_shell_size_m;
  config.dynamic_filter.enabled = parameters.dynamic_filter_enabled;
  config.initialization.minimum_imu_samples =
      static_cast<std::size_t>(parameters.minimum_imu_samples);
  config.initialization.require_stationary = parameters.require_stationary;
  config.ikfom.maximum_iterations =
      static_cast<std::size_t>(parameters.maximum_registration_iterations);
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
  rclcpp::NodeOptions options;
  options.arguments(
      {"--ros-args", "--params-file",
       std::filesystem::absolute(config_path).lexically_normal().string()});
  rclcpp::Node node("fast_lio", options);
  requireCanonicalFields(node);
  return makeEstimatorProfile(ParameterLoader::declareAndLoad(node));
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
    throw std::invalid_argument("M1 production baseline requires extrinsic.estimate_online=false");
  }
  if (p.maximum_imu_gap_ns <= 0 || p.minimum_imu_samples <= 0 ||
      p.maximum_registration_iterations <= 0 || p.minimum_range_m < 0.0 ||
      p.maximum_range_m <= p.minimum_range_m || p.voxel_size_m <= 0.0 ||
      p.local_map_rate_hz <= 0.0 || !std::isfinite(p.minimum_range_m) ||
      !std::isfinite(p.maximum_range_m) || !std::isfinite(p.voxel_size_m) ||
      !std::isfinite(p.local_map_rate_hz)) {
    throw std::invalid_argument("numeric estimator parameter is out of range");
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
  const Eigen::Map<const Eigen::Vector3d> local_half_extent(
      p.local_map_half_extent_m.data());
  if (!local_half_extent.allFinite() ||
      (local_half_extent.array() <= 0.0).any() ||
      !(p.local_map_crop_trigger_distance_m > 0.0) ||
      !std::isfinite(p.local_map_crop_trigger_distance_m) ||
      p.local_map_target_point_count_after_prune <= 0 ||
      p.local_map_target_point_count_after_prune >= p.local_map_soft_point_limit ||
      p.local_map_soft_point_limit >= p.local_map_hard_point_limit ||
      !(p.local_map_distance_shell_size_m > 0.0) ||
      !std::isfinite(p.local_map_distance_shell_size_m)) {
    throw std::invalid_argument("invalid mapping.local_map configuration");
  }
}

}  // namespace uav::nav::lio
