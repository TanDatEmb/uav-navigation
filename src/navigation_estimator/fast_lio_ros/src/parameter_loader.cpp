#include "fast_lio_ros/parameter_loader.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <vector>

namespace uav::nav::lio {

RosParameters ParameterLoader::declareAndLoad(rclcpp::Node& node) {
  RosParameters result;
  result.odom_frame = node.declare_parameter("frames.odom", "odom");
  result.base_frame = node.declare_parameter("frames.base", "base_link");
  result.imu_frame = node.declare_parameter("frames.imu", "imu_link");
  result.lidar_frame = node.declare_parameter("frames.lidar", "lidar_link");
  result.lidar_topic = node.declare_parameter("input.lidar_topic", "/lidar/points");
  result.imu_topic = node.declare_parameter("input.imu_topic", "/lidar/imu");
  result.lidar_message_type = node.declare_parameter("input.lidar_message_type", "pointcloud2");
  result.lidar_timing_mode = node.declare_parameter("timing.lidar_mode", "simultaneous_scan");
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
  result.maximum_registration_iterations =
      node.declare_parameter<std::int64_t>("registration.maximum_iterations", 4);
  result.publish_registered_points =
      node.declare_parameter("output.publish_registered_points", true);
  result.publish_local_map = node.declare_parameter("output.publish_local_map", true);
  result.local_map_rate_hz = node.declare_parameter("output.local_map_rate_hz", 1.0);
  validate(result);
  return result;
}

void ParameterLoader::validate(const RosParameters& p) {
  if (p.odom_frame.empty() || p.base_frame.empty() || p.imu_frame.empty() ||
      p.lidar_frame.empty()) {
    throw std::invalid_argument("frame names must not be empty");
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
  const double quaternion_norm =
      std::sqrt(p.rotation_imu_lidar_xyzw[0] * p.rotation_imu_lidar_xyzw[0] +
                p.rotation_imu_lidar_xyzw[1] * p.rotation_imu_lidar_xyzw[1] +
                p.rotation_imu_lidar_xyzw[2] * p.rotation_imu_lidar_xyzw[2] +
                p.rotation_imu_lidar_xyzw[3] * p.rotation_imu_lidar_xyzw[3]);
  if (!std::isfinite(quaternion_norm) || quaternion_norm < 1e-9) {
    throw std::invalid_argument("extrinsic rotation quaternion is invalid");
  }
  if (p.estimate_extrinsic_online) {
    throw std::invalid_argument("M1 production baseline requires extrinsic.estimate_online=false");
  }
  if (p.maximum_imu_gap_ns <= 0 || p.minimum_imu_samples <= 0 ||
      p.maximum_registration_iterations <= 0 || p.minimum_range_m < 0.0 ||
      p.maximum_range_m <= p.minimum_range_m || p.voxel_size_m <= 0.0 ||
      p.local_map_rate_hz <= 0.0) {
    throw std::invalid_argument("numeric estimator parameter is out of range");
  }
  if (p.lidar_message_type == "livox_custom") {
    throw std::invalid_argument(
        "livox_custom selected but this build has no livox_ros_driver2 adapter");
  }
}

}  // namespace uav::nav::lio
