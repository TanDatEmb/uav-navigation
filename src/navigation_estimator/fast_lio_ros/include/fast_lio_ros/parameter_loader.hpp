#pragma once

#include <array>
#include <rclcpp/node.hpp>
#include <string>
#include <string_view>

#include "fast_lio_core/configuration/estimator_config.hpp"
#include "fast_lio_core/time/clock_domain.hpp"
#include "fast_lio_ros/ros_lidar_adapter.hpp"
#include "fast_lio_ros/ros_livox_custom_adapter.hpp"

namespace uav::nav::lio {

struct RosParameters {
  std::string config_path;
  std::string config_sha256;
  std::string odom_frame;
  std::string base_frame;
  std::string imu_frame;
  std::string lidar_frame;
  std::string lidar_topic;
  std::string imu_topic;
  std::string lidar_input_frame;
  std::string imu_input_frame;
  std::string lidar_message_type;
  std::string lidar_timing_mode;
  std::string point_time_field;
  std::string point_time_encoding;
  std::string point_time_scan_reference;
  std::int64_t maximum_scan_duration_ns{};
  std::int64_t maximum_header_offset_ns{};
  std::string input_clock_domain;
  std::string livox_timestamp_policy;
  std::int64_t maximum_imu_gap_ns{};
  bool reject_timestamp_regression{};
  bool estimate_extrinsic_online{};
  std::array<double, 3> translation_imu_lidar_m{};
  std::array<double, 4> rotation_imu_lidar_xyzw{};
  std::int64_t minimum_imu_samples{};
  bool require_stationary{};
  double minimum_range_m{};
  double maximum_range_m{};
  double voxel_size_m{};
  std::int64_t maximum_registration_iterations{};
  bool publish_registered_points{};
  bool publish_local_map{};
  double local_map_rate_hz{};
};

struct EstimatorProfile {
  EstimatorConfig estimator;
  std::string config_path;
  std::string config_sha256;
  std::string lidar_topic;
  std::string imu_topic;
  std::string lidar_input_frame;
  std::string imu_input_frame;
  std::string lidar_message_type;
  std::string lidar_timing_mode;
  PointTimeConfig point_time;
  ClockDomain clock_domain{ClockDomain::kRosTime};
  LivoxTimestampPolicy timestamp_policy{
      LivoxTimestampPolicy::kRequireHeaderMatchesTimebase};
};

[[nodiscard]] ClockDomain parseClockDomain(std::string_view value);
[[nodiscard]] EstimatorProfile makeEstimatorProfile(
    const RosParameters& parameters);
[[nodiscard]] EstimatorProfile loadCanonicalEstimatorProfile(
    const std::string& config_path);
[[nodiscard]] std::string sha256File(const std::string& path);

class ParameterLoader {
 public:
  [[nodiscard]] static RosParameters declareAndLoad(rclcpp::Node& node);
  static void validate(const RosParameters& parameters);
};

}  // namespace uav::nav::lio
