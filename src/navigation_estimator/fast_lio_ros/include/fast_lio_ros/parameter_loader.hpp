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
  std::string input_qos_reliability;
  std::string lidar_timing_mode;
  std::string point_time_field;
  std::string point_time_encoding;
  std::string point_time_scan_reference;
  std::int64_t maximum_scan_duration_ns{};
  std::int64_t maximum_header_offset_ns{};
  std::int64_t maximum_boundary_overlap_ns{};
  std::int64_t minimum_points_after_overlap_trim{};
  std::string input_clock_domain;
  std::string livox_timestamp_policy;
  std::int64_t maximum_imu_gap_ns{};
  std::int64_t maximum_recoverable_imu_gap_ns{};
  std::int64_t recovery_confirmation_updates{};
  double discontinuity_covariance_inflation{};
  bool reject_timestamp_regression{};
  bool estimate_extrinsic_online{};
  std::array<double, 3> translation_imu_lidar_m{};
  std::array<double, 4> rotation_imu_lidar_xyzw{};
  std::int64_t minimum_imu_samples{};
  bool require_stationary{};
  double minimum_range_m{};
  double maximum_range_m{};
  double scan_voxel_size_m{};
  double registration_map_voxel_size_m{};
  std::array<double, 3> local_map_half_extent_m{};
  double local_map_crop_trigger_distance_m{};
  std::int64_t local_map_soft_point_limit{};
  std::int64_t local_map_hard_point_limit{};
  std::int64_t local_map_target_point_count_after_prune{};
  double local_map_distance_shell_size_m{};
  bool dynamic_filter_enabled{};
  std::int64_t maximum_registration_iterations{};
  bool publish_registered_points{};
  bool publish_local_map{};
  std::int64_t imu_queue_capacity{};
  std::int64_t lidar_queue_capacity{};
  std::int64_t maximum_processing_lag_ms{};
  std::string overload_policy;
  bool propagated_odometry_enabled{false};
  double propagated_odometry_publish_rate_hz{50.0};
  std::int64_t propagated_odometry_imu_ingress_capacity{4096};
  std::int64_t propagated_odometry_imu_history_duration_ns{1'000'000'000};
  std::int64_t propagated_odometry_maximum_correction_age_ns{300'000'000};
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
