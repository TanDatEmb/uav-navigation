#pragma once

#include <array>
#include <rclcpp/node.hpp>
#include <string>

namespace uav::nav::lio {

struct RosParameters {
  std::string odom_frame;
  std::string base_frame;
  std::string imu_frame;
  std::string lidar_frame;
  std::string lidar_topic;
  std::string imu_topic;
  std::string lidar_message_type;
  std::string lidar_timing_mode;
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

class ParameterLoader {
 public:
  [[nodiscard]] static RosParameters declareAndLoad(rclcpp::Node& node);
  static void validate(const RosParameters& parameters);
};

}  // namespace uav::nav::lio
