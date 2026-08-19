#pragma once

#include <string>
#include <vector>

#include <Eigen/Core>

namespace px4_navigation_external_mode {

struct MissionWaypoint {
  std::string id;
  Eigen::Vector3d position_enu{Eigen::Vector3d::Zero()};
  double acceptance_radius_m{0.5};
  double hold_s{0.0};
};

struct MissionPlanningConfig {
  double replan_rate_hz{5.0};
  double max_velocity_mps{1.0};
  double max_acceleration_mps2{2.0};
  double max_deceleration_mps2{2.0};
  double max_jerk_mps3{6.0};
  std::string unknown_policy{"blocked"};
};

struct MissionControlConfig {
  std::string output{"auto"};
  double acceptance_speed_mps{0.15};
  double acceptance_confirmation_s{0.5};
};

struct Mission {
  int version{1};
  std::string id;
  std::string frame;
  std::vector<MissionWaypoint> waypoints;
  MissionPlanningConfig planning;
  MissionControlConfig control;
};

[[nodiscard]] Mission loadMission(const std::string& path, const std::string& expected_frame);

}  // namespace px4_navigation_external_mode
