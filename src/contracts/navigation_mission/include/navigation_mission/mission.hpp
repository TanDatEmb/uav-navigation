#pragma once

#include <cstdint>
#include <string>
#include <vector>

#include <Eigen/Core>
#include <navigation_world_model/world_model_view.hpp>

namespace navigation_mission {

struct MissionWaypoint {
  enum class Behavior : std::uint8_t { PassThrough = 0U, Stop = 1U };

  std::string id;
  Eigen::Vector3d position_enu{Eigen::Vector3d::Zero()};
  double acceptance_radius_m{0.5};
  double hold_s{0.0};
  Behavior behavior{Behavior::PassThrough};
};

struct MissionPlanningConfig {
  double max_velocity_mps{1.0};
  double max_acceleration_mps2{2.0};
  double max_jerk_mps3{6.0};
  navigation_world_model::UnknownPolicy unknown_policy{
      navigation_world_model::UnknownPolicy::kRequireKnownFree};
};

struct MissionControlConfig {
  double acceptance_speed_mps{0.15};
  double acceptance_confirmation_s{0.5};
};

struct Mission {
  int schema_version{1};
  std::string id;
  std::string frame;
  std::vector<MissionWaypoint> waypoints;
  MissionPlanningConfig planning;
  MissionControlConfig control;
};

[[nodiscard]] Mission loadMission(const std::string& path,
                                  const std::string& expected_frame = {});

}  // namespace navigation_mission
