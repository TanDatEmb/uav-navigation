#pragma once

#include <cmath>
#include <cstdint>
#include <set>
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

  [[nodiscard]] bool valid() const noexcept {
    if (schema_version != 1 || id.empty() || frame.empty() || waypoints.empty() ||
        !std::isfinite(planning.max_velocity_mps) || planning.max_velocity_mps <= 0.0 ||
        !std::isfinite(planning.max_acceleration_mps2) ||
        planning.max_acceleration_mps2 <= 0.0 ||
        !std::isfinite(planning.max_jerk_mps3) || planning.max_jerk_mps3 <= 0.0 ||
        (planning.unknown_policy != navigation_world_model::UnknownPolicy::kAllowUnknown &&
         planning.unknown_policy != navigation_world_model::UnknownPolicy::kRequireKnownFree) ||
        !std::isfinite(control.acceptance_speed_mps) || control.acceptance_speed_mps < 0.0 ||
        !std::isfinite(control.acceptance_confirmation_s) ||
        control.acceptance_confirmation_s < 0.0) {
      return false;
    }
    std::set<std::string> waypoint_ids;
    for (const auto& waypoint : waypoints) {
      if (waypoint.id.empty() || !waypoint.position_enu.allFinite() ||
          !std::isfinite(waypoint.acceptance_radius_m) ||
          waypoint.acceptance_radius_m <= 0.0 || !std::isfinite(waypoint.hold_s) ||
          waypoint.hold_s < 0.0 ||
          (waypoint.behavior != MissionWaypoint::Behavior::PassThrough &&
           waypoint.behavior != MissionWaypoint::Behavior::Stop) ||
          (waypoint.behavior == MissionWaypoint::Behavior::PassThrough &&
           waypoint.hold_s > 0.0) ||
          !waypoint_ids.insert(waypoint.id).second) {
        return false;
      }
    }
    return true;
  }
};

[[nodiscard]] Mission loadMission(const std::string& path,
                                  const std::string& expected_frame = {});

}  // namespace navigation_mission
