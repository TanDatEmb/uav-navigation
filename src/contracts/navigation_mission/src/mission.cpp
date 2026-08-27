#include "navigation_mission/mission.hpp"

#include <cmath>
#include <filesystem>
#include <set>
#include <stdexcept>

#include <yaml-cpp/yaml.h>

namespace navigation_mission {
namespace {

void requireMap(const YAML::Node& node, const char* name) {
  if (!node || !node.IsMap()) {
    throw std::invalid_argument(std::string("mission field '") + name + "' must be a map");
  }
}

void rejectUnknownKeys(const YAML::Node& node, const std::set<std::string>& allowed,
                       const char* name) {
  for (const auto& entry : node) {
    const auto key = entry.first.as<std::string>();
    if (allowed.find(key) == allowed.end()) {
      throw std::invalid_argument(std::string("unsupported mission field '") + name + "." +
                                  key + "'");
    }
  }
}

double finiteScalar(const YAML::Node& node, const char* name, double minimum,
                    bool allow_zero = false) {
  if (!node || !node.IsScalar()) {
    throw std::invalid_argument(std::string("mission field '") + name + "' must be scalar");
  }
  const double value = node.as<double>();
  const bool valid = std::isfinite(value) && (allow_zero ? value >= minimum : value > minimum);
  if (!valid) {
    throw std::invalid_argument(std::string("mission field '") + name + "' is out of range");
  }
  return value;
}

}  // namespace

Mission loadMission(const std::string& path, const std::string& expected_frame) {
  if (path.empty()) {
    throw std::invalid_argument("navigation.mission_file must not be empty");
  }
  if (!std::filesystem::is_regular_file(path)) {
    throw std::invalid_argument("mission file does not exist: " + path);
  }

  try {
    const YAML::Node root = YAML::LoadFile(path);
    requireMap(root, "root");
    rejectUnknownKeys(root, {"mission"}, "root");
    const YAML::Node node = root["mission"];
    requireMap(node, "mission");
    rejectUnknownKeys(node, {"version", "id", "frame", "waypoints", "planning", "control"},
                      "mission");

    Mission mission;
    mission.schema_version = node["version"] ? node["version"].as<int>() : 0;
    if (mission.schema_version != 1) {
      throw std::invalid_argument("mission.version must be 1");
    }
    mission.id = node["id"] ? node["id"].as<std::string>() : "";
    mission.frame = node["frame"] ? node["frame"].as<std::string>() : "";
    if (mission.id.empty() || mission.frame.empty() ||
        (!expected_frame.empty() && mission.frame != expected_frame)) {
      throw std::invalid_argument("mission id/frame is empty or frame does not match planning frame");
    }

    const YAML::Node waypoints = node["waypoints"];
    if (!waypoints || !waypoints.IsSequence() || waypoints.size() == 0U) {
      throw std::invalid_argument("mission.waypoints must be a non-empty sequence");
    }
    std::set<std::string> waypoint_ids;
    for (std::size_t index = 0; index < waypoints.size(); ++index) {
      const YAML::Node waypoint_node = waypoints[index];
      requireMap(waypoint_node, "waypoints[]");
      rejectUnknownKeys(waypoint_node,
                        {"id", "position", "acceptance_radius_m", "hold_s", "behavior"},
                        "waypoints[]");
      MissionWaypoint waypoint;
      waypoint.id = waypoint_node["id"] ? waypoint_node["id"].as<std::string>() : "";
      if (waypoint.id.empty() || !waypoint_ids.insert(waypoint.id).second) {
        throw std::invalid_argument("mission waypoint IDs must be non-empty and unique");
      }
      const YAML::Node position = waypoint_node["position"];
      if (!position || !position.IsSequence() || position.size() != 3U) {
        throw std::invalid_argument("mission waypoint position must contain exactly 3 values");
      }
      for (int axis = 0; axis < 3; ++axis) {
        waypoint.position_enu[axis] = position[axis].as<double>();
        if (!std::isfinite(waypoint.position_enu[axis])) {
          throw std::invalid_argument("mission waypoint position must be finite");
        }
      }
      waypoint.acceptance_radius_m = finiteScalar(
          waypoint_node["acceptance_radius_m"], "waypoints[].acceptance_radius_m", 0.0);
      waypoint.hold_s = waypoint_node["hold_s"]
                           ? finiteScalar(waypoint_node["hold_s"], "waypoints[].hold_s", 0.0, true)
                           : 0.0;
      const std::string behavior = waypoint_node["behavior"]
                                       ? waypoint_node["behavior"].as<std::string>()
                                       : (waypoint.hold_s > 0.0 || index + 1U == waypoints.size()
                                              ? "stop"
                                              : "pass_through");
      if (behavior == "pass_through") {
        waypoint.behavior = MissionWaypoint::Behavior::PassThrough;
        if (waypoint.hold_s > 0.0) {
          throw std::invalid_argument("pass_through waypoint cannot specify a positive hold_s");
        }
      } else if (behavior == "stop") {
        waypoint.behavior = MissionWaypoint::Behavior::Stop;
      } else {
        throw std::invalid_argument("waypoints[].behavior must be pass_through or stop");
      }
      mission.waypoints.push_back(waypoint);
    }

    if (node["planning"]) {
      requireMap(node["planning"], "planning");
      rejectUnknownKeys(node["planning"], {"max_velocity_mps", "max_acceleration_mps2",
                                            "max_jerk_mps3", "unknown_policy"},
                        "planning");
      const auto planning = node["planning"];
      if (planning["max_velocity_mps"]) {
        mission.planning.max_velocity_mps = finiteScalar(
            planning["max_velocity_mps"], "planning.max_velocity_mps", 0.0);
      }
      if (planning["max_acceleration_mps2"]) {
        mission.planning.max_acceleration_mps2 = finiteScalar(
            planning["max_acceleration_mps2"], "planning.max_acceleration_mps2", 0.0);
      }
      if (planning["max_jerk_mps3"]) {
        mission.planning.max_jerk_mps3 = finiteScalar(
            planning["max_jerk_mps3"], "planning.max_jerk_mps3", 0.0);
      }
      if (planning["unknown_policy"]) {
        const auto policy = planning["unknown_policy"].as<std::string>();
        if (policy == "allow_unknown") {
          mission.planning.unknown_policy =
              navigation_world_model::UnknownPolicy::kAllowUnknown;
        } else if (policy != "blocked") {
          throw std::invalid_argument(
              "mission planning unknown_policy must be 'blocked' or 'allow_unknown'");
        }
      }
    }

    const YAML::Node control = node["control"];
    if (control && !control.IsNull()) {
      requireMap(control, "control");
      rejectUnknownKeys(control, {"acceptance_speed_mps", "acceptance_confirmation_s"},
                        "control");
      if (control["acceptance_speed_mps"]) {
        mission.control.acceptance_speed_mps = finiteScalar(
            control["acceptance_speed_mps"], "control.acceptance_speed_mps", 0.0, true);
      }
      if (control["acceptance_confirmation_s"]) {
        mission.control.acceptance_confirmation_s = finiteScalar(
            control["acceptance_confirmation_s"], "control.acceptance_confirmation_s", 0.0, true);
      }
    }
    return mission;
  } catch (const YAML::Exception& error) {
    throw std::invalid_argument("invalid mission YAML: " + std::string(error.what()));
  }
}

}  // namespace navigation_mission
