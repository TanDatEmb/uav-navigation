#include "navigation_runtime/mission_dynamics.hpp"

#include <cmath>
#include <stdexcept>
#include <string>

#include <yaml-cpp/yaml.h>

namespace navigation_runtime {
namespace {

double positiveScalar(const YAML::Node& node, const char* path, double fallback) {
  if (!node) return fallback;
  double value = 0.0;
  try {
    value = node.as<double>();
  } catch (const YAML::Exception& error) {
    throw std::invalid_argument(std::string(path) + " must be numeric: " + error.what());
  }
  if (!std::isfinite(value) || value <= 0.0) {
    throw std::invalid_argument(std::string(path) + " must be finite and positive");
  }
  return value;
}

}  // namespace

super_planner::DynamicLimits loadMissionDynamicLimits(
    const std::filesystem::path& mission_file) {
  if (mission_file.empty()) {
    throw std::invalid_argument("SUPER mission file must not be empty");
  }
  YAML::Node document;
  try {
    document = YAML::LoadFile(mission_file.string());
  } catch (const YAML::Exception& error) {
    throw std::invalid_argument(
        "cannot load SUPER mission file " + mission_file.string() + ": " + error.what());
  }
  const YAML::Node mission = document["mission"];
  if (!mission || !mission.IsMap()) {
    throw std::invalid_argument("SUPER mission file must contain a mission mapping");
  }
  const YAML::Node planning = mission["planning"];
  if (planning && !planning.IsMap()) {
    throw std::invalid_argument("SUPER mission planning must be a mapping");
  }

  // Match the External Mode mission contract defaults when a field is absent.
  return super_planner::DynamicLimits{
      positiveScalar(planning["max_velocity_mps"],
                     "mission.planning.max_velocity_mps", 1.0),
      positiveScalar(planning["max_acceleration_mps2"],
                     "mission.planning.max_acceleration_mps2", 2.0),
      positiveScalar(planning["max_jerk_mps3"],
                     "mission.planning.max_jerk_mps3", 6.0)};
}

}  // namespace navigation_runtime
