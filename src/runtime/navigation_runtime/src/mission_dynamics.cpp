#include "navigation_runtime/mission_dynamics.hpp"

#include <cmath>
#include <stdexcept>
#include <navigation_mission/mission.hpp>

namespace navigation_runtime {

navigation_planning::DynamicLimits loadMissionDynamicLimits(
    const std::filesystem::path& mission_file,
    const std::string& expected_frame) {
  if (mission_file.empty()) {
    throw std::invalid_argument("planner backend mission file must not be empty");
  }
  const auto mission = navigation_mission::loadMission(mission_file.string(), expected_frame);
  navigation_planning::DynamicLimits limits;
  limits.intent.requested_cruise_speed_mps =
      mission.planning.requested_cruise_speed_mps;
  limits.unknown_space_policy = mission.planning.unknown_policy;
  return limits;
}

}  // namespace navigation_runtime
