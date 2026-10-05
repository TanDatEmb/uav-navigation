#include <chrono>
#include <filesystem>
#include <fstream>
#include <string>

#include <gtest/gtest.h>

#include <navigation_mission/mission.hpp>

namespace {

std::filesystem::path writeMission(const std::string& content) {
  const auto path = std::filesystem::temp_directory_path() /
                    ("uav_navigation_mission_" +
                     std::to_string(std::chrono::steady_clock::now().time_since_epoch().count()) +
                     ".yaml");
  std::ofstream file(path);
  file << content;
  return path;
}

constexpr char kValidMission[] = R"yaml(
mission:
  version: 1
  id: test_route
  frame: lio_odom
  waypoints:
    - id: first
      position: [1.0, 0.0, 3.0]
      acceptance_radius_m: 0.4
      hold_s: 0.1
    - id: second
      position: [2.0, 0.0, 3.0]
      acceptance_radius_m: 0.4
  planning:
    requested_cruise_speed_mps: 1.0
    unknown_policy: blocked
  control:
    acceptance_confirmation_s: 0.0
)yaml";

}  // namespace

TEST(MissionLoader, LoadsCompatibleControlContract) {
  const auto path = writeMission(kValidMission);
  const auto mission = navigation_mission::loadMission(path.string(), "lio_odom");
  std::filesystem::remove(path);
  EXPECT_EQ(mission.id, "test_route");
  ASSERT_EQ(mission.waypoints.size(), 2U);
  EXPECT_DOUBLE_EQ(mission.waypoints[0].position_enu.x(), 1.0);
  EXPECT_DOUBLE_EQ(mission.waypoints[0].hold_s, 0.1);
  EXPECT_DOUBLE_EQ(mission.planning.requested_cruise_speed_mps, 1.0);
  EXPECT_EQ(mission.waypoints[0].behavior,
            navigation_mission::MissionWaypoint::Behavior::Stop);
}

TEST(MissionLoader, RejectsRemovedRouteGuidePlanningMetadata) {
  const auto path = writeMission(R"yaml(
mission:
  version: 1
  id: route_guide_metadata
  frame: lio_odom
  waypoints:
    - {id: start, position: [0.0, 0.0, 3.0], acceptance_radius_m: 0.4}
    - {id: finish, position: [10.0, 0.0, 3.0], acceptance_radius_m: 0.4}
  planning:
    route_guide_enabled: true
    route_guide_sample_spacing_m: 0.5
    route_guide_collision_sample_spacing_m: 0.2
    route_guide_lateral_offset_m: 3.6
    route_guide_lateral_transition_m: 3.0
    unknown_policy: blocked
)yaml");
  EXPECT_THROW(navigation_mission::loadMission(path.string(), "lio_odom"),
               std::invalid_argument);
  std::filesystem::remove(path);
}

TEST(MissionLoader, AllowsExplicitExplorationIntoUnknownSpace) {
  const auto path = writeMission(R"yaml(
mission:
  version: 1
  id: explore_unknown
  frame: lio_odom
  waypoints:
    - {id: start, position: [0.0, 0.0, 3.0], acceptance_radius_m: 0.4}
    - {id: finish, position: [10.0, 0.0, 3.0], acceptance_radius_m: 0.4}
  planning:
    unknown_policy: allow_unknown
)yaml");
  const auto mission = navigation_mission::loadMission(path.string(), "lio_odom");
  std::filesystem::remove(path);
  EXPECT_EQ(mission.planning.unknown_policy,
            navigation_world_model::UnknownPolicy::kAllowUnknown);
}

TEST(MissionLoader, AllowsEmptyOptionalControlSection) {
  const auto path = writeMission(R"yaml(
mission:
  version: 1
  id: empty_control
  frame: lio_odom
  waypoints:
    - {id: start, position: [0.0, 0.0, 3.0], acceptance_radius_m: 0.4}
    - {id: finish, position: [1.0, 0.0, 3.0], acceptance_radius_m: 0.4}
  control:
)yaml");
  const auto mission = navigation_mission::loadMission(path.string(), "lio_odom");
  std::filesystem::remove(path);
  EXPECT_EQ(mission.id, "empty_control");
  EXPECT_DOUBLE_EQ(mission.control.acceptance_speed_mps, 0.15);
}

TEST(MissionLoader, DefaultsIntermediateWaypointToPassThrough) {
  const auto path = writeMission(R"yaml(
mission:
  version: 1
  id: pass_through_route
  frame: lio_odom
  waypoints:
    - {id: middle, position: [1.0, 0.0, 3.0], acceptance_radius_m: 0.4}
    - {id: finish, position: [2.0, 0.0, 3.0], acceptance_radius_m: 0.4}
)yaml");
  const auto mission = navigation_mission::loadMission(path.string(), "lio_odom");
  std::filesystem::remove(path);
  EXPECT_EQ(mission.waypoints[0].behavior,
            navigation_mission::MissionWaypoint::Behavior::PassThrough);
  EXPECT_EQ(mission.waypoints[1].behavior,
            navigation_mission::MissionWaypoint::Behavior::Stop);
}

TEST(MissionLoader, RejectsActionsAndWrongFrame) {
  const auto action_path = writeMission(R"yaml(
mission:
  version: 1
  id: test_route
  frame: lio_odom
  actions: [takeoff]
  waypoints:
    - id: first
      position: [1.0, 0.0, 3.0]
      acceptance_radius_m: 0.4
)yaml");
  EXPECT_THROW(navigation_mission::loadMission(action_path.string(), "lio_odom"),
               std::invalid_argument);
  std::filesystem::remove(action_path);

  const auto frame_path = writeMission(R"yaml(
mission:
  version: 1
  id: test_route
  frame: map
  waypoints:
    - id: first
      position: [1.0, 0.0, 3.0]
      acceptance_radius_m: 0.4
)yaml");
  EXPECT_THROW(navigation_mission::loadMission(frame_path.string(), "lio_odom"),
               std::invalid_argument);
  std::filesystem::remove(frame_path);
}
