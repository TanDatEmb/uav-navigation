#include <chrono>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <limits>

#include <gtest/gtest.h>

#include "px4_navigation_external_mode/mission.hpp"
#include "px4_navigation_external_mode/mission_controller.hpp"

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

void expectWaypointAccepted(const px4_navigation_external_mode::MissionControllerEvent& event,
                            std::size_t waypoint_index) {
  EXPECT_TRUE(event.waypoint_accepted);
  EXPECT_EQ(event.accepted_waypoint_index, waypoint_index);
  EXPECT_TRUE(std::isfinite(event.acceptance_position_error_m));
  EXPECT_TRUE(std::isfinite(event.acceptance_speed_mps));
  EXPECT_GE(event.acceptance_position_error_m, 0.0);
  EXPECT_GE(event.acceptance_speed_mps, 0.0);
}

void expectNoWaypointAccepted(
    const px4_navigation_external_mode::MissionControllerEvent& event) {
  EXPECT_FALSE(event.waypoint_accepted);
  EXPECT_TRUE(std::isfinite(event.acceptance_position_error_m));
  EXPECT_TRUE(std::isfinite(event.acceptance_speed_mps));
}

}  // namespace

TEST(MissionLoader, LoadsCompatibleControlContract) {
  const auto path = writeMission(kValidMission);
  const auto mission = px4_navigation_external_mode::loadMission(path.string(), "lio_odom");
  std::filesystem::remove(path);
  EXPECT_EQ(mission.id, "test_route");
  ASSERT_EQ(mission.waypoints.size(), 2U);
  EXPECT_DOUBLE_EQ(mission.waypoints[0].position_enu.x(), 1.0);
  EXPECT_DOUBLE_EQ(mission.waypoints[0].hold_s, 0.1);
  EXPECT_DOUBLE_EQ(mission.planning.requested_cruise_speed_mps, 1.0);
  EXPECT_EQ(mission.waypoints[0].behavior,
            px4_navigation_external_mode::MissionWaypoint::Behavior::Stop);
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
  EXPECT_THROW(px4_navigation_external_mode::loadMission(path.string(), "lio_odom"),
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
  const auto mission = px4_navigation_external_mode::loadMission(path.string(), "lio_odom");
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
  const auto mission = px4_navigation_external_mode::loadMission(path.string(), "lio_odom");
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
  const auto mission = px4_navigation_external_mode::loadMission(path.string(), "lio_odom");
  std::filesystem::remove(path);
  EXPECT_EQ(mission.waypoints[0].behavior,
            px4_navigation_external_mode::MissionWaypoint::Behavior::PassThrough);
  EXPECT_EQ(mission.waypoints[1].behavior,
            px4_navigation_external_mode::MissionWaypoint::Behavior::Stop);
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
  EXPECT_THROW(px4_navigation_external_mode::loadMission(action_path.string(), "lio_odom"),
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
  EXPECT_THROW(px4_navigation_external_mode::loadMission(frame_path.string(), "lio_odom"),
               std::invalid_argument);
  std::filesystem::remove(frame_path);
}

TEST(MissionController, PublishesWaypointsAndCompletesWithoutFlightActions) {
  const auto path = writeMission(kValidMission);
  const auto mission = px4_navigation_external_mode::loadMission(path.string(), "lio_odom");
  std::filesystem::remove(path);
  px4_navigation_external_mode::MissionController controller(mission);

  controller.activate(0.0);
  auto event = controller.update(0.0, std::nullopt);
  EXPECT_EQ(event.type, px4_navigation_external_mode::MissionControllerEvent::Type::PublishGoal);
  EXPECT_EQ(event.waypoint_index, 0U);
  EXPECT_EQ(event.request_id, 1U);
  EXPECT_EQ(controller.activeRequestId(), 1U);

  controller.onTrajectory(true, 0.0);
  event = controller.update(0.1, Eigen::Vector3d{1.0, 0.0, 3.0}, true,
                            Eigen::Vector3d::Zero());
  EXPECT_EQ(controller.state(), px4_navigation_external_mode::MissionControllerState::Holding);
  EXPECT_EQ(event.type, px4_navigation_external_mode::MissionControllerEvent::Type::None);

  event = controller.update(0.3, Eigen::Vector3d{1.0, 0.0, 3.0}, true,
                            Eigen::Vector3d::Zero());
  EXPECT_EQ(event.type, px4_navigation_external_mode::MissionControllerEvent::Type::PublishGoal);
  expectWaypointAccepted(event, 0U);
  EXPECT_EQ(event.waypoint_index, 1U);
  EXPECT_EQ(event.request_id, 2U);
  EXPECT_EQ(controller.activeRequestId(), 2U);

  controller.onTrajectory(true, 0.2);
  event = controller.update(0.3, Eigen::Vector3d{2.0, 0.0, 3.0}, true,
                            Eigen::Vector3d::Zero());
  EXPECT_EQ(controller.state(), px4_navigation_external_mode::MissionControllerState::Holding);
  event = controller.update(0.4, Eigen::Vector3d{2.0, 0.0, 3.0}, true,
                            Eigen::Vector3d::Zero());
  EXPECT_EQ(event.type, px4_navigation_external_mode::MissionControllerEvent::Type::Complete);
  expectWaypointAccepted(event, 1U);
  EXPECT_EQ(controller.state(), px4_navigation_external_mode::MissionControllerState::Complete);
  EXPECT_EQ(controller.activeWaypointIndex(), mission.waypoints.size());
  EXPECT_FALSE(controller.activeWaypoint().has_value());
  EXPECT_FALSE(controller.waypointAt(controller.activeWaypointIndex()).has_value());
  const auto terminal_waypoint = controller.waypointAt(event.waypoint_index);
  ASSERT_TRUE(terminal_waypoint.has_value());
  EXPECT_EQ(terminal_waypoint->id, "second");
}

TEST(MissionController, WaitsForAirborneBeforePublishingMissionGoal) {
  const auto path = writeMission(kValidMission);
  const auto mission = px4_navigation_external_mode::loadMission(path.string(), "lio_odom");
  std::filesystem::remove(path);
  px4_navigation_external_mode::MissionController controller(mission);

  controller.activate(0.0);
  EXPECT_EQ(controller.state(), px4_navigation_external_mode::MissionControllerState::WaitingForAirborne);
  EXPECT_EQ(controller.update(0.0, std::nullopt, false).type,
            px4_navigation_external_mode::MissionControllerEvent::Type::None);
  EXPECT_EQ(controller.update(0.0, std::nullopt, true).type,
            px4_navigation_external_mode::MissionControllerEvent::Type::PublishGoal);
}

TEST(MissionController, NativeTrajectoryReadySurvivesAirborneTransitionRace) {
  const auto path = writeMission(kValidMission);
  const auto mission = px4_navigation_external_mode::loadMission(path.string(), "lio_odom");
  std::filesystem::remove(path);
  px4_navigation_external_mode::MissionController controller(mission);

  controller.activate(0.0);
  EXPECT_EQ(controller.update(0.0, std::nullopt, false).type,
            px4_navigation_external_mode::MissionControllerEvent::Type::None);
  ASSERT_EQ(controller.update(0.0, std::nullopt, true).type,
            px4_navigation_external_mode::MissionControllerEvent::Type::PublishGoal);
  // The native planner backend callback may arrive immediately after goal publication,
  // before the next mission timer tick. It must not be ignored because the
  // controller has only just crossed the airborne transition.
  controller.onNativeTrajectoryReady();

  auto event = controller.update(0.1, Eigen::Vector3d{1.0, 0.0, 3.0}, true,
                                 Eigen::Vector3d::Zero());
  EXPECT_EQ(controller.state(), px4_navigation_external_mode::MissionControllerState::Holding);
  EXPECT_EQ(event.type, px4_navigation_external_mode::MissionControllerEvent::Type::None);
  event = controller.update(0.3, Eigen::Vector3d{1.0, 0.0, 3.0}, true,
                            Eigen::Vector3d::Zero());
  EXPECT_EQ(event.type, px4_navigation_external_mode::MissionControllerEvent::Type::PublishGoal);
  expectWaypointAccepted(event, 0U);
}

TEST(MissionController, HighSpeedFlyThroughDoesNotCompleteWaypoint) {
  const auto path = writeMission(kValidMission);
  const auto mission = px4_navigation_external_mode::loadMission(path.string(), "lio_odom");
  std::filesystem::remove(path);
  px4_navigation_external_mode::MissionController controller(mission);

  controller.activate(0.0);
  ASSERT_EQ(controller.update(0.0, std::nullopt).type,
            px4_navigation_external_mode::MissionControllerEvent::Type::PublishGoal);
  controller.onTrajectory(true, 0.0);
  const auto event = controller.update(0.1, Eigen::Vector3d{1.0, 0.0, 3.0}, true,
                                       Eigen::Vector3d{0.5, 0.0, 0.0});
  EXPECT_EQ(event.type, px4_navigation_external_mode::MissionControllerEvent::Type::None);
  EXPECT_EQ(controller.state(),
            px4_navigation_external_mode::MissionControllerState::ExecutingWaypoint);
}

TEST(MissionController, PassThroughAdvancesWithoutStopping) {
  const auto path = writeMission(R"yaml(
mission:
  version: 1
  id: pass_through_route
  frame: lio_odom
  waypoints:
    - {id: middle, position: [1.0, 0.0, 3.0], acceptance_radius_m: 0.4}
    - {id: finish, position: [2.0, 0.0, 3.0], acceptance_radius_m: 0.4}
)yaml");
  const auto mission = px4_navigation_external_mode::loadMission(path.string(), "lio_odom");
  std::filesystem::remove(path);
  px4_navigation_external_mode::MissionController controller(mission);
  controller.activate(0.0);
  ASSERT_EQ(controller.update(0.0, std::nullopt).type,
            px4_navigation_external_mode::MissionControllerEvent::Type::PublishGoal);
  controller.onTrajectory(true, 0.0);
  const auto event = controller.update(0.1, Eigen::Vector3d{1.0, 0.0, 3.0}, true,
                                       Eigen::Vector3d{1.0, 0.0, 0.0});
  EXPECT_EQ(event.type,
            px4_navigation_external_mode::MissionControllerEvent::Type::PublishGoal);
  expectWaypointAccepted(event, 0U);
  EXPECT_EQ(event.waypoint_index, 1U);
  EXPECT_EQ(controller.state(),
            px4_navigation_external_mode::MissionControllerState::ExecutingWaypoint);
  EXPECT_EQ(controller.nextWaypoint(), std::nullopt);
}

TEST(MissionController, PassThroughDoesNotAdvanceWithoutCurrentGoalTrajectory) {
  const auto path = writeMission(R"yaml(
mission:
  version: 1
  id: pass_through_handoff_contract
  frame: lio_odom
  waypoints:
    - {id: first, position: [1.0, 0.0, 3.0], behavior: pass_through, acceptance_radius_m: 0.4}
    - {id: second, position: [2.0, 0.0, 3.0], behavior: pass_through, acceptance_radius_m: 0.4}
    - {id: finish, position: [3.0, 0.0, 3.0], behavior: stop, acceptance_radius_m: 0.4}
  control:
    acceptance_speed_mps: 0.15
)yaml");
  const auto mission = px4_navigation_external_mode::loadMission(path.string(), "lio_odom");
  std::filesystem::remove(path);
  px4_navigation_external_mode::MissionController controller(mission);

  controller.activate(0.0);
  ASSERT_EQ(controller.update(0.0, std::nullopt).type,
            px4_navigation_external_mode::MissionControllerEvent::Type::PublishGoal);
  controller.onTrajectory(true, 0.0);

  const auto first = controller.update(0.1, Eigen::Vector3d{1.0, 0.0, 3.0}, true,
                                       Eigen::Vector3d{1.0, 0.0, 0.0});
  ASSERT_EQ(first.type,
            px4_navigation_external_mode::MissionControllerEvent::Type::PublishGoal);
  expectWaypointAccepted(first, 0U);
  ASSERT_EQ(controller.activeWaypointIndex(), 1U);

  // The vehicle is inside the second pass-through waypoint, but no native
  // trajectory for that new goal has been acknowledged.  Position and speed
  // alone must not advance the mission across an unowned command boundary.
  const auto stale = controller.update(0.2, Eigen::Vector3d{2.0, 0.0, 3.0}, true,
                                       Eigen::Vector3d::Zero());
  EXPECT_EQ(stale.type,
            px4_navigation_external_mode::MissionControllerEvent::Type::None);
  expectNoWaypointAccepted(stale);
  EXPECT_EQ(controller.activeWaypointIndex(), 1U);

  controller.onTrajectory(true, 0.2);
  const auto second = controller.update(0.3, Eigen::Vector3d{2.0, 0.0, 3.0}, true,
                                        Eigen::Vector3d::Zero());
  EXPECT_EQ(second.type,
            px4_navigation_external_mode::MissionControllerEvent::Type::PublishGoal);
  expectWaypointAccepted(second, 1U);
  EXPECT_EQ(second.waypoint_index, 2U);
}

TEST(MissionController, PassThroughAcceptanceDoesNotBypassAcceptanceRadius) {
  const auto path = writeMission(R"yaml(
mission:
  version: 1
  id: lookahead_route
  frame: lio_odom
  waypoints:
    - {id: origin, position: [0.0, 0.0, 3.0], acceptance_radius_m: 0.4}
    - {id: middle, position: [1.0, 0.0, 3.0], acceptance_radius_m: 0.4}
    - {id: finish, position: [2.0, 0.0, 3.0], acceptance_radius_m: 0.4}
)yaml");
  const auto mission = px4_navigation_external_mode::loadMission(path.string(), "lio_odom");
  std::filesystem::remove(path);
  px4_navigation_external_mode::MissionController controller(mission);
  controller.activate(0.0);
  ASSERT_EQ(controller.update(0.0, std::nullopt).type,
            px4_navigation_external_mode::MissionControllerEvent::Type::PublishGoal);
  controller.onTrajectory(true, 0.0);
  auto event = controller.update(0.1, Eigen::Vector3d{0.0, 0.0, 3.0}, true,
                                 Eigen::Vector3d::Zero());
  ASSERT_EQ(event.type,
            px4_navigation_external_mode::MissionControllerEvent::Type::PublishGoal);
  expectWaypointAccepted(event, 0U);
  controller.onTrajectory(true, 0.1);
  event = controller.update(0.2, Eigen::Vector3d{0.0, 0.0, 3.0}, true,
                                       Eigen::Vector3d{1.0, 0.0, 0.0});
  EXPECT_EQ(event.type, px4_navigation_external_mode::MissionControllerEvent::Type::None);
  expectNoWaypointAccepted(event);
  EXPECT_EQ(controller.activeWaypointIndex(), 1U);

  event = controller.update(0.3, Eigen::Vector3d{0.7, 0.0, 3.0}, true,
                             Eigen::Vector3d{1.0, 0.0, 0.0});
  EXPECT_EQ(event.type,
            px4_navigation_external_mode::MissionControllerEvent::Type::PublishGoal);
  expectWaypointAccepted(event, 1U);
  EXPECT_EQ(event.waypoint_index, 2U);
}

TEST(MissionController, PassThroughRequiresMeasuredPositionInsideAcceptanceRadius) {
  const auto path = writeMission(R"yaml(
mission:
  version: 1
  id: pass_through_acceptance_gate
  frame: lio_odom
  waypoints:
    - id: middle
      position: [1.0, 0.0, 3.0]
      behavior: pass_through
      acceptance_radius_m: 0.4
    - id: finish
      position: [2.0, 0.0, 3.0]
      behavior: stop
      acceptance_radius_m: 0.4
      hold_s: 0.1
)yaml");
  const auto mission = px4_navigation_external_mode::loadMission(path.string(), "lio_odom");
  std::filesystem::remove(path);
  px4_navigation_external_mode::MissionController controller(mission);

  controller.activate(0.0);
  ASSERT_EQ(controller.update(0.0, std::nullopt).type,
            px4_navigation_external_mode::MissionControllerEvent::Type::PublishGoal);
  controller.onTrajectory(true, 0.0);

  const auto outside = controller.update(0.1, Eigen::Vector3d{1.5, 0.0, 3.0}, true,
                                         Eigen::Vector3d::Zero());
  EXPECT_EQ(outside.type,
            px4_navigation_external_mode::MissionControllerEvent::Type::None);
  expectNoWaypointAccepted(outside);
  EXPECT_EQ(controller.activeWaypointIndex(), 0U);
  EXPECT_EQ(controller.state(),
            px4_navigation_external_mode::MissionControllerState::ExecutingWaypoint);

  const auto accepted = controller.update(0.2, Eigen::Vector3d{1.2, 0.0, 3.0}, true,
                                          Eigen::Vector3d::Zero());
  EXPECT_EQ(accepted.type,
            px4_navigation_external_mode::MissionControllerEvent::Type::PublishGoal);
  expectWaypointAccepted(accepted, 0U);
  EXPECT_EQ(accepted.waypoint_index, 1U);
  EXPECT_EQ(controller.activeWaypointIndex(), 1U);
}

TEST(MissionController, PassThroughAcceptsRecentMeasuredSegmentCrossingAtCruiseSpeed) {
  const auto path = writeMission(R"yaml(
mission:
  version: 1
  id: pass_through_segment_crossing
  frame: lio_odom
  waypoints:
    - id: origin
      position: [0.0, 0.0, 3.0]
      behavior: pass_through
      acceptance_radius_m: 0.4
    - id: middle
      position: [1.0, 0.0, 3.0]
      behavior: pass_through
      acceptance_radius_m: 0.4
    - id: finish
      position: [2.0, 0.0, 3.0]
      behavior: stop
      acceptance_radius_m: 0.4
      hold_s: 0.1
  control:
    acceptance_speed_mps: 0.15
)yaml");
  const auto mission = px4_navigation_external_mode::loadMission(path.string(), "lio_odom");
  std::filesystem::remove(path);
  px4_navigation_external_mode::MissionController controller(mission);

  controller.activate(0.0);
  ASSERT_EQ(controller.update(0.0, std::nullopt).type,
            px4_navigation_external_mode::MissionControllerEvent::Type::PublishGoal);
  controller.onTrajectory(true, 0.0);
  const auto origin = controller.update(
      0.05, Eigen::Vector3d{0.0, 0.0, 3.0}, true, Eigen::Vector3d::Zero());
  ASSERT_TRUE(origin.waypoint_accepted);
  controller.onTrajectory(true, 0.05);

  // Neither endpoint is inside the 0.4 m ball around x=1.0, but the two
  // recent measured samples cross it at 5 m/s. Pass-through must not lose the
  // checkpoint solely because the timer did not sample the ball's interior.
  const auto crossing = controller.update(
      0.10, Eigen::Vector3d{2.0, 0.0, 3.0}, true,
      Eigen::Vector3d{5.0, 0.0, 0.0});
  EXPECT_EQ(crossing.type,
            px4_navigation_external_mode::MissionControllerEvent::Type::PublishGoal);
  expectWaypointAccepted(crossing, 1U);
  EXPECT_NEAR(crossing.acceptance_position_error_m, 0.0, 1.0e-12);
  EXPECT_DOUBLE_EQ(crossing.acceptance_speed_mps, 5.0);
  EXPECT_EQ(crossing.waypoint_index, 2U);
}

TEST(MissionController, PassThroughDoesNotAcceptLaterNearSelfCrossingBranch) {
  const auto path = writeMission(R"yaml(
mission:
  version: 1
  id: pass_through_near_self_crossing
  frame: lio_odom
  waypoints:
    - id: origin
      position: [0.0, 0.0, 3.0]
      behavior: pass_through
      acceptance_radius_m: 0.4
    - id: active
      position: [10.0, 0.0, 3.0]
      behavior: pass_through
      acceptance_radius_m: 1.2
    - id: branch
      position: [20.0, 10.0, 3.0]
      behavior: pass_through
      acceptance_radius_m: 0.4
    - id: later
      position: [10.0, 1.0, 3.0]
      behavior: stop
      acceptance_radius_m: 0.4
  control:
    acceptance_speed_mps: 0.15
)yaml");
  const auto mission = px4_navigation_external_mode::loadMission(path.string(), "lio_odom");
  std::filesystem::remove(path);
  px4_navigation_external_mode::MissionController controller(mission);

  controller.activate(0.0);
  ASSERT_EQ(controller.update(0.0, std::nullopt).type,
            px4_navigation_external_mode::MissionControllerEvent::Type::PublishGoal);
  controller.onTrajectory(true, 0.0);
  const auto origin = controller.update(
      0.05, Eigen::Vector3d{0.0, 0.0, 3.0}, true, Eigen::Vector3d::Zero());
  ASSERT_TRUE(origin.waypoint_accepted);
  EXPECT_EQ(origin.waypoint_index, 1U);
  controller.onTrajectory(true, 0.05);

  // Move onto a later branch while the active checkpoint remains unaccepted.
  const auto branch = controller.update(
      0.10, Eigen::Vector3d{20.0, 10.0, 3.0}, true,
      Eigen::Vector3d{1.0, 0.0, 0.0});
  EXPECT_EQ(branch.type,
            px4_navigation_external_mode::MissionControllerEvent::Type::None);
  expectNoWaypointAccepted(branch);
  EXPECT_EQ(controller.activeWaypointIndex(), 1U);

  // This later-branch point lies inside active's acceptance sphere. Spatial
  // proximity alone must not advance the ordered mission checkpoint.
  const auto later = controller.update(
      0.15, Eigen::Vector3d{10.0, 1.0, 3.0}, true,
      Eigen::Vector3d{1.0, 0.0, 0.0});
  EXPECT_EQ(later.type,
            px4_navigation_external_mode::MissionControllerEvent::Type::None);
  expectNoWaypointAccepted(later);
  EXPECT_EQ(controller.activeWaypointIndex(), 1U);
}

TEST(MissionController, PassThroughRejectsCrossingAcrossStaleMeasuredGap) {
  const auto path = writeMission(R"yaml(
mission:
  version: 1
  id: pass_through_stale_crossing
  frame: lio_odom
  waypoints:
    - id: origin
      position: [0.0, 0.0, 3.0]
      behavior: pass_through
      acceptance_radius_m: 0.4
    - id: middle
      position: [1.0, 0.0, 3.0]
      behavior: pass_through
      acceptance_radius_m: 0.4
    - id: finish
      position: [2.0, 0.0, 3.0]
      behavior: stop
      acceptance_radius_m: 0.4
  control:
    acceptance_speed_mps: 0.15
)yaml");
  const auto mission = px4_navigation_external_mode::loadMission(path.string(), "lio_odom");
  std::filesystem::remove(path);
  px4_navigation_external_mode::MissionController controller(mission);

  controller.activate(0.0);
  ASSERT_EQ(controller.update(0.0, std::nullopt).type,
            px4_navigation_external_mode::MissionControllerEvent::Type::PublishGoal);
  controller.onTrajectory(true, 0.0);
  ASSERT_TRUE(controller.update(
      0.05, Eigen::Vector3d{0.0, 0.0, 3.0}, true,
      Eigen::Vector3d::Zero()).waypoint_accepted);
  controller.onTrajectory(true, 0.05);

  const auto stale_crossing = controller.update(
      0.35, Eigen::Vector3d{2.0, 0.0, 3.0}, true,
      Eigen::Vector3d{5.0, 0.0, 0.0});
  EXPECT_FALSE(stale_crossing.waypoint_accepted);
  EXPECT_EQ(controller.activeWaypointIndex(), 1U);
}

TEST(MissionController, PassThroughCornerAdvancesOnMeasuredAcceptance) {
  const auto path = writeMission(R"yaml(
mission:
  version: 1
  id: pass_through_corner_gate
  frame: lio_odom
  waypoints:
    - id: origin
      position: [0.0, 0.0, 3.0]
      behavior: pass_through
      acceptance_radius_m: 0.4
    - id: corner
      position: [1.0, 0.0, 3.0]
      behavior: pass_through
      acceptance_radius_m: 0.4
    - id: finish
      position: [1.0, 1.0, 3.0]
      behavior: stop
      acceptance_radius_m: 0.4
      hold_s: 0.1
  control:
    acceptance_speed_mps: 0.15
)yaml");
  const auto mission = px4_navigation_external_mode::loadMission(path.string(), "lio_odom");
  std::filesystem::remove(path);
  px4_navigation_external_mode::MissionController controller(mission);

  controller.activate(0.0);
  ASSERT_EQ(controller.update(0.0, std::nullopt).type,
            px4_navigation_external_mode::MissionControllerEvent::Type::PublishGoal);
  controller.onTrajectory(true, 0.1);
  const auto origin = controller.update(0.2, Eigen::Vector3d{0.0, 0.0, 3.0}, true,
                                        Eigen::Vector3d::Zero());
  ASSERT_EQ(origin.type,
            px4_navigation_external_mode::MissionControllerEvent::Type::PublishGoal);
  ASSERT_TRUE(origin.waypoint_accepted);
  controller.onTrajectory(true, 0.3);

  const auto incoming_velocity = controller.update(
      0.4, Eigen::Vector3d{1.0, 0.0, 3.0}, true, Eigen::Vector3d{1.0, 0.0, 0.0});
  EXPECT_EQ(incoming_velocity.type,
            px4_navigation_external_mode::MissionControllerEvent::Type::PublishGoal);
  EXPECT_TRUE(incoming_velocity.waypoint_accepted);
  EXPECT_EQ(incoming_velocity.accepted_waypoint_index, 1U);
  EXPECT_EQ(incoming_velocity.waypoint_index, 2U);
}

TEST(MissionController, NativeTerminalHoldCanAdvanceCornerOnMeasuredAcceptance) {
  const auto path = writeMission(R"yaml(
mission:
  version: 1
  id: native_terminal_corner_hold
  frame: lio_odom
  waypoints:
    - id: origin
      position: [0.0, 0.0, 3.0]
      behavior: pass_through
      acceptance_radius_m: 0.4
    - id: corner
      position: [1.0, 0.0, 3.0]
      behavior: pass_through
      acceptance_radius_m: 0.4
    - id: finish
      position: [1.0, 1.0, 3.0]
      behavior: stop
      acceptance_radius_m: 0.4
  control:
    acceptance_speed_mps: 0.15
)yaml");
  const auto mission = px4_navigation_external_mode::loadMission(path.string(), "lio_odom");
  std::filesystem::remove(path);
  px4_navigation_external_mode::MissionController controller(mission);

  controller.activate(0.0);
  ASSERT_EQ(controller.update(0.0, std::nullopt).type,
            px4_navigation_external_mode::MissionControllerEvent::Type::PublishGoal);
  controller.onTrajectory(true, 0.0);
  ASSERT_TRUE(controller.update(0.1, Eigen::Vector3d{0.0, 0.0, 3.0}, true,
                                Eigen::Vector3d::Zero()).waypoint_accepted);
  controller.onTrajectory(true, 0.1);
  controller.onNativeTerminalHoldObserved();

  const auto moving = controller.update(0.2, Eigen::Vector3d{1.0, 0.0, 3.0}, true,
                                        Eigen::Vector3d{1.0, 0.0, 0.0});
  EXPECT_EQ(moving.type,
            px4_navigation_external_mode::MissionControllerEvent::Type::PublishGoal);
  expectWaypointAccepted(moving, 1U);
  EXPECT_EQ(moving.waypoint_index, 2U);
}

TEST(MissionController, CoincidentPassThroughStopRestoresTerminalReadiness) {
  const auto path = writeMission(R"yaml(
mission:
  version: 1
  id: coincident_terminal_readiness
  frame: lio_odom
  waypoints:
    - id: origin
      position: [0.0, 0.0, 3.0]
      behavior: pass_through
      acceptance_radius_m: 0.4
    - id: checkpoint
      position: [1.0, 0.0, 3.0]
      behavior: pass_through
      acceptance_radius_m: 0.4
    - id: finish
      position: [1.0, 0.0, 3.0]
      behavior: stop
      acceptance_radius_m: 0.4
  control:
    acceptance_speed_mps: 0.15
)yaml");
  const auto mission = px4_navigation_external_mode::loadMission(path.string(), "lio_odom");
  std::filesystem::remove(path);
  px4_navigation_external_mode::MissionController controller(mission);

  controller.activate(0.0);
  ASSERT_EQ(controller.update(0.0, std::nullopt).type,
            px4_navigation_external_mode::MissionControllerEvent::Type::PublishGoal);
  controller.onTrajectory(true, 0.1);
  ASSERT_TRUE(controller.update(0.2, Eigen::Vector3d{0.0, 0.0, 3.0}, true,
                                Eigen::Vector3d::Zero()).waypoint_accepted);
  controller.onTrajectory(true, 0.3);
  controller.onNativeSafetyTrajectoryObserved();
  controller.onNativeTerminalHoldObserved();

  EXPECT_TRUE(controller.terminalHoldPending());
  EXPECT_TRUE(controller.nativeTrajectoryReady());
  const auto moving = controller.update(
      0.4, Eigen::Vector3d{1.0, 0.0, 3.0}, true,
      Eigen::Vector3d{1.0, 0.0, 0.0});
  EXPECT_EQ(moving.type,
            px4_navigation_external_mode::MissionControllerEvent::Type::None);
  EXPECT_TRUE(controller.terminalHoldPending());

  const auto stop_goal = controller.update(
      0.5, Eigen::Vector3d{1.0, 0.0, 3.0}, true, Eigen::Vector3d::Zero());
  EXPECT_EQ(stop_goal.type,
            px4_navigation_external_mode::MissionControllerEvent::Type::PublishGoal);
  expectWaypointAccepted(stop_goal, 1U);
  EXPECT_EQ(stop_goal.waypoint_index, 2U);
}

TEST(MissionController, TerminalStopRequiresMeasuredPositionInsideAcceptanceRadius) {
  const auto path = writeMission(R"yaml(
mission:
  version: 1
  id: terminal_acceptance_gate
  frame: lio_odom
  waypoints:
    - id: finish
      position: [2.0, 0.0, 3.0]
      behavior: stop
      acceptance_radius_m: 0.4
      hold_s: 0.1
  control:
    acceptance_confirmation_s: 0.0
)yaml");
  const auto mission = px4_navigation_external_mode::loadMission(path.string(), "lio_odom");
  std::filesystem::remove(path);
  px4_navigation_external_mode::MissionController controller(mission);

  controller.activate(0.0);
  ASSERT_EQ(controller.update(0.0, std::nullopt).type,
            px4_navigation_external_mode::MissionControllerEvent::Type::PublishGoal);
  controller.onTrajectory(true, 0.0);

  const auto outside = controller.update(1.0, Eigen::Vector3d{20.0, 0.0, 3.0}, true,
                                         Eigen::Vector3d::Zero());
  EXPECT_EQ(outside.type,
            px4_navigation_external_mode::MissionControllerEvent::Type::None);
  expectNoWaypointAccepted(outside);
  EXPECT_EQ(controller.state(),
            px4_navigation_external_mode::MissionControllerState::ExecutingWaypoint);

  const auto inside = controller.update(1.1, Eigen::Vector3d{2.2, 0.0, 3.0}, true,
                                        Eigen::Vector3d::Zero());
  EXPECT_EQ(inside.type,
            px4_navigation_external_mode::MissionControllerEvent::Type::None);
  EXPECT_EQ(controller.state(), px4_navigation_external_mode::MissionControllerState::Holding);

  EXPECT_EQ(controller.update(1.15, Eigen::Vector3d{2.2, 0.0, 3.0}, true,
                              Eigen::Vector3d::Zero()).type,
            px4_navigation_external_mode::MissionControllerEvent::Type::None);
  EXPECT_EQ(controller.state(), px4_navigation_external_mode::MissionControllerState::Holding);

  const auto complete = controller.update(1.25, Eigen::Vector3d{2.2, 0.0, 3.0}, true,
                                          Eigen::Vector3d::Zero());
  EXPECT_EQ(complete.type,
            px4_navigation_external_mode::MissionControllerEvent::Type::Complete);
  expectWaypointAccepted(complete, 0U);
  EXPECT_EQ(controller.state(), px4_navigation_external_mode::MissionControllerState::Complete);
}

TEST(MissionController, TerminalStopRequiresLowMeasuredSpeed) {
  const auto path = writeMission(R"yaml(
mission:
  version: 1
  id: terminal_speed_gate
  frame: lio_odom
  waypoints:
    - id: finish
      position: [2.0, 0.0, 3.0]
      behavior: stop
      acceptance_radius_m: 0.4
      hold_s: 0.1
  control:
    acceptance_speed_mps: 0.2
    acceptance_confirmation_s: 0.0
)yaml");
  const auto mission = px4_navigation_external_mode::loadMission(path.string(), "lio_odom");
  std::filesystem::remove(path);
  px4_navigation_external_mode::MissionController controller(mission);

  controller.activate(0.0);
  ASSERT_EQ(controller.update(0.0, std::nullopt).type,
            px4_navigation_external_mode::MissionControllerEvent::Type::PublishGoal);
  controller.onTrajectory(true, 0.0);

  const auto fast = controller.update(0.1, Eigen::Vector3d{2.0, 0.0, 3.0}, true,
                                      Eigen::Vector3d{0.5, 0.0, 0.0});
  EXPECT_EQ(fast.type,
            px4_navigation_external_mode::MissionControllerEvent::Type::None);
  expectNoWaypointAccepted(fast);
  EXPECT_EQ(controller.state(),
            px4_navigation_external_mode::MissionControllerState::ExecutingWaypoint);

  const auto stopped = controller.update(0.2, Eigen::Vector3d{2.0, 0.0, 3.0}, true,
                                         Eigen::Vector3d::Zero());
  EXPECT_EQ(stopped.type,
            px4_navigation_external_mode::MissionControllerEvent::Type::None);
  expectNoWaypointAccepted(stopped);
  EXPECT_EQ(controller.state(), px4_navigation_external_mode::MissionControllerState::Holding);

  const auto complete = controller.update(0.31, Eigen::Vector3d{2.0, 0.0, 3.0}, true,
                                          Eigen::Vector3d::Zero());
  EXPECT_EQ(complete.type,
            px4_navigation_external_mode::MissionControllerEvent::Type::Complete);
  expectWaypointAccepted(complete, 0U);
}

TEST(MissionController, NativeReadinessCannotClearPendingTerminalStopHold) {
  const auto path = writeMission(R"yaml(
mission:
  version: 1
  id: terminal_hold_ownership
  frame: lio_odom
  waypoints:
    - id: finish
      position: [2.0, 0.0, 3.0]
      behavior: stop
      acceptance_radius_m: 0.4
  control:
    acceptance_speed_mps: 0.2
    acceptance_confirmation_s: 0.5
)yaml");
  const auto mission = px4_navigation_external_mode::loadMission(path.string(), "lio_odom");
  std::filesystem::remove(path);
  px4_navigation_external_mode::MissionController controller(mission);

  controller.activate(0.0);
  ASSERT_EQ(controller.update(0.0, std::nullopt).type,
            px4_navigation_external_mode::MissionControllerEvent::Type::PublishGoal);
  controller.onNativeSafetyTrajectoryObserved();
  controller.onNativeTerminalHoldObserved();
  ASSERT_TRUE(controller.terminalHoldPending());
  ASSERT_TRUE(controller.nativeTrajectoryReady());

  // A generic MAIN readiness notification may race the terminal BACKUP
  // callback, but it must not release the STOP hold while the vehicle is
  // still moving inside the acceptance ball.
  controller.onNativeTrajectoryReady();
  EXPECT_TRUE(controller.terminalHoldPending());
  const auto moving = controller.update(0.1, Eigen::Vector3d{2.0, 0.0, 3.0}, true,
                                        Eigen::Vector3d{0.5, 0.0, 0.0});
  EXPECT_EQ(moving.type,
            px4_navigation_external_mode::MissionControllerEvent::Type::None);
  expectNoWaypointAccepted(moving);
  EXPECT_EQ(controller.state(),
            px4_navigation_external_mode::MissionControllerState::ExecutingWaypoint);
  EXPECT_TRUE(controller.terminalHoldPending());

  const auto premature = controller.update(0.2, Eigen::Vector3d{2.0, 0.0, 3.0}, true,
                                           Eigen::Vector3d::Zero());
  EXPECT_EQ(premature.type,
            px4_navigation_external_mode::MissionControllerEvent::Type::None);
  EXPECT_TRUE(controller.terminalHoldPending());
  EXPECT_EQ(controller.state(), px4_navigation_external_mode::MissionControllerState::ExecutingWaypoint);

  const auto lost_speed_gate = controller.update(0.3, Eigen::Vector3d{2.0, 0.0, 3.0}, true,
                                                 Eigen::Vector3d{0.5, 0.0, 0.0});
  EXPECT_EQ(lost_speed_gate.type,
            px4_navigation_external_mode::MissionControllerEvent::Type::None);
  EXPECT_TRUE(controller.terminalHoldPending());

  const auto settling = controller.update(0.9, Eigen::Vector3d{2.0, 0.0, 3.0}, true,
                                          Eigen::Vector3d::Zero());
  EXPECT_EQ(settling.type,
            px4_navigation_external_mode::MissionControllerEvent::Type::None);
  EXPECT_TRUE(controller.terminalHoldPending());

  const auto settled = controller.update(1.5, Eigen::Vector3d{2.0, 0.0, 3.0}, true,
                                         Eigen::Vector3d::Zero());
  EXPECT_EQ(settled.type,
            px4_navigation_external_mode::MissionControllerEvent::Type::None);
  EXPECT_FALSE(controller.terminalHoldPending());
  EXPECT_EQ(controller.state(), px4_navigation_external_mode::MissionControllerState::Holding);
}

TEST(MissionController, HoldingDoesNotReplanForTransientSpeedOvershootInsideAcceptance) {
  const auto path = writeMission(R"yaml(
mission:
  version: 1
  id: holding_speed_hysteresis
  frame: lio_odom
  waypoints:
    - id: finish
      position: [2.0, 0.0, 3.0]
      behavior: stop
      acceptance_radius_m: 0.4
      hold_s: 0.2
  control:
    acceptance_speed_mps: 0.15
    acceptance_confirmation_s: 0.0
)yaml");
  const auto mission = px4_navigation_external_mode::loadMission(path.string(), "lio_odom");
  std::filesystem::remove(path);
  px4_navigation_external_mode::MissionController controller(mission);

  controller.activate(0.0);
  ASSERT_EQ(controller.update(0.0, std::nullopt).type,
            px4_navigation_external_mode::MissionControllerEvent::Type::PublishGoal);
  controller.onTrajectory(true, 0.0);
  ASSERT_EQ(controller.update(0.1, Eigen::Vector3d{2.0, 0.0, 3.0}, true,
                              Eigen::Vector3d::Zero()).type,
            px4_navigation_external_mode::MissionControllerEvent::Type::None);
  ASSERT_EQ(controller.state(), px4_navigation_external_mode::MissionControllerState::Holding);

  const auto transient = controller.update(0.2, Eigen::Vector3d{2.0, 0.0, 3.0}, true,
                                           Eigen::Vector3d{0.3, 0.0, 0.0});
  EXPECT_EQ(transient.type,
            px4_navigation_external_mode::MissionControllerEvent::Type::None);
  EXPECT_FALSE(transient.waypoint_accepted);
  EXPECT_EQ(controller.state(), px4_navigation_external_mode::MissionControllerState::Holding);

  const auto still_holding = controller.update(0.3, Eigen::Vector3d{2.0, 0.0, 3.0}, true,
                                               Eigen::Vector3d::Zero());
  EXPECT_EQ(still_holding.type,
            px4_navigation_external_mode::MissionControllerEvent::Type::None);
  EXPECT_EQ(controller.state(), px4_navigation_external_mode::MissionControllerState::Holding);

  const auto complete = controller.update(0.41, Eigen::Vector3d{2.0, 0.0, 3.0}, true,
                                          Eigen::Vector3d::Zero());
  EXPECT_EQ(complete.type,
            px4_navigation_external_mode::MissionControllerEvent::Type::Complete);
  expectWaypointAccepted(complete, 0U);
}

TEST(MissionController, LostVehicleCannotCompleteMission) {
  const auto path = writeMission(R"yaml(
mission:
  version: 1
  id: lost_vehicle_no_complete
  frame: lio_odom
  waypoints:
    - id: finish
      position: [2.0, 0.0, 3.0]
      behavior: stop
      acceptance_radius_m: 0.4
      hold_s: 0.1
)yaml");
  const auto mission = px4_navigation_external_mode::loadMission(path.string(), "lio_odom");
  std::filesystem::remove(path);
  px4_navigation_external_mode::MissionController controller(mission);

  controller.activate(0.0);
  ASSERT_EQ(controller.update(0.0, std::nullopt).type,
            px4_navigation_external_mode::MissionControllerEvent::Type::PublishGoal);
  controller.onTrajectory(true, 0.0);

  for (double now_s : {1.0, 2.0, 5.0}) {
    const auto event = controller.update(now_s, Eigen::Vector3d{100.0, -80.0, 20.0}, true,
                                         Eigen::Vector3d::Zero());
    EXPECT_EQ(event.type,
              px4_navigation_external_mode::MissionControllerEvent::Type::None);
    expectNoWaypointAccepted(event);
    EXPECT_NE(controller.state(), px4_navigation_external_mode::MissionControllerState::Complete);
  }
  EXPECT_EQ(controller.activeWaypointIndex(), 0U);
}

TEST(MissionController, RetriesFailedRequestWithoutRegressingCorrelation) {
  const auto path = writeMission(kValidMission);
  const auto mission = px4_navigation_external_mode::loadMission(path.string(), "lio_odom");
  std::filesystem::remove(path);
  px4_navigation_external_mode::MissionController controller(mission);

  controller.activate(10.0);
  auto event = controller.update(10.0, std::nullopt);
  ASSERT_EQ(event.type, px4_navigation_external_mode::MissionControllerEvent::Type::PublishGoal);
  ASSERT_EQ(event.request_id, 1U);

  controller.onTrajectory(false, 10.1);
  event = controller.update(10.1, std::nullopt);
  EXPECT_EQ(event.type,
            px4_navigation_external_mode::MissionControllerEvent::Type::RequestPositionControl);
  EXPECT_EQ(controller.state(), px4_navigation_external_mode::MissionControllerState::Paused);
}

TEST(MissionController, SafetyFallbackBrakesAndRequestsPositionControl) {
  const auto path = writeMission(kValidMission);
  const auto mission = px4_navigation_external_mode::loadMission(path.string(), "lio_odom");
  std::filesystem::remove(path);
  px4_navigation_external_mode::MissionController controller(mission);

  controller.activate(0.0);
  ASSERT_EQ(controller.update(0.0, std::nullopt).type,
            px4_navigation_external_mode::MissionControllerEvent::Type::PublishGoal);

  controller.onTrajectory(true, 1U, 0.0);
  EXPECT_EQ(controller.state(),
            px4_navigation_external_mode::MissionControllerState::Braking);
  EXPECT_EQ(controller.update(0.1, Eigen::Vector3d{1.0, 0.0, 3.0}, true,
                              Eigen::Vector3d{1.0, 0.0, 0.0}).type,
            px4_navigation_external_mode::MissionControllerEvent::Type::None);
  EXPECT_EQ(controller.update(0.25, std::nullopt, true,
                              Eigen::Vector3d::Zero()).type,
            px4_navigation_external_mode::MissionControllerEvent::Type::None);
  const auto handover = controller.update(0.80, std::nullopt, true,
                                           Eigen::Vector3d::Zero());
  EXPECT_EQ(handover.type,
            px4_navigation_external_mode::MissionControllerEvent::Type::RequestPositionControl);
  EXPECT_EQ(controller.state(), px4_navigation_external_mode::MissionControllerState::Paused);
}

TEST(MissionController, RejectsUnrepresentableBrakingDeadline) {
  const auto path = writeMission(kValidMission);
  const auto mission = px4_navigation_external_mode::loadMission(path.string(), "lio_odom");
  std::filesystem::remove(path);
  px4_navigation_external_mode::MissionController controller(mission);

  controller.activate(1.0);
  ASSERT_EQ(controller.update(1.0, std::nullopt).type,
            px4_navigation_external_mode::MissionControllerEvent::Type::PublishGoal);
  controller.onTrajectory(true, 1U, 2U, std::numeric_limits<double>::max(),
                          std::numeric_limits<double>::max());
  EXPECT_EQ(controller.state(),
            px4_navigation_external_mode::MissionControllerState::Paused);
  EXPECT_EQ(controller.update(2.0, std::nullopt, true, Eigen::Vector3d::Zero()).type,
            px4_navigation_external_mode::MissionControllerEvent::Type::RequestPositionControl);
}

TEST(MissionController, RejectsMalformedSafetyStopDuration) {
  for (const double duration : {-1.0, std::numeric_limits<double>::quiet_NaN(),
                                std::numeric_limits<double>::infinity()}) {
    const auto path = writeMission(kValidMission);
    const auto mission = px4_navigation_external_mode::loadMission(path.string(), "lio_odom");
    std::filesystem::remove(path);
    px4_navigation_external_mode::MissionController controller(mission);

    controller.activate(0.0);
    ASSERT_EQ(controller.update(0.0, std::nullopt).type,
              px4_navigation_external_mode::MissionControllerEvent::Type::PublishGoal);
    controller.onTrajectory(true, 1U, 2U, 0.0, duration);
    EXPECT_EQ(controller.state(),
              px4_navigation_external_mode::MissionControllerState::Paused);
    EXPECT_EQ(controller.update(0.1, std::nullopt, true, Eigen::Vector3d::Zero()).type,
              px4_navigation_external_mode::MissionControllerEvent::Type::RequestPositionControl);
  }
}

TEST(MissionController, RejectsUnknownSuccessfulTrajectoryMetadata) {
  const auto path = writeMission(kValidMission);
  const auto mission = px4_navigation_external_mode::loadMission(path.string(), "lio_odom");
  std::filesystem::remove(path);
  px4_navigation_external_mode::MissionController controller(mission);

  controller.activate(0.0);
  ASSERT_EQ(controller.update(0.0, std::nullopt).type,
            px4_navigation_external_mode::MissionControllerEvent::Type::PublishGoal);
  controller.onTrajectory(true, 99U, 99U, 0.0, 0.0);
  EXPECT_EQ(controller.state(),
            px4_navigation_external_mode::MissionControllerState::Paused);
  EXPECT_EQ(controller.update(0.1, std::nullopt, true, Eigen::Vector3d::Zero()).type,
            px4_navigation_external_mode::MissionControllerEvent::Type::RequestPositionControl);
}

TEST(MissionController, CompletedBackupInsidePassThroughAcceptanceAdvances) {
  const auto path = writeMission(R"yaml(
mission:
  version: 1
  id: completed_backup_pass_through
  frame: lio_odom
  waypoints:
    - id: middle
      position: [1.0, 0.0, 3.0]
      behavior: pass_through
      acceptance_radius_m: 0.9
    - id: finish
      position: [2.0, 0.0, 3.0]
      behavior: stop
      acceptance_radius_m: 0.4
  control:
    acceptance_speed_mps: 0.15
)yaml");
  const auto mission = px4_navigation_external_mode::loadMission(path.string(), "lio_odom");
  std::filesystem::remove(path);
  px4_navigation_external_mode::MissionController controller(mission);

  controller.activate(0.0);
  ASSERT_EQ(controller.update(0.0, std::nullopt).type,
            px4_navigation_external_mode::MissionControllerEvent::Type::PublishGoal);

  // A completed native BACKUP is a position hold. If the measured vehicle and
  // the certified endpoint are already inside this pass-through waypoint,
  // retain the hold long enough for MissionController to apply its normal
  // measured-speed acceptance gate instead of forcing PX4 Hold.
  controller.onTrajectory(true, 1U, 2U, 0.0, 0.0);
  const auto event = controller.update(0.1, Eigen::Vector3d{1.2, 0.0, 3.0}, true,
                                       Eigen::Vector3d::Zero());
  EXPECT_EQ(event.type,
            px4_navigation_external_mode::MissionControllerEvent::Type::PublishGoal);
  expectWaypointAccepted(event, 0U);
  EXPECT_EQ(event.waypoint_index, 1U);
  EXPECT_EQ(controller.activeWaypointIndex(), 1U);
}

TEST(MissionController, CertifiedPassThroughSuffixStopAllowsMeasuredProgression) {
  const auto path = writeMission(R"yaml(
mission:
  version: 1
  id: certified_suffix_pass_through
  frame: lio_odom
  waypoints:
    - id: first
      position: [1.0, 0.0, 3.0]
      behavior: pass_through
      acceptance_radius_m: 0.4
    - id: second
      position: [2.0, 0.0, 3.0]
      behavior: pass_through
      acceptance_radius_m: 0.4
  control:
    acceptance_speed_mps: 0.15
)yaml");
  const auto mission = px4_navigation_external_mode::loadMission(path.string(), "lio_odom");
  std::filesystem::remove(path);
  px4_navigation_external_mode::MissionController controller(mission);

  controller.activate(0.0);
  ASSERT_EQ(controller.update(0.0, std::nullopt).type,
            px4_navigation_external_mode::MissionControllerEvent::Type::PublishGoal);
  controller.onTrajectory(true, 0U, 0U, 0.0, 0.0);
  const auto first = controller.update(0.1, Eigen::Vector3d{1.0, 0.0, 3.0}, true,
                                       Eigen::Vector3d::Zero());
  ASSERT_TRUE(first.waypoint_accepted);
  EXPECT_EQ(first.waypoint_index, 1U);
  EXPECT_EQ(controller.activeWaypointIndex(), 1U);

  // No trajectory callback for the new waypoint is available yet. The
  // certified suffix witness enables only the normal measured acceptance gate.
  controller.onCertifiedPassThroughSuffixStop();
  const auto second = controller.update(0.2, Eigen::Vector3d{2.0, 0.0, 3.0}, true,
                                        Eigen::Vector3d::Zero());
  EXPECT_EQ(second.type,
            px4_navigation_external_mode::MissionControllerEvent::Type::Complete);
  expectWaypointAccepted(second, 1U);
}

TEST(MissionController, SafetyTrajectoryCannotAdvancePassThroughUntilCertifiedStop) {
  const auto path = writeMission(R"yaml(
mission:
  version: 1
  id: safety_readiness_ownership
  frame: lio_odom
  waypoints:
    - id: first
      position: [1.0, 0.0, 3.0]
      behavior: pass_through
      acceptance_radius_m: 0.4
    - id: second
      position: [2.0, 0.0, 3.0]
      behavior: pass_through
      acceptance_radius_m: 0.4
  control:
    acceptance_speed_mps: 0.15
)yaml");
  const auto mission = px4_navigation_external_mode::loadMission(path.string(), "lio_odom");
  std::filesystem::remove(path);
  px4_navigation_external_mode::MissionController controller(mission);

  controller.activate(0.0);
  ASSERT_EQ(controller.update(0.0, std::nullopt).type,
            px4_navigation_external_mode::MissionControllerEvent::Type::PublishGoal);
  controller.onNativeTrajectoryReady();
  const auto first = controller.update(0.1, Eigen::Vector3d{1.0, 0.0, 3.0}, true,
                                       Eigen::Vector3d::Zero());
  ASSERT_TRUE(first.waypoint_accepted);
  EXPECT_EQ(first.waypoint_index, 1U);

  // A BACKUP/EMERGENCY trajectory is not a nominal readiness witness. Even
  // when the vehicle is inside the next checkpoint, do not advance until the
  // suffix-stop certificate arrives.
  controller.onNativeSafetyTrajectoryObserved();
  const auto blocked = controller.update(0.2, Eigen::Vector3d{2.0, 0.0, 3.0}, true,
                                         Eigen::Vector3d::Zero());
  EXPECT_EQ(blocked.type, px4_navigation_external_mode::MissionControllerEvent::Type::None);
  EXPECT_EQ(controller.activeWaypointIndex(), 1U);

  controller.onCertifiedPassThroughSuffixStop();
  const auto progressed = controller.update(0.3, Eigen::Vector3d{2.0, 0.0, 3.0}, true,
                                            Eigen::Vector3d::Zero());
  EXPECT_EQ(progressed.type,
            px4_navigation_external_mode::MissionControllerEvent::Type::Complete);
  expectWaypointAccepted(progressed, 1U);
}

TEST(MissionController, SafetyStopAllowsRollingRouteReplacement) {
  const auto path = writeMission(R"yaml(
mission:
  version: 1
  id: safety_replan_grace
  frame: lio_odom
  waypoints:
    - id: finish
      position: [2.0, 0.0, 3.0]
      behavior: stop
      acceptance_radius_m: 0.4
      hold_s: 0.1
)yaml");
  const auto mission = px4_navigation_external_mode::loadMission(path.string(), "lio_odom");
  std::filesystem::remove(path);
  px4_navigation_external_mode::MissionController controller(mission);

  controller.activate(0.0);
  ASSERT_EQ(controller.update(0.0, std::nullopt).type,
            px4_navigation_external_mode::MissionControllerEvent::Type::PublishGoal);
  controller.onTrajectory(true, 1U, 2U, 0.0, 0.2);
  EXPECT_EQ(controller.update(0.3, std::nullopt, true, Eigen::Vector3d::Zero()).type,
            px4_navigation_external_mode::MissionControllerEvent::Type::None);
  EXPECT_EQ(controller.update(0.6, std::nullopt, true, Eigen::Vector3d::Zero()).type,
            px4_navigation_external_mode::MissionControllerEvent::Type::None);
  EXPECT_EQ(controller.state(), px4_navigation_external_mode::MissionControllerState::Braking);

  // A verified route arriving before the stationary confirmation completes
  // resumes execution instead of handing the vehicle to POSCTL.
  controller.onTrajectory(true, 1U, 1U, 0.7, 0.8);
  EXPECT_EQ(controller.state(), px4_navigation_external_mode::MissionControllerState::ExecutingWaypoint);
  EXPECT_EQ(controller.update(0.8, std::nullopt, true, Eigen::Vector3d::Zero()).type,
            px4_navigation_external_mode::MissionControllerEvent::Type::None);
}

TEST(MissionController, SafetyStopFailsClosedAfterConfirmation) {
  const auto path = writeMission(R"yaml(
mission:
  version: 1
  id: safety_replan_grace_timeout
  frame: lio_odom
  waypoints:
    - id: finish
      position: [2.0, 0.0, 3.0]
      behavior: stop
      acceptance_radius_m: 0.4
)yaml");
  const auto mission = px4_navigation_external_mode::loadMission(path.string(), "lio_odom");
  std::filesystem::remove(path);
  px4_navigation_external_mode::MissionController controller(mission);
  controller.activate(0.0);
  ASSERT_EQ(controller.update(0.0, std::nullopt).type,
            px4_navigation_external_mode::MissionControllerEvent::Type::PublishGoal);
  controller.onTrajectory(true, 1U, 2U, 0.0, 0.2);
  EXPECT_EQ(controller.update(0.4, std::nullopt, true, Eigen::Vector3d::Zero()).type,
            px4_navigation_external_mode::MissionControllerEvent::Type::None);
  EXPECT_EQ(controller.update(1.5, std::nullopt, true, Eigen::Vector3d::Zero()).type,
            px4_navigation_external_mode::MissionControllerEvent::Type::RequestPositionControl);
}

TEST(MissionController, BrakingWaitsForTrajectoryEndBeforeConfirmation) {
  const auto path = writeMission(kValidMission);
  const auto mission = px4_navigation_external_mode::loadMission(path.string(), "lio_odom");
  std::filesystem::remove(path);
  px4_navigation_external_mode::MissionController controller(mission);

  controller.activate(0.0);
  ASSERT_EQ(controller.update(0.0, std::nullopt).type,
            px4_navigation_external_mode::MissionControllerEvent::Type::PublishGoal);
  controller.onTrajectory(true, 1U, 2U, 0.0, 2.0);
  EXPECT_EQ(controller.update(1.0, std::nullopt, true, Eigen::Vector3d::Zero()).type,
            px4_navigation_external_mode::MissionControllerEvent::Type::None);
  EXPECT_EQ(controller.update(2.4, std::nullopt, true, Eigen::Vector3d::Zero()).type,
            px4_navigation_external_mode::MissionControllerEvent::Type::None);
  EXPECT_EQ(controller.update(3.0, std::nullopt, true, Eigen::Vector3d::Zero()).type,
            px4_navigation_external_mode::MissionControllerEvent::Type::RequestPositionControl);
}

TEST(MissionController, BrakingReplacementRestartsItsOwnConfirmationWindow) {
  const auto path = writeMission(kValidMission);
  const auto mission = px4_navigation_external_mode::loadMission(path.string(), "lio_odom");
  std::filesystem::remove(path);
  px4_navigation_external_mode::MissionController controller(mission);

  controller.activate(0.0);
  ASSERT_EQ(controller.update(0.0, std::nullopt).type,
            px4_navigation_external_mode::MissionControllerEvent::Type::PublishGoal);
  controller.onTrajectory(true, 1U, 2U, 0.0, 1.0);
  controller.onTrajectory(true, 1U, 2U, 0.8, 2.0);
  EXPECT_EQ(controller.state(), px4_navigation_external_mode::MissionControllerState::Braking);
  EXPECT_EQ(controller.update(1.9, std::nullopt, true, Eigen::Vector3d::Zero()).type,
            px4_navigation_external_mode::MissionControllerEvent::Type::None);
  EXPECT_EQ(controller.update(3.4, std::nullopt, true, Eigen::Vector3d::Zero()).type,
            px4_navigation_external_mode::MissionControllerEvent::Type::None);
  EXPECT_EQ(controller.update(3.9, std::nullopt, true, Eigen::Vector3d::Zero()).type,
            px4_navigation_external_mode::MissionControllerEvent::Type::RequestPositionControl);
}

TEST(MissionController, RepeatedZeroDurationSafetyRefreshPreservesConfirmation) {
  const auto path = writeMission(kValidMission);
  const auto mission = px4_navigation_external_mode::loadMission(path.string(), "lio_odom");
  std::filesystem::remove(path);
  px4_navigation_external_mode::MissionController controller(mission);

  controller.activate(0.0);
  ASSERT_EQ(controller.update(0.0, std::nullopt).type,
            px4_navigation_external_mode::MissionControllerEvent::Type::PublishGoal);
  controller.onTrajectory(true, 1U, 2U, 0.0, 0.0);
  EXPECT_EQ(controller.update(0.1, std::nullopt, true, Eigen::Vector3d::Zero()).type,
            px4_navigation_external_mode::MissionControllerEvent::Type::None);

  controller.onTrajectory(true, 1U, 2U, 0.2, 0.0);
  EXPECT_EQ(controller.update(0.4, std::nullopt, true, Eigen::Vector3d::Zero()).type,
            px4_navigation_external_mode::MissionControllerEvent::Type::None);
  EXPECT_EQ(controller.update(0.7, std::nullopt, true, Eigen::Vector3d::Zero()).type,
            px4_navigation_external_mode::MissionControllerEvent::Type::RequestPositionControl);
}

TEST(MissionController, BrakingReplacementFailureRequestsPositionControl) {
  const auto path = writeMission(kValidMission);
  const auto mission = px4_navigation_external_mode::loadMission(path.string(), "lio_odom");
  std::filesystem::remove(path);
  px4_navigation_external_mode::MissionController controller(mission);

  controller.activate(0.0);
  ASSERT_EQ(controller.update(0.0, std::nullopt).type,
            px4_navigation_external_mode::MissionControllerEvent::Type::PublishGoal);
  controller.onTrajectory(true, 1U, 2U, 0.0, 2.0);
  controller.onTrajectory(false, 1U, 2U, 0.5, 0.0);
  EXPECT_EQ(controller.state(), px4_navigation_external_mode::MissionControllerState::Paused);
  EXPECT_EQ(controller.update(0.6, std::nullopt, true, Eigen::Vector3d::Zero()).type,
            px4_navigation_external_mode::MissionControllerEvent::Type::RequestPositionControl);
}

TEST(MissionController, SafetyRouteCountsAsWaypointTrajectory) {
  const auto path = writeMission(kValidMission);
  const auto mission = px4_navigation_external_mode::loadMission(path.string(), "lio_odom");
  std::filesystem::remove(path);
  px4_navigation_external_mode::MissionController controller(mission);

  controller.activate(0.0);
  ASSERT_EQ(controller.update(0.0, std::nullopt).type,
            px4_navigation_external_mode::MissionControllerEvent::Type::PublishGoal);

  controller.onTrajectory(true, 1U, 1U, 0.0);
  EXPECT_EQ(controller.update(0.1, Eigen::Vector3d{1.0, 0.0, 3.0}, true,
                              Eigen::Vector3d::Zero()).type,
            px4_navigation_external_mode::MissionControllerEvent::Type::None);
  EXPECT_EQ(controller.state(), px4_navigation_external_mode::MissionControllerState::Holding);
}

TEST(MissionController, DeactivationStopsFurtherGoalPublication) {
  const auto path = writeMission(kValidMission);
  const auto mission = px4_navigation_external_mode::loadMission(path.string(), "lio_odom");
  std::filesystem::remove(path);
  px4_navigation_external_mode::MissionController controller(mission);

  controller.activate(0.0);
  ASSERT_EQ(controller.update(0.0, std::nullopt).type,
            px4_navigation_external_mode::MissionControllerEvent::Type::PublishGoal);
  controller.deactivate();

  EXPECT_EQ(controller.state(), px4_navigation_external_mode::MissionControllerState::Idle);
  EXPECT_EQ(controller.update(1.0, std::nullopt).type,
            px4_navigation_external_mode::MissionControllerEvent::Type::None);
}

TEST(MissionController, ReactivationResumesCheckpointWaypoint) {
  const auto path = writeMission(kValidMission);
  const auto mission = px4_navigation_external_mode::loadMission(path.string(), "lio_odom");
  std::filesystem::remove(path);
  px4_navigation_external_mode::MissionController controller(mission);

  controller.activate(0.0);
  ASSERT_EQ(controller.update(0.0, std::nullopt).type,
            px4_navigation_external_mode::MissionControllerEvent::Type::PublishGoal);
  controller.onTrajectory(true, 0.0);
  ASSERT_EQ(controller.update(0.1, Eigen::Vector3d{1.0, 0.0, 3.0}, true,
                              Eigen::Vector3d::Zero()).type,
            px4_navigation_external_mode::MissionControllerEvent::Type::None);
  controller.deactivate();
  controller.activate(1.0);
  const auto resumed = controller.update(1.0, std::nullopt);
  EXPECT_EQ(resumed.type,
            px4_navigation_external_mode::MissionControllerEvent::Type::PublishGoal);
  EXPECT_EQ(resumed.waypoint_index, 0U);
  EXPECT_GT(resumed.request_id, 1U);
}
