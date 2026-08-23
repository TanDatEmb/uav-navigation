#include <chrono>
#include <cmath>
#include <filesystem>
#include <fstream>

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
    replan_rate_hz: 5.0
    max_velocity_mps: 1.0
    max_acceleration_mps2: 2.0
    max_jerk_mps3: 6.0
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
  EXPECT_DOUBLE_EQ(mission.planning.max_jerk_mps3, 6.0);
  EXPECT_DOUBLE_EQ(mission.control.pass_through_lookahead_m, 0.0);
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
  // The native SUPER callback may arrive immediately after goal publication,
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

TEST(MissionController, PassThroughLookaheadDoesNotBypassAcceptanceRadius) {
  const auto path = writeMission(R"yaml(
mission:
  version: 1
  id: lookahead_route
  frame: lio_odom
  waypoints:
    - {id: origin, position: [0.0, 0.0, 3.0], acceptance_radius_m: 0.4}
    - {id: middle, position: [1.0, 0.0, 3.0], acceptance_radius_m: 0.4}
    - {id: finish, position: [2.0, 0.0, 3.0], acceptance_radius_m: 0.4}
  control:
    pass_through_lookahead_m: 1.5
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

TEST(MissionController, PassThroughCornerWaitsForOutgoingVelocityAlignment) {
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
            px4_navigation_external_mode::MissionControllerEvent::Type::None);
  EXPECT_FALSE(incoming_velocity.waypoint_accepted);
  EXPECT_EQ(controller.activeWaypointIndex(), 1U);

  const auto outgoing_velocity = controller.update(
      0.5, Eigen::Vector3d{1.0, 0.0, 3.0}, true, Eigen::Vector3d{0.0, 1.0, 0.0});
  EXPECT_EQ(outgoing_velocity.type,
            px4_navigation_external_mode::MissionControllerEvent::Type::PublishGoal);
  EXPECT_TRUE(outgoing_velocity.waypoint_accepted);
  EXPECT_EQ(outgoing_velocity.accepted_waypoint_index, 1U);
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

TEST(MissionController, SafetyStopGraceAllowsRollingRouteReplacement) {
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
  control:
    safety_stop_replan_grace_s: 2.0
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
  EXPECT_EQ(controller.update(1.0, std::nullopt, true, Eigen::Vector3d::Zero()).type,
            px4_navigation_external_mode::MissionControllerEvent::Type::None);
  EXPECT_EQ(controller.state(), px4_navigation_external_mode::MissionControllerState::Braking);

  // A verified route arriving during the grace window resumes execution
  // instead of handing the vehicle to POSCTL.
  controller.onTrajectory(true, 1U, 1U, 1.1, 0.8);
  EXPECT_EQ(controller.state(), px4_navigation_external_mode::MissionControllerState::ExecutingWaypoint);
  EXPECT_EQ(controller.update(1.2, std::nullopt, true, Eigen::Vector3d::Zero()).type,
            px4_navigation_external_mode::MissionControllerEvent::Type::None);
}

TEST(MissionController, SafetyStopGraceStillFailsClosedAfterTimeout) {
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
  control:
    safety_stop_replan_grace_s: 1.0
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
