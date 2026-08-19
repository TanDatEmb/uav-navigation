#include <chrono>
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
    unknown_policy: blocked
  control:
    output: velocity
    acceptance_confirmation_s: 0.0
)yaml";

}  // namespace

TEST(MissionLoader, LoadsCompatibleControlContract) {
  const auto path = writeMission(kValidMission);
  const auto mission = px4_navigation_external_mode::loadMission(path.string(), "lio_odom");
  std::filesystem::remove(path);
  EXPECT_EQ(mission.id, "test_route");
  ASSERT_EQ(mission.waypoints.size(), 2U);
  EXPECT_DOUBLE_EQ(mission.waypoints[0].position_enu.x(), 1.0);
  EXPECT_DOUBLE_EQ(mission.waypoints[0].hold_s, 0.1);
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

  event = controller.update(0.2, Eigen::Vector3d{1.0, 0.0, 3.0}, true,
                            Eigen::Vector3d::Zero());
  EXPECT_EQ(event.type, px4_navigation_external_mode::MissionControllerEvent::Type::PublishGoal);
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
