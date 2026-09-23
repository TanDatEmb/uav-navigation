#include "navigation_runtime/mission_progress.hpp"

#include <gtest/gtest.h>
#include <tuple>
#include <vector>

namespace navigation_runtime {
namespace {
navigation_mission::Mission mission() {
  navigation_mission::Mission result;
  result.id = "core-mission";
  result.frame = "lio_odom";
  result.control.acceptance_speed_mps = 0.2;
  result.control.acceptance_confirmation_s = 0.5;
  navigation_mission::MissionWaypoint pass;
  pass.id = "pass";
  pass.position_enu = Eigen::Vector3d{0, 0, 3};
  pass.acceptance_radius_m = 0.3;
  pass.behavior = navigation_mission::MissionWaypoint::Behavior::PassThrough;
  navigation_mission::MissionWaypoint stop;
  stop.id = "stop";
  stop.position_enu = Eigen::Vector3d{5, 0, 3};
  stop.acceptance_radius_m = 0.4;
  stop.hold_s = 0.2;
  stop.behavior = navigation_mission::MissionWaypoint::Behavior::Stop;
  result.waypoints = {pass, stop};
  return result;
}

MissionMeasuredSample measured(double x, std::int64_t milliseconds,
                               std::uint64_t sequence, double speed = 2.0) {
  return {Eigen::Vector3d{x, 0, 3}, Eigen::Vector3d{speed, 0, 0},
          milliseconds * 1'000'000, sequence, 1U};
}

MissionProgressDecision observe(MissionProgress& progress,
                                const MissionMeasuredSample& sample) {
  return progress.observeMeasured(sample, sample.source_stamp_ns);
}

MissionContinuationWitness mainWitness(const MissionProgress& progress,
                                        std::int64_t valid_until_ms = 2000) {
  return {progress.routeIdentity(), progress.currentGate(),
          MissionContinuationWitness::Kind::Main, 4U, 1'000'000,
          valid_until_ms * 1'000'000};
}

navigation_mission::Mission threeGateMission() {
  auto result = mission();
  auto intermediate = result.waypoints[0];
  intermediate.id = "intermediate";
  intermediate.position_enu.x() = 1.0;
  result.waypoints.insert(result.waypoints.begin() + 1, intermediate);
  return result;
}

void acceptInitialPass(MissionProgress& progress) {
  (void)observe(progress, measured(0.0, 100, 1, 0.0));
  EXPECT_EQ(progress.activate().kind, MissionProgressDecision::Kind::Goal);
  EXPECT_EQ(observe(progress, measured(0.0, 110, 2, 0.0)).kind,
            MissionProgressDecision::Kind::Goal);
  EXPECT_EQ(progress.currentGate().waypoint_index, 1U);
}

TEST(MissionProgressTest, CrossingBeforeContinuationSurvivesLeavingBall) {
  auto definition = mission();
  definition.waypoints[0].position_enu.x() = 1.0;
  MissionProgress progress(definition);
  progress.resetIdentity(1U, 1U);
  EXPECT_EQ(observe(progress,measured(0.0, 100, 1)).kind,
            MissionProgressDecision::Kind::None);
  EXPECT_EQ(progress.activate().kind, MissionProgressDecision::Kind::Goal);
  EXPECT_EQ(observe(progress,measured(0.8, 150, 2)).kind,
            MissionProgressDecision::Kind::None);
  ASSERT_TRUE(progress.crossing().has_value());
  EXPECT_EQ(observe(progress,measured(1.9, 200, 3)).kind,
            MissionProgressDecision::Kind::None);
  ASSERT_TRUE(progress.crossing().has_value());
  const auto decision = progress.observeContinuation(mainWitness(progress), 210'000'000);
  EXPECT_EQ(decision.kind, MissionProgressDecision::Kind::Goal);
  ASSERT_TRUE(decision.accepted.has_value());
  EXPECT_EQ(decision.accepted->waypoint_index, 0U);
  EXPECT_EQ(progress.currentGate().waypoint_index, 1U);
  EXPECT_FALSE(progress.crossing().has_value());
}

TEST(MissionProgressTest, ContinuationBeforeSkippedBallCrossing) {
  auto definition = mission();
  auto intermediate = definition.waypoints[0];
  intermediate.id = "intermediate";
  intermediate.position_enu.x() = 1.0;
  definition.waypoints.insert(definition.waypoints.begin() + 1, intermediate);
  MissionProgress progress(definition);
  progress.resetIdentity(1U, 1U);
  (void)observe(progress,measured(0.0, 100, 1, 0.0));
  (void)progress.activate();
  EXPECT_EQ(observe(progress,measured(0.0, 110, 2, 0.0)).kind,
            MissionProgressDecision::Kind::Goal);
  EXPECT_EQ(progress.currentGate().waypoint_index, 1U);
  EXPECT_EQ(progress.observeContinuation(mainWitness(progress), 120'000'000).kind,
            MissionProgressDecision::Kind::None);
  const auto decision = observe(progress,measured(1.5, 150, 3));
  EXPECT_EQ(decision.kind, MissionProgressDecision::Kind::Goal);
  EXPECT_EQ(progress.currentGate().request_id, 3U);
}

TEST(MissionProgressTest, LocalizationAndRouteChangesInvalidateWitness) {
  auto definition = mission();
  definition.waypoints[0].position_enu.x() = 1.0;
  MissionProgress progress(definition);
  progress.resetIdentity(1U, 1U);
  (void)observe(progress,measured(0.0, 100, 1));
  (void)progress.activate();
  (void)observe(progress,measured(1.0, 150, 2));
  ASSERT_TRUE(progress.crossing().has_value());
  const auto stale = mainWitness(progress);
  progress.resetIdentity(1U, 2U);
  EXPECT_FALSE(progress.crossing().has_value());
  EXPECT_EQ(progress.observeContinuation(stale, 200'000'000).kind,
            MissionProgressDecision::Kind::None);
  progress.resetIdentity(2U, 2U);
  EXPECT_EQ(progress.currentGate().waypoint_index, 0U);
}

TEST(MissionProgressTest, ReverseAndOutOfOrderInvalidateCrossing) {
  auto definition = mission();
  definition.waypoints[0].position_enu.x() = 1.0;
  MissionProgress progress(definition);
  progress.resetIdentity(1U, 1U);
  (void)observe(progress,measured(0.0, 100, 1));
  (void)progress.activate();
  (void)observe(progress,measured(1.0, 150, 2));
  ASSERT_TRUE(progress.crossing().has_value());
  (void)observe(progress,measured(0.0, 200, 3, -2.0));
  EXPECT_FALSE(progress.crossing().has_value());
  (void)observe(progress,measured(1.0, 250, 4));
  ASSERT_TRUE(progress.crossing().has_value());
  (void)observe(progress,measured(1.1, 240, 5));
  EXPECT_FALSE(progress.crossing().has_value());
}

TEST(MissionProgressTest, StopRequiresMeasuredSpeedConfirmationAndHold) {
  auto definition = mission();
  definition.waypoints.erase(definition.waypoints.begin());
  MissionProgress progress(definition);
  progress.resetIdentity(1U, 1U);
  (void)observe(progress,measured(5.0, 100, 1, 2.0));
  (void)progress.activate();
  const auto witness = mainWitness(progress, 3000);
  (void)progress.observeContinuation(witness, 110'000'000);
  EXPECT_EQ(observe(progress,measured(5.0, 150, 2, 2.0)).kind,
            MissionProgressDecision::Kind::None);
  EXPECT_EQ(observe(progress,measured(5.0, 200, 3, 0.0)).kind,
            MissionProgressDecision::Kind::None);
  EXPECT_EQ(observe(progress,measured(5.0, 600, 4, 0.0)).kind,
            MissionProgressDecision::Kind::None);
  EXPECT_EQ(observe(progress,measured(5.0, 700, 5, 0.0)).kind,
            MissionProgressDecision::Kind::None);
  EXPECT_EQ(observe(progress,measured(5.0, 900, 6, 0.0)).kind,
            MissionProgressDecision::Kind::Complete);
  EXPECT_TRUE(progress.complete());
}

TEST(MissionProgressTest, GapOver250MsCannotCertifySkippedBall) {
  MissionProgress progress(threeGateMission());
  progress.resetIdentity(1U, 1U);
  acceptInitialPass(progress);
  (void)progress.observeContinuation(mainWitness(progress), 120'000'000);
  EXPECT_EQ(observe(progress, measured(1.5, 450, 3)).kind,
            MissionProgressDecision::Kind::None);
  EXPECT_FALSE(progress.crossing().has_value());
  EXPECT_EQ(progress.currentGate().waypoint_index, 1U);
}

TEST(MissionProgressTest, ExistingCrossingIsInvalidatedByOdometryGap) {
  auto definition = mission();
  definition.waypoints[0].position_enu.x() = 1.0;
  MissionProgress progress(definition);
  progress.resetIdentity(1U, 1U);
  (void)observe(progress, measured(0.0, 100, 1));
  (void)progress.activate();
  (void)observe(progress, measured(1.0, 150, 2));
  ASSERT_TRUE(progress.crossing().has_value());
  (void)observe(progress, measured(1.9, 500, 3));
  EXPECT_FALSE(progress.crossing().has_value());
  EXPECT_EQ(progress.observeContinuation(mainWitness(progress), 510'000'000).kind,
            MissionProgressDecision::Kind::None);
  EXPECT_EQ(progress.currentGate().waypoint_index, 0U);
}

TEST(MissionProgressTest, RepeatedImmutableStateLeaseDoesNotEraseCrossing) {
  auto definition = mission();
  definition.waypoints[0].position_enu.x() = 1.0;
  MissionProgress progress(definition);
  progress.resetIdentity(1U, 1U);
  (void)observe(progress, measured(0.0, 100, 1));
  (void)progress.activate();
  const auto crossing_sample = measured(1.0, 150, 2);
  (void)observe(progress, crossing_sample);
  ASSERT_TRUE(progress.crossing().has_value());
  (void)observe(progress, crossing_sample);
  ASSERT_TRUE(progress.crossing().has_value());
  EXPECT_EQ(progress.observeContinuation(mainWitness(progress), 160'000'000).kind,
            MissionProgressDecision::Kind::Goal);
}

TEST(MissionProgressTest, WrongRequestAndDuplicateWitnessDoNotAdvanceGate) {
  MissionProgress progress(threeGateMission());
  progress.resetIdentity(1U, 1U);
  acceptInitialPass(progress);
  (void)observe(progress, measured(1.0, 150, 3));
  ASSERT_TRUE(progress.crossing().has_value());
  auto stale = mainWitness(progress);
  --stale.gate.request_id;
  EXPECT_EQ(progress.observeContinuation(stale, 155'000'000).kind,
            MissionProgressDecision::Kind::None);
  EXPECT_EQ(progress.currentGate().waypoint_index, 1U);
  const auto exact = mainWitness(progress);
  EXPECT_EQ(progress.observeContinuation(exact, 160'000'000).kind,
            MissionProgressDecision::Kind::Goal);
  EXPECT_EQ(progress.observeContinuation(exact, 170'000'000).kind,
            MissionProgressDecision::Kind::None);
  EXPECT_EQ(progress.currentGate().waypoint_index, 2U);
}

TEST(MissionProgressTest, SharpCornerDoesNotRequireOutgoingVelocityAlignment) {
  auto definition = threeGateMission();
  definition.waypoints.back().position_enu = Eigen::Vector3d{1, 1, 3};
  MissionProgress progress(definition);
  progress.resetIdentity(1U, 1U);
  acceptInitialPass(progress);
  (void)progress.observeContinuation(mainWitness(progress), 120'000'000);
  const auto decision = observe(progress, measured(1.0, 150, 3, 2.0));
  EXPECT_EQ(decision.kind, MissionProgressDecision::Kind::Goal);
  EXPECT_EQ(progress.currentGate().waypoint_index, 2U);
}

TEST(MissionProgressTest, CoincidentPassStopWaitsForMeasuredSettling) {
  auto definition = threeGateMission();
  definition.waypoints.back().position_enu =
      definition.waypoints[1].position_enu;
  MissionProgress progress(definition);
  progress.resetIdentity(1U, 1U);
  acceptInitialPass(progress);
  (void)progress.observeContinuation(mainWitness(progress), 120'000'000);
  EXPECT_EQ(observe(progress, measured(1.0, 150, 3, 2.0)).kind,
            MissionProgressDecision::Kind::None);
  EXPECT_EQ(progress.currentGate().waypoint_index, 1U);
  EXPECT_EQ(observe(progress, measured(1.0, 200, 4, 0.0)).kind,
            MissionProgressDecision::Kind::Goal);
  EXPECT_EQ(progress.currentGate().waypoint_index, 2U);
  EXPECT_EQ(observe(progress, measured(1.0, 250, 5, 0.0)).kind,
            MissionProgressDecision::Kind::None);
  EXPECT_FALSE(progress.complete());
}

TEST(MissionProgressTest, SelfIntersectionKeepsOrderedBranch) {
  auto definition = mission();
  definition.waypoints.clear();
  for (const auto& [id, x, y] : std::vector<std::tuple<const char*, double, double>>{
           {"a", 0, 0}, {"b", 2, 2}, {"c", 0, 2}, {"d", 2, 0}}) {
    navigation_mission::MissionWaypoint waypoint;
    waypoint.id = id;
    waypoint.position_enu = Eigen::Vector3d{x, y, 3};
    waypoint.acceptance_radius_m = 0.2;
    definition.waypoints.push_back(waypoint);
  }
  definition.waypoints.back().behavior =
      navigation_mission::MissionWaypoint::Behavior::Stop;
  MissionProgress progress(definition);
  progress.resetIdentity(1U, 1U);
  (void)observe(progress, measured(0.0, 100, 1, 0.0));
  (void)progress.activate();
  (void)observe(progress, measured(0.0, 110, 2, 0.0));
  const MissionMeasuredSample intersection{
      Eigen::Vector3d{1, 1, 3}, Eigen::Vector3d{2, 2, 0},
      150'000'000, 3U, 1U};
  EXPECT_EQ(observe(progress, intersection).kind,
            MissionProgressDecision::Kind::None);
  EXPECT_EQ(progress.currentGate().waypoint_index, 1U);
  EXPECT_EQ(progress.routeSnapshot().measured_progress.projection.segment_index, 0U);
}

TEST(MissionProgressTest, NearbyParallelLegCannotJumpPastConnectingTurn) {
  auto definition = mission();
  definition.waypoints.clear();
  for (const auto& [id, x, y] : std::vector<std::tuple<const char*, double, double>>{
           {"start", 0, 0}, {"east", 5, 0}, {"turn", 5, 0.3}, {"west", 0, 0.3}}) {
    navigation_mission::MissionWaypoint waypoint;
    waypoint.id = id;
    waypoint.position_enu = Eigen::Vector3d{x, y, 3};
    waypoint.acceptance_radius_m = 0.2;
    definition.waypoints.push_back(waypoint);
  }
  definition.waypoints.back().behavior =
      navigation_mission::MissionWaypoint::Behavior::Stop;
  MissionProgress progress(definition);
  progress.resetIdentity(1U, 1U);
  (void)observe(progress, measured(0.0, 100, 1, 0.0));
  (void)progress.activate();
  (void)observe(progress, measured(0.0, 110, 2, 0.0));
  const MissionMeasuredSample near_later_leg{
      Eigen::Vector3d{2.5, 0.2, 3}, Eigen::Vector3d{2, 0, 0},
      150'000'000, 3U, 1U};
  EXPECT_EQ(observe(progress, near_later_leg).kind,
            MissionProgressDecision::Kind::None);
  EXPECT_EQ(progress.currentGate().waypoint_index, 1U);
  EXPECT_EQ(progress.routeSnapshot().measured_progress.projection.segment_index, 0U);
  const MissionMeasuredSample turn{
      Eigen::Vector3d{5.0, 0.15, 3}, Eigen::Vector3d{0, 2, 0},
      190'000'000, 4U, 1U};
  (void)observe(progress, turn);
  EXPECT_EQ(progress.routeSnapshot().measured_progress.projection.segment_index, 1U);
  const MissionMeasuredSample outgoing{
      Eigen::Vector3d{4.8, 0.3, 3}, Eigen::Vector3d{-2, 0, 0},
      230'000'000, 5U, 1U};
  (void)observe(progress, outgoing);
  EXPECT_EQ(progress.routeSnapshot().measured_progress.projection.segment_index, 2U);
}
}  // namespace
}  // namespace navigation_runtime
