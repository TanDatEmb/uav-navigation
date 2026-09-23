#include "navigation_runtime/mission_progress.hpp"
#include "navigation_runtime/mission_goal.hpp"
#include "navigation_runtime/execution_lifecycle_view.hpp"
#include "navigation_runtime/runtime_boundaries.hpp"

#include <navigation_contracts/navigation_command_contract.hpp>
#include <navigation_execution/command_sampler.hpp>
#include <navigation_execution/execution_anchor.hpp>

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

navigation_planning::CandidateBundle handoffCandidate(
    const navigation_world_model::WorldSnapshotIdentity& world,
    std::uint64_t goal_epoch, std::uint64_t request,
    std::uint64_t generation, std::int64_t start_ns,
    double position_offset_m = 0.0) {
  navigation_planning::CandidateBundle candidate;
  candidate.world_identity = world;
  candidate.pinned_world_identity = world;
  candidate.localization_epoch = 1U;
  candidate.goal_epoch = goal_epoch;
  candidate.request_id = request;
  candidate.bundle_generation = generation;
  candidate.valid_from_ns = start_ns;
  candidate.valid_until_ns = start_ns + 900'000'000;
  candidate.activation_stamp_ns = start_ns;
  candidate.declared_start_ns = start_ns;
  candidate.declared_end_ns = start_ns + 900'000'000;
  candidate.start_wall_time_s = static_cast<double>(start_ns) * 1.0e-9;
  candidate.duration_s = 0.9;
  candidate.kind = navigation_planning::CandidateBundleKind::kTerminalStop;
  candidate.certificates = {true, true, true, true};
  candidate.protected_region.minimum = Eigen::Vector3d{0.0, -1.0, 2.0};
  candidate.protected_region.maximum = Eigen::Vector3d{6.0, 1.0, 4.0};
  candidate.role_schedule = {{0.0, 0.9, navigation_planning::CandidateRole::kMain}};
  candidate.evaluator = [start_ns, position_offset_m](std::int64_t stamp,
                                   navigation_planning::TrajectoryPoint& point) {
    point.position_world = Eigen::Vector3d{
        position_offset_m + static_cast<double>(stamp - start_ns) * 1.0e-9,
        0.0, 3.0};
    point.velocity_world = Eigen::Vector3d{1.0, 0.0, 0.0};
    point.trajectory_time_s = static_cast<double>(stamp - start_ns) * 1.0e-9;
    return true;
  };
  return candidate;
}

struct HandoffAdapterIdentityModel {
  std::optional<navigation_contracts::msg::NavigationCommand> last;

  bool admit(const navigation_contracts::msg::NavigationCommand& command,
             std::int64_t now_ns) {
    if (!navigation_contracts::commandValidAt(command, now_ns) ||
        command.mode_activation_id != 1U ||
        (last && (command.mission_id != last->mission_id ||
                  command.sample_id <= last->sample_id ||
                  command.request_id < last->request_id ||
                  !navigation_contracts::commandWorldIdentityNonRegressing(
                      command, *last)))) {
      return false;
    }
    last = command;
    return true;
  }
};

TEST(MissionProgressTest, EndToEndHotHandoffRetainsPredecessorUntilAtomicCutover) {
  MissionProgress progress(threeGateMission());
  progress.resetIdentity(1U, 1U);
  (void)observe(progress, measured(0.0, 100, 1, 0.0));
  ASSERT_EQ(progress.activate().kind, MissionProgressDecision::Kind::Goal);
  ASSERT_EQ(observe(progress, measured(0.0, 110, 2, 0.0)).kind,
            MissionProgressDecision::Kind::Goal);
  ASSERT_EQ(progress.currentGate().waypoint_index, 1U);
  const auto predecessor_goal = makeMissionGoal(
      progress, *navigation_common::nanosecondsToRosTime(110'000'000));
  ASSERT_TRUE(predecessor_goal);

  navigation_execution::ExecutionAuthority execution_authority;
  const navigation_world_model::WorldSnapshotIdentity world{
      1U, 4U, 1U, 100'000'000};
  const auto empty = execution_authority.snapshot();
  ASSERT_EQ(execution_authority.publishWorldIdentityIfCurrent(
                world, empty.version, {}, false),
            navigation_world_model::WorldCommitDecision::kCommitted);
  ASSERT_TRUE(execution_authority.setAdmissionGoalEpoch(9U));
  const auto predecessor = std::make_shared<const navigation_planning::CandidateBundle>(
      handoffCandidate(world, 9U, 2U, 4U, 100'000'000));
  ASSERT_TRUE(predecessor->valid());
  ASSERT_EQ(execution_authority.tryCommit(
                {world, 9U, 1U},
                std::make_shared<const navigation_contracts::msg::NavigationGoal>(
                    *predecessor_goal), predecessor),
            navigation_execution::CommitDecision::kCommitted);
  navigation_execution::CommandSampler sampler(execution_authority);

  // Crossing arrives before the downstream receipt. Neither the crossing nor
  // an unacknowledged sampled command fabricates mission acceptance.
  ASSERT_EQ(observe(progress, measured(1.5, 150, 3)).kind,
            MissionProgressDecision::Kind::None);
  ASSERT_TRUE(progress.crossing());
  ASSERT_EQ(progress.currentGate().request_id, 2U);
  const MissionContinuationWitness admitted_predecessor{
      progress.routeIdentity(), progress.currentGate(),
      MissionContinuationWitness::Kind::Main, 4U, 150'000'000, 1'000'000'000};
  ASSERT_EQ(progress.observeContinuation(admitted_predecessor, 170'000'000).kind,
            MissionProgressDecision::Kind::Goal);
  ASSERT_EQ(progress.currentGate().waypoint_index, 2U);
  const auto successor_goal = makeMissionGoal(
      progress, *navigation_common::nanosecondsToRosTime(170'000'000));
  ASSERT_TRUE(successor_goal);
  ASSERT_EQ(successor_goal->request_id, 3U);
  ASSERT_TRUE(execution_authority.setAdmissionGoalEpoch(10U, true));
  EXPECT_EQ(execution_authority.load(), predecessor);
  const auto execution = execution_authority.snapshot();
  EXPECT_EQ(execution.activeGeneration(), 4U);
  EXPECT_EQ(execution.admission_goal_epoch, 10U);
  EXPECT_EQ(execution.activeGoalEpoch(), 9U);

  HandoffAdapterIdentityModel adapter;
  std::uint64_t sample_id = 1U;
  const auto commandFor = [&](const navigation_contracts::msg::NavigationGoal& goal,
                              std::uint64_t goal_epoch,
                              std::uint64_t generation,
                              std::int64_t stamp_ns) {
    navigation_contracts::msg::NavigationCommand command;
    command.header.stamp = *navigation_common::nanosecondsToRosTime(stamp_ns);
    command.valid_until = *navigation_common::nanosecondsToRosTime(
        stamp_ns + 100'000'000);
    command.world_observation_stamp = *navigation_common::nanosecondsToRosTime(
        world.observation_stamp_ns);
    command.state_source_stamp = command.header.stamp;
    command.mode_activation_id = 1U;
    command.mission_id = goal.mission_id;
    command.localization_epoch = 1U;
    command.goal_epoch = goal_epoch;
    command.waypoint_index = goal.waypoint_index;
    command.request_id = goal.request_id;
    command.bundle_generation = generation;
    command.sample_id = sample_id++;
    command.world_generation = world.generation;
    command.world_revision = world.revision;
    return command;
  };
  for (const auto stamp_ns : {220'000'000LL, 270'000'000LL,
                              320'000'000LL, 390'000'000LL}) {
    ASSERT_TRUE(sampler.sample(stamp_ns, 9U));
    EXPECT_TRUE(sameExecutionPublicationIdentity(
        predecessor_goal, predecessor_goal, 9U, 9U, 1U, 1U));
    EXPECT_TRUE(adapter.admit(commandFor(*predecessor_goal, 9U, 4U, stamp_ns),
                              stamp_ns));
  }
  const auto lease_boundary_command =
      commandFor(*predecessor_goal, 9U, 4U, 390'000'000);
  EXPECT_TRUE(navigation_contracts::commandValidAt(
      lease_boundary_command, 490'000'000));
  EXPECT_FALSE(navigation_contracts::commandValidAt(
      lease_boundary_command, 490'000'001));
  // Failed planning/certificate admission leaves the exact predecessor in
  // place. It cannot turn a desired gate change into execution cutover.
  auto invalid_world = world;
  invalid_world.revision = 2U;
  invalid_world.observation_stamp_ns = 200'000'000;
  const auto failed_successor =
      std::make_shared<const navigation_planning::CandidateBundle>(
          handoffCandidate(invalid_world, 10U, 3U, 5U, 400'000'000));
  ASSERT_EQ(execution_authority.tryCommit(
                {invalid_world, 10U, 2U},
                std::make_shared<const navigation_contracts::msg::NavigationGoal>(
                    *successor_goal), failed_successor),
            navigation_execution::CommitDecision::kWorldAdvanced);
  EXPECT_EQ(execution_authority.load(), predecessor);
  const auto anchor = execution_authority.reserveAnchor(390'000'000, 400'000'000);
  ASSERT_TRUE(anchor);
  const auto discontinuous_successor =
      handoffCandidate(world, 10U, 3U, 5U, 400'000'000);
  EXPECT_EQ(navigation_execution::candidateMatchesAnchor(
                discontinuous_successor, *anchor),
            navigation_execution::AnchorMatchResult::kPositionMismatch);
  EXPECT_EQ(execution_authority.load(), predecessor);
  const auto continuous_successor =
      handoffCandidate(world, 10U, 3U, 5U, 400'000'000, 0.3);
  for (const auto mismatch : {
           navigation_execution::AnchorMatchResult::kVelocityMismatch,
           navigation_execution::AnchorMatchResult::kAccelerationMismatch,
           navigation_execution::AnchorMatchResult::kJerkMismatch}) {
    auto bad = continuous_successor;
    const auto evaluator = bad.evaluator;
    bad.evaluator = [evaluator, mismatch](std::int64_t stamp,
                                          navigation_planning::TrajectoryPoint& point) {
      if (!evaluator(stamp, point)) return false;
      if (mismatch == navigation_execution::AnchorMatchResult::kVelocityMismatch) {
        point.velocity_world.x() += 0.1;
      } else if (mismatch ==
                 navigation_execution::AnchorMatchResult::kAccelerationMismatch) {
        point.acceleration_world.x() += 0.1;
      } else {
        point.jerk_world.x() += 0.1;
      }
      return true;
    };
    EXPECT_EQ(navigation_execution::candidateMatchesAnchor(bad, *anchor), mismatch);
    EXPECT_EQ(execution_authority.load(), predecessor);
  }
  const auto successor = std::make_shared<const navigation_planning::CandidateBundle>(
      continuous_successor);
  ASSERT_EQ(navigation_execution::candidateMatchesAnchor(*successor, *anchor),
            navigation_execution::AnchorMatchResult::kMatch);
  ASSERT_EQ(execution_authority.tryCommit(
                {world, 10U, 3U},
                std::make_shared<const navigation_contracts::msg::NavigationGoal>(
                    *successor_goal), successor),
            navigation_execution::CommitDecision::kCommitted);
  ASSERT_TRUE(sampler.sample(400'000'000, 10U));
  EXPECT_FALSE(sameExecutionPublicationIdentity(
      predecessor_goal, successor_goal, 9U, 10U, 1U, 1U));
  EXPECT_TRUE(adapter.admit(commandFor(*successor_goal, 10U, 5U, 400'000'000),
                            400'000'000));
  EXPECT_FALSE(adapter.admit(commandFor(*predecessor_goal, 9U, 4U, 410'000'000),
                             410'000'000));
  EXPECT_EQ(progress.currentGate().waypoint_index, 2U);
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
