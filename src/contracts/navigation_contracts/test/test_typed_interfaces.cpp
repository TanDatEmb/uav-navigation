#include <gtest/gtest.h>

#include <functional>
#include <limits>
#include <vector>

#include <navigation_contracts/msg/estimator_health.hpp>
#include <navigation_contracts/msg/navigation_command.hpp>
#include <navigation_contracts/msg/navigation_execution_diagnostics.hpp>
#include <navigation_contracts/msg/navigation_goal.hpp>
#include <navigation_contracts/msg/propagated_odometry.hpp>
#include <navigation_contracts/msg/registered_scan.hpp>
#include <navigation_contracts/command_safety_contract.hpp>
#include <navigation_contracts/navigation_command_contract.hpp>

namespace {

using Command = navigation_contracts::msg::NavigationCommand;

Command validAdmissionCommand() {
  Command c;
  c.header.frame_id = "lio_odom";
  c.header.stamp.sec = 10;
  c.valid_until.sec = 11;
  c.world_observation_stamp.sec = 9;
  c.state_source_stamp.sec = 9;
  c.mission_id = "mission-a";
  c.localization_epoch = 3;
  c.goal_epoch = 5;
  c.world_generation = 13;
  c.world_revision = 21;
  c.bundle_generation = 34;
  c.sample_id = 55;
  c.role = Command::ROLE_MAIN;
  c.status = Command::STATUS_READY;
  return c;
}

// Frozen BASE predicate for an old/new truth-table comparison. It must not be
// called by product admission code.
bool baselineCommandContractValid(const Command& c, std::string_view frame) {
  const auto h = navigation_contracts::commandStampNanoseconds(c.header.stamp);
  const auto u = navigation_contracts::commandStampNanoseconds(c.valid_until);
  const auto w = navigation_contracts::commandStampNanoseconds(c.world_observation_stamp);
  const auto s = navigation_contracts::commandStampNanoseconds(c.state_source_stamp);
  const bool rejected = c.status == Command::STATUS_REJECTED;
  const bool normal = c.status == Command::STATUS_READY || c.status == Command::STATUS_COMPLETED;
  const bool braking = c.status == Command::STATUS_BRAKING;
  const bool role_valid =
      (normal && (c.role == Command::ROLE_MAIN || c.role == Command::ROLE_BACKUP) &&
       c.bundle_generation != 0U) ||
      (braking && c.role == Command::ROLE_EMERGENCY && c.bundle_generation != 0U) ||
      (rejected && c.role == Command::ROLE_EMERGENCY);
  const auto finite = [](double x) { return std::isfinite(x); };
  const bool pvaj_finite = finite(c.position.x) && finite(c.position.y) &&
      finite(c.position.z) && finite(c.velocity.x) && finite(c.velocity.y) &&
      finite(c.velocity.z) && finite(c.acceleration.x) && finite(c.acceleration.y) &&
      finite(c.acceleration.z) && finite(c.jerk.x) && finite(c.jerk.y) &&
      finite(c.jerk.z) && finite(c.yaw) && finite(c.yaw_rate) &&
      finite(c.trajectory_time_s);
  return !frame.empty() && !c.mission_id.empty() && c.header.frame_id == frame &&
      h > 0 && u > h && w > 0 && s > 0 && c.localization_epoch != 0U &&
      c.goal_epoch != 0U && c.world_generation != 0U && c.world_revision != 0U &&
      c.sample_id != 0U && c.status != Command::STATUS_EMPTY &&
      (normal || braking || rejected) && role_valid && pvaj_finite &&
      navigation_contracts::certifiedMainContinuationFieldsValid(c) &&
      c.trajectory_time_s >= 0.0;
}

bool baselineCommandValidAt(const Command& c, std::int64_t now) {
  if (now <= 0) return false;
  const auto h = navigation_contracts::commandStampNanoseconds(c.header.stamp);
  const auto u = navigation_contracts::commandStampNanoseconds(c.valid_until);
  return h > 0 && h <= now && u > h && now <= u;
}

}  // namespace

TEST(NavigationContracts, TypedCommandReasonsPreserveBaselineTruthTable) {
  using Reason = navigation_contracts::CommandContractReason;
  const std::vector<std::pair<Reason, std::function<void(Command&)>>> cases{
      {Reason::kMissionIdEmpty, [](auto& c) { c.mission_id.clear(); }},
      {Reason::kFrameMismatch, [](auto& c) { c.header.frame_id = "map"; }},
      {Reason::kHeaderStampInvalid, [](auto& c) { c.header.stamp.sec = 0; }},
      {Reason::kValidityWindowInvalid, [](auto& c) { c.valid_until.sec = 10; }},
      {Reason::kWorldStampInvalid, [](auto& c) { c.world_observation_stamp.sec = 0; }},
      {Reason::kStateStampInvalid, [](auto& c) { c.state_source_stamp.sec = 0; }},
      {Reason::kLocalizationIdentityInvalid, [](auto& c) { c.localization_epoch = 0; }},
      {Reason::kGoalIdentityInvalid, [](auto& c) { c.goal_epoch = 0; }},
      {Reason::kWorldIdentityInvalid, [](auto& c) { c.world_revision = 0; }},
      {Reason::kSampleIdentityInvalid, [](auto& c) { c.sample_id = 0; }},
      {Reason::kStatusInvalid, [](auto& c) { c.status = Command::STATUS_EMPTY; }},
      {Reason::kRoleStatusMismatch, [](auto& c) { c.role = Command::ROLE_EMERGENCY; }},
      {Reason::kPvajOrYawNonFinite, [](auto& c) {
          c.velocity.x = std::numeric_limits<double>::quiet_NaN(); }},
      {Reason::kContinuationContractInvalid, [](auto& c) {
          c.continuation_boundary_stamp_ns = 10'500'000'000ULL; }},
      {Reason::kTrajectoryTimeInvalid, [](auto& c) { c.trajectory_time_s = -1.0; }},
  };
  auto valid = validAdmissionCommand();
  EXPECT_EQ(navigation_contracts::assessCommandContract(valid, "lio_odom"), Reason::kValid);
  EXPECT_TRUE(baselineCommandContractValid(valid, "lio_odom"));
  EXPECT_EQ(navigation_contracts::assessCommandContract(valid, ""),
            Reason::kExpectedFrameMissing);
  EXPECT_FALSE(baselineCommandContractValid(valid, ""));
  for (const auto& [expected, mutate] : cases) {
    auto c = valid;
    mutate(c);
    EXPECT_EQ(navigation_contracts::assessCommandContract(c, "lio_odom"), expected);
    EXPECT_EQ(navigation_contracts::commandContractValid(c, "lio_odom"),
              baselineCommandContractValid(c, "lio_odom"));
  }
}

TEST(NavigationContracts, TypedCommandLeasePreservesInclusiveEdges) {
  using Reason = navigation_contracts::CommandTemporalReason;
  auto c = validAdmissionCommand();
  const std::vector<std::pair<std::int64_t, Reason>> cases{
      {-1, Reason::kNowInvalid}, {0, Reason::kNowInvalid},
      {9'999'999'999LL, Reason::kNotYetValid},
      {10'000'000'000LL, Reason::kValid},
      {10'500'000'000LL, Reason::kValid},
      {11'000'000'000LL, Reason::kValid},
      {11'000'000'001LL, Reason::kExpired},
  };
  for (const auto& [now, expected] : cases) {
    EXPECT_EQ(navigation_contracts::assessCommandTemporalLease(c, now), expected);
    EXPECT_EQ(navigation_contracts::commandValidAt(c, now), baselineCommandValidAt(c, now));
  }
  c.valid_until = c.header.stamp;
  EXPECT_EQ(navigation_contracts::assessCommandTemporalLease(c, 10'000'000'000LL),
            Reason::kInvalidWindow);
  EXPECT_FALSE(baselineCommandValidAt(c, 10'000'000'000LL));
}

TEST(NavigationContracts, RegisteredScanCarriesAtomicIdentityAndPayload) {
  navigation_contracts::msg::RegisteredScan message;
  message.localization_epoch = 7U;
  message.scan_sequence = 11U;
  message.header.frame_id = "lio_odom";
  message.body_frame_id = "base_link";
  message.sensor_origin_valid = true;
  message.sensor_origin_pose.position.x = 0.25;
  message.points.header.frame_id = message.header.frame_id;
  message.points.header.stamp = message.header.stamp;
  message.free_space_endpoints.header.frame_id = message.header.frame_id;
  message.free_space_endpoints.header.stamp = message.header.stamp;

  EXPECT_EQ(message.localization_epoch, 7U);
  EXPECT_EQ(message.scan_sequence, 11U);
  EXPECT_EQ(message.points.header.frame_id, "lio_odom");
  EXPECT_EQ(message.free_space_endpoints.header.frame_id, "lio_odom");
  EXPECT_EQ(message.body_frame_id, "base_link");
  EXPECT_TRUE(message.sensor_origin_valid);
  EXPECT_DOUBLE_EQ(message.sensor_origin_pose.position.x, 0.25);
}

TEST(NavigationContracts, PropagatedOdometryCarriesEpochSequenceAndSourceState) {
  navigation_contracts::msg::PropagatedOdometry message;
  message.localization_epoch = 7U;
  message.sequence = 11U;
  message.odometry.header.frame_id = "lio_odom";
  message.odometry.child_frame_id = "base_link";
  message.odometry.header.stamp.sec = 12;
  message.odometry.header.stamp.nanosec = 34U;

  EXPECT_EQ(message.localization_epoch, 7U);
  EXPECT_EQ(message.sequence, 11U);
  EXPECT_EQ(message.odometry.header.frame_id, "lio_odom");
  EXPECT_EQ(message.odometry.child_frame_id, "base_link");
  EXPECT_EQ(message.odometry.header.stamp.sec, 12);
  EXPECT_EQ(message.odometry.header.stamp.nanosec, 34U);
}

TEST(NavigationContracts, NavigationGoalCarriesVersionedRouteAndMeasuredProgress) {
  navigation_contracts::msg::NavigationGoal message;
  message.header.frame_id = "lio_odom";
  message.mission_id = "three-pillars";
  message.waypoint_index = 1U;
  message.request_id = 7U;
  message.route.mission_id = message.mission_id;
  message.route.frame_id = message.header.frame_id;
  message.route.route_revision = 3U;
  message.route.request_id = message.request_id;
  message.route.active_waypoint_index = message.waypoint_index;
  message.route.measured_progress_valid = true;
  message.route.measured_segment_index = 0U;
  message.route.measured_progress_arc_m = 8.5;
  message.route.measured_projection_arc_m = 8.0;
  message.route.measured_lateral_error_m = 0.2;
  geometry_msgs::msg::Point point;
  point.x = 10.0;
  point.z = 2.0;
  message.route.waypoint_positions.push_back(point);
  message.route.waypoint_ids.push_back("wp-1");
  message.route.waypoint_acceptance_radii_m.push_back(1.0);
  message.route.waypoint_behaviors.push_back(
      navigation_contracts::msg::RouteSnapshot::BEHAVIOR_PASS_THROUGH);

  EXPECT_EQ(message.route.mission_id, message.mission_id);
  EXPECT_EQ(message.route.frame_id, message.header.frame_id);
  EXPECT_EQ(message.route.route_revision, 3U);
  EXPECT_EQ(message.route.request_id, message.request_id);
  EXPECT_EQ(message.route.active_waypoint_index, message.waypoint_index);
  ASSERT_EQ(message.route.waypoint_positions.size(), 1U);
  EXPECT_EQ(message.route.waypoint_ids.front(), "wp-1");
  EXPECT_DOUBLE_EQ(message.route.measured_progress_arc_m, 8.5);
  EXPECT_DOUBLE_EQ(message.route.measured_projection_arc_m, 8.0);
}

TEST(NavigationContracts, EstimatorHealthUsesTypedStateAndIndependentFlags) {
  navigation_contracts::msg::EstimatorHealth message;
  message.state = navigation_contracts::msg::EstimatorHealth::TRACKING;
  message.navigation_valid = true;
  message.covariance_valid = true;
  message.observability_valid = false;
  message.correction_fresh = true;
  message.propagation_valid = true;
  message.last_propagated_state_stamp.sec = 12;
  message.last_propagated_state_stamp.nanosec = 34U;

  EXPECT_EQ(message.state,
            navigation_contracts::msg::EstimatorHealth::TRACKING);
  EXPECT_TRUE(message.navigation_valid);
  EXPECT_TRUE(message.covariance_valid);
  EXPECT_FALSE(message.observability_valid);
  EXPECT_TRUE(message.correction_fresh);
  EXPECT_TRUE(message.propagation_valid);
  EXPECT_EQ(message.last_propagated_state_stamp.sec, 12);
  EXPECT_EQ(message.last_propagated_state_stamp.nanosec, 34U);
}

TEST(NavigationContracts, NavigationCommandExposesAllProvenanceDimensions) {
  navigation_contracts::msg::NavigationCommand message;
  navigation_contracts::msg::NavigationExecutionDiagnostics diagnostic;
  message.localization_epoch = 3U;
  message.goal_epoch = 5U;
  message.request_id = 8U;
  message.world_generation = 13U;
  message.world_revision = 21U;
  message.bundle_generation = 34U;
  message.sample_id = 55U;
  diagnostic.execution_authorization = navigation_contracts::msg::NavigationExecutionDiagnostics::
      EXECUTION_AUTHORIZATION_GRANTED;
  diagnostic.execution_authorization_steady_ns = 89U;
  diagnostic.sample_id = message.sample_id;
  message.role = navigation_contracts::msg::NavigationCommand::ROLE_BACKUP;
  message.status = navigation_contracts::msg::NavigationCommand::STATUS_READY;

  EXPECT_EQ(message.role,
            navigation_contracts::msg::NavigationCommand::ROLE_BACKUP);
  EXPECT_EQ(message.status,
            navigation_contracts::msg::NavigationCommand::STATUS_READY);
  EXPECT_EQ(message.localization_epoch, 3U);
  EXPECT_EQ(message.goal_epoch, 5U);
  EXPECT_EQ(message.world_generation, 13U);
  EXPECT_EQ(message.world_revision, 21U);
  EXPECT_EQ(message.bundle_generation, 34U);
  EXPECT_EQ(message.sample_id, 55U);
  EXPECT_EQ(diagnostic.execution_authorization,
            navigation_contracts::msg::NavigationExecutionDiagnostics::
                EXECUTION_AUTHORIZATION_GRANTED);
  EXPECT_EQ(diagnostic.execution_authorization_steady_ns, 89U);
  EXPECT_EQ(diagnostic.sample_id, message.sample_id);
}

TEST(NavigationContracts, CommandsRequireHealthyTypedEpochHandshake) {
  EXPECT_FALSE(navigation_contracts::estimatorHealthAllowsCommand(false, true, 3U, 3U));
  EXPECT_FALSE(navigation_contracts::estimatorHealthAllowsCommand(true, false, 3U, 3U));
  EXPECT_FALSE(navigation_contracts::estimatorHealthAllowsCommand(true, true, 2U, 3U));
  EXPECT_TRUE(navigation_contracts::estimatorHealthAllowsCommand(true, true, 3U, 3U));
}

TEST(NavigationContracts, ContinuationWitnessLeaseFailsClosedPerFreshnessInput) {
  navigation_contracts::ExecutionStateFreshness fresh;
  fresh.reason = navigation_contracts::ExecutionStateFreshnessReason::kValid;
  const auto valid = [&]() {
    return navigation_contracts::continuationWitnessLeaseValid(
        true, 2'000'000'000LL, 1'500'000'000LL, 1'000'000'000LL,
        fresh, fresh, true, false, false, false);
  };
  EXPECT_TRUE(valid());

  auto source_stale = fresh;
  source_stale.reason = navigation_contracts::ExecutionStateFreshnessReason::kSourceStale;
  EXPECT_FALSE(navigation_contracts::continuationWitnessLeaseValid(
      true, 2'000'000'000LL, 1'500'000'000LL, 1'000'000'000LL,
      source_stale, fresh, true, false, false, false));
  auto receive_stale = fresh;
  receive_stale.reason = navigation_contracts::ExecutionStateFreshnessReason::kReceiveStale;
  EXPECT_FALSE(navigation_contracts::continuationWitnessLeaseValid(
      true, 2'000'000'000LL, 1'500'000'000LL, 1'000'000'000LL,
      receive_stale, fresh, true, false, false, false));
  EXPECT_FALSE(navigation_contracts::continuationWitnessLeaseValid(
      true, 2'000'000'000LL, 500'000'000LL, 1'000'000'000LL,
      fresh, fresh, true, false, false, false));
  EXPECT_FALSE(navigation_contracts::continuationWitnessLeaseValid(
      true, 2'000'000'000LL, 1'500'000'000LL, 1'000'000'000LL,
      fresh, fresh, false, false, false, false));
  EXPECT_FALSE(navigation_contracts::continuationWitnessLeaseValid(
      false, 2'000'000'000LL, 1'500'000'000LL, 1'000'000'000LL,
      fresh, fresh, true, false, false, false));
  auto health_stale = fresh;
  health_stale.reason = navigation_contracts::ExecutionStateFreshnessReason::kSourceStale;
  EXPECT_FALSE(navigation_contracts::continuationWitnessLeaseValid(
      true, 2'000'000'000LL, 1'500'000'000LL, 1'000'000'000LL,
      fresh, health_stale, true, false, false, false));
}

TEST(NavigationContracts, CertifiedContinuationIsExplicitAndMainBound) {
  navigation_contracts::msg::NavigationCommand command;
  command.header.frame_id = "lio_odom";
  command.header.stamp.sec = 10;
  command.valid_until.sec = 11;
  command.world_observation_stamp.sec = 9;
  command.state_source_stamp.sec = 9;
  command.mission_id = "mission-a";
  command.localization_epoch = 3U;
  command.goal_epoch = 5U;
  command.world_generation = 13U;
  command.world_revision = 21U;
  command.bundle_generation = 34U;
  command.sample_id = 55U;
  command.role = navigation_contracts::msg::NavigationCommand::ROLE_MAIN;
  command.status = navigation_contracts::msg::NavigationCommand::STATUS_READY;
  EXPECT_TRUE(navigation_contracts::commandContractValid(command, "lio_odom"));

  command.certified_main_continuation = true;
  command.continuation_boundary_stamp_ns = 10'500'000'000ULL;
  EXPECT_TRUE(navigation_contracts::certifiedMainContinuationFieldsValid(command));
  EXPECT_TRUE(navigation_contracts::commandContractValid(command, "lio_odom"));
  // The boundary is on the candidate timeline and may be beyond this
  // individual command's short lease. Freshness is checked separately by
  // commandValidAt()/continuationWitnessLeaseValid().
  command.continuation_boundary_stamp_ns = 12'000'000'000ULL;
  EXPECT_TRUE(navigation_contracts::certifiedMainContinuationFieldsValid(command));
  EXPECT_TRUE(navigation_contracts::commandContractValid(command, "lio_odom"));
  command.status = navigation_contracts::msg::NavigationCommand::STATUS_COMPLETED;
  EXPECT_FALSE(navigation_contracts::certifiedMainContinuationFieldsValid(command));
  command.status = navigation_contracts::msg::NavigationCommand::STATUS_READY;

  command.role = navigation_contracts::msg::NavigationCommand::ROLE_BACKUP;
  EXPECT_FALSE(navigation_contracts::certifiedMainContinuationFieldsValid(command));
  command.role = navigation_contracts::msg::NavigationCommand::ROLE_MAIN;
  command.continuation_boundary_stamp_ns = 0U;
  EXPECT_FALSE(navigation_contracts::certifiedMainContinuationFieldsValid(command));
  command.certified_main_continuation = false;
  EXPECT_TRUE(navigation_contracts::certifiedMainContinuationFieldsValid(command));
  command.continuation_boundary_stamp_ns = 10'500'000'000ULL;
  EXPECT_FALSE(navigation_contracts::certifiedMainContinuationFieldsValid(command));
}

TEST(NavigationContracts, NavigationCommandContractRejectsMalformedOrRegressedIdentity) {
  navigation_contracts::msg::NavigationCommand previous;
  previous.header.frame_id = "lio_odom";
  previous.header.stamp.sec = 10;
  previous.valid_until.sec = 11;
  previous.world_observation_stamp.sec = 9;
  previous.state_source_stamp.sec = 9;
  previous.mission_id = "mission-a";
  previous.localization_epoch = 3U;
  previous.goal_epoch = 5U;
  previous.world_generation = 13U;
  previous.world_revision = 21U;
  previous.bundle_generation = 34U;
  previous.sample_id = 55U;
  previous.role = navigation_contracts::msg::NavigationCommand::ROLE_MAIN;
  previous.status = navigation_contracts::msg::NavigationCommand::STATUS_READY;
  EXPECT_TRUE(navigation_contracts::commandContractValid(previous, "lio_odom"));

  auto newer = previous;
  newer.header.stamp.sec = 12;
  newer.valid_until.sec = 13;
  newer.world_revision = 22U;
  newer.state_source_stamp.sec = 10;
  newer.sample_id = 56U;
  EXPECT_TRUE(navigation_contracts::commandWorldIdentityNonRegressing(newer, previous));

  auto regressed = newer;
  regressed.world_revision = previous.world_revision - 1U;
  EXPECT_FALSE(navigation_contracts::commandWorldIdentityNonRegressing(regressed, previous));
  regressed = newer;
  regressed.header.frame_id = "map";
  EXPECT_FALSE(navigation_contracts::commandContractValid(regressed, "lio_odom"));
  auto malformed = previous;
  malformed.mission_id.clear();
  EXPECT_FALSE(navigation_contracts::commandContractValid(malformed, "lio_odom"));
  malformed = previous;
  malformed.trajectory_time_s = -1.0;
  EXPECT_FALSE(navigation_contracts::commandContractValid(malformed, "lio_odom"));
  EXPECT_FALSE(navigation_contracts::commandContractValid(previous, ""));
  EXPECT_TRUE(navigation_contracts::commandValidAt(previous, 10'500'000'000LL));
  EXPECT_FALSE(navigation_contracts::commandValidAt(previous, 11'000'000'001LL));
  EXPECT_TRUE(navigation_contracts::commandMissionIdentityMatches(
      previous, "", 0U, 0U) == false);
  previous.mission_id = "mission-a";
  previous.waypoint_index = 2U;
  previous.request_id = 8U;
  EXPECT_TRUE(navigation_contracts::commandMissionIdentityMatches(
      previous, "mission-a", 2U, 8U));
  EXPECT_FALSE(navigation_contracts::commandMissionIdentityMatches(
      previous, "mission-b", 2U, 8U));
}

TEST(NavigationContracts, CertifiedEmergencyUsesBrakingStatus) {
  navigation_contracts::msg::NavigationCommand command;
  command.header.frame_id = "lio_odom";
  command.header.stamp.sec = 10;
  command.valid_until.sec = 11;
  command.world_observation_stamp.sec = 9;
  command.state_source_stamp.sec = 9;
  command.mission_id = "mission-a";
  command.localization_epoch = 3U;
  command.goal_epoch = 5U;
  command.world_generation = 13U;
  command.world_revision = 21U;
  command.bundle_generation = 34U;
  command.sample_id = 55U;
  command.role = navigation_contracts::msg::NavigationCommand::ROLE_EMERGENCY;
  command.status = navigation_contracts::msg::NavigationCommand::STATUS_BRAKING;
  EXPECT_TRUE(navigation_contracts::commandContractValid(command, "lio_odom"));

  command.status = navigation_contracts::msg::NavigationCommand::STATUS_READY;
  EXPECT_FALSE(navigation_contracts::commandContractValid(command, "lio_odom"));
}
