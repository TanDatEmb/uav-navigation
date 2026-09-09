#include <gtest/gtest.h>

#include <navigation_runtime/path_relative_tracking.hpp>
#include <navigation_runtime/experimental_tracking.hpp>

namespace {

using navigation_planning::CandidateBundle;
using navigation_planning::CandidateBundleKind;
using navigation_planning::CandidateRole;
using navigation_planning::TrajectoryPoint;
using navigation_runtime::PathRelativeTrackingStatus;
using navigation_runtime::PathRelativeTrackingResult;

CandidateBundle makeLinearMainBundle(double speed = 5.0) {
  CandidateBundle bundle;
  bundle.world_identity.localization_epoch = 1;
  bundle.world_identity.generation = 1;
  bundle.world_identity.revision = 1;
  bundle.world_identity.observation_stamp_ns = 1;
  bundle.pinned_world_identity = bundle.world_identity;
  bundle.localization_epoch = bundle.goal_epoch = bundle.request_id =
      bundle.bundle_generation = 1;
  bundle.start_wall_time_s = 10.0;
  bundle.duration_s = 2.0;
  bundle.backup_start_time_s = 1.5;
  bundle.declared_start_ns = bundle.valid_from_ns = bundle.activation_stamp_ns =
      10'000'000'000LL;
  bundle.declared_end_ns = bundle.valid_until_ns = 12'000'000'000LL;
  bundle.role = CandidateRole::kMain;
  bundle.kind = CandidateBundleKind::kMainWithBackup;
  bundle.backup_available = true;
  bundle.certificates = {true, true, true, false};
  bundle.protected_region.minimum = Eigen::Vector3d::Constant(-20.0);
  bundle.protected_region.maximum = Eigen::Vector3d::Constant(20.0);
  bundle.role_schedule = {{0.0, 1.5, CandidateRole::kMain},
                          {1.5, 2.0, CandidateRole::kBackup}};
  bundle.evaluator = [speed](std::int64_t stamp, TrajectoryPoint& point) {
    point.trajectory_time_s = (stamp - 10'000'000'000LL) * 1.0e-9;
    point.position_world = {speed * point.trajectory_time_s, 0.0, 0.0};
    point.velocity_world = {speed, 0.0, 0.0};
    point.role = point.trajectory_time_s < 1.5
        ? CandidateRole::kMain : CandidateRole::kBackup;
    return true;
  };
  return bundle;
}

CandidateBundle makeTerminalMainBundle() {
  auto bundle = makeLinearMainBundle(5.0);
  bundle.kind = CandidateBundleKind::kTerminalStop;
  bundle.backup_available = false;
  bundle.terminal_stop = true;
  bundle.certificates.terminal_stop = true;
  bundle.backup_start_time_s = bundle.duration_s;
  bundle.role_schedule = {{0.0, 2.0, CandidateRole::kMain}};
  bundle.evaluator = [](std::int64_t stamp, TrajectoryPoint& point) {
    const double t = (stamp - 10'000'000'000LL) * 1.0e-9;
    point.trajectory_time_s = t;
    point.position_world = {5.0 * t - 1.25 * t * t, 0.0, 0.0};
    point.velocity_world = {5.0 - 2.5 * t, 0.0, 0.0};
    point.role = CandidateRole::kMain;
    return true;
  };
  return bundle;
}

PathRelativeTrackingResult assess(
    const CandidateBundle& bundle, const Eigen::Vector3d& position,
    const Eigen::Vector3d& velocity, std::int64_t now_ns = 10'600'000'000LL,
    std::int64_t source_stamp_ns = 10'600'000'000LL) {
  return navigation_runtime::assessPathRelativeTracking(
      bundle, position, velocity, now_ns, source_stamp_ns, 0.12, 0.10,
      0.25, 0.75, true, true, true);
}

TEST(PathRelativeTracking, AcceptsPhysicalPhaseLeadAndLagOnTheSamePath) {
  for (const double speed : {3.0, 5.0, 8.0, 12.0}) {
    for (const double phase : {-0.09, 0.09}) {
      const auto bundle = makeLinearMainBundle(speed);
      const auto result = assess(
          bundle, {speed * (0.6 + phase), 0.0, 0.0}, {speed, 0.0, 0.0});
      ASSERT_TRUE(result.accepted()) << "speed=" << speed
                                     << " phase=" << phase
                                     << " status="
                                     << static_cast<int>(result.status);
      EXPECT_GT(result.raw_error_m, 0.25);
      EXPECT_NEAR(result.phase_offset_s, phase, 1.0e-6);
      EXPECT_NEAR(result.path_error_m, 0.0, 1.0e-5);
      EXPECT_LT(result.evaluation_count, 150U);
    }
  }
}

TEST(PathRelativeTracking, KeepsLateralAndVerticalTrackingBounded) {
  const auto bundle = makeLinearMainBundle();
  EXPECT_EQ(assess(bundle, {3.0, 0.26, 0.0}, {5.0, 0.0, 0.0}).status,
            PathRelativeTrackingStatus::kOutsidePathTube);
  EXPECT_EQ(assess(bundle, {3.0, 0.0, 0.26}, {5.0, 0.0, 0.0}).status,
            PathRelativeTrackingStatus::kOutsidePathTube);
  EXPECT_EQ(assess(bundle, {3.0, 0.24, 0.0}, {5.0, 1.0, 0.0}).status,
            PathRelativeTrackingStatus::kPredictionOutsideTube);
}

TEST(PathRelativeTracking, DoesNotExtendLeaseOrCrossMainBackupSeam) {
  auto bundle = makeLinearMainBundle();
  bundle.valid_from_ns = bundle.activation_stamp_ns = 10'590'000'000LL;
  const auto accepted = assess(bundle, {5.0 * 0.51, 0.0, 0.0}, {5.0, 0.0, 0.0});
  EXPECT_TRUE(accepted.accepted());
  EXPECT_LT(accepted.projected_stamp_ns, bundle.valid_from_ns);

  bundle.valid_until_ns = 10'650'000'000LL;
  EXPECT_FALSE(assess(bundle, {2.55, 0.0, 0.0}, {5.0, 0.0, 0.0}).accepted());

  EXPECT_FALSE(assess(
      makeLinearMainBundle(), {5.0 * 1.44, 0.0, 0.0}, {5.0, 0.0, 0.0},
      11'350'000'000LL, 11'350'000'000LL).accepted());
}

TEST(PathRelativeTracking, RejectsAmbiguousOrUnsafeMotion) {
  auto bundle = makeLinearMainBundle();
  EXPECT_EQ(assess(bundle, {3.0, 0.0, 0.0}, {-5.0, 0.0, 0.0}).status,
            PathRelativeTrackingStatus::kReverseMotion);
  EXPECT_FALSE(navigation_runtime::assessPathRelativeTracking(
      bundle, Eigen::Vector3d::Constant(std::numeric_limits<double>::quiet_NaN()),
      {5.0, 0.0, 0.0}, 10'600'000'000LL, 10'600'000'000LL, 0.12, 0.10,
      0.25, 0.75, true, true, true).accepted());

  bundle.evaluator = [](std::int64_t stamp, TrajectoryPoint& point) {
    point.trajectory_time_s = (stamp - 10'000'000'000LL) * 1.0e-9;
    const double angle = 1.25 * point.trajectory_time_s;
    point.position_world = {4.0 * std::cos(angle), 4.0 * std::sin(angle), 0.0};
    point.velocity_world = {-5.0 * std::sin(angle), 5.0 * std::cos(angle), 0.0};
    point.role = point.trajectory_time_s < 1.5
        ? CandidateRole::kMain : CandidateRole::kBackup;
    return true;
  };
  const auto left = bundle.sampleAtDeclaredStamp(10'200'000'000LL);
  const auto right = bundle.sampleAtDeclaredStamp(11'000'000'000LL);
  const auto middle = bundle.sampleAtDeclaredStamp(10'600'000'000LL);
  ASSERT_TRUE(left && right && middle);
  EXPECT_EQ(assess(
      bundle, (left->position_world + right->position_world) / 2.0,
      middle->velocity_world).status,
      PathRelativeTrackingStatus::kOutsidePathTube);
}

TEST(PathRelativeTracking, AcceptsCurvedPhysicalPhaseAndRejectsShortcut) {
  auto bundle = makeLinearMainBundle();
  bundle.evaluator = [](std::int64_t stamp, TrajectoryPoint& point) {
    point.trajectory_time_s = (stamp - 10'000'000'000LL) * 1.0e-9;
    const double angle = 1.25 * point.trajectory_time_s;
    point.position_world = {4.0 * std::cos(angle), 4.0 * std::sin(angle), 0.0};
    point.velocity_world = {-5.0 * std::sin(angle), 5.0 * std::cos(angle), 0.0};
    point.role = point.trajectory_time_s < 1.5
        ? CandidateRole::kMain : CandidateRole::kBackup;
    return true;
  };
  for (const double phase : {-0.08, 0.08}) {
    const auto state = bundle.sampleAtDeclaredStamp(
        10'600'000'000LL + static_cast<std::int64_t>(phase * 1.0e9));
    ASSERT_TRUE(state);
    const auto result = assess(bundle, state->position_world, state->velocity_world);
    EXPECT_TRUE(result.accepted()) << static_cast<int>(result.status);
    EXPECT_NEAR(result.path_error_m, 0.0, 1.0e-5);
  }
  const auto left = bundle.sampleAtDeclaredStamp(10'200'000'000LL);
  const auto right = bundle.sampleAtDeclaredStamp(11'000'000'000LL);
  const auto middle = bundle.sampleAtDeclaredStamp(10'600'000'000LL);
  ASSERT_TRUE(left && right && middle);
  EXPECT_EQ(assess(
      bundle, (left->position_world + right->position_world) / 2.0,
      middle->velocity_world).status,
      PathRelativeTrackingStatus::kOutsidePathTube);
}

TEST(PathRelativeTracking, RejectsPhaseOutsideLocalWindowAtSearchBoundary) {
  const auto bundle = makeLinearMainBundle();
  EXPECT_EQ(assess(bundle, {5.0 * 0.72, 0.0, 0.0}, {5.0, 0.0, 0.0}).status,
            PathRelativeTrackingStatus::kPhaseExceeded);
  EXPECT_EQ(assess(bundle, {5.0 * 0.48, 0.0, 0.0}, {5.0, 0.0, 0.0}).status,
            PathRelativeTrackingStatus::kPhaseExceeded);
}

TEST(PathRelativeTracking, RejectsAmbiguousFigureEightProjection) {
  auto bundle = makeLinearMainBundle();
  bundle.evaluator = [](std::int64_t stamp, TrajectoryPoint& point) {
    point.trajectory_time_s = (stamp - 10'000'000'000LL) * 1.0e-9;
    const double omega = 2.0 * std::acos(-1.0) / 0.12;
    const double t = point.trajectory_time_s;
    point.position_world = {std::sin(omega * t), std::sin(2.0 * omega * t), 0.0};
    point.velocity_world = {
        omega * std::cos(omega * t), 2.0 * omega * std::cos(2.0 * omega * t), 0.0};
    point.role = point.trajectory_time_s < 1.5
        ? CandidateRole::kMain : CandidateRole::kBackup;
    return true;
  };
  const auto result = assess(bundle, {0.0, 0.0, 0.0}, {5.0, 0.0, 0.0});
  EXPECT_EQ(result.status, PathRelativeTrackingStatus::kAmbiguousProjection);
  EXPECT_LT(result.evaluation_count, 150U);
}


TEST(TrackingExperiment, BaseAndVelocityCoefficientsHaveMetreUnits) {
  navigation_contracts::TrackingExperimentPolicy policy;
  policy.enabled = true;
  policy.base_m = 0.2;
  policy.lateral_alpha_s = 0.05;
  policy.longitudinal_beta_s = 0.15;
  for (double speed : {0.0, 1.0, 3.0, 8.0}) {
    const auto a = navigation_contracts::assessAdaptiveTracking(
        policy, Eigen::Vector3d::Zero(), {speed, 0, 0},
        Eigen::Vector3d::Zero(), {speed, 0, 0});
    ASSERT_TRUE(a.valid);
    EXPECT_NEAR(a.lateral_limit_m, .2 + .05 * speed, 1e-12);
    EXPECT_NEAR(a.longitudinal_limit_m, .2 + .15 * speed, 1e-12);
  }
  const auto stopped_vehicle = navigation_contracts::assessAdaptiveTracking(
      policy, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(),
      Eigen::Vector3d::Zero(), {3, 0, 0});
  EXPECT_NEAR(stopped_vehicle.lateral_limit_m, .35, 1e-12);
  EXPECT_TRUE(navigation_contracts::assessAdaptiveTracking(policy,
      {.2, 0, 0}, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(),
      Eigen::Vector3d::Zero()).within_limits);
  EXPECT_FALSE(navigation_contracts::assessAdaptiveTracking(policy,
      {.201, 0, 0}, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(),
      Eigen::Vector3d::Zero()).within_limits);
}

TEST(TrackingExperiment, AdaptiveModeAllowsMetreProgressBeyondOldPhaseWindow) {
  navigation_contracts::TrackingExperimentPolicy policy;
  policy.enabled = true;
  policy.base_m = 0.2;
  policy.lateral_alpha_s = 0.05;
  policy.longitudinal_beta_s = 0.15;
  const auto b = makeLinearMainBundle(3.0);
  for (double phase : {-.15, .15}) {
    const Eigen::Vector3d p(3 * (.6 + phase), 0, 0);
    EXPECT_FALSE(assess(b, p, {3, 0, 0}).accepted());
    const auto r = navigation_runtime::assessExperimentalTracking(
        policy, b, p, {3, 0, 0}, 10'600'000'000LL, 10'600'000'000LL,
        .12, true, true, true);
    EXPECT_TRUE(r.accepted);
    EXPECT_FALSE(r.suppression_used);
  }
}

TEST(TrackingExperiment, RelaxedModeExplicitlyRecordsBypassedTracking) {
  navigation_contracts::TrackingExperimentPolicy policy;
  policy.enabled = true;
  policy.base_m = 0.2;
  policy.lateral_alpha_s = 0.05;
  policy.longitudinal_beta_s = 0.15;
  const auto b = makeLinearMainBundle(3.0);
  auto run = [&](const auto& p) { return navigation_runtime::assessExperimentalTracking(
      p, b, {1.8, .5, .5}, {3, 0, 0}, 10'600'000'000LL, 10'600'000'000LL,
      .12, true, true, true); };
  EXPECT_FALSE(run(policy).accepted);
  policy.suppress_braking = true;
  const auto relaxed = run(policy);
  EXPECT_TRUE(relaxed.accepted);
  EXPECT_TRUE(relaxed.suppression_used);
  EXPECT_GT(relaxed.current.lateral_error_m, relaxed.current.lateral_limit_m);
}

TEST(TrackingExperiment, RelaxedNeverBypassesInputWorldLeaseOrRoleChecks) {
  navigation_contracts::TrackingExperimentPolicy policy;
  policy.enabled = policy.suppress_braking = true;
  auto b = makeLinearMainBundle(3.0);
  auto run = [&](bool fresh, bool body, bool path) {
    return navigation_runtime::assessExperimentalTracking(
        policy, b, {1.8, .5, 0}, {3, 0, 0}, 10'600'000'000LL,
        10'600'000'000LL, .12, fresh, body, path);
  };
  EXPECT_FALSE(run(false, true, true).accepted);
  EXPECT_FALSE(run(true, false, true).accepted);
  EXPECT_FALSE(run(true, true, false).accepted);
  b.valid_until_ns = 10'650'000'000LL;
  EXPECT_FALSE(run(true, true, true).accepted);
  b = makeLinearMainBundle(3.0);
  b.terminal_stop = true;
  b.certificates.terminal_stop = true;
  EXPECT_TRUE(run(true, true, true).accepted);
  EXPECT_TRUE(run(true, true, true).support_valid);
  b = makeLinearMainBundle(3.0);
  b.role = navigation_planning::CandidateRole::kBackup;
  EXPECT_FALSE(run(true, true, true).accepted);
  b = makeLinearMainBundle(3.0);
  EXPECT_FALSE(navigation_runtime::assessExperimentalTracking(policy, b,
      Eigen::Vector3d::Constant(std::numeric_limits<double>::quiet_NaN()),
      {3, 0, 0}, 10'600'000'000LL, 10'600'000'000LL, .12, true, true, true).accepted);
}

TEST(TrackingExperiment, TerminalMainUsesAdaptiveAllowanceUntilExactEndpoint) {
  navigation_contracts::TrackingExperimentPolicy policy;
  policy.enabled = true;
  policy.base_m = 0.2;
  policy.lateral_alpha_s = 0.05;
  policy.longitudinal_beta_s = 0.15;
  const auto bundle = makeTerminalMainBundle();
  ASSERT_TRUE(bundle.valid());
  const auto reference = bundle.sample(11'880'000'000LL);
  ASSERT_TRUE(reference);
  const auto result = navigation_runtime::assessExperimentalTracking(
      policy, bundle, reference->position_world + Eigen::Vector3d{0.1, 0.0, 0.0},
      reference->velocity_world, 11'880'000'000LL, 11'880'000'000LL,
      0.12, true, true, true);
  EXPECT_TRUE(result.support_valid);
  EXPECT_TRUE(result.accepted);
  EXPECT_TRUE(result.predicted.valid);

  const auto endpoint = bundle.sampleAtDeclaredEnd();
  ASSERT_TRUE(endpoint);
  EXPECT_TRUE(endpoint->finished);
  EXPECT_NEAR(endpoint->position_world.x(), 5.0, 1.0e-9);
  EXPECT_NEAR(endpoint->velocity_world.norm(), 0.0, 1.0e-9);

  auto lease_ends_before_endpoint = bundle;
  lease_ends_before_endpoint.valid_until_ns = 11'950'000'000LL;
  EXPECT_FALSE(navigation_runtime::assessExperimentalTracking(
      policy, lease_ends_before_endpoint, reference->position_world,
      reference->velocity_world, 11'900'000'000LL, 11'900'000'000LL,
      0.2, true, true, true).accepted);

  auto near_endpoint_policy = policy;
  near_endpoint_policy.base_m = 0.3;
  const auto near_endpoint = bundle.sample(11'950'000'000LL);
  ASSERT_TRUE(near_endpoint);
  const auto clamped = navigation_runtime::assessExperimentalTracking(
      near_endpoint_policy, bundle,
      near_endpoint->position_world + Eigen::Vector3d{0.0, 0.0, 0.3},
      near_endpoint->velocity_world, 11'950'000'000LL, 11'950'000'000LL,
      0.12, true, true, true);
  EXPECT_TRUE(clamped.accepted);
  EXPECT_TRUE(clamped.predicted.valid);
  EXPECT_GT(near_endpoint_policy.base_m, 0.25);

  auto backup = bundle;
  backup.role = CandidateRole::kBackup;
  EXPECT_FALSE(navigation_runtime::assessExperimentalTracking(
      policy, backup, reference->position_world, reference->velocity_world,
      11'880'000'000LL, 11'880'000'000LL, 0.12, true, true, true).accepted);
}

TEST(TrackingExperiment, ZeroCoefficientsDisableGateAndInvalidValuesFail) {
  navigation_contracts::TrackingExperimentPolicy policy;
  policy.enabled = true;
  policy.suppress_braking = true;
  policy.suppress_estimator_health_response = true;
  EXPECT_TRUE(policy.valid());
  EXPECT_FALSE(policy.trackingGateEnabled());
  const auto disabled = navigation_contracts::assessAdaptiveTracking(
      policy, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(),
      Eigen::Vector3d{100.0, 0.0, 0.0}, Eigen::Vector3d::Zero());
  EXPECT_TRUE(navigation_contracts::experimentPermitsTracking(policy, disabled));

  policy.enabled = false;
  policy.suppress_estimator_health_response = true;
  EXPECT_FALSE(policy.valid());
  policy.suppress_estimator_health_response = false;
  policy.suppress_braking = true;
  EXPECT_FALSE(policy.valid());
  policy.enabled = true;
  policy.base_m = -0.1;
  EXPECT_FALSE(policy.valid());
  policy.base_m = 0.2;
  policy.lateral_alpha_s = -1;
  EXPECT_FALSE(policy.valid());
  policy.lateral_alpha_s = 0;
  policy.longitudinal_beta_s = std::numeric_limits<double>::infinity();
  EXPECT_FALSE(policy.valid());
}

}  // namespace
