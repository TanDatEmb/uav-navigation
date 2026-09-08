#include <gtest/gtest.h>

#include <navigation_runtime/path_relative_tracking.hpp>

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

PathRelativeTrackingResult assess(
    const CandidateBundle& bundle, const Eigen::Vector3d& position,
    const Eigen::Vector3d& velocity, std::int64_t now_ns = 10'600'000'000LL,
    std::int64_t source_stamp_ns = 10'600'000'000LL) {
  return navigation_runtime::assessPathRelativeTracking(
      bundle, position, velocity, now_ns, source_stamp_ns, 0.12, 0.10,
      0.25, 0.75, true, true, true);
}

TEST(PathRelativeTracking, AcceptsPhysicalPhaseLeadAndLagOnTheSamePath) {
  for (const double speed : {3.0, 5.0, 8.0}) {
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

}  // namespace
