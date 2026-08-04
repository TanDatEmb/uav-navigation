#include <gtest/gtest.h>

#include <cmath>

#include "odometry_supervisor/alignment_lifecycle_manager.hpp"

namespace {

using odometry_supervisor::AlignmentCandidateObservation;
using odometry_supervisor::AlignmentLifecycleConfig;
using odometry_supervisor::AlignmentLifecycleManager;
using odometry_supervisor::AlignmentLifecycleState;
using odometry_supervisor::AlignmentRevalidationObservation;
using odometry_supervisor::WorldAlignment;

WorldAlignment alignment(double x, double yaw, double covariance = 0.01) {
  WorldAlignment value;
  value.valid = true;
  value.target_frame = "lio_odom";
  value.source_frame = "px4_odom";
  value.target_from_source_translation = Eigen::Vector3d(x, 2.0, 0.0);
  value.yaw_rad = yaw;
  value.target_from_source_orientation = Eigen::Quaterniond(
      Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()));
  value.covariance = Eigen::Matrix4d::Identity() * covariance;
  value.epoch_ns = 1'000'000'000;
  return value;
}

AlignmentCandidateObservation candidate(std::uint64_t id, WorldAlignment value,
                                         std::size_t novel = 4,
                                         std::uint64_t frame = 1) {
  return {std::move(value), 7, frame, 3, id, novel};
}

AlignmentLifecycleManager manager() {
  AlignmentLifecycleConfig config;
  config.stable_candidate_estimates = 3;
  config.minimum_novel_pairs = 4;
  config.candidate_history_capacity = 4;
  config.max_translation_step_m = 0.10;
  config.max_yaw_step_rad = 0.05;
  config.max_cluster_translation_m = 0.15;
  config.max_cluster_yaw_rad = 0.10;
  return AlignmentLifecycleManager(config);
}

void lock(AlignmentLifecycleManager& value, double x = 0.0, double yaw = 0.0) {
  ASSERT_TRUE(value.observeCandidate(candidate(1, alignment(x, yaw))));
  ASSERT_TRUE(value.observeCandidate(candidate(2, alignment(x, yaw))));
  ASSERT_TRUE(value.observeCandidate(candidate(3, alignment(x, yaw))));
  ASSERT_TRUE(value.locked());
}

}  // namespace

TEST(AlignmentLifecycleManager, IdenticalThreeCandidatesLock) {
  auto value = manager();
  lock(value);
  EXPECT_EQ(value.state(), AlignmentLifecycleState::kLocked);
  ASSERT_TRUE(value.lockedAlignment().has_value());
  EXPECT_DOUBLE_EQ(value.lockedAlignment()->yaw_rad, 0.0);
  EXPECT_EQ(value.snapshot().candidate_estimate_count, 3U);
}

TEST(AlignmentLifecycleManager, UnstableCandidateResetsProvisionalCluster) {
  auto value = manager();
  EXPECT_TRUE(value.observeCandidate(candidate(1, alignment(0.0, 0.0))));
  EXPECT_TRUE(value.observeCandidate(candidate(2, alignment(0.0, 0.0))));
  EXPECT_FALSE(value.observeCandidate(candidate(3, alignment(0.11, 0.0))));
  EXPECT_EQ(value.state(), AlignmentLifecycleState::kProvisional);
  EXPECT_EQ(value.snapshot().stable_candidate_count, 1U);
  EXPECT_FALSE(value.locked());
}

TEST(AlignmentLifecycleManager, ClusterDiameterRejectsSlowTranslationCreep) {
  auto value = manager();
  EXPECT_TRUE(value.observeCandidate(candidate(1, alignment(0.0, 0.0))));
  EXPECT_TRUE(value.observeCandidate(candidate(2, alignment(0.09, 0.0))));
  EXPECT_FALSE(value.observeCandidate(candidate(3, alignment(0.18, 0.0))));
  EXPECT_FALSE(value.locked());
  EXPECT_EQ(value.snapshot().stable_candidate_count, 1U);
}

TEST(AlignmentLifecycleManager, YawWrapUsesCircularDistance) {
  auto value = manager();
  EXPECT_TRUE(value.observeCandidate(candidate(1, alignment(0.0, 3.13))));
  EXPECT_TRUE(value.observeCandidate(candidate(2, alignment(0.0, -3.14))));
  EXPECT_TRUE(value.observeCandidate(candidate(3, alignment(0.0, 3.135))));
  EXPECT_TRUE(value.locked());
}

TEST(AlignmentLifecycleManager, OverlapDoesNotCountWithoutNovelPairs) {
  auto value = manager();
  EXPECT_FALSE(value.observeCandidate(candidate(1, alignment(0.0, 0.0), 1)));
  EXPECT_FALSE(value.observeCandidate(candidate(2, alignment(0.0, 0.0), 1)));
  EXPECT_FALSE(value.observeCandidate(candidate(3, alignment(0.0, 0.0), 1)));
  EXPECT_FALSE(value.observeCandidate(candidate(4, alignment(0.0, 0.0), 0)));
  EXPECT_EQ(value.snapshot().candidate_estimate_count, 0U);
  EXPECT_FALSE(value.locked());
  EXPECT_TRUE(value.observeCandidate(candidate(5, alignment(0.0, 0.0), 1)));
  EXPECT_TRUE(value.observeCandidate(candidate(6, alignment(0.0, 0.0), 4)));
  EXPECT_TRUE(value.observeCandidate(candidate(7, alignment(0.0, 0.0), 4)));
  EXPECT_TRUE(value.locked());
}

TEST(AlignmentLifecycleManager, CovarianceNisRejectsInconsistentCandidate) {
  auto value = manager();
  EXPECT_TRUE(value.observeCandidate(candidate(1, alignment(0.0, 0.0, 1e-6))));
  EXPECT_FALSE(value.observeCandidate(candidate(2, alignment(0.01, 0.0, 1e-6))));
  EXPECT_FALSE(value.observeCandidate(candidate(3, alignment(0.02, 0.0, 1e-6))));
  EXPECT_FALSE(value.locked());
}

TEST(AlignmentLifecycleManager, FrameChangeClearsFrozenTransformButResetEventDoesNot) {
  auto value = manager();
  lock(value);
  const auto frozen = value.lockedAlignment()->target_from_source_translation;
  value.observeResetEvent(1, 2'000'000'000, true);
  EXPECT_TRUE(value.revalidating());
  EXPECT_TRUE(value.lockedAlignment()->target_from_source_translation.isApprox(frozen));
  value.observeBindingGeneration(7, 2, 3);
  EXPECT_FALSE(value.locked());
  EXPECT_FALSE(value.lockedAlignment().has_value());
}

TEST(AlignmentLifecycleManager, RevalidationKeepsFrozenTransformAndReturnsAfterKSamples) {
  auto value = manager();
  lock(value, 4.0, 0.2);
  const auto frozen = *value.lockedAlignment();
  value.beginRevalidation("temporary diagnostics loss", 5'000'000'000);
  EXPECT_TRUE(value.revalidating());
  for (int i = 0; i < 2; ++i) {
    EXPECT_FALSE(value.observeRevalidation({true, true, 5'100'000'000 + i * 10'000'000,
                                            7, 1, 3}));
  }
  EXPECT_TRUE(value.observeRevalidation({true, true, 5'200'000'000, 7, 1, 3}));
  EXPECT_TRUE(value.locked());
  EXPECT_TRUE(value.lockedAlignment()->target_from_source_translation.isApprox(
      frozen.target_from_source_translation));
  EXPECT_TRUE(value.lockedAlignment()->target_from_source_orientation.coeffs().isApprox(
      frozen.target_from_source_orientation.coeffs()));
}

TEST(AlignmentLifecycleManager, RevalidationFailureEventuallyInvalidates) {
  auto value = manager();
  lock(value);
  value.beginRevalidation("residual validation failure", 1'000'000'000);
  EXPECT_FALSE(value.observeRevalidation({false, false, 1, 7, 1, 3}));
  EXPECT_FALSE(value.observeRevalidation({false, false, 2, 7, 1, 3}));
  EXPECT_FALSE(value.observeRevalidation({false, false, 3, 7, 1, 3}));
  EXPECT_EQ(value.state(), AlignmentLifecycleState::kInvalid);
  EXPECT_FALSE(value.lockedAlignment().has_value());
}

TEST(AlignmentLifecycleManager, CandidateNeverActsAsProductionAlignment) {
  auto value = manager();
  EXPECT_TRUE(value.observeCandidate(candidate(1, alignment(0.0, 0.0))));
  EXPECT_TRUE(value.observeCandidate(candidate(2, alignment(0.0, 0.0))));
  EXPECT_EQ(value.state(), AlignmentLifecycleState::kProvisional);
  EXPECT_TRUE(value.candidateValid());
  EXPECT_FALSE(value.locked());
}
