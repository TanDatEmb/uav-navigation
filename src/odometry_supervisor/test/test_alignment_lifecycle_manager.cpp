#include <gtest/gtest.h>

#include <cmath>
#include <stdexcept>

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

AlignmentRevalidationObservation revalidation(std::int64_t epoch_ns, std::uint64_t evidence_id,
                                               double position = 0.1,
                                               double velocity = 0.1,
                                               double orientation = 0.1,
                                               double yaw = 0.1) {
  AlignmentRevalidationObservation observation;
  observation.exact_time_pair_valid = true;
  observation.residual.valid = true;
  observation.residual.heading_observable = true;
  observation.residual.timestamp_ns = epoch_ns;
  observation.residual.position_error_m = position;
  observation.residual.velocity_error_m_s = velocity;
  observation.residual.orientation_error_rad = orientation;
  observation.residual.yaw_error_rad = yaw;
  observation.epoch_ns = epoch_ns;
  observation.lio_generation = 7;
  observation.frame_generation = 1;
  observation.time_generation = 3;
  observation.evidence_id = evidence_id;
  return observation;
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

TEST(AlignmentLifecycleManager, CompensatedResetKeepsTransformButFrameChangeClearsIt) {
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

TEST(AlignmentLifecycleManager, UncompensatedResetInvalidatesLockedTransformImmediately) {
  auto value = manager();
  lock(value);
  value.observeResetEvent(1, 2'000'000'000, false);
  EXPECT_EQ(value.state(), AlignmentLifecycleState::kInvalid);
  EXPECT_FALSE(value.lockedAlignment().has_value());
}

TEST(AlignmentLifecycleManager, RevalidationKeepsFrozenTransformAndReturnsAfterKSamples) {
  auto value = manager();
  lock(value, 4.0, 0.2);
  const auto frozen = *value.lockedAlignment();
  value.beginRevalidation("temporary diagnostics loss", 5'000'000'000);
  EXPECT_TRUE(value.revalidating());
  for (int i = 0; i < 2; ++i) {
    EXPECT_FALSE(value.observeRevalidation(
        revalidation(5'100'000'000 + i * 10'000'000, static_cast<std::uint64_t>(i + 1))));
  }
  EXPECT_TRUE(value.observeRevalidation(revalidation(5'200'000'000, 3)));
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
  EXPECT_FALSE(value.observeRevalidation(revalidation(1, 1, 10.0)));
  EXPECT_FALSE(value.observeRevalidation(revalidation(2, 2, 10.0)));
  EXPECT_FALSE(value.observeRevalidation(revalidation(3, 3, 10.0)));
  EXPECT_EQ(value.state(), AlignmentLifecycleState::kInvalid);
  EXPECT_FALSE(value.lockedAlignment().has_value());
}

TEST(AlignmentLifecycleManager, LargeYawResidualFailsRevalidation) {
  auto value = manager();
  lock(value);
  value.beginRevalidation("residual validation", 1'000'000'000);
  EXPECT_FALSE(value.observeRevalidation(revalidation(1, 1, 0.1, 0.1, 0.1, 1.0)));
  EXPECT_EQ(value.snapshot().revalidation_failure_count, 1U);
}

TEST(AlignmentLifecycleManager, DuplicateRevalidationEvidenceDoesNotCount) {
  auto value = manager();
  lock(value);
  value.beginRevalidation("residual validation", 1'000'000'000);
  EXPECT_FALSE(value.observeRevalidation(revalidation(1, 1)));
  EXPECT_FALSE(value.observeRevalidation(revalidation(1, 2)));
  EXPECT_EQ(value.snapshot().revalidation_sample_count, 1U);
  EXPECT_EQ(value.snapshot().revalidation_failure_count, 0U);
}

TEST(AlignmentLifecycleManager, ValidRevalidationDoesNotChangeFrozenTransform) {
  auto value = manager();
  lock(value, 4.0, 0.2);
  const auto frozen = *value.lockedAlignment();
  value.beginRevalidation("residual validation", 1'000'000'000);
  EXPECT_FALSE(value.observeRevalidation(revalidation(1, 1)));
  EXPECT_FALSE(value.observeRevalidation(revalidation(2, 2)));
  EXPECT_TRUE(value.observeRevalidation(revalidation(3, 3)));
  ASSERT_TRUE(value.lockedAlignment().has_value());
  EXPECT_TRUE(value.lockedAlignment()->target_from_source_translation.isApprox(
      frozen.target_from_source_translation));
  EXPECT_TRUE(value.lockedAlignment()->target_from_source_orientation.coeffs().isApprox(
      frozen.target_from_source_orientation.coeffs()));
}

TEST(AlignmentLifecycleManager, TransportFailureDoesNotBecomeGeometricFailure) {
  auto value = manager();
  lock(value);
  value.beginRevalidation("temporary loss", 1'000'000'000);
  value.observeTransportFailure("query timeout");
  EXPECT_TRUE(value.revalidating());
  EXPECT_EQ(value.snapshot().revalidation_sample_count, 0U);
  EXPECT_EQ(value.snapshot().revalidation_failure_count, 0U);
  EXPECT_TRUE(value.lockedAlignment().has_value());
}

TEST(AlignmentLifecycleManager, InvalidLifecycleConfigurationIsRejected) {
  AlignmentLifecycleConfig config;
  config.stable_candidate_estimates = 0;
  EXPECT_THROW({ auto unused = AlignmentLifecycleManager(config); (void)unused; },
               std::invalid_argument);

  config = {};
  config.candidate_history_capacity = 1;
  config.stable_candidate_estimates = 2;
  EXPECT_THROW({ auto unused = AlignmentLifecycleManager(config); (void)unused; },
               std::invalid_argument);

  config = {};
  config.revalidation_residual.yaw_rad = 0.0;
  EXPECT_THROW({ auto unused = AlignmentLifecycleManager(config); (void)unused; },
               std::invalid_argument);
}

TEST(AlignmentLifecycleManager, AsymmetricOrNonPsdCovarianceIsRejected) {
  auto asymmetric = alignment(0.0, 0.0);
  asymmetric.covariance(0, 1) = 0.1;
  auto value = manager();
  EXPECT_FALSE(value.observeCandidate(candidate(1, asymmetric)));

  auto non_psd = alignment(0.0, 0.0);
  non_psd.covariance(0, 0) = -0.1;
  value = manager();
  EXPECT_FALSE(value.observeCandidate(candidate(1, non_psd)));
}

TEST(AlignmentLifecycleManager, CandidateNeverActsAsProductionAlignment) {
  auto value = manager();
  EXPECT_TRUE(value.observeCandidate(candidate(1, alignment(0.0, 0.0))));
  EXPECT_TRUE(value.observeCandidate(candidate(2, alignment(0.0, 0.0))));
  EXPECT_EQ(value.state(), AlignmentLifecycleState::kProvisional);
  EXPECT_TRUE(value.candidateValid());
  EXPECT_FALSE(value.locked());
}
