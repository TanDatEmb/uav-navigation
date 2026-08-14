#include <gtest/gtest.h>

#include "fast_lio_core/mapping/dynamic_map_evidence.hpp"

namespace uav::nav::lio {

TEST(DynamicMapEvidenceTest, DisabledFlagPreservesBaseline) {
  DynamicMapEvidence evidence;
  const std::vector<Eigen::Vector3d> points{{1.0, 0.0, 0.0}};
  evidence.observeHits(points, 1);
  EXPECT_EQ(evidence.voxelCount(), 0U);
  EXPECT_EQ(evidence.candidateCount(100), 0U);
}

TEST(DynamicMapEvidenceTest, OcclusionIsNotFreeSpaceContradiction) {
  DynamicFilterConfig config;
  config.enabled = true;
  DynamicMapEvidence evidence(config);
  const Eigen::Vector3d point{5.0, 0.0, 0.0};
  const std::vector<Eigen::Vector3d> hits{point, point};
  evidence.observeHits(hits, 1);
  FreeSpaceObservation observation{
      true, true, true, true, true, 5.0, 3.0, 0.5};
  EXPECT_FALSE(evidence.observeContradiction(point, observation, 10));
  EXPECT_EQ(evidence.candidateCount(10), 0U);
}

TEST(DynamicMapEvidenceTest,
     RepeatedIndependentFreeSpaceEvidenceOnlyMarksCandidate) {
  DynamicFilterConfig config;
  config.enabled = true;
  DynamicMapEvidence evidence(config);
  const Eigen::Vector3d point{5.0, 0.0, 0.0};
  const std::vector<Eigen::Vector3d> hits{point, point};
  evidence.observeHits(hits, 1);
  FreeSpaceObservation observation{
      true, true, true, true, false, 5.0, 8.0, 0.5};
  EXPECT_FALSE(evidence.observeContradiction(point, observation, 4));
  EXPECT_FALSE(evidence.observeContradiction(point, observation, 5));
  EXPECT_TRUE(evidence.observeContradiction(point, observation, 6));
  EXPECT_EQ(evidence.candidateCount(6), 1U);
}

TEST(DynamicMapEvidenceTest, RejectsInvalidThresholdHysteresis) {
  DynamicFilterConfig config;
  config.keep_threshold = config.insert_threshold;
  EXPECT_THROW(static_cast<void>(DynamicMapEvidence{config}),
               std::invalid_argument);
}

}  // namespace uav::nav::lio
