#include <navigation_runtime/world_temporal_assessment.hpp>

#include <gtest/gtest.h>

namespace {

using navigation_runtime::WorldTemporalReason;
using navigation_runtime::assessWorldTemporal;

navigation_world_model::WorldSnapshotIdentity identity(std::int64_t stamp) {
  return {1U, 2U, 3U, stamp};
}

TEST(WorldTemporalAssessment, DistinguishesMissingSnapshotAndSourceStamp) {
  EXPECT_EQ(assessWorldTemporal(false, identity(10), 20, 5).reason,
            WorldTemporalReason::kNoSnapshot);
  EXPECT_EQ(assessWorldTemporal(true, identity(0), 20, 5).reason,
            WorldTemporalReason::kSourceStampMissing);
}

TEST(WorldTemporalAssessment, PreservesCurrentStaleAndFutureBoundaries) {
  EXPECT_EQ(assessWorldTemporal(true, identity(100), 105, 5).reason,
            WorldTemporalReason::kCurrent);
  EXPECT_EQ(assessWorldTemporal(true, identity(99), 105, 5).reason,
            WorldTemporalReason::kSourceStale);
  // Existing timestamp contract accepts small bounded future skew and rejects
  // only future evidence beyond the same configured window.
  EXPECT_EQ(assessWorldTemporal(true, identity(110), 105, 5).reason,
            WorldTemporalReason::kCurrent);
  EXPECT_EQ(assessWorldTemporal(true, identity(111), 105, 5).reason,
            WorldTemporalReason::kSourceFuture);
}

TEST(WorldTemporalAssessment, RejectsInvalidClockOrAgeContract) {
  EXPECT_EQ(assessWorldTemporal(true, identity(10), 0, 5).reason,
            WorldTemporalReason::kInvalidTimeContract);
  EXPECT_EQ(assessWorldTemporal(true, identity(10), 20, 0).reason,
            WorldTemporalReason::kInvalidTimeContract);
}

TEST(WorldTemporalAssessment, KeepsIdentityAsEvidenceWithoutCallingItFreshness) {
  const auto source_identity = identity(100);
  const auto assessment = assessWorldTemporal(true, source_identity, 100, 1);
  EXPECT_TRUE(assessment.sourceCurrent());
  EXPECT_EQ(assessment.identity.localization_epoch,
            source_identity.localization_epoch);
  EXPECT_EQ(assessment.identity.generation, source_identity.generation);
  EXPECT_EQ(assessment.identity.revision, source_identity.revision);
  EXPECT_EQ(assessment.identity.observation_stamp_ns,
            source_identity.observation_stamp_ns);
}

}  // namespace
