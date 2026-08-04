#include <gtest/gtest.h>

#include "odometry_supervisor/query_failure_accounting.hpp"

TEST(QueryFailureAccounting, CountsEachTerminalFailureOnceAndKeepsSubsets) {
  odometry_supervisor::QueryFailureCounters counters;
  counters.record(odometry_supervisor::QueryFailureKind::kTransport);
  counters.record(odometry_supervisor::QueryFailureKind::kGenerationMismatch);
  counters.record(odometry_supervisor::QueryFailureKind::kGeometric);
  counters.record(odometry_supervisor::QueryFailureKind::kContract);

  EXPECT_EQ(counters.failure_count, 4U);
  EXPECT_EQ(counters.transport_failure_count, 1U);
  EXPECT_EQ(counters.generation_mismatch_count, 1U);
  EXPECT_EQ(counters.geometric_failure_count, 1U);
  EXPECT_LE(counters.transport_failure_count, counters.failure_count);
  EXPECT_LE(counters.generation_mismatch_count, counters.failure_count);
  EXPECT_LE(counters.geometric_failure_count, counters.failure_count);
}
