#include "navigation_runtime/timestamp_freshness.hpp"

#include <gtest/gtest.h>

namespace navigation_runtime {

TEST(TimestampFreshness, DistinguishesPastFutureAndInvalidTime) {
  constexpr std::int64_t now = 10'000;
  constexpr std::int64_t limit = 500;
  EXPECT_EQ(classifyTimestampFreshness(now, 0, limit), TimestampFreshness::INVALID);
  EXPECT_EQ(classifyTimestampFreshness(now, now - limit - 1, limit),
            TimestampFreshness::STALE);
  EXPECT_EQ(classifyTimestampFreshness(now, now + limit + 1, limit),
            TimestampFreshness::FUTURE);
}

TEST(TimestampFreshness, BoundaryIsAcceptedWithoutAbsoluteTimeComparison) {
  constexpr std::int64_t now = 10'000;
  constexpr std::int64_t limit = 500;
  EXPECT_EQ(classifyTimestampFreshness(now, now - limit, limit),
            TimestampFreshness::VALID);
  EXPECT_EQ(classifyTimestampFreshness(now, now + limit, limit),
            TimestampFreshness::VALID);
}

}  // namespace navigation_runtime
