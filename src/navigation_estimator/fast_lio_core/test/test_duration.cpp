#include <gtest/gtest.h>

#include <cstdint>
#include <limits>

#include "fast_lio_core/time/duration.hpp"

namespace uav::nav::lio {
namespace {

TEST(DurationTest, ConvertsOnlyRelativeDurationToSeconds) {
  const Duration duration(1'500'000'000);
  EXPECT_DOUBLE_EQ(duration.seconds(), 1.5);
  EXPECT_EQ(duration.nanoseconds(), 1'500'000'000);
}

TEST(DurationTest, SupportsNegativeDurations) {
  const Duration duration(-250'000'000);
  EXPECT_DOUBLE_EQ(duration.seconds(), -0.25);
}

TEST(DurationTest, CheckedAddRejectsOverflow) {
  const auto result = checkedAdd(Duration(std::numeric_limits<std::int64_t>::max()), Duration(1));
  ASSERT_FALSE(result.ok());
  EXPECT_EQ(result.status().code(), StatusCode::kOutOfRange);
}

TEST(DurationTest, CheckedSubtractReturnsExactNanoseconds) {
  const auto result = checkedSubtract(Duration(10'000'000'001), Duration(9'000'000'000));
  ASSERT_TRUE(result.ok());
  EXPECT_EQ(result.value().nanoseconds(), 1'000'000'001);
}

TEST(DurationTest, SubtractingTwoMinimumValuesIsValid) {
  const auto result = checkedSubtract(Duration(std::numeric_limits<std::int64_t>::min()),
                                      Duration(std::numeric_limits<std::int64_t>::min()));
  ASSERT_TRUE(result.ok());
  EXPECT_EQ(result.value().nanoseconds(), 0);
}

}  // namespace
}  // namespace uav::nav::lio
