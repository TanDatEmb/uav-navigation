#include <gtest/gtest.h>

#include <cmath>
#include <cstdint>
#include <limits>

#include "uavnav/core/time_convert.hpp"

using namespace uavnav::time;

static_assert(uavnav::ReasonEnum<TimeError>);

TEST(TimeConvert, ErrorNames) {
  EXPECT_EQ(to_string(TimeError::kNegativeSeconds), "NEGATIVE_SECONDS");
  EXPECT_EQ(to_string(TimeError::kNanosecondsOutOfRange), "NANOSECONDS_OUT_OF_RANGE");
  EXPECT_EQ(to_string(TimeError::kNotFinite), "NOT_FINITE");
  EXPECT_EQ(to_string(TimeError::kOutOfRange), "OUT_OF_RANGE");
}

TEST(TimeConvert, AcceptsValidStamp) {
  EXPECT_EQ(from_stamp<RosTag>(2, 500).value().ns, 2'000'000'500);
  EXPECT_EQ(from_stamp<SensorTag>(0, 0).value().ns, 0);
  EXPECT_EQ(from_stamp<SteadyTag>(1, 999'999'999).value().ns, 1'999'999'999);
  EXPECT_EQ(from_stamp<Px4Tag>(0, 7).value().ns, 7);
}

TEST(TimeConvert, AcceptsUpperBound) {
  const auto r = from_stamp<RosTag>(9'000'000'000, 0);
  ASSERT_TRUE(r.has_value());
  EXPECT_EQ(r->ns, 9'000'000'000'000'000'000);
}

TEST(TimeConvert, RejectsBadStamps) {
  EXPECT_EQ(from_stamp<RosTag>(-1, 0).error(), TimeError::kNegativeSeconds);
  EXPECT_EQ(from_stamp<RosTag>(1, 1'000'000'000).error(), TimeError::kNanosecondsOutOfRange);
  EXPECT_EQ(from_stamp<RosTag>(9'000'000'001, 0).error(), TimeError::kOutOfRange);
}

TEST(TimeConvert, UpperBoundPlusNanosecondsIsOutOfRange) {
  // 9e9 s + 1 ns would still fit int64 (9.000000000000000001e18 < 9.22e18) but the contract is sec <= 9e9
  // with any nanosec; verify the largest legal stamp is accepted and the next second is not.
  EXPECT_TRUE(from_stamp<RosTag>(9'000'000'000, 999'999'999).has_value());
  EXPECT_EQ(from_stamp<RosTag>(9'000'000'001, 999'999'999).error(), TimeError::kOutOfRange);
}

TEST(TimeConvert, NoOverflowOnExtremeInputs) {
  constexpr auto kMax = std::numeric_limits<std::int64_t>::max();
  constexpr auto kMin = std::numeric_limits<std::int64_t>::min();
  EXPECT_EQ(from_stamp<RosTag>(kMax, 0).error(), TimeError::kOutOfRange);
  EXPECT_EQ(from_stamp<RosTag>(kMin, 0).error(), TimeError::kNegativeSeconds);
  EXPECT_EQ(from_stamp<RosTag>(kMax, std::numeric_limits<std::uint32_t>::max()).error(),
            TimeError::kNanosecondsOutOfRange);
}

TEST(TimeConvert, ErrorPrecedence) {
  // Negative seconds beats bad nanoseconds; bad nanoseconds beats out-of-range seconds.
  EXPECT_EQ(from_stamp<RosTag>(-1, 2'000'000'000).error(), TimeError::kNegativeSeconds);
  EXPECT_EQ(from_stamp<RosTag>(9'000'000'001, 1'000'000'000).error(), TimeError::kNanosecondsOutOfRange);
}

TEST(TimeConvert, DurationFromSeconds) {
  EXPECT_EQ(duration_from_seconds(0.25).value().ns, 250'000'000);
  EXPECT_EQ(duration_from_seconds(-0.25).value().ns, -250'000'000);
  EXPECT_EQ(duration_from_seconds(0.0).value().ns, 0);
  EXPECT_EQ(duration_from_seconds(1e6).value().ns, 1'000'000'000'000'000);
  EXPECT_EQ(duration_from_seconds(-1e6).value().ns, -1'000'000'000'000'000);
  EXPECT_EQ(duration_from_seconds(1.4e-9).value().ns, 1);  // rounds to nearest
  EXPECT_EQ(duration_from_seconds(1.6e-9).value().ns, 2);
  EXPECT_EQ(duration_from_seconds(-1.6e-9).value().ns, -2);
}

TEST(TimeConvert, DurationFromSecondsRejects) {
  constexpr double kInf = std::numeric_limits<double>::infinity();
  EXPECT_EQ(duration_from_seconds(std::nan("")).error(), TimeError::kNotFinite);
  EXPECT_EQ(duration_from_seconds(kInf).error(), TimeError::kNotFinite);
  EXPECT_EQ(duration_from_seconds(-kInf).error(), TimeError::kNotFinite);
  EXPECT_EQ(duration_from_seconds(2e6).error(), TimeError::kOutOfRange);
  EXPECT_EQ(duration_from_seconds(-2e6).error(), TimeError::kOutOfRange);
  EXPECT_EQ(duration_from_seconds(1e6 + 1e-3).error(), TimeError::kOutOfRange);
  EXPECT_EQ(duration_from_seconds(1e300).error(), TimeError::kOutOfRange);
}
