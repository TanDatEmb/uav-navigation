#include <gtest/gtest.h>

#include <cstdint>
#include <limits>

#include <navigation_common/frame_conventions.hpp>
#include <navigation_common/time.hpp>

TEST(NavigationCommon, ConvertsRosTimeWithoutFloatingPointLoss) {
  builtin_interfaces::msg::Time source;
  source.sec = 42;
  source.nanosec = 123'456'789U;

  const auto nanoseconds = navigation_common::rosTimeToNanoseconds(source);
  ASSERT_TRUE(nanoseconds.has_value());
  EXPECT_EQ(*nanoseconds, 42'123'456'789LL);

  const auto round_trip = navigation_common::nanosecondsToRosTime(*nanoseconds);
  ASSERT_TRUE(round_trip.has_value());
  EXPECT_EQ(round_trip->sec, source.sec);
  EXPECT_EQ(round_trip->nanosec, source.nanosec);
}

TEST(NavigationCommon, RejectsMalformedAndNegativeRosTime) {
  builtin_interfaces::msg::Time malformed;
  malformed.sec = 1;
  malformed.nanosec = 1'000'000'000U;
  EXPECT_FALSE(navigation_common::rosTimeToNanoseconds(malformed).has_value());
  EXPECT_FALSE(navigation_common::nanosecondsToRosTime(-1).has_value());
}

TEST(NavigationCommon, RejectsSecondsOutsideRosTimeRangeBeforeNarrowing) {
  EXPECT_FALSE(navigation_common::secondsToRosTime(-1.0).has_value());
  EXPECT_FALSE(navigation_common::secondsToRosTime(
                   std::numeric_limits<double>::max())
                   .has_value());
  const auto maximum = navigation_common::secondsToRosTime(
      static_cast<double>(std::numeric_limits<std::int32_t>::max()));
  ASSERT_TRUE(maximum.has_value());
  EXPECT_EQ(maximum->sec, std::numeric_limits<std::int32_t>::max());
  EXPECT_EQ(maximum->nanosec, 0U);
}

TEST(NavigationCommon, ConvertsPositiveMicrosecondsWithOverflowProtection) {
  const auto converted = navigation_common::microsecondsToNanoseconds(2'345U);
  ASSERT_TRUE(converted.has_value());
  EXPECT_EQ(*converted, 2'345'000LL);
  EXPECT_FALSE(navigation_common::microsecondsToNanoseconds(0U).has_value());
  EXPECT_FALSE(navigation_common::microsecondsToNanoseconds(
                   std::numeric_limits<std::uint64_t>::max())
                   .has_value());
}

TEST(NavigationCommon, SecondsToNanosecondsChecksRoundedProductBoundary) {
  const double max_seconds = static_cast<double>(std::numeric_limits<std::int64_t>::max()) /
                             static_cast<double>(navigation_common::kNanosecondsPerSecond);
  const auto near_max = navigation_common::secondsToNanoseconds(
      std::nextafter(max_seconds, 0.0));
  if (near_max.has_value()) {
    EXPECT_LE(*near_max, std::numeric_limits<std::int64_t>::max());
  }
  EXPECT_FALSE(navigation_common::secondsToNanoseconds(
                   std::numeric_limits<double>::max()).has_value());
  EXPECT_FALSE(navigation_common::secondsSumToNanoseconds(
                   std::numeric_limits<double>::max(), 1.0).has_value());
}

TEST(NavigationCommon, ConvertsNanosecondsToPositiveMicroseconds) {
  const auto converted = navigation_common::nanosecondsToMicroseconds(2'345'678LL);
  ASSERT_TRUE(converted.has_value());
  EXPECT_EQ(*converted, 2'345U);
  EXPECT_FALSE(navigation_common::nanosecondsToMicroseconds(999).has_value());
  EXPECT_FALSE(navigation_common::nanosecondsToMicroseconds(0).has_value());
  EXPECT_FALSE(navigation_common::nanosecondsToMicroseconds(-1).has_value());
}

TEST(NavigationCommon, ProvidesAStableMonotonicSteadyClockSample) {
  const auto first = navigation_common::steadyClockNowNanoseconds();
  const auto second = navigation_common::steadyClockNowNanoseconds();
  EXPECT_GE(second, first);
}

TEST(NavigationCommon, FrameConversionsAreSelfInverse) {
  const Eigen::Vector3d enu(1.0, 2.0, 3.0);
  const Eigen::Vector3d flu(-4.0, 5.0, -6.0);
  EXPECT_TRUE(navigation_common::nedToEnu(navigation_common::enuToNed(enu)).isApprox(enu));
  EXPECT_TRUE(navigation_common::frdToFlu(navigation_common::fluToFrd(flu)).isApprox(flu));
}
