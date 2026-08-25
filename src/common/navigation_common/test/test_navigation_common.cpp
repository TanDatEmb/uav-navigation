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

TEST(NavigationCommon, ConvertsPositiveMicrosecondsWithOverflowProtection) {
  const auto converted = navigation_common::microsecondsToNanoseconds(2'345U);
  ASSERT_TRUE(converted.has_value());
  EXPECT_EQ(*converted, 2'345'000LL);
  EXPECT_FALSE(navigation_common::microsecondsToNanoseconds(0U).has_value());
  EXPECT_FALSE(navigation_common::microsecondsToNanoseconds(
                   std::numeric_limits<std::uint64_t>::max())
                   .has_value());
}

TEST(NavigationCommon, FrameConversionsAreSelfInverse) {
  const Eigen::Vector3d enu(1.0, 2.0, 3.0);
  const Eigen::Vector3d flu(-4.0, 5.0, -6.0);
  EXPECT_TRUE(navigation_common::nedToEnu(navigation_common::enuToNed(enu)).isApprox(enu));
  EXPECT_TRUE(navigation_common::frdToFlu(navigation_common::fluToFrd(flu)).isApprox(flu));
}
