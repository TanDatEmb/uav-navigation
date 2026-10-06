#include <gtest/gtest.h>

#include <cstdint>
#include <limits>

#include "uavnav/px4bridge/px4_time.hpp"

using namespace uavnav;
using namespace uavnav::px4bridge;

TEST(Px4Time, IdentityInSimulationRealtimeRejected) {
  const auto ok = to_px4(time::SensorTime{5'000'000}, ClockMode::kSimulationIdentity);
  ASSERT_TRUE(ok.has_value());
  EXPECT_EQ(ok->ns, 5'000'000);
  const auto rt = to_px4(time::SensorTime{5'000'000}, ClockMode::kRealtime);
  ASSERT_FALSE(rt.has_value());
  EXPECT_EQ(rt.error(), Px4TimeError::kRealtimeNotSupported);
}

TEST(Px4Time, ZeroAndLargeValuesPass) {
  const auto zero = to_px4(time::SensorTime{0}, ClockMode::kSimulationIdentity);
  ASSERT_TRUE(zero.has_value());
  EXPECT_EQ(zero->ns, 0);
  const auto big = to_px4(time::SensorTime{std::numeric_limits<std::int64_t>::max()}, ClockMode::kSimulationIdentity);
  ASSERT_TRUE(big.has_value());
  EXPECT_EQ(big->ns, std::numeric_limits<std::int64_t>::max());
}

TEST(Px4Time, NegativeRejected) {
  const auto neg = to_px4(time::SensorTime{-1}, ClockMode::kSimulationIdentity);
  ASSERT_FALSE(neg.has_value());
  EXPECT_EQ(neg.error(), Px4TimeError::kNegative);
  const auto min = to_px4(time::SensorTime{std::numeric_limits<std::int64_t>::min()}, ClockMode::kSimulationIdentity);
  ASSERT_FALSE(min.has_value());
  EXPECT_EQ(min.error(), Px4TimeError::kNegative);
}

TEST(Px4Time, RealtimeErrorWinsOverNegative) {
  const auto r = to_px4(time::SensorTime{-5}, ClockMode::kRealtime);
  ASSERT_FALSE(r.has_value());
  EXPECT_EQ(r.error(), Px4TimeError::kRealtimeNotSupported);
}

TEST(Px4Time, UnknownModeFailsClosed) {
  const auto r = to_px4(time::SensorTime{1'000}, static_cast<ClockMode>(7));
  ASSERT_FALSE(r.has_value());
  EXPECT_EQ(r.error(), Px4TimeError::kRealtimeNotSupported);
}

TEST(Px4Time, MicrosecondsTruncateTowardZero) {
  EXPECT_EQ(to_px4_us(time::Px4Time{0}), 0U);
  EXPECT_EQ(to_px4_us(time::Px4Time{999}), 0U);
  EXPECT_EQ(to_px4_us(time::Px4Time{1'000}), 1U);
  EXPECT_EQ(to_px4_us(time::Px4Time{1'999}), 1U);
  EXPECT_EQ(to_px4_us(time::Px4Time{2'000}), 2U);
  EXPECT_EQ(to_px4_us(time::Px4Time{5'000'000'000}), 5'000'000U);
  EXPECT_EQ(to_px4_us(time::Px4Time{std::numeric_limits<std::int64_t>::max()}), 9'223'372'036'854'775U);
}

TEST(Px4Time, NegativeSaturatesToZero) {
  EXPECT_EQ(to_px4_us(time::Px4Time{-1}), 0U);
  EXPECT_EQ(to_px4_us(time::Px4Time{-5'000'000}), 0U);
  EXPECT_EQ(to_px4_us(time::Px4Time{std::numeric_limits<std::int64_t>::min()}), 0U);
}

TEST(Px4Time, RoundTripWithinOneMicrosecond) {
  for (std::int64_t ns : {0LL, 1LL, 999LL, 1'000LL, 123'456'789LL, 7'000'000'001LL}) {
    const auto p = to_px4(time::SensorTime{ns}, ClockMode::kSimulationIdentity);
    ASSERT_TRUE(p.has_value());
    const std::int64_t back = static_cast<std::int64_t>(to_px4_us(*p)) * 1000;
    EXPECT_LE(back, ns);
    EXPECT_LT(ns - back, 1000);
  }
}

TEST(Px4Time, ReasonNamesAreUpperSnake) {
  EXPECT_EQ(to_string(Px4TimeError::kRealtimeNotSupported), "REALTIME_NOT_SUPPORTED");
  EXPECT_EQ(to_string(Px4TimeError::kNegative), "NEGATIVE");
  static_assert(ReasonEnum<Px4TimeError>);
}
