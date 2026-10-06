#include <gtest/gtest.h>

#include <cstdint>
#include <type_traits>

#include "uavnav/core/time.hpp"

using namespace uavnav::time;

template <class A, class B>
concept Subtractable = requires(A a, B b) { a - b; };
template <class A, class B>
concept Addable = requires(A a, B b) { a + b; };

// Same-domain arithmetic compiles; cross-domain and raw-integer mixes do not.
static_assert(Subtractable<RosTime, RosTime>);
static_assert(!Subtractable<RosTime, SensorTime>);
static_assert(!Subtractable<SteadyTime, Px4Time>);
static_assert(!Subtractable<Px4Time, SensorTime>);
static_assert(!Subtractable<RosTime, std::int64_t>);
static_assert(!Subtractable<std::int64_t, RosTime>);
static_assert(Addable<RosTime, Duration>);
static_assert(!Addable<RosTime, RosTime>);
static_assert(!Addable<RosTime, std::int64_t>);
static_assert(!Addable<Duration, std::int64_t>);
static_assert(std::is_same_v<decltype(RosTime{} - RosTime{}), Duration>);
static_assert(std::is_same_v<decltype(RosTime{} + Duration{}), RosTime>);

// No implicit conversion between domains, nor from/to raw integers.
static_assert(!std::is_convertible_v<SensorTime, RosTime>);
static_assert(!std::is_convertible_v<RosTime, SteadyTime>);
static_assert(!std::is_convertible_v<Px4Time, SensorTime>);
static_assert(!std::is_convertible_v<std::int64_t, SteadyTime>);
static_assert(!std::is_convertible_v<std::int64_t, Duration>);
static_assert(!std::is_convertible_v<SteadyTime, std::int64_t>);
static_assert(!std::is_convertible_v<Duration, std::int64_t>);
static_assert(!std::is_convertible_v<Duration, double>);
static_assert(!std::is_same_v<SensorTime, RosTime>);

// Cross-domain comparison must not compile either.
template <class A, class B>
concept Comparable = requires(A a, B b) { a < b; };
static_assert(Comparable<RosTime, RosTime>);
static_assert(!Comparable<RosTime, SensorTime>);

// Constants fold at compile time.
static_assert(milliseconds(250).ns == 250'000'000);
static_assert(seconds(2).ns == 2'000'000'000);
static_assert(nanoseconds(-7).ns == -7);
static_assert(to_seconds(milliseconds(500)) == 0.5);

TEST(Time, ArithmeticWithinDomain) {
  const RosTime a{1'000'000'000};
  const RosTime b = a + milliseconds(250);
  EXPECT_EQ((b - a).ns, 250'000'000);
  EXPECT_EQ((a - b).ns, -250'000'000);  // ordering preserved, no clamping
  EXPECT_DOUBLE_EQ(to_seconds(b - a), 0.25);
  EXPECT_LT(a, b);
}

TEST(Time, NegativeAndSubtractedDurationsAreKept) {
  const SensorTime t{100};
  EXPECT_EQ((t - seconds(1)).ns, 100 - 1'000'000'000);  // goes negative, not clamped to 0
  EXPECT_EQ((milliseconds(1) - seconds(1)).ns, -999'000'000);
  EXPECT_EQ((seconds(1) + milliseconds(1)).ns, 1'001'000'000);
  EXPECT_DOUBLE_EQ(to_seconds(nanoseconds(-1'500'000'000)), -1.5);
}

TEST(Time, DefaultIsZeroAndComparesEqual) {
  EXPECT_EQ(Px4Time{}.ns, 0);
  EXPECT_EQ(Px4Time{5}, Px4Time{5});
  EXPECT_NE(Px4Time{5}, Px4Time{6});
  EXPECT_EQ(Duration{}, nanoseconds(0));
}

TEST(Time, SnapshotKeepsBothDomains) {
  const TimeSnapshot s{SteadyTime{10}, RosTime{20}};
  EXPECT_EQ(s.steady.ns, 10);
  EXPECT_EQ(s.ros.ns, 20);
}

TEST(Time, SteadyNowIsMonotonic) {
  const SteadyTime t0 = steady_now();
  const SteadyTime t1 = steady_now();
  EXPECT_LE(t0, t1);
}
