#include <gtest/gtest.h>

#include <cstdint>
#include <limits>

#include "fast_lio_core/time/timestamp.hpp"
#include "fast_lio_core/time/timestamp_validator.hpp"

namespace uav::nav::lio {
namespace {

TEST(TimestampTest, PreservesIntegerNanosecondsBeyondDoublePrecision) {
  constexpr std::int64_t timestamp_ns = 9'007'199'254'740'993LL;
  const Timestamp timestamp(timestamp_ns, ClockDomain::kSensorTime);
  EXPECT_EQ(timestamp.nanoseconds(), timestamp_ns);
  EXPECT_EQ(timestamp.clock_domain(), ClockDomain::kSensorTime);
}

TEST(TimestampTest, RejectsArithmeticAcrossClockDomains) {
  const Timestamp ros_time(100, ClockDomain::kRosTime);
  const Timestamp sensor_time(90, ClockDomain::kSensorTime);
  const auto difference = checkedDifference(ros_time, sensor_time);
  ASSERT_FALSE(difference.ok());
  EXPECT_EQ(difference.status().code(), StatusCode::kClockDomainMismatch);
}

TEST(TimestampTest, CheckedArithmeticRejectsOverflow) {
  const Timestamp maximum(std::numeric_limits<std::int64_t>::max(), ClockDomain::kSteadyTime);
  const auto sum = checkedAdd(maximum, Duration(1));
  ASSERT_FALSE(sum.ok());
  EXPECT_EQ(sum.status().code(), StatusCode::kOutOfRange);
}

TEST(TimestampValidatorTest, DetectsRegressionAndDomainChange) {
  TimestampValidator validator;
  EXPECT_TRUE(validator.validate(Timestamp(100, ClockDomain::kSimulationTime)).ok());
  const Status regression = validator.validate(Timestamp(99, ClockDomain::kSimulationTime));
  EXPECT_EQ(regression.code(), StatusCode::kTimestampRegression);
  const Status mismatch = validator.validate(Timestamp(101, ClockDomain::kRosTime));
  EXPECT_EQ(mismatch.code(), StatusCode::kClockDomainMismatch);
  EXPECT_EQ(validator.regressionCount(), 1U);
  EXPECT_EQ(validator.clockMismatchCount(), 1U);
}

}  // namespace
}  // namespace uav::nav::lio
