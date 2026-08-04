#include <gtest/gtest.h>

#include "px4_odometry_bridge/timestamp_conversion.hpp"

namespace px4_odometry_bridge {
namespace {

TEST(TimestampConversionTest, ValidMonotonicSimulationTimestamps) {
  TimestampConverter converter(150'000'000);
  const auto result = converter.convert(1'000'000'000, 1'020'000'000, true, 7);
  EXPECT_TRUE(result.valid);
  EXPECT_EQ(result.source_domain, "ROS_SIMULATION_TIME");
  EXPECT_EQ(result.target_domain, "PX4_SIMULATION_TIME");
  EXPECT_EQ(result.measurement_time_us, 1'000'000U);
  EXPECT_EQ(result.publication_time_us, 1'020'000U);
  EXPECT_EQ(result.generation, 7U);
  EXPECT_EQ(result.timestamp_age_ns, 20'000'000);
  EXPECT_TRUE(converter.convert(1'001'000'000, 1'021'000'000, true, 7).valid);
}

TEST(TimestampConversionTest, RegressionIsRejectedAndCounted) {
  TimestampConverter converter(150'000'000);
  ASSERT_TRUE(converter.convert(2'000'000'000, 2'001'000'000, true, 1).valid);
  const auto result = converter.convert(1'999'000'000, 2'002'000'000, true, 1);
  EXPECT_FALSE(result.valid);
  EXPECT_EQ(result.reason, "TIMESTAMP_REGRESSION");
  EXPECT_EQ(converter.diagnostics().regression_count, 1U);
  EXPECT_EQ(converter.diagnostics().conversion_failure_count, 1U);
}

TEST(TimestampConversionTest, GenerationChangeStartsANewMonotonicEpoch) {
  TimestampConverter converter(150'000'000);
  ASSERT_TRUE(converter.convert(3'000'000'000, 3'001'000'000, true, 1).valid);
  EXPECT_TRUE(converter.convert(1'000'000'000, 1'001'000'000, true, 2).valid);
}

TEST(TimestampConversionTest, StaleFutureUnresolvedAndZeroAreFailClosed) {
  TimestampConverter stale(150'000'000);
  EXPECT_EQ(stale.convert(1'000'000'000, 1'200'000'001, true, 1).reason,
            "TIMESTAMP_STALE");
  TimestampConverter future(150'000'000);
  EXPECT_EQ(future.convert(1'200'000'001, 1'000'000'000, true, 1).reason,
            "TIMESTAMP_FUTURE");
  TimestampConverter unresolved(150'000'000);
  EXPECT_EQ(unresolved.convert(1'000'000'000, 1'001'000'000, false, 1).reason,
            "TIME_DOMAIN_UNRESOLVED");
  TimestampConverter zero(150'000'000);
  EXPECT_EQ(zero.convert(0, 1'000'000, true, 1).reason,
            "TIMESTAMP_ZERO_OR_OVERFLOW");
  EXPECT_FALSE(nanoseconds_to_microseconds(0).has_value());
}

}  // namespace
}  // namespace px4_odometry_bridge
