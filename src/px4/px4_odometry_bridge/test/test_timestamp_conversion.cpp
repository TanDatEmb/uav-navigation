#include <gtest/gtest.h>

#include "px4_odometry_bridge/geometric_jump_latch.hpp"
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
  EXPECT_EQ(result.timestamp_mapping_generation, 7U);
  EXPECT_EQ(result.timestamp_age_ns, 20'000'000);
  EXPECT_TRUE(converter.convert(1'001'000'000, 1'021'000'000, true, 7).valid);
}

TEST(TimestampConversionTest, EqualSimulationPublicationEpochIsMonotonic) {
  TimestampConverter converter(150'000'000);
  ASSERT_TRUE(converter.convert(1'000'000'000, 1'020'000'000, true, 7).valid);
  const auto result = converter.convert(1'001'000'000, 1'020'000'000, true, 7);
  EXPECT_TRUE(result.valid);
  EXPECT_EQ(converter.diagnostics().regression_count, 0U);
}

TEST(TimestampConversionTest, EqualMeasurementEpochIsSuppressedWithoutFailure) {
  TimestampConverter converter(150'000'000);
  ASSERT_TRUE(converter.convert(2'000'000'000, 2'001'000'000, true, 1).valid);
  const auto result = converter.convert(2'000'000'999, 2'002'000'999, true, 1);
  EXPECT_FALSE(result.valid);
  EXPECT_TRUE(result.suppressed);
  EXPECT_EQ(result.reason, "DUPLICATE_MEASUREMENT_SUPPRESSED");
  EXPECT_EQ(converter.diagnostics().duplicate_measurement_suppressed_count, 1U);
  EXPECT_EQ(converter.diagnostics().regression_count, 0U);
  EXPECT_EQ(converter.diagnostics().conversion_failure_count, 0U);
}

TEST(TimestampConversionTest, SampleRegressionIsRejectedAndCounted) {
  TimestampConverter converter(150'000'000);
  ASSERT_TRUE(converter.convert(2'001'000'000, 2'002'000'000, true, 1).valid);
  const auto result = converter.convert(2'000'000'000, 2'003'000'000, true, 1);
  EXPECT_FALSE(result.valid);
  EXPECT_EQ(result.reason, "TIMESTAMP_SAMPLE_REGRESSION");
  EXPECT_EQ(converter.diagnostics().regression_count, 1U);
  EXPECT_EQ(converter.diagnostics().timestamp_sample_regression_count, 1U);
  EXPECT_EQ(converter.diagnostics().publication_timestamp_regression_count, 0U);
  EXPECT_EQ(converter.diagnostics().conversion_failure_count, 1U);
}

TEST(TimestampConversionTest, PublicationRegressionIsRejectedAndCounted) {
  TimestampConverter converter(150'000'000);
  ASSERT_TRUE(converter.convert(2'000'000'000, 2'002'000'000, true, 1).valid);
  const auto result = converter.convert(2'001'000'000, 2'001'000'999, true, 1);
  EXPECT_FALSE(result.valid);
  EXPECT_EQ(result.reason, "PUBLICATION_TIMESTAMP_REGRESSION");
  EXPECT_EQ(converter.diagnostics().publication_timestamp_regression_count, 1U);
  EXPECT_EQ(converter.diagnostics().timestamp_sample_regression_count, 0U);
  EXPECT_EQ(converter.diagnostics().conversion_failure_count, 1U);
}

TEST(TimestampConversionTest, TimestampMappingGenerationChangeStartsANewEpoch) {
  TimestampConverter converter(150'000'000);
  ASSERT_TRUE(converter.convert(3'000'000'000, 3'001'000'000, true, 1).valid);
  EXPECT_TRUE(converter.convert(1'000'000'000, 1'001'000'000, true, 2).valid);
  EXPECT_EQ(converter.diagnostics().timestamp_mapping_generation, 2U);
  EXPECT_EQ(converter.diagnostics().timestamp_mapping_generation_change_count, 1U);
  EXPECT_FALSE(converter.convert(999'000'000, 1'000'000'000, true, 2).valid);
}

TEST(TimestampConversionTest, PublicFrameGenerationDoesNotResetTimestampMapping) {
  TimestampConverter converter(150'000'000);
  ASSERT_TRUE(converter.convert(3'000'000'000, 3'001'000'000, true, 7).valid);
  // A public-frame generation change is intentionally not an argument to the
  // converter. Keeping mapping generation at 7 models the unchanged clock
  // mapping while the producer changes its reset-counter generation.
  EXPECT_FALSE(converter.convert(2'000'000'000, 2'001'000'000, true, 7).valid);
  EXPECT_EQ(converter.diagnostics().timestamp_mapping_generation_change_count, 0U);
  EXPECT_EQ(public_frame_generation_to_reset_counter(1), 1U);
  EXPECT_EQ(public_frame_generation_to_reset_counter(2), 2U);
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
