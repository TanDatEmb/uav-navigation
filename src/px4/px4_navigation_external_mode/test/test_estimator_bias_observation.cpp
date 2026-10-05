#include <array>
#include <cstdint>
#include <limits>

#include <gtest/gtest.h>

#include "px4_navigation_external_mode/estimator_bias_observation.hpp"

namespace px4_navigation_external_mode {
namespace {

using BiasArrays = std::array<float, 3>;

TEST(EstimatorBiasObservation, AcceptsFiniteTimestampedIdentityBearingSample) {
  const auto observation = makeEstimatorBiasObservation(
      2000U, 1900U, 42U, BiasArrays{1.0F, -2.0F, 0.5F},
      BiasArrays{0.1F, 0.2F, 0.3F}, BiasArrays{0.01F, -0.02F, 0.03F},
      BiasArrays{0.4F, 0.5F, 0.6F}, BiasArrays{0.7F, 0.8F, 0.9F});

  ASSERT_TRUE(observation.has_value());
  EXPECT_EQ(observation->timestamp_us, 2000U);
  EXPECT_EQ(observation->timestamp_sample_us, 1900U);
  EXPECT_EQ(observation->device_id, 42U);
  EXPECT_DOUBLE_EQ(observation->bias_m.x(), 1.0);
  EXPECT_DOUBLE_EQ(observation->bias_m.y(), -2.0);
  EXPECT_DOUBLE_EQ(observation->bias_m.z(), 0.5);
}

TEST(EstimatorBiasObservation, RejectsMissingIdentityOrInvalidTime) {
  const BiasArrays finite{0.0F, 0.0F, 0.0F};
  EXPECT_FALSE(makeEstimatorBiasObservation(0U, 1U, 42U, finite, finite, finite,
                                            finite, finite));
  EXPECT_FALSE(makeEstimatorBiasObservation(2U, 3U, 42U, finite, finite, finite,
                                            finite, finite));
  EXPECT_FALSE(makeEstimatorBiasObservation(2U, 1U, 0U, finite, finite, finite,
                                            finite, finite));
}

TEST(EstimatorBiasObservation, RejectsNonFiniteAndNegativeGateValues) {
  const BiasArrays finite{0.0F, 0.0F, 0.0F};
  const BiasArrays nan{0.0F, 0.0F, std::numeric_limits<float>::quiet_NaN()};
  const BiasArrays negative{-1.0F, 0.0F, 0.0F};
  EXPECT_FALSE(makeEstimatorBiasObservation(2U, 1U, 42U, nan, finite, finite,
                                            finite, finite));
  EXPECT_FALSE(makeEstimatorBiasObservation(2U, 1U, 42U, finite, negative, finite,
                                            finite, finite));
  EXPECT_FALSE(makeEstimatorBiasObservation(2U, 1U, 42U, finite, finite, finite,
                                            finite, negative));
}

}  // namespace
}  // namespace px4_navigation_external_mode
