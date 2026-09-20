#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <vector>

#include "fast_lio_core/navigation/angular_velocity_resolver.hpp"

namespace uav::nav::lio {
namespace {

ImuSample sample(const std::int64_t time_ns, const Eigen::Vector3d& omega) {
  return ImuSample{Timestamp(time_ns), omega, Eigen::Vector3d(0.0, 0.0, 9.8)};
}

StateEstimate estimate(const std::int64_t time_ns,
                       const Eigen::Vector3d& gyro_bias = Eigen::Vector3d::Zero()) {
  StateEstimate result;
  result.time = Timestamp(time_ns);
  result.state.set_gyro_bias_rad_s(gyro_bias);
  return result;
}

TEST(AngularVelocityResolverTest, ExactSampleSubtractsBiasAtSameEpoch) {
  AngularVelocityDiagnostics diagnostics;
  const auto resolved = AngularVelocityResolver::resolve(
      estimate(10, {0.1, 0.2, 0.3}),
      std::vector<ImuSample>{sample(1, {0.0, 0.0, 0.0}),
                             sample(10, {1.0, 2.0, 3.0})},
      &diagnostics);
  ASSERT_TRUE(resolved.ok()) << resolved.status().message();
  EXPECT_TRUE(resolved.value().angular_velocity_imu_rad_s.isApprox(
      Eigen::Vector3d(0.9, 1.8, 2.7)));
  EXPECT_EQ(diagnostics.exact_sample_count, 1U);
  EXPECT_EQ(diagnostics.interpolated_count, 0U);
}

TEST(AngularVelocityResolverTest, InterpolatesOnlyInsideIntegerEpochBracket) {
  AngularVelocityDiagnostics diagnostics;
  const auto resolved = AngularVelocityResolver::resolve(
      estimate(1'500'000'000LL),
      std::vector<ImuSample>{sample(1'000'000'000LL, {0.0, 0.0, 0.0}),
                             sample(2'000'000'000LL, {2.0, 4.0, 6.0})},
      &diagnostics);
  ASSERT_TRUE(resolved.ok()) << resolved.status().message();
  EXPECT_TRUE(resolved.value().angular_velocity_imu_rad_s.isApprox(
      Eigen::Vector3d(1.0, 2.0, 3.0)));
  EXPECT_EQ(diagnostics.interpolated_count, 1U);
}

TEST(AngularVelocityResolverTest, MissingBracketDoesNotExtrapolate) {
  AngularVelocityDiagnostics diagnostics;
  const auto resolved = AngularVelocityResolver::resolve(
      estimate(5), std::vector<ImuSample>{sample(10, Eigen::Vector3d::Ones())},
      &diagnostics);
  EXPECT_FALSE(resolved.ok());
  EXPECT_EQ(resolved.status().code(), StatusCode::kMissingStartBracket);
  EXPECT_EQ(diagnostics.missing_bracket_count, 1U);
}

TEST(AngularVelocityResolverTest, RejectsDuplicateAndNonFiniteSamples) {
  AngularVelocityDiagnostics diagnostics;
  const auto duplicate = AngularVelocityResolver::resolve(
      estimate(10),
      std::vector<ImuSample>{sample(1, Eigen::Vector3d::Zero()),
                             sample(10, Eigen::Vector3d::Ones()),
                             sample(10, Eigen::Vector3d::Ones())},
      &diagnostics);
  EXPECT_FALSE(duplicate.ok());
  EXPECT_EQ(duplicate.status().code(), StatusCode::kTimestampRegression);

  const auto nonfinite = AngularVelocityResolver::resolveExact(
      estimate(20), sample(20, Eigen::Vector3d(
                                 std::numeric_limits<double>::quiet_NaN(), 0.0,
                                 0.0)),
      &diagnostics);
  EXPECT_FALSE(nonfinite.ok());
  EXPECT_EQ(nonfinite.status().code(), StatusCode::kInvalidArgument);
  EXPECT_EQ(diagnostics.nonfinite_reject_count, 1U);
}

}  // namespace
}  // namespace uav::nav::lio
