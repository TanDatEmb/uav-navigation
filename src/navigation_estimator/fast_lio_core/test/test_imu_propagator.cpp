#include <gtest/gtest.h>

#include <cmath>
#include <numbers>
#include <vector>

#include "fast_lio_core/common/constants.hpp"
#include "fast_lio_core/estimation/imu_propagator.hpp"

namespace uav::nav::lio {
namespace {

std::vector<ImuSample> makeImuSequence(const Eigen::Vector3d& angular_velocity,
                                       const Eigen::Vector3d& acceleration,
                                       std::int64_t duration_ns,
                                       std::int64_t step_ns = 10'000'000) {
  std::vector<ImuSample> samples;
  for (std::int64_t time_ns = 0; time_ns <= duration_ns; time_ns += step_ns) {
    ImuSample sample;
    sample.time = Timestamp(time_ns, ClockDomain::kSensorTime);
    sample.angular_velocity_imu_rad_s = angular_velocity;
    sample.linear_acceleration_imu_m_s2 = acceleration;
    samples.push_back(sample);
  }
  return samples;
}

TEST(ImuPropagatorTest, StationarySpecificForceCancelsGravity) {
  ManifoldState state;
  ManifoldState::Covariance covariance = ManifoldState::Covariance::Identity() * 1e-3;
  const auto samples = makeImuSequence(
      Eigen::Vector3d::Zero(), Eigen::Vector3d(0.0, 0.0, kStandardGravityMps2), 1'000'000'000);
  ImuPropagator propagator;
  const auto trajectory =
      propagator.propagate(state, covariance, samples, Timestamp(0, ClockDomain::kSensorTime),
                           Timestamp(1'000'000'000, ClockDomain::kSensorTime));
  ASSERT_TRUE(trajectory.ok()) << trajectory.status().message();
  EXPECT_TRUE(state.position_odom_imu_m().isZero(1e-9));
  EXPECT_TRUE(state.velocity_odom_imu_m_s().isZero(1e-9));
  EXPECT_TRUE(state.orientation_odom_imu().isApprox(Eigen::Quaterniond::Identity(), 1e-12));
  EXPECT_EQ(trajectory.value().size(), 101U);
  EXPECT_TRUE(covariance.allFinite());
}

TEST(ImuPropagatorTest, IntegratesConstantWorldAcceleration) {
  ManifoldState state;
  ManifoldState::Covariance covariance = ManifoldState::Covariance::Identity() * 1e-3;
  const auto samples = makeImuSequence(
      Eigen::Vector3d::Zero(), Eigen::Vector3d(1.0, 0.0, kStandardGravityMps2), 1'000'000'000);
  ImuPropagator propagator;
  const auto result =
      propagator.propagate(state, covariance, samples, Timestamp(0, ClockDomain::kSensorTime),
                           Timestamp(1'000'000'000, ClockDomain::kSensorTime));
  ASSERT_TRUE(result.ok()) << result.status().message();
  EXPECT_NEAR(state.velocity_odom_imu_m_s().x(), 1.0, 1e-9);
  EXPECT_NEAR(state.position_odom_imu_m().x(), 0.5, 1e-9);
  EXPECT_NEAR(state.position_odom_imu_m().y(), 0.0, 1e-9);
  EXPECT_NEAR(state.position_odom_imu_m().z(), 0.0, 1e-9);
}

TEST(ImuPropagatorTest, IntegratesPositiveYawUsingFluConvention) {
  ManifoldState state;
  ManifoldState::Covariance covariance = ManifoldState::Covariance::Identity() * 1e-3;
  const auto samples =
      makeImuSequence(Eigen::Vector3d(0.0, 0.0, std::numbers::pi / 2.0),
                      Eigen::Vector3d(0.0, 0.0, kStandardGravityMps2), 1'000'000'000);
  ImuPropagator propagator;
  const auto result =
      propagator.propagate(state, covariance, samples, Timestamp(0, ClockDomain::kSensorTime),
                           Timestamp(1'000'000'000, ClockDomain::kSensorTime));
  ASSERT_TRUE(result.ok()) << result.status().message();
  const Eigen::Vector3d forward_in_odom = state.orientation_odom_imu() * Eigen::Vector3d::UnitX();
  EXPECT_TRUE(forward_in_odom.isApprox(Eigen::Vector3d::UnitY(), 1e-9));
}

TEST(ImuPropagatorTest, RejectsUnbracketedInterval) {
  ManifoldState state;
  ManifoldState::Covariance covariance = ManifoldState::Covariance::Identity();
  const auto samples = makeImuSequence(
      Eigen::Vector3d::Zero(), Eigen::Vector3d(0.0, 0.0, kStandardGravityMps2), 100'000'000);
  ImuPropagator propagator;
  const auto result =
      propagator.propagate(state, covariance, samples, Timestamp(-1, ClockDomain::kSensorTime),
                           Timestamp(100'000'000, ClockDomain::kSensorTime));
  ASSERT_FALSE(result.ok());
  EXPECT_EQ(result.status().code(), StatusCode::kInsufficientData);
}

TEST(ImuPropagatorTest, FailureDoesNotPartiallyMutateState) {
  ManifoldState state;
  state.set_position_odom_imu_m(Eigen::Vector3d(3.0, 2.0, 1.0));
  const ManifoldState original_state = state;
  ManifoldState::Covariance covariance = ManifoldState::Covariance::Identity();
  const ManifoldState::Covariance original_covariance = covariance;
  const auto samples =
      makeImuSequence(Eigen::Vector3d::Zero(), Eigen::Vector3d(1.0, 0.0, kStandardGravityMps2),
                      100'000'000, 50'000'000);
  ImuPropagator propagator;
  const auto result =
      propagator.propagate(state, covariance, samples, Timestamp(0, ClockDomain::kSensorTime),
                           Timestamp(100'000'000, ClockDomain::kSensorTime));
  ASSERT_FALSE(result.ok());
  EXPECT_TRUE(state.position_odom_imu_m().isApprox(original_state.position_odom_imu_m()));
  EXPECT_TRUE(state.velocity_odom_imu_m_s().isApprox(original_state.velocity_odom_imu_m_s()));
  EXPECT_TRUE(covariance.isApprox(original_covariance));
}

TEST(ImuPropagatorTest, GravityTangentCovarianceAffectsVelocityAndPosition) {
  ManifoldState state;
  ManifoldState::Covariance covariance = ManifoldState::Covariance::Zero();
  covariance.block<2, 2>(ManifoldState::kGravityOffset, ManifoldState::kGravityOffset) =
      Eigen::Matrix2d::Identity();
  ImuPropagatorConfig config;
  config.noise = ImuNoise{0.0, 0.0, 0.0, 0.0};
  ImuPropagator propagator(config);
  const auto samples = makeImuSequence(Eigen::Vector3d::Zero(),
                                       Eigen::Vector3d(0.0, 0.0, kStandardGravityMps2), 10'000'000);
  const auto result =
      propagator.propagate(state, covariance, samples, Timestamp(0, ClockDomain::kSensorTime),
                           Timestamp(10'000'000, ClockDomain::kSensorTime));
  ASSERT_TRUE(result.ok()) << result.status().message();
  const Eigen::Matrix<double, 3, 2> basis = state.gravityTangentBasis();
  const Eigen::Matrix<double, 3, 2> velocity_gravity =
      covariance.block<3, 2>(ManifoldState::kVelocityOffset, ManifoldState::kGravityOffset);
  const Eigen::Matrix<double, 3, 2> position_gravity =
      covariance.block<3, 2>(ManifoldState::kPositionOffset, ManifoldState::kGravityOffset);
  EXPECT_TRUE(velocity_gravity.isApprox(basis * 0.01, 1e-12));
  EXPECT_TRUE(position_gravity.isApprox(0.5 * basis * 0.01 * 0.01, 1e-12));
}

}  // namespace
}  // namespace uav::nav::lio
