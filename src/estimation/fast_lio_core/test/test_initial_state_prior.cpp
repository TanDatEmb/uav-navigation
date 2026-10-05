#include <gtest/gtest.h>

#include <cmath>

#include "fast_lio_core/geometry/frame_ids.hpp"
#include "fast_lio_core/initialization/initial_state_prior_applicator.hpp"
#include "fast_lio_core/initialization/initial_state_prior_policy.hpp"

namespace uav::nav::lio {
namespace {

ManifoldState makeImuState(const Eigen::Quaterniond& orientation,
                           const Eigen::Vector3d& position = Eigen::Vector3d::Zero()) {
  ManifoldState state;
  state.set_orientation_odom_imu(orientation);
  state.set_position_odom_imu_m(position);
  state.set_gravity_odom_m_s2(Eigen::Vector3d(0.0, 0.0, -9.80665));
  state.normalize();
  return state;
}

InitialStatePrior makePrior(PriorAttitudeMode attitude = PriorAttitudeMode::kNone) {
  InitialStatePrior prior;
  prior.sample_time = Timestamp(100, ClockDomain::kSensorTime);
  prior.reference_frame = lioOdomFrame();
  prior.body_frame = baseFrame();
  prior.source = InitialStatePriorSource::kFixed;
  prior.mask = {true, true, attitude};
  prior.linear_velocity_base_m_s = Eigen::Vector3d::Zero();
  prior.angular_velocity_base_rad_s = Eigen::Vector3d::Zero();
  return prior;
}

TEST(InitialStatePriorTest, ZeroPositionMeansBaseOriginAndAppliesLeverArm) {
  InitialStatePriorApplicator applicator(RigidTransform(
      baseFrame(), imuFrame(), Eigen::Quaterniond::Identity(),
      Eigen::Vector3d(0.0, 0.0, 0.28)));
  ManifoldState output;
  const Status status = applicator.apply(makePrior(), makeImuState(Eigen::Quaterniond::Identity()),
                                         0.2, output);
  ASSERT_TRUE(status.ok()) << status.message();
  EXPECT_TRUE(output.position_odom_imu_m().isApprox(Eigen::Vector3d(0.0, 0.0, 0.28)));
  EXPECT_TRUE(output.velocity_odom_imu_m_s().isApprox(Eigen::Vector3d::Zero()));
}

TEST(InitialStatePriorTest, YawOnlyPreservesImuTiltAndWrapsAcrossPi) {
  const Eigen::Quaterniond imu_orientation(
      Eigen::AngleAxisd(0.1, Eigen::Vector3d::UnitZ()) *
      Eigen::AngleAxisd(0.3, Eigen::Vector3d::UnitY()) *
      Eigen::AngleAxisd(-0.2, Eigen::Vector3d::UnitX()));
  InitialStatePrior prior = makePrior(PriorAttitudeMode::kYawOnly);
  prior.position_odom_base_m.setZero();
  prior.orientation_odom_base = Eigen::Quaterniond(
      Eigen::AngleAxisd(-3.12, Eigen::Vector3d::UnitZ()));
  InitialStatePriorApplicator applicator(
      RigidTransform(baseFrame(), imuFrame(), Eigen::Quaterniond::Identity(),
                     Eigen::Vector3d::Zero()));
  ManifoldState output;
  ASSERT_TRUE(applicator.apply(prior, makeImuState(imu_orientation), 0.2, output).ok());
  const Eigen::Matrix3d expected =
      Eigen::AngleAxisd(3.063185307179586, Eigen::Vector3d::UnitZ()).toRotationMatrix() *
      imu_orientation.toRotationMatrix();
  EXPECT_TRUE(output.orientation_odom_imu().toRotationMatrix().isApprox(expected, 1e-10));
}

TEST(InitialStatePriorTest, FullAttitudeRejectsGravityTiltDisagreement) {
  InitialStatePrior prior = makePrior(PriorAttitudeMode::kFull);
  prior.orientation_odom_base = Eigen::Quaterniond(
      Eigen::AngleAxisd(0.5, Eigen::Vector3d::UnitY()));
  InitialStatePriorApplicator applicator(
      RigidTransform(baseFrame(), imuFrame(), Eigen::Quaterniond::Identity(),
                     Eigen::Vector3d::Zero()));
  ManifoldState output;
  const Status status = applicator.apply(prior, makeImuState(Eigen::Quaterniond::Identity()),
                                         0.1, output);
  EXPECT_EQ(status.code(), StatusCode::kInitializationRejected);
}

TEST(InitialStatePriorTest, VelocityUsesBaseTwistAndLeverArmAngularRate) {
  InitialStatePrior prior = makePrior();
  prior.linear_velocity_base_m_s = Eigen::Vector3d(1.0, 2.0, 3.0);
  prior.angular_velocity_base_rad_s = Eigen::Vector3d(0.0, 0.0, 2.0);
  const Eigen::Vector3d lever_arm(0.4, -0.2, 0.1);
  InitialStatePriorApplicator applicator(
      RigidTransform(baseFrame(), imuFrame(), Eigen::Quaterniond::Identity(), lever_arm));
  ManifoldState output;
  ASSERT_TRUE(applicator.apply(prior, makeImuState(Eigen::Quaterniond::Identity()), 0.2, output).ok());
  EXPECT_TRUE(output.velocity_odom_imu_m_s().isApprox(
      prior.linear_velocity_base_m_s.value() + prior.angular_velocity_base_rad_s->cross(lever_arm)));
}

TEST(InitialStatePriorTest, PolicyRejectsInFlightFallback) {
  InitialStatePriorPolicy policy;
  policy.source = InitialStatePriorSource::kTopic;
  policy.context = InitialStatePriorContext::kInFlightReinitialization;
  policy.ground_fallback = InitialPriorFallback::kZero;
  EXPECT_EQ(policy.validate().code(), StatusCode::kInvalidArgument);
}

}  // namespace
}  // namespace uav::nav::lio
