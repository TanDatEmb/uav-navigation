#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <random>

#include <Eigen/Eigenvalues>

#include "fast_lio_core/estimation/ikfom_state.hpp"
#include "fast_lio_core/geometry/frame_ids.hpp"
#include "fast_lio_core/navigation/base_link_covariance_projector.hpp"
#include "fast_lio_core/navigation/base_link_state_converter.hpp"

namespace uav::nav::lio {
namespace {

static_assert(ManifoldState::kErrorStateDimension == 23);
static_assert(ManifoldState::kPositionOffset == 0);
static_assert(ManifoldState::kOrientationOffset == 3);
static_assert(ManifoldState::kExtrinsicRotationOffset == 6);
static_assert(ManifoldState::kExtrinsicPositionOffset == 9);
static_assert(ManifoldState::kVelocityOffset == 12);
static_assert(ManifoldState::kGyroBiasOffset == 15);
static_assert(ManifoldState::kAccelBiasOffset == 18);
static_assert(ManifoldState::kGravityOffset == 21);

ManifoldState fromIkfom(const IkfomState& state) {
  ManifoldState output;
  output.set_position_odom_imu_m(state.pos);
  output.set_orientation_odom_imu(Eigen::Quaterniond(
      state.rot.w(), state.rot.x(), state.rot.y(), state.rot.z()));
  output.set_rotation_imu_lidar(Eigen::Quaterniond(
      state.offset_R_L_I.w(), state.offset_R_L_I.x(), state.offset_R_L_I.y(),
      state.offset_R_L_I.z()));
  output.set_position_imu_lidar_m(state.offset_T_L_I);
  output.set_velocity_odom_imu_m_s(state.vel);
  output.set_gyro_bias_rad_s(state.bg);
  output.set_accel_bias_m_s2(state.ba);
  output.set_gravity_odom_m_s2(state.grav.get_vect());
  output.normalize();
  return output;
}

IkfomState makeIkfomState() {
  IkfomState state;
  state.pos = Eigen::Vector3d(1.2, -0.7, 2.1);
  state.rot = IkfomSo3{Eigen::Quaterniond(
      Eigen::AngleAxisd(0.63, Eigen::Vector3d(0.3, -0.5, 0.8).normalized()))};
  state.offset_R_L_I = IkfomSo3{Eigen::Quaterniond(
      Eigen::AngleAxisd(-0.27, Eigen::Vector3d(-0.4, 0.7, 0.2).normalized()))};
  state.offset_T_L_I = Eigen::Vector3d(0.2, -0.1, 0.08);
  state.vel = Eigen::Vector3d(1.1, -0.4, 0.6);
  state.bg = Eigen::Vector3d(0.04, -0.03, 0.02);
  state.ba = Eigen::Vector3d(-0.1, 0.06, 0.03);
  state.grav = IkfomGravity{IkfomVector3{Eigen::Vector3d(0.2, -0.3, -9.8)}};
  return state;
}

StateEstimate makeEstimate(const IkfomState& state) {
  StateEstimate estimate;
  estimate.time = Timestamp(123456789, ClockDomain::kSensorTime);
  estimate.state = fromIkfom(state);
  estimate.covariance = ManifoldState::Covariance::Identity();
  return estimate;
}

BaseLinkStateConverter makeConverter(const Eigen::Quaterniond& rotation,
                                     const Eigen::Vector3d& translation) {
  return BaseLinkStateConverter(
      RigidTransform(baseFrame(), imuFrame(), rotation, translation));
}

KinematicStateEstimate makeKinematic(const StateEstimate& estimate,
                                      const Eigen::Vector3d& raw_gyro) {
  return KinematicStateEstimate{estimate, raw_gyro};
}

Eigen::Vector3d rotationLog(const Eigen::Matrix3d& rotation) {
  const Eigen::AngleAxisd angle_axis(rotation);
  return angle_axis.axis() * angle_axis.angle();
}

Eigen::Matrix<double, 6, 1> twistValue(const RigidBodyState& state) {
  return (Eigen::Matrix<double, 6, 1>() << state.linear_velocity_body_m_s,
          *state.angular_velocity_body_rad_s)
      .finished();
}

void expectJacobiansMatchFiniteDifference(
    const Eigen::Quaterniond& rotation_base_imu,
    const Eigen::Vector3d& translation_base_imu, const IkfomState& nominal,
    const Eigen::Vector3d& raw_gyro) {
  const BaseLinkStateConverter converter =
      makeConverter(rotation_base_imu, translation_base_imu);
  const BaseLinkCovarianceProjector projector(converter.baseToImu());
  const StateEstimate nominal_estimate = makeEstimate(nominal);
  const auto nominal_base = converter.convert(
      nominal_estimate, raw_gyro - nominal_estimate.state.gyro_bias_rad_s());
  ASSERT_TRUE(nominal_base.ok()) << nominal_base.status().message();
  const KinematicStateEstimate nominal_kinematic =
      makeKinematic(nominal_estimate, raw_gyro);
  const auto analytical_pose =
      projector.poseStateJacobian(nominal_kinematic, nominal_base.value());
  const auto analytical_twist =
      projector.twistStateJacobian(nominal_kinematic, nominal_base.value());

  constexpr double kEpsilon = 1e-7;
  Matrix6x23d numerical_pose = Matrix6x23d::Zero();
  Matrix6x23d numerical_twist = Matrix6x23d::Zero();
  for (int column = 0; column < ManifoldState::kErrorStateDimension; ++column) {
    IkfomState plus = nominal;
    IkfomState minus = nominal;
    Eigen::Matrix<double, IkfomState::DOF, 1> delta =
        Eigen::Matrix<double, IkfomState::DOF, 1>::Zero();
    delta[column] = kEpsilon;
    plus.boxplus(delta);
    delta[column] = -kEpsilon;
    minus.boxplus(delta);
    const auto plus_estimate = makeEstimate(plus);
    const auto minus_estimate = makeEstimate(minus);
    const auto plus_base = converter.convert(
        plus_estimate, raw_gyro - plus_estimate.state.gyro_bias_rad_s());
    const auto minus_base = converter.convert(
        minus_estimate, raw_gyro - minus_estimate.state.gyro_bias_rad_s());
    ASSERT_TRUE(plus_base.ok()) << plus_base.status().message();
    ASSERT_TRUE(minus_base.ok()) << minus_base.status().message();
    numerical_pose.col(column).template head<3>() =
        (plus_base.value().position_reference_body_m -
         minus_base.value().position_reference_body_m) /
        (2.0 * kEpsilon);
    numerical_pose.col(column).template tail<3>() =
        rotationLog(plus_base.value().orientation_reference_body.toRotationMatrix() *
                    minus_base.value().orientation_reference_body.toRotationMatrix()
                        .transpose()) /
        (2.0 * kEpsilon);
    numerical_twist.col(column) =
        (twistValue(plus_base.value()) - twistValue(minus_base.value())) /
        (2.0 * kEpsilon);
  }
  const double pose_error = (analytical_pose - numerical_pose).cwiseAbs().maxCoeff();
  const double twist_error =
      (analytical_twist - numerical_twist).cwiseAbs().maxCoeff();
  EXPECT_LT(pose_error, 2e-6);
  EXPECT_LT(twist_error, 2e-6);
}

TEST(BaseLinkCovarianceProjectorTest, StateMappingIsThe23DofSourceContract) {
  EXPECT_EQ(ManifoldState::kGravityOffset + 2,
            ManifoldState::kErrorStateDimension);
  EXPECT_EQ(ManifoldState::kOrientationOffset + 3,
            ManifoldState::kExtrinsicRotationOffset);
  EXPECT_EQ(ManifoldState::kExtrinsicRotationOffset + 3,
            ManifoldState::kExtrinsicPositionOffset);
  EXPECT_EQ(ManifoldState::kExtrinsicPositionOffset + 3,
            ManifoldState::kVelocityOffset);
  EXPECT_EQ(ManifoldState::kVelocityOffset + 3, ManifoldState::kGyroBiasOffset);
  EXPECT_EQ(ManifoldState::kGyroBiasOffset + 3, ManifoldState::kAccelBiasOffset);
  EXPECT_EQ(ManifoldState::kAccelBiasOffset + 3, ManifoldState::kGravityOffset);
}

TEST(BaseLinkCovarianceProjectorTest, IdentityGeometryUsesStateAndBiasBlocks) {
  const auto state = makeIkfomState();
  auto estimate = makeEstimate(state);
  estimate.covariance.setZero();
  estimate.covariance.block<3, 3>(ManifoldState::kPositionOffset,
                                 ManifoldState::kPositionOffset) =
      2.0 * Eigen::Matrix3d::Identity();
  estimate.covariance.block<3, 3>(ManifoldState::kOrientationOffset,
                                 ManifoldState::kOrientationOffset) =
      3.0 * Eigen::Matrix3d::Identity();
  estimate.covariance.block<3, 3>(ManifoldState::kVelocityOffset,
                                 ManifoldState::kVelocityOffset) =
      4.0 * Eigen::Matrix3d::Identity();
  estimate.covariance.block<3, 3>(ManifoldState::kGyroBiasOffset,
                                 ManifoldState::kGyroBiasOffset) =
      5.0 * Eigen::Matrix3d::Identity();
  const auto converter =
      makeConverter(Eigen::Quaterniond::Identity(), Eigen::Vector3d::Zero());
  const BaseLinkCovarianceProjector projector(converter.baseToImu());
  const Eigen::Vector3d raw_gyro(0.4, -0.2, 0.7);
  const auto base = converter.convert(
      estimate, raw_gyro - estimate.state.gyro_bias_rad_s());
  ASSERT_TRUE(base.ok());
  const auto result = projector.project(makeKinematic(estimate, raw_gyro),
                                         base.value());
  ASSERT_TRUE(result.ok()) << result.status().message();
  EXPECT_TRUE((result.value().pose_covariance_odom.block<3, 3>(0, 0).isApprox(
      2.0 * Eigen::Matrix3d::Identity(), 1e-12)));
  EXPECT_TRUE((result.value().twist_covariance_base.block<3, 3>(3, 3).isApprox(
      5.0 * Eigen::Matrix3d::Identity(), 1e-12)));
}

TEST(BaseLinkCovarianceProjectorTest, FullNonidentityGeometryJacobianPasses) {
  const Eigen::Quaterniond rotation_base_imu(
      Eigen::AngleAxisd(-0.42, Eigen::Vector3d(0.3, 0.7, -0.4).normalized()));
  expectJacobiansMatchFiniteDifference(
      rotation_base_imu, Eigen::Vector3d(0.37, -0.82, 1.13), makeIkfomState(),
      Eigen::Vector3d(0.6, -1.1, 0.8));
}

TEST(BaseLinkCovarianceProjectorTest, ZeroLeverArmRemovesDirectLeverTerms) {
  const auto state = makeIkfomState();
  auto estimate = makeEstimate(state);
  const auto converter =
      makeConverter(Eigen::Quaterniond::Identity(), Eigen::Vector3d::Zero());
  const BaseLinkCovarianceProjector projector(converter.baseToImu());
  const auto base = converter.convert(
      estimate, Eigen::Vector3d(0.2, -0.4, 0.8) - estimate.state.gyro_bias_rad_s());
  ASSERT_TRUE(base.ok());
  const auto jacobian = projector.twistStateJacobian(
      makeKinematic(estimate, Eigen::Vector3d(0.2, -0.4, 0.8)), base.value());
  EXPECT_TRUE((jacobian.block<3, 3>(0, ManifoldState::kGyroBiasOffset).isZero()));
}

TEST(BaseLinkCovarianceProjectorTest, CrossCovarianceIsPreservedAndExtrinsicsAreZero) {
  const auto state = makeIkfomState();
  auto estimate = makeEstimate(state);
  Eigen::Matrix<double, ManifoldState::kErrorStateDimension,
                ManifoldState::kErrorStateDimension>
      factor = Eigen::Matrix<double, ManifoldState::kErrorStateDimension,
                             ManifoldState::kErrorStateDimension>::Identity();
  std::mt19937 generator(42U);
  std::normal_distribution<double> distribution(0.0, 0.03);
  for (int row = 0; row < ManifoldState::kErrorStateDimension; ++row) {
    for (int column = 0; column <= row; ++column) {
      factor(row, column) += distribution(generator);
    }
  }
  estimate.covariance = factor * factor.transpose();
  const auto converter = makeConverter(
      Eigen::Quaterniond(Eigen::AngleAxisd(0.5, Eigen::Vector3d::UnitY())),
      Eigen::Vector3d(0.4, -0.2, 0.6));
  const BaseLinkCovarianceProjector projector(converter.baseToImu());
  const Eigen::Vector3d raw_gyro(0.3, -0.4, 0.9);
  const auto base = converter.convert(
      estimate, raw_gyro - estimate.state.gyro_bias_rad_s());
  ASSERT_TRUE(base.ok());
  const KinematicStateEstimate kinematic = makeKinematic(estimate, raw_gyro);
  const auto result = projector.project(kinematic, base.value());
  ASSERT_TRUE(result.ok()) << result.status().message();
  const auto pose_jacobian = projector.poseStateJacobian(kinematic, base.value());
  const auto twist_jacobian = projector.twistStateJacobian(kinematic, base.value());
  EXPECT_TRUE(result.value().pose_covariance_odom.isApprox(
      pose_jacobian * estimate.covariance * pose_jacobian.transpose(), 1e-10));
  EXPECT_TRUE(result.value().twist_covariance_base.isApprox(
      twist_jacobian * estimate.covariance * twist_jacobian.transpose(), 1e-10));
  EXPECT_TRUE((pose_jacobian.block<6, 6>(0, ManifoldState::kExtrinsicRotationOffset)
                  .isZero()));
  EXPECT_TRUE((pose_jacobian.block<6, 6>(0, ManifoldState::kExtrinsicPositionOffset)
                  .isZero()));
}

TEST(BaseLinkCovarianceProjectorTest, InvalidSourceAndZeroOutputFailClosed) {
  const auto state = makeIkfomState();
  const auto converter = makeConverter(Eigen::Quaterniond::Identity(),
                                       Eigen::Vector3d(0.4, -0.2, 0.6));
  const BaseLinkCovarianceProjector projector(converter.baseToImu());
  const Eigen::Vector3d raw_gyro(0.3, -0.4, 0.9);
  auto estimate = makeEstimate(state);
  const auto base = converter.convert(
      estimate, raw_gyro - estimate.state.gyro_bias_rad_s());
  ASSERT_TRUE(base.ok());
  BaseLinkCovarianceProjectionDiagnostics diagnostics;

  estimate.covariance.setZero();
  auto result = projector.project(makeKinematic(estimate, raw_gyro), base.value(),
                                  &diagnostics);
  EXPECT_FALSE(result.ok());
  EXPECT_TRUE(diagnostics.source_zero);

  estimate.covariance.setIdentity();
  estimate.covariance(0, 1) = 0.1;
  result = projector.project(makeKinematic(estimate, raw_gyro), base.value(),
                             &diagnostics);
  EXPECT_FALSE(result.ok());
  EXPECT_TRUE(diagnostics.source_asymmetry);

  estimate.covariance.setIdentity();
  estimate.covariance(0, 0) = -1.0;
  result = projector.project(makeKinematic(estimate, raw_gyro), base.value(),
                             &diagnostics);
  EXPECT_FALSE(result.ok());
  EXPECT_TRUE(diagnostics.source_non_psd);

  estimate.covariance.setZero();
  estimate.covariance.block<3, 3>(ManifoldState::kExtrinsicRotationOffset,
                                 ManifoldState::kExtrinsicRotationOffset) =
      Eigen::Matrix3d::Identity();
  result = projector.project(makeKinematic(estimate, raw_gyro), base.value(),
                             &diagnostics);
  EXPECT_FALSE(result.ok());
  EXPECT_TRUE(diagnostics.output_pose_non_psd);
  EXPECT_TRUE(diagnostics.output_twist_non_psd);
}

TEST(BaseLinkCovarianceProjectorTest, TinyNegativeEigenvalueIsRoundoffRepairable) {
  const auto state = makeIkfomState();
  auto estimate = makeEstimate(state);
  estimate.covariance.setIdentity();
  estimate.covariance(0, 0) = -5e-11;
  const auto converter = makeConverter(Eigen::Quaterniond::Identity(),
                                       Eigen::Vector3d::Zero());
  const BaseLinkCovarianceProjector projector(converter.baseToImu());
  const Eigen::Vector3d raw_gyro(0.3, -0.4, 0.9);
  const auto base = converter.convert(
      estimate, raw_gyro - estimate.state.gyro_bias_rad_s());
  ASSERT_TRUE(base.ok());
  BaseLinkCovarianceProjectionDiagnostics diagnostics;
  const auto result = projector.project(makeKinematic(estimate, raw_gyro),
                                         base.value(), &diagnostics);
  EXPECT_TRUE(result.ok()) << result.status().message();
  EXPECT_TRUE(diagnostics.roundoff_repair);
}

}  // namespace
}  // namespace uav::nav::lio
