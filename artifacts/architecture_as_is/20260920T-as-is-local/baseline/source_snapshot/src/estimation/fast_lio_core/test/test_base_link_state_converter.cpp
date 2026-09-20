#include <gtest/gtest.h>

#include <array>
#include <cmath>
#include <limits>
#include <numbers>

#include <Eigen/Geometry>

#include "fast_lio_core/geometry/frame_ids.hpp"
#include "fast_lio_core/navigation/base_link_state_converter.hpp"

namespace uav::nav::lio {
namespace {

constexpr double kTolerance = 1e-12;

template <typename T>
concept HasCovarianceField = requires(T value) { value.covariance; };

template <typename T>
concept HasPoseOnlyConversion = requires(const T& converter,
                                         const StateEstimate& estimate) {
  converter.convert(estimate);
};

static_assert(!HasCovarianceField<RigidBodyState>);
static_assert(!HasPoseOnlyConversion<BaseLinkStateConverter>);

StateEstimate makeEstimate(const Eigen::Quaterniond& orientation_odom_imu,
                           const Eigen::Vector3d& position_odom_imu,
                           const Eigen::Vector3d& velocity_odom_imu,
                           Timestamp time = Timestamp(123456789,
                                                       ClockDomain::kSensorTime)) {
  StateEstimate estimate;
  estimate.time = time;
  estimate.state.set_orientation_odom_imu(orientation_odom_imu);
  estimate.state.set_position_odom_imu_m(position_odom_imu);
  estimate.state.set_velocity_odom_imu_m_s(velocity_odom_imu);
  return estimate;
}

BaseLinkStateConverter makeConverter(const Eigen::Quaterniond& rotation_base_imu,
                                     const Eigen::Vector3d& translation_base_imu) {
  return BaseLinkStateConverter(
      RigidTransform(baseFrame(), imuFrame(), rotation_base_imu, translation_base_imu));
}

void expectVectorNear(const Eigen::Vector3d& actual, const Eigen::Vector3d& expected) {
  EXPECT_TRUE(actual.isApprox(expected, kTolerance))
      << "actual=" << actual.transpose() << " expected=" << expected.transpose();
}

TEST(BaseLinkStateConverterTest, IdentityTransformPreservesPoseAndCorrectsExpressions) {
  const BaseLinkStateConverter converter =
      makeConverter(Eigen::Quaterniond::Identity(), Eigen::Vector3d::Zero());
  const Eigen::Quaterniond orientation_odom_imu(
      Eigen::AngleAxisd(0.7, Eigen::Vector3d(1.0, -2.0, 0.5).normalized()));
  const Eigen::Vector3d position_odom_imu(1.2, -0.8, 2.4);
  const Eigen::Vector3d velocity_odom_imu(0.4, -1.1, 2.0);
  const Eigen::Vector3d omega_imu(0.3, -0.4, 0.8);
  const StateEstimate estimate =
      makeEstimate(orientation_odom_imu, position_odom_imu, velocity_odom_imu);

  const auto result = converter.convert(estimate, omega_imu);
  ASSERT_TRUE(result.ok()) << result.status().message();
  const RigidBodyState& output = result.value();

  EXPECT_EQ(output.time, estimate.time);
  EXPECT_EQ(output.source_frame, imuFrame());
  EXPECT_EQ(output.reference_frame, lioOdomFrame());
  EXPECT_EQ(output.body_frame, baseFrame());
  expectVectorNear(output.position_reference_body_m, position_odom_imu);
  EXPECT_TRUE(output.orientation_reference_body.isApprox(orientation_odom_imu,
                                                         kTolerance));
  expectVectorNear(output.linear_velocity_reference_body_m_s, velocity_odom_imu);
  expectVectorNear(output.linear_velocity_body_m_s,
                   orientation_odom_imu.conjugate() * velocity_odom_imu);
  ASSERT_TRUE(output.angular_velocity_body_rad_s.has_value());
  expectVectorNear(*output.angular_velocity_body_rad_s, omega_imu);
}

TEST(BaseLinkStateConverterTest, HoverTranslationAppliesFullThreeDimensionalLeverArm) {
  const Eigen::Vector3d r_base_imu(0.4, -0.5, 0.6);
  const BaseLinkStateConverter converter =
      makeConverter(Eigen::Quaterniond::Identity(), r_base_imu);
  const Eigen::Vector3d position_odom_imu(2.0, -3.0, 4.0);
  const Eigen::Vector3d velocity_odom_imu(1.0, 2.0, 3.0);

  const auto result = converter.convert(
      makeEstimate(Eigen::Quaterniond::Identity(), position_odom_imu,
                   velocity_odom_imu),
      Eigen::Vector3d::Zero());
  ASSERT_TRUE(result.ok()) << result.status().message();

  expectVectorNear(result.value().position_reference_body_m,
                   position_odom_imu - r_base_imu);
  expectVectorNear(result.value().linear_velocity_reference_body_m_s,
                   velocity_odom_imu);
  expectVectorNear(result.value().linear_velocity_body_m_s, velocity_odom_imu);
}

TEST(BaseLinkStateConverterTest, PureYawStationaryBaseCancelsLeverArmVelocity) {
  const Eigen::Vector3d r_base_imu(1.2, -0.7, 0.4);
  const Eigen::Vector3d omega_base(0.0, 0.0, 2.5);
  const Eigen::Vector3d position_base(0.3, -0.8, 1.0);
  const BaseLinkStateConverter converter =
      makeConverter(Eigen::Quaterniond::Identity(), r_base_imu);
  const Eigen::Vector3d position_imu = position_base + r_base_imu;
  const Eigen::Vector3d velocity_imu = omega_base.cross(r_base_imu);

  const auto result = converter.convert(
      makeEstimate(Eigen::Quaterniond::Identity(), position_imu, velocity_imu),
      omega_base);
  ASSERT_TRUE(result.ok()) << result.status().message();

  expectVectorNear(result.value().position_reference_body_m, position_base);
  expectVectorNear(result.value().linear_velocity_reference_body_m_s,
                   Eigen::Vector3d::Zero());
  expectVectorNear(result.value().linear_velocity_body_m_s,
                   Eigen::Vector3d::Zero());
}

TEST(BaseLinkStateConverterTest, PureYawWithKnownBaseTranslationRecoversVelocity) {
  const Eigen::Quaterniond orientation_odom_base(
      Eigen::AngleAxisd(0.6, Eigen::Vector3d::UnitZ()));
  const Eigen::Vector3d position_odom_base(1.1, -2.2, 0.7);
  const Eigen::Vector3d r_base_imu(0.8, -0.3, 0.5);
  const Eigen::Vector3d omega_base(0.0, 0.0, 1.7);
  const Eigen::Vector3d velocity_odom_base(0.9, -0.4, 0.2);
  const BaseLinkStateConverter converter =
      makeConverter(Eigen::Quaterniond::Identity(), r_base_imu);
  const RigidTransform T_odom_base(lioOdomFrame(), baseFrame(), orientation_odom_base,
                                   position_odom_base);
  const auto T_odom_imu_result = T_odom_base.compose(converter.baseToImu());
  ASSERT_TRUE(T_odom_imu_result.ok());
  const RigidTransform& T_odom_imu = T_odom_imu_result.value();
  const Eigen::Vector3d velocity_odom_imu =
      velocity_odom_base +
      orientation_odom_base * omega_base.cross(r_base_imu);

  const auto result = converter.convert(
      makeEstimate(T_odom_imu.rotation(), T_odom_imu.translation(),
                   velocity_odom_imu),
      omega_base);
  ASSERT_TRUE(result.ok()) << result.status().message();

  expectVectorNear(result.value().position_reference_body_m, position_odom_base);
  expectVectorNear(result.value().linear_velocity_reference_body_m_s,
                   velocity_odom_base);
  expectVectorNear(result.value().linear_velocity_body_m_s,
                   orientation_odom_base.conjugate() * velocity_odom_base);
}

TEST(BaseLinkStateConverterTest, RollPitchYawAndAsymmetricLeverArmUseRigidComposition) {
  const Eigen::Quaterniond orientation_odom_base =
      Eigen::AngleAxisd(0.4, Eigen::Vector3d::UnitZ()) *
      Eigen::AngleAxisd(-0.3, Eigen::Vector3d::UnitY()) *
      Eigen::AngleAxisd(0.2, Eigen::Vector3d::UnitX());
  const Eigen::Vector3d position_odom_base(2.3, -1.7, 0.9);
  const Eigen::Vector3d r_base_imu(0.37, -0.82, 1.13);
  const Eigen::Vector3d omega_base(0.6, -1.1, 0.8);
  const Eigen::Vector3d velocity_odom_base(-0.7, 1.4, 0.25);
  const BaseLinkStateConverter converter =
      makeConverter(Eigen::Quaterniond::Identity(), r_base_imu);
  const RigidTransform T_odom_base(lioOdomFrame(), baseFrame(), orientation_odom_base,
                                   position_odom_base);
  const auto T_odom_imu_result = T_odom_base.compose(converter.baseToImu());
  ASSERT_TRUE(T_odom_imu_result.ok());
  const RigidTransform& T_odom_imu = T_odom_imu_result.value();
  const Eigen::Vector3d velocity_odom_imu =
      velocity_odom_base +
      orientation_odom_base * omega_base.cross(r_base_imu);

  const auto result = converter.convert(
      makeEstimate(T_odom_imu.rotation(), T_odom_imu.translation(),
                   velocity_odom_imu),
      omega_base);
  ASSERT_TRUE(result.ok()) << result.status().message();

  expectVectorNear(result.value().position_reference_body_m, position_odom_base);
  expectVectorNear(result.value().linear_velocity_reference_body_m_s,
                   velocity_odom_base);
  expectVectorNear(result.value().linear_velocity_body_m_s,
                   orientation_odom_base.conjugate() * velocity_odom_base);
}

TEST(BaseLinkStateConverterTest, MountingRotationRotatesAngularVelocityIntoBase) {
  const Eigen::Quaterniond rotation_base_imu(
      Eigen::AngleAxisd(-0.5, Eigen::Vector3d::UnitY()));
  const Eigen::Quaterniond orientation_odom_base(
      Eigen::AngleAxisd(0.8, Eigen::Vector3d(1.0, 2.0, -1.0).normalized()));
  const Eigen::Vector3d r_base_imu(-0.23, 0.41, 0.77);
  const Eigen::Vector3d omega_imu(0.9, -0.6, 1.2);
  const Eigen::Vector3d position_odom_base(-1.0, 2.4, -0.3);
  const Eigen::Vector3d velocity_odom_base(0.2, 0.8, -1.3);
  const BaseLinkStateConverter converter =
      makeConverter(rotation_base_imu, r_base_imu);
  const RigidTransform T_odom_base(lioOdomFrame(), baseFrame(), orientation_odom_base,
                                   position_odom_base);
  const auto T_odom_imu_result = T_odom_base.compose(converter.baseToImu());
  ASSERT_TRUE(T_odom_imu_result.ok());
  const RigidTransform& T_odom_imu = T_odom_imu_result.value();
  const Eigen::Vector3d omega_base = rotation_base_imu * omega_imu;
  const Eigen::Vector3d velocity_odom_imu =
      velocity_odom_base +
      orientation_odom_base * omega_base.cross(r_base_imu);

  const auto result = converter.convert(
      makeEstimate(T_odom_imu.rotation(), T_odom_imu.translation(),
                   velocity_odom_imu),
      omega_imu);
  ASSERT_TRUE(result.ok()) << result.status().message();

  EXPECT_TRUE(result.value().orientation_reference_body.isApprox(
      orientation_odom_base, kTolerance));
  expectVectorNear(result.value().linear_velocity_reference_body_m_s,
                   velocity_odom_base);
  ASSERT_TRUE(result.value().angular_velocity_body_rad_s.has_value());
  expectVectorNear(*result.value().angular_velocity_body_rad_s, omega_base);
}

TEST(BaseLinkStateConverterTest, CachesStaticTransformAndPreservesTimestampAndInputs) {
  Eigen::Quaterniond rotation_base_imu(
      Eigen::AngleAxisd(0.3, Eigen::Vector3d(1.0, -2.0, 0.4).normalized()));
  rotation_base_imu.coeffs() *= 1.0001;
  const Eigen::Vector3d translation_base_imu(0.31, -0.52, 0.87);
  const BaseLinkStateConverter converter =
      makeConverter(rotation_base_imu, translation_base_imu);
  const StateEstimate estimate = makeEstimate(
      Eigen::Quaterniond(Eigen::AngleAxisd(-0.25, Eigen::Vector3d::UnitX())),
      Eigen::Vector3d(1.0, 2.0, 3.0), Eigen::Vector3d(-0.4, 0.2, 1.1),
      Timestamp(9876543210123LL, ClockDomain::kSensorTime));
  const StateEstimate estimate_before = estimate;
  const Eigen::Vector3d omega(0.2, -0.3, 0.5);
  const Eigen::Vector3d cached_translation = converter.baseToImu().translation();
  const Eigen::Quaterniond cached_rotation = converter.baseToImu().rotation();

  const auto result = converter.convert(estimate, omega);
  ASSERT_TRUE(result.ok()) << result.status().message();
  EXPECT_EQ(result.value().time, estimate_before.time);
  EXPECT_TRUE(estimate.state.position_odom_imu_m().isApprox(
      estimate_before.state.position_odom_imu_m(), kTolerance));
  EXPECT_TRUE(estimate.state.orientation_odom_imu().isApprox(
      estimate_before.state.orientation_odom_imu(), kTolerance));
  EXPECT_TRUE(estimate.state.velocity_odom_imu_m_s().isApprox(
      estimate_before.state.velocity_odom_imu_m_s(), kTolerance));
  EXPECT_TRUE(estimate.covariance.isApprox(estimate_before.covariance, kTolerance));
  EXPECT_TRUE(converter.baseToImu().translation().isApprox(cached_translation, kTolerance));
  EXPECT_TRUE(converter.baseToImu().rotation().isApprox(cached_rotation, kTolerance));
  EXPECT_NEAR(converter.baseToImu().rotation().norm(), 1.0, kTolerance);
  EXPECT_TRUE(converter.imuToBase().targetFrame() == imuFrame());
  EXPECT_TRUE(converter.imuToBase().sourceFrame() == baseFrame());
}

TEST(BaseLinkStateConverterTest, RejectsWrongStaticTransformDirections) {
  const std::array<RigidTransform, 3> wrong_transforms{
      RigidTransform(imuFrame(), baseFrame(), Eigen::Quaterniond::Identity(),
                     Eigen::Vector3d::Zero()),
      RigidTransform(baseFrame(), lidarFrame(), Eigen::Quaterniond::Identity(),
                     Eigen::Vector3d::Zero()),
      RigidTransform(lioOdomFrame(), imuFrame(), Eigen::Quaterniond::Identity(),
                     Eigen::Vector3d::Zero()),
  };
  for (const RigidTransform& transform : wrong_transforms) {
    const auto result = BaseLinkStateConverter::Create(transform);
    EXPECT_FALSE(result.ok());
    EXPECT_EQ(result.status().code(), StatusCode::kInvalidArgument);
  }
}

TEST(BaseLinkStateConverterTest, RejectsNonFiniteStaticAndDynamicInputs) {
  const double nan = std::numeric_limits<double>::quiet_NaN();
  const double infinity = std::numeric_limits<double>::infinity();
  const auto nonfinite_translation = RigidTransform::Create(
      baseFrame(), imuFrame(), Eigen::Quaterniond::Identity(),
      Eigen::Vector3d(nan, 0.0, 0.0));
  EXPECT_FALSE(nonfinite_translation.ok());

  Eigen::Quaterniond zero_quaternion(0.0, 0.0, 0.0, 0.0);
  const auto degenerate_rotation = RigidTransform::Create(
      baseFrame(), imuFrame(), zero_quaternion, Eigen::Vector3d::Zero());
  EXPECT_FALSE(degenerate_rotation.ok());

  const BaseLinkStateConverter converter =
      makeConverter(Eigen::Quaterniond::Identity(), Eigen::Vector3d(0.1, 0.2, 0.3));
  StateEstimate invalid_position = makeEstimate(
      Eigen::Quaterniond::Identity(), Eigen::Vector3d(nan, 0.0, 0.0),
      Eigen::Vector3d::Zero());
  EXPECT_FALSE(converter.convert(invalid_position, Eigen::Vector3d::Zero()).ok());

  StateEstimate invalid_velocity = makeEstimate(
      Eigen::Quaterniond::Identity(), Eigen::Vector3d::Zero(),
      Eigen::Vector3d(infinity, 0.0, 0.0));
  EXPECT_FALSE(converter.convert(invalid_velocity, Eigen::Vector3d::Zero()).ok());
  EXPECT_FALSE(converter.convert(
                   makeEstimate(Eigen::Quaterniond::Identity(),
                                Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero()),
                   Eigen::Vector3d(nan, 0.0, 0.0))
                   .ok());
}

}  // namespace
}  // namespace uav::nav::lio
