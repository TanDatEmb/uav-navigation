#include <gtest/gtest.h>

#include <Eigen/Geometry>
#include <vector>

#include "fast_lio_core/estimation/ikfom_estimator.hpp"
#include "fast_lio_core/mapping/ikd_tree_registration_map.hpp"

namespace uav::nav::lio {
namespace {

TEST(IkfomEstimatorTest, UsesUpstreamStateAndProcessJacobians) {
  static_assert(IkfomState::DOF == 23);
  static_assert(IkfomState::DIM == 24);

  IkfomState state;
  state.grav =
      IkfomGravity{IkfomVector3{Eigen::Vector3d(0.0, 0.0, -9.80665)}};
  IkfomInput input;
  input.acc = IkfomVector3{Eigen::Vector3d(0.0, 0.0, 9.80665)};
  input.gyro = IkfomVector3{Eigen::Vector3d(0.1, -0.2, 0.3)};

  const auto derivative = ikfomProcessModel(state, input);
  const auto state_jacobian = ikfomProcessJacobianState(state, input);
  const auto noise_jacobian = ikfomProcessJacobianNoise(state, input);

  EXPECT_TRUE(derivative.allFinite());
  EXPECT_TRUE(state_jacobian.allFinite());
  EXPECT_TRUE(noise_jacobian.allFinite());
  // FAST-LIO's S2 type fixes gravity magnitude at 9.809 m/s^2.
  EXPECT_LT(derivative.segment<3>(12).norm(), 3e-3);
  EXPECT_EQ(state_jacobian.rows(), 24);
  EXPECT_EQ(state_jacobian.cols(), 23);
}

TEST(IkfomEstimatorTest, PredictsThroughUpstreamFilterAndKeepsFixedExtrinsic) {
  IkfomEstimatorConfig config;
  config.maximum_integration_step_ns = 20'000'000;
  config.minimum_accepted_residuals = 3;
  ResidualBuilderConfig residual_config;
  IkfomEstimator estimator(config, residual_config);

  ManifoldState initial;
  initial.set_rotation_imu_lidar(
      Eigen::Quaterniond(Eigen::AngleAxisd(0.2, Eigen::Vector3d::UnitZ())));
  initial.set_position_imu_lidar_m(Eigen::Vector3d(0.1, -0.2, 0.3));
  estimator.initialize(initial);

  std::vector<ImuSample> samples(3);
  for (std::size_t index = 0; index < samples.size(); ++index) {
    samples[index].time = Timestamp(static_cast<std::int64_t>(index) * 10'000'000);
    samples[index].linear_acceleration_imu_m_s2 =
        Eigen::Vector3d(0.0, 0.0, 9.80665);
  }
  const auto trajectory =
      estimator.predict(samples, Timestamp(0), Timestamp(20'000'000));
  ASSERT_TRUE(trajectory.ok()) << trajectory.status().message();
  EXPECT_EQ(trajectory.value().size(), 3U);

  const ManifoldState predicted = estimator.stateView();
  EXPECT_LT(predicted.position_odom_imu_m().norm(), 1e-6);
  EXPECT_LT(predicted.rotation_imu_lidar().angularDistance(
                initial.rotation_imu_lidar()),
            1e-12);
  EXPECT_LT((predicted.position_imu_lidar_m() -
             initial.position_imu_lidar_m())
                .norm(),
            1e-12);
}

TEST(IkfomEstimatorTest, RejectedUpdateRestoresPredictionTransactionally) {
  IkfomEstimatorConfig config;
  config.minimum_accepted_residuals = 3;
  IkfomEstimator estimator(config, ResidualBuilderConfig{});
  estimator.initialize(ManifoldState{});
  const ManifoldState before = estimator.stateView();
  IkdTreeRegistrationMap empty_map;
  const std::vector<Eigen::Vector3d> points{{1.0, 0.0, 0.0}};

  const auto correction = estimator.correct(points, empty_map);

  EXPECT_FALSE(correction.successful);
  const ManifoldState after = estimator.stateView();
  EXPECT_LT((after.position_odom_imu_m() -
             before.position_odom_imu_m())
                .norm(),
            1e-12);
  EXPECT_LT(after.orientation_odom_imu().angularDistance(
                before.orientation_odom_imu()),
            1e-12);
}

}  // namespace
}  // namespace uav::nav::lio
