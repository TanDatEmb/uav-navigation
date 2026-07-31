#include <gtest/gtest.h>

#include <Eigen/Geometry>
#include <Eigen/Eigenvalues>
#include <cmath>
#include <vector>

#include "fast_lio_core/estimation/ikfom_estimator.hpp"
#include "fast_lio_core/mapping/ikd_tree_registration_map.hpp"

namespace uav::nav::lio {
namespace {

void expectStateAndCovarianceEqual(
    const ManifoldState& expected_state,
    const ManifoldState::Covariance& expected_covariance,
    const IkfomEstimator& estimator) {
  const ManifoldState actual_state = estimator.stateView();
  EXPECT_LT((actual_state.position_odom_imu_m() -
             expected_state.position_odom_imu_m()).norm(), 1e-12);
  EXPECT_LT(actual_state.orientation_odom_imu().angularDistance(
                expected_state.orientation_odom_imu()), 1e-12);
  EXPECT_LT((actual_state.velocity_odom_imu_m_s() -
             expected_state.velocity_odom_imu_m_s()).norm(), 1e-12);
  EXPECT_LT((estimator.covariance() - expected_covariance).norm(), 1e-12);
}

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

TEST(IkfomEstimatorTest, ReportsConvergenceFromFinalUpstreamIncrement) {
  IkfomEstimatorConfig config;
  config.maximum_iterations = 5;
  config.minimum_accepted_residuals = 5;
  config.convergence_limit = 1e-4;
  ResidualBuilderConfig residual_config;
  residual_config.correspondence_search.neighbor_count = 5;
  residual_config.correspondence_search.maximum_neighbor_distance_m = 1.0;
  IkfomEstimator estimator(config, residual_config);
  ManifoldState initial;
  initial.set_position_odom_imu_m({0.0, 0.0, 0.05});
  estimator.initialize(initial);

  IkdTreeRegistrationMapConfig map_config;
  map_config.voxel_size_m = 0.02;
  IkdTreeRegistrationMap map(map_config);
  std::vector<Eigen::Vector3d> plane;
  for (int x = -4; x <= 4; ++x) {
    for (int y = -4; y <= 4; ++y) {
      plane.emplace_back(0.2 * x, 0.2 * y, 0.0);
    }
  }
  ASSERT_GT(map.insert(plane), 0U);
  const std::vector<Eigen::Vector3d> scan{
      {-0.5, -0.5, 0.0}, {0.0, -0.5, 0.0}, {0.5, -0.5, 0.0},
      {-0.5, 0.0, 0.0},  {0.0, 0.0, 0.0},  {0.5, 0.0, 0.0},
      {-0.5, 0.5, 0.0},  {0.0, 0.5, 0.0},  {0.5, 0.5, 0.0},
  };

  const auto correction = estimator.correct(scan, map);

  EXPECT_TRUE(correction.successful) << correction.reason;
  EXPECT_TRUE(correction.converged);
  EXPECT_GE(correction.iteration_count, 2U);
  EXPECT_LE(correction.iteration_count, config.maximum_iterations);
  EXPECT_TRUE(std::isfinite(correction.final_increment_norm));
  EXPECT_LT(correction.final_increment_norm, config.convergence_limit);
}

TEST(IkfomEstimatorTest, CovarianceRemainsSymmetricAndPsdAcrossHundredsOfPredictions) {
  IkfomEstimatorConfig config;
  config.maximum_integration_step_ns = 20'000'000;
  IkfomEstimator estimator(config, ResidualBuilderConfig{});
  estimator.initialize(ManifoldState{});
  for (std::int64_t step = 0; step < 300; ++step) {
    const std::int64_t start_ns = step * 10'000'000;
    const std::int64_t end_ns = start_ns + 10'000'000;
    std::vector<ImuSample> samples(2);
    samples[0].time = Timestamp(start_ns);
    samples[1].time = Timestamp(end_ns);
    for (auto& sample : samples) {
      sample.angular_velocity_imu_rad_s = {0.001, -0.002, 0.0015};
      sample.linear_acceleration_imu_m_s2 = {0.01, -0.02, 9.80665};
    }
    const auto prediction =
        estimator.predict(samples, Timestamp(start_ns), Timestamp(end_ns));
    ASSERT_TRUE(prediction.ok()) << prediction.status().message();
    const auto covariance = estimator.covariance();
    ASSERT_TRUE(covariance.allFinite());
    EXPECT_LT((covariance - covariance.transpose()).cwiseAbs().maxCoeff(),
              1e-8);
    const auto symmetric = 0.5 * (covariance + covariance.transpose());
    Eigen::SelfAdjointEigenSolver<ManifoldState::Covariance> solver(
        symmetric, Eigen::EigenvaluesOnly);
    ASSERT_EQ(solver.info(), Eigen::Success);
    EXPECT_GE(solver.eigenvalues().minCoeff(), -1e-10);
    EXPECT_TRUE(std::isfinite(covariance.trace()));
  }
}

TEST(IkfomEstimatorTest, OversizedLaterIntervalDoesNotPartiallyPredict) {
  IkfomEstimatorConfig config;
  config.maximum_integration_step_ns = 20'000'000;
  IkfomEstimator estimator(config, ResidualBuilderConfig{});
  estimator.initialize(ManifoldState{});
  const ManifoldState state_before = estimator.stateView();
  const auto covariance_before = estimator.covariance();
  std::vector<ImuSample> samples(3);
  samples[0].time = Timestamp(0);
  samples[1].time = Timestamp(10'000'000);
  samples[2].time = Timestamp(40'000'000);
  for (auto& sample : samples) {
    sample.linear_acceleration_imu_m_s2 = {0.1, 0.0, 9.80665};
  }

  const auto result =
      estimator.predict(samples, Timestamp(0), Timestamp(40'000'000));

  ASSERT_FALSE(result.ok());
  EXPECT_EQ(result.status().code(), StatusCode::kInsufficientData);
  expectStateAndCovarianceEqual(state_before, covariance_before, estimator);
}

TEST(IkfomEstimatorTest, NumericalFailureRollsBackStateAndCovariance) {
  IkfomEstimatorConfig config;
  config.maximum_integration_step_ns = 20'000'000;
  IkfomEstimator estimator(config, ResidualBuilderConfig{});
  estimator.initialize(ManifoldState{});
  const ManifoldState state_before = estimator.stateView();
  const auto covariance_before = estimator.covariance();
  std::vector<ImuSample> samples(3);
  samples[0].time = Timestamp(0);
  samples[1].time = Timestamp(10'000'000);
  samples[2].time = Timestamp(20'000'000);
  samples[0].linear_acceleration_imu_m_s2 = {0.0, 0.0, 9.80665};
  samples[1].linear_acceleration_imu_m_s2 = {0.0, 0.0, 9.80665};
  samples[2].linear_acceleration_imu_m_s2 = {
      std::numeric_limits<double>::max(), 0.0, 9.80665};

  const auto result =
      estimator.predict(samples, Timestamp(0), Timestamp(20'000'000));

  ASSERT_FALSE(result.ok());
  EXPECT_EQ(result.status().code(), StatusCode::kNumericalFailure);
  expectStateAndCovarianceEqual(state_before, covariance_before, estimator);

  samples.resize(2);
  samples[0].time = Timestamp(0);
  samples[1].time = Timestamp(10'000'000);
  samples[1].linear_acceleration_imu_m_s2 = {0.0, 0.0, 9.80665};
  const auto recovery =
      estimator.predict(samples, Timestamp(0), Timestamp(10'000'000));
  ASSERT_TRUE(recovery.ok()) << recovery.status().message();
}

}  // namespace
}  // namespace uav::nav::lio
