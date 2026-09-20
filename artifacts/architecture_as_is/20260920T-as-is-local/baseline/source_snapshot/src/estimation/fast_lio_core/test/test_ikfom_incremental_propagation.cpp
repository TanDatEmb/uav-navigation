#include <gtest/gtest.h>

#include <Eigen/Eigenvalues>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <iostream>
#include <numeric>
#include <vector>

#include "fast_lio_core/estimation/ikfom_estimator.hpp"

namespace uav::nav::lio {
namespace {

constexpr std::int64_t kSamplePeriodNs = 2'500'000;

IkfomEstimatorConfig estimatorConfig() {
  IkfomEstimatorConfig config;
  config.maximum_integration_step_ns = 20'000'000;
  return config;
}

std::vector<ImuSample> mixedMotion(std::size_t count,
                                   std::int64_t period_ns = kSamplePeriodNs) {
  std::vector<ImuSample> samples;
  samples.reserve(count);
  for (std::size_t index = 0; index < count; ++index) {
    const auto sample_index = static_cast<std::int64_t>(index + 1U);
    const double time_s = static_cast<double>(sample_index * period_ns) * 1e-9;
    samples.push_back(ImuSample{
        Timestamp(sample_index * period_ns),
        {0.08 + 0.02 * std::sin(0.7 * time_s),
         -0.04 + 0.01 * std::cos(0.4 * time_s), 0.12},
        {0.3 * std::sin(0.3 * time_s), 0.2 * std::cos(0.5 * time_s),
         9.80665 + 0.1 * std::sin(0.2 * time_s)}});
  }
  return samples;
}

struct Difference {
  double position{0.0};
  double orientation{0.0};
  double velocity{0.0};
  double gyro_bias{0.0};
  double accel_bias{0.0};
  double gravity{0.0};
  double covariance{0.0};
};

Difference difference(const IkfomEstimator& lhs, const IkfomEstimator& rhs) {
  const auto a = lhs.stateView();
  const auto b = rhs.stateView();
  return {(a.position_odom_imu_m() - b.position_odom_imu_m()).norm(),
          a.orientation_odom_imu().angularDistance(
              b.orientation_odom_imu()),
          (a.velocity_odom_imu_m_s() - b.velocity_odom_imu_m_s()).norm(),
          (a.gyro_bias_rad_s() - b.gyro_bias_rad_s()).norm(),
          (a.accel_bias_m_s2() - b.accel_bias_m_s2()).norm(),
          (a.gravity_odom_m_s2() - b.gravity_odom_m_s2()).norm(),
          (lhs.covariance() - rhs.covariance()).norm()};
}

void expectEquivalent(const IkfomEstimator& lhs, const IkfomEstimator& rhs) {
  const Difference error = difference(lhs, rhs);
  EXPECT_LE(error.position, 2e-12);
  EXPECT_LE(error.orientation, 2e-12);
  EXPECT_LE(error.velocity, 2e-12);
  EXPECT_LE(error.gyro_bias, 2e-12);
  EXPECT_LE(error.accel_bias, 2e-12);
  EXPECT_LE(error.gravity, 2e-12);
  EXPECT_LE(error.covariance, 2e-11);
}

void predictIncrementally(IkfomEstimator& estimator,
                          const std::vector<ImuSample>& samples,
                          std::size_t first = 0U) {
  for (std::size_t index = std::max<std::size_t>(first + 1U, 1U);
       index < samples.size(); ++index) {
    const std::span<const ImuSample> pair(samples.data() + index - 1U, 2U);
    const auto result = estimator.predict(pair, samples[index - 1U].time,
                                          samples[index].time);
    ASSERT_TRUE(result.ok()) << result.status().message();
  }
}

TEST(IkfomIncrementalPropagationGate, BatchAndIncrementalAreEquivalent) {
  const auto samples = mixedMotion(401U);
  IkfomEstimator batch(estimatorConfig(), ResidualBuilderConfig{});
  IkfomEstimator incremental(estimatorConfig(), ResidualBuilderConfig{});
  batch.initialize(ManifoldState{});
  incremental.initialize(ManifoldState{});

  const auto batch_result =
      batch.predict(samples, samples.front().time, samples.back().time);
  ASSERT_TRUE(batch_result.ok()) << batch_result.status().message();
  predictIncrementally(incremental, samples);

  const Difference error = difference(batch, incremental);
  std::cout << "batch_incremental position=" << error.position
            << " orientation=" << error.orientation
            << " velocity=" << error.velocity
            << " gyro_bias=" << error.gyro_bias
            << " accel_bias=" << error.accel_bias
            << " gravity=" << error.gravity
            << " covariance=" << error.covariance << '\n';
  expectEquivalent(batch, incremental);
}

TEST(IkfomIncrementalPropagationGate, ReanchorBatchAndReplayAreEquivalent) {
  const auto samples = mixedMotion(401U);
  constexpr std::size_t correction_index = 160U;
  ManifoldState corrected;
  corrected.set_position_odom_imu_m({1.0, -0.5, 0.2});
  corrected.set_orientation_odom_imu(Eigen::Quaterniond(
      Eigen::AngleAxisd(0.15, Eigen::Vector3d(0.2, 0.3, 0.9).normalized())));
  ManifoldState::Covariance covariance =
      0.002 * ManifoldState::Covariance::Identity();
  covariance.block<3, 3>(6, 6) = 1e-12 * Eigen::Matrix3d::Identity();
  covariance.block<3, 3>(9, 9) = 1e-12 * Eigen::Matrix3d::Identity();

  IkfomEstimator batch(estimatorConfig(), ResidualBuilderConfig{});
  IkfomEstimator replay(estimatorConfig(), ResidualBuilderConfig{});
  batch.rebase(corrected, covariance);
  replay.rebase(corrected, covariance);
  const std::span<const ImuSample> replay_samples(
      samples.data() + correction_index,
      samples.size() - correction_index);
  const auto result = batch.predict(replay_samples,
                                    samples[correction_index].time,
                                    samples.back().time);
  ASSERT_TRUE(result.ok()) << result.status().message();
  predictIncrementally(replay, samples, correction_index);
  expectEquivalent(batch, replay);
}

TEST(IkfomIncrementalPropagationGate, InterpolatesCorrectionBoundary) {
  const auto samples = mixedMotion(202U, 5'000'000);
  const Timestamp boundary(307'500'000);
  ASSERT_LT(samples[60].time.nanoseconds(), boundary.nanoseconds());
  ASSERT_GT(samples[61].time.nanoseconds(), boundary.nanoseconds());
  ManifoldState corrected;
  corrected.set_position_odom_imu_m({0.4, 0.1, -0.2});
  const auto covariance = 0.001 * ManifoldState::Covariance::Identity();
  IkfomEstimator batch(estimatorConfig(), ResidualBuilderConfig{});
  IkfomEstimator replay(estimatorConfig(), ResidualBuilderConfig{});
  batch.rebase(corrected, covariance);
  replay.rebase(corrected, covariance);

  const std::span<const ImuSample> bracketed(samples.data() + 60U,
                                             samples.size() - 60U);
  const auto full = batch.predict(bracketed, boundary, samples.back().time);
  ASSERT_TRUE(full.ok()) << full.status().message();
  const std::span<const ImuSample> first_pair(samples.data() + 60U, 2U);
  auto step = replay.predict(first_pair, boundary, samples[61].time);
  ASSERT_TRUE(step.ok()) << step.status().message();
  predictIncrementally(replay, samples, 61U);
  expectEquivalent(batch, replay);
}

TEST(IkfomIncrementalPropagationGate, FailedPredictionRollsBackExactly) {
  const auto valid = mixedMotion(6U, 10'000'000);
  IkfomEstimator estimator(estimatorConfig(), ResidualBuilderConfig{});
  estimator.initialize(ManifoldState{});
  const auto warmup = estimator.predict(valid, valid.front().time,
                                        valid.back().time);
  ASSERT_TRUE(warmup.ok());
  const ManifoldState state_before = estimator.stateView();
  const auto covariance_before = estimator.covariance();

  auto invalid = mixedMotion(4U, 10'000'000);
  for (auto& sample : invalid) {
    sample.time = Timestamp(sample.time.nanoseconds() + 50'000'000);
  }
  invalid[3].time = Timestamp(75'000'000);
  const auto failed = estimator.predict(invalid, invalid.front().time,
                                        invalid.back().time);
  ASSERT_FALSE(failed.ok());
  EXPECT_EQ(failed.status().code(), StatusCode::kTimestampRegression);
  const auto state_after = estimator.stateView();
  EXPECT_EQ((state_after.position_odom_imu_m() -
             state_before.position_odom_imu_m()).norm(), 0.0);
  EXPECT_EQ(state_after.orientation_odom_imu().angularDistance(
                state_before.orientation_odom_imu()), 0.0);
  EXPECT_EQ((estimator.covariance() - covariance_before).norm(), 0.0);
}

TEST(IkfomIncrementalPropagationGate, LongRunStateAndCovarianceRemainValid) {
  const auto samples = mixedMotion(8'001U);
  IkfomEstimator estimator(estimatorConfig(), ResidualBuilderConfig{});
  estimator.initialize(ManifoldState{});
  predictIncrementally(estimator, samples);
  const auto state = estimator.stateView();
  const auto covariance = estimator.covariance();
  ASSERT_TRUE(state.allFinite());
  EXPECT_NEAR(state.orientation_odom_imu().norm(), 1.0, 1e-12);
  ASSERT_TRUE(covariance.allFinite());
  EXPECT_LE((covariance - covariance.transpose()).cwiseAbs().maxCoeff(),
            1e-8);
  Eigen::SelfAdjointEigenSolver<ManifoldState::Covariance> solver(
      0.5 * (covariance + covariance.transpose()), Eigen::EigenvaluesOnly);
  ASSERT_EQ(solver.info(), Eigen::Success);
  EXPECT_GE(solver.eigenvalues().minCoeff(), -1e-10);
}

TEST(IkfomIncrementalPropagationGate, MeasuresIncrementalPredictionRuntime) {
  for (const std::int64_t period_ns : {5'000'000LL, 2'500'000LL}) {
    const auto samples = mixedMotion(2'001U, period_ns);
    IkfomEstimator estimator(estimatorConfig(), ResidualBuilderConfig{});
    estimator.initialize(ManifoldState{});
    std::vector<double> runtime_us;
    runtime_us.reserve(samples.size() - 1U);
    const auto wall_start = std::chrono::steady_clock::now();
    for (std::size_t index = 1; index < samples.size(); ++index) {
      const auto start = std::chrono::steady_clock::now();
      const std::span<const ImuSample> pair(samples.data() + index - 1U, 2U);
      const auto result = estimator.predict(pair, samples[index - 1U].time,
                                            samples[index].time);
      ASSERT_TRUE(result.ok()) << result.status().message();
      runtime_us.push_back(std::chrono::duration<double, std::micro>(
                               std::chrono::steady_clock::now() - start)
                               .count());
    }
    const double total_s = std::chrono::duration<double>(
                               std::chrono::steady_clock::now() - wall_start)
                               .count();
    std::sort(runtime_us.begin(), runtime_us.end());
    const auto percentile = [&](double fraction) {
      return runtime_us[static_cast<std::size_t>(
          fraction * static_cast<double>(runtime_us.size() - 1U))];
    };
    const double mean =
        std::accumulate(runtime_us.begin(), runtime_us.end(), 0.0) /
        static_cast<double>(runtime_us.size());
    const double input_duration_s =
        static_cast<double>(period_ns * (samples.size() - 1U)) * 1e-9;
    std::cout << "incremental_benchmark rate_hz=" << 1e9 / period_ns
              << " mean_us=" << mean << " p95_us=" << percentile(0.95)
              << " p99_us=" << percentile(0.99)
              << " max_us=" << runtime_us.back()
              << " cpu_realtime_ratio=" << total_s / input_duration_s
              << '\n';
  }
}

}  // namespace
}  // namespace uav::nav::lio
