#include <gtest/gtest.h>

#include <chrono>
#include <condition_variable>
#include <mutex>
#include <optional>
#include <vector>

#include "fast_lio_ros/propagated_odometry_worker.hpp"

namespace uav::nav::lio {
namespace {

using namespace std::chrono_literals;

ImuSample sample(std::int64_t time_ns) {
  return ImuSample{Timestamp(time_ns), {0.01, 0.02, -0.01},
                   {0.1, 0.0, 9.80665}};
}

StateEstimate correction(std::int64_t time_ns) {
  StateEstimate result;
  result.time = Timestamp(time_ns);
  result.covariance = 0.001 * ManifoldState::Covariance::Identity();
  return result;
}

struct OutputCollector {
  void push(const std::optional<StateEstimate>& estimate) {
    std::lock_guard lock(mutex);
    outputs.push_back(estimate);
    ready.notify_all();
  }
  bool waitFor(std::size_t count) {
    std::unique_lock lock(mutex);
    return ready.wait_for(lock, 2s, [&] { return outputs.size() >= count; });
  }
  std::mutex mutex;
  std::condition_variable ready;
  std::vector<std::optional<StateEstimate>> outputs;
};

TEST(PropagatedOdometryWorkerTest, PublishesOnlyFromFollowingImuEvent) {
  OutputCollector collector;
  PropagatedOdometryWorker worker(
      PropagatedOdometryWorkerConfig{},
      [&](const auto& output) { collector.push(output); });
  worker.start();
  ASSERT_TRUE(worker.enqueueImu(sample(0)));
  ASSERT_TRUE(worker.enqueueImu(sample(10'000'000)));
  ASSERT_TRUE(worker.enqueueEstimatorState(
      {EstimatorStatus::kTracking, true, correction(0), 1U}));
  ASSERT_TRUE(collector.waitFor(2U));
  {
    std::lock_guard lock(collector.mutex);
    ASSERT_EQ(collector.outputs.size(), 2U);
    EXPECT_FALSE(collector.outputs[0].has_value());
    EXPECT_FALSE(collector.outputs[1].has_value());
  }
  ASSERT_TRUE(worker.enqueueImu(sample(20'000'000)));
  ASSERT_TRUE(collector.waitFor(3U));
  worker.stop();
  std::lock_guard lock(collector.mutex);
  ASSERT_TRUE(collector.outputs.back().has_value());
  EXPECT_EQ(collector.outputs.back()->time.nanoseconds(), 20'000'000);
}

TEST(PropagatedOdometryWorkerTest, StaleCorrectionStopsOnImuEvent) {
  OutputCollector collector;
  PropagatedOdometryWorkerConfig config;
  config.maximum_correction_age_ns = 25'000'000;
  PropagatedOdometryWorker worker(
      config, [&](const auto& output) { collector.push(output); });
  worker.start();
  ASSERT_TRUE(worker.enqueueImu(sample(0)));
  ASSERT_TRUE(worker.enqueueImu(sample(10'000'000)));
  ASSERT_TRUE(worker.enqueueEstimatorState(
      {EstimatorStatus::kTracking, true, correction(0), 1U}));
  ASSERT_TRUE(worker.enqueueImu(sample(20'000'000)));
  ASSERT_TRUE(worker.enqueueImu(sample(30'000'000)));
  ASSERT_TRUE(collector.waitFor(4U));
  worker.stop();
  std::lock_guard lock(collector.mutex);
  ASSERT_EQ(collector.outputs.size(), 4U);
  EXPECT_TRUE(collector.outputs[2].has_value());
  EXPECT_FALSE(collector.outputs[3].has_value());
  EXPECT_EQ(worker.diagnostics().stale_stop_count, 1U);
}

TEST(PropagatedOdometryWorkerTest, MainInvalidationStopsImmediately) {
  OutputCollector collector;
  PropagatedOdometryWorker worker(
      PropagatedOdometryWorkerConfig{},
      [&](const auto& output) { collector.push(output); });
  worker.start();
  ASSERT_TRUE(worker.enqueueImu(sample(0)));
  ASSERT_TRUE(worker.enqueueImu(sample(10'000'000)));
  ASSERT_TRUE(worker.enqueueEstimatorState(
      {EstimatorStatus::kTracking, true, correction(0), 1U}));
  ASSERT_TRUE(worker.enqueueImu(sample(20'000'000)));
  ASSERT_TRUE(worker.enqueueEstimatorState(
      {EstimatorStatus::kDegraded, false, std::nullopt, 1U}));
  ASSERT_TRUE(worker.enqueueImu(sample(30'000'000)));
  ASSERT_TRUE(collector.waitFor(4U));
  worker.stop();
  std::lock_guard lock(collector.mutex);
  EXPECT_TRUE(collector.outputs[2].has_value());
  EXPECT_FALSE(collector.outputs[3].has_value());
}

TEST(PropagatedOdometryWorkerTest, StopDrainsEventsInEnqueueOrder) {
  OutputCollector collector;
  PropagatedOdometryWorker worker(
      PropagatedOdometryWorkerConfig{},
      [&](const auto& output) { collector.push(output); });
  worker.start();
  for (std::int64_t index = 0; index < 20; ++index) {
    ASSERT_TRUE(worker.enqueueImu(sample(index * 10'000'000)));
  }
  worker.stop();
  std::lock_guard lock(collector.mutex);
  EXPECT_EQ(collector.outputs.size(), 20U);
}

}  // namespace
}  // namespace uav::nav::lio
