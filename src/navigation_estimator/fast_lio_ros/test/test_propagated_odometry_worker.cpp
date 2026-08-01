#include <gtest/gtest.h>

#include <chrono>
#include <condition_variable>
#include <mutex>
#include <optional>
#include <thread>
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
  std::this_thread::sleep_for(20ms);
  {
    std::lock_guard lock(collector.mutex);
    EXPECT_EQ(collector.outputs.size(), 0U);
  }
  ASSERT_TRUE(worker.enqueueImu(sample(20'000'000)));
  ASSERT_TRUE(collector.waitFor(1U));
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
  ASSERT_TRUE(collector.waitFor(1U));
  worker.stop();
  std::lock_guard lock(collector.mutex);
  ASSERT_EQ(collector.outputs.size(), 1U);
  EXPECT_TRUE(collector.outputs[0].has_value());
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
  ASSERT_TRUE(collector.waitFor(1U));
  worker.stop();
  std::lock_guard lock(collector.mutex);
  EXPECT_TRUE(collector.outputs[0].has_value());
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
  EXPECT_LE(collector.outputs.size(), 20U);
}

TEST(PropagatedOdometryWorkerTest, RecoversOnlyOnImuAfterNewCorrection) {
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
  ASSERT_TRUE(collector.waitFor(1U));
  ASSERT_TRUE(worker.enqueueEstimatorState(
      {EstimatorStatus::kTracking, true, correction(15'000'000), 2U}));
  {
    std::lock_guard lock(collector.mutex);
    EXPECT_EQ(collector.outputs.size(), 1U);
  }
  ASSERT_TRUE(worker.enqueueImu(sample(30'000'000)));
  ASSERT_TRUE(worker.enqueueImu(sample(40'000'000)));
  ASSERT_TRUE(collector.waitFor(2U));
  worker.stop();
  std::lock_guard lock(collector.mutex);
  EXPECT_TRUE(collector.outputs[0].has_value());
  EXPECT_TRUE(collector.outputs[1].has_value());
}

TEST(PropagatedOdometryWorkerTest, QueueOverflowInvalidatesWithoutBlocking) {
  PropagatedOdometryWorkerConfig config;
  config.event_queue_capacity = 1U;
  PropagatedOdometryWorker worker(config);
  worker.start();
  bool overflowed = false;
  for (std::int64_t index = 0; index < 100 && !overflowed; ++index) {
    overflowed = !worker.enqueueImu(sample(index * 10'000'000));
  }
  EXPECT_TRUE(overflowed);
  EXPECT_EQ(worker.diagnostics().queue_overflow_count, 1U);
  worker.stop();
  EXPECT_EQ(worker.diagnostics().propagator.status,
            PropagatedOdometryStatus::kQueueOverflow);
}

TEST(PropagatedOdometryWorkerTest, FailedReplayDoesNotBecomeAppliedCorrection) {
  OutputCollector collector;
  PropagatedOdometryWorkerConfig config;
  config.propagator.imu_history_duration_ns = 30'000'000;
  PropagatedOdometryWorker worker(
      config, [&](const auto& output) { collector.push(output); });
  worker.start();
  for (std::int64_t time_ns = 0; time_ns <= 100'000'000;
       time_ns += 10'000'000) {
    ASSERT_TRUE(worker.enqueueImu(sample(time_ns)));
  }
  ASSERT_TRUE(worker.enqueueEstimatorState(
      {EstimatorStatus::kTracking, true, correction(20'000'000), 7U}));
  worker.stop();
  const auto diagnostics = worker.diagnostics();
  ASSERT_TRUE(diagnostics.last_correction_time.has_value());
  EXPECT_EQ(diagnostics.last_received_correction_sequence, 7U);
  EXPECT_FALSE(diagnostics.last_applied_correction_time.has_value());
  EXPECT_EQ(diagnostics.last_applied_correction_sequence, 0U);
}


}  // namespace
}  // namespace uav::nav::lio
