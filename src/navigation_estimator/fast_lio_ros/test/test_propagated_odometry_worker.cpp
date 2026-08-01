#include <gtest/gtest.h>

#include <chrono>
#include <condition_variable>
#include <functional>
#include <mutex>
#include <optional>
#include <thread>
#include <vector>

#include "fast_lio_ros/propagated_odometry_worker.hpp"

namespace uav::nav::lio {
namespace {

using namespace std::chrono_literals;

ImuSample sample(const std::int64_t time_ns) {
  return ImuSample{Timestamp(time_ns), {0.01, 0.02, -0.01},
                   {0.1, 0.0, 9.80665}};
}

StateEstimate correction(const std::int64_t time_ns) {
  StateEstimate result;
  result.time = Timestamp(time_ns);
  result.covariance = 0.001 * ManifoldState::Covariance::Identity();
  return result;
}

EstimatorStateUpdate trackingCorrection(const std::int64_t time_ns,
                                         const std::uint64_t sequence) {
  return {EstimatorStatus::kTracking, true, correction(time_ns), sequence};
}

bool waitForDiagnostics(
    PropagatedOdometryWorker& worker,
    const std::function<bool(const PropagatedOdometryWorkerDiagnostics&)>&
        predicate,
    const std::chrono::milliseconds timeout = 2s) {
  const auto deadline = std::chrono::steady_clock::now() + timeout;
  while (std::chrono::steady_clock::now() < deadline) {
    if (predicate(worker.diagnostics())) {
      return true;
    }
    std::this_thread::sleep_for(1ms);
  }
  return predicate(worker.diagnostics());
}

struct OutputCollector {
  void push(const std::optional<StateEstimate>& estimate) {
    std::lock_guard lock(mutex);
    outputs.push_back(estimate);
    ready.notify_all();
  }

  bool waitFor(const std::size_t count, const std::chrono::milliseconds timeout = 2s) {
    std::unique_lock lock(mutex);
    return ready.wait_for(lock, timeout, [&] { return outputs.size() >= count; });
  }

  std::mutex mutex;
  std::condition_variable ready;
  std::vector<std::optional<StateEstimate>> outputs;
};

TEST(PropagatedOdometryWorkerTest, PendingCorrectionMailboxIsClearedAfterTake) {
  PropagatedOdometryWorker worker(PropagatedOdometryWorkerConfig{});
  worker.start();
  ASSERT_TRUE(worker.enqueueImu(sample(0)));
  ASSERT_TRUE(worker.enqueueImu(sample(10'000'000)));
  ASSERT_TRUE(worker.enqueueEstimatorState(trackingCorrection(0, 1U)));
  ASSERT_TRUE(waitForDiagnostics(worker, [](const auto& diagnostics) {
    return diagnostics.last_applied_correction_sequence == 1U;
  }));
  EXPECT_EQ(worker.diagnostics().last_applied_correction_sequence, 1U);
  worker.stop();
}

TEST(PropagatedOdometryWorkerTest, IdleWorkerDoesNotBusyLoopAfterCorrection) {
  PropagatedOdometryWorker worker(PropagatedOdometryWorkerConfig{});
  worker.start();
  ASSERT_TRUE(worker.enqueueEstimatorState(trackingCorrection(100'000'000, 1U)));
  ASSERT_TRUE(waitForDiagnostics(worker, [](const auto& diagnostics) {
    return diagnostics.correction_waiting_for_bracket_count == 1U;
  }));
  const auto wakeups = worker.diagnostics().worker_wakeup_count;
  std::this_thread::sleep_for(20ms);
  EXPECT_EQ(worker.diagnostics().worker_wakeup_count, wakeups);
  worker.stop();
}

TEST(PropagatedOdometryWorkerTest,
     PublishesOnlyFromFollowingImuEventAndNotCorrection) {
  OutputCollector collector;
  PropagatedOdometryWorker worker(
      PropagatedOdometryWorkerConfig{},
      [&](const auto& output) { collector.push(output); });
  worker.start();
  ASSERT_TRUE(worker.enqueueEstimatorState(trackingCorrection(0, 1U)));
  ASSERT_TRUE(waitForDiagnostics(worker, [](const auto& diagnostics) {
    return diagnostics.correction_waiting_for_bracket_count == 1U;
  }));
  {
    std::lock_guard lock(collector.mutex);
    EXPECT_TRUE(collector.outputs.empty());
  }
  ASSERT_TRUE(worker.enqueueImu(sample(0)));
  ASSERT_TRUE(worker.enqueueImu(sample(10'000'000)));
  ASSERT_TRUE(collector.waitFor(1U));
  worker.stop();
  std::lock_guard lock(collector.mutex);
  ASSERT_EQ(collector.outputs.size(), 1U);
  ASSERT_TRUE(collector.outputs.front().has_value());
  EXPECT_EQ(collector.outputs.front()->time.nanoseconds(), 10'000'000);
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
  ASSERT_TRUE(worker.enqueueEstimatorState(trackingCorrection(0, 1U)));
  ASSERT_TRUE(waitForDiagnostics(worker, [](const auto& diagnostics) {
    return diagnostics.last_applied_correction_sequence == 1U;
  }));
  ASSERT_TRUE(worker.enqueueImu(sample(20'000'000)));
  ASSERT_TRUE(collector.waitFor(1U));
  ASSERT_TRUE(worker.enqueueImu(sample(30'000'000)));
  ASSERT_TRUE(waitForDiagnostics(worker, [](const auto& diagnostics) {
    return diagnostics.propagator.status == PropagatedOdometryStatus::kStaleCorrection;
  }));
  worker.stop();
  std::lock_guard lock(collector.mutex);
  ASSERT_EQ(collector.outputs.size(), 1U);
  EXPECT_EQ(worker.diagnostics().stale_stop_count, 1U);
}

TEST(PropagatedOdometryWorkerTest, CorrectionDoesNotCrossMainInvalidationGeneration) {
  OutputCollector collector;
  PropagatedOdometryWorker worker(
      PropagatedOdometryWorkerConfig{},
      [&](const auto& output) { collector.push(output); });
  worker.start();
  ASSERT_TRUE(worker.enqueueImu(sample(0)));
  ASSERT_TRUE(worker.enqueueImu(sample(10'000'000)));
  ASSERT_TRUE(waitForDiagnostics(worker, [](const auto& diagnostics) {
    return diagnostics.imu_batch_count >= 1U;
  }));
  ASSERT_TRUE(worker.enqueueEstimatorState(trackingCorrection(100'000'000, 1U)));
  ASSERT_TRUE(waitForDiagnostics(worker, [](const auto& diagnostics) {
    return diagnostics.correction_waiting_for_bracket_count == 1U;
  }));
  ASSERT_TRUE(worker.enqueueEstimatorState(
      {EstimatorStatus::kDegraded, false, std::nullopt, 2U}));
  ASSERT_TRUE(waitForDiagnostics(worker, [](const auto& diagnostics) {
    return diagnostics.control_generation == 1U &&
           diagnostics.main_status == EstimatorStatus::kDegraded;
  }));
  ASSERT_TRUE(worker.enqueueImu(sample(20'000'000)));
  ASSERT_TRUE(waitForDiagnostics(worker, [](const auto& diagnostics) {
    return diagnostics.propagator.latest_imu_time.has_value() &&
           diagnostics.propagator.latest_imu_time->nanoseconds() == 20'000'000;
  }));
  EXPECT_EQ(worker.diagnostics().last_applied_correction_sequence, 0U);
  EXPECT_EQ(worker.diagnostics().stale_generation_correction_drop_count, 1U);
  {
    std::lock_guard lock(collector.mutex);
    EXPECT_TRUE(collector.outputs.empty());
  }
  worker.stop();
}

TEST(PropagatedOdometryWorkerTest, CorrectionDoesNotCancelLoadShedding) {
  PropagatedOdometryWorker worker(PropagatedOdometryWorkerConfig{});
  worker.start();
  ASSERT_TRUE(worker.enqueueEstimatorState(trackingCorrection(100'000'000, 1U)));
  ASSERT_TRUE(waitForDiagnostics(worker, [](const auto& diagnostics) {
    return diagnostics.correction_waiting_for_bracket_count == 1U;
  }));
  worker.requestLoadShedding();
  ASSERT_TRUE(waitForDiagnostics(worker, [](const auto& diagnostics) {
    return diagnostics.load_shedding_transition_count == 1U &&
           diagnostics.control_generation == 1U;
  }));
  const auto diagnostics = worker.diagnostics();
  EXPECT_EQ(diagnostics.propagator.status, PropagatedOdometryStatus::kQueueOverflow);
  EXPECT_EQ(diagnostics.last_applied_correction_sequence, 0U);
  EXPECT_EQ(diagnostics.stale_generation_correction_drop_count, 1U);
  worker.stop();
}

TEST(PropagatedOdometryWorkerTest, OldGenerationCorrectionIsDiscarded) {
  PropagatedOdometryWorker worker(PropagatedOdometryWorkerConfig{});
  worker.start();
  ASSERT_TRUE(worker.enqueueEstimatorState(trackingCorrection(100'000'000, 1U)));
  ASSERT_TRUE(waitForDiagnostics(worker, [](const auto& diagnostics) {
    return diagnostics.correction_waiting_for_bracket_count == 1U;
  }));
  ASSERT_TRUE(worker.enqueueEstimatorState(
      {EstimatorStatus::kLost, false, std::nullopt, 2U}));
  ASSERT_TRUE(waitForDiagnostics(worker, [](const auto& diagnostics) {
    return diagnostics.control_generation == 1U &&
           diagnostics.stale_generation_correction_drop_count == 1U;
  }));
  EXPECT_EQ(worker.diagnostics().last_applied_correction_sequence, 0U);
  worker.stop();
}

TEST(PropagatedOdometryWorkerTest, CorrectionAheadOfHistoryWaitsForBracket) {
  OutputCollector collector;
  PropagatedOdometryWorker worker(
      PropagatedOdometryWorkerConfig{},
      [&](const auto& output) { collector.push(output); });
  worker.start();
  ASSERT_TRUE(worker.enqueueEstimatorState(trackingCorrection(30'000'000, 1U)));
  ASSERT_TRUE(waitForDiagnostics(worker, [](const auto& diagnostics) {
    return diagnostics.correction_waiting_for_bracket_count == 1U;
  }));
  ASSERT_TRUE(worker.enqueueImu(sample(0)));
  ASSERT_TRUE(worker.enqueueImu(sample(10'000'000)));
  ASSERT_TRUE(worker.enqueueImu(sample(20'000'000)));
  ASSERT_TRUE(waitForDiagnostics(worker, [](const auto& diagnostics) {
    return diagnostics.propagator.latest_imu_time.has_value() &&
           diagnostics.propagator.latest_imu_time->nanoseconds() == 20'000'000;
  }));
  EXPECT_EQ(worker.diagnostics().last_applied_correction_sequence, 0U);
  ASSERT_TRUE(worker.enqueueImu(sample(30'000'000)));
  ASSERT_TRUE(waitForDiagnostics(worker, [](const auto& diagnostics) {
    return diagnostics.last_applied_correction_sequence == 1U;
  }));
  {
    std::lock_guard lock(collector.mutex);
    EXPECT_TRUE(collector.outputs.empty());
  }
  ASSERT_TRUE(worker.enqueueImu(sample(40'000'000)));
  ASSERT_TRUE(collector.waitFor(1U));
  worker.stop();
}

TEST(PropagatedOdometryWorkerTest, SuspendedWorkerRecordsReplayHistory) {
  PropagatedOdometryWorker worker(PropagatedOdometryWorkerConfig{});
  worker.start();
  worker.requestLoadShedding();
  ASSERT_TRUE(waitForDiagnostics(worker, [](const auto& diagnostics) {
    return diagnostics.load_shedding_transition_count == 1U;
  }));
  ASSERT_TRUE(worker.enqueueImu(sample(0)));
  ASSERT_TRUE(worker.enqueueImu(sample(10'000'000)));
  ASSERT_TRUE(worker.enqueueImu(sample(20'000'000)));
  ASSERT_TRUE(waitForDiagnostics(worker, [](const auto& diagnostics) {
    return diagnostics.propagator.latest_imu_time.has_value() &&
           diagnostics.propagator.latest_imu_time->nanoseconds() == 20'000'000;
  }));
  auto diagnostics = worker.diagnostics();
  EXPECT_EQ(diagnostics.propagator.current_imu_history_size, 3U);
  EXPECT_FALSE(diagnostics.propagator.propagated_time.has_value());
  EXPECT_EQ(diagnostics.suspended_imu_drop_count, 0U);
  ASSERT_TRUE(worker.enqueueEstimatorState(trackingCorrection(10'000'000, 1U)));
  ASSERT_TRUE(waitForDiagnostics(worker, [](const auto& current) {
    return current.last_applied_correction_sequence == 1U;
  }));
  diagnostics = worker.diagnostics();
  ASSERT_TRUE(diagnostics.propagator.propagated_time.has_value());
  EXPECT_EQ(diagnostics.propagator.propagated_time->nanoseconds(), 20'000'000);
  worker.stop();
}

TEST(PropagatedOdometryWorkerTest, RecoveryRequiresCurrentGenerationCorrection) {
  OutputCollector collector;
  PropagatedOdometryWorker worker(
      PropagatedOdometryWorkerConfig{},
      [&](const auto& output) { collector.push(output); });
  worker.start();
  worker.requestLoadShedding();
  ASSERT_TRUE(waitForDiagnostics(worker, [](const auto& diagnostics) {
    return diagnostics.control_generation == 1U;
  }));
  ASSERT_TRUE(worker.enqueueImu(sample(0)));
  ASSERT_TRUE(worker.enqueueImu(sample(10'000'000)));
  ASSERT_TRUE(waitForDiagnostics(worker, [](const auto& diagnostics) {
    return diagnostics.propagator.latest_imu_time.has_value();
  }));
  EXPECT_EQ(worker.diagnostics().publication_count, 0U);
  ASSERT_TRUE(worker.enqueueEstimatorState(trackingCorrection(0, 1U)));
  ASSERT_TRUE(waitForDiagnostics(worker, [](const auto& diagnostics) {
    return diagnostics.last_applied_correction_sequence == 1U;
  }));
  EXPECT_EQ(worker.diagnostics().publication_count, 0U);
  ASSERT_TRUE(worker.enqueueImu(sample(20'000'000)));
  ASSERT_TRUE(collector.waitFor(1U));
  worker.stop();
}

TEST(PropagatedOdometryWorkerTest, WorkerProcessesOnePublicationPerDrainedBatch) {
  OutputCollector collector;
  PropagatedOdometryWorker worker(
      PropagatedOdometryWorkerConfig{},
      [&](const auto& output) { collector.push(output); });
  worker.start();
  ASSERT_TRUE(worker.enqueueEstimatorState(trackingCorrection(0, 1U)));
  ASSERT_TRUE(waitForDiagnostics(worker, [](const auto& diagnostics) {
    return diagnostics.correction_waiting_for_bracket_count == 1U;
  }));
  for (std::int64_t time_ns = 0; time_ns <= 100'000'000;
       time_ns += 10'000'000) {
    ASSERT_TRUE(worker.enqueueImu(sample(time_ns)));
  }
  ASSERT_TRUE(collector.waitFor(1U));
  ASSERT_TRUE(waitForDiagnostics(worker, [](const auto& diagnostics) {
    return diagnostics.total_imu_samples_drained == 11U;
  }));
  const auto diagnostics = worker.diagnostics();
  {
    std::lock_guard lock(collector.mutex);
    EXPECT_EQ(collector.outputs.size(), diagnostics.publication_count);
  }
  EXPECT_LE(diagnostics.publication_count, diagnostics.imu_batch_count);
  worker.stop();
}

TEST(PropagatedOdometryWorkerTest, WorkerDrainsMultipleImuSamplesPerWakeup) {
  PropagatedOdometryWorkerConfig config;
  config.event_queue_capacity = 10'000U;
  PropagatedOdometryWorker worker(config);
  worker.start();
  for (std::int64_t index = 0; index < 1'000; ++index) {
    ASSERT_TRUE(worker.enqueueImu(sample(index * 10'000'000)));
  }
  ASSERT_TRUE(waitForDiagnostics(worker, [](const auto& diagnostics) {
    return diagnostics.total_imu_samples_drained == 1'000U;
  }));
  EXPECT_GT(worker.diagnostics().maximum_imu_batch_size, 1U);
  worker.stop();
}

TEST(PropagatedOdometryWorkerTest, RepeatedLoadSheddingRequestIsIdempotent) {
  PropagatedOdometryWorker worker(PropagatedOdometryWorkerConfig{});
  worker.start();
  worker.requestLoadShedding();
  worker.requestLoadShedding();
  worker.requestLoadShedding();
  ASSERT_TRUE(waitForDiagnostics(worker, [](const auto& diagnostics) {
    return diagnostics.load_shedding_transition_count == 1U;
  }));
  std::this_thread::sleep_for(10ms);
  const auto diagnostics = worker.diagnostics();
  EXPECT_EQ(diagnostics.load_shedding_transition_count, 1U);
  EXPECT_EQ(diagnostics.control_generation, 1U);
  worker.stop();
}

TEST(PropagatedOdometryWorkerTest, StopReturnsWithoutDrainingBacklog) {
  PropagatedOdometryWorkerConfig config;
  config.event_queue_capacity = 10'000U;
  PropagatedOdometryWorker worker(config);
  worker.start();
  for (std::int64_t index = 0; index < 1'000; ++index) {
    ASSERT_TRUE(worker.enqueueImu(sample(index * 10'000'000)));
  }
  worker.stop();
  EXPECT_FALSE(worker.enqueueImu(sample(20'000'000'000)));
}

TEST(PropagatedOdometryWorkerTest, StopPreventsFurtherPublication) {
  OutputCollector collector;
  PropagatedOdometryWorker worker(
      PropagatedOdometryWorkerConfig{},
      [&](const auto& output) { collector.push(output); });
  worker.start();
  ASSERT_TRUE(worker.enqueueImu(sample(0)));
  ASSERT_TRUE(worker.enqueueEstimatorState(trackingCorrection(0, 1U)));
  worker.stop();
  std::size_t count = 0U;
  {
    std::lock_guard lock(collector.mutex);
    count = collector.outputs.size();
  }
  EXPECT_FALSE(worker.enqueueEstimatorState(trackingCorrection(10'000'000, 2U)));
  {
    std::lock_guard lock(collector.mutex);
    EXPECT_EQ(collector.outputs.size(), count);
  }
}

TEST(PropagatedOdometryWorkerTest, StopWithPendingCorrectionDoesNotDeadlock) {
  PropagatedOdometryWorker worker(PropagatedOdometryWorkerConfig{});
  worker.start();
  ASSERT_TRUE(worker.enqueueEstimatorState(trackingCorrection(100'000'000, 1U)));
  ASSERT_TRUE(waitForDiagnostics(worker, [](const auto& diagnostics) {
    return diagnostics.correction_waiting_for_bracket_count == 1U;
  }));
  worker.stop();
  SUCCEED();
}

TEST(PropagatedOdometryWorkerTest, StopWithFullImuIngressDoesNotDeadlock) {
  PropagatedOdometryWorkerConfig config;
  config.event_queue_capacity = 1U;
  PropagatedOdometryWorker worker(config);
  worker.start();
  bool overflowed = false;
  for (std::int64_t index = 0; index < 100 && !overflowed; ++index) {
    overflowed = !worker.enqueueImu(sample(index * 10'000'000));
  }
  EXPECT_TRUE(overflowed);
  worker.stop();
  SUCCEED();
}

TEST(PropagatedOdometryWorkerTest, FailedReplayDoesNotBecomeAppliedCorrection) {
  PropagatedOdometryWorkerConfig config;
  config.propagator.imu_history_duration_ns = 30'000'000;
  PropagatedOdometryWorker worker(config);
  worker.start();
  for (std::int64_t time_ns = 0; time_ns <= 100'000'000;
       time_ns += 10'000'000) {
    ASSERT_TRUE(worker.enqueueImu(sample(time_ns)));
  }
  ASSERT_TRUE(waitForDiagnostics(worker, [](const auto& diagnostics) {
    return diagnostics.propagator.latest_imu_time.has_value() &&
           diagnostics.propagator.latest_imu_time->nanoseconds() ==
               100'000'000;
  }));
  ASSERT_TRUE(worker.enqueueEstimatorState(trackingCorrection(20'000'000, 7U)));
  ASSERT_TRUE(waitForDiagnostics(worker, [](const auto& diagnostics) {
    return diagnostics.last_received_correction_sequence == 7U &&
           diagnostics.correction_waiting_for_bracket_count == 1U;
  }));
  EXPECT_FALSE(worker.diagnostics().last_applied_correction_time.has_value());
  EXPECT_EQ(worker.diagnostics().last_applied_correction_sequence, 0U);
  worker.stop();
}

}  // namespace
}  // namespace uav::nav::lio
