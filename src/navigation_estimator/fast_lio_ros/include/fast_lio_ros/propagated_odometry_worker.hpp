#pragma once

#include <atomic>
#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <functional>
#include <mutex>
#include <optional>
#include <thread>
#include <chrono>

#include "fast_lio_core/pipeline/estimator_diagnostics.hpp"
#include "fast_lio_core/propagation/imu_state_propagator.hpp"

namespace uav::nav::lio {

struct PropagatedOdometryWorkerConfig {
  ImuStatePropagatorConfig propagator{};
  std::size_t event_queue_capacity{4096U};
  std::int64_t maximum_correction_age_ns{300'000'000};
  double publish_rate_hz{50.0};
};

struct EstimatorStateUpdate {
  EstimatorStatus status{EstimatorStatus::kWaitingForSensors};
  bool navigation_valid{false};
  std::optional<StateEstimate> corrected_estimate;
  std::uint64_t correction_sequence{0U};
};

struct PropagatedOdometryWorkerDiagnostics {
  ImuStatePropagatorDiagnostics propagator;
  bool navigation_valid{false};
  EstimatorStatus main_status{EstimatorStatus::kWaitingForSensors};
  std::optional<Timestamp> last_correction_time;
  std::optional<Timestamp> last_applied_correction_time;
  std::uint64_t last_received_correction_sequence{0U};
  std::uint64_t last_applied_correction_sequence{0U};
  std::uint64_t correction_sequence{0U};
  std::uint64_t queue_overflow_count{0U};
  std::uint64_t stale_stop_count{0U};
  std::int64_t last_replay_runtime_us{0};
  std::int64_t maximum_replay_runtime_us{0};
  std::size_t current_event_queue_depth{0U};
  std::size_t maximum_event_queue_depth{0U};
  std::uint64_t worker_wakeup_count{0U};
  std::uint64_t imu_batch_count{0U};
  std::uint64_t total_imu_samples_drained{0U};
  std::size_t maximum_imu_batch_size{0U};
  std::uint64_t control_generation{0U};
  std::uint64_t correction_waiting_for_bracket_count{0U};
  std::uint64_t load_shedding_transition_count{0U};
  std::uint64_t suspended_imu_drop_count{0U};
  std::optional<Timestamp> last_published_time;
  std::optional<Timestamp> next_publish_deadline;
  std::uint64_t publication_count{0U};
  std::uint64_t publication_skip_count{0U};
  std::uint64_t load_shedding_count{0U};
  std::uint64_t correction_coalesced_count{0U};
};

class PropagatedOdometryWorker {
 public:
  using ImuProcessedCallback =
      std::function<void(const std::optional<StateEstimate>&)>;

  explicit PropagatedOdometryWorker(
      PropagatedOdometryWorkerConfig config,
      ImuProcessedCallback imu_processed_callback = {});
  ~PropagatedOdometryWorker();

  PropagatedOdometryWorker(const PropagatedOdometryWorker&) = delete;
  PropagatedOdometryWorker& operator=(const PropagatedOdometryWorker&) = delete;

  void start();
  void stop();
  [[nodiscard]] bool enqueueImu(const ImuSample& sample);
  [[nodiscard]] bool enqueueEstimatorState(EstimatorStateUpdate update);
  void requestLoadShedding() noexcept;
  [[nodiscard]] PropagatedOdometryWorkerDiagnostics diagnostics() const;

 private:
  void run();
  void process(ImuSample event);
  void process(EstimatorStateUpdate update);
  void updateSnapshot();
  void maybePublishOnImu(const ImuSample& sample);

  PropagatedOdometryWorkerConfig config_;
  ImuStatePropagator propagator_;
  ImuProcessedCallback imu_processed_callback_;
  mutable std::mutex mutex_;
  std::condition_variable ready_;
  std::deque<ImuSample> imu_ingress_;
  std::optional<EstimatorStateUpdate> pending_correction_;
  PropagatedOdometryWorkerDiagnostics diagnostics_;
  bool started_{false};
  bool accepting_{false};
  std::atomic<bool> stop_requested_{false};
  std::atomic<bool> load_shed_requested_{false};
  std::atomic<bool> invalidation_requested_{false};
  bool suspended_{false};
  std::uint64_t control_generation_{0U};
  std::thread thread_;
  std::int64_t publish_period_ns_{20'000'000};
  std::optional<Timestamp> last_published_time_;
  std::optional<Timestamp> next_publish_deadline_;
  std::uint64_t publication_count_{0U};
  std::uint64_t publication_skip_count_{0U};
};

}  // namespace uav::nav::lio
