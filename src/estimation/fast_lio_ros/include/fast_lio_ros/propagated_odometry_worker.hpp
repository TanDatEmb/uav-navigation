#pragma once

#include <atomic>
#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <functional>
#include <mutex>
#include <optional>
#include <span>
#include <string>
#include <thread>
#include <chrono>

#include "fast_lio_core/pipeline/estimator_diagnostics.hpp"
#include "fast_lio_core/propagation/imu_state_propagator.hpp"

namespace uav::nav::lio {

struct PropagatedOdometryWorkerConfig {
  ImuStatePropagatorConfig propagator{};
  std::size_t imu_ingress_capacity{4096U};
  // The default leaves 250 ms of one-second IMU history for re-anchoring.
  std::int64_t maximum_correction_age_ns{250'000'000};
  double publish_rate_hz{50.0};
};

struct EstimatorStateUpdate {
  EstimatorStatus status{EstimatorStatus::kWaitingForSensors};
  bool navigation_valid{false};
  std::optional<StateEstimate> corrected_estimate;
  std::uint64_t correction_sequence{0U};
};

struct PendingCorrection {
  StateEstimate estimate;
  EstimatorStatus status{EstimatorStatus::kTracking};
  bool navigation_valid{true};
  std::uint64_t correction_sequence{0U};
  std::uint64_t control_generation{0U};
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
  bool replay_in_progress{false};
  std::size_t current_imu_ingress_depth{0U};
  std::size_t maximum_imu_ingress_depth{0U};
  std::uint64_t worker_wakeup_count{0U};
  std::uint64_t imu_batch_count{0U};
  std::uint64_t total_imu_samples_drained{0U};
  std::size_t maximum_imu_batch_size{0U};
  std::uint64_t control_generation{0U};
  std::uint64_t correction_waiting_for_bracket_count{0U};
  std::uint64_t correction_missing_end_wait_count{0U};
  std::uint64_t correction_missing_start_drop_count{0U};
  std::uint64_t old_sequence_correction_drop_count{0U};
  std::uint64_t old_timestamp_correction_drop_count{0U};
  std::uint64_t duplicate_correction_drop_count{0U};
  std::uint64_t correction_superseded_during_replay_count{0U};
  std::uint64_t load_shedding_transition_count{0U};
  std::uint64_t suspended_imu_drop_count{0U};
  std::optional<Timestamp> last_published_time;
  std::optional<Timestamp> next_publish_deadline;
  std::uint64_t publication_count{0U};
  std::uint64_t publication_skip_count{0U};
  std::uint64_t load_shedding_count{0U};
  std::uint64_t correction_coalesced_count{0U};
  std::uint64_t stale_generation_correction_drop_count{0U};
  bool worker_failed{false};
  std::string worker_failure_message;
};

class PropagatedOdometryWorker {
 public:
  using ImuProcessedCallback =
      std::function<void(const std::optional<KinematicStateEstimate>&)>;

  explicit PropagatedOdometryWorker(
      PropagatedOdometryWorkerConfig config,
      ImuProcessedCallback imu_processed_callback = {});
  ~PropagatedOdometryWorker();

  PropagatedOdometryWorker(const PropagatedOdometryWorker&) = delete;
  PropagatedOdometryWorker& operator=(const PropagatedOdometryWorker&) = delete;

  // A worker is single-use: stop() is terminal and start() may not be called
  // again because the ingress, propagator history, and runtime diagnostics
  // intentionally retain their lifetime state.
  void start();
  void stop();
  [[nodiscard]] bool enqueueImu(const ImuSample& sample);
  [[nodiscard]] bool enqueueEstimatorState(EstimatorStateUpdate update);
  void requestLoadShedding() noexcept;
  [[nodiscard]] PropagatedOdometryWorkerDiagnostics diagnostics() const;

 private:
  void run();
  void processImuBatch(std::span<const ImuSample> batch);
  void handleContinuityReset();
  [[nodiscard]] std::optional<Timestamp> processPendingCorrection();
  [[nodiscard]] std::optional<PendingCorrection>
  takePendingCorrectionLocked();
  void requeuePendingCorrectionLocked(PendingCorrection correction);
  void discardPendingCorrectionLocked(bool stale_generation);
  [[nodiscard]] bool rejectCorrectionNotNewerThanAppliedLocked(
      std::uint64_t correction_sequence, const Timestamp& correction_time);
  void updateSnapshot();
  void maybePublishOnImu(const ImuSample& sample);

  PropagatedOdometryWorkerConfig config_;
  ImuStatePropagator propagator_;
  ImuProcessedCallback imu_processed_callback_;
  mutable std::mutex mutex_;
  std::condition_variable ready_;
  std::deque<ImuSample> imu_ingress_;
  std::optional<PendingCorrection> pending_correction_;
  PropagatedOdometryWorkerDiagnostics diagnostics_;
  bool started_{false};
  bool ever_started_{false};
  bool accepting_{false};
  std::atomic<bool> stop_requested_{false};
  std::atomic<bool> load_shed_requested_{false};
  std::atomic<bool> invalidation_requested_{false};
  std::atomic<bool> correction_requested_{false};
  std::atomic<bool> suspended_{false};
  std::uint64_t control_generation_{0U};
  std::optional<std::uint64_t> waiting_correction_sequence_;
  std::thread thread_;
  std::int64_t publish_period_ns_{20'000'000};
  std::optional<Timestamp> last_published_time_;
  std::optional<Timestamp> next_publish_deadline_;
  std::uint64_t publication_count_{0U};
  std::uint64_t publication_skip_count_{0U};
};

}  // namespace uav::nav::lio
