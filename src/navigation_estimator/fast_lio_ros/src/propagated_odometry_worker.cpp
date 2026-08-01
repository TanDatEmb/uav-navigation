#include "fast_lio_ros/propagated_odometry_worker.hpp"

#include <algorithm>
#include <chrono>
#include <pthread.h>
#include <stdexcept>
#include <utility>
#include <cmath>

namespace uav::nav::lio {

PropagatedOdometryWorker::PropagatedOdometryWorker(
    PropagatedOdometryWorkerConfig config,
    ImuProcessedCallback imu_processed_callback)
    : config_(std::move(config)),
      propagator_(config_.propagator),
      imu_processed_callback_(std::move(imu_processed_callback)) {
  if (config_.event_queue_capacity == 0U ||
      config_.maximum_correction_age_ns <= 0) {
    throw std::invalid_argument("Propagated odometry worker config is invalid");
  }
  if (!(config_.publish_rate_hz > 0.0) || !std::isfinite(config_.publish_rate_hz)) {
    throw std::invalid_argument("Propagated odometry publish rate is invalid");
  }
  publish_period_ns_ = static_cast<std::int64_t>(
      std::llround(1e9 / config_.publish_rate_hz));
}

PropagatedOdometryWorker::~PropagatedOdometryWorker() { stop(); }

void PropagatedOdometryWorker::start() {
  std::lock_guard lock(mutex_);
  if (started_) {
    return;
  }
  started_ = true;
  accepting_ = true;
  thread_ = std::thread([this] { run(); });
}

void PropagatedOdometryWorker::stop() {
  {
    std::lock_guard lock(mutex_);
    if (!started_) {
      return;
    }
    accepting_ = false;
    stop_requested_.store(true, std::memory_order_release);
  }
  ready_.notify_one();
  if (thread_.joinable()) {
    thread_.join();
  }
  std::lock_guard lock(mutex_);
  started_ = false;
}

bool PropagatedOdometryWorker::enqueueImu(const ImuSample& sample) {
  bool notify = false;
  {
    std::lock_guard lock(mutex_);
    if (!accepting_) return false;
    if (imu_ingress_.size() >= config_.event_queue_capacity) {
      ++diagnostics_.queue_overflow_count;
      load_shed_requested_.store(true, std::memory_order_release);
      ready_.notify_one();
      return false;
    }
    notify = imu_ingress_.empty() && !suspended_;
    imu_ingress_.push_back(sample);
    diagnostics_.current_event_queue_depth = imu_ingress_.size();
    diagnostics_.maximum_event_queue_depth =
        std::max(diagnostics_.maximum_event_queue_depth, imu_ingress_.size());
  }
  if (notify) ready_.notify_one();
  return true;
}

bool PropagatedOdometryWorker::enqueueEstimatorState(
    EstimatorStateUpdate update) {
  {
    std::lock_guard lock(mutex_);
    if (!accepting_) return false;
    diagnostics_.main_status = update.status;
    diagnostics_.navigation_valid = update.navigation_valid;
    if (update.status != EstimatorStatus::kTracking || !update.navigation_valid) {
      ++control_generation_;
      diagnostics_.control_generation = control_generation_;
      pending_correction_.reset();
      invalidation_requested_.store(true, std::memory_order_release);
    } else if (update.corrected_estimate.has_value()) {
      diagnostics_.last_received_correction_sequence =
          std::max(diagnostics_.last_received_correction_sequence,
                   update.correction_sequence);
      if (pending_correction_.has_value()) {
        if (update.correction_sequence <= pending_correction_->correction_sequence ||
            update.corrected_estimate->time.nanoseconds() <
                pending_correction_->corrected_estimate->time.nanoseconds()) {
          return true;
        }
        ++diagnostics_.correction_coalesced_count;
      }
      pending_correction_ = std::move(update);
      diagnostics_.last_correction_time =
          pending_correction_->corrected_estimate->time;
    }
  }
  ready_.notify_one();
  return true;
}

void PropagatedOdometryWorker::requestLoadShedding() noexcept {
  if (!load_shed_requested_.exchange(true, std::memory_order_acq_rel)) {
    ready_.notify_one();
  }
}

PropagatedOdometryWorkerDiagnostics
PropagatedOdometryWorker::diagnostics() const {
  std::lock_guard lock(mutex_);
  return diagnostics_;
}

void PropagatedOdometryWorker::run() {
  (void)pthread_setname_np(pthread_self(), "lio_propagated");
  while (true) {
    std::deque<ImuSample> batch;
    std::optional<EstimatorStateUpdate> correction;
    {
      std::unique_lock lock(mutex_);
      ready_.wait(lock, [this] { return stop_requested_.load() ||
                                        !imu_ingress_.empty() || pending_correction_.has_value() ||
                                        load_shed_requested_.load() ||
                                        invalidation_requested_.load(); });
      if (stop_requested_.load()) return;
      ++diagnostics_.worker_wakeup_count;
      batch.swap(imu_ingress_);
      correction = std::move(pending_correction_);
      diagnostics_.current_event_queue_depth = imu_ingress_.size();
    }
    if (load_shed_requested_.exchange(false, std::memory_order_acq_rel)) {
      if (!suspended_) {
        suspended_ = true;
        ++control_generation_;
        ++diagnostics_.load_shedding_count;
        ++diagnostics_.load_shedding_transition_count;
      }
      propagator_.invalidate(PropagatedOdometryStatus::kQueueOverflow);
    }
    if (invalidation_requested_.exchange(false, std::memory_order_acq_rel)) {
      propagator_.invalidate(PropagatedOdometryStatus::kMainEstimatorInvalid);
    }
    if (correction.has_value()) process(std::move(*correction));
    if (!suspended_) {
      for (auto& sample : batch) process(std::move(sample));
    }
    updateSnapshot();
  }
}

void PropagatedOdometryWorker::process(ImuSample event) {
  const Status accepted = propagator_.acceptImu(event);
  Status status = accepted;
  if (accepted.ok() &&
      (!next_publish_deadline_.has_value() ||
       event.time.nanoseconds() >= next_publish_deadline_->nanoseconds())) {
    status = propagator_.flushPendingPrediction();
  }
  std::optional<Timestamp> last_correction_time;
  bool navigation_valid = false;
  EstimatorStatus main_status = EstimatorStatus::kWaitingForSensors;
  {
    std::lock_guard lock(mutex_);
    last_correction_time = diagnostics_.last_applied_correction_time;
    navigation_valid = diagnostics_.navigation_valid;
    main_status = diagnostics_.main_status;
  }
  if (status.ok() && propagator_.diagnostics().latest_imu_time.has_value() &&
      last_correction_time.has_value()) {
    const std::int64_t correction_age =
        propagator_.diagnostics().latest_imu_time->nanoseconds() -
        last_correction_time->nanoseconds();
    if (correction_age > config_.maximum_correction_age_ns) {
      if (propagator_.diagnostics().status !=
          PropagatedOdometryStatus::kStaleCorrection) {
        std::lock_guard lock(mutex_);
        ++diagnostics_.stale_stop_count;
      }
      propagator_.invalidate(PropagatedOdometryStatus::kStaleCorrection);
    }
  }
  const bool main_valid = navigation_valid &&
                          main_status == EstimatorStatus::kTracking;
  if (status.ok() && main_valid && propagator_.valid()) {
    maybePublishOnImu(event);
  } else {
    ++publication_skip_count_;
  }
}

void PropagatedOdometryWorker::process(EstimatorStateUpdate update) {
  {
    std::lock_guard lock(mutex_);
    diagnostics_.main_status = update.status;
    diagnostics_.navigation_valid = update.navigation_valid;
    diagnostics_.correction_sequence =
        std::max(diagnostics_.correction_sequence,
                 update.correction_sequence);
    if (update.corrected_estimate.has_value()) {
    diagnostics_.last_correction_time = update.corrected_estimate->time;
    diagnostics_.last_received_correction_sequence =
        std::max(diagnostics_.last_received_correction_sequence,
                 update.correction_sequence);
    }
  }
  if (update.status != EstimatorStatus::kTracking || !update.navigation_valid) {
    propagator_.invalidate(PropagatedOdometryStatus::kMainEstimatorInvalid);
  }
  if (update.corrected_estimate.has_value()) {
    const auto started = std::chrono::steady_clock::now();
    const Status replay_status =
        propagator_.reanchorAndReplay(*update.corrected_estimate);
    const auto runtime_us = std::chrono::duration_cast<std::chrono::microseconds>(
                                std::chrono::steady_clock::now() - started)
                                .count();
    {
      std::lock_guard lock(mutex_);
      diagnostics_.last_replay_runtime_us = runtime_us;
      diagnostics_.maximum_replay_runtime_us =
          std::max(diagnostics_.maximum_replay_runtime_us, runtime_us);
    }
    if (replay_status.ok()) {
      suspended_ = false;
      std::lock_guard lock(mutex_);
      diagnostics_.last_applied_correction_time = update.corrected_estimate->time;
      diagnostics_.last_applied_correction_sequence =
          update.correction_sequence;
    }
    if (update.status != EstimatorStatus::kTracking || !update.navigation_valid) {
      propagator_.invalidate(PropagatedOdometryStatus::kMainEstimatorInvalid);
    }
  }
}

void PropagatedOdometryWorker::maybePublishOnImu(const ImuSample& sample) {
  if (!next_publish_deadline_.has_value()) {
    next_publish_deadline_ = sample.time;
  }
  if (sample.time.nanoseconds() < next_publish_deadline_->nanoseconds()) {
    ++publication_skip_count_;
    return;
  }
  const auto estimate = propagator_.estimate();
  if (!estimate.has_value() ||
      (last_published_time_.has_value() &&
       estimate->time.nanoseconds() <= last_published_time_->nanoseconds())) {
    ++publication_skip_count_;
    return;
  }
  if (imu_processed_callback_) {
    imu_processed_callback_(estimate);
  }
  last_published_time_ = estimate->time;
  ++publication_count_;
  do {
    next_publish_deadline_ = Timestamp(
        next_publish_deadline_->nanoseconds() + publish_period_ns_,
        next_publish_deadline_->clock_domain());
  } while (next_publish_deadline_->nanoseconds() <= sample.time.nanoseconds());
}

void PropagatedOdometryWorker::updateSnapshot() {
  std::lock_guard lock(mutex_);
  diagnostics_.propagator = propagator_.diagnostics();
  diagnostics_.last_published_time = last_published_time_;
  diagnostics_.next_publish_deadline = next_publish_deadline_;
  diagnostics_.publication_count = publication_count_;
  diagnostics_.publication_skip_count = publication_skip_count_;
}

}  // namespace uav::nav::lio
