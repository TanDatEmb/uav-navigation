#include "fast_lio_ros/propagated_odometry_worker.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <pthread.h>
#include <stdexcept>
#include <utility>
#include <vector>

namespace uav::nav::lio {

PropagatedOdometryWorker::PropagatedOdometryWorker(
    PropagatedOdometryWorkerConfig config,
    ImuProcessedCallback imu_processed_callback)
    : config_(std::move(config)),
      propagator_(config_.propagator),
      imu_processed_callback_(std::move(imu_processed_callback)) {
  if (config_.imu_ingress_capacity == 0U ||
      config_.maximum_correction_age_ns <= 0) {
    throw std::invalid_argument("Propagated odometry worker config is invalid");
  }
  if (!(config_.publish_rate_hz > 0.0) ||
      !std::isfinite(config_.publish_rate_hz)) {
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
  if (ever_started_) {
    throw std::logic_error(
        "Propagated odometry worker instances are single-use");
  }
  stop_requested_.store(false, std::memory_order_release);
  load_shed_requested_.store(false, std::memory_order_release);
  invalidation_requested_.store(false, std::memory_order_release);
  correction_requested_.store(false, std::memory_order_release);
  ever_started_ = true;
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
    if (!accepting_) {
      return false;
    }
    if (imu_ingress_.size() >= config_.imu_ingress_capacity) {
      ++diagnostics_.queue_overflow_count;
      load_shed_requested_.store(true, std::memory_order_release);
      ready_.notify_one();
      return false;
    }
    notify = imu_ingress_.empty();
    imu_ingress_.push_back(sample);
    diagnostics_.current_imu_ingress_depth = imu_ingress_.size();
    diagnostics_.maximum_imu_ingress_depth =
        std::max(diagnostics_.maximum_imu_ingress_depth, imu_ingress_.size());
  }
  if (notify) {
    ready_.notify_one();
  }
  return true;
}

bool PropagatedOdometryWorker::enqueueEstimatorState(
    EstimatorStateUpdate update) {
  {
    std::lock_guard lock(mutex_);
    if (!accepting_) {
      return false;
    }
    diagnostics_.main_status = update.status;
    diagnostics_.navigation_valid = update.navigation_valid;
    if (update.status != EstimatorStatus::kTracking ||
        !update.navigation_valid) {
      ++control_generation_;
      diagnostics_.control_generation = control_generation_;
      discardPendingCorrectionLocked(true);
      invalidation_requested_.store(true, std::memory_order_release);
    } else if (update.corrected_estimate.has_value()) {
      const auto& estimate = *update.corrected_estimate;
      if (rejectCorrectionNotNewerThanAppliedLocked(
              update.correction_sequence, estimate.time)) {
        diagnostics_.last_received_correction_sequence =
            std::max(diagnostics_.last_received_correction_sequence,
                     update.correction_sequence);
        return true;
      }
      diagnostics_.last_received_correction_sequence =
          std::max(diagnostics_.last_received_correction_sequence,
                   update.correction_sequence);
      PendingCorrection pending{std::move(*update.corrected_estimate),
                                update.status,
                                update.navigation_valid,
                                update.correction_sequence,
                                control_generation_};
      if (pending_correction_.has_value()) {
        if (pending.correction_sequence <=
                pending_correction_->correction_sequence ||
            pending.estimate.time.nanoseconds() <
                pending_correction_->estimate.time.nanoseconds()) {
          return true;
        }
        ++diagnostics_.correction_coalesced_count;
      }
      pending_correction_ = std::move(pending);
      waiting_correction_sequence_.reset();
      diagnostics_.last_correction_time = pending_correction_->estimate.time;
      correction_requested_.store(true, std::memory_order_release);
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
    std::vector<ImuSample> batch;
    {
      std::unique_lock lock(mutex_);
      ready_.wait(lock, [this] {
        return stop_requested_.load(std::memory_order_acquire) ||
               !imu_ingress_.empty() || load_shed_requested_.load() ||
               invalidation_requested_.load() || correction_requested_.load();
      });
      if (stop_requested_.load(std::memory_order_acquire)) {
        return;
      }
      ++diagnostics_.worker_wakeup_count;
      correction_requested_.store(false, std::memory_order_release);
      batch.reserve(imu_ingress_.size());
      while (!imu_ingress_.empty()) {
        batch.push_back(std::move(imu_ingress_.front()));
        imu_ingress_.pop_front();
      }
      if (!batch.empty()) {
        ++diagnostics_.imu_batch_count;
        diagnostics_.total_imu_samples_drained += batch.size();
        diagnostics_.maximum_imu_batch_size =
            std::max(diagnostics_.maximum_imu_batch_size, batch.size());
      }
      diagnostics_.current_imu_ingress_depth = imu_ingress_.size();
    }

    if (load_shed_requested_.exchange(false, std::memory_order_acq_rel)) {
      {
        std::lock_guard lock(mutex_);
        if (!suspended_.load(std::memory_order_acquire)) {
          suspended_.store(true, std::memory_order_release);
          ++control_generation_;
          diagnostics_.control_generation = control_generation_;
          ++diagnostics_.load_shedding_count;
          ++diagnostics_.load_shedding_transition_count;
          discardPendingCorrectionLocked(true);
        }
      }
      propagator_.invalidate(PropagatedOdometryStatus::kQueueOverflow);
    }
    if (invalidation_requested_.exchange(false, std::memory_order_acq_rel)) {
      suspended_.store(true, std::memory_order_release);
      propagator_.invalidate(PropagatedOdometryStatus::kMainEstimatorInvalid);
    }

    if (!batch.empty()) {
      processImuBatch(std::span<const ImuSample>(batch));
    } else {
      (void)processPendingCorrection();
    }
    updateSnapshot();
  }
}

void PropagatedOdometryWorker::processImuBatch(
    const std::span<const ImuSample> batch) {
  bool suspended = suspended_.load(std::memory_order_acquire);
  std::uint64_t cycle_generation = 0U;
  bool navigation_valid = false;
  EstimatorStatus main_status = EstimatorStatus::kWaitingForSensors;
  {
    std::lock_guard lock(mutex_);
    cycle_generation = control_generation_;
    navigation_valid = diagnostics_.navigation_valid;
    main_status = diagnostics_.main_status;
  }

  Status status = Status::Ok();
  std::optional<ImuSample> latest_recorded_sample;
  bool batch_suspended = suspended;
  for (const auto& sample : batch) {
    const ImuRecordResult record =
        batch_suspended ? propagator_.recordImuForReplay(sample)
                        : propagator_.acceptImu(sample);
    if (record.disposition == ImuRecordDisposition::kContinuityRestarted) {
      handleContinuityReset();
      batch_suspended = true;
      latest_recorded_sample = sample;
      continue;
    }
    if (!record.ok()) {
      status = record.status;
      break;
    }
    latest_recorded_sample = sample;
  }

  const auto applied_correction_time = processPendingCorrection();

  bool transition_pending =
      load_shed_requested_.load(std::memory_order_acquire) ||
      invalidation_requested_.load(std::memory_order_acquire);
  {
    std::lock_guard lock(mutex_);
    transition_pending = transition_pending || control_generation_ != cycle_generation;
    navigation_valid = diagnostics_.navigation_valid;
    main_status = diagnostics_.main_status;
  }
  suspended = suspended_.load(std::memory_order_acquire);
  if (!transition_pending && !suspended && status.ok() &&
      latest_recorded_sample.has_value() &&
      (!next_publish_deadline_.has_value() ||
       latest_recorded_sample->time.nanoseconds() >=
           next_publish_deadline_->nanoseconds())) {
    status = propagator_.flushPendingPrediction();
  }

  std::optional<Timestamp> last_correction_time;
  {
    std::lock_guard lock(mutex_);
    last_correction_time = diagnostics_.last_applied_correction_time;
  }
  if (!transition_pending && status.ok() &&
      propagator_.diagnostics().latest_imu_time.has_value() &&
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
      status = Status(StatusCode::kInsufficientData,
                      "Propagated correction age exceeded configured maximum");
    }
  }

  // Recheck the control channel immediately before publication. A producer
  // may have requested invalidation or load shedding while replay/flush was
  // running; that transition must make this cycle fail closed.
  bool publication_transition_pending =
      load_shed_requested_.load(std::memory_order_acquire) ||
      invalidation_requested_.load(std::memory_order_acquire);
  {
    std::lock_guard lock(mutex_);
    publication_transition_pending =
        publication_transition_pending ||
        control_generation_ != cycle_generation ||
        diagnostics_.main_status != EstimatorStatus::kTracking ||
        !diagnostics_.navigation_valid || pending_correction_.has_value() ||
        diagnostics_.replay_in_progress ||
        correction_requested_.load(std::memory_order_acquire);
    navigation_valid = diagnostics_.navigation_valid;
    main_status = diagnostics_.main_status;
  }
  suspended = suspended_.load(std::memory_order_acquire);
  const bool main_valid = navigation_valid &&
                          main_status == EstimatorStatus::kTracking;
  if (!publication_transition_pending && !suspended && status.ok() &&
      main_valid &&
      propagator_.valid() && latest_recorded_sample.has_value() &&
      (!applied_correction_time.has_value() ||
       latest_recorded_sample->time.nanoseconds() >
           applied_correction_time->nanoseconds())) {
    maybePublishOnImu(*latest_recorded_sample);
  } else {
    ++publication_skip_count_;
  }
}

void PropagatedOdometryWorker::handleContinuityReset() {
  std::lock_guard lock(mutex_);
  suspended_.store(true, std::memory_order_release);
  ++control_generation_;
  diagnostics_.control_generation = control_generation_;
  discardPendingCorrectionLocked(true);
}

std::optional<Timestamp> PropagatedOdometryWorker::processPendingCorrection() {
  PendingCorrection correction;
  {
    std::lock_guard lock(mutex_);
    const auto pending = takePendingCorrectionLocked();
    if (!pending.has_value()) {
      return std::nullopt;
    }
    if (pending->control_generation != control_generation_ ||
        diagnostics_.main_status != EstimatorStatus::kTracking ||
        !diagnostics_.navigation_valid) {
      if (pending->control_generation != control_generation_) {
        ++diagnostics_.stale_generation_correction_drop_count;
      }
      return std::nullopt;
    }
    if (rejectCorrectionNotNewerThanAppliedLocked(
            pending->correction_sequence, pending->estimate.time)) {
      return std::nullopt;
    }
    diagnostics_.correction_sequence =
        std::max(diagnostics_.correction_sequence,
                 pending->correction_sequence);
    correction = *pending;
    diagnostics_.replay_in_progress = true;
  }

  // Replay is intentionally outside mutex_. Producers must remain able to
  // enqueue IMU/state updates and read diagnostics while replay is running.
  const auto started = std::chrono::steady_clock::now();
  const Status replay_status =
      propagator_.reanchorAndReplay(correction.estimate);
  const auto runtime_us = std::chrono::duration_cast<std::chrono::microseconds>(
                              std::chrono::steady_clock::now() - started)
                              .count();
  std::lock_guard lock(mutex_);
  diagnostics_.replay_in_progress = false;
  diagnostics_.last_replay_runtime_us = runtime_us;
  diagnostics_.maximum_replay_runtime_us =
      std::max(diagnostics_.maximum_replay_runtime_us, runtime_us);

  const bool generation_stable =
      control_generation_ == correction.control_generation;
  const bool main_state_valid =
      diagnostics_.main_status == EstimatorStatus::kTracking &&
      diagnostics_.navigation_valid;
  const bool transition_pending =
      load_shed_requested_.load(std::memory_order_acquire) ||
      invalidation_requested_.load(std::memory_order_acquire);
  if (!generation_stable) {
    ++diagnostics_.stale_generation_correction_drop_count;
    return std::nullopt;
  }
  if (!main_state_valid || transition_pending) {
    return std::nullopt;
  }

  if (replay_status.ok()) {
    const bool newer_pending_correction =
        pending_correction_.has_value() &&
        pending_correction_->control_generation == correction.control_generation &&
        pending_correction_->correction_sequence >
            correction.correction_sequence &&
        pending_correction_->estimate.time.nanoseconds() >=
            correction.estimate.time.nanoseconds();
    if (newer_pending_correction) {
      ++diagnostics_.correction_superseded_during_replay_count;
      suspended_.store(true, std::memory_order_release);
      propagator_.invalidate(PropagatedOdometryStatus::kWaitingForCorrection);
      return std::nullopt;
    }
    waiting_correction_sequence_.reset();
    suspended_.store(false, std::memory_order_release);
    diagnostics_.last_applied_correction_time = correction.estimate.time;
    diagnostics_.last_applied_correction_sequence =
        correction.correction_sequence;
    if (pending_correction_.has_value() &&
        pending_correction_->control_generation == correction.control_generation &&
        rejectCorrectionNotNewerThanAppliedLocked(
            pending_correction_->correction_sequence,
            pending_correction_->estimate.time)) {
      pending_correction_.reset();
    }
    return correction.estimate.time;
  }

  if (replay_status.code() == StatusCode::kMissingEndBracket) {
    if (!waiting_correction_sequence_.has_value() ||
        *waiting_correction_sequence_ != correction.correction_sequence) {
      ++diagnostics_.correction_waiting_for_bracket_count;
      ++diagnostics_.correction_missing_end_wait_count;
      waiting_correction_sequence_ = correction.correction_sequence;
    }
    requeuePendingCorrectionLocked(std::move(correction));
  } else if (replay_status.code() == StatusCode::kMissingStartBracket) {
    ++diagnostics_.correction_missing_start_drop_count;
    waiting_correction_sequence_.reset();
  }
  return std::nullopt;
}

std::optional<PendingCorrection>
PropagatedOdometryWorker::takePendingCorrectionLocked() {
  if (!pending_correction_.has_value()) {
    return std::nullopt;
  }
  auto correction = std::move(pending_correction_);
  pending_correction_.reset();
  return correction;
}

void PropagatedOdometryWorker::requeuePendingCorrectionLocked(
    PendingCorrection correction) {
  if (pending_correction_.has_value()) {
    if (correction.correction_sequence <=
            pending_correction_->correction_sequence ||
        correction.estimate.time.nanoseconds() <
            pending_correction_->estimate.time.nanoseconds()) {
      return;
    }
    ++diagnostics_.correction_coalesced_count;
  }
  pending_correction_ = std::move(correction);
}

void PropagatedOdometryWorker::discardPendingCorrectionLocked(
    const bool stale_generation) {
  if (pending_correction_.has_value() && stale_generation) {
    ++diagnostics_.stale_generation_correction_drop_count;
  }
  pending_correction_.reset();
  waiting_correction_sequence_.reset();
}

bool PropagatedOdometryWorker::rejectCorrectionNotNewerThanAppliedLocked(
    const std::uint64_t correction_sequence,
    const Timestamp& correction_time) {
  const bool old_sequence =
      diagnostics_.last_applied_correction_sequence != 0U &&
      correction_sequence < diagnostics_.last_applied_correction_sequence;
  const bool old_timestamp =
      diagnostics_.last_applied_correction_time.has_value() &&
      correction_time.nanoseconds() <
          diagnostics_.last_applied_correction_time->nanoseconds();
  const bool duplicate =
      (diagnostics_.last_applied_correction_sequence != 0U &&
       correction_sequence == diagnostics_.last_applied_correction_sequence) ||
      (diagnostics_.last_applied_correction_time.has_value() &&
       correction_time.nanoseconds() ==
           diagnostics_.last_applied_correction_time->nanoseconds());
  if (old_sequence) {
    ++diagnostics_.old_sequence_correction_drop_count;
  }
  if (old_timestamp) {
    ++diagnostics_.old_timestamp_correction_drop_count;
  }
  if (duplicate) {
    ++diagnostics_.duplicate_correction_drop_count;
  }
  return old_sequence || old_timestamp || duplicate;
}

void PropagatedOdometryWorker::maybePublishOnImu(const ImuSample& sample) {
  if (stop_requested_.load(std::memory_order_acquire)) {
    return;
  }
  {
    std::lock_guard lock(mutex_);
    if (pending_correction_.has_value() ||
        correction_requested_.load(std::memory_order_acquire) ||
        diagnostics_.replay_in_progress) {
      ++publication_skip_count_;
      return;
    }
  }
  if (propagator_.diagnostics().requires_reanchor) {
    ++publication_skip_count_;
    return;
  }
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
