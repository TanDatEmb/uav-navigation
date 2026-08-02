#include "fast_lio_core/propagation/imu_state_propagator.hpp"

#include <Eigen/Eigenvalues>
#include <algorithm>
#include <stdexcept>
#include <vector>

namespace uav::nav::lio {
namespace {

bool covarianceValid(const ManifoldState::Covariance& covariance) {
  if (!covariance.allFinite() ||
      (covariance - covariance.transpose()).cwiseAbs().maxCoeff() > 1e-8) {
    return false;
  }
  Eigen::SelfAdjointEigenSolver<ManifoldState::Covariance> solver(
      0.5 * (covariance + covariance.transpose()), Eigen::EigenvaluesOnly);
  return solver.info() == Eigen::Success &&
         solver.eigenvalues().minCoeff() >= -1e-10;
}

}  // namespace

const char* toString(PropagatedOdometryStatus status) noexcept {
  switch (status) {
    case PropagatedOdometryStatus::kWaitingForCorrection:
      return "WAITING_FOR_CORRECTION";
    case PropagatedOdometryStatus::kReady:
      return "READY";
    case PropagatedOdometryStatus::kTimestampRegression:
      return "TIMESTAMP_REGRESSION";
    case PropagatedOdometryStatus::kImuGap:
      return "IMU_GAP";
    case PropagatedOdometryStatus::kMissingBracket:
      return "MISSING_BRACKET";
    case PropagatedOdometryStatus::kInvalidState:
      return "INVALID_STATE";
    case PropagatedOdometryStatus::kStaleCorrection:
      return "STALE_CORRECTION";
    case PropagatedOdometryStatus::kMainEstimatorInvalid:
      return "MAIN_ESTIMATOR_INVALID";
    case PropagatedOdometryStatus::kQueueOverflow:
      return "QUEUE_OVERFLOW";
  }
  return "UNKNOWN";
}

ImuStatePropagator::ImuStatePropagator(ImuStatePropagatorConfig config)
    : config_(std::move(config)),
      estimator_(config_.ikfom, config_.residual_builder) {
  if (config_.imu_history_duration_ns <= 0) {
    throw std::invalid_argument("IMU history duration must be positive");
  }
}

ImuRecordResult ImuStatePropagator::acceptImu(const ImuSample& sample) {
  return validateAndRecordImu(sample, true);
}

ImuRecordResult ImuStatePropagator::recordImuForReplay(
    const ImuSample& sample) {
  return validateAndRecordImu(sample, false);
}

ImuRecordResult ImuStatePropagator::validateAndRecordImu(
    const ImuSample& sample, const bool append_pending_prediction) {
  const Status sample_status = sample.validate();
  if (!sample_status.ok()) {
    setFailure(sample_status);
    return {sample_status, ImuRecordDisposition::kRejected};
  }
  if (diagnostics_.latest_imu_time.has_value()) {
    if (!sample.time.sameClockDomain(*diagnostics_.latest_imu_time)) {
      const Status failure(StatusCode::kClockDomainMismatch,
                           "Propagated IMU clock domain changed");
      setFailure(failure);
      return {failure, ImuRecordDisposition::kRejected};
    }
    if (sample.time.nanoseconds() <=
        diagnostics_.latest_imu_time->nanoseconds()) {
      const Status failure(StatusCode::kTimestampRegression,
                           "Propagated IMU timestamp is not increasing");
      setFailure(failure);
      return {failure, ImuRecordDisposition::kRejected};
    }
    if (sample.time.nanoseconds() -
                diagnostics_.latest_imu_time->nanoseconds() >
            config_.ikfom.maximum_integration_step_ns) {
      return restartContinuity(sample);
    }
  }

  history_.push_back(sample);
  diagnostics_.latest_imu_time = sample.time;
  diagnostics_.maximum_imu_history_size =
      std::max(diagnostics_.maximum_imu_history_size, history_.size());
  if (!previous_imu_sample_.has_value()) {
    previous_imu_sample_ = sample;
  }

  if (append_pending_prediction && !requires_reanchor_ &&
      diagnostics_.propagated_time.has_value()) {
    pending_prediction_samples_.push_back(sample);
  }
  pruneHistory();
  diagnostics_.current_imu_history_size = history_.size();
  return {Status::Ok(), ImuRecordDisposition::kRecorded};
}

ImuRecordResult ImuStatePropagator::restartContinuity(const ImuSample& sample) {
  const Status restart(StatusCode::kImuGap,
                       "IMU integration gap started a new continuity epoch");
  valid_ = false;
  requires_reanchor_ = true;
  diagnostics_.requires_reanchor = true;
  ++continuity_epoch_;
  diagnostics_.continuity_epoch = continuity_epoch_;
  ++diagnostics_.continuity_reset_count;
  diagnostics_.last_continuity_reset_time = sample.time;
  ++diagnostics_.imu_gap_count;
  diagnostics_.status = PropagatedOdometryStatus::kImuGap;
  history_.clear();
  pending_prediction_samples_.clear();
  previous_imu_sample_ = sample;
  diagnostics_.anchor_time.reset();
  diagnostics_.propagated_time.reset();
  diagnostics_.latest_imu_time = sample.time;
  history_.push_back(sample);
  diagnostics_.maximum_imu_history_size =
      std::max(diagnostics_.maximum_imu_history_size, history_.size());
  diagnostics_.current_imu_history_size = history_.size();
  return {restart, ImuRecordDisposition::kContinuityRestarted};
}

Status ImuStatePropagator::flushPendingPrediction() {
  if (!diagnostics_.propagated_time.has_value() ||
      pending_prediction_samples_.empty()) {
    return Status::Ok();
  }
  if (requires_reanchor_) {
    return Status(StatusCode::kNotReady,
                  "Propagated output requires a successful correction replay");
  }
  std::vector<ImuSample> samples;
  samples.reserve(pending_prediction_samples_.size() + 1U);
  samples.push_back(*previous_imu_sample_);
  samples.insert(samples.end(), pending_prediction_samples_.begin(),
                 pending_prediction_samples_.end());
  const auto prediction = estimator_.predict(
      std::span<const ImuSample>(samples), *diagnostics_.propagated_time,
      pending_prediction_samples_.back().time);
  if (!prediction.ok()) {
    setFailure(prediction.status());
    return prediction.status();
  }
  diagnostics_.propagated_time = pending_prediction_samples_.back().time;
  previous_imu_sample_ = pending_prediction_samples_.back();
  pending_prediction_samples_.clear();
  valid_ = true;
  requires_reanchor_ = false;
  diagnostics_.requires_reanchor = false;
  diagnostics_.status = PropagatedOdometryStatus::kReady;
  return Status::Ok();
}

Status ImuStatePropagator::reanchorAndReplay(
    const StateEstimate& corrected) {
  if (!corrected.allFinite() || !covarianceValid(corrected.covariance)) {
    const Status failure(StatusCode::kNumericalFailure,
                         "Corrected propagated anchor is invalid");
    setFailure(failure);
    return failure;
  }
  std::size_t first_index = 0U;
  const Status bracket = bracketHistory(corrected.time, first_index);
  if (!bracket.ok()) {
    setFailure(bracket);
    return bracket;
  }
  if (!corrected.time.sameClockDomain(*diagnostics_.latest_imu_time)) {
    const Status failure(StatusCode::kClockDomainMismatch,
                         "Correction and propagated IMU clocks differ");
    setFailure(failure);
    return failure;
  }

  const ManifoldState previous_state = estimator_.stateView();
  const auto previous_covariance = estimator_.covariance();
  const auto previous_anchor = diagnostics_.anchor_time;
  const auto previous_propagated = diagnostics_.propagated_time;
  const bool previous_valid = valid_;
  estimator_.rebase(corrected.state, corrected.covariance);
  diagnostics_.anchor_time = corrected.time;
  diagnostics_.propagated_time = corrected.time;

  const std::vector<ImuSample> replay_samples(history_.begin() + first_index,
                                               history_.end());
  const auto replay = estimator_.predict(replay_samples, corrected.time,
                                         *diagnostics_.latest_imu_time);
  if (!replay.ok()) {
    estimator_.rebase(previous_state, previous_covariance);
    diagnostics_.anchor_time = previous_anchor;
    diagnostics_.propagated_time = previous_propagated;
    valid_ = previous_valid;
    setFailure(replay.status());
    return replay.status();
  }
  diagnostics_.propagated_time = diagnostics_.latest_imu_time;
  previous_imu_sample_ = history_.back();
  pending_prediction_samples_.clear();
  const Status state_status = validateState();
  if (!state_status.ok()) {
    estimator_.rebase(previous_state, previous_covariance);
    diagnostics_.anchor_time = previous_anchor;
    diagnostics_.propagated_time = previous_propagated;
    valid_ = previous_valid;
    setFailure(state_status);
    return state_status;
  }
  valid_ = true;
  requires_reanchor_ = false;
  diagnostics_.requires_reanchor = false;
  diagnostics_.status = PropagatedOdometryStatus::kReady;
  ++diagnostics_.reanchor_count;
  ++diagnostics_.replay_count;
  diagnostics_.last_replay_sample_count = replay_samples.size();
  pruneHistory();
  diagnostics_.current_imu_history_size = history_.size();
  return Status::Ok();
}

void ImuStatePropagator::invalidate(PropagatedOdometryStatus status) noexcept {
  valid_ = false;
  requires_reanchor_ = true;
  diagnostics_.requires_reanchor = true;
  pending_prediction_samples_.clear();
  diagnostics_.status = status;
}

bool ImuStatePropagator::valid() const noexcept { return valid_; }

std::optional<StateEstimate> ImuStatePropagator::estimate() const {
  if (!valid_ || !diagnostics_.propagated_time.has_value()) {
    return std::nullopt;
  }
  return StateEstimate{*diagnostics_.propagated_time, estimator_.stateView(),
                       estimator_.covariance()};
}

std::optional<KinematicStateEstimate>
ImuStatePropagator::kinematicEstimate() {
  const auto state_estimate = estimate();
  if (!state_estimate.has_value() || history_.empty() ||
      history_.back().time != state_estimate->time) {
    if (!history_.empty() && state_estimate.has_value()) {
      ++diagnostics_.angular_velocity.timestamp_mismatch_count;
    }
    return std::nullopt;
  }
  const auto resolved = AngularVelocityResolver::resolveExact(
      *state_estimate, history_.back(), &diagnostics_.angular_velocity);
  if (!resolved.ok()) {
    return std::nullopt;
  }
  return resolved.value();
}

const ImuStatePropagatorDiagnostics& ImuStatePropagator::diagnostics() const noexcept {
  return diagnostics_;
}

Status ImuStatePropagator::validateState() const {
  if (!estimator_.stateView().allFinite() ||
      !covarianceValid(estimator_.covariance())) {
    return Status(StatusCode::kNumericalFailure,
                  "Propagated state or covariance is invalid");
  }
  return Status::Ok();
}

Status ImuStatePropagator::bracketHistory(const Timestamp& boundary,
                                          std::size_t& first_index) const {
  if (history_.empty()) {
    return Status(StatusCode::kMissingStartBracket,
                  "No IMU history is available for replay");
  }
  const auto upper = std::lower_bound(
      history_.begin(), history_.end(), boundary.nanoseconds(),
      [](const ImuSample& sample, std::int64_t time_ns) {
        return sample.time.nanoseconds() < time_ns;
      });
  if (upper == history_.end()) {
    return Status(StatusCode::kMissingEndBracket,
                  "No IMU sample at or after replay boundary");
  }
  if (upper->time.nanoseconds() == boundary.nanoseconds()) {
    first_index = static_cast<std::size_t>(upper - history_.begin());
  } else {
    if (upper == history_.begin()) {
      return Status(StatusCode::kMissingStartBracket,
                    "No IMU sample at or before replay boundary");
    }
    first_index = static_cast<std::size_t>(upper - history_.begin() - 1);
  }
  return Status::Ok();
}

void ImuStatePropagator::pruneHistory() {
  if (!diagnostics_.latest_imu_time.has_value()) {
    return;
  }
  const std::int64_t cutoff = diagnostics_.latest_imu_time->nanoseconds() -
                              config_.imu_history_duration_ns;
  while (history_.size() >= 2U && history_[1].time.nanoseconds() < cutoff) {
    history_.pop_front();
  }
  diagnostics_.current_imu_history_size = history_.size();
}

void ImuStatePropagator::setFailure(const Status& failure) {
  valid_ = false;
  requires_reanchor_ = true;
  diagnostics_.requires_reanchor = true;
  pending_prediction_samples_.clear();
  switch (failure.code()) {
    case StatusCode::kTimestampRegression:
      diagnostics_.status = PropagatedOdometryStatus::kTimestampRegression;
      ++diagnostics_.timestamp_regression_count;
      break;
    case StatusCode::kImuGap:
    case StatusCode::kInsufficientData:
      diagnostics_.status = PropagatedOdometryStatus::kImuGap;
      ++diagnostics_.imu_gap_count;
      break;
    case StatusCode::kMissingStartBracket:
    case StatusCode::kMissingEndBracket:
      diagnostics_.status = PropagatedOdometryStatus::kMissingBracket;
      ++diagnostics_.missing_bracket_count;
      break;
    default:
      diagnostics_.status = PropagatedOdometryStatus::kInvalidState;
      ++diagnostics_.invalid_state_count;
      break;
  }
}

}  // namespace uav::nav::lio
