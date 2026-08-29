#include "fast_lio_core/pipeline/fast_lio_pipeline.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <utility>
#include <Eigen/Eigenvalues>

#include "fast_lio_core/geometry/frame_ids.hpp"
#include "fast_lio_core/geometry/rigid_transform.hpp"
#include "fast_lio_core/registration/residual_builder.hpp"

namespace uav::nav::lio {
namespace {

constexpr std::int64_t kPriorImuHistoryDurationNs = 1'000'000'000;
// Repeated recovery events must not exponentially grow weakly observed state
// blocks until the IKFoM normal equations overflow. This is a numerical guard,
// not a claim that uncertainty above the cap has become smaller or trustworthy;
// navigation validity is still controlled by the lifecycle state machine.
constexpr double kMaximumRecoveryCovarianceEigenvalue = 100.0;

struct RecoveryCovarianceInflationResult {
  bool successful{false};
  bool clamped{false};
  double maximum_eigenvalue_before_clamp{0.0};
  double maximum_eigenvalue_after_clamp{0.0};
};

[[nodiscard]] RecoveryCovarianceInflationResult inflateRecoveryCovariance(
    ManifoldState::Covariance& covariance, const double inflation) {
  RecoveryCovarianceInflationResult result;
  const ManifoldState::Covariance symmetric =
      0.5 * (covariance + covariance.transpose());
  Eigen::SelfAdjointEigenSolver<ManifoldState::Covariance> solver(symmetric);
  if (solver.info() != Eigen::Success || !solver.eigenvalues().allFinite() ||
      solver.eigenvalues().minCoeff() < -1e-9) {
    return result;
  }
  Eigen::VectorXd eigenvalues = solver.eigenvalues().cwiseMax(0.0);
  result.maximum_eigenvalue_before_clamp =
      eigenvalues.maxCoeff() * inflation;
  result.clamped = result.maximum_eigenvalue_before_clamp >
                   kMaximumRecoveryCovarianceEigenvalue;
  for (Eigen::Index index = 0; index < eigenvalues.size(); ++index) {
    eigenvalues[index] = std::min(
        kMaximumRecoveryCovarianceEigenvalue,
        eigenvalues[index] * inflation);
  }
  covariance = solver.eigenvectors() * eigenvalues.asDiagonal() *
               solver.eigenvectors().transpose();
  covariance = 0.5 * (covariance + covariance.transpose());
  result.maximum_eigenvalue_after_clamp = eigenvalues.maxCoeff();
  result.successful = covariance.allFinite();
  return result;
}

EstimatorConfig normalizeConfig(EstimatorConfig config) {
  return config;
}

bool statusUsesEstimatorState(EstimatorStatus status) {
  return status == EstimatorStatus::kInitializingMap || status == EstimatorStatus::kTracking ||
         status == EstimatorStatus::kDegraded ||
         status == EstimatorStatus::kLost;
}

bool timestampLess(const Timestamp& left, const Timestamp& right) {
  return left.nanoseconds() < right.nanoseconds();
}

std::vector<Eigen::Vector3d> toDoublePoints(const LidarScan& scan) {
  std::vector<Eigen::Vector3d> points;
  points.reserve(scan.points.size());
  for (const LidarPoint& point : scan.points) {
    points.push_back(point.position_lidar_m.cast<double>());
  }
  return points;
}

std::vector<Eigen::Vector3d> transformPointsToOdom(
    const std::vector<Eigen::Vector3d>& points_lidar_m, const ManifoldState& state) {
  std::vector<Eigen::Vector3d> points_odom_m;
  points_odom_m.reserve(points_lidar_m.size());
  for (const Eigen::Vector3d& point_lidar_m : points_lidar_m) {
    points_odom_m.push_back(state.transformLidarPointToOdom(point_lidar_m));
  }
  return points_odom_m;
}

}  // namespace

FastLioPipeline::FastLioPipeline(EstimatorConfig config)
    : config_(normalizeConfig(std::move(config))),
      buffer_(config_.measurement_buffer),
      synchronizer_(config_.synchronization),
      initializer_(config_.initialization),
      estimator_(config_.ikfom, config_.residual_builder),
      deskewer_(config_.deskew),
      preprocessor_(config_.preprocessing),
      registration_map_(config_.registration_map),
      bootstrap_map_(config_.registration_map),
      local_map_manager_(config_.local_map),
      insertion_policy_(config_.insertion_policy),
      initial_prior_applicator_(RigidTransform(baseFrame(), imuFrame(),
                                               Eigen::Quaterniond::Identity(),
                                               Eigen::Vector3d::Zero())) {
  if (!config_.extrinsic.rotation_imu_lidar.coeffs().allFinite() ||
      config_.extrinsic.rotation_imu_lidar.squaredNorm() < 1e-18 ||
      !config_.extrinsic.translation_imu_lidar_m.allFinite() ||
      config_.lifecycle.maximum_initial_map_registration_failures == 0U ||
      config_.lifecycle.lost_after_registration_failures == 0U ||
      config_.tracking.maximum_recoverable_imu_gap_ns <= 0 ||
      config_.tracking.recovery_confirmation_updates == 0U ||
      !std::isfinite(config_.tracking.discontinuity_covariance_inflation) ||
      config_.tracking.discontinuity_covariance_inflation < 1.0 ||
      config_.tracking.discontinuity_covariance_inflation > 1000.0 ||
      config_.tracking.maximum_recoverable_imu_gap_ns <
          config_.synchronization.maximum_imu_gap_ns) {
    throw std::invalid_argument("invalid FAST-LIO pipeline configuration");
  }
  const Status prior_status = config_.initial_prior.validate();
  if (!prior_status.ok()) {
    throw std::invalid_argument(prior_status.message());
  }
  state_.set_rotation_imu_lidar(config_.extrinsic.rotation_imu_lidar);
  state_.set_position_imu_lidar_m(config_.extrinsic.translation_imu_lidar_m);
  diagnostics_.status = status_;
  diagnostics_.previous_status = status_;
  diagnostics_.deskew.deskew_mode = config_.deskew.mode;
  diagnostics_.initial_prior.source = config_.initial_prior.source;
  diagnostics_.initial_prior.context = config_.initial_prior.context;
  diagnostics_.initial_prior.attitude_mode = config_.initial_prior.mask.attitude;
  diagnostics_.initial_prior.position_enabled = config_.initial_prior.mask.position;
  diagnostics_.initial_prior.velocity_enabled = config_.initial_prior.mask.velocity;
  diagnostics_.initial_prior.status =
      config_.initial_prior.source == InitialStatePriorSource::kZero
          ? InitialPriorStatus::kNotRequired
          : InitialPriorStatus::kWaiting;
}

Status FastLioPipeline::pushImu(const ImuSample& sample) {
  const Status buffer_status = buffer_.pushImu(sample);
  if (!buffer_status.ok()) {
    ++diagnostics_.sensor.imu_drop_count;
    if (buffer_status.code() == StatusCode::kTimestampRegression) {
      ++diagnostics_.sensor.timestamp_regression_count;
    }
    diagnostics_.reason = buffer_status.message();
    return buffer_status;
  }
  retainPriorImuSample(sample);
  if (status_ == EstimatorStatus::kWaitingForSensors) {
    transitionTo(EstimatorStatus::kCollectingImu, "FIRST_IMU_SAMPLE_ACCEPTED");
  }
  if (status_ == EstimatorStatus::kCollectingImu || status_ == EstimatorStatus::kInitializingImu) {
    const Status initialization_status = addInitializationSample(sample);
    if (!initialization_status.ok()) {
      diagnostics_.reason = initialization_status.message();
      return initialization_status;
    }
    tryCompleteImuInitialization(sample.time);
  }
  return Status::Ok();
}

Status FastLioPipeline::submitInitialStatePrior(
    InitialStatePrior prior,
    const InitialStatePriorLateSubmissionPolicy late_submission_policy) {
  std::scoped_lock lock(initial_prior_mutex_);
  if (initial_prior_gate_closed_ || estimator_initialized_) {
    if (late_submission_policy ==
        InitialStatePriorLateSubmissionPolicy::kRecordRejection) {
      ++prior_candidate_count_;
      ++prior_late_rejected_count_;
      ++prior_rejected_count_;
    }
    return Status(StatusCode::kNotReady, "initial-state prior gate is closed");
  }
  ++prior_candidate_count_;
  if (prior.source != InitialStatePriorSource::kTopic ||
      prior.context != config_.initial_prior.context ||
      prior.reference_frame != lioOdomFrame() || prior.body_frame != baseFrame()) {
    ++prior_frame_rejected_count_;
    ++prior_rejected_count_;
    return Status(StatusCode::kFrameMismatch, "initial-state topic prior contract rejected");
  }
  if (prior.sample_time.nanoseconds() == 0 || !prior.allFinite()) {
    ++prior_invalid_value_count_;
    ++prior_rejected_count_;
    return Status(StatusCode::kInvalidArgument, "initial-state topic prior is zero or non-finite");
  }
  if (last_prior_candidate_time_.has_value()) {
    if (!prior.sample_time.sameClockDomain(*last_prior_candidate_time_)) {
      ++prior_timestamp_rejected_count_;
      ++prior_rejected_count_;
      return Status(StatusCode::kClockDomainMismatch, "initial-state prior clock domain changed");
    }
    if (prior.sample_time.nanoseconds() <= last_prior_candidate_time_->nanoseconds()) {
      ++prior_timestamp_rejected_count_;
      ++prior_rejected_count_;
      return Status(StatusCode::kTimestampRegression, "initial-state prior timestamp is not monotonic");
    }
  }
  last_prior_candidate_time_ = prior.sample_time;
  initial_prior_candidate_ = std::move(prior);
  ++prior_accepted_count_;
  return Status::Ok();
}

Status FastLioPipeline::setInitialStatePriorGeometry(RigidTransform base_to_imu) {
  if (estimator_initialized_ || initial_prior_gate_closed_) {
    return Status(StatusCode::kNotReady, "initial-state prior geometry can only be set before startup");
  }
  return initial_prior_applicator_.setGeometry(std::move(base_to_imu));
}

Status FastLioPipeline::pushLidar(LidarScan scan) {
  ++diagnostics_.processing.raw_lidar_count;
  const Status status = buffer_.pushLidar(std::move(scan));
  if (!status.ok()) {
    ++diagnostics_.sensor.lidar_drop_count;
    if (status.code() == StatusCode::kTimestampRegression) {
      ++diagnostics_.sensor.timestamp_regression_count;
      ++diagnostics_.processing.invalid_timestamp_rejected_count;
    }
    diagnostics_.reason = status.message();
  }
  if (status.ok()) {
    ++diagnostics_.processing.buffer_accepted_lidar_count;
  }
  return status;
}

std::optional<ProcessResult> FastLioPipeline::processNext() {
  resetTransientDiagnostics();
  if (imu_initialization_result_.has_value() && !estimator_initialized_ &&
      config_.initial_prior.source == InitialStatePriorSource::kTopic) {
    return std::nullopt;
  }
  auto synchronized = synchronizer_.synchronizeNext(buffer_);
  if (!synchronized.ok()) {
    ProcessResult result;
    result.status_before = status_;
    result.rejection_reason = synchronized.status().message();
    diagnostics_.synchronization.sync_rejection_reason = synchronized.status().message();
    diagnostics_.reason = synchronized.status().message();
    recordUncorrectedUpdate(LidarUpdateFailureClass::kSynchronization);
    if (synchronized.status().code() ==
        StatusCode::kOverlappingLidarInterval) {
      ++diagnostics_.processing.overlap_rejected_count;
    } else if (synchronized.status().code() ==
                   StatusCode::kMissingStartBracket ||
               synchronized.status().code() ==
                   StatusCode::kMissingEndBracket) {
      ++diagnostics_.processing.missing_bracket_rejected_count;
    } else if (synchronized.status().code() ==
                   StatusCode::kTimestampRegression) {
      ++diagnostics_.processing.invalid_timestamp_rejected_count;
    }
    return finalizeResult(std::move(result));
  }
  if (synchronized.value().discontinuity.has_value()) {
    return recoverFromDiscontinuity(*synchronized.value().discontinuity);
  }
  if (!synchronized.value().measurement_group.has_value()) {
    return std::nullopt;
  }
  ++diagnostics_.processing.synchronized_group_count;
  diagnostics_.synchronization.synchronized = true;
  return processInternal(*synchronized.value().measurement_group, true);
}

ProcessResult FastLioPipeline::process(const MeasurementGroup& group) {
  resetTransientDiagnostics();
  diagnostics_.synchronization.synchronized = true;
  return processInternal(group, false);
}

ProcessResult FastLioPipeline::processInternal(const MeasurementGroup& group,
                                               bool imu_samples_already_ingested) {
  const auto total_started = std::chrono::steady_clock::now();
  ProcessResult result = makeBaseResult(group);
  diagnostics_.synchronization = result.diagnostics.synchronization;
  diagnostics_.synchronization.synchronized = true;
  diagnostics_.synchronization.sync_rejection_reason.clear();
  const EstimatorStatus status_before = status_;
  result.status_before = status_before;

  const auto scan_interval = checkedDifference(group.scan.end_time,
                                               group.scan.start_time);
  const auto propagation_interval = checkedDifference(
      group.scan.end_time, group.propagation_start_time);
  if (!scan_interval.ok() || scan_interval.value().nanoseconds() < 0) {
    result.rejection_reason = "INVALID_SCAN_TIME_INTERVAL";
    diagnostics_.reason = result.rejection_reason;
    recordUncorrectedUpdate(LidarUpdateFailureClass::kSynchronization);
    return finalizeResult(std::move(result));
  }
  if (!propagation_interval.ok() || propagation_interval.value().nanoseconds() < 0) {
    result.rejection_reason = "INVALID_PROPAGATION_START_TIME";
    diagnostics_.reason = result.rejection_reason;
    recordUncorrectedUpdate(LidarUpdateFailureClass::kSynchronization);
    return finalizeResult(std::move(result));
  }

  if (!group.fullyBracketed()) {
    result.rejection_reason = "MEASUREMENT_GROUP_NOT_FULLY_BRACKETED";
    diagnostics_.synchronization.sync_rejection_reason = result.rejection_reason;
    diagnostics_.reason = result.rejection_reason;
    recordUncorrectedUpdate(LidarUpdateFailureClass::kSynchronization);
    return finalizeResult(std::move(result));
  }
  const Status scan_status = group.scan.validate();
  if (!scan_status.ok()) {
    result.rejection_reason = scan_status.message();
    diagnostics_.synchronization.sync_rejection_reason = result.rejection_reason;
    diagnostics_.reason = result.rejection_reason;
    recordUncorrectedUpdate(LidarUpdateFailureClass::kSynchronization);
    return finalizeResult(std::move(result));
  }
  // Direct callers provide the synchronized IMU group without going through
  // pushImu(). Retain the same bounded history used by the ROS ingress so a
  // prior or a rebased state can be propagated from its exact epoch. The
  // history mailbox deduplicates samples already retained by pushImu().
  for (const ImuSample& sample : group.imu_samples) {
    retainPriorImuSample(sample);
  }
  if (status_ == EstimatorStatus::kWaitingForSensors) {
    transitionTo(EstimatorStatus::kCollectingImu, "SYNCHRONIZED_MEASUREMENT_RECEIVED");
  }
  if (status_ == EstimatorStatus::kCollectingImu || status_ == EstimatorStatus::kInitializingImu) {
    if (!imu_samples_already_ingested) {
      for (const ImuSample& sample : group.imu_samples) {
        const Status sample_status = addInitializationSample(sample);
        if (!sample_status.ok()) {
          result.rejection_reason = sample_status.message();
          diagnostics_.reason = sample_status.message();
          return finalizeResult(std::move(result));
        }
      }
    }
    const std::optional<Timestamp> progress_time =
        group.imu_samples.empty() ? last_initialization_imu_time_
                                   : std::optional<Timestamp>(group.imu_samples.back().time);
    tryCompleteImuInitialization(progress_time);
    if (status_ != EstimatorStatus::kInitializingMap) {
      result.rejection_reason =
          (imu_initialization_result_.has_value() ||
           (config_.initial_prior.context == InitialStatePriorContext::kInFlightReinitialization &&
            diagnostics_.initial_prior.status == InitialPriorStatus::kWaiting))
              ? "INITIAL_STATE_PRIOR_PENDING"
              : "IMU_INITIALIZATION_NOT_READY";
      diagnostics_.reason = result.rejection_reason;
      return finalizeResult(std::move(result));
    }
  }

  if (!statusUsesEstimatorState(status_)) {
    result.rejection_reason = "ESTIMATOR_STATE_NOT_PROCESSABLE";
    diagnostics_.reason = result.rejection_reason;
    return finalizeResult(std::move(result));
  }

  Timestamp propagation_start = group.propagation_start_time;
  if (!state_time_.has_value()) {
    const bool configured_prior_owns_epoch =
        config_.initial_prior.source != InitialStatePriorSource::kZero &&
        !initial_prior_fallback_applied_ && initial_prior_state_time_.has_value();
    const Timestamp initial_epoch =
        configured_prior_owns_epoch ? *initial_prior_state_time_
                                    : group.propagation_start_time;
    if (!initial_epoch.sameClockDomain(group.scan.start_time)) {
      result.rejection_reason = "INITIAL_PRIOR_CLOCK_DOMAIN_MISMATCH";
      diagnostics_.reason = result.rejection_reason;
      recordUncorrectedUpdate(LidarUpdateFailureClass::kPrediction);
      return finalizeResult(std::move(result));
    }
    if (last_initialization_imu_time_.has_value() &&
        (!last_initialization_imu_time_->sameClockDomain(group.scan.start_time) ||
         group.scan.start_time.nanoseconds() < last_initialization_imu_time_->nanoseconds())) {
      result.rejection_reason = "SCAN_PRECEDES_IMU_INITIALIZATION_EPOCH";
      diagnostics_.reason = result.rejection_reason;
      return finalizeResult(std::move(result));
    }
    if (initial_epoch.nanoseconds() > group.scan.start_time.nanoseconds()) {
      result.rejection_reason = "INITIAL_PRIOR_AFTER_APPLICATION_EPOCH";
      diagnostics_.reason = result.rejection_reason;
      return finalizeResult(std::move(result));
    }
    // A prior owns its sample epoch. If it predates the scan, predict from
    // that exact epoch using the bracketed IMU history.
    state_time_ = initial_epoch;
    propagation_start = initial_epoch;
  } else {
    if (!state_time_->sameClockDomain(group.propagation_start_time) ||
        state_time_->nanoseconds() != group.propagation_start_time.nanoseconds()) {
      result.rejection_reason = "PROPAGATION_START_DOES_NOT_MATCH_STATE_TIME";
      diagnostics_.reason = result.rejection_reason;
      recordUncorrectedUpdate(LidarUpdateFailureClass::kPrediction);
      return finalizeResult(std::move(result));
    }
    propagation_start = *state_time_;
  }
  if (!state_time_.has_value() ||
      !state_time_->sameClockDomain(propagation_start) ||
      state_time_->nanoseconds() != propagation_start.nanoseconds()) {
    result.rejection_reason = "PROPAGATION_START_DOES_NOT_MATCH_STATE_TIME";
    diagnostics_.reason = result.rejection_reason;
    recordUncorrectedUpdate(LidarUpdateFailureClass::kPrediction);
    return finalizeResult(std::move(result));
  }

  const auto prediction_started = std::chrono::steady_clock::now();
  diagnostics_.prediction = PredictionDiagnostics{};
  diagnostics_.prediction.attempted = true;
  diagnostics_.prediction.start_time_ns = propagation_start.nanoseconds();
  diagnostics_.prediction.end_time_ns = group.scan.end_time.nanoseconds();
  diagnostics_.prediction.interval_ns =
      group.scan.end_time.nanoseconds() - propagation_start.nanoseconds();
  std::vector<ImuSample> prediction_samples;
  const Status sample_status = buildPredictionImuSamples(
      group, propagation_start, group.scan.end_time, prediction_samples);
  if (!sample_status.ok()) {
    diagnostics_.prediction.rejection_reason = sample_status.message();
    result.rejection_reason = sample_status.message();
    diagnostics_.reason = result.rejection_reason;
    recordUncorrectedUpdate(LidarUpdateFailureClass::kPrediction);
    return finalizeResult(std::move(result));
  }
  diagnostics_.prediction.imu_sample_count = prediction_samples.size();
  diagnostics_.prediction.imu_first_time_ns =
      prediction_samples.front().time.nanoseconds();
  diagnostics_.prediction.imu_last_time_ns =
      prediction_samples.back().time.nanoseconds();
  diagnostics_.prediction.integration_interval_count =
      prediction_samples.empty() ? 0U : prediction_samples.size() - 1U;
  initial_prior_gate_closed_ = true;
  diagnostics_.initial_prior.status = InitialPriorStatus::kClosed;
  auto trajectory =
      estimator_.predict(prediction_samples, propagation_start,
                         group.scan.end_time);
  diagnostics_.timing.imu_prediction_us =
      std::chrono::duration_cast<std::chrono::microseconds>(
          std::chrono::steady_clock::now() - prediction_started)
          .count();
  if (!trajectory.ok()) {
    diagnostics_.prediction.rejection_reason = trajectory.status().message();
    result.rejection_reason = trajectory.status().message();
    diagnostics_.reason = result.rejection_reason;
    recordUncorrectedUpdate(LidarUpdateFailureClass::kPrediction);
    return finalizeResult(std::move(result));
  }
  diagnostics_.prediction.successful =
      group.scan.end_time.nanoseconds() > propagation_start.nanoseconds();
  if (diagnostics_.initial_prior.applied && initial_prior_state_time_.has_value() &&
      diagnostics_.prediction.successful &&
      initial_prior_state_time_->sameClockDomain(group.scan.end_time) &&
      initial_prior_state_time_->nanoseconds() < group.scan.end_time.nanoseconds()) {
    diagnostics_.initial_prior.propagated_to_application = true;
  }
  const ManifoldState predicted_state = estimator_.stateView();
  const ManifoldState::Covariance predicted_covariance =
      estimator_.covariance();
  state_ = predicted_state;
  covariance_ = predicted_covariance;
  state_time_ = group.scan.end_time;
  result.predicted_estimate =
      StateEstimate{group.scan.end_time, predicted_state, predicted_covariance};
  result.estimate_validity = EstimateValidity::kPredictedOnly;

  const RigidTransform T_imu_lidar(imuFrame(), lidarFrame(), predicted_state.rotation_imu_lidar(),
                                   predicted_state.position_imu_lidar_m());
  const auto deskew_started = std::chrono::steady_clock::now();
  diagnostics_.deskew.deskew_attempted = true;
  auto deskewed = deskewer_.deskew(group.scan, trajectory.value(), T_imu_lidar);
  diagnostics_.deskew.deskew_runtime_us = std::chrono::duration_cast<std::chrono::microseconds>(
                                              std::chrono::steady_clock::now() - deskew_started)
                                              .count();
  diagnostics_.timing.deskew_us = diagnostics_.deskew.deskew_runtime_us;
  if (!deskewed.ok()) {
    result.rejection_reason = deskewed.status().message();
    diagnostics_.reason = result.rejection_reason;
    recordUncorrectedUpdate(LidarUpdateFailureClass::kDeskew);
    return finalizeResult(std::move(result));
  }
  diagnostics_.deskew.deskew_status = deskewed.value().status;
  diagnostics_.deskew.deskew_applied = deskewed.value().deskew_applied;
  diagnostics_.deskew.point_time_min_ns = deskewed.value().point_time_min_ns;
  diagnostics_.deskew.point_time_max_ns = deskewed.value().point_time_max_ns;
  diagnostics_.deskew.interpolation_failure_count = deskewed.value().interpolation_failure_count;

  const auto preprocessing_started = std::chrono::steady_clock::now();
  auto preprocessed = preprocessor_.process(deskewed.value().scan);
  diagnostics_.timing.preprocessing_us =
      std::chrono::duration_cast<std::chrono::microseconds>(
          std::chrono::steady_clock::now() - preprocessing_started)
          .count();
  if (!preprocessed.ok()) {
    result.rejection_reason = preprocessed.status().message();
    diagnostics_.reason = result.rejection_reason;
    recordUncorrectedUpdate(LidarUpdateFailureClass::kPreprocessing);
    return finalizeResult(std::move(result));
  }
  diagnostics_.registration.input_point_count = preprocessed.value().stats.input_point_count;
  diagnostics_.registration.filtered_point_count = preprocessed.value().stats.output_point_count;
  const std::vector<Eigen::Vector3d> points_lidar_m = toDoublePoints(preprocessed.value().scan);
  if (points_lidar_m.size() < config_.insertion_policy.minimum_point_count) {
    result.lidar_update_status = LidarUpdateStatus::kRejected;
    result.rejection_reason = "INSUFFICIENT_FILTERED_POINTS";
    diagnostics_.reason = result.rejection_reason;
    recordUncorrectedUpdate(LidarUpdateFailureClass::kInsufficientPoints);
    return finalizeResult(std::move(result));
  }

  if (status_ == EstimatorStatus::kInitializingMap && bootstrap_map_.size() == 0U) {
    bootstrap_reference_points_odom_m_ = transformPointsToOdom(points_lidar_m, predicted_state);
    static_cast<void>(bootstrap_map_.insert(bootstrap_reference_points_odom_m_));
    result.rejection_reason = "INITIAL_MAP_REFERENCE_CAPTURED";
    diagnostics_.reason = result.rejection_reason;
    diagnostics_.initial_prior.bootstrap_map_after_prior = diagnostics_.initial_prior.applied;
    diagnostics_.map.map_point_count = registration_map_.size();
    return finalizeResult(std::move(result));
  }

  const RegistrationMap& association_map =
      registration_map_.size() == 0U ? static_cast<const RegistrationMap&>(bootstrap_map_)
                                     : static_cast<const RegistrationMap&>(registration_map_);
  result.lidar_update_status = LidarUpdateStatus::kRejected;
  diagnostics_.registration.correction_attempted = true;
  ++diagnostics_.processing.correction_attempt_count;
  const IkfomCorrectionResult correction =
      estimator_.correct(points_lidar_m, association_map);
  diagnostics_.timing.residual_build_us =
      correction.residual_build_runtime_us;
  diagnostics_.timing.ikfom_update_us = correction.ikfom_update_runtime_us;
  diagnostics_.timing.measurement_model_us =
      correction.measurement_model_runtime_us;
  diagnostics_.timing.nearest_search_us =
      correction.nearest_search_runtime_us;
  diagnostics_.timing.plane_and_gate_us =
      correction.plane_and_gate_runtime_us;
  diagnostics_.timing.jacobian_build_us =
      correction.jacobian_build_runtime_us;
  diagnostics_.timing.ikfom_solver_only_us =
      correction.solver_only_runtime_us;

  diagnostics_.registration.query_count =
      correction.residual_diagnostics.query_count;
  diagnostics_.registration.valid_plane_count =
      correction.residual_diagnostics.valid_plane_count;
  diagnostics_.registration.accepted_residual_count =
      correction.residual_diagnostics.accepted_residual_count;
  diagnostics_.registration.rejected_residual_count =
      correction.residual_diagnostics.rejected_residual_count;
  diagnostics_.registration.measurement_callback_count =
      correction.measurement_callback_count;
  diagnostics_.registration.observability_rejection_count =
      correction.observability_rejection_count;
  diagnostics_.registration.residual_rms_m =
      correction.residual_diagnostics.residual_rms_m;
  diagnostics_.registration.translation_observability_min_eigenvalue =
      correction.residual_diagnostics.translation_observability_min_eigenvalue;
  diagnostics_.registration.translation_observability_max_eigenvalue =
      correction.residual_diagnostics.translation_observability_max_eigenvalue;
  diagnostics_.registration.translation_observability_ratio =
      correction.residual_diagnostics.translation_observability_ratio;
  diagnostics_.registration.translation_observability_valid =
      correction.residual_diagnostics.translation_observability_valid;
  diagnostics_.registration.iteration_count = correction.iteration_count;
  diagnostics_.registration.final_increment_norm =
      correction.final_increment_norm;
  diagnostics_.registration.converged = correction.converged;
  diagnostics_.state.correction_translation_norm_m = correction.correction_translation_norm_m;
  diagnostics_.state.correction_rotation_norm_rad = correction.correction_rotation_norm_rad;

  if (!correction.successful || !correction.finite ||
      !correction.corrected_state.allFinite()) {
    ++diagnostics_.processing.correction_failure_count;
    ++consecutive_registration_failures_;
    diagnostics_.consecutive_registration_failure_count = consecutive_registration_failures_;
    if (status_ == EstimatorStatus::kInitializingMap) {
      ++initial_map_registration_failures_;
      if (initial_map_registration_failures_ >=
          config_.lifecycle.maximum_initial_map_registration_failures) {
        transitionTo(EstimatorStatus::kLost, "INITIAL_MAP_REGISTRATION_EXHAUSTED");
      }
    }
    recordUncorrectedUpdate(LidarUpdateFailureClass::kRegistration);
    result.rejection_reason = correction.reason;
    diagnostics_.reason = correction.reason;
    return finalizeResult(std::move(result));
  }

  state_ = estimator_.stateView();
  covariance_ = estimator_.covariance();
  last_correction_time_ = group.scan.end_time;
  consecutive_registration_failures_ = 0U;
  initial_map_registration_failures_ = 0U;
  diagnostics_.consecutive_registration_failure_count = 0U;
  if (!tracking_ever_confirmed_) {
    tracking_ever_confirmed_ = true;
    consecutive_uncorrected_lidar_updates_ = 0U;
    consecutive_recovery_successes_ = 0U;
    transitionTo(
        EstimatorStatus::kTracking,
        correction.converged ? "LIDAR_CORRECTION_CONVERGED"
                             : "LIDAR_CORRECTION_ACCEPTED_AT_ITERATION_LIMIT");
  } else if (status_ == EstimatorStatus::kDegraded ||
             status_ == EstimatorStatus::kLost) {
    ++consecutive_recovery_successes_;
    if (map_recovery_required_) {
      if (status_ != EstimatorStatus::kLost) {
        transitionTo(EstimatorStatus::kDegraded,
                     "LOCAL_MAP_RECOVERY_REQUIRED");
      }
    } else if (consecutive_recovery_successes_ >=
        config_.tracking.recovery_confirmation_updates) {
      consecutive_uncorrected_lidar_updates_ = 0U;
      consecutive_recovery_successes_ = 0U;
      transitionTo(EstimatorStatus::kTracking,
                   "LIDAR_RECOVERY_CONFIRMED");
    } else {
      transitionTo(EstimatorStatus::kDegraded,
                   "LIDAR_RECOVERY_CONFIRMATION_PENDING");
    }
  }
  diagnostics_.last_update_failure_class =
      LidarUpdateFailureClass::kNone;

  result.lidar_update_status = LidarUpdateStatus::kSucceeded;
  diagnostics_.registration.correction_succeeded = true;
  ++diagnostics_.processing.correction_success_count;
  result.estimate_validity = EstimateValidity::kCorrected;
  result.scan_time = group.scan.end_time;
  result.corrected_estimate = StateEstimate{group.scan.end_time, state_, covariance_};
  const auto kinematic = AngularVelocityResolver::resolve(
      *result.corrected_estimate, group.imu_samples,
      &diagnostics_.corrected_angular_velocity);
  if (kinematic.ok()) {
    result.corrected_kinematic_estimate = kinematic.value();
  }
  result.registered_points_odom_m = transformPointsToOdom(points_lidar_m, state_);
  MapInsertionContext insertion_context;
  insertion_context.estimator_tracking = status_ == EstimatorStatus::kTracking;
  insertion_context.lidar_update_successful =
      result.lidar_update_status == LidarUpdateStatus::kSucceeded;
  insertion_context.correction_usable = correction.successful;
  insertion_context.transform_finite = state_.allFinite();
  insertion_context.filtered_point_count = points_lidar_m.size();
  const bool insertion_allowed = local_map_manager_.insertionAllowed();
  const IkdTreeStageTimingTotals map_timing_before =
      registration_map_.stageTimingTotals();
  diagnostics_.map.map_insertion_frozen = !insertion_allowed;
  if (!insertion_allowed) {
    map_recovery_required_ = true;
  }
  if (insertion_allowed && insertion_policy_.permits(insertion_context)) {
    diagnostics_.map.map_update_performed = true;
    const auto map_update_started = std::chrono::steady_clock::now();
    diagnostics_.map.map_size_before_insert = registration_map_.size();
    diagnostics_.map.map_candidate_count =
        result.registered_points_odom_m.size();
    std::size_t inserted = 0U;
    if (registration_map_.size() == 0U && !bootstrap_reference_points_odom_m_.empty()) {
      inserted += registration_map_.insert(bootstrap_reference_points_odom_m_);
      bootstrap_reference_points_odom_m_.clear();
      bootstrap_map_.clear();
    }
    inserted += registration_map_.insert(result.registered_points_odom_m);
    diagnostics_.map.map_inserted_count = inserted;
    diagnostics_.map.map_size_after_insert = registration_map_.size();
    diagnostics_.map.map_size_before_maintenance = registration_map_.size();
    const auto maintenance_started = std::chrono::steady_clock::now();
    const LocalMapUpdate local_map_update =
        local_map_manager_.update(registration_map_, state_.position_odom_imu_m());
    diagnostics_.map.map_maintenance_us =
        std::chrono::duration_cast<std::chrono::microseconds>(
            std::chrono::steady_clock::now() - maintenance_started)
            .count();
    diagnostics_.map.map_size_after_maintenance = registration_map_.size();
    diagnostics_.map.crop_performed = local_map_update.crop_performed;
    diagnostics_.map.crop_removed_count = local_map_update.removed_point_count;
    diagnostics_.map.crop_triggered_by_motion =
        local_map_update.crop_triggered_by_motion;
    diagnostics_.map.absolute_guard_triggered =
        local_map_update.absolute_guard_triggered;
    diagnostics_.map.absolute_guard_recovery_failed =
        local_map_update.absolute_guard_recovery_failed;
    diagnostics_.map.map_insertion_frozen =
        local_map_update.insertion_frozen;
    if (local_map_update.absolute_guard_recovery_failed) {
      map_recovery_required_ = true;
      if (status_ == EstimatorStatus::kTracking) {
        transitionTo(EstimatorStatus::kDegraded,
                     "LOCAL_MAP_ABSOLUTE_GUARD_EXCEEDED");
      }
    }
    diagnostics_.map.map_count_before =
        local_map_update.map_count_before;
    diagnostics_.map.map_count_after_crop =
        local_map_update.map_count_after_crop;
    diagnostics_.map.inserted_point_count = inserted;
    diagnostics_.map.removed_point_count = local_map_update.removed_point_count;
    diagnostics_.map.local_map_center_odom_m = local_map_update.center_odom_m;
    diagnostics_.map.local_map_half_extent_m = local_map_update.half_extent_m;
    diagnostics_.map.map_update_runtime_us =
        std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now() -
                                                              map_update_started)
            .count();
    diagnostics_.timing.map_insert_crop_us =
        diagnostics_.map.map_update_runtime_us;
    diagnostics_.timing.map_maintenance_us =
        diagnostics_.map.map_maintenance_us;
  }
  diagnostics_.map.map_point_count = registration_map_.size();
  const IkdTreeStageTimingTotals map_timing_after =
      registration_map_.stageTimingTotals();
  diagnostics_.timing.map_prepare_us =
      map_timing_after.prepare_us - map_timing_before.prepare_us;
  diagnostics_.timing.map_sort_us =
      map_timing_after.sort_us - map_timing_before.sort_us;
  diagnostics_.timing.map_add_points_us =
      map_timing_after.add_points_us - map_timing_before.add_points_us;
  diagnostics_.timing.map_crop_us =
      map_timing_after.crop_us - map_timing_before.crop_us;
  diagnostics_.timing.map_bookkeeping_us =
      map_timing_after.bookkeeping_us - map_timing_before.bookkeeping_us;
  ++corrected_scan_count_;
  diagnostics_.reason =
      correction.converged
          ? "LIDAR_CORRECTION_CONVERGED"
          : "LIDAR_CORRECTION_ACCEPTED_AT_ITERATION_LIMIT";
  if (map_recovery_required_) {
    diagnostics_.reason = "LOCAL_MAP_RECOVERY_REQUIRED";
  }
  diagnostics_.timing.total_processing_us =
      std::chrono::duration_cast<std::chrono::microseconds>(
          std::chrono::steady_clock::now() - total_started)
          .count();
  return finalizeResult(std::move(result));
}

void FastLioPipeline::reset() {
  transitionTo(EstimatorStatus::kResetting, "RESET_REQUESTED");
  buffer_.clear();
  synchronizer_.reset();
  initializer_.reset();
  registration_map_.clear();
  bootstrap_map_.clear();
  local_map_manager_.reset();
  state_ = ManifoldState{};
  state_.set_rotation_imu_lidar(config_.extrinsic.rotation_imu_lidar);
  state_.set_position_imu_lidar_m(config_.extrinsic.translation_imu_lidar_m);
  estimator_.reset(state_);
  {
    std::scoped_lock lock(initial_prior_mutex_);
    initial_prior_candidate_.reset();
    last_prior_candidate_time_.reset();
    prior_candidate_count_ = 0;
    prior_accepted_count_ = 0;
    prior_rejected_count_ = 0;
    prior_late_rejected_count_ = 0;
    prior_stale_rejected_count_ = 0;
    prior_future_rejected_count_ = 0;
    prior_frame_rejected_count_ = 0;
    prior_timestamp_rejected_count_ = 0;
    prior_invalid_value_count_ = 0;
  }
  imu_initialization_result_.reset();
  prior_wait_start_time_.reset();
  estimator_initialized_ = false;
  initial_prior_gate_closed_ = false;
  initial_prior_applied_ = false;
  initial_prior_fallback_applied_ = false;
  initial_prior_state_time_.reset();
  prior_imu_history_.clear();
  ++generation_;
  covariance_ = estimator_.covariance();
  state_time_.reset();
  last_initialization_imu_time_.reset();
  last_correction_time_.reset();
  bootstrap_reference_points_odom_m_.clear();
  consecutive_registration_failures_ = 0U;
  initial_map_registration_failures_ = 0U;
  corrected_scan_count_ = 0U;
  consecutive_uncorrected_lidar_updates_ = 0U;
  consecutive_recovery_successes_ = 0U;
  map_recovery_required_ = false;
  tracking_ever_confirmed_ = false;
  diagnostics_ = EstimatorDiagnostics{};
  diagnostics_.deskew.deskew_mode = config_.deskew.mode;
  diagnostics_.initial_prior.source = config_.initial_prior.source;
  diagnostics_.initial_prior.context = config_.initial_prior.context;
  diagnostics_.initial_prior.attitude_mode = config_.initial_prior.mask.attitude;
  diagnostics_.initial_prior.position_enabled = config_.initial_prior.mask.position;
  diagnostics_.initial_prior.velocity_enabled = config_.initial_prior.mask.velocity;
  diagnostics_.initial_prior.status =
      config_.initial_prior.source == InitialStatePriorSource::kZero
          ? InitialPriorStatus::kNotRequired
          : InitialPriorStatus::kWaiting;
  status_ = EstimatorStatus::kWaitingForSensors;
  diagnostics_.status = status_;
  diagnostics_.previous_status = EstimatorStatus::kResetting;
  diagnostics_.status_transition_count = 2U;
  diagnostics_.reason = "RESET_COMPLETE";
}

Status FastLioPipeline::buildPredictionImuSamples(
    const MeasurementGroup& group, const Timestamp& start_time,
    const Timestamp& end_time, std::vector<ImuSample>& output) const {
  if (!start_time.sameClockDomain(end_time)) {
    return Status(StatusCode::kClockDomainMismatch,
                  "Prediction interval clock domain changed");
  }
  if (end_time.nanoseconds() < start_time.nanoseconds()) {
    return Status(StatusCode::kTimestampRegression,
                  "Prediction interval end precedes start");
  }

  const auto validate_sequence = [&](const auto& samples) -> Status {
    std::optional<Timestamp> previous;
    for (const ImuSample& sample : samples) {
      const Status sample_status = sample.validate();
      if (!sample_status.ok()) {
        return sample_status;
      }
      if (!sample.time.sameClockDomain(start_time)) {
        return Status(StatusCode::kClockDomainMismatch,
                      "Prediction IMU clock domain changed");
      }
      if (previous.has_value() &&
          sample.time.nanoseconds() <= previous->nanoseconds()) {
        return Status(StatusCode::kTimestampRegression,
                      "Prediction IMU history is not strictly increasing");
      }
      previous = sample.time;
    }
    return Status::Ok();
  };

  const Status group_status = validate_sequence(group.imu_samples);
  if (!group_status.ok()) {
    return group_status;
  }

  std::vector<ImuSample> merged_history;
  std::span<const ImuSample> prediction_samples;
  // During normal tracking, synchronization already supplies the complete
  // bracket for the current scan. Do not merge the retained prior history on
  // every correction. The history path is reserved for a prior/recovery whose
  // application epoch predates the synchronized group.
  const bool steady_state =
      start_time.nanoseconds() == group.scan.start_time.nanoseconds();
  if (steady_state) {
    prediction_samples = group.imu_samples;
  } else {
    const Status history_status = validate_sequence(prior_imu_history_);
    if (!history_status.ok()) {
      return history_status;
    }
    merged_history.reserve(prior_imu_history_.size() + group.imu_samples.size());
    std::size_t history_index = 0U;
    std::size_t group_index = 0U;
    while (history_index < prior_imu_history_.size() ||
           group_index < group.imu_samples.size()) {
      if (history_index == prior_imu_history_.size()) {
        merged_history.push_back(group.imu_samples[group_index++]);
        continue;
      }
      if (group_index == group.imu_samples.size()) {
        merged_history.push_back(prior_imu_history_[history_index++]);
        continue;
      }
      const auto history_time =
          prior_imu_history_[history_index].time.nanoseconds();
      const auto group_time = group.imu_samples[group_index].time.nanoseconds();
      if (history_time < group_time) {
        merged_history.push_back(prior_imu_history_[history_index++]);
      } else if (group_time < history_time) {
        merged_history.push_back(group.imu_samples[group_index++]);
      } else {
        // The synchronized group is authoritative at duplicate epochs.
        merged_history.push_back(group.imu_samples[group_index++]);
        ++history_index;
      }
    }
    prediction_samples = merged_history;
  }
  if (prediction_samples.empty()) {
    return Status(StatusCode::kMissingStartBracket,
                  "Prediction IMU history is empty");
  }

  const auto first_after_start = std::lower_bound(
      prediction_samples.begin(), prediction_samples.end(), start_time.nanoseconds(),
      [](const ImuSample& sample, std::int64_t time_ns) {
        return sample.time.nanoseconds() < time_ns;
      });
  std::size_t first_index =
      static_cast<std::size_t>(first_after_start - prediction_samples.begin());
  if (first_index == prediction_samples.size() ||
      (first_index > 0U &&
       prediction_samples[first_index].time.nanoseconds() != start_time.nanoseconds())) {
    if (first_index == 0U) {
      return Status(StatusCode::kMissingStartBracket,
                    "Prediction IMU start bracket is missing");
    }
    --first_index;
  }
  if (prediction_samples[first_index].time.nanoseconds() > start_time.nanoseconds()) {
    return Status(StatusCode::kMissingStartBracket,
                  "Prediction IMU start bracket is missing");
  }

  const auto last_before_end = std::upper_bound(
      prediction_samples.begin(), prediction_samples.end(), end_time.nanoseconds(),
      [](std::int64_t time_ns, const ImuSample& sample) {
        return time_ns < sample.time.nanoseconds();
      });
  if (last_before_end == prediction_samples.begin()) {
    return Status(StatusCode::kMissingEndBracket,
                  "Prediction IMU end bracket is missing");
  }
  const std::size_t last_index =
      static_cast<std::size_t>((last_before_end - prediction_samples.begin()) - 1U);
  std::size_t end_index = last_index;
  if (prediction_samples[last_index].time.nanoseconds() < end_time.nanoseconds() &&
      last_index + 1U >= prediction_samples.size()) {
    return Status(StatusCode::kMissingEndBracket,
                  "Prediction IMU end bracket is missing");
  }
  if (prediction_samples[last_index].time.nanoseconds() < end_time.nanoseconds()) {
    end_index = last_index + 1U;
  }

  const std::int64_t configured_gap =
      group.max_imu_gap_ns > 0 ? group.max_imu_gap_ns
                               : config_.synchronization.maximum_imu_gap_ns;
  // Validate only gaps that participate in [start_time, end_time]. The
  // retained sample immediately before an exact start bracket may itself be
  // separated by a discontinuity that was already handled by the rebase.
  for (std::size_t index = first_index + 1U; index <= end_index; ++index) {
    const std::int64_t gap = prediction_samples[index].time.nanoseconds() -
                             prediction_samples[index - 1U].time.nanoseconds();
    if (gap <= 0) {
      return Status(StatusCode::kTimestampRegression,
                    "Prediction IMU history timestamp regressed");
    }
    if (configured_gap > 0 && gap > configured_gap) {
      return Status(StatusCode::kImuGap,
                    "Prediction IMU bracket contains an excessive gap");
    }
  }
  if (end_index == first_index && start_time.nanoseconds() != end_time.nanoseconds()) {
    return Status(StatusCode::kMissingEndBracket,
                  "Prediction IMU end bracket is missing");
  }

  output.assign(
      prediction_samples.begin() + static_cast<std::ptrdiff_t>(first_index),
      prediction_samples.begin() + static_cast<std::ptrdiff_t>(end_index + 1U));
  if (output.empty()) {
    return Status(StatusCode::kInsufficientData,
                  "Prediction IMU bracket produced no samples");
  }
  return Status::Ok();
}

void FastLioPipeline::retainPriorImuSample(const ImuSample& sample) {
  if (!prior_imu_history_.empty()) {
    if (!sample.time.sameClockDomain(prior_imu_history_.back().time) ||
        sample.time.nanoseconds() < prior_imu_history_.back().time.nanoseconds()) {
      return;
    }
    if (sample.time.nanoseconds() == prior_imu_history_.back().time.nanoseconds()) {
      prior_imu_history_.back() = sample;
      return;
    }
  }
  prior_imu_history_.push_back(sample);
  while (prior_imu_history_.size() >= 2U &&
         sample.time.nanoseconds() -
                 prior_imu_history_.front().time.nanoseconds() >
             kPriorImuHistoryDurationNs) {
    prior_imu_history_.pop_front();
  }
}

EstimatorStatus FastLioPipeline::status() const noexcept { return status_; }

const ManifoldState& FastLioPipeline::state() const noexcept { return state_; }

const ManifoldState::Covariance& FastLioPipeline::covariance() const noexcept {
  return covariance_;
}

EstimatorDiagnostics FastLioPipeline::diagnostics() const {
  EstimatorDiagnostics output = diagnostics_;
  copyInitialPriorMailboxDiagnostics(output);
  fillStateDiagnostics(output);
  return output;
}

std::size_t FastLioPipeline::pendingLidarCount() const { return buffer_.lidarSize(); }

void FastLioPipeline::copyInitialPriorMailboxDiagnostics(
    EstimatorDiagnostics& output) const {
  std::scoped_lock lock(initial_prior_mutex_);
  if (initial_prior_candidate_.has_value() && !output.initial_prior.applied) {
    output.initial_prior.status = InitialPriorStatus::kCandidateAvailable;
  }
  output.initial_prior.candidate_count = prior_candidate_count_;
  output.initial_prior.accepted_count = prior_accepted_count_;
  output.initial_prior.rejected_count = prior_rejected_count_;
  output.initial_prior.late_rejected_count = prior_late_rejected_count_;
  output.initial_prior.stale_rejected_count = prior_stale_rejected_count_;
  output.initial_prior.future_rejected_count = prior_future_rejected_count_;
  output.initial_prior.frame_rejected_count = prior_frame_rejected_count_;
  output.initial_prior.timestamp_rejected_count = prior_timestamp_rejected_count_;
  output.initial_prior.invalid_value_count = prior_invalid_value_count_;
}

std::size_t FastLioPipeline::registrationMapSize() const noexcept {
  return registration_map_.size();
}

const std::optional<Timestamp>& FastLioPipeline::stateTime() const noexcept {
  return state_time_;
}

const std::optional<Timestamp>&
FastLioPipeline::synchronizationEpoch() const noexcept {
  return synchronizer_.epoch();
}

ProcessResult FastLioPipeline::recoverFromDiscontinuity(
    const PropagationDiscontinuity& discontinuity) {
  ProcessResult result;
  result.status_before = status_;
  result.scan_time = discontinuity.scan_end;
  result.lidar_update_status = LidarUpdateStatus::kRejected;
  result.rejection_reason = "IMU_PROPAGATION_DISCONTINUITY";
  diagnostics_.synchronization.scan_start_ns =
      discontinuity.scan_start.nanoseconds();
  diagnostics_.synchronization.scan_end_ns =
      discontinuity.scan_end.nanoseconds();
  diagnostics_.synchronization.imu_gap_max_ns =
      discontinuity.gap_duration_ns;
  diagnostics_.synchronization.sync_rejection_reason =
      result.rejection_reason;
  ++diagnostics_.propagation_discontinuity_count;
  diagnostics_.last_propagation_gap_ns = discontinuity.gap_duration_ns;

  if (statusUsesEstimatorState(status_) && state_time_.has_value()) {
    const bool long_gap =
        discontinuity.gap_duration_ns >
        config_.tracking.maximum_recoverable_imu_gap_ns;
    const RecoveryCovarianceInflationResult inflation =
        inflateRecoveryCovariance(
            covariance_,
            config_.tracking.discontinuity_covariance_inflation);
    diagnostics_.recovery_covariance_maximum_eigenvalue_before_clamp =
        std::max(
            diagnostics_.recovery_covariance_maximum_eigenvalue_before_clamp,
            inflation.maximum_eigenvalue_before_clamp);
    diagnostics_.recovery_covariance_maximum_eigenvalue_after_clamp =
        std::max(
            diagnostics_.recovery_covariance_maximum_eigenvalue_after_clamp,
            inflation.maximum_eigenvalue_after_clamp);
    if (inflation.clamped) {
      ++diagnostics_.recovery_covariance_clamp_count;
    }
    if (!inflation.successful) {
      transitionTo(EstimatorStatus::kLost,
                   "IMU_DISCONTINUITY_COVARIANCE_INFLATION_FAILED");
      result.rejection_reason = diagnostics_.reason;
      return finalizeResult(std::move(result));
    }
    estimator_.rebase(state_, covariance_);
    state_time_ = discontinuity.resume_time;
    recordUncorrectedUpdate(
        LidarUpdateFailureClass::kPropagationDiscontinuity);
    if (!tracking_ever_confirmed_) {
      if (long_gap) {
        bootstrap_map_.clear();
        bootstrap_reference_points_odom_m_.clear();
        initial_map_registration_failures_ = 0U;
        transitionTo(EstimatorStatus::kInitializingMap,
                     registration_map_.size() == 0U
                         ? "INITIAL_MAP_RESTART_AFTER_LONG_IMU_DISCONTINUITY"
                         : "INITIAL_MAP_REGISTRATION_MAP_INVARIANT_VIOLATION");
      } else {
        transitionTo(
            EstimatorStatus::kInitializingMap,
            "INITIAL_MAP_RECOVERY_AFTER_SHORT_IMU_DISCONTINUITY");
      }
    } else if (long_gap) {
      transitionTo(EstimatorStatus::kLost, "IMU_DISCONTINUITY_LOCAL_LOST");
    } else {
      diagnostics_.reason = "IMU_DISCONTINUITY_RECOVERY_REBASE";
    }
  }
  result.rejection_reason = diagnostics_.reason;
  return finalizeResult(std::move(result));
}

Status FastLioPipeline::addInitializationSample(const ImuSample& sample) {
  if (last_initialization_imu_time_.has_value()) {
    if (!sample.time.sameClockDomain(*last_initialization_imu_time_)) {
      return Status(StatusCode::kClockDomainMismatch, "Initialization IMU clock domain changed");
    }
    if (!timestampLess(*last_initialization_imu_time_, sample.time)) {
      // Synchronized groups intentionally overlap at bracket samples.
      return Status::Ok();
    }
  }
  const Status status = initializer_.addSample(sample);
  if (status.ok()) {
    last_initialization_imu_time_ = sample.time;
    updateInitializationDiagnostics();
    if (initializer_.hasEnoughSamples() && status_ == EstimatorStatus::kCollectingImu) {
      transitionTo(EstimatorStatus::kInitializingImu, "MINIMUM_IMU_SAMPLES_COLLECTED");
    }
  }
  return status;
}

void FastLioPipeline::tryCompleteImuInitialization(
    const std::optional<Timestamp>& progress_time) {
  if (!initializer_.hasEnoughSamples() || status_ == EstimatorStatus::kInitializingMap ||
      status_ == EstimatorStatus::kTracking || status_ == EstimatorStatus::kDegraded ||
      status_ == EstimatorStatus::kLost) {
    return;
  }
  if (!imu_initialization_result_.has_value()) {
    transitionTo(EstimatorStatus::kInitializingImu, "EVALUATING_IMU_INITIALIZATION");
    prior_wait_start_time_ = progress_time.has_value() ? progress_time : last_initialization_imu_time_;
    diagnostics_.initial_prior.application_timestamp_ns =
        prior_wait_start_time_.has_value() ? prior_wait_start_time_->nanoseconds() : 0;
    if (config_.initial_prior.context == InitialStatePriorContext::kInFlightReinitialization) {
      // In-flight recovery is prior-driven. It may collect IMU for timing and
      // evidence, but it must not invoke the stationary initializer.
      const Timestamp application_time = progress_time.value_or(
          prior_wait_start_time_.value_or(last_initialization_imu_time_.value_or(Timestamp{})));
      if (!resolveInitialStatePrior(application_time)) return;
      InitialStatePrior prior;
      {
        std::scoped_lock lock(initial_prior_mutex_);
        if (initial_prior_candidate_.has_value()) prior = *initial_prior_candidate_;
      }
      if (prior.mask.attitude != PriorAttitudeMode::kFull) {
        diagnostics_.initial_prior.status = InitialPriorStatus::kRejected;
        diagnostics_.initial_prior.reason = "IN_FLIGHT_PRIOR_REQUIRES_FULL_ATTITUDE";
        diagnostics_.reason = diagnostics_.initial_prior.reason;
        return;
      }
      const auto quality = initializer_.quality();
      if (quality.samples_collected == 0U || !quality.accel_mean_m_s2.allFinite() ||
          quality.measured_gravity_norm_m_s2 < 1e-6) {
        diagnostics_.initial_prior.status = InitialPriorStatus::kRejected;
        diagnostics_.initial_prior.reason = "IN_FLIGHT_PRIOR_IMU_EVIDENCE_INVALID";
        diagnostics_.reason = diagnostics_.initial_prior.reason;
        return;
      }
      InitializationResult result;
      result.quality = quality;
      result.orientation_odom_imu = Eigen::Quaterniond::Identity();
      result.gyro_bias_rad_s = Eigen::Vector3d::Zero();
      result.accel_bias_m_s2 = Eigen::Vector3d::Zero();
      result.gravity_odom_m_s2 = Eigen::Vector3d(0.0, 0.0, -9.80665);
      imu_initialization_result_ = result;
    } else {
      auto initialized = initializer_.tryInitialize();
      updateInitializationDiagnostics();
      if (!initialized.ok()) {
        diagnostics_.initialization.initialization_status = initialized.status().message();
        diagnostics_.reason = initialized.status().message();
        return;
      }
      imu_initialization_result_ = initialized.value();
    }
  }

  const Timestamp application_time = progress_time.value_or(
      prior_wait_start_time_.value_or(last_initialization_imu_time_.value_or(Timestamp{})));
  if (!resolveInitialStatePrior(application_time)) {
    return;
  }

  state_.set_orientation_odom_imu(imu_initialization_result_->orientation_odom_imu);
  state_.set_gyro_bias_rad_s(imu_initialization_result_->gyro_bias_rad_s);
  state_.set_accel_bias_m_s2(imu_initialization_result_->accel_bias_m_s2);
  state_.set_gravity_odom_m_s2(imu_initialization_result_->gravity_odom_m_s2);
  state_.set_rotation_imu_lidar(config_.extrinsic.rotation_imu_lidar);
  state_.set_position_imu_lidar_m(config_.extrinsic.translation_imu_lidar_m);
  ManifoldState prior_state;
  prior_state.set_orientation_odom_imu(imu_initialization_result_->orientation_odom_imu);
  prior_state.set_gyro_bias_rad_s(imu_initialization_result_->gyro_bias_rad_s);
  prior_state.set_accel_bias_m_s2(imu_initialization_result_->accel_bias_m_s2);
  prior_state.set_gravity_odom_m_s2(imu_initialization_result_->gravity_odom_m_s2);
  prior_state.set_rotation_imu_lidar(config_.extrinsic.rotation_imu_lidar);
  prior_state.set_position_imu_lidar_m(config_.extrinsic.translation_imu_lidar_m);
  prior_state.normalize();

  InitialStatePrior prior;
  {
    std::scoped_lock lock(initial_prior_mutex_);
    if (config_.initial_prior.source == InitialStatePriorSource::kTopic &&
        initial_prior_candidate_.has_value()) {
      prior = *initial_prior_candidate_;
    }
  }
  if (config_.initial_prior.source == InitialStatePriorSource::kZero) {
    prior = makeZeroPrior(application_time);
  } else if (config_.initial_prior.source == InitialStatePriorSource::kFixed) {
    prior = makeFixedPrior(application_time);
  } else if (initial_prior_fallback_applied_) {
    prior = config_.initial_prior.ground_fallback == InitialPriorFallback::kFixed
                ? makeFixedPrior(application_time)
                : makeZeroPrior(application_time);
  }
  ManifoldState applied_state;
  const Status apply_status = initial_prior_applicator_.apply(
      prior, prior_state, config_.initial_prior.maximum_full_attitude_tilt_disagreement_rad,
      applied_state);
  if (!apply_status.ok()) {
    diagnostics_.initial_prior.status = InitialPriorStatus::kRejected;
    diagnostics_.initial_prior.reason = apply_status.message();
    diagnostics_.reason = apply_status.message();
    return;
  }

  std::optional<ManifoldState::Covariance> prior_covariance;
  if (prior.covariance.has_value()) {
    const auto symmetric =
        0.5 * (*prior.covariance + prior.covariance->transpose());
    Eigen::SelfAdjointEigenSolver<ManifoldState::Covariance> solver(symmetric);
    if (solver.info() != Eigen::Success || !symmetric.allFinite() ||
        solver.eigenvalues().minCoeff() < -1e-9) {
      diagnostics_.initial_prior.status = InitialPriorStatus::kRejected;
      diagnostics_.initial_prior.reason = "INITIAL_PRIOR_COVARIANCE_NOT_PSD";
      diagnostics_.reason = diagnostics_.initial_prior.reason;
      return;
    }
    prior_covariance = symmetric;
  }

  state_ = applied_state;
  estimator_.initialize(state_);
  estimator_initialized_ = true;
  covariance_ = estimator_.covariance();
  initial_prior_state_time_ = prior.sample_time;
  diagnostics_.initial_prior.state_time_ns = prior.sample_time.nanoseconds();
  diagnostics_.initial_prior.generation = prior.generation;
  if (prior_covariance.has_value()) {
    covariance_ = *prior_covariance;
    estimator_.rebase(state_, covariance_);
    diagnostics_.initial_prior.covariance_applied = true;
  }
  diagnostics_.initialization.initialization_status = "INITIALIZED";
  diagnostics_.initial_prior.applied = true;
  diagnostics_.initial_prior.fallback_applied = initial_prior_fallback_applied_;
  diagnostics_.initial_prior.propagated_to_application = false;
  transitionTo(EstimatorStatus::kInitializingMap,
               initial_prior_fallback_applied_ ? "IMU_INITIALIZATION_PRIOR_FALLBACK_ACCEPTED"
                                               : "IMU_INITIALIZATION_PRIOR_ACCEPTED");
}

InitialStatePrior FastLioPipeline::makeZeroPrior(const Timestamp& sample_time) const {
  InitialStatePrior prior;
  prior.sample_time = sample_time;
  prior.source = InitialStatePriorSource::kZero;
  prior.context = config_.initial_prior.context;
  prior.mask = config_.initial_prior.mask;
  prior.provenance = "zero";
  prior.linear_velocity_base_m_s = Eigen::Vector3d::Zero();
  prior.angular_velocity_base_rad_s = Eigen::Vector3d::Zero();
  return prior;
}

InitialStatePrior FastLioPipeline::makeFixedPrior(const Timestamp& sample_time) const {
  InitialStatePrior prior = config_.initial_prior.fixed_prior;
  prior.sample_time = sample_time;
  prior.source = InitialStatePriorSource::kFixed;
  prior.context = config_.initial_prior.context;
  prior.mask = config_.initial_prior.mask;
  return prior;
}

bool FastLioPipeline::resolveInitialStatePrior(const Timestamp& application_time) {
  const auto source = config_.initial_prior.source;
  diagnostics_.initial_prior.application_timestamp_ns = application_time.nanoseconds();
  diagnostics_.initial_prior.clock_domain =
      std::string(toString(application_time.clock_domain()));
  diagnostics_.initial_prior.waiting_for_sensor_time = false;
  if (source == InitialStatePriorSource::kZero || source == InitialStatePriorSource::kFixed) {
    diagnostics_.initial_prior.status = InitialPriorStatus::kApplied;
    if (source == InitialStatePriorSource::kFixed) {
      diagnostics_.initial_prior.reason = "FIXED_PRIOR_ACCEPTED";
    } else {
      diagnostics_.initial_prior.reason = "ZERO_PRIOR_ACCEPTED";
    }
    initial_prior_applied_ = true;
    return true;
  }

  std::optional<InitialStatePrior> candidate;
  {
    std::scoped_lock lock(initial_prior_mutex_);
    candidate = initial_prior_candidate_;
  }
  if (candidate.has_value()) {
    diagnostics_.initial_prior.candidate_timestamp_ns = candidate->sample_time.nanoseconds();
    diagnostics_.initial_prior.time_delta_ns =
        application_time.nanoseconds() - candidate->sample_time.nanoseconds();
    if (!candidate->sample_time.sameClockDomain(application_time)) {
      std::scoped_lock lock(initial_prior_mutex_);
      ++prior_timestamp_rejected_count_;
      ++prior_rejected_count_;
      initial_prior_candidate_.reset();
      diagnostics_.initial_prior.reason = "TOPIC_PRIOR_CLOCK_DOMAIN_MISMATCH";
    } else if (candidate->sample_time.nanoseconds() > application_time.nanoseconds()) {
      diagnostics_.initial_prior.waiting_for_sensor_time = true;
      diagnostics_.initial_prior.status = InitialPriorStatus::kWaiting;
      diagnostics_.initial_prior.reason = "TOPIC_PRIOR_WAITING_FOR_SENSOR_TIME";
    } else {
      const std::int64_t age = application_time.nanoseconds() - candidate->sample_time.nanoseconds();
      diagnostics_.initial_prior.candidate_age_ns = age;
      diagnostics_.initial_prior.candidate_timestamp_ns = candidate->sample_time.nanoseconds();
      if (age <= config_.initial_prior.maximum_topic_prior_age_ns) {
        diagnostics_.initial_prior.status = InitialPriorStatus::kApplied;
        diagnostics_.initial_prior.reason = "TOPIC_PRIOR_ACCEPTED";
        initial_prior_applied_ = true;
        return true;
      }
      std::scoped_lock lock(initial_prior_mutex_);
      ++prior_stale_rejected_count_;
      ++prior_rejected_count_;
      initial_prior_candidate_.reset();
      diagnostics_.initial_prior.reason = "TOPIC_PRIOR_STALE";
    }
  }

  diagnostics_.initial_prior.status = InitialPriorStatus::kWaiting;
  const Timestamp wait_start = prior_wait_start_time_.value_or(application_time);
  if (!wait_start.sameClockDomain(application_time)) {
    diagnostics_.initial_prior.status = InitialPriorStatus::kRejected;
    diagnostics_.initial_prior.reason = "TOPIC_PRIOR_WAIT_CLOCK_DOMAIN_MISMATCH";
    return false;
  }
  const std::int64_t elapsed = application_time.nanoseconds() - wait_start.nanoseconds();
  if (elapsed < config_.initial_prior.topic_wait_timeout_ns) {
    return false;
  }
  ++diagnostics_.initial_prior.wait_timeout_count;
  if (candidate.has_value() &&
      candidate->sample_time.sameClockDomain(application_time) &&
      candidate->sample_time.nanoseconds() > application_time.nanoseconds()) {
    // A candidate in the same clock domain can become applicable as the
    // sensor epoch advances.  The wall-clock-independent timeout must not
    // discard it or trigger fallback while that catch-up is still possible.
    diagnostics_.initial_prior.status = InitialPriorStatus::kWaiting;
    diagnostics_.initial_prior.waiting_for_sensor_time = true;
    diagnostics_.initial_prior.reason = "TOPIC_PRIOR_WAITING_FOR_SENSOR_TIME";
    return false;
  }
  if (config_.initial_prior.context == InitialStatePriorContext::kInFlightReinitialization ||
      config_.initial_prior.ground_fallback == InitialPriorFallback::kReject) {
    diagnostics_.initial_prior.status = InitialPriorStatus::kRejected;
    diagnostics_.initial_prior.reason = "TOPIC_PRIOR_TIMEOUT_REJECTED";
    return false;
  }
  initial_prior_fallback_applied_ = true;
  diagnostics_.initial_prior.status = InitialPriorStatus::kFallbackApplied;
  if (config_.initial_prior.ground_fallback == InitialPriorFallback::kFixed) {
    ++diagnostics_.initial_prior.fixed_fallback_count;
    diagnostics_.initial_prior.reason = "TOPIC_PRIOR_TIMEOUT_FIXED_FALLBACK";
  } else {
    ++diagnostics_.initial_prior.zero_fallback_count;
    diagnostics_.initial_prior.reason = "TOPIC_PRIOR_TIMEOUT_ZERO_FALLBACK";
  }
  return true;
}

void FastLioPipeline::transitionTo(EstimatorStatus next, std::string reason) {
  if (status_ != next) {
    diagnostics_.previous_status = status_;
    status_ = next;
    diagnostics_.status = status_;
    ++diagnostics_.status_transition_count;
  }
  diagnostics_.reason = std::move(reason);
}

ProcessResult FastLioPipeline::makeBaseResult(const MeasurementGroup& group) const {
  ProcessResult result;
  result.status_before = status_;
  result.status_after = status_;
  result.scan_time = group.scan.end_time;
  result.diagnostics = diagnostics_;
  result.diagnostics.synchronization.scan_start_ns = group.scan.start_time.nanoseconds();
  result.diagnostics.synchronization.scan_end_ns = group.scan.end_time.nanoseconds();
  const auto scan_duration = checkedDifference(group.scan.end_time, group.scan.start_time);
  result.diagnostics.synchronization.scan_duration_ns =
      scan_duration.ok() ? scan_duration.value().nanoseconds() : 0;
  result.diagnostics.synchronization.imu_samples_per_scan = group.imu_samples.size();
  result.diagnostics.synchronization.imu_gap_max_ns = group.max_imu_gap_ns;
  result.diagnostics.synchronization.has_start_bracket = group.has_start_bracket;
  result.diagnostics.synchronization.has_end_bracket = group.has_end_bracket;
  return result;
}

ProcessResult FastLioPipeline::finalizeResult(ProcessResult result) {
  result.status_after = status_;
  result.last_lidar_correction_time = last_correction_time_;
  fillStateDiagnostics(diagnostics_);
  copyInitialPriorMailboxDiagnostics(diagnostics_);

  OutputDiagnostics output;
  output.estimate_validity = result.estimate_validity;
  output.lidar_update_status = result.lidar_update_status;
  output.predicted_estimate_valid = result.hasPredictedOutput();
  output.corrected_estimate_valid = result.hasCorrectedOutput();
  output.registered_scan_valid = result.hasRegisteredScanOutput();
  if (result.corrected_estimate.has_value()) {
    output.output_time_ns = result.corrected_estimate->time.nanoseconds();
    output.clock_domain = std::string(toString(result.corrected_estimate->time.clock_domain()));
  } else if (result.predicted_estimate.has_value()) {
    output.output_time_ns = result.predicted_estimate->time.nanoseconds();
    output.clock_domain = std::string(toString(result.predicted_estimate->time.clock_domain()));
  } else if (result.scan_time.has_value()) {
    output.clock_domain = std::string(toString(result.scan_time->clock_domain()));
  }
  if (last_correction_time_.has_value()) {
    output.last_lidar_correction_time_ns = last_correction_time_->nanoseconds();
  }
  diagnostics_.output = std::move(output);
  result.diagnostics = diagnostics_;
  return result;
}

void FastLioPipeline::resetTransientDiagnostics() {
  diagnostics_.reason.clear();
  diagnostics_.synchronization = SynchronizationDiagnostics{};
  diagnostics_.prediction = PredictionDiagnostics{};

  const DeskewMode deskew_mode = diagnostics_.deskew.deskew_mode;
  diagnostics_.deskew = DeskewDiagnostics{};
  diagnostics_.deskew.deskew_mode = deskew_mode;

  diagnostics_.registration = RegistrationDiagnostics{};

  const std::size_t map_point_count = registration_map_.size();
  diagnostics_.map = MapDiagnostics{};
  diagnostics_.map.map_point_count = map_point_count;
  diagnostics_.map.valid_point_count_busy_count =
      registration_map_.validPointCountBusyCount();

  diagnostics_.state.correction_translation_norm_m = 0.0;
  diagnostics_.state.correction_rotation_norm_rad = 0.0;
  diagnostics_.output = OutputDiagnostics{};
  diagnostics_.timing = StageTimingDiagnostics{};
}

void FastLioPipeline::fillStateDiagnostics(EstimatorDiagnostics& diagnostics) const {
  diagnostics.status = status_;
  diagnostics.lio_generation = generation_;
  diagnostics.lio_generation_locked = tracking_ever_confirmed_ &&
                                       status_ == EstimatorStatus::kTracking;
  diagnostics.consecutive_uncorrected_lidar_updates =
      consecutive_uncorrected_lidar_updates_;
  diagnostics.consecutive_recovery_successes =
      consecutive_recovery_successes_;
  diagnostics.recovery_confirmation_updates_required =
      config_.tracking.recovery_confirmation_updates;
  diagnostics.map_insertion_frozen =
      tracking_ever_confirmed_ && status_ != EstimatorStatus::kTracking;
  diagnostics.navigation_valid =
      tracking_ever_confirmed_ && status_ == EstimatorStatus::kTracking;
  diagnostics.state.position_odom_imu_m = state_.position_odom_imu_m();
  diagnostics.state.velocity_odom_imu_m_s = state_.velocity_odom_imu_m_s();
  diagnostics.state.gyro_bias_rad_s = state_.gyro_bias_rad_s();
  diagnostics.state.accel_bias_m_s2 = state_.accel_bias_m_s2();
  diagnostics.state.gravity_odom_m_s2 = state_.gravity_odom_m_s2();
  diagnostics.state.covariance_trace = covariance_.trace();
  diagnostics.state.covariance_maximum_asymmetry =
      (covariance_ - covariance_.transpose()).cwiseAbs().maxCoeff();
  const ManifoldState::Covariance symmetric =
      0.5 * (covariance_ + covariance_.transpose());
  Eigen::SelfAdjointEigenSolver<ManifoldState::Covariance> solver(
      symmetric, Eigen::EigenvaluesOnly);
  diagnostics.state.covariance_minimum_eigenvalue =
      solver.info() == Eigen::Success
          ? solver.eigenvalues().minCoeff()
          : -std::numeric_limits<double>::infinity();
  if (last_correction_time_.has_value() && state_time_.has_value() &&
      last_correction_time_->sameClockDomain(*state_time_)) {
    diagnostics.state.last_lidar_correction_age_ns =
        state_time_->nanoseconds() - last_correction_time_->nanoseconds();
  } else {
    diagnostics.state.last_lidar_correction_age_ns = -1;
  }
}

void FastLioPipeline::updateInitializationDiagnostics() {
  const InitializationQuality quality = initializer_.quality();
  diagnostics_.initialization.samples_collected = quality.samples_collected;
  diagnostics_.initialization.gyro_mean_rad_s = quality.gyro_mean_rad_s;
  diagnostics_.initialization.gyro_variance_rad2_s2 = quality.gyro_variance_rad2_s2;
  diagnostics_.initialization.accel_mean_m_s2 = quality.accel_mean_m_s2;
  diagnostics_.initialization.accel_variance_m2_s4 = quality.accel_variance_m2_s4;
  diagnostics_.initialization.gravity_norm_m_s2 = quality.measured_gravity_norm_m_s2;
  if (quality.samples_collected < config_.initialization.minimum_imu_samples) {
    diagnostics_.initialization.initialization_status = "COLLECTING_IMU";
  } else if (!quality.stationary && config_.initialization.require_stationary) {
    diagnostics_.initialization.initialization_status = "STATIONARITY_GATE_REJECTED";
  } else {
    diagnostics_.initialization.initialization_status = "READY";
  }
}

void FastLioPipeline::recordUncorrectedUpdate(
    LidarUpdateFailureClass failure_class) {
  diagnostics_.last_update_failure_class = failure_class;
  if (!tracking_ever_confirmed_) {
    return;
  }
  ++consecutive_uncorrected_lidar_updates_;
  consecutive_recovery_successes_ = 0U;
  if (consecutive_uncorrected_lidar_updates_ >=
      config_.lifecycle.lost_after_registration_failures) {
    transitionTo(EstimatorStatus::kLost, "LIDAR_UPDATE_FAILURE_LIMIT_REACHED");
  } else if (status_ != EstimatorStatus::kLost) {
    transitionTo(EstimatorStatus::kDegraded, "LIDAR_UPDATE_UNCORRECTED");
  }
}

}  // namespace uav::nav::lio
