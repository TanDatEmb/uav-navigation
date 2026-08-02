#pragma once

#include <cstddef>
#include <cstdint>
#include <atomic>
#include <optional>
#include <mutex>
#include <string>
#include <vector>

#include "fast_lio_core/common/status.hpp"
#include "fast_lio_core/configuration/estimator_config.hpp"
#include "fast_lio_core/estimation/ikfom_estimator.hpp"
#include "fast_lio_core/estimation/manifold_state.hpp"
#include "fast_lio_core/initialization/initialization_result.hpp"
#include "fast_lio_core/initialization/initial_state_prior.hpp"
#include "fast_lio_core/initialization/initial_state_prior_applicator.hpp"
#include "fast_lio_core/geometry/rigid_transform.hpp"
#include "fast_lio_core/mapping/ikd_tree_registration_map.hpp"
#include "fast_lio_core/mapping/dynamic_map_evidence.hpp"
#include "fast_lio_core/navigation/angular_velocity_resolver.hpp"
#include "fast_lio_core/pipeline/estimator_diagnostics.hpp"
#include "fast_lio_core/pipeline/process_result.hpp"
#include "fast_lio_core/sensor/imu_sample.hpp"
#include "fast_lio_core/sensor/lidar_scan.hpp"
#include "fast_lio_core/sensor/measurement_group.hpp"
#include "fast_lio_core/synchronization/measurement_buffer.hpp"

namespace uav::nav::lio {

class FastLioPipeline {
 public:
  explicit FastLioPipeline(EstimatorConfig config = {});

  [[nodiscard]] Status pushImu(const ImuSample& sample);
  [[nodiscard]] Status pushLidar(LidarScan scan);
  [[nodiscard]] Status submitInitialStatePrior(InitialStatePrior prior);
  [[nodiscard]] Status setInitialStatePriorGeometry(RigidTransform base_to_imu);

  // Returns nullopt while waiting for a complete synchronized group. Permanent
  // synchronization rejection is returned as a ProcessResult with no output.
  [[nodiscard]] std::optional<ProcessResult> processNext();

  // Direct path for offline evaluation and deterministic unit tests. It uses
  // exactly the same estimator/map state as processNext().
  [[nodiscard]] ProcessResult process(const MeasurementGroup& group);

  void reset();
  [[nodiscard]] EstimatorStatus status() const noexcept;
  [[nodiscard]] const ManifoldState& state() const noexcept;
  [[nodiscard]] const ManifoldState::Covariance& covariance() const noexcept;
  [[nodiscard]] EstimatorDiagnostics diagnostics() const;
  [[nodiscard]] std::vector<Eigen::Vector3d> registrationMapSnapshot() const;
  [[nodiscard]] const std::optional<Timestamp>& stateTime() const noexcept;
  [[nodiscard]] const std::optional<Timestamp>& synchronizationEpoch()
      const noexcept;
  [[nodiscard]] std::size_t pendingLidarCount() const;

 private:
  [[nodiscard]] Status addInitializationSample(const ImuSample& sample);
  [[nodiscard]] ProcessResult processInternal(const MeasurementGroup& group,
                                              bool imu_samples_already_ingested);
  void tryCompleteImuInitialization(
      const std::optional<Timestamp>& progress_time = std::nullopt);
  [[nodiscard]] bool resolveInitialStatePrior(const Timestamp& application_time);
  [[nodiscard]] InitialStatePrior makeZeroPrior(const Timestamp& sample_time) const;
  [[nodiscard]] InitialStatePrior makeFixedPrior(const Timestamp& sample_time) const;
  void copyInitialPriorMailboxDiagnostics(EstimatorDiagnostics& output) const;
  void transitionTo(EstimatorStatus next, std::string reason);
  [[nodiscard]] ProcessResult makeBaseResult(const MeasurementGroup& group) const;
  [[nodiscard]] ProcessResult finalizeResult(ProcessResult result);
  void resetTransientDiagnostics();
  void fillStateDiagnostics(EstimatorDiagnostics& diagnostics) const;
  void updateInitializationDiagnostics();
  void recordUncorrectedUpdate(LidarUpdateFailureClass failure_class);
  [[nodiscard]] ProcessResult recoverFromDiscontinuity(
      const PropagationDiscontinuity& discontinuity);

  EstimatorConfig config_;
  MeasurementBuffer buffer_;
  MeasurementSynchronizer synchronizer_;
  ImuInitializer initializer_;
  IkfomEstimator estimator_;
  ScanDeskewer deskewer_;
  PointCloudPreprocessor preprocessor_;
  IkdTreeRegistrationMap registration_map_;
  IkdTreeRegistrationMap bootstrap_map_;
  LocalMapManager local_map_manager_;
  MapInsertionPolicy insertion_policy_;
  DynamicMapEvidence dynamic_map_evidence_;

  EstimatorStatus status_{EstimatorStatus::kWaitingForSensors};
  ManifoldState state_{};
  ManifoldState::Covariance covariance_{ManifoldState::Covariance::Identity()};
  std::optional<Timestamp> state_time_;
  std::optional<Timestamp> last_initialization_imu_time_;
  std::optional<Timestamp> last_correction_time_;
  std::vector<Eigen::Vector3d> bootstrap_reference_points_odom_m_;
  EstimatorDiagnostics diagnostics_{};
  InitialStatePriorApplicator initial_prior_applicator_;
  mutable std::mutex initial_prior_mutex_;
  std::optional<InitialStatePrior> initial_prior_candidate_;
  std::optional<Timestamp> last_prior_candidate_time_;
  std::size_t prior_candidate_count_{0};
  std::size_t prior_accepted_count_{0};
  std::size_t prior_rejected_count_{0};
  std::size_t prior_late_rejected_count_{0};
  std::size_t prior_stale_rejected_count_{0};
  std::size_t prior_future_rejected_count_{0};
  std::size_t prior_frame_rejected_count_{0};
  std::size_t prior_timestamp_rejected_count_{0};
  std::size_t prior_invalid_value_count_{0};
  std::optional<InitializationResult> imu_initialization_result_;
  std::optional<Timestamp> prior_wait_start_time_;
  std::atomic<bool> estimator_initialized_{false};
  std::atomic<bool> initial_prior_gate_closed_{false};
  bool initial_prior_applied_{false};
  bool initial_prior_fallback_applied_{false};
  std::size_t consecutive_registration_failures_{0};
  std::size_t initial_map_registration_failures_{0};
  std::size_t corrected_scan_count_{0};
  std::size_t consecutive_uncorrected_lidar_updates_{0};
  std::size_t consecutive_recovery_successes_{0};
  bool tracking_ever_confirmed_{false};
};

}  // namespace uav::nav::lio
