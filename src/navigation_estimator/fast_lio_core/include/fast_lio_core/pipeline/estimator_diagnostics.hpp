#pragma once

#include <Eigen/Core>
#include <cstddef>
#include <cstdint>
#include <string>

#include "fast_lio_core/deskew/deskew_mode.hpp"
#include "fast_lio_core/deskew/deskew_result.hpp"
#include "fast_lio_core/navigation/angular_velocity_resolver.hpp"
#include "fast_lio_core/initialization/initial_state_prior.hpp"

namespace uav::nav::lio {

enum class EstimatorStatus {
  kWaitingForSensors,
  kCollectingImu,
  kInitializingImu,
  kInitializingMap,
  kTracking,
  kDegraded,
  kLost,
  kResetting,
};

[[nodiscard]] const char* toString(EstimatorStatus status) noexcept;

enum class EstimateValidity {
  kUnavailable,
  kPredictedOnly,
  kCorrected,
};

[[nodiscard]] const char* toString(EstimateValidity validity) noexcept;

enum class LidarUpdateStatus {
  kNotAttempted,
  kSucceeded,
  kRejected,
};

[[nodiscard]] const char* toString(LidarUpdateStatus status) noexcept;

enum class LidarUpdateFailureClass {
  kNone,
  kSynchronization,
  kPrediction,
  kDeskew,
  kPreprocessing,
  kInsufficientPoints,
  kRegistration,
  kPropagationDiscontinuity,
  kStateCorruption,
};

[[nodiscard]] const char* toString(
    LidarUpdateFailureClass failure_class) noexcept;

struct SensorDiagnostics {
  std::size_t ros_received_imu_count{0};
  std::size_t ros_received_lidar_count{0};
  std::size_t core_accepted_imu_count{0};
  std::size_t core_accepted_lidar_count{0};
  std::size_t lidar_drop_count{0};
  std::size_t imu_drop_count{0};
  std::size_t timestamp_regression_count{0};
  std::int64_t ros_maximum_imu_gap_ns{0};
  std::size_t imu_subscription_sequence_gap_count{0};
  std::size_t lidar_subscription_sequence_gap_count{0};
  std::size_t imu_queue_high_water_mark{0};
  std::size_t lidar_queue_high_water_mark{0};
  std::size_t processing_queue_high_water_mark{0};
  std::size_t invalid_point_count{0};
};

struct SynchronizationDiagnostics {
  bool synchronized{false};
  std::int64_t scan_start_ns{0};
  std::int64_t scan_end_ns{0};
  std::int64_t scan_duration_ns{0};
  std::size_t imu_samples_per_scan{0};
  std::int64_t imu_gap_max_ns{0};
  bool has_start_bracket{false};
  bool has_end_bracket{false};
  std::string sync_rejection_reason;
};

struct ProcessingStatistics {
  std::size_t raw_lidar_count{0};
  std::size_t buffer_accepted_lidar_count{0};
  std::size_t overlap_rejected_count{0};
  std::size_t missing_bracket_rejected_count{0};
  std::size_t invalid_timestamp_rejected_count{0};
  std::size_t synchronized_group_count{0};
  std::size_t correction_attempt_count{0};
  std::size_t correction_success_count{0};
  std::size_t correction_failure_count{0};

  [[nodiscard]] double bufferAcceptanceRatio() const noexcept {
    return raw_lidar_count == 0
               ? 0.0
               : static_cast<double>(buffer_accepted_lidar_count) /
                     static_cast<double>(raw_lidar_count);
  }
  [[nodiscard]] double synchronizationRatio() const noexcept {
    return buffer_accepted_lidar_count == 0
               ? 0.0
               : static_cast<double>(synchronized_group_count) /
                     static_cast<double>(buffer_accepted_lidar_count);
  }
  [[nodiscard]] double correctionSuccessRatio() const noexcept {
    return correction_attempt_count == 0
               ? 0.0
               : static_cast<double>(correction_success_count) /
                     static_cast<double>(correction_attempt_count);
  }
};

struct InitializationDiagnostics {
  std::size_t samples_collected{0};
  Eigen::Vector3d gyro_mean_rad_s{Eigen::Vector3d::Zero()};
  Eigen::Vector3d gyro_variance_rad2_s2{Eigen::Vector3d::Zero()};
  Eigen::Vector3d accel_mean_m_s2{Eigen::Vector3d::Zero()};
  Eigen::Vector3d accel_variance_m2_s4{Eigen::Vector3d::Zero()};
  double gravity_norm_m_s2{0.0};
  std::string initialization_status;
};

struct InitialPriorDiagnostics {
  InitialPriorStatus status{InitialPriorStatus::kNotRequired};
  InitialStatePriorSource source{InitialStatePriorSource::kZero};
  InitialStatePriorContext context{InitialStatePriorContext::kGroundStartup};
  PriorAttitudeMode attitude_mode{PriorAttitudeMode::kNone};
  bool position_enabled{false};
  bool velocity_enabled{false};
  bool applied{false};
  bool fallback_applied{false};
  bool covariance_applied{false};
  bool propagated_to_application{false};
  std::uint64_t generation{0};
  bool bootstrap_map_after_prior{false};
  std::int64_t candidate_timestamp_ns{0};
  std::int64_t state_time_ns{0};
  std::int64_t application_timestamp_ns{0};
  std::int64_t candidate_age_ns{0};
  std::int64_t time_delta_ns{0};
  std::string clock_domain{"unknown"};
  bool waiting_for_sensor_time{false};
  std::size_t candidate_count{0};
  std::size_t accepted_count{0};
  std::size_t rejected_count{0};
  std::size_t late_rejected_count{0};
  std::size_t stale_rejected_count{0};
  std::size_t future_rejected_count{0};
  std::size_t frame_rejected_count{0};
  std::size_t timestamp_rejected_count{0};
  std::size_t invalid_value_count{0};
  std::size_t wait_timeout_count{0};
  std::size_t zero_fallback_count{0};
  std::size_t fixed_fallback_count{0};
  std::string reason;
};

struct DeskewDiagnostics {
  DeskewMode deskew_mode{DeskewMode::kPerPoint};
  DeskewStatus deskew_status{DeskewStatus::kBypassedSimultaneousScan};
  bool deskew_attempted{false};
  bool deskew_applied{false};
  std::uint32_t point_time_min_ns{0};
  std::uint32_t point_time_max_ns{0};
  std::size_t interpolation_failure_count{0};
  std::int64_t deskew_runtime_us{0};
};

struct RegistrationDiagnostics {
  bool correction_attempted{false};
  bool correction_succeeded{false};
  std::size_t input_point_count{0};
  std::size_t filtered_point_count{0};
  std::size_t query_count{0};
  std::size_t valid_plane_count{0};
  std::size_t accepted_residual_count{0};
  std::size_t rejected_residual_count{0};
  std::size_t measurement_callback_count{0};
  double residual_rms_m{0.0};
  std::size_t iteration_count{0};
  double final_increment_norm{0.0};
  bool converged{false};
};

struct MapDiagnostics {
  bool map_update_performed{false};
  std::size_t map_point_count{0};
  std::size_t valid_point_count_busy_count{0};
  std::size_t inserted_point_count{0};
  std::size_t removed_point_count{0};
  std::size_t map_size_before_insert{0};
  std::size_t map_candidate_count{0};
  std::size_t map_inserted_count{0};
  std::size_t map_size_after_insert{0};
  bool crop_performed{false};
  std::size_t crop_removed_count{0};
  bool crop_triggered_by_motion{false};
  bool absolute_guard_triggered{false};
  bool absolute_guard_recovery_failed{false};
  bool map_insertion_frozen{false};
  std::size_t map_count_before{0};
  std::size_t map_count_after_crop{0};
  std::size_t map_size_before_maintenance{0};
  std::size_t map_size_after_maintenance{0};
  Eigen::Vector3d local_map_center_odom_m{Eigen::Vector3d::Zero()};
  Eigen::Vector3d local_map_half_extent_m{Eigen::Vector3d::Zero()};
  std::int64_t map_update_runtime_us{0};
  std::int64_t map_maintenance_us{0};
  bool dynamic_filter_enabled{false};
  std::size_t dynamic_evidence_voxel_count{0};
  std::size_t dynamic_candidate_count{0};
};

struct StateDiagnostics {
  Eigen::Vector3d position_odom_imu_m{Eigen::Vector3d::Zero()};
  Eigen::Vector3d velocity_odom_imu_m_s{Eigen::Vector3d::Zero()};
  Eigen::Vector3d gyro_bias_rad_s{Eigen::Vector3d::Zero()};
  Eigen::Vector3d accel_bias_m_s2{Eigen::Vector3d::Zero()};
  Eigen::Vector3d gravity_odom_m_s2{Eigen::Vector3d::Zero()};
  double covariance_trace{0.0};
  double covariance_minimum_eigenvalue{0.0};
  double covariance_maximum_asymmetry{0.0};
  std::int64_t last_lidar_correction_age_ns{-1};
  double correction_translation_norm_m{0.0};
  double correction_rotation_norm_rad{0.0};
};

struct OutputDiagnostics {
  EstimateValidity estimate_validity{EstimateValidity::kUnavailable};
  LidarUpdateStatus lidar_update_status{LidarUpdateStatus::kNotAttempted};
  bool predicted_estimate_valid{false};
  bool corrected_estimate_valid{false};
  bool registered_scan_valid{false};
  std::int64_t output_time_ns{-1};
  std::int64_t last_lidar_correction_time_ns{-1};
  std::string clock_domain;
};

struct StageTimingDiagnostics {
  std::int64_t synchronization_us{0};
  std::int64_t imu_prediction_us{0};
  std::int64_t deskew_us{0};
  std::int64_t preprocessing_us{0};
  std::int64_t residual_build_us{0};
  std::int64_t ikfom_update_us{0};
  std::int64_t measurement_model_us{0};
  std::int64_t nearest_search_us{0};
  std::int64_t plane_and_gate_us{0};
  std::int64_t jacobian_build_us{0};
  std::int64_t ikfom_solver_only_us{0};
  std::int64_t map_insert_crop_us{0};
  std::int64_t map_maintenance_us{0};
  std::int64_t map_prepare_us{0};
  std::int64_t map_sort_us{0};
  std::int64_t map_add_points_us{0};
  std::int64_t map_crop_us{0};
  std::int64_t map_bookkeeping_us{0};
  std::int64_t total_processing_us{0};
};

struct PredictionDiagnostics {
  bool attempted{false};
  bool successful{false};
  std::int64_t start_time_ns{0};
  std::int64_t end_time_ns{0};
  std::int64_t interval_ns{0};
  std::int64_t imu_first_time_ns{0};
  std::int64_t imu_last_time_ns{0};
  std::size_t imu_sample_count{0};
  std::size_t integration_interval_count{0};
  std::string rejection_reason;
};

struct EstimatorDiagnostics {
  EstimatorStatus status{EstimatorStatus::kWaitingForSensors};
  std::uint64_t lio_generation{1};
  bool lio_generation_locked{false};
  EstimatorStatus previous_status{EstimatorStatus::kWaitingForSensors};
  std::size_t status_transition_count{0};
  std::size_t consecutive_registration_failure_count{0};
  std::size_t propagation_discontinuity_count{0};
  std::int64_t last_propagation_gap_ns{0};
  std::size_t recovery_covariance_clamp_count{0};
  double recovery_covariance_maximum_eigenvalue_before_clamp{0.0};
  double recovery_covariance_maximum_eigenvalue_after_clamp{0.0};
  std::size_t consecutive_uncorrected_lidar_updates{0};
  std::size_t consecutive_recovery_successes{0};
  std::size_t recovery_confirmation_updates_required{0};
  bool map_insertion_frozen{false};
  bool navigation_valid{false};
  LidarUpdateFailureClass last_update_failure_class{
      LidarUpdateFailureClass::kNone};
  std::string reason;
  SensorDiagnostics sensor;
  SynchronizationDiagnostics synchronization;
  ProcessingStatistics processing;
  InitializationDiagnostics initialization;
  InitialPriorDiagnostics initial_prior;
  DeskewDiagnostics deskew;
  RegistrationDiagnostics registration;
  MapDiagnostics map;
  StateDiagnostics state;
  AngularVelocityDiagnostics corrected_angular_velocity;
  OutputDiagnostics output;
  PredictionDiagnostics prediction;
  StageTimingDiagnostics timing;
};

}  // namespace uav::nav::lio
