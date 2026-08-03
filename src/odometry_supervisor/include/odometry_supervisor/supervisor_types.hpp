#pragma once

#include <cstdint>
#include <string>

#include <Eigen/Geometry>

namespace odometry_supervisor {

enum class HealthState : std::uint8_t { kStartup = 0, kHealthy, kSuspect, kDegraded, kDiverged };
enum class ReferenceMode : std::uint8_t { kIndependent = 0, kCorrelated = 1 };

enum class ReasonCode : std::uint16_t {
  kNone = 0,
  kStartup = 1,
  kHealthy = 2,
  kMonitoringUnavailable = 3,
  kOriginNotAligned = 4,
  kAlignmentFailed = 5,
  kResidualSuspect = 6,
  kResidualDegraded = 7,
  kResidualDiverged = 8,
  kLioLost = 9,
  kStateCorruption = 10,
  kLioResetting = 11,
  kPx4ResetGrace = 12,
  kDiagnosticSchemaMismatch = 13,
  kStaleInput = 14,
  kLioDiagnosticsStale = 15,
  kLioDiagnosticSchemaMismatch = 16,
  kPx4DiagnosticsStale = 17,
  kPx4DiagnosticSchemaMismatch = 18,
  kAlignedComparisonStale = 19,
  kQueryGenerationMismatch = 20,
  kQueryInvalidComponent = 21,
};

struct OdometryState {
  std::int64_t timestamp_ns{0};
  Eigen::Vector3d position_odom{Eigen::Vector3d::Zero()};
  Eigen::Quaterniond orientation_odom_base{Eigen::Quaterniond::Identity()};
  Eigen::Vector3d velocity_base{Eigen::Vector3d::Zero()};
  bool valid{false};
  std::string frame_id;
  std::string child_frame_id;
};

struct Residual {
  bool valid{false};
  bool heading_observable{false};
  std::int64_t timestamp_ns{0};
  double position_error_m{0.0};
  double velocity_error_m_s{0.0};
  double orientation_error_rad{0.0};
  double yaw_error_rad{0.0};
  double euler_yaw_error_rad{0.0};
  double robust_heading_lio_rad{0.0};
  double robust_heading_px4_rad{0.0};
  Eigen::Vector3d q_error_axis{Eigen::Vector3d::Zero()};
  double body_z_dot{1.0};
  double body_x_horizontal_norm_lio{0.0};
  double body_x_horizontal_norm_px4{0.0};
  double position_error_growth_m_s{0.0};
};

struct AlignedComparison {
  OdometryState lio;
  OdometryState px4;
  Residual residual;
  std::int64_t comparison_epoch_ns{0};
  std::int64_t response_received_ros_time_ns{0};
  std::uint64_t query_sequence{0};
  std::uint64_t reset_generation{0};
  std::uint64_t time_generation{0};
  std::uint32_t component_validity_mask{0};
  std::uint32_t covariance_availability_mask{0};
  bool interpolated{false};
};

struct ResidualThresholds {
  double position_m{0.0};
  double velocity_m_s{0.0};
  double orientation_rad{0.0};
  double yaw_rad{0.0};
};

struct SupervisorConfig {
  ReferenceMode reference_mode{ReferenceMode::kIndependent};
  double evaluation_rate_hz{20.0};
  std::int64_t propagated_max_age_ns{120'000'000};
  std::int64_t corrected_max_age_ns{350'000'000};
  std::int64_t px4_max_age_ns{250'000'000};
  std::int64_t diagnostics_max_age_ns{500'000'000};
  std::int64_t maximum_alignment_gap_ns{50'000'000};
  std::int64_t service_timeout_ns{100'000'000};
  std::int64_t maximum_comparison_age_ns{150'000'000};
  ResidualThresholds suspect{0.30, 0.30, 0.2094395102, 0.1396263402};
  ResidualThresholds degraded{0.75, 0.75, 0.5235987756, 0.3490658504};
  ResidualThresholds diverged{1.50, 1.50, 1.0471975512, 0.6981317008};
  double clear_ratio{0.70};
  std::int64_t suspect_enter_ns{300'000'000};
  std::int64_t degraded_enter_ns{750'000'000};
  std::int64_t diverged_enter_ns{1'500'000'000};
  std::int64_t recovery_confirm_ns{2'000'000'000};
  std::int64_t reset_grace_ns{500'000'000};
  std::int64_t lio_diagnostics_invalid_enter_ns{500'000'000};
  float suspect_speed_limit_m_s{2.0F};
  float degraded_speed_limit_m_s{0.5F};
};

struct EvaluationInput {
  std::int64_t evaluation_time_ns{0};
  bool lio_valid{false};
  bool lio_lost{false};
  bool lio_state_corruption{false};
  bool lio_degraded{false};
  bool lio_resetting{false};
  bool propagated_fresh{false};
  bool corrected_fresh{false};
  bool px4_available{false};
  bool px4_fresh{false};
  bool px4_continuity_valid{false};
  bool px4_post_reset_stable{false};
  bool origin_aligned{false};
  bool lio_diagnostics_valid{false};
  bool px4_diagnostics_valid{false};
  bool lio_diagnostics_schema_valid{false};
  bool px4_diagnostics_schema_valid{false};
  bool lio_diagnostics_stale{false};
  bool px4_diagnostics_stale{false};
  bool time_generation_changed{false};
  std::uint64_t px4_reset_generation{0};
  std::uint64_t px4_time_generation{0};
  std::int64_t propagated_age_ns{-1};
  std::int64_t corrected_age_ns{-1};
  std::int64_t px4_age_ns{-1};
  std::int64_t alignment_gap_ns{-1};
  std::int64_t aligned_comparison_age_ns{-1};
  std::int64_t latest_eligible_epoch_ns{0};
  std::int64_t comparison_lag_to_latest_eligible_ns{-1};
  std::int64_t pending_query_epoch_ns{0};
  std::int64_t pending_query_age_ns{-1};
  std::int64_t comparison_epoch_ns{0};
  bool new_comparison_sample{false};
  bool aligned_comparison_fresh{false};
  std::uint64_t query_sequence{0};
  std::uint32_t component_validity_mask{0};
  std::uint32_t covariance_availability_mask{0};
  std::uint64_t query_invalid_component_count{0};
  std::uint64_t query_generation_mismatch_count{0};
  std::uint64_t query_stale_sequence_count{0};
  std::uint64_t query_timeout_count{0};
  std::uint64_t query_service_unavailable_count{0};
  std::uint64_t query_success_count{0};
  std::uint64_t query_failure_count{0};
  std::uint64_t query_rtt_count{0};
  double query_rtt_p50_ms{0.0};
  double query_rtt_p95_ms{0.0};
  double query_rtt_p99_ms{0.0};
  double query_rtt_max_ms{0.0};
  std::uint64_t stale_residual_reuse_count{0};
  Residual residual;
};

struct SupervisorOutput {
  HealthState health{HealthState::kStartup};
  ReferenceMode reference_mode{ReferenceMode::kIndependent};
  std::uint16_t reason_code{1};
  std::string reason{"STARTUP"};
  bool monitoring_available{false};
  bool comparison_valid{false};
  bool lio_valid{false};
  bool px4_valid{false};
  bool time_aligned{false};
  bool external_odometry_allowed{false};
  bool reinitialization_requested{false};
  std::uint64_t reinitialization_request_sequence{0};
  bool planner_speed_limit_active{false};
  float planner_speed_limit_m_s{0.0F};
  bool hover_or_failsafe_requested{false};
  std::int64_t evaluation_time_ns{0};
  std::int64_t lio_propagated_age_ns{-1};
  std::int64_t lio_corrected_age_ns{-1};
  std::int64_t px4_age_ns{-1};
  std::int64_t alignment_gap_ns{-1};
  std::int64_t aligned_comparison_age_ns{-1};
  std::int64_t latest_eligible_epoch_ns{0};
  std::int64_t comparison_lag_to_latest_eligible_ns{-1};
  std::int64_t pending_query_epoch_ns{0};
  std::int64_t pending_query_age_ns{-1};
  Residual residual;
  std::uint64_t px4_reset_generation{0};
  std::uint64_t px4_time_generation{0};
  std::uint64_t state_transition_count{0};
  std::uint64_t evaluation_count{0};
  std::uint64_t alignment_failure_count{0};
  std::int64_t comparison_epoch_ns{0};
  bool new_comparison_sample{false};
  bool aligned_comparison_fresh{false};
  bool lio_diagnostics_valid{false};
  bool px4_diagnostics_valid{false};
  bool lio_diagnostics_schema_valid{false};
  bool px4_diagnostics_schema_valid{false};
  bool lio_diagnostics_stale{false};
  bool px4_diagnostics_stale{false};
  std::uint64_t query_sequence{0};
  std::uint32_t component_validity_mask{0};
  std::uint32_t covariance_availability_mask{0};
  std::uint64_t query_invalid_component_count{0};
  std::uint64_t query_generation_mismatch_count{0};
  std::uint64_t query_stale_sequence_count{0};
  std::uint64_t query_timeout_count{0};
  std::uint64_t query_service_unavailable_count{0};
  std::uint64_t query_success_count{0};
  std::uint64_t query_failure_count{0};
  std::uint64_t query_rtt_count{0};
  double query_rtt_p50_ms{0.0};
  double query_rtt_p95_ms{0.0};
  double query_rtt_p99_ms{0.0};
  double query_rtt_max_ms{0.0};
  std::uint64_t stale_residual_reuse_count{0};
};

}  // namespace odometry_supervisor
