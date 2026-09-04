#pragma once

#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <limits>
#include <string>

#include <Eigen/Core>

#include <navigation_planning/candidate_bundle.hpp>
#include <navigation_world_model/world_model_view.hpp>

namespace navigation_planning {

struct OptimizationDiagnostics {
  int lbfgs_attempt_count{0};
  int lbfgs_evaluation_count{0};
  int lbfgs_first_attempt_evaluation_count{0};
  int lbfgs_last_attempt_evaluation_count{0};
  int retry_count{0};
  int retry_violation_mask{0};
  int retry_stop_reason{0};
  int first_lbfgs_return_code{-1};
  int last_lbfgs_return_code{-1};
  bool cancelled{false};
  bool valid{false};
  bool used_certified_seed{false};
  int certified_seed_failure_stage{0};
  int corridor_seed_build_failure_stage{0};
  int corridor_seed_retry_attempt_count{0};
  int corridor_seed_retry_build_valid_count{0};
  int corridor_seed_retry_last_certificate_stage{0};
  int corridor_seed_selected_mode{0};
  double corridor_seed_selected_max_duration_scale{
      std::numeric_limits<double>::quiet_NaN()};
  double initial_normalized_dynamic_violation{std::numeric_limits<double>::quiet_NaN()};
  double best_normalized_dynamic_violation{std::numeric_limits<double>::quiet_NaN()};
  double final_normalized_dynamic_violation{std::numeric_limits<double>::quiet_NaN()};
  double last_candidate_maximum_velocity_mps{std::numeric_limits<double>::quiet_NaN()};
  double last_candidate_maximum_acceleration_mps2{
      std::numeric_limits<double>::quiet_NaN()};
  double last_candidate_maximum_jerk_mps3{std::numeric_limits<double>::quiet_NaN()};
  double certified_seed_maximum_velocity_mps{std::numeric_limits<double>::quiet_NaN()};
  double certified_seed_maximum_acceleration_mps2{
      std::numeric_limits<double>::quiet_NaN()};
  double certified_seed_maximum_jerk_mps3{std::numeric_limits<double>::quiet_NaN()};
  double initial_duration_s{std::numeric_limits<double>::quiet_NaN()};
  double initial_minimum_piece_duration_s{std::numeric_limits<double>::quiet_NaN()};
  double initial_maximum_piece_duration_s{std::numeric_limits<double>::quiet_NaN()};
  double final_duration_s{std::numeric_limits<double>::quiet_NaN()};
  double retry_duration_lower_bound_min_s{std::numeric_limits<double>::quiet_NaN()};
  double retry_duration_lower_bound_max_s{std::numeric_limits<double>::quiet_NaN()};
  double retry_free_duration_seed_min_s{std::numeric_limits<double>::quiet_NaN()};
  double retry_free_duration_seed_max_s{std::numeric_limits<double>::quiet_NaN()};
  std::int64_t retry_budget_remaining_us{-1};
  int nonfinite_evaluation_count{0};
  int first_nonfinite_stage{0};
  int first_nonfinite_value_mask{0};
  int first_nonfinite_attempt{0};
  int first_nonfinite_iteration{0};
  double first_nonfinite_min_duration_s{std::numeric_limits<double>::quiet_NaN()};
  double first_nonfinite_max_duration_s{std::numeric_limits<double>::quiet_NaN()};
  double first_nonfinite_cost{std::numeric_limits<double>::quiet_NaN()};
  double first_nonfinite_gradient_norm{std::numeric_limits<double>::quiet_NaN()};
};

struct CommitDiagnostics {
  std::uint64_t generation{0};
  std::uint64_t previous_generation{0};
  double candidate_start_wall_time_s{0.0};
  Eigen::Vector3d candidate_start_position{Eigen::Vector3d::Zero()};
  Eigen::Vector3d candidate_start_velocity{Eigen::Vector3d::Zero()};
  Eigen::Vector3d candidate_start_acceleration{Eigen::Vector3d::Zero()};
  Eigen::Vector3d candidate_start_jerk{Eigen::Vector3d::Zero()};
  double candidate_start_yaw{0.0};
  double candidate_start_yaw_rate{0.0};
  bool previous_valid{false};
  double previous_sample_time_s{0.0};
  Eigen::Vector3d position_residual{Eigen::Vector3d::Zero()};
  Eigen::Vector3d velocity_residual{Eigen::Vector3d::Zero()};
  Eigen::Vector3d acceleration_residual{Eigen::Vector3d::Zero()};
  Eigen::Vector3d jerk_residual{Eigen::Vector3d::Zero()};
  double yaw_residual{0.0};
  double yaw_rate_residual{0.0};
};

struct WorldCommitCertificate {
  navigation_world_model::WorldSnapshotIdentity pinned_world{};
  navigation_world_model::WorldSnapshotIdentity validated_world{};
  double validation_begin_time_s{0.0};
};

struct TrajectorySnapshot {
  double start_wall_time_s{0.0};
  double duration_s{0.0};
  std::function<bool(double, TrajectoryPoint&)> evaluator;
  std::function<CandidateRole(double)> role_evaluator;

  [[nodiscard]] bool empty() const noexcept {
    return !std::isfinite(start_wall_time_s) || !std::isfinite(duration_s) ||
           duration_s < 0.0 || !static_cast<bool>(evaluator);
  }

  [[nodiscard]] bool sample(double trajectory_time_s, TrajectoryPoint& output) const {
    if (empty() || !std::isfinite(trajectory_time_s) || trajectory_time_s < 0.0 ||
        trajectory_time_s > duration_s) {
      return false;
    }
    if (!evaluator(trajectory_time_s, output) || !output.finite()) return false;
    if (role_evaluator) output.role = role_evaluator(trajectory_time_s);
    output.finished = trajectory_time_s >= duration_s;
    return output.finite();
  }
};

struct CommittedTrajectorySnapshot {
  TrajectorySnapshot position;
  std::uint64_t generation{0};
  WorldCommitCertificate certificate{};
  CommitDiagnostics diagnostics{};
  bool backup_available{false};
  double backup_start_time_s{0.0};
  bool terminal_stop{false};

  [[nodiscard]] bool empty() const noexcept {
    return position.empty();
  }
};

struct CommittedTrajectoryMetadata {
  std::uint64_t generation{0};
  CommitDiagnostics diagnostics{};
};

struct TrajectoryValidationResult {
  bool valid{false};
  // True only when immutable changed-region provenance proved that the
  // previous full certificate remains valid. This is observability for the
  // mapping fast path; it is never a certificate without that proof.
  bool reused_unchanged_certificate{false};
  navigation_world_model::WorldSnapshotIdentity pinned_world{};
  navigation_world_model::WorldSnapshotIdentity validated_world{};
  double begin_time_s{0.0};
  double first_blocked_time_s{0.0};
  Eigen::Vector3d first_blocked_position{Eigen::Vector3d::Constant(
      std::numeric_limits<double>::quiet_NaN())};
  int first_blocked_cell_state{0};
  // Product-facing copy of the swept validator reason.  Keeping the reason
  // here makes world-recertification failures diagnosable without weakening
  // the fail-closed decision.
  int failure_code{0};
  int blocked_role{0};
  std::size_t sample_count{0};
  std::size_t segment_count{0};
};

// Diagnostics for the planner-owned BACKUP certificate search.  These fields
// are observability only: the corresponding safety gates remain unchanged and
// a zero/unknown value must never be interpreted as a certificate pass.
struct BackupCertificateDiagnostics {
  bool attempted{false};
  std::uint32_t switch_candidate_count{0};
  std::uint32_t feasible_seed_count{0};
  std::uint32_t visibility_hull_pass_count{0};
  std::uint32_t aligned_sfc_built_count{0};
  std::uint32_t aligned_hull_pass_count{0};
  std::uint32_t known_free_check_count{0};
  std::uint32_t known_free_pass_count{0};
  bool selected{false};
  int last_reject_stage{0};
  int last_known_free_failure_code{0};
  int last_known_free_cell_state{0};
  int last_known_free_blocked_role{0};
  double last_known_free_first_blocked_time_s{
      std::numeric_limits<double>::quiet_NaN()};
  Eigen::Vector3d last_known_free_blocked_position{Eigen::Vector3d::Constant(
      std::numeric_limits<double>::quiet_NaN())};
  double last_seed_switch_time_s{std::numeric_limits<double>::quiet_NaN()};
  double last_seed_duration_s{std::numeric_limits<double>::quiet_NaN()};
  double last_seed_initial_velocity_mps{std::numeric_limits<double>::quiet_NaN()};
  double last_seed_max_velocity_mps{std::numeric_limits<double>::quiet_NaN()};
  double last_seed_max_acceleration_mps2{std::numeric_limits<double>::quiet_NaN()};
  double last_seed_max_jerk_mps3{std::numeric_limits<double>::quiet_NaN()};
  Eigen::Vector3d last_seed_endpoint{Eigen::Vector3d::Constant(
      std::numeric_limits<double>::quiet_NaN())};
};

enum class BackupCertificateRejectStage : std::uint8_t {
  kNone = 0,
  kCommandBoundary = 1,
  kSeed = 2,
  kVisibilityHull = 3,
  kAlignedSfc = 4,
  kAlignedHull = 5,
  kKnownFree = 6,
  kRefinementKnownFree = 7,
  kYaw = 8,
  kDynamic = 9,
  kDeadline = 10,
};

inline const char* backupCertificateRejectStageName(
    const BackupCertificateRejectStage stage) noexcept {
  switch (stage) {
    case BackupCertificateRejectStage::kNone: return "none";
    case BackupCertificateRejectStage::kCommandBoundary: return "command_boundary";
    case BackupCertificateRejectStage::kSeed: return "seed";
    case BackupCertificateRejectStage::kVisibilityHull: return "visibility_hull";
    case BackupCertificateRejectStage::kAlignedSfc: return "aligned_sfc";
    case BackupCertificateRejectStage::kAlignedHull: return "aligned_hull";
    case BackupCertificateRejectStage::kKnownFree: return "known_free";
    case BackupCertificateRejectStage::kRefinementKnownFree:
      return "refinement_known_free";
    case BackupCertificateRejectStage::kYaw: return "yaw";
    case BackupCertificateRejectStage::kDynamic: return "dynamic";
    case BackupCertificateRejectStage::kDeadline: return "deadline";
  }
  return "unknown";
}

struct PlannerDiagnostics {
  int solve_stage{0};
  std::size_t solve_point_count{0};
  std::string solve_stage_name;
  int replan_return_code{0};
  int commit_decision{0};
  double solve_deadline_s{0.0};
  double requested_cruise_speed_mps{std::numeric_limits<double>::quiet_NaN()};
  double effective_cruise_speed_mps{std::numeric_limits<double>::quiet_NaN()};
  double control_max_velocity_mps{std::numeric_limits<double>::quiet_NaN()};
  double control_max_acceleration_mps2{std::numeric_limits<double>::quiet_NaN()};
  double control_max_jerk_mps3{std::numeric_limits<double>::quiet_NaN()};
  double physical_max_velocity_mps{std::numeric_limits<double>::quiet_NaN()};
  double physical_max_acceleration_mps2{std::numeric_limits<double>::quiet_NaN()};
  double physical_max_jerk_mps3{std::numeric_limits<double>::quiet_NaN()};
  // These are the extrema of the latest independently evaluated MAIN
  // candidate. They are diagnostic evidence; candidate admission remains
  // governed by the existing hard certificates.
  double candidate_maximum_velocity_mps{std::numeric_limits<double>::quiet_NaN()};
  double candidate_maximum_acceleration_mps2{
      std::numeric_limits<double>::quiet_NaN()};
  double candidate_maximum_jerk_mps3{std::numeric_limits<double>::quiet_NaN()};
  std::array<double, 4> module_time_us{};
  Eigen::Vector3d latest_guide_start{Eigen::Vector3d::Constant(
      std::numeric_limits<double>::quiet_NaN())};
  Eigen::Vector3d latest_guide_end{Eigen::Vector3d::Constant(
      std::numeric_limits<double>::quiet_NaN())};
  Eigen::Vector3d latest_guide_min{Eigen::Vector3d::Constant(
      std::numeric_limits<double>::quiet_NaN())};
  Eigen::Vector3d latest_guide_max{Eigen::Vector3d::Constant(
      std::numeric_limits<double>::quiet_NaN())};
  double latest_guide_path_length_m{std::numeric_limits<double>::quiet_NaN()};
  double latest_guide_duration_s{std::numeric_limits<double>::quiet_NaN()};
  double required_lookahead_m{std::numeric_limits<double>::quiet_NaN()};
  double certified_lookahead_m{std::numeric_limits<double>::quiet_NaN()};
  bool lookahead_complete{false};
  Eigen::Vector3d requested_goal{Eigen::Vector3d::Constant(
      std::numeric_limits<double>::quiet_NaN())};
  Eigen::Vector3d planning_goal{Eigen::Vector3d::Constant(
      std::numeric_limits<double>::quiet_NaN())};
  double goal_acceptance_radius_m{std::numeric_limits<double>::quiet_NaN()};
  bool goal_endpoint_adjusted{false};
  int requested_goal_inflated_state{0};
  int planning_goal_inflated_state{0};
  int route_yaw_source{4};
  double route_yaw_target_rad{std::numeric_limits<double>::quiet_NaN()};
  double route_yaw_lookahead_m{std::numeric_limits<double>::quiet_NaN()};
  double route_yaw_progress_arc_m{std::numeric_limits<double>::quiet_NaN()};
  double yaw_rate_limit_rad_s{std::numeric_limits<double>::quiet_NaN()};
  double yaw_acceleration_limit_rad_s2{std::numeric_limits<double>::quiet_NaN()};
  double candidate_maximum_yaw_rate_rad_s{std::numeric_limits<double>::quiet_NaN()};
  double candidate_maximum_yaw_acceleration_rad_s2{
      std::numeric_limits<double>::quiet_NaN()};
  OptimizationDiagnostics optimization{};
  BackupCertificateDiagnostics backup_certificate{};
};

}  // namespace navigation_planning
