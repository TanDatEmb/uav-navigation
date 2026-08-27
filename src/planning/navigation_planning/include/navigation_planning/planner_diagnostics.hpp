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
  int retry_count{0};
  int retry_violation_mask{0};
  int retry_stop_reason{0};
  int first_lbfgs_return_code{-1};
  int last_lbfgs_return_code{-1};
  bool cancelled{false};
  bool valid{false};
  double initial_normalized_dynamic_violation{std::numeric_limits<double>::quiet_NaN()};
  double best_normalized_dynamic_violation{std::numeric_limits<double>::quiet_NaN()};
  double final_normalized_dynamic_violation{std::numeric_limits<double>::quiet_NaN()};
  double initial_duration_s{std::numeric_limits<double>::quiet_NaN()};
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

struct PlannerDiagnostics {
  int solve_stage{0};
  std::size_t solve_point_count{0};
  std::string solve_stage_name;
  int replan_return_code{0};
  int commit_decision{0};
  double solve_deadline_s{0.0};
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
  Eigen::Vector3d requested_goal{Eigen::Vector3d::Constant(
      std::numeric_limits<double>::quiet_NaN())};
  Eigen::Vector3d planning_goal{Eigen::Vector3d::Constant(
      std::numeric_limits<double>::quiet_NaN())};
  double goal_acceptance_radius_m{std::numeric_limits<double>::quiet_NaN()};
  bool goal_endpoint_adjusted{false};
  int requested_goal_inflated_state{0};
  int planning_goal_inflated_state{0};
  OptimizationDiagnostics optimization{};
};

}  // namespace navigation_planning
