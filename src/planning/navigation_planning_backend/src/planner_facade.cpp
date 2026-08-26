#include <navigation_planning_backend/planner_facade.hpp>

#include <navigation_planning_backend/planner.hpp>

#include <data_structure/cmd_traj.h>
#include <planner_core/log_utils.hpp>
#include <planner_core/planning_stage.hpp>
#include <planner_core/trajectory_world_validator.hpp>
#include <planner_runtime_context/planner_runtime_context.hpp>
#include <traj_opt/exp_traj_optimizer_s4.h>

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <utility>

namespace navigation_planning_backend {
namespace {

navigation_planning::PlannerStatus toProductStatus(const RET_CODE result) noexcept {
  switch (result) {
    case SUCCESS:
      return navigation_planning::PlannerStatus::kSuccess;
    case FINISH:
      return navigation_planning::PlannerStatus::kFinished;
    case NO_NEED:
      return navigation_planning::PlannerStatus::kNoNeed;
    case NEW_TRAJ:
      return navigation_planning::PlannerStatus::kRestartFromRest;
    case EMER:
      return navigation_planning::PlannerStatus::kEmergency;
    case OPT_FAILED:
      return navigation_planning::PlannerStatus::kOptimizationFailed;
    case FAILED:
    default:
      return navigation_planning::PlannerStatus::kFailed;
  }
}

Eigen::Vector3d toVector3d(const auto& value) {
  return Eigen::Vector3d{value.x(), value.y(), value.z()};
}

navigation_planning::OptimizationDiagnostics toProductDiagnostics(
    const traj_opt::ExpOptimizationDiagnostics& source) {
  navigation_planning::OptimizationDiagnostics output;
  output.lbfgs_attempt_count = source.lbfgs_attempt_count;
  output.retry_count = source.retry_count;
  output.retry_violation_mask = source.retry_violation_mask;
  output.retry_stop_reason = source.retry_stop_reason;
  output.first_lbfgs_return_code = source.first_lbfgs_return_code;
  output.last_lbfgs_return_code = source.last_lbfgs_return_code;
  output.cancelled = source.cancelled;
  output.valid = source.valid;
  output.initial_normalized_dynamic_violation = source.initial_normalized_dynamic_violation;
  output.best_normalized_dynamic_violation = source.best_normalized_dynamic_violation;
  output.final_normalized_dynamic_violation = source.final_normalized_dynamic_violation;
  output.initial_duration_s = source.initial_duration_s;
  output.final_duration_s = source.final_duration_s;
  output.retry_duration_lower_bound_min_s = source.retry_duration_lower_bound_min_s;
  output.retry_duration_lower_bound_max_s = source.retry_duration_lower_bound_max_s;
  output.retry_free_duration_seed_min_s = source.retry_free_duration_seed_min_s;
  output.retry_free_duration_seed_max_s = source.retry_free_duration_seed_max_s;
  output.retry_budget_remaining_us = source.retry_budget_remaining_us;
  output.nonfinite_evaluation_count = source.nonfinite_evaluation_count;
  output.first_nonfinite_stage = source.first_nonfinite_stage;
  output.first_nonfinite_value_mask = source.first_nonfinite_value_mask;
  output.first_nonfinite_attempt = source.first_nonfinite_attempt;
  output.first_nonfinite_iteration = source.first_nonfinite_iteration;
  output.first_nonfinite_min_duration_s = source.first_nonfinite_min_duration_s;
  output.first_nonfinite_max_duration_s = source.first_nonfinite_max_duration_s;
  output.first_nonfinite_cost = source.first_nonfinite_cost;
  output.first_nonfinite_gradient_norm = source.first_nonfinite_gradient_norm;
  return output;
}

navigation_planning::CommitDiagnostics toProductDiagnostics(
    const navigation_planning_backend::CommitDiagnostics& source) {
  navigation_planning::CommitDiagnostics output;
  output.generation = source.generation;
  output.previous_generation = source.previous_generation;
  output.candidate_start_wall_time_s = source.candidate_start_wall_time;
  output.candidate_start_position = toVector3d(source.candidate_start_pvaj.col(0));
  output.candidate_start_velocity = toVector3d(source.candidate_start_pvaj.col(1));
  output.candidate_start_acceleration = toVector3d(source.candidate_start_pvaj.col(2));
  output.candidate_start_jerk = toVector3d(source.candidate_start_pvaj.col(3));
  output.candidate_start_yaw = source.candidate_start_yaw;
  output.candidate_start_yaw_rate = source.candidate_start_yaw_rate;
  output.previous_valid = source.previous_valid;
  output.previous_sample_time_s = source.previous_sample_tt;
  output.position_residual = toVector3d(source.position_residual);
  output.velocity_residual = toVector3d(source.velocity_residual);
  output.acceleration_residual = toVector3d(source.acceleration_residual);
  output.jerk_residual = toVector3d(source.jerk_residual);
  output.yaw_residual = source.yaw_residual;
  output.yaw_rate_residual = source.yaw_rate_residual;
  return output;
}

navigation_planning::TrajectorySnapshot toTrajectorySnapshot(
    const geometry_utils::Trajectory& position,
    const geometry_utils::Trajectory& yaw,
    const std::vector<CandidateRoleInterval>& roles) {
  navigation_planning::TrajectorySnapshot output;
  output.start_wall_time_s = position.start_WT;
  output.duration_s = position.getTotalDuration();
  auto position_copy = std::make_shared<geometry_utils::Trajectory>(position);
  auto yaw_copy = std::make_shared<geometry_utils::Trajectory>(yaw);
  auto roles_copy = std::make_shared<std::vector<CandidateRoleInterval>>(roles);
  output.evaluator = [position_copy, yaw_copy](
                          const double trajectory_time_s,
                          navigation_planning::TrajectoryPoint& point) {
    if (position_copy->empty() || yaw_copy->empty()) return false;
    const auto state = position_copy->getState(trajectory_time_s);
    const auto yaw_state = yaw_copy->getState(trajectory_time_s);
    if (!state.allFinite() || !yaw_state.allFinite()) return false;
    point.position_world = state.col(0);
    point.velocity_world = state.col(1);
    point.acceleration_world = state.col(2);
    point.jerk_world = state.col(3);
    point.yaw = yaw_state(0, 0);
    point.yaw_rate = yaw_state(0, 1);
    point.trajectory_time_s = trajectory_time_s;
    return true;
  };
  output.role_evaluator = [roles_copy](const double trajectory_time_s) {
    for (const auto& interval : *roles_copy) {
      if (trajectory_time_s >= interval.begin_tt && trajectory_time_s <= interval.end_tt &&
          interval.role == CandidateTrajectoryRole::BACKUP) {
        return navigation_planning::CandidateRole::kBackup;
      }
    }
    return navigation_planning::CandidateRole::kMain;
  };
  return output;
}

}  // namespace

struct PlannerFacade::Impl {
  navigation_planner_context::PlannerRuntimeContext::Ptr context;
  std::unique_ptr<Planner> planner;
};

PlannerFacade::PlannerFacade(
    const std::string& config_path,
    navigation_world_model::WorldModelViewPtr world,
    const std::optional<navigation_planning::DynamicLimits>& mission_limits,
    navigation_world_model::WorldCommitAuthorizer& commit_authorizer,
    std::function<double()> ros_time_seconds)
    : impl_(std::make_unique<Impl>()) {
  if (!ros_time_seconds) throw std::invalid_argument("planner facade requires a ROS time provider");
  impl_->context = std::make_shared<navigation_planner_context::PlannerRuntimeContext>(
      std::move(ros_time_seconds));
  impl_->planner = std::make_unique<Planner>(
      config_path, impl_->context, std::move(world), mission_limits, commit_authorizer);
}

PlannerFacade::~PlannerFacade() = default;

void PlannerFacade::cancelActiveSolve() noexcept {
  if (impl_ && impl_->planner) impl_->planner->cancelActiveSolve();
}

void PlannerFacade::resetSolveCancellation() noexcept {
  if (impl_ && impl_->planner) impl_->planner->resetSolveCancellation();
}

void PlannerFacade::resetOptimizationDiagnostics() noexcept {
  if (impl_ && impl_->planner) impl_->planner->resetExpOptimizationDiagnostics();
}

void PlannerFacade::setWorldModelView(navigation_world_model::WorldModelViewPtr world) {
  impl_->planner->setWorldModelView(std::move(world));
}

bool PlannerFacade::setState(const navigation_planning::KinematicState& state) {
  return impl_->planner->setState(state);
}

navigation_planning::PlannerStatus PlannerFacade::planFromRest(
    const Eigen::Vector3d& target_world, const double target_yaw_rad, const bool new_goal) {
  const Vec3f target = target_world;
  return toProductStatus(impl_->planner->PlanFromRest(target, target_yaw_rad, new_goal));
}

navigation_planning::PlannerStatus PlannerFacade::replanOnce(
    const Eigen::Vector3d& target_world, const double target_yaw_rad, const bool new_goal) {
  const Vec3f target = target_world;
  return toProductStatus(impl_->planner->ReplanOnce(target, target_yaw_rad, new_goal));
}

std::optional<navigation_planning::CandidateBundle> PlannerFacade::exportCommandCandidate(
    const std::uint64_t localization_epoch,
    const std::uint64_t goal_epoch,
    const std::uint64_t request_id,
    const std::int64_t valid_from_ns,
    const std::int64_t valid_until_ns) const {
  return impl_->planner->exportCommandCandidate(
      localization_epoch, goal_epoch, request_id, valid_from_ns, valid_until_ns);
}

bool PlannerFacade::commitEmergencyBrake(
    const navigation_planning::TrajectoryPoint& measured_command,
    const double start_wall_time_s) {
  if (!measured_command.finite()) return false;
  StatePVAJ state = StatePVAJ::Zero();
  state.col(0) = measured_command.position_world;
  state.col(1) = measured_command.velocity_world;
  state.col(2) = measured_command.acceleration_world;
  state.col(3) = measured_command.jerk_world;
  return impl_->planner->commitEmergencyBrake(
      state, measured_command.yaw, measured_command.yaw_rate, start_wall_time_s);
}

navigation_planning::CommittedTrajectorySnapshot PlannerFacade::committedSnapshot() const {
  const auto snapshot = impl_->planner->committedTrajectorySnapshot();
  navigation_planning::CommittedTrajectorySnapshot output;
  output.position = toTrajectorySnapshot(snapshot.position, snapshot.yaw, snapshot.roles);
  output.generation = snapshot.generation;
  output.certificate.pinned_world = snapshot.certificate.pinned_world;
  output.certificate.validated_world = snapshot.certificate.validated_world;
  output.certificate.validation_begin_time_s = snapshot.certificate.validation_begin_tt;
  output.diagnostics = toProductDiagnostics(snapshot.diagnostics);
  output.backup_available = snapshot.backup_available;
  output.backup_start_time_s = snapshot.backup_start_tt;
  if (snapshot.empty) output.position = {};
  return output;
}

navigation_planning::CommittedTrajectoryMetadata PlannerFacade::committedMetadata() const {
  const auto metadata = impl_->planner->committedMetadataSnapshot();
  return {metadata.generation, toProductDiagnostics(metadata.diagnostics)};
}

navigation_planning::TrajectoryValidationResult PlannerFacade::validateCommittedTrajectory(
    const navigation_world_model::WorldModelViewPtr& world,
    const double authorization_wall_time_s) const {
  navigation_planning::TrajectoryValidationResult output;
  if (!world) return output;
  const auto snapshot = impl_->planner->committedTrajectorySnapshot();
  if (snapshot.empty || snapshot.position.empty() || snapshot.yaw.empty()) return output;
  output.pinned_world = snapshot.certificate.pinned_world;
  output.validated_world = world->identity();
  CandidateCommandBundle candidate;
  candidate.position = snapshot.position;
  candidate.yaw = snapshot.yaw;
  candidate.start_wall_time = snapshot.position.start_WT;
  const auto validation = validateExecutableCandidate(*world, candidate, authorization_wall_time_s);
  output.valid = validation.valid;
  output.begin_time_s = validation.begin_tt;
  output.first_blocked_time_s = validation.first_blocked_tt;
  output.sample_count = validation.sample_count;
  output.segment_count = validation.segment_count;
  if (!validation.valid) {
    const double duration = candidate.position.getTotalDuration();
    const double sample_time = std::clamp(validation.first_blocked_tt, 0.0, duration);
    output.first_blocked_position = toVector3d(candidate.position.getPos(sample_time));
    const auto state = world->classify(
        candidate.position.getPos(sample_time), navigation_world_model::GridLayer::kInflated);
    output.first_blocked_cell_state = static_cast<int>(state);
  }
  return output;
}

std::uint64_t PlannerFacade::committedGeneration() const noexcept {
  return impl_->planner->committedGenerationSnapshot();
}

bool PlannerFacade::committedBackupAvailable() const noexcept {
  return impl_->planner->committedBackupTrajectoryAvailable();
}

double PlannerFacade::committedBackupStartTime() const noexcept {
  return impl_->planner->getCommittedBackupStartTrajectoryTime();
}

int PlannerFacade::solveStage() const noexcept {
  return impl_->planner->solveStage();
}

std::size_t PlannerFacade::solvePointCount() const noexcept {
  return impl_->planner->solvePointCount();
}

navigation_planning::PlannerDiagnostics PlannerFacade::diagnostics() const {
  navigation_planning::PlannerDiagnostics output;
  output.solve_stage = impl_->planner->solveStage();
  output.solve_point_count = impl_->planner->solvePointCount();
  output.solve_stage_name = std::string(solveStageName(output.solve_stage));
  output.replan_return_code = impl_->planner->latestReplanReturnCode();
  output.commit_decision = impl_->planner->latestCommitDecision();
  output.solve_deadline_s = impl_->planner->solveDeadlineSeconds();
  std::vector<double> module_times;
  module_times = impl_->planner->moduleTimeConsumingSnapshot();
  const auto read_time = [&module_times](const std::size_t index) {
    return module_times.size() > index ? module_times[index] * 1.0e6 : 0.0;
  };
  output.module_time_us = {read_time(EPX_TRAJ_FRONTEND), read_time(EXP_TRAJ_OPT),
                           read_time(BACK_TRAJ_FRONTEND), read_time(BACK_TRAJ_OPT)};
  output.latest_guide_start = toVector3d(impl_->planner->latestGuideStart());
  output.latest_guide_end = toVector3d(impl_->planner->latestGuideEnd());
  output.latest_guide_min = toVector3d(impl_->planner->latestGuideMin());
  output.latest_guide_max = toVector3d(impl_->planner->latestGuideMax());
  output.latest_guide_path_length_m = impl_->planner->latestGuidePathLengthMeters();
  output.latest_guide_duration_s = impl_->planner->latestGuideDurationSeconds();
  output.optimization = toProductDiagnostics(impl_->planner->latestExpOptimizationDiagnostics());
  return output;
}

}  // namespace navigation_planning_backend
