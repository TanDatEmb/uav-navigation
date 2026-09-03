#pragma once

#include <cstddef>
#include <cstdint>
#include <functional>
#include <memory>
#include <optional>
#include <string>

#include <Eigen/Core>

#include <navigation_planning/candidate_bundle.hpp>
#include <navigation_planning/kinematic_state.hpp>
#include <navigation_planning/planner_diagnostics.hpp>
#include <navigation_planning/planner_status.hpp>
#include <navigation_planning/planning_outcome.hpp>
#include <navigation_planning/planning_request.hpp>
#include <navigation_planning/planning_limits.hpp>
#include <navigation_mission/route_progress.hpp>
#include <navigation_world_model/world_commit_authorizer.hpp>
#include <navigation_world_model/world_model_view.hpp>

namespace navigation_planning_backend {

class PlannerFacade final {
 public:
  PlannerFacade(const std::string& config_path,
                navigation_world_model::WorldModelViewPtr world,
                const std::optional<navigation_planning::DynamicLimits>& mission_limits,
                navigation_world_model::WorldCommitAuthorizer& commit_authorizer,
                std::function<double()> ros_time_seconds);
  ~PlannerFacade();

  PlannerFacade(const PlannerFacade&) = delete;
  PlannerFacade& operator=(const PlannerFacade&) = delete;

  void cancelActiveSolve() noexcept;
  void resetSolveCancellation() noexcept;
  void resetOptimizationDiagnostics() noexcept;
  void setCommandIdentity(std::uint64_t localization_epoch,
                          std::uint64_t goal_epoch,
                          std::uint64_t request_id);
  void discardCommandCandidate() noexcept;
  void onExecutionTimelineActivated(std::uint64_t generation) noexcept;
  [[nodiscard]] bool hasStagedCommandCandidate() const;
  [[nodiscard]] navigation_planning::TrajectoryValidationResult
  validateStagedCommandCandidate(
      const navigation_world_model::WorldModelViewPtr& world,
      double authorization_wall_time_s,
      std::uint64_t expected_generation) const;
  void setWorldModelView(navigation_world_model::WorldModelViewPtr world);
  void setGoalAcceptanceRadius(double radius_m) noexcept;
  [[nodiscard]] bool setRouteSnapshot(
      const navigation_mission::ImmutableRouteSnapshot& route) noexcept;
  void setPassThroughNextTarget(
      const std::optional<Eigen::Vector3d>& next_target) noexcept;
  bool setState(const navigation_planning::KinematicState& state);
  // Initial planning is valid only for a stopped/hold state. The runtime
  // owns that gate; this API names the lifecycle contract explicitly.
  [[nodiscard]] navigation_planning::PlannerStatus planInitialFromStoppedState(
      const Eigen::Vector3d& target_world, double target_yaw_rad, bool new_goal);
  // All in-flight renewal is successor planning from the execution timeline.
  [[nodiscard]] navigation_planning::PlannerStatus planSuccessorFromExecutionAnchor(
      const Eigen::Vector3d& target_world, double target_yaw_rad, bool new_goal);

  // Product-facing planning transaction. The request is immutable for the
  // solve and the outcome owns the only candidate handed to execution.
  [[nodiscard]] navigation_planning::PlanningOutcome plan(
      const navigation_planning::PlanningRequest& request);

  // Source-compatibility aliases for tools that have not yet migrated. They
  // are not used by the runtime lifecycle and do not authorize moving resets.
  [[deprecated("use planInitialFromStoppedState")]]
  [[nodiscard]] navigation_planning::PlannerStatus planFromRest(
      const Eigen::Vector3d& target_world, double target_yaw_rad, bool new_goal);
  [[deprecated("use planSuccessorFromExecutionAnchor")]]
  [[nodiscard]] navigation_planning::PlannerStatus replanOnce(
      const Eigen::Vector3d& target_world, double target_yaw_rad, bool new_goal);

  [[nodiscard]] std::optional<navigation_planning::CandidateBundle> exportCommandCandidate(
      std::uint64_t localization_epoch,
      std::uint64_t goal_epoch,
      std::uint64_t request_id,
      std::int64_t valid_from_ns,
      std::int64_t valid_until_ns) const;

  [[nodiscard]] bool commitEmergencyBrake(
      const navigation_planning::TrajectoryPoint& initial_command,
      double start_wall_time_s,
      // Optional execution-owned terminal altitude. The caller must bound it
      // against the measured boundary; the planner still certifies the full
      // emergency trajectory before committing it.
      std::optional<double> terminal_altitude_m = std::nullopt);

  [[nodiscard]] navigation_planning::CommittedTrajectorySnapshot committedSnapshot() const;
  [[nodiscard]] navigation_planning::CommittedTrajectoryMetadata committedMetadata() const;
  [[nodiscard]] navigation_planning::TrajectoryValidationResult validateCommittedTrajectory(
      const navigation_world_model::WorldModelViewPtr& world,
      double authorization_wall_time_s,
      std::uint64_t expected_generation = 0U) const;
  [[nodiscard]] std::uint64_t committedGeneration() const noexcept;
  [[nodiscard]] bool committedBackupAvailable() const noexcept;
  [[nodiscard]] double committedBackupStartTime() const noexcept;
  [[nodiscard]] int solveStage() const noexcept;
  [[nodiscard]] std::size_t solvePointCount() const noexcept;
  [[nodiscard]] double solveDeadlineSeconds() const noexcept;
  [[nodiscard]] double replanForwardSeconds() const noexcept;
  [[nodiscard]] double trackingErrorBudgetMeters() const noexcept;

  [[nodiscard]] navigation_planning::PlannerDiagnostics diagnostics() const;

 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace navigation_planning_backend
