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

  void discardCommandCandidate() noexcept;
  // Caller owns an unadmitted retained candidate; zero/wrong/retired
  // generations cannot clear the currently registered heading owner.
  [[nodiscard]] bool discardRetainedPositionHeadingCandidate(
      std::uint64_t expected_generation) noexcept;
  void onExecutionTimelineActivated(std::uint64_t generation) noexcept;
  [[nodiscard]] bool hasStagedCommandCandidate() const;
  [[nodiscard]] navigation_planning::TrajectoryValidationResult
  validateStagedCommandCandidate(
      const navigation_world_model::WorldModelViewPtr& world,
      double authorization_wall_time_s,
      std::uint64_t expected_generation) const;
  [[nodiscard]] std::optional<navigation_planning::CandidateBundle>
  buildImmediateHeadingRebindCandidate(
      const navigation_world_model::WorldModelViewPtr& world,
      const navigation_mission::ImmutableRouteSnapshot& route,
      const Eigen::Vector3d& measured_position,
      const Eigen::Vector3d& measured_velocity,
      double measured_yaw_rad,
      const std::optional<Eigen::Vector3d>& mission_start_position,
      double activation_wall_time_s,
      std::uint64_t localization_epoch,
      std::uint64_t goal_epoch,
      std::uint64_t request_id,
      std::int64_t valid_from_ns,
      std::int64_t valid_until_ns);
  // Product-facing planning transaction. The request is immutable for the
  // solve and the outcome owns the only candidate handed to execution.
  [[nodiscard]] navigation_planning::PlanningOutcome plan(
      const navigation_planning::PlanningRequest& request);

  [[nodiscard]] navigation_planning::CommittedTrajectorySnapshot committedSnapshot() const;
  [[nodiscard]] navigation_planning::CommittedTrajectoryMetadata committedMetadata() const;
  [[nodiscard]] navigation_planning::TrajectoryValidationResult validateCommittedTrajectory(
      const navigation_planning::CandidateBundle& execution_bundle,
      const navigation_world_model::WorldModelViewPtr& world,
      double authorization_wall_time_s) const;
  [[nodiscard]] std::uint64_t committedGeneration() const noexcept;
  [[nodiscard]] bool committedBackupAvailable() const noexcept;
  [[nodiscard]] double committedBackupStartTime() const noexcept;
  [[nodiscard]] int solveStage() const noexcept;
  [[nodiscard]] std::size_t solvePointCount() const noexcept;
  [[nodiscard]] double solveDeadlineSeconds() const noexcept;
  [[nodiscard]] double replanForwardSeconds() const noexcept;
  [[nodiscard]] double trackingErrorBudgetMeters() const noexcept;
  [[nodiscard]] double yawRateLimitRadS() const noexcept;
  [[nodiscard]] double yawAccelerationLimitRadS2() const noexcept;

  [[nodiscard]] navigation_planning::PlannerDiagnostics diagnostics() const;

 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace navigation_planning_backend
