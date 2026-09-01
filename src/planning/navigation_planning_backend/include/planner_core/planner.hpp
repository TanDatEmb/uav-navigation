/*
 * Product-owned navigation implementation.
 * Algorithmic provenance and external attributions are documented in the
 * package documentation; they are not part of the runtime API or behaviour.
 */

#pragma once

#include <algorithm>
#include <cmath>
#include <limits>

#include <iostream>
#include <fstream>
#include <cstdint>
#include <optional>
#include <fmt/color.h>
#include "Eigen/Eigen"


#include <planner_core/config.hpp>
#include <planner_core/absolute_deadline.hpp>
#include <planner_runtime_context/planner_runtime_context.hpp>
#include <data_structure/base/trajectory.h>

#include <data_structure/base/polytope.h>


#include "traj_opt/nominal_trajectory_optimizer.hpp"
#include "traj_opt/backup_trajectory_optimizer.hpp"
#include "path_search/astar.h"
#include <navigation_world_model/world_model_view.hpp>
#include <navigation_world_model/goal_contract.hpp>
#include <navigation_planning/kinematic_state.hpp>
#include <navigation_planning/planner_diagnostics.hpp>
#include <navigation_world_model/world_commit_authorizer.hpp>
#include "planner_core/corridor_generator.h"
#include "planner_core/fov_checker.h"
#include "planner_core/pass_through_terminal_velocity.hpp"

#include "traj_opt/yaw_traj_opt.h"
#include "planner_core/planner_result.hpp"
#include "utils/header/fmt_eigen.hpp"

#include <planner_core/log_utils.hpp>
#include <data_structure/exp_traj.h>
#include <data_structure/cmd_traj.h>
#include <data_structure/backup_traj.h>
#include <navigation_planning/candidate_bundle.hpp>
#include <navigation_mission/route_progress.hpp>
#include <planner_core/route_yaw_reference.hpp>


namespace navigation_planning_backend {
    using navigation_math::RET_CODE;
    using namespace color_text;
    using namespace geometry_utils;

    class Planner {
        LogOneReplan latest_replan;
        navigation_planning_backend::Config cfg_;
        navigation_world_model::WorldModelViewPtr map_ptr_;
        navigation_world_model::WorldCommitAuthorizer* commit_authorizer_{nullptr};
        CorridorGenerator::Ptr cg_ptr_;
        path_search::Astar::Ptr astar_ptr_;
        navigation_planner_context::PlannerRuntimeContext::Ptr planner_context_;
        Vec3f shifted_sfc_start_pt_;

        traj_opt::ExpTrajOpt::Ptr exp_traj_opt_;
        traj_opt::BackupTrajOpt::Ptr back_traj_opt_;
        traj_opt::YawTrajOpt::Ptr yaw_traj_opt_;

        CIRI::Ptr ciri_;

        navigation_math::RobotState robot_state_;
        // Stable per-solve state. `robot_state_` is ingress-owned and may be
        // replaced by an odometry callback while a solve is running.
        navigation_math::RobotState solve_state_;
        bool robot_acceleration_estimated_{false};
        bool robot_jerk_estimated_{false};
        bool solve_acceleration_estimated_{false};
        bool solve_jerk_estimated_{false};
        double nominal_exp_max_velocity_mps_{0.0};
        double nominal_backup_max_velocity_mps_{0.0};

        std::mutex drone_state_mutex_;
        mutable std::mutex replan_lock_;
        mutable std::mutex solve_commit_mutex_;
        mutable std::mutex command_identity_mutex_;
        CommandIdentity command_identity_{};

        struct StagedCommandCandidate {
            CandidateCommandBundle command;
            CommandCertificate certificate;
            std::uint64_t generation{0};
            // Planner history is promoted only with the execution-store ACK.
            // A staged candidate may still be rejected by the runtime after
            // export, so history must not move ahead of command authority.
            std::optional<ExpTraj> pending_exp_history;
            bool clear_new_goal_on_ack{false};
        };
        std::optional<StagedCommandCandidate> staged_command_candidate_;

        Vec3f local_start_p_;

        // Keep the mission target for diagnostics, but allow the geometric
        // planner to use a certified free cell inside the waypoint acceptance
        // ball when the exact inflated voxel is occupied.
        double goal_acceptance_radius_m_{
            navigation_world_model::kGoalCompletionToleranceM};
        Vec3f requested_goal_p_{Vec3f::Constant(
            std::numeric_limits<float>::quiet_NaN())};
        Vec3f planning_goal_p_{Vec3f::Constant(
            std::numeric_limits<float>::quiet_NaN())};
        navigation_world_model::CellState requested_goal_inflated_state_{
            navigation_world_model::CellState::kUndefined};
        navigation_world_model::CellState planning_goal_inflated_state_{
            navigation_world_model::CellState::kUndefined};
        bool goal_endpoint_adjusted_{false};
        bool terminal_stop_required_{false};
        // A remote mission STOP must not turn each bounded local prefix into
        // a terminal stop. This flag is set only after the current candidate
        // endpoint is actually connected to that STOP.
        bool candidate_terminal_stop_active_{false};
        bool pass_through_coincident_terminal_stop_{false};
        // A projected terminal endpoint is leased for one immutable mission
        // request. Re-running the optimizer on a changing occupancy revision
        // must not flip the endpoint between homotopy sides while the old
        // endpoint is still known-free and certifiable.
        std::optional<Vec3f> acceptance_endpoint_lease_;
        std::uint64_t acceptance_endpoint_lease_request_id_{0U};
        std::uint64_t acceptance_endpoint_lease_route_revision_{0U};
        std::size_t acceptance_endpoint_lease_waypoint_index_{0U};
        std::optional<Vec3f> pass_through_next_target_;
        std::optional<navigation_mission::ImmutableRouteSnapshot> route_snapshot_;
        RouteYawReference route_yaw_reference_{};
        std::optional<double> last_route_yaw_target_rad_;
        double latest_candidate_maximum_yaw_rate_rad_s_{
            std::numeric_limits<double>::quiet_NaN()};
        double latest_candidate_maximum_yaw_acceleration_rad_s2_{
            std::numeric_limits<double>::quiet_NaN()};

        // use negative value to indicate the traj is not available
        double on_backup_start_WT{-1}, on_backup_end_WT{-1};

        double planner_process_start_WT_;

        struct GoalInfo {
            Vec3f goal_p{0, 0, 0};
            double goal_yaw{0};
            bool new_goal{true};
            bool goal_valid{true};
        } gi_;

        FOVChecker::Ptr fov_checker_;

        CmdTraj cmd_traj_info_;
        ExpTraj last_exp_traj_info_;

        vector<double> time_consuming_;
        // 0 idle, 1 setup, 2 A*, 3 corridor/CIRI, 4 main MINCO,
        // 5 backup generation/MINCO. Read by the external watchdog.
        std::atomic<int> solve_stage_{0};
        std::atomic_bool solve_cancelled_{false};
        std::atomic<int> latest_commit_decision_{
            static_cast<int>(navigation_world_model::WorldCommitDecision::kNotAttempted)};
        bool estimated_boundary_warning_emitted_{false};
        std::uint64_t backup_refinement_success_count_{0};
        std::uint64_t backup_refinement_fallback_count_{0};
        navigation_planning::BackupCertificateDiagnostics backup_certificate_diagnostics_{};

        Vec3f latest_guide_start_{Vec3f::Constant(std::numeric_limits<double>::quiet_NaN())};
        Vec3f latest_guide_end_{Vec3f::Constant(std::numeric_limits<double>::quiet_NaN())};
        Vec3f latest_guide_min_{Vec3f::Constant(std::numeric_limits<double>::quiet_NaN())};
        Vec3f latest_guide_max_{Vec3f::Constant(std::numeric_limits<double>::quiet_NaN())};
        double latest_guide_path_length_m_{std::numeric_limits<double>::quiet_NaN()};
        double latest_guide_duration_s_{std::numeric_limits<double>::quiet_NaN()};
        double required_lookahead_m_{std::numeric_limits<double>::quiet_NaN()};
        double certified_lookahead_m_{std::numeric_limits<double>::quiet_NaN()};
        bool lookahead_complete_{false};

        bool authorizeAndStage(CandidateCommandBundle&& candidate);

        bool stageCommandHistoryForCandidate(const ExpTraj& exp_traj);

        [[nodiscard]] std::optional<navigation_planning::CandidateBundle>
        exportStagedCommandCandidate(
            const CandidateCommandBundle& command,
            const CommandCertificate& certificate,
            std::uint64_t generation,
            std::uint64_t localization_epoch,
            std::uint64_t goal_epoch,
            std::uint64_t request_id,
            std::int64_t valid_from_ns,
            std::int64_t valid_until_ns) const;

        [[nodiscard]] CommandIdentity commandIdentitySnapshot() const {
            std::lock_guard<std::mutex> guard(command_identity_mutex_);
            return command_identity_;
        }

        PlannerResultCode classifySolveFailure(
            const AbsoluteDeadline &solve_deadline,
            bool elapsed_budget_exceeded = false,
            PlannerResultCode fallback = PLANNER_BACKUP_FAILED) const;

        [[nodiscard]] Vec3f resolveGoalForPlanning(const Vec3f& requested_goal);

    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        explicit Planner(const std::string &cfg_path,
                              const navigation_planner_context::PlannerRuntimeContext::Ptr &planner_context,
                              navigation_world_model::WorldModelViewPtr map_ptr,
                              const std::optional<navigation_planning::DynamicLimits> &mission_limits,
                              navigation_world_model::WorldCommitAuthorizer& commit_authorizer);

        ~Planner() = default;

        bool goalValid() const {
            return gi_.goal_valid;
        }

        typedef std::shared_ptr<Planner> Ptr;

        void getOneHeartbeatTime(double &start_WT_pos, bool &traj_finish);

        Trajectory getCommittedPositionTrajectory();

        Trajectory getCommittedYawTrajectory();

        // Runtime safety metadata for the currently committed command. These
        // accessors keep backend trajectory ownership private while allowing the
        // mission/PX4 FSM to validate its optimized safety suffix.
        bool committedBackupTrajectoryAvailable() const {
            return cmd_traj_info_.backupTrajectoryAvailable();
        }

        double getCommittedBackupStartTrajectoryTime() const {
            return cmd_traj_info_.getBackupTrajStartTT();
        }

        std::uint64_t getCommittedGeneration() const {
            return cmd_traj_info_.generation();
        }

        CommandCertificate getCommittedCertificate() const {
            return cmd_traj_info_.certificate();
        }

        CommittedTrajectorySnapshot committedTrajectorySnapshot() const {
            return cmd_traj_info_.snapshot();
        }

        std::uint64_t committedGenerationSnapshot() const {
            return cmd_traj_info_.generationSnapshot();
        }

        CommittedTrajectoryMetadata committedMetadataSnapshot() const {
            return cmd_traj_info_.metadataSnapshot();
        }

        Vec3f latestGuideStart() const { return latest_guide_start_; }
        Vec3f latestGuideEnd() const { return latest_guide_end_; }
        Vec3f latestGuideMin() const { return latest_guide_min_; }
        Vec3f latestGuideMax() const { return latest_guide_max_; }
        double latestGuidePathLengthMeters() const { return latest_guide_path_length_m_; }
        double latestGuideDurationSeconds() const { return latest_guide_duration_s_; }
        double requiredLookaheadMeters() const noexcept { return required_lookahead_m_; }
        double certifiedLookaheadMeters() const noexcept { return certified_lookahead_m_; }
        bool lookaheadComplete() const noexcept { return lookahead_complete_; }
        Vec3f requestedGoal() const { return requested_goal_p_; }
        Vec3f planningGoal() const { return planning_goal_p_; }
        double goalAcceptanceRadiusMeters() const noexcept {
            return goal_acceptance_radius_m_;
        }
        bool goalEndpointAdjusted() const noexcept { return goal_endpoint_adjusted_; }
        RouteYawReference routeYawReference() const noexcept {
            return route_yaw_reference_;
        }
        double yawRateLimitRadS() const noexcept { return cfg_.yaw_rate_max_rad_s; }
        double yawAccelerationLimitRadS2() const noexcept {
            return cfg_.yaw_acceleration_max_rad_s2;
        }
        double latestCandidateMaximumYawRateRadS() const noexcept {
            return latest_candidate_maximum_yaw_rate_rad_s_;
        }
        double latestCandidateMaximumYawAccelerationRadS2() const noexcept {
            return latest_candidate_maximum_yaw_acceleration_rad_s2_;
        }
        navigation_world_model::CellState requestedGoalInflatedState() const noexcept {
            return requested_goal_inflated_state_;
        }
        navigation_world_model::CellState planningGoalInflatedState() const noexcept {
            return planning_goal_inflated_state_;
        }
        navigation_planning::BackupCertificateDiagnostics backupCertificateDiagnostics() const noexcept {
            return backup_certificate_diagnostics_;
        }
        int solveStage() const noexcept {
            const int stage = solve_stage_.load();
            return stage == 3 && cg_ptr_ ? 30 + cg_ptr_->solveStage() : stage;
        }
        int latestReplanReturnCode() const noexcept {
            return latest_replan.getRetCode();
        }
        int latestCommitDecision() const noexcept {
            return latest_commit_decision_.load();
        }
        void resetExpOptimizationDiagnostics() noexcept {
            if (exp_traj_opt_) exp_traj_opt_->resetDiagnostics();
        }
        traj_opt::ExpOptimizationDiagnostics latestExpOptimizationDiagnostics() const noexcept {
            return exp_traj_opt_ ? exp_traj_opt_->diagnostics()
                                 : traj_opt::ExpOptimizationDiagnostics{};
        }
        double solveDeadlineSeconds() const noexcept {
            return cfg_.solve_deadline_s;
        }
        double replanForwardSeconds() const noexcept {
            return cfg_.replan_forward_dt_s;
        }
        double trackingErrorBudgetMeters() const noexcept {
            return cfg_.tracking_error_budget_m;
        }
        navigation_world_model::UnknownPolicy unknownPolicy() const noexcept {
            return cfg_.unknown_space_policy;
        }
        void resetSolveCancellation() noexcept {
            solve_cancelled_.store(false);
        }

        // The execution coordinator calls this only after its own atomic
        // candidate commit succeeds. The committed trajectory then becomes planning history,
        // never the source sampled by the command timer.
        bool acknowledgeCommandCandidate(std::uint64_t generation);
        void discardCommandCandidate() noexcept;
        [[nodiscard]] bool hasStagedCommandCandidate() const {
            std::lock_guard<std::mutex> guard(solve_commit_mutex_);
            return staged_command_candidate_.has_value();
        }

        // Runtime sets the immutable mission identity before a solve starts.
        // The identity is copied into the backend candidate and checked again
        // at export; callers cannot relabel a trajectory after it is planned.
        void setCommandIdentity(const CommandIdentity& identity) {
            if (!identity.valid()) {
                throw std::invalid_argument("command identity must be non-zero");
            }
            {
                std::lock_guard<std::mutex> guard(solve_commit_mutex_);
                staged_command_candidate_.reset();
            }
            std::lock_guard<std::mutex> guard(command_identity_mutex_);
            command_identity_ = identity;
        }

        // Planning-thread-only. Runtime pins one immutable revision before a
        // solve; A* and corridor generation receive that same pointer.
        void setWorldModelView(navigation_world_model::WorldModelViewPtr view) {
            if (!view) throw std::invalid_argument("WorldModelView must not be null");
            map_ptr_ = std::move(view);
            astar_ptr_->setWorldModelView(map_ptr_);
            cg_ptr_->setWorldModelView(map_ptr_);
        }

        // Runtime supplies the mission-owned waypoint acceptance radius before
        // each solve. This never changes collision, UNKNOWN or OUT_OF_MAP
        // policy; it only bounds endpoint representation.
        void setGoalAcceptanceRadius(const double radius_m) noexcept {
            goal_acceptance_radius_m_ =
                std::isfinite(radius_m) && radius_m > 0.0
                    ? std::max(radius_m,
                               navigation_world_model::kGoalCompletionToleranceM)
                    : navigation_world_model::kGoalCompletionToleranceM;
        }

        // Planning-thread-only mission look-ahead. A pass-through goal uses
        // this only to shape its terminal velocity; the current waypoint
        // remains the geometric endpoint and all safety certificates remain
        // authoritative.
        void setPassThroughNextTarget(
                const std::optional<Eigen::Vector3d>& next_target) noexcept {
            if (next_target.has_value() && next_target->allFinite()) {
                pass_through_next_target_ = *next_target;
                pass_through_coincident_terminal_stop_ = false;
            } else {
                pass_through_next_target_.reset();
            }
        }

        // Planning-thread-only immutable mission route. Mission, planner
        // endpoint/tangent selection and diagnostics share this identity;
        // M2 will drive yaw from its route lookahead. Compatibility mirrors
        // are never authoritative here.
        bool setRouteSnapshot(
            const navigation_mission::ImmutableRouteSnapshot& route) noexcept {
            if (!route.valid()) {
                route_snapshot_.reset();
                pass_through_next_target_.reset();
                last_route_yaw_target_rad_.reset();
                terminal_stop_required_ = false;
                pass_through_coincident_terminal_stop_ = false;
                return false;
            }
            if (!route_snapshot_.has_value() ||
                route_snapshot_->mission_id != route.mission_id ||
                route_snapshot_->route_revision != route.route_revision ||
                route_snapshot_->request_id != route.request_id ||
                route_snapshot_->active_waypoint_index != route.active_waypoint_index) {
                last_route_yaw_target_rad_.reset();
                acceptance_endpoint_lease_.reset();
                acceptance_endpoint_lease_request_id_ = 0U;
                acceptance_endpoint_lease_route_revision_ = 0U;
                acceptance_endpoint_lease_waypoint_index_ = 0U;
            }
            route_snapshot_ = route;
            pass_through_coincident_terminal_stop_ =
                navigation_mission::passThroughNextWaypointIsCoincidentStop(route);
            terminal_stop_required_ =
                route.waypoints[route.active_waypoint_index].behavior ==
                    navigation_mission::MissionWaypoint::Behavior::Stop ||
                pass_through_coincident_terminal_stop_;
            const std::size_t next_index = route.active_waypoint_index + 1U;
            if (route.waypoints[route.active_waypoint_index].behavior ==
                    navigation_mission::MissionWaypoint::Behavior::PassThrough &&
                next_index < route.waypoints.size()) {
                if (pass_through_coincident_terminal_stop_) {
                    pass_through_next_target_.reset();
                } else {
                    pass_through_next_target_ =
                        route.waypoints[next_index].position_enu;
                }
            } else {
                pass_through_next_target_.reset();
            }
            return true;
        }

        bool updateRouteYawReference() noexcept;

        void cancelActiveSolve() {
            std::lock_guard<std::mutex> guard(solve_commit_mutex_);
            solve_cancelled_.store(true);
        }
        std::size_t solvePointCount() const noexcept {
            return cg_ptr_ ? cg_ptr_->solvePointCount() : 0;
        }

        void getOneCommandFromTraj(StatePVAJ &pvaj,
                                   double &yaw,
                                   double &yaw_dot,
                                   bool &on_backup_traj,
                                   bool &traj_finish);

        enum class TrajectoryRole : std::uint8_t { MAIN = 0, BACKUP = 1 };
        struct CommandSample {
            StatePVAJ pvaj{StatePVAJ::Zero()};
            double yaw{0.0};
            double yaw_rate{0.0};
            TrajectoryRole role{TrajectoryRole::MAIN};
            bool finished{false};
            bool valid{false};
            std::uint64_t generation{0};
            double trajectory_time_s{0.0};
            navigation_world_model::WorldSnapshotIdentity certificate_world{};
        };
        CommandSample sampleCommand();

        // Export one immutable product candidate for the execution boundary.
        // The backend retains its private trajectory representation; callers do
        // not sample or lock the backend's mutable command state directly.
        std::optional<navigation_planning::CandidateBundle> exportCommandCandidate(
            std::uint64_t localization_epoch,
            std::uint64_t goal_epoch,
            std::uint64_t request_id,
            std::int64_t valid_from_ns,
            std::int64_t valid_until_ns) const;

        // Last-resort planner-owned braking bundle.  This is intentionally not
        // a main-only adapter trajectory: it is committed as BACKUP only
        // after dynamic and inflated-map gates pass.
        bool commitEmergencyBrake(const StatePVAJ &initial_command_state,
                                  double initial_command_yaw,
                                  double initial_command_yaw_dot,
                                  double start_WT);

        void getModuleTimeConsuming(vector<double> &time);
        vector<double> moduleTimeConsumingSnapshot() const;

        /* Tow type of replan strategy */
        RET_CODE PlanFromRest(const Vec3f &goal_p,
                              const double &goal_yaw,
                              const bool &new_goal);

        RET_CODE
        ReplanOnce(const Vec3f &goal_p,
                   const double &goal_yaw,
                   const bool &new_goal);

    private:
        RET_CODE generateExpTraj(ExpTraj &last_exp_traj_info,
                                 ExpTraj &out_exp_traj_info,
                                 const AbsoluteDeadline &solve_deadline);

        /* For Backup traj generation */
        RET_CODE generateBackupTrajectory(ExpTraj &ref_exp_traj,
                                          BackupTraj &back_traj_info,
                                          const AbsoluteDeadline &solve_deadline);

        int getNearestFurtherGoalPoint(const vec_E<Vec3f> &goals, const Vec3f &start_pt);

        bool PathSearch(const Vec3f &start_pt, const Vec3f &goal,
                        const double &searching_horizon,
                        vec_Vec3f &path,
                        const AbsoluteDeadline &solve_deadline,
                        bool allow_partial_route = false);


    public:
        bool setState(const navigation_planning::KinematicState &state);
        // Planning-thread-only recovery envelope. A value below one is a
        // conservative speed request, never a relaxation of V/A/J limits.
        void setRecoveryVelocityScale(double scale) noexcept;

        bool isEasyGoal(const Vec3f &goal_position);

        double ft{0}, bt{0};
        int ft_cnt{0}, bt_cnt{0};

        double getFrontendTime() {
            if (ft_cnt == 0) return -1;
            double ave_t = ft / ft_cnt;
            ft = 0;
            ft_cnt = 0;
            return ave_t;
        }

        double getBackendTime() {
            if (bt_cnt == 0) return -1;
            double ave_t = bt / bt_cnt;
            bt = 0;
            bt_cnt = 0;
            return ave_t;
        }

        LogOneReplan getLatestReplanLog() {
            latest_replan.setSfcPc(cg_ptr_->getLatestCloud());
            latest_replan.setComptT(time_consuming_);
            return latest_replan;
        }
    };
}
