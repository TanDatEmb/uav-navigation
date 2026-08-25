/**
* This file is part of SUPER
*
* Copyright 2025 Yunfan REN, MaRS Lab, University of Hong Kong, <mars.hku.hk>
* Developed by Yunfan REN <renyf at connect dot hku dot hk>
* for more information see <https://github.com/hku-mars/SUPER>.
* If you use this code, please cite the respective publications as
* listed on the above website.
*
* SUPER is free software: you can redistribute it and/or modify
* it under the terms of the GNU Lesser General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* SUPER is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU Lesser General Public License
* along with SUPER. If not, see <http://www.gnu.org/licenses/>.
*/

#include <super_core/super_planner.h>
#include <super_core/absolute_deadline.hpp>
#include <super_core/backup_braking.hpp>
#include <super_core/command_time.hpp>
#include <super_core/guide_endpoint.hpp>
#include <super_core/replan_contract.hpp>
#include <super_core/trajectory_world_validator.hpp>
#include <navigation_world_model/goal_contract.hpp>
#include <traj_opt/trajectory_dynamics.hpp>
#include <cmath>
#include <memory>
#include <stdexcept>
#include <super_utils/scope_timer.hpp>
#include <fmt/color.h>

using namespace super_utils;
using std::isnan;

namespace super_planner {

    SUPER_RET_CODE SuperPlanner::classifySolveFailure(
            const AbsoluteDeadline &solve_deadline,
            const bool elapsed_budget_exceeded) const {
        if (solve_cancelled_.load(std::memory_order_relaxed)) {
            return SUPER_SOLVE_CANCELLED;
        }
        if (elapsed_budget_exceeded ||
            solve_deadline.expired(ros_ptr_->getSimTime()) ||
            solve_deadline.steadyExpired()) {
            return SUPER_SOLVE_TIMEOUT;
        }
        return SUPER_BACKUP_FAILED;
    }
    SuperPlanner::SuperPlanner
            (const std::string &cfg_path,
             const ros_interface::RosInterface::Ptr &ros_ptr,
             navigation_world_model::WorldModelViewPtr map_ptr,
             const std::optional<DynamicLimits> &mission_limits,
             navigation_world_model::WorldCommitAuthorizer& commit_authorizer
            ) : cfg_(Config(cfg_path, mission_limits)), map_ptr_(std::move(map_ptr)),
                commit_authorizer_(&commit_authorizer), ros_ptr_(ros_ptr) {

        ros_ptr_->setResolution(cfg_.resolution);
        ros_ptr_->setVisualizationEn(cfg_.visualization_en);
        exp_traj_opt_ = std::make_shared<traj_opt::ExpTrajOpt>(cfg_.exp_traj_cfg, ros_ptr_);
        back_traj_opt_ = std::make_shared<traj_opt::BackupTrajOpt>(cfg_.back_traj_cfg, ros_ptr_);
        yaw_traj_opt_ = std::make_shared<traj_opt::YawTrajOpt>(cfg_.yaw_dot_max);
        const auto world_geometry = map_ptr_->geometry();
        const double occupied_inflation_radius = world_geometry.occupied_inflation_radius_m;
        if (occupied_inflation_radius + 1.0e-9 < cfg_.robot_r) {
            throw std::invalid_argument(
                    "ROG-Map occupied inflation radius is smaller than SUPER robot_r");
        }
        astar_ptr_ = std::make_shared<path_search::Astar>(cfg_path, ros_ptr_, map_ptr_);
        cg_ptr_ = std::make_shared<CorridorGenerator>(ros_ptr_, map_ptr_, cfg_.corridor_bound_dis,
                                                      cfg_.corridor_line_max_length,
                                                      cfg_.resolution,
                                                      world_geometry.effective_virtual_ground_m,
                                                      world_geometry.effective_virtual_ceiling_m,
                                                      cfg_.robot_r,
                                                      cfg_.obs_skip_num,
                                                      cfg_.iris_iter_num);
        cg_ptr_->SetLineNeighborList(cfg_.seed_line_neighbour);


        time_consuming_.resize(8);

        robot_state_.rcv = false;
        planner_process_start_WT_ = ros_ptr_->getSimTime();
        fov_checker_ = std::make_shared<FOVChecker>(FOVType::OMNI,
                                                    -1.0,
                                                    -35.0,
                                                    35.0);

        const int neighbor_step = floor(cfg_.robot_r / cfg_.resolution);
        astar_ptr_->setFineInfNeighbors(neighbor_step);
    }

    bool SuperPlanner::authorizeAndCommit(CandidateCommandBundle&& candidate) {
        if (commit_authorizer_ == nullptr || !map_ptr_) {
            latest_commit_decision_.store(static_cast<int>(
                navigation_world_model::WorldCommitDecision::kCandidateRejected));
            ros_ptr_->error(" -- [SUPER] command rejected: no WorldModel commit authorizer");
            return false;
        }
        const auto pinned_identity = map_ptr_->identity();
        const auto lease = commit_authorizer_->latest();
        if (!lease || lease.identity.generation != pinned_identity.generation) {
            latest_commit_decision_.store(static_cast<int>(
                lease ? navigation_world_model::WorldCommitDecision::kWorldAdvanced
                      : navigation_world_model::WorldCommitDecision::kNoPublishedWorld));
            ros_ptr_->warn(" -- [SUPER] command rejected: WorldModel generation changed");
            return false;
        }
        const double authorization_wall_time = ros_ptr_->getSimTime();
        const auto validation = validateExecutableCandidate(
            *lease.view, candidate, authorization_wall_time);
        if (!validation.valid) {
            latest_commit_decision_.store(static_cast<int>(
                navigation_world_model::WorldCommitDecision::kCandidateRejected));
            ros_ptr_->warn(
                " -- [SUPER] command rejected by latest WorldModel at trajectory time {}",
                validation.first_blocked_tt);
            return false;
        }
        const CommandCertificate certificate{
            pinned_identity, lease.identity, validation.begin_tt};
        const auto decision = commit_authorizer_->commitIfCurrent(
            lease.identity, [&]() {
                std::lock_guard<std::mutex> commit_guard(solve_commit_mutex_);
                if (solve_cancelled_.load()) return false;
                return cmd_traj_info_.commitCandidate(
                    std::move(candidate), certificate);
            });
        latest_commit_decision_.store(static_cast<int>(decision));
        if (decision != navigation_world_model::WorldCommitDecision::kCommitted) {
            ros_ptr_->warn(" -- [SUPER] command authorization rejected with reason {}",
                           static_cast<int>(decision));
            return false;
        }
        return true;
    }

    RET_CODE
    SuperPlanner::PlanFromRest(const Vec3f &goal_p,
                               const double &goal_yaw,
                               const bool &new_goal) {
        std::lock_guard<std::mutex> guard(replan_lock_);
        const AbsoluteDeadline solve_deadline(
                ros_ptr_->getSimTime(), cfg_.solve_deadline_s);
        solve_stage_.store(1);
        latest_commit_decision_.store(static_cast<int>(
            navigation_world_model::WorldCommitDecision::kNotAttempted));
        latest_replan.reset();
        latest_replan.setGoal(goal_p, goal_yaw, robot_state_);
        if (robot_state_.rcv == false) {
            ros_ptr_->warn(" -- [SUPER] in [PlanFromRest]: No odom, force return.");
            latest_replan.setRetCode(SUPER_RET_CODE::SUPER_NO_ODOM);
            return FAILED;
        }
        gi_.goal_p = goal_p;
        gi_.goal_yaw = goal_yaw;
        gi_.new_goal = new_goal;
        gi_.goal_valid = true;
        vec_Vec3f viz_pts{goal_p, robot_state_.p};

        {
            TimeConsuming t_viz("viz goal path", false);
            ros_ptr_->vizGoalPath(viz_pts);
            time_consuming_[VISUALIZATION] += t_viz.stop();
        }


        /// 1) First, shift the start_point to free space.
        const auto nearest_start = map_ptr_->nearestNotOccupied(
                robot_state_.p, navigation_world_model::GridLayer::kEvidence, 3.0);
        if (!nearest_start) {
            ros_ptr_->error(
                    " -- [SUPER] in [PlanFromRest] Local start point is deeply occupied, which should not happened.");
            latest_replan.setRetCode(SUPER_RET_CODE::SUPER_NO_START_POINT);
            return FAILED;
        }
        Vec3f local_star_pt = *nearest_start;
        latest_replan.setLocalStartP(local_star_pt);

        /// 2) Generate Exp traj
        ExpTraj exp_traj_info;
        BackupTraj back_traj_info;
        last_exp_traj_info_.setEmpty();
        local_start_p_ = local_star_pt;
        ExpTraj previous_exp_snapshot = last_exp_traj_info_;
        RET_CODE exp_ret_code = generateExpTraj(
                previous_exp_snapshot, exp_traj_info, solve_deadline);
        //GenerateRestToRestExpTraj(local_star_pt, exp_traj_info);
        if (exp_ret_code == FAILED) {
            if (solve_cancelled_.load() || solve_deadline.expired(ros_ptr_->getSimTime()) ||
                solve_deadline.steadyExpired()) {
                latest_replan.setRetCode(classifySolveFailure(solve_deadline));
            }
            ros_ptr_->warn(" -- [SUPER] in [PlanFromRest] GenerateExpTrajectory failed with {}.",
                           RET_CODE_STR[exp_ret_code].c_str());
            return FAILED;
        } else {
            ros_ptr_->info(" -- [SUPER] in [PlanFromRest] GenerateExpTrajectory SUCCESS.");
        }

        back_traj_info.setEmpty();
        solve_stage_.store(5);
        if (solve_deadline.expired(ros_ptr_->getSimTime()) ||
            solve_deadline.steadyExpired()) {
            ros_ptr_->warn(" -- [SUPER] solve deadline exhausted before backup stage");
            latest_replan.setRetCode(classifySolveFailure(solve_deadline));
            return FAILED;
        }
        RET_CODE back_ret_code = generateBackupTrajectory(
                exp_traj_info, back_traj_info, solve_deadline);

        if (solve_cancelled_.load() ||
            solve_deadline.expired(ros_ptr_->getSimTime()) ||
            solve_deadline.steadyExpired()) {
            if (!solve_cancelled_.load()) {
                ros_ptr_->warn(" -- [SUPER] solve deadline exhausted during backup stage");
            }
            latest_replan.setRetCode(classifySolveFailure(solve_deadline));
            return FAILED;
        }

        if (!backupResultMayBuildCommandCandidate(back_ret_code)) {
            latest_replan.setRetCode(SUPER_RET_CODE::SUPER_BACKUP_FAILED);
            ros_ptr_->warn(
                " -- [SUPER] in [PlanFromRest]: backup result is not executable; "
                "leaving CmdTraj unchanged");
            return FAILED;
        }

        if (back_ret_code == SUCCESS) {
            if (cfg_.print_log) {
                ros_ptr_->info(" -- [SUPER] in [PlanFromRest] generateBackupTrajectory SUCCESS.");
            }

            auto candidate = CmdTraj::buildCandidate(
                exp_traj_info, &back_traj_info, BackupDisposition::SUCCESS);
            if (!candidate || !authorizeAndCommit(std::move(*candidate))) {
                latest_replan.setRetCode(classifySolveFailure(solve_deadline));
                return FAILED;
            }
            last_exp_traj_info_ = exp_traj_info;
            gi_.new_goal = false;

            // For visualization
            {
                TimeConsuming t_viz("viz goal VisualizeCommitTrajectory", false);
                ros_ptr_->vizCommittedTraj(cmd_traj_info_.posTraj(), cmd_traj_info_.getBackupTrajStartTT());
                time_consuming_[VISUALIZATION] += t_viz.stop();
                latest_replan.setRetCode(SUPER_RET_CODE::SUPER_SUCCESS_WITH_BACKUP);
            }

            return SUCCESS;
        } else if (back_ret_code == FINISH || back_ret_code == NO_NEED) {
            if (cfg_.print_log) {
                ros_ptr_->info(" -- [SUPER] in [PlanFromRest] generateBackupTrajectory Finish or NO_NEED.");
            }
            const auto disposition = back_ret_code == FINISH
                ? BackupDisposition::FINISH : BackupDisposition::NO_NEED;
            auto candidate = CmdTraj::buildCandidate(
                exp_traj_info, nullptr, disposition);
            if (!candidate || !authorizeAndCommit(std::move(*candidate))) {
                latest_replan.setRetCode(classifySolveFailure(solve_deadline));
                return FAILED;
            }
            last_exp_traj_info_ = exp_traj_info;
            gi_.new_goal = false;

            // For visualization
            TimeConsuming t_viz("viz goal VisualizeCommitTrajectory", false);
            {
                ros_ptr_->vizCommittedTraj(cmd_traj_info_.posTraj(), -1);
                time_consuming_[VISUALIZATION] += t_viz.stop();
            }
            latest_replan.setRetCode(SUPER_RET_CODE::SUPER_SUCCESS_NO_BACKUP);
            return SUCCESS;
        }
        // A candidate is not safe to commit without a feasible backup.  Keep
        // the previously committed atomic bundle so the runtime can drain its
        // existing safety suffix or enter its fail-closed handover path.
        ros_ptr_->warn(
                " -- [SUPER] in [PlanFromRest] backup generation returned [{}]; "
                "rejecting candidate because backup is required",
                RET_CODE_STR[back_ret_code].c_str());
        latest_replan.setRetCode(SUPER_RET_CODE::SUPER_BACKUP_FAILED);
        return FAILED;
    }


    RET_CODE
    SuperPlanner::ReplanOnce(const Vec3f &goal_p,
                             const double &goal_yaw,
                             const bool &new_goal) {
        TimeConsuming replan_total_t("ReplanOnce", false);
        std::lock_guard<std::mutex> guard(replan_lock_);
        const AbsoluteDeadline solve_deadline(
                ros_ptr_->getSimTime(), cfg_.solve_deadline_s);
        solve_stage_.store(1);
        latest_commit_decision_.store(static_cast<int>(
            navigation_world_model::WorldCommitDecision::kNotAttempted));

        gi_.goal_p = goal_p;
        gi_.goal_yaw = goal_yaw;
        gi_.new_goal = new_goal;
        gi_.goal_valid = true;
        latest_replan.reset();
        latest_replan.setGoal(goal_p, goal_yaw, robot_state_);

        vec_Vec3f viz_pts{goal_p, robot_state_.p};

        {
            TimeConsuming t_viz("tviz", false);
            ros_ptr_->vizGoalPath(viz_pts);
            time_consuming_[VISUALIZATION] += t_viz.stop();
        }


        /// 1) Replan EXP traj
        ExpTraj exp_traj_info;
        TimeConsuming t_exp("t_exp", false);
        ExpTraj previous_exp_snapshot = last_exp_traj_info_;
        RET_CODE exp_ret_code = generateExpTraj(
                previous_exp_snapshot, exp_traj_info, solve_deadline);
        time_consuming_[GENERATE_EXP_TRAJ] = t_exp.stop();

        if (exp_ret_code == FAILED) {
            if (solve_cancelled_.load() || solve_deadline.expired(ros_ptr_->getSimTime()) ||
                solve_deadline.steadyExpired()) {
                latest_replan.setRetCode(classifySolveFailure(solve_deadline));
            }
            ros_ptr_->warn(" -- [SUPER] in [ReplanOnce]: GenerateExpTrajectory failed, force return");
            return FAILED;
        } else if (exp_ret_code == NEW_TRAJ) {
            if (cfg_.print_log) {
                ros_ptr_->info(" -- [SUPER] in [ReplanOnce]: Last epx traj end, switch to new traj.");
            }
            return NEW_TRAJ;
        } else if (exp_ret_code == EMER) {
            ros_ptr_->warn(" -- [SUPER] in [ReplanOnce]: Replan failed, switch to emer.");
            return EMER;
        } else if (exp_ret_code == SUCCESS) {
            if (cfg_.print_log) {
                ros_ptr_->info(" -- [SUPER] in [ReplanOnce]: Replan a new exp traj success.");
            }
        } else if (exp_ret_code == NO_NEED &&
                   !expResultMayBuildCommandCandidate(exp_ret_code)) {
            // generateExpTraj returns the historical EXP snapshot for NO_NEED.
            // It is planner history, not a new executable candidate.  Keep the
            // currently committed immutable command bundle unchanged and let
            // the runtime revalidate/expose that bundle as the retained command.
            if (cfg_.print_log) {
                ros_ptr_->info(
                    " -- [SUPER] in [ReplanOnce]: No new EXP trajectory is needed; "
                    "retain the committed command without generating or committing backup.");
            }
            latest_replan.setRetCode(SUPER_RET_CODE::SUPER_SUCCESS);
            return NO_NEED;
        }

        {
            TimeConsuming t_viz("tviz", false);
            ros_ptr_->vizYawTraj(exp_traj_info.posTraj(), exp_traj_info.yawTraj());
            time_consuming_[VISUALIZATION] += t_viz.stop();
        }


        BackupTraj back_traj_info;
        // 2）生成back轨迹
        solve_stage_.store(5);
        TimeConsuming t_back("t_back", false);
        if (solve_deadline.expired(ros_ptr_->getSimTime()) ||
            solve_deadline.steadyExpired()) {
            ros_ptr_->warn(" -- [SUPER] solve deadline exhausted before backup stage");
            latest_replan.setRetCode(classifySolveFailure(solve_deadline));
            return FAILED;
        }
        RET_CODE back_ret_code = generateBackupTrajectory(
                exp_traj_info, back_traj_info, solve_deadline);
        time_consuming_[GENERATE_BACK_TRAJ] = t_back.stop();

        {
            ft += time_consuming_[EPX_TRAJ_FRONTEND] + time_consuming_[BACK_TRAJ_FRONTEND];
            ft_cnt++;
            bt += time_consuming_[BACK_TRAJ_OPT] + time_consuming_[EXP_TRAJ_OPT];
            bt_cnt++;
        }

        double replan_dt = replan_total_t.stop();
        if (solve_deadline.expired(ros_ptr_->getSimTime()) ||
            solve_deadline.steadyExpired() ||
            replan_dt > cfg_.solve_deadline_s) {
            ros_ptr_->warn(" -- [SUPER] in [ReplanOnce]: Replan overtime, check parameters, replan dt = {}.", replan_dt);
            latest_replan.setRetCode(classifySolveFailure(
                solve_deadline, replan_dt > cfg_.solve_deadline_s));
            return FAILED;
        }

        if (solve_cancelled_.load()) {
            latest_replan.setRetCode(classifySolveFailure(solve_deadline));
            return FAILED;
        }

        if (!backupResultMayBuildCommandCandidate(back_ret_code)) {
            latest_replan.setRetCode(SUPER_RET_CODE::SUPER_BACKUP_FAILED);
            ros_ptr_->warn(
                " -- [SUPER] in [ReplanOnce]: backup result is not executable; "
                "leaving CmdTraj unchanged");
            return FAILED;
        }

        if (back_ret_code == SUCCESS) {
            auto candidate = CmdTraj::buildCandidate(
                exp_traj_info, &back_traj_info, BackupDisposition::SUCCESS);
            if (!candidate || !authorizeAndCommit(std::move(*candidate))) {
                latest_replan.setRetCode(classifySolveFailure(solve_deadline));
                return FAILED;
            }
            last_exp_traj_info_ = exp_traj_info;
            gi_.new_goal = false;

            {
                // For visualization
                TimeConsuming t_viz("tviz", false);
                ros_ptr_->vizCommittedTraj(cmd_traj_info_.posTraj(), cmd_traj_info_.getBackupTrajStartTT());
                time_consuming_[VISUALIZATION] += t_viz.stop();
            }

            latest_replan.setRetCode(SUPER_SUCCESS_WITH_BACKUP);
            if (cfg_.print_log)
                ros_ptr_->info(" -- [SUPER] in [ReplanOnce]: Replan a new back traj success, all replan success.");
            return SUCCESS;
        } else if (back_ret_code == NO_NEED) {
            // 这次生成backup轨迹的点没有意义,
            auto candidate = CmdTraj::buildCandidate(
                exp_traj_info, nullptr, BackupDisposition::NO_NEED);
            if (!candidate || !authorizeAndCommit(std::move(*candidate))) {
                latest_replan.setRetCode(classifySolveFailure(solve_deadline));
                return FAILED;
            }
            last_exp_traj_info_ = exp_traj_info;
            gi_.new_goal = false;


            {
                TimeConsuming t_viz("tviz", false);
                ros_ptr_->vizCommittedTraj(cmd_traj_info_.posTraj(), -1);
                time_consuming_[VISUALIZATION] += t_viz.stop();

            }

            if (cfg_.print_log)
                ros_ptr_->info(" -- [SUPER] in [ReplanOnce]: No need back traj success, all replan success.");
            latest_replan.setRetCode(SUPER_SUCCESS_NO_BACKUP);
            return SUCCESS;
        } else if (back_ret_code == FINISH) {
            // Which means the exp traj is all in known free, no need for backup traj
            auto candidate = CmdTraj::buildCandidate(
                exp_traj_info, nullptr, BackupDisposition::FINISH);
            if (!candidate || !authorizeAndCommit(std::move(*candidate))) {
                latest_replan.setRetCode(classifySolveFailure(solve_deadline));
                return FAILED;
            }
            last_exp_traj_info_ = exp_traj_info;
            gi_.new_goal = false;

            {
                TimeConsuming t_viz("tviz", false);
                ros_ptr_->vizCommittedTraj(cmd_traj_info_.posTraj(), -1);
                time_consuming_[VISUALIZATION] += t_viz.stop();
            }

            if (cfg_.print_log)
                ros_ptr_->info(" -- [SUPER] in [ReplanOnce]: No need back traj success, all replan success.");
            latest_replan.setRetCode(SUPER_SUCCESS_NO_BACKUP);
            return SUCCESS;
        }
        // Main and backup form one atomic safety bundle.  A failed backup
        // must leave CmdTraj untouched so the runtime can retain/drain the
        // previously committed suffix instead of executing main-only EXP.
        ros_ptr_->warn(
                " -- [SUPER] in [ReplanOnce]: backup generation returned {}; "
                "rejecting candidate because backup is required",
                RET_CODE_STR[back_ret_code].c_str());
        latest_replan.setRetCode(SUPER_RET_CODE::SUPER_BACKUP_FAILED);
        return FAILED;
    }

    void SuperPlanner::getOneHeartbeatTime(double &start_WT_pos, bool &traj_finish) {
        cmd_traj_info_.lock();
        double eval_t = (ros_ptr_->getSimTime() - cmd_traj_info_.getStartWallTime());
        traj_finish = false;
        double total_dur = cmd_traj_info_.getTotalDuration();
        if (eval_t > total_dur) {
            traj_finish = true;
            eval_t = total_dur;
        }
        start_WT_pos = cmd_traj_info_.getStartWallTime();
        cmd_traj_info_.unlock();
    }

    Trajectory SuperPlanner::getCommittedPositionTrajectory() {
        return cmd_traj_info_.posTraj();
    }

    Trajectory SuperPlanner::getCommittedYawTrajectory() {
        return cmd_traj_info_.yawTraj();
    }


    void SuperPlanner::getOneCommandFromTraj(StatePVAJ &pvaj,
                                             double &yaw,
                                             double &yaw_dot,
                                             bool &on_backup_traj,
                                             bool &traj_finish) {
        const auto sample = sampleCommand();
        pvaj = sample.pvaj;
        yaw = sample.yaw;
        yaw_dot = sample.yaw_rate;
        on_backup_traj = sample.role == TrajectoryRole::BACKUP;
        traj_finish = sample.finished;
    }

    SuperPlanner::CommandSample SuperPlanner::sampleCommand() {
        CommandSample sample;
        cmd_traj_info_.lock();
        const double &cur_t = ros_ptr_->getSimTime();
        const double &cmd_start_WT = cmd_traj_info_.getStartWallTime();
//        const bool &backup_avilibale = cmd_traj_info_.backupTrajAvilibale();
//        const double &backup_start_TT = cmd_traj_info_.getBackupTrajStartTT();
        const double &total_dur = cmd_traj_info_.getTotalDuration();

        const auto command_time = commandTrajectoryTime(cur_t, cmd_start_WT, total_dur);
        sample.finished = command_time.finished;
        const double eval_t = command_time.trajectory_time_s;
        sample.trajectory_time_s = eval_t;
        sample.role = cmd_traj_info_.isTTOnBackupTraj(eval_t)
                          ? TrajectoryRole::BACKUP : TrajectoryRole::MAIN;
        sample.pvaj = cmd_traj_info_.posTraj().getState(eval_t);
        sample.yaw = cmd_traj_info_.getYaw(eval_t)[0];
        sample.yaw_rate = cmd_traj_info_.getYawRate(eval_t)[0];
        sample.generation = cmd_traj_info_.generation();
        sample.certificate_world = cmd_traj_info_.certificate().validated_world;
        sample.valid = sample.pvaj.allFinite() && std::isfinite(sample.yaw) &&
                       std::isfinite(sample.yaw_rate) &&
                       std::isfinite(sample.trajectory_time_s);

//        if (last_round_robot_on_backup_traj != robot_on_backup_traj_) {
//            if (last_round_robot_on_backup_traj) {
//                ros_ptr_->info(" -- [CMD] Emergency Stop End ========================");
//            } else {
//                ros_ptr_->info(" -- [CMD] Emergency Stop Start ========================");
//            }
//        }

//        double cur_yaw = geometry_utils::get_yaw_from_quaternion(robot_state_.q);
        cmd_traj_info_.unlock();
        return sample;
    }

    bool SuperPlanner::commitEmergencyBrake(const StatePVAJ &measured_state,
                                            const double measured_yaw,
                                            const double measured_yaw_dot,
                                            const double start_WT) {
        if (!measured_state.allFinite() || !std::isfinite(measured_yaw) ||
            !std::isfinite(measured_yaw_dot) || !std::isfinite(start_WT)) {
            ros_ptr_->error(" -- [SUPER] emergency brake rejected: non-finite initial state");
            return false;
        }

        StatePVAJ bounded_state = measured_state;
        const auto clamp_state_column = [&bounded_state](const int column,
                                                        const double limit) {
            const double magnitude = bounded_state.col(column).norm();
            if (std::isfinite(magnitude) && magnitude > limit) {
                bounded_state.col(column) *= limit / magnitude;
            }
        };
        // Position and velocity are measured and must remain continuous at the
        // handover. Only the unmeasured command-boundary acceleration and jerk
        // may be saturated to the mission envelope.
        clamp_state_column(2, cfg_.back_traj_cfg.max_acc);
        clamp_state_column(3, cfg_.back_traj_cfg.max_jerk);

        const auto seed = makeBackupBrakingSeed(
                0.0, bounded_state,
                cfg_.back_traj_cfg.max_vel,
                cfg_.back_traj_cfg.max_acc,
                cfg_.back_traj_cfg.max_jerk,
                cfg_.sample_traj_dt,
                0.0);
        if (!seed.feasible || !std::isfinite(seed.duration_s) ||
            seed.duration_s <= 0.0) {
            ros_ptr_->error(" -- [SUPER] emergency brake rejected: no feasible PVAJ stop seed");
            return false;
        }

        Trajectory position_trajectory;
        position_trajectory.emplace_back(
                minimumSnapStopPiece(bounded_state, seed.duration_s));
        position_trajectory.start_WT = start_WT;

        StatePVAJ yaw_state = StatePVAJ::Zero();
        yaw_state(0, 0) = measured_yaw;
        yaw_state(0, 1) = measured_yaw_dot;
        Trajectory yaw_trajectory;
        yaw_trajectory.emplace_back(
                minimumSnapStopPiece(yaw_state, seed.duration_s));
        yaw_trajectory.start_WT = start_WT;

        traj_opt::TrajectoryDynamicReport dynamic_report;
        if (!traj_opt::trajectorySatisfiesFlatnessEnvelope(
                position_trajectory, cfg_.back_traj_cfg, &dynamic_report,
                0.005, &yaw_trajectory)) {
            ros_ptr_->error(
                    " -- [SUPER] emergency brake dynamic gate failed: body_rate={} thrust=[{},{}]",
                    dynamic_report.maximum_body_rate_rad_s,
                    dynamic_report.minimum_thrust_n,
                    dynamic_report.maximum_thrust_n);
            return false;
        }

        auto candidate = CmdTraj::buildEmergencyCandidate(
            position_trajectory, yaw_trajectory);
        if (!candidate || !authorizeAndCommit(std::move(*candidate))) {
            ros_ptr_->error(" -- [SUPER] emergency brake atomic commit rejected");
            return false;
        }
        ros_ptr_->warn(
                " -- [SUPER] committed measured-state emergency brake: "
                "initial_speed={} speed_limit={} initial_overspeed={} duration={} distance={} "
                "peak_vel={} peak_acc={} peak_jerk={}",
                seed.initial_velocity_mps, cfg_.back_traj_cfg.max_vel,
                seed.initial_overspeed,
                seed.duration_s,
                (seed.endpoint - bounded_state.col(0)).norm(),
                seed.maximum_velocity_mps, seed.maximum_acceleration_mps2,
                seed.maximum_jerk_mps3);
        return true;
    }


    void SuperPlanner::getModuleTimeConsuming(vector<double> &time) {
        time = time_consuming_;
        std::fill(time_consuming_.begin(), time_consuming_.end(), 0);
    }


    RET_CODE SuperPlanner::generateExpTraj(
            ExpTraj &last_exp_traj_info, ExpTraj &out_exp_traj_info,
            const AbsoluteDeadline &solve_deadline) {
        /* 1) Log the exp traj frontend time*/
        TimeConsuming t_exp_frontend("t_exp_frontend", false);

        // use hot init or not, just prepare a guide path, a guide t, init and fina state and sfc for exp traj opt
        StatePVAJ pos_init_state, pos_fina_state;
        PolytopeVec sfc;
        vec_Vec3f guide_path;
        // the guide_stamp saves a TT
        vector<double> guide_stamp;
        double guide_path_end_vel{0.0};
        int reserve_size = cfg_.planning_horizon / cfg_.resolution * 1.2;
        guide_path.reserve(reserve_size);
        guide_stamp.reserve(reserve_size);

        Vec4f init_yaw{robot_state_.yaw, 0, 0, 0};
        Vec4f fina_yaw{0, 0, 0, 0};


        // alias for last_exp_traj_info
        Trajectory guide_pos_traj, guide_yaw_traj, last_exp_traj;

        // record the wall time (WT) and the trajectory time (TT) at the start of the replan.
        const double replan_process_start_WT = ros_ptr_->getSimTime();
        double replan_process_start_TT, replan_state_TT;

        /* 2) Check last exp traj */
        if (last_exp_traj_info.empty()) {
            /* 2.1) Perform rest2rest exp traj generation */
            // just skip the first part of the guide trajectory
            pos_init_state.setZero();
            pos_init_state.col(0) = local_start_p_;
            replan_process_start_TT = -1;
            replan_state_TT = -1;
        } else {
            guide_pos_traj = cmd_traj_info_.posTraj(); // last_exp_traj;
            guide_yaw_traj = cmd_traj_info_.yawTraj(); //last_exp_traj_info.exp_yaw_traj;
            last_exp_traj = last_exp_traj_info.posTraj();

            replan_process_start_TT = replan_process_start_WT - last_exp_traj.start_WT;
            replan_state_TT = replan_process_start_TT + cfg_.replan_forward_dt;
            /* 2.2) Perform collision check on last exp traj*/
            vector<TimePosPair> last_exp_traj_time_pos;
            vector<double> last_exp_traj_vel;


            // check early exit condition
            // 1) if the replan state is beyond the last cmd traj, return NO_NEED
            if (replan_state_TT >= cmd_traj_info_.getTotalDuration()) {
                out_exp_traj_info = last_exp_traj_info;

                if (cmd_traj_info_.isTTOnBackupTraj(replan_process_start_TT)) {
                    if (cfg_.print_log)
                        ros_ptr_->warn(
                                " -- [SUPER] Replan, emergency stop, return FAILED and wait for plan form rest.");
                    return FAILED;
                }

                if (cfg_.print_log) {
                    ros_ptr_->warn(
                            " -- [generateExpTraj] replan_state_TT >= cmd_traj_info_.pos_traj.getTotalDuration(), return NONEED and wait for plan form rest.");
                }
                return NO_NEED;
            }

            if (!last_exp_traj_info.empty()) {
                if (replan_state_TT >= last_exp_traj.getTotalDuration()) {
                    out_exp_traj_info = last_exp_traj_info;
                    if (cfg_.print_log)
                        ros_ptr_->warn(
                                " -- [generateExpTraj] replan_state_TT >= last_exp_traj.getTotalDuration(), return NONEED and wait for plan form rest.");
                    if (cmd_traj_info_.isTTOnBackupTraj(replan_process_start_TT)) {
                        if (cfg_.print_log)
                            ros_ptr_->warn(
                                    " -- [SUPER] Replan, emergency stop, return FAILED and wait for plan form rest.");
                        return FAILED;
                    } else {
                        return NO_NEED;
                    }
                }

                /// 1) Check a series of early termination conditions.
                if (!gi_.new_goal && last_exp_traj_info.getSFCSize() == 1 && last_exp_traj_info.connectedToGoal()) {
                    if (cfg_.print_log) {
                        ros_ptr_->warn(
                                " -- [SUPER] Replan, last exp have only one corridor and connected to goal return NONEED.");
                    }

                    out_exp_traj_info = last_exp_traj_info;
                    if (cmd_traj_info_.isTTOnBackupTraj(replan_process_start_TT)) {
                        if (cfg_.print_log)
                            ros_ptr_->warn(
                                    " -- [SUPER] Replan, emergency stop, return FAILED and wait for plan form rest.");
                        return FAILED;
                    } else {
                        return NO_NEED;
                    }
                }

                const bool near_goal_shortcut =
                    !gi_.new_goal &&
                    (gi_.goal_p - last_exp_traj.getPos(replan_state_TT)).norm() <=
                        navigation_world_model::kNearGoalShortcutToleranceM &&
                    navigation_world_model::isGoalSegmentTraversable(
                        *map_ptr_,
                        last_exp_traj.getPos(replan_state_TT).cast<double>(),
                        gi_.goal_p.cast<double>());
                if (near_goal_shortcut) {
                    // Return if the traj close to goal
                    out_exp_traj_info = last_exp_traj_info;
                    out_exp_traj_info.setGoalConnectedFlag(true);

                    ros_ptr_->warn(" -- [SUPER] Replan, close to goal and return NONEED.");
                    if (cmd_traj_info_.isTTOnBackupTraj(replan_process_start_TT)) {
                        ros_ptr_->warn(
                                " -- [SUPER] Replan, emergency stop, return FAILED and wait for plan form rest.");
                        return FAILED;
                    } else {
                        return NO_NEED;
                    }
                }
            }
            /// Ready for replan.
            out_exp_traj_info.setGoalConnectedFlag(false);

            // * 2) Check if in backup trajectory. While in backup trajectory,
            // *    the guide trajectory should be a part of cmd trajectory.
            // TODO: Why cannot directly replan on cmd traj? 241121

            // * 3) Perform collision check on the guide trajectory.
            // TODO 0929 critical change for hot init.
            double eval_t = replan_state_TT; //replan_process_start_TT;
            double guide_pos_traj_total_time = guide_pos_traj.getTotalDuration();

            Vec3f temp_pt, last_sample_pt;
            last_exp_traj_time_pos.clear();
            last_sample_pt = guide_pos_traj.getPos(eval_t);
            eval_t += cfg_.sample_traj_dt;
            // * 4) 记录replan点在evaluated_pts上的id
            int replan_id = -1;
            for (; eval_t < guide_pos_traj_total_time; eval_t += cfg_.sample_traj_dt) {
                temp_pt = guide_pos_traj.getPos(eval_t);
                if ((temp_pt - last_sample_pt).norm() < cfg_.resolution * 0.8) {
                    continue;
                }

                const auto temp_grid = map_ptr_->classify(
                        temp_pt, navigation_world_model::GridLayer::kInflated);

                if (temp_grid == navigation_world_model::CellState::kOccupied ||
                    temp_grid == navigation_world_model::CellState::kOutOfMap) {
                    break;
                }
                if (eval_t > replan_state_TT && replan_id == -1) {
                    replan_id = last_exp_traj_time_pos.size();
                }
                last_exp_traj_time_pos.emplace_back(eval_t, temp_pt);
                last_exp_traj_vel.emplace_back(guide_pos_traj.getVel(eval_t).norm());
                last_sample_pt = temp_pt;
            }


            // * 6) Decide where to split the original exp trajecory and re-plan a new one with an A*,
            // *    If the whole trajectory if free,  the whole trajectory should be receding and if not, or a new goal
            // *    is given, we should only receiding a small distance and replan new trajectory ASAP
            double split_dis = cfg_.receding_dis;
            // Do not turn a collision-free committed trajectory into an
            // infinite immutable guide.  That upstream shortcut recursively
            // feeds optimizer drift back into every later replan and lets the
            // displayed normal path advance independently of newly sensed
            // geometry. Keep only the configured continuity prefix; A* owns
            // the rest of the route on every planning cycle.


            // * 7）Begin replan process, first get the replan state from the committed trajectory.
            if (!guide_pos_traj.getState(replan_state_TT, pos_init_state)) {
                ros_ptr_->warn(" -- [SUPER] Invalid traj or eval t");
                return FAILED;
            }
            // * Generate guide path with time stampe, for hot trajectory initialization
            // * the guide stamp is time from the replan start t
            guide_stamp.clear();
            guide_path.clear();
            if (split_dis <= 0 || last_exp_traj_time_pos.empty()) {
                /// No need receding, just path search.
                guide_path.push_back(pos_init_state.col(0));
                guide_stamp.push_back(0.0);
                last_exp_traj_time_pos.clear();
                last_exp_traj_time_pos.emplace_back(replan_state_TT, pos_init_state.col(0));
                guide_path_end_vel = robot_state_.v.norm();
            } else {
                temp_pt = last_exp_traj_time_pos.back().second;
                // * 8) Pop all evaluated pts after the sampled point.
                while (map_ptr_->classify(temp_pt, navigation_world_model::GridLayer::kInflated) ==
                           navigation_world_model::CellState::kOccupied ||
                       (temp_pt - pos_init_state.col(0)).norm() > split_dis) {
                    last_exp_traj_time_pos.pop_back();
                    last_exp_traj_vel.pop_back();
                    if (last_exp_traj_time_pos.empty()) {
                        ros_ptr_->warn(" -- [SUPER] WARN, all traj is collide in INF2");
                        break;
                    }
                    temp_pt = last_exp_traj_time_pos.back().second;
                }
                if (!last_exp_traj_time_pos.empty()) {
                    for (long unsigned int i = 0; i < last_exp_traj_time_pos.size(); i++) {
                        guide_path.push_back(last_exp_traj_time_pos[i].second);
                        guide_stamp.push_back(last_exp_traj_time_pos[i].first - last_exp_traj_time_pos.front().first);
                        guide_path_end_vel = last_exp_traj_vel[i];
                    }
                } else {
                    guide_path.push_back(pos_init_state.col(0));
                    guide_stamp.push_back(0.0);
                    last_exp_traj_time_pos.emplace_back(replan_state_TT, pos_init_state.col(0));
                    guide_path_end_vel = robot_state_.v.norm();
                }
            }
        }

        // second, geometry part of the guide path
        ///=================The Second Part of Guide Path ================================================

        double guide_path_length = geometry_utils::computePathLength(guide_path);
        double temp_horizon = cfg_.planning_horizon - guide_path_length;

        vector<int> path_passed_waypoint_id;
        vec_Vec3f inside_poly_goals;
        vector<int> sfc_waypoint_ids;

        if (guide_path.empty() ||
            ((guide_path.front() - pos_init_state.col(0)).norm() > 1e-2)) {
            guide_path.insert(guide_path.begin(), pos_init_state.col(0));
            guide_stamp.insert(guide_stamp.begin(), 0.0);
        }

        // if need a geometry path
        if (temp_horizon > cfg_.resolution * 2) {
            /// start point TT + exp_traj start_WT
//            double path_search_start_point_WT = guide_stamp.back() + guide_pos_traj.start_WT;
            // if the goal is close to the last point of the guide path, just add the goal to the guide path
            if ((guide_path.back() - gi_.goal_p).norm() <=
                    navigation_world_model::kNearGoalShortcutToleranceM &&
                navigation_world_model::isGoalSegmentTraversable(
                    *map_ptr_, guide_path.back().cast<double>(), gi_.goal_p.cast<double>())) {
                guide_stamp.push_back(guide_stamp.back() +
                                      (guide_path.back() - gi_.goal_p).norm() / cfg_.exp_traj_cfg.max_vel);
                guide_path.push_back(gi_.goal_p);
                // NO NEED
            } else {
                vec_Vec3f new_path;
                solve_stage_.store(2);
                if (!PathSearch(guide_path.back(), gi_.goal_p, temp_horizon,
                                new_path, solve_deadline)) {
                    ros_ptr_->warn(" -- [SUPER] PathSearch for new path failed");
                    return FAILED;
                }
                if (new_path.size() < 2) {
                    ros_ptr_->warn(" -- [SUPER] PathSearch for new path failed");
                    return FAILED;
                }

                // compute total dis
                // backward compute dis for all points
                double total_dis{0.0};
                vector<double> dis(new_path.size());
                Vec3f last_p = new_path.back();
                for (int i = new_path.size() - 2; i >= 0; i--) {
                    auto d = (new_path[i] - last_p).norm();
                    total_dis += d;
                    dis[i+1] = total_dis;
                    last_p = new_path[i];
                }
                total_dis += (new_path.front() - guide_path.back()).norm();
                dis[0] = total_dis;
//                for (int i = 0; i < dis.size(); i++) {
//                    cout << dis[i] << " ";
//                }
//                cout << endl;
                vector<double> stamps(new_path.size(), 0);
                vector<double> dt(new_path.size(), 0);
                double last_stamp = 0;
                for (int i = dis.size() - 1; i >= 0; i--) {
                    double vel;
                    geometry_utils::simplePMTimeAllocator(cfg_.exp_traj_cfg.max_acc, cfg_.exp_traj_cfg.max_vel,
                                                          guide_path_end_vel,
                                                          total_dis,
                                                          dis[i], stamps[i], vel);
                    dt[i] = stamps[i] - last_stamp;
                    last_stamp = stamps[i];
                }
                double time_stamp = guide_stamp.back();

//                for (int i = 0; i < stamps.size(); i++) {
//                    cout << stamps[i] << " ";
//                }
//                cout << endl;
//
//                for (int i = 0; i < dt.size(); i++) {
//                    cout << dt[i] << " ";
//                }
//                cout << endl;

                for (long unsigned int i = 1; i < new_path.size(); i++) {
                    double t = dt[i];
                    time_stamp += t;
                    guide_path.emplace_back(new_path[i]);
                    guide_stamp.emplace_back(time_stamp);
                }
            }
        }

        // Resolve the terminal point before corridor construction. Snapping
        // only the MINCO tail after CIRI has certified the unsnapped guide can
        // place the fixed endpoint outside every generated polytope.
        const bool direct_goal_segment_safe =
            navigation_world_model::isGoalSegmentTraversable(
                *map_ptr_, guide_path.back().cast<double>(), gi_.goal_p.cast<double>());
        const GuideEndpoint resolved_endpoint = direct_goal_segment_safe
            ? resolveGuideEndpoint(
                  guide_path.back(), gi_.goal_p,
                  navigation_world_model::kGoalConnectionToleranceM)
            : GuideEndpoint{guide_path.back(), false};
        guide_path.back() = resolved_endpoint.position;
        const bool connected_goal = resolved_endpoint.goal_connected;
        out_exp_traj_info.setGoalConnectedFlag(connected_goal);

        latest_guide_start_ = guide_path.front();
        latest_guide_end_ = guide_path.back();
        latest_guide_min_ = guide_path.front();
        latest_guide_max_ = guide_path.front();
        for (const auto &point : guide_path) {
            latest_guide_min_ = latest_guide_min_.cwiseMin(point);
            latest_guide_max_ = latest_guide_max_.cwiseMax(point);
        }

        sfc.clear();
        {
            TimeConsuming t_viz("tviz", false);
            ros_ptr_->vizFrontendPath(guide_path);
            time_consuming_[VISUALIZATION] += t_viz.stop();
        }
        shifted_sfc_start_pt_ = Vec3f(9999,9999,9999);
        solve_stage_.store(3);
        bool bool_ret_code = cg_ptr_->SearchPolytopeOnPath(guide_path, sfc, shifted_sfc_start_pt_, cfg_.use_fov_cut);

        if (!bool_ret_code) {
            ros_ptr_->warn(" -- [SUPER] SearchPolytopeOnPath for new path failed");
            return FAILED;
        }
        {
            TimeConsuming t_viz("tviz", false);
            ros_ptr_->vizExpSfc(sfc);
            time_consuming_[VISUALIZATION] += t_viz.stop();
        }

        time_consuming_[EPX_TRAJ_FRONTEND] = t_exp_frontend.stop();


        pos_fina_state.setZero();
        pos_fina_state.col(0) = guide_path.back();
        if (cfg_.goal_vel_en && (gi_.goal_p - robot_state_.p).norm() > cfg_.planning_horizon / 2) {
            pos_fina_state.col(1) = (gi_.goal_p - robot_state_.p).normalized() * cfg_.exp_traj_cfg.max_vel / 2;
        }
        if (connected_goal) {
            pos_fina_state.col(1).setZero();
        }

        // optimize and update exp traj
        bool temp_ret;
        Trajectory out_traj;
        TimeConsuming t_exp_opt("t_exp_opt", false);
        auto original_sfc = sfc;
        solve_stage_.store(4);
        exp_traj_opt_->setSolveBudget(
                &solve_cancelled_, solve_deadline.steadyDeadlineNanoseconds());
        temp_ret = exp_traj_opt_->optimize(pos_init_state,
                                           pos_fina_state,
                                           guide_path,
                                           guide_stamp,
                                           sfc,
                                           out_traj);
        time_consuming_[EXP_TRAJ_OPT] = t_exp_opt.stop();
        {
            VecDf init_ts;
            vec_Vec3f init_ps;
            exp_traj_opt_->getInitValue(init_ts, init_ps);
            latest_replan.setExpCondition(init_ts, init_ps, pos_init_state, pos_fina_state, sfc);
        }
        if (!temp_ret) {
            ros_ptr_->warn(" -- [SUPER] OptimizationExpTrajInPolytopes for new path failed");
            return FAILED;
        }
        double replan_total_t = (ros_ptr_->getSimTime() - replan_process_start_WT);
        if (!last_exp_traj_info.empty() && replan_total_t > cfg_.replan_forward_dt) {
            ros_ptr_->warn(" -- [SUPER] Replan over time({})!!!! Return FAILED", replan_total_t);
            return FAILED;
        }

        {
            TimeConsuming t_viz("tviz", false);
            ros_ptr_->vizExpTraj(out_traj);
            time_consuming_[VISUALIZATION] += t_viz.stop();
        }

        // A rest-to-rest solve holds the vehicle while optimizing.  Start its
        // command clock at commit time so optimizer latency cannot advance the
        // first PVA sample several metres ahead of the stationary vehicle.
        // Hot replans retain the original future-state stitching timestamp.
        double new_traj_WT = last_exp_traj_info.empty()
                                 ? ros_ptr_->getSimTime()
                                 : replan_process_start_WT;

        replan_process_start_TT = replan_process_start_WT - guide_pos_traj.start_WT;
        Trajectory temp_exp_traj;
        if (!last_exp_traj_info_.empty() &&
            !guide_pos_traj.getPartialTrajectoryByTime(replan_process_start_TT, replan_state_TT,
                                                       temp_exp_traj)) {
            ros_ptr_->error(" -- [SUPER] in [generateExpTraj]: getPartialTrajectoryByTime failed, force return");
            return FAILED;
        }
        out_exp_traj_info.setSFC(sfc);
        temp_exp_traj = temp_exp_traj + out_traj;
        temp_exp_traj.start_WT = new_traj_WT; //last_exp_traj_info.replan_start_WT ;

        if (!last_exp_traj_info.empty()) {
            StatePVAJ yaw_replan_state;
            if (!guide_yaw_traj.getState(replan_state_TT, yaw_replan_state)) {
                ros_ptr_->warn(" -- [SUPER] Invalid traj or eval t");
                return FAILED;
            }
            init_yaw = yaw_replan_state.row(0);
        }


        bool free_end{true};
        if (cfg_.goal_yaw_en && !isnan(gi_.goal_yaw) && connected_goal) {
            free_end = false;
            fina_yaw[0] = gi_.goal_yaw;
        }
        Trajectory new_traj, old_traj;

        if (!yaw_traj_opt_->optimize(init_yaw, fina_yaw, out_traj, new_traj, 3, false, free_end)) {
            ros_ptr_->error(" -- [SUPER] in [generateExpTraj]: YawTrajOpt failed, force return");
            return FAILED;
        }
        if (!last_exp_traj_info.empty()) {
            if (!guide_yaw_traj.getPartialTrajectoryByTime(replan_process_start_TT, replan_state_TT,
                                                           old_traj)) {
                ros_ptr_->error(" -- [SUPER] in [generateExpTraj]: getPartialTrajectoryByTime failed, force return");
                return FAILED;
            }
        }

        const auto temp_yaw_traj = old_traj + new_traj;
        traj_opt::TrajectoryDynamicReport exp_dynamic_report;
        if (!traj_opt::trajectorySatisfiesFlatnessEnvelope(
                temp_exp_traj, cfg_.exp_traj_cfg, &exp_dynamic_report,
                0.01, &temp_yaw_traj)) {
            ros_ptr_->error(
                    " -- [SUPER] combined EXP position/yaw body-rate or thrust gate failed: "
                    "body_rate={} thrust=[{},{}]",
                    exp_dynamic_report.maximum_body_rate_rad_s,
                    exp_dynamic_report.minimum_thrust_n,
                    exp_dynamic_report.maximum_thrust_n);
            return FAILED;
        }
        // check if part of the exp on last backup
        double on_backup_end_TT{-1}, on_backup_start_TT{-1};
        if (!last_exp_traj_info.empty() && replan_state_TT > cmd_traj_info_.getBackupTrajStartTT()) {
            on_backup_start_TT = cmd_traj_info_.getBackupTrajStartTT() - replan_process_start_TT;
            on_backup_end_TT = replan_state_TT - replan_process_start_TT;
        }
        out_exp_traj_info.setTrajectory(new_traj_WT, temp_exp_traj, temp_yaw_traj, on_backup_start_TT,
                                        on_backup_end_TT);

        latest_replan.setExpYawTraj(temp_yaw_traj);
        latest_replan.setExpTraj(temp_exp_traj);

        return SUCCESS;
    }

    RET_CODE SuperPlanner::generateBackupTrajectory(
            ExpTraj &ref_exp_traj,
            BackupTraj &back_traj_info,
            const AbsoluteDeadline &solve_deadline) {
        drone_state_mutex_.lock();
        back_traj_info.setRobotPos(robot_state_.p);
        drone_state_mutex_.unlock();
        TimeConsuming t_back_frontend("t_back_frontend", false);
        double total_dur = ref_exp_traj.getTotalDuration();
        double start_t = ros_ptr_->getSimTime() - ref_exp_traj.getStartWallTime();


        if (start_t > total_dur - 0.01) {
            if (cfg_.print_log) {
                ros_ptr_->info(" -- [SUPER] in [generateBackupTrajectory]: start_t > total_dur, return NO_NEED");
            }
            return NO_NEED;
        }

        Vec3f temp_point;
        double out_t;
        bool all_traj_visible{true};
        vector<TimePosPair> eval_ps;

        // Collect geometrically distinct samples first. A nearly straight
        // trajectory shares one sensor ray, so checking every longer prefix
        // from the robot is quadratic in trajectory length at fine map
        // resolution. In that common open-space case one farthest-endpoint
        // ray is equivalent for occupied-grid visibility.
        vector<TimePosPair> candidate_ps;
        Vec3f last_pos = ref_exp_traj.getPos(start_t);
        for (out_t = start_t; out_t < total_dur; out_t += cfg_.sample_traj_dt) {
            temp_point = ref_exp_traj.getPos(out_t);
            if ((last_pos - temp_point).norm() < cfg_.resolution * 0.8) {
                continue;
            }
            last_pos = temp_point;
            candidate_ps.emplace_back(out_t, temp_point);
        }

        bool shared_visibility_ray{false};
        const Vec3f visibility_origin = back_traj_info.getRobotPos();
        if (!candidate_ps.empty()) {
            const Vec3f farthest_delta = candidate_ps.back().second - visibility_origin;
            const double farthest_distance = farthest_delta.norm();
            if (farthest_distance > cfg_.resolution) {
                const Vec3f ray_direction = farthest_delta / farthest_distance;
                double last_projection{-1.0};
                shared_visibility_ray = true;
                // isLineFree checks the configured robot-radius neighbor tube
                // around the ray. Any stitched prefix contained by that same
                // tube is covered by the endpoint check as well.
                // A single inflated ray may represent another sample only when
                // their centerlines are the same ray. Merely being within one
                // robot radius is unsafe: the two radius-r tubes then overlap
                // but neither contains the other.
                const double lateral_tolerance =
                        std::max(1.0e-9, cfg_.resolution * 1.0e-6);
                for (const auto &sample : candidate_ps) {
                    const Vec3f delta = sample.second - visibility_origin;
                    const double projection = delta.dot(ray_direction);
                    const double lateral_error = (delta - projection * ray_direction).norm();
                    if (projection + cfg_.resolution < last_projection || projection < 0.0 ||
                        projection > farthest_distance + cfg_.resolution ||
                        lateral_error > lateral_tolerance) {
                        shared_visibility_ray = false;
                        break;
                    }
                    last_projection = projection;
                }
            }
        }

        const double visibility_limit =
                cfg_.sensing_horizon > 0 ? std::min(cfg_.sensing_horizon, cfg_.safe_corridor_line_max_length)
                                         : cfg_.safe_corridor_line_max_length;
        const auto inflated_line_visible = [&](const Vec3f &endpoint) {
            if (visibility_limit > 0.0 &&
                (endpoint - visibility_origin).norm() > visibility_limit) {
                return false;
            }
            // SUPER's endpoint-only ROG-Map deliberately leaves unobserved
            // cells UNKNOWN.  Backup visibility is therefore the upstream
            // obstacle-free sensor tube, not a persisted probabilistic FREE
            // label.  The inflated layer supplies the robot-radius tube in one
            // lookup and preserves the upstream unknown-as-visible semantics.
            return map_ptr_->isSegmentTraversable(
                    visibility_origin, endpoint,
                    navigation_world_model::GridLayer::kInflated,
                    navigation_world_model::UnknownPolicy::kAllowUnknown);
        };
        if (shared_visibility_ray &&
            inflated_line_visible(candidate_ps.back().second)) {
            eval_ps = candidate_ps;
            out_t = total_dur;
        } else {
            eval_ps.clear();
            for (const auto &sample : candidate_ps) {
                out_t = sample.first;
                eval_ps.push_back(sample);
                if (!inflated_line_visible(sample.second)) {
                    all_traj_visible = false;
                    break;
                }
            }
            if (all_traj_visible) out_t = total_dur;
        }
        if (all_traj_visible) {
            // No backup trajectory is needed when every remaining EXP sample
            // is visible. The upstream branch generated a long corridor from
            // the robot to the terminal point and then immediately discarded
            // it by returning FINISH. At fine map resolution that dead IRIS
            // solve dominates ReplanOnce and can make a valid plan miss its
            // stitching deadline. Keep the observable result identical and
            // avoid constructing an unused safety corridor.
            back_traj_info.setEmpty();
            time_consuming_[BACK_TRAJ_FRONTEND] = t_back_frontend.stop();
            return FINISH;
        }
        Vec3f invisible_p = eval_ps.back().second;
        while (out_t > start_t) {
            out_t -= cfg_.sample_traj_dt;
            Vec3f out_p = ref_exp_traj.getPos(out_t);
            if ((out_p - invisible_p).norm() > cfg_.robot_r) {
                break;
            }
        }

        double seed_point_t = std::max(start_t, out_t);

        // TODO check this logic, comment on Dec. 13
        // if
        // 1) last exp traj has a backup traj
        // 2) last backup WT is larger than this term
        // 3) last exp is collision free
        // if (ref_exp_traj.back_traj_start_TT > 0 &&
        // seed_point_t < ref_exp_traj.back_traj_start_TT) {
        // return NO_NEED;
        // }


        Vec3f seed_point = ref_exp_traj.getPos(seed_point_t);

        Vec3f shifted_robot_p = shifted_sfc_start_pt_.norm()> 999?robot_state_.p:shifted_sfc_start_pt_;
        const auto nearest_start = map_ptr_->nearestNotOccupied(
                shifted_robot_p, navigation_world_model::GridLayer::kEvidence, 3.0);
        if (!nearest_start) {
            ros_ptr_->error(
                    " -- [SUPER] in [PlanFromRest] Local start point is deeply occupied, which should not happened.");
            latest_replan.setRetCode(SUPER_RET_CODE::SUPER_NO_START_POINT);
            return FAILED;
        }
        shifted_robot_p = *nearest_start;

        Line line{shifted_robot_p, seed_point};
        Polytope temp_poly;
        if (!cg_ptr_->GeneratePolytopeFromLine(line, temp_poly)) {
            ros_ptr_->warn(" -- [SUPER] GeneratePolytopeFromLine failed, force return");
            return FAILED;
        }
        Eigen::Vector3d inner;
        Eigen::Matrix3Xd vPoly;
        if (!geometry_utils::findInterior(temp_poly.GetPlanes(), inner)) {
            ros_ptr_->warn(" -- [SUPER] Cannot generate feasible backup sfc, force return");
            vec_Vec3f seed{back_traj_info.getRobotPos(), seed_point};
            return FAILED;
        }

        if (cfg_.use_fov_cut) {
            if (!fov_checker_->cutPolyByFov(robot_state_.p, robot_state_.q, seed_point,
                                            temp_poly)) {
                ros_ptr_->warn(" -- [SUPER] cutPolyByFov failed, force return");
                return FAILED;
            }
        }
        // cut by sensing horizon
        if (cfg_.sensing_horizon > 0 &&
            !fov_checker_->cutPolyBySensingHorizon(robot_state_.p, seed_point, cfg_.sensing_horizon,
                                                   temp_poly)) {
            ros_ptr_->warn(" -- [SUPER] cutPolyBySensingHorizon failed, force return");
            vec_Vec3f seed{back_traj_info.getRobotPos(), seed_point};
            return FAILED;
        }

        back_traj_info.setSFC(temp_poly);

        {
            TimeConsuming t_viz("tviz", false);
            ros_ptr_->vizBackupSfc(temp_poly);
            time_consuming_[VISUALIZATION] += t_viz.stop();
        }

//        Vec3f out_p = temp_point;
//        double t_R = 0.0;
        double eval_t = eval_ps.back().first + cfg_.sample_traj_dt;
        last_pos = eval_ps.back().second;
        while (temp_poly.PointIsInside(eval_ps.back().second) && eval_t < total_dur) {
            Vec3f cur_pos = ref_exp_traj.getPos(eval_t);

            if ((cur_pos - last_pos).norm() < cfg_.resolution * 0.8) {
                eval_t += cfg_.sample_traj_dt;
                continue;
            }
            eval_ps.emplace_back(eval_t, cur_pos);
            last_pos = cur_pos;
            eval_t += cfg_.sample_traj_dt;
        }
        if (eval_ps.size() <= 1U) {
            ros_ptr_->warn(
                    " -- [SUPER] backup visibility produced no certified seed before first invisible sample");
            return FAILED;
        }
        eval_ps.pop_back();
        seed_point = eval_ps.back().second;
        seed_point_t = eval_ps.back().first;

        //        bool use_new{true};
        //        if (use_new) {
        double t0 = ros_ptr_->getSimTime() -
                    ref_exp_traj.getStartWallTime() + 0.01;
        double te = seed_point_t;
        //            cout << "t0: " << t0 << endl;
        //            cout << "te: " << te << endl;
        //            cout << "exp_traj_dur: " << ref_exp_traj.optimized_exp_traj.getTotalDuration() << endl;
        double vel_e_n = ref_exp_traj.getVel(te).norm();
        double heu_ts = std::max((t0 + te) / 2, te - vel_e_n / cfg_.back_traj_cfg.max_acc);
        // Preserve SUPER's visibility/braking-derived switch point.  The
        // main+backup bundle is already committed atomically; forcing the
        // switch into a fixed number of replanning cycles makes a vehicle at
        // low speed brake before it can enter an obstacle detour.
        heu_ts = std::clamp(heu_ts, t0, te);
        StatePVAJ switch_state;
        BackupBrakingSeed braking_seed;
        bool braking_seed_inside_sfc = false;
        const double initial_switch_guess = heu_ts;
        // The latest visibility-derived switch is desirable for progress, but
        // its braking endpoint may lie outside the generated safety corridor.
        // Search backward on the EXP trajectory until the complete Bezier
        // hull of a dynamically feasible stop is contained by that corridor.
        // This retains the latest certifiable switch instead of either
        // disabling backup or imposing an unrelated fixed replan horizon.
        for (double candidate_ts = heu_ts;;) {
            switch_state = ref_exp_traj.posTraj().getState(candidate_ts);
            braking_seed = makeBackupBrakingSeed(
                    candidate_ts, switch_state,
                    cfg_.back_traj_cfg.max_vel, cfg_.back_traj_cfg.max_acc,
                    cfg_.back_traj_cfg.max_jerk, cfg_.sample_traj_dt,
                    0.0);
            braking_seed_inside_sfc = braking_seed.feasible &&
                    braking_seed.duration_s > cfg_.sample_traj_dt;
            if (braking_seed_inside_sfc) {
                const auto braking_control_points = minimumSnapStopBezierControlPoints(
                        switch_state, braking_seed.duration_s);
                for (int i = 0;
                     braking_seed_inside_sfc && i < braking_control_points.cols(); ++i) {
                    braking_seed_inside_sfc =
                            temp_poly.PointIsInside(braking_control_points.col(i));
                }
            }
            if (braking_seed_inside_sfc) {
                heu_ts = candidate_ts;
                break;
            }
            if (candidate_ts <= t0 + 1.0e-9) break;
            candidate_ts = std::max(t0, candidate_ts - cfg_.sample_traj_dt);
        }
        if (!braking_seed_inside_sfc) {
            ros_ptr_->warn(
                    " -- [SUPER] no dynamically feasible minimum-snap backup hull inside SFC");
            return OPT_FAILED;
        }
        if (cfg_.print_log && initial_switch_guess - heu_ts > cfg_.sample_traj_dt * 0.5) {
            ros_ptr_->info(
                    " -- [SUPER] moved backup switch backward from {} to {} for certified hull",
                    initial_switch_guess, heu_ts);
        }
        double heu_dur = braking_seed.duration_s;
        // Keep the optimized switch close to the derived braking state. A
        // small interval avoids mapping an exact interval endpoint to
        // infinity while preventing the split-time reward from drifting back
        // toward the visibility boundary.
        const double backup_switch_upper_bound = std::min(
                te, heu_ts + std::max(0.01, cfg_.replan_forward_dt * 0.25));
        time_consuming_[BACK_TRAJ_FRONTEND] = t_back_frontend.stop();
        TimeConsuming t_back_opt("t_back_opt", false);
        double opt_ts = heu_ts;
        Trajectory temp_pos_traj;
        VecDf seed_times(cfg_.back_traj_cfg.piece_num);
        seed_times.setConstant(heu_dur / cfg_.back_traj_cfg.piece_num);
        vec_Vec3f seed_points;
        seed_points.reserve(cfg_.back_traj_cfg.piece_num);
        const auto braking_piece = minimumSnapStopPiece(switch_state, heu_dur);
        for (int i = 1; i <= cfg_.back_traj_cfg.piece_num; ++i) {
            seed_points.emplace_back(braking_piece.getPos(
                    heu_dur * static_cast<double>(i) /
                    cfg_.back_traj_cfg.piece_num));
        }
        back_traj_opt_->setSolveBudget(
                &solve_cancelled_, solve_deadline.steadyDeadlineNanoseconds());
        bool temp_ret = back_traj_opt_->optimize(ref_exp_traj.posTraj(),
                                                 t0,
                                                 backup_switch_upper_bound,
                                                 heu_ts,
                                                 back_traj_info.getSFC(),
                                                 seed_times,
                                                 seed_points,
                                                 temp_pos_traj,
                                                 opt_ts);
        time_consuming_[BACK_TRAJ_OPT] = t_back_opt.stop();

        {
            double init_ts;
            VecDf init_times;
            vec_Vec3f init_ps;
            back_traj_opt_->getInitValue(init_ts, init_times, init_ps);
            latest_replan.setBackupCondition(init_ts, init_times, init_ps,
                                             t0, backup_switch_upper_bound,
                                             back_traj_info.getSFC());
        }

        if (!temp_ret) {
            // The seed has already passed exact PVAJ boundary checks,
            // analytic velocity/acceleration/jerk extrema, and full Bezier
            // hull containment. L-BFGS is an optional refinement, not an
            // authority to discard that certified safety trajectory.
            ++backup_refinement_fallback_count_;
            ros_ptr_->warn(
                    " -- [SUPER] backup refinement failed; using certified minimum-snap seed "
                    "backup_refinement_success={} backup_refinement_fallback={}",
                    backup_refinement_success_count_, backup_refinement_fallback_count_);
            temp_pos_traj.clear();
            temp_pos_traj.emplace_back(braking_piece);
            opt_ts = heu_ts;
        } else {
            ++backup_refinement_success_count_;
            ros_ptr_->info(
                    " -- [SUPER] backup refinement accepted: "
                    "backup_refinement_success={} backup_refinement_fallback={}",
                    backup_refinement_success_count_, backup_refinement_fallback_count_);
        }
        Vec4f yaw_init_vec = ref_exp_traj.getYawState(opt_ts).row(0);
        Vec4f yaw_goal{0, 0, 0, 0};
        bool free_end{true};
        if (cfg_.goal_yaw_en) {
            if (!isnan(gi_.goal_yaw)) {
                free_end = false;
                yaw_goal[0] = gi_.goal_yaw;
            }
        }
        Trajectory temp_yaw_traj;
        if (!yaw_traj_opt_->optimize(yaw_init_vec, yaw_goal, temp_pos_traj,
                                     temp_yaw_traj, 3, false, free_end)) {
            ros_ptr_->error(" -- [SUPER] in [generateBackupTrajectory] YawTrajOpt FAILD.");
            return OPT_FAILED;
        }
        traj_opt::TrajectoryDynamicReport backup_dynamic_report;
        if (!traj_opt::trajectorySatisfiesFlatnessEnvelope(
                temp_pos_traj, cfg_.back_traj_cfg, &backup_dynamic_report,
                0.01, &temp_yaw_traj)) {
            ros_ptr_->error(
                    " -- [SUPER] combined backup position/yaw body-rate or thrust gate failed: "
                    "body_rate={} thrust=[{},{}]",
                    backup_dynamic_report.maximum_body_rate_rad_s,
                    backup_dynamic_report.minimum_thrust_n,
                    backup_dynamic_report.maximum_thrust_n);
            return OPT_FAILED;
        }


        if (opt_ts < t0) {
            ros_ptr_->error(" -- [SUPER] opt_ts {} < t0 {}", opt_ts, t0);
            return OPT_FAILED;
        }
        double new_ts_WT = ref_exp_traj.getStartWallTime() + opt_ts;
        const double committed_start_wt = cmd_traj_info_.getStartWallTime();
        const double new_ts_TT = new_ts_WT - committed_start_wt;
        const double committed_ts_TT = cmd_traj_info_.getBackupTrajStartTT();
        if (committed_ts_TT < cmd_traj_info_.getTotalDuration() &&
            new_ts_TT < committed_ts_TT) {
            ros_ptr_->error(" -- [SUPER] new_ts_TT {} < committed_ts_TT {}",
                           new_ts_TT, committed_ts_TT);
            return OPT_FAILED;
        }


        {
            TimeConsuming t_viz("tviz", false);
            ros_ptr_->vizBackupTraj(temp_pos_traj);
            time_consuming_[VISUALIZATION] += t_viz.stop();
        }

        back_traj_info.setTrajectory(new_ts_WT, opt_ts, temp_pos_traj, temp_yaw_traj);
        latest_replan.setBackupTraj(temp_pos_traj);
        latest_replan.setBackupYawTraj(temp_yaw_traj);
        return SUCCESS;
    }

    int SuperPlanner::getNearestFurtherGoalPoint(const vec_E<Vec3f> &goals, const Vec3f &start_pt) {
        if (goals.size() == 1) {
            return 0;
        }
        Vec3f a = start_pt, b;
        int min_id = 0;
        double min_dis = 1e10;
        for (long unsigned int i = 0; i < goals.size() - 1; i++) {
            b = goals[i];
            double dis = geometry_utils::pointLineSegmentDistance(start_pt, a, b);
            if (dis < min_dis) {
                min_dis = dis;
                min_id = i;
            }
            a = b;
        }
        return min_id;
    }

    bool
    SuperPlanner::PathSearch(const Vec3f &start_pt, const Vec3f &goal,
                             const double &searching_horizon,
                             vec_Vec3f &path,
                             const AbsoluteDeadline &solve_deadline) {
        using namespace path_search;
        if (searching_horizon <= 0.0) {
            ros_ptr_->error(" -- [SUPER] Goal waypoints empty or searching horizon negative, force return.");
            return false;
        }

        // 1) check and shift pts
        // 		For start point, must be collision free
        const auto start_type = map_ptr_->classify(
                start_pt, navigation_world_model::GridLayer::kEvidence);

        /// If the start_pt is obstacle in prob map, just shift it to the nearest free point.
        if (start_type == navigation_world_model::CellState::kOccupied ||
            start_type == navigation_world_model::CellState::kOutOfMap) {
            ros_ptr_->warn(
                    " -- [SUPER] The start point in obstacle, this should not happen since the start point should be shift before pathsearch.");
            return false;
        }
        vec_E<Vec3f> start_point_escape_path;

        int flag_es = ON_PROB_MAP | (cfg_.frontend_in_known_free ? UNKNOWN_AS_OCCUPIED : UNKNOWN_AS_FREE);
        vec_Vec3f out_path;
        RET_CODE ret_es = astar_ptr_->escapePathSearch(
                start_pt, flag_es, out_path, true);
        if (ret_es != NO_NEED && ret_es != REACH_HORIZON &&
            ret_es != REACH_GOAL && ret_es != INIT_ERROR) {
            ros_ptr_->warn(
                    " -- [Astar] Preferred-altitude escape failed with [{}]; "
                    "retry unrestricted 3-D escape.", RET_CODE_STR[ret_es].c_str());
            ret_es = astar_ptr_->escapePathSearch(
                    start_pt, flag_es, out_path, false);
        }
        if (ret_es != NO_NEED) {
            if (ret_es != REACH_HORIZON && ret_es != REACH_GOAL) {
                ros_ptr_->error(
                        " -- [SUPER] Escape path search failed with [{}], force return.",
                        RET_CODE_STR[ret_es].c_str());
                return false;
            } else {
                start_point_escape_path = out_path;
            }
        }

        Vec3f shifted_start_pt = start_pt;

        if (!start_point_escape_path.empty()) {
            shifted_start_pt = start_point_escape_path.back();
        }

        Vec3f temp_goal_point, temp_start_point;
        temp_start_point = shifted_start_pt;
        double temp_plannning_horizon = searching_horizon;
        // Preferred-altitude, unrestricted, and probability-map variants are
        // alternatives within one search stage, not independent solves. Share
        // an absolute budget so a timeout cannot multiply callback latency.
        const double stage_budget = std::min(
                cfg_.astar_total_time_limit_s,
                solve_deadline.conservativeRemaining(ros_ptr_->getSimTime()));
        if (stage_budget <= 0.0) {
            ros_ptr_->warn(" -- [Astar] solve deadline exhausted before path search");
            return false;
        }
        const AbsoluteDeadline search_deadline(
                ros_ptr_->getSimTime(), stage_budget);
        const auto remaining_search_budget = [&]() {
            return std::min(search_deadline.remaining(ros_ptr_->getSimTime()),
                            solve_deadline.conservativeRemaining(ros_ptr_->getSimTime()));
        };
        //            int start_id = getNearestFurtherGoalPoint(goal_waypoints, start_pt);

        int flag = ON_INF_MAP | (cfg_.frontend_in_known_free ? UNKNOWN_AS_OCCUPIED : UNKNOWN_AS_FREE) | DONT_USE_INF_NEIGHBOR;

        RET_CODE ret_code = astar_ptr_->pointToPointPathSearch(
                temp_start_point, goal, flag, temp_plannning_horizon,
                path, std::min(cfg_.astar_search_time_limit_s,
                               remaining_search_budget()), true);

        if (ret_code != REACH_HORIZON && ret_code != REACH_GOAL &&
            ret_code != INIT_ERROR &&
            !search_deadline.expired(ros_ptr_->getSimTime())) {
            ros_ptr_->warn(
                    " -- [Astar] Preferred start-goal altitude search failed with [{}]; "
                    "retry unrestricted 3-D search.", RET_CODE_STR[ret_code].c_str());
            ret_code = astar_ptr_->pointToPointPathSearch(
                    temp_start_point, goal, flag, temp_plannning_horizon,
                    path, remaining_search_budget(), false);
        }

        if(ret_code == INIT_ERROR){
            gi_.goal_valid = false;
            return false;
        }
        //add may23, if failed on inf map, use prob map try again

        if (ret_code == NO_PATH &&
            !search_deadline.expired(ros_ptr_->getSimTime())) {
            flag = ON_PROB_MAP | (cfg_.frontend_in_known_free ? UNKNOWN_AS_OCCUPIED : UNKNOWN_AS_FREE) |
                   USE_INF_NEIGHBOR;
            fmt::print(fg(fmt::color::indian_red) | fmt::emphasis::bold,
                       " -- [Astar] Path search failed on inf map, try again on prob map.\n");
            ret_code = astar_ptr_->pointToPointPathSearch(
                    temp_start_point, goal, flag, temp_plannning_horizon,
                    path, remaining_search_budget(), true);
            if (ret_code != REACH_HORIZON && ret_code != REACH_GOAL &&
                ret_code != INIT_ERROR &&
                !search_deadline.expired(ros_ptr_->getSimTime())) {
                ret_code = astar_ptr_->pointToPointPathSearch(
                        temp_start_point, goal, flag, temp_plannning_horizon,
                        path, remaining_search_budget(), false);
            }
            if (ret_code == SUCCESS || ret_code == REACH_HORIZON || ret_code == REACH_GOAL) {
                fmt::print(fg(fmt::color::lime_green) | fmt::emphasis::bold,
                           " -- [Astar] Path search on prob map success.\n");
            } else {
                fmt::print(fg(fmt::color::indian_red) | fmt::emphasis::bold,
                           " -- [Astar] Path search failed on prob map still failed.\n");
            }
        }
        if (ret_code != REACH_HORIZON && ret_code != REACH_GOAL) {
            ros_ptr_->error(
                    " -- [SUPER] Path search failed with [{}], force return.\n", RET_CODE_STR[ret_code].c_str());
            return false;
        }

        // Prefer a mission-altitude route when the same A* lateral detour is
        // collision-free at the start-to-goal height profile. With
        // UNKNOWN_AS_FREE and no raycasting, an unconstrained 3-D graph can
        // otherwise take a numerically equivalent shortcut toward the unseen
        // ground; that shortcut becomes OCCUPIED only after the vehicle is too
        // close for its certified brake. Preserve the original 3-D route when
        // vertical avoidance is actually required.
        if (path.size() >= 2) {
            const Vec3f route_delta = goal - shifted_start_pt;
            const double horizontal_distance = route_delta.head<2>().norm();
            if (horizontal_distance > cfg_.resolution) {
                const Eigen::Vector2d horizontal_direction =
                        route_delta.head<2>() / horizontal_distance;
                vec_Vec3f altitude_preserving_path = path;
                for (auto &point : altitude_preserving_path) {
                    const double progress = std::clamp(
                            (point.head<2>() - shifted_start_pt.head<2>())
                                    .dot(horizontal_direction) / horizontal_distance,
                            0.0, 1.0);
                    point.z() = shifted_start_pt.z() + progress * route_delta.z();
                }
                bool altitude_projection_free = true;
                for (std::size_t index = 1;
                     index < altitude_preserving_path.size(); ++index) {
                    if (!map_ptr_->isSegmentTraversable(
                            altitude_preserving_path[index - 1],
                            altitude_preserving_path[index],
                            navigation_world_model::GridLayer::kInflated,
                            navigation_world_model::UnknownPolicy::kAllowUnknown)) {
                        altitude_projection_free = false;
                        break;
                    }
                }
                if (altitude_projection_free) {
                    path = std::move(altitude_preserving_path);
                }
            }
        }
        if (!start_point_escape_path.empty()) {
            path.insert(path.begin(), start_point_escape_path.begin(),
                        start_point_escape_path.end());
        }

        // Keep the route inside the exact metric box used by corridor
        // generation. ROG's circular-buffer insideLocalMap predicate is index
        // based and can accept a point beyond this box near a sliding edge,
        // which leaves MINCO's tail outside every generated polytope.
        const auto world_geometry = map_ptr_->geometry();
        const Vec3f corridor_center = world_geometry.local_center_m;
        const Vec3f corridor_half_size = 0.5 * world_geometry.local_size_m;
        const Vec3f corridor_margin = Vec3f::Constant(
                2.0 * world_geometry.inflated_resolution_m);
        const Vec3f corridor_min = corridor_center - corridor_half_size + corridor_margin;
        const Vec3f corridor_max = corridor_center + corridor_half_size - corridor_margin;
        const auto inside_corridor_map = [&](const Vec3f &point) {
            return (point.array() >= corridor_min.array()).all() &&
                   (point.array() <= corridor_max.array()).all();
        };
        bool trimmed_to_corridor_map = false;
        while (!path.empty() && !inside_corridor_map(path.back())) {
            path.pop_back();
            trimmed_to_corridor_map = true;
        }

        if (path.empty()) {
            ros_ptr_->warn(
                    " -- [SUPER] Path search failed with empty segments, force return.");
            return false;
        }
        path.insert(path.begin(), start_pt);
        if (ret_code == REACH_GOAL && !trimmed_to_corridor_map &&
            inside_corridor_map(goal)) {
            path.push_back(goal);
        }
        return true;
    }


    void SuperPlanner::getRobotState(super_utils::RobotState &out) {
        out = robot_state_;
    }

    bool SuperPlanner::setPlannerExecutionState(const super_utils::RobotState &state) {
        if (!state.rcv || !state.p.allFinite() || !state.v.allFinite() ||
            !state.a.allFinite() || !state.j.allFinite() ||
            !state.q.coeffs().allFinite() || state.q.norm() <= 1.0e-6 ||
            !std::isfinite(state.yaw) || !std::isfinite(state.rcv_time) ||
            state.rcv_time <= 0.0) {
            return false;
        }
        std::lock_guard<std::mutex> guard(drone_state_mutex_);
        robot_state_ = state;
        robot_state_.q.normalize();
        return true;
    }
}
