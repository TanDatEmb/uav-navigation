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

#pragma once

#include <iostream>
#include <fstream>
#include <cstdint>
#include <optional>
#include <fmt/color.h>
#include "Eigen/Eigen"


#include <super_core/config.hpp>
#include <super_core/absolute_deadline.hpp>
#include <ros_interface/ros1/ros1_interface.hpp>
#include <data_structure/base/trajectory.h>

#include <data_structure/base/polytope.h>


#include "traj_opt/exp_traj_optimizer_s4.h"
#include "traj_opt/backup_traj_optimizer_s4.h"
#include "path_search/astar.h"
#include <navigation_world_model/world_model_view.hpp>
#include <navigation_world_model/world_commit_authorizer.hpp>
#include "super_core/corridor_generator.h"
#include "super_core/fov_checker.h"

#include "traj_opt/yaw_traj_opt.h"
#include "super_core/super_ret_code.hpp"
#include "utils/header/fmt_eigen.hpp"

#include <super_core/log_utils.hpp>
#include <data_structure/exp_traj.h>
#include <data_structure/cmd_traj.h>
#include <data_structure/backup_traj.h>


namespace super_planner {
    using namespace color_text;
    using namespace geometry_utils;

    class SuperPlanner {
        LogOneReplan latest_replan;
        super_planner::Config cfg_;
        navigation_world_model::WorldModelViewPtr map_ptr_;
        navigation_world_model::WorldCommitAuthorizer* commit_authorizer_{nullptr};
        CorridorGenerator::Ptr cg_ptr_;
        path_search::Astar::Ptr astar_ptr_;
        ros_interface::RosInterface::Ptr ros_ptr_;
        Vec3f shifted_sfc_start_pt_;

        traj_opt::ExpTrajOpt::Ptr exp_traj_opt_;
        traj_opt::BackupTrajOpt::Ptr back_traj_opt_;
        traj_opt::YawTrajOpt::Ptr yaw_traj_opt_;

        CIRI::Ptr ciri_;

        super_utils::RobotState robot_state_;

        std::mutex drone_state_mutex_;
        std::mutex replan_lock_;
        std::mutex solve_commit_mutex_;

        Vec3f local_start_p_;

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
        std::uint64_t backup_refinement_success_count_{0};
        std::uint64_t backup_refinement_fallback_count_{0};

        Vec3f latest_guide_start_{Vec3f::Constant(std::numeric_limits<double>::quiet_NaN())};
        Vec3f latest_guide_end_{Vec3f::Constant(std::numeric_limits<double>::quiet_NaN())};
        Vec3f latest_guide_min_{Vec3f::Constant(std::numeric_limits<double>::quiet_NaN())};
        Vec3f latest_guide_max_{Vec3f::Constant(std::numeric_limits<double>::quiet_NaN())};

        bool authorizeAndCommit(CandidateCommandBundle&& candidate);

    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        explicit SuperPlanner(const std::string &cfg_path,
                              const ros_interface::RosInterface::Ptr &ros_ptr,
                              navigation_world_model::WorldModelViewPtr map_ptr,
                              const std::optional<DynamicLimits> &mission_limits,
                              navigation_world_model::WorldCommitAuthorizer& commit_authorizer);

        ~SuperPlanner() = default;

        void lockCommittedTraj() {
            cmd_traj_info_.lock();
        }

        void unlockCommittedTraj() {
            cmd_traj_info_.unlock();
        }

        bool goalValid() const {
            return gi_.goal_valid;
        }

        typedef std::shared_ptr<SuperPlanner> Ptr;

        void getOneHeartbeatTime(double &start_WT_pos, bool &traj_finish);

        Trajectory getCommittedPositionTrajectory();

        Trajectory getCommittedYawTrajectory();

        // Runtime safety metadata for the currently committed command. These
        // accessors keep CmdTraj ownership inside SUPER while allowing the
        // mission/PX4 FSM to validate its optimized safety suffix.
        bool committedBackupTrajectoryAvailable() const {
            return cmd_traj_info_.backupTrajAvilibale();
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

        Vec3f latestGuideStart() const { return latest_guide_start_; }
        Vec3f latestGuideEnd() const { return latest_guide_end_; }
        Vec3f latestGuideMin() const { return latest_guide_min_; }
        Vec3f latestGuideMax() const { return latest_guide_max_; }
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
        double solveDeadlineSeconds() const noexcept {
            return cfg_.solve_deadline_s;
        }
        void resetSolveCancellation() noexcept {
            solve_cancelled_.store(false);
        }

        // Planning-thread-only. Runtime pins one immutable revision before a
        // solve; A* and corridor generation receive that same pointer.
        void setWorldModelView(navigation_world_model::WorldModelViewPtr view) {
            if (!view) throw std::invalid_argument("WorldModelView must not be null");
            map_ptr_ = std::move(view);
            astar_ptr_->setWorldModelView(map_ptr_);
            cg_ptr_->setWorldModelView(map_ptr_);
        }

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
            navigation_world_model::WorldSnapshotIdentity certificate_world{};
        };
        CommandSample sampleCommand();

        // Last-resort SUPER-owned braking bundle.  This is intentionally not
        // a main-only adapter trajectory: it is committed as BACKUP only
        // after dynamic and inflated-map gates pass.
        bool commitEmergencyBrake(const StatePVAJ &measured_state,
                                  double measured_yaw,
                                  double measured_yaw_dot,
                                  double start_WT);

        void getModuleTimeConsuming(vector<double> &time);

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
        RET_CODE generateBackupTrajectory(ExpTraj &ref_exp_traj, BackupTraj &back_traj_info);

        int getNearestFurtherGoalPoint(const vec_E<Vec3f> &goals, const Vec3f &start_pt);

        bool PathSearch(const Vec3f &start_pt, const Vec3f &goal,
                        const double &searching_horizon,
                        vec_Vec3f &path,
                        const AbsoluteDeadline &solve_deadline);


    public:
        void getRobotState(super_utils::RobotState &out);

        bool setPlannerExecutionState(const super_utils::RobotState &state);

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
