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

#ifndef LOG_UTILS_HPP
#define LOG_UTILS_HPP

#include "ros_interface/ros_interface.hpp"

namespace super_planner {
    enum LogTime {
        EPX_TRAJ_FRONTEND = 0,
        EXP_TRAJ_OPT,
        GENERATE_EXP_TRAJ,
        TOTAL_REPLAN,
        VISUALIZATION
    };

    static vector<string> log_time_str
            {
                    "EPX_TRAJ_FRONTEND",
                    " EXP_TRAJ_OPT",
                    " GENERATE_EXP_TRAJ",
                    " TOTAL_REPLAN",
                    " VISUALIZATION"
            };

    class LogOneReplan {
        // replan goal
        Vec3f robot_p;
        Vec4f robot_q;
        Vec3f goal_p;
        double goal_yaw;
        double replan_stamp;
        Vec3f local_start_p;
        // for path search
        vec_Vec3f reference_path;
        vec_Vec3f pc_for_sfc;
        PolytopeVec exp_sfc;

        // for exp traj
        StatePVAJ exp_init_state, exp_fina_state;
        VecDf exp_init_t_vec;
        vec_Vec3f exp_init_ps;
        Trajectory exp_traj;
        Trajectory exp_yaw_traj;

        // RET
        int ret_code{SUPPER_UNDEFINED};

        // comp_t
        double mapping_t{0.0}, astar_t{0.0}, exp_sfc_t{0.0}, exp_opt_t{0.0},
                total_t{0.0}, viz_t{0.0};

    public:
        void print() {
            fmt::print("Goal: {}\n", goal_p.transpose());
            fmt::print("Goal yaw: {}\n", goal_yaw);
        }

        int getRetCode() const {
            return ret_code;
        }

        Vec3f getRobotP() const {
            return robot_p;
        }

        double getTotalCompT() const {
            return total_t;
        }

        void visualize(ros_interface::RosInterface::Ptr &viz_ptr) {
            viz_ptr->vizReplanLog(exp_traj, exp_yaw_traj, exp_sfc, pc_for_sfc, ret_code);
            viz_ptr->vizFrontendPath(exp_init_ps);
        }

    public:
        template<class Archive>
        void serialize(Archive &archive) {
            archive(robot_p, robot_q,
                    goal_p, goal_yaw, replan_stamp, local_start_p, reference_path,
                    pc_for_sfc,
                    exp_sfc,
                    exp_init_state, exp_fina_state, exp_init_t_vec, exp_init_ps, exp_traj, exp_yaw_traj,
                    ret_code,
                    mapping_t, astar_t, exp_sfc_t, exp_opt_t, total_t, viz_t);
        }

        void reset() {
            mapping_t = 0.0;
            astar_t = 0.0;
            exp_sfc_t = 0.0;
            exp_opt_t = 0.0;
            total_t = 0.0;
            viz_t = 0.0;
            ret_code = SUPER_RET_CODE::SUPPER_UNDEFINED;
            exp_traj.clear();
            exp_yaw_traj.clear();
            exp_sfc.clear();
            exp_init_ps.clear();
            reference_path.clear();
        }

        void setGoal(const Vec3f &_goal_p, const double &_goal_yaw,
                     const super_utils::RobotState &_robot) {
            robot_p = _robot.p;
            robot_q = _robot.q.coeffs();
            goal_p = _goal_p;
            goal_yaw = _goal_yaw;
        }

        void setRetCode(const int &_ret) {
            ret_code = _ret;
        }

        void setLocalStartP(const Vec3f &_local_start_p) {
            local_start_p = _local_start_p;
        }


        void setGuidePath(const vec_Vec3f &_path) {
            reference_path = _path;
        }

        void setExpCondition(const VecDf &_ts, const vec_Vec3f &_ps,
                             const StatePVAJ &_init_state, const StatePVAJ &_fina_state,
                             const PolytopeVec &_sfcs) {
            exp_init_t_vec = _ts;
            exp_init_ps = _ps;
            exp_init_state = _init_state;
            exp_fina_state = _fina_state;
            exp_sfc = _sfcs;
        }

        void setExpTraj(const Trajectory &_traj) {
            exp_traj = _traj;
        }

        void setExpYawTraj(const Trajectory &_traj) {
            exp_yaw_traj = _traj;
        }

        void setComptT(const vector<double> &comp_times) {
            astar_t = comp_times[LogTime::EPX_TRAJ_FRONTEND];
            exp_opt_t = comp_times[LogTime::EXP_TRAJ_OPT];
            viz_t = comp_times[LogTime::VISUALIZATION];
            total_t = astar_t + exp_opt_t + viz_t;
        }

        void setSfcPc(const vec_Vec3f &_pc) {
            pc_for_sfc = _pc;
        }

        void replanExpTrajectory(const ExpTrajOpt::Ptr &exp_traj_opt, Trajectory &traj) {
            if (exp_sfc.size() == 0 || exp_init_ps.empty() || exp_init_t_vec.size() == 0) {
                return;
            }
            fmt::print("Replan exp trajectory==========================\n");
            fmt::print("init_state: {}\n", exp_init_state);
            fmt::print("fina_state: {}\n", exp_fina_state);
            fmt::print("sfcs: {}\n", exp_sfc.size());
            fmt::print("init_ps: \n{}\n", exp_init_ps.size());
            fmt::print("init_ts: {}\n", exp_init_t_vec.transpose());

            traj.clear();
            exp_traj_opt->optimize(exp_init_state,
                                   exp_fina_state,
                                   exp_sfc,
                                   exp_init_ps,
                                   exp_init_t_vec,
                                   traj
            );
        }

    };

}

#endif //LOG_UTILS_HPP
