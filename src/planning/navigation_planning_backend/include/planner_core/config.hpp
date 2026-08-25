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


#ifndef PLANNER_CONFIG_HPP
#define PLANNER_CONFIG_HPP

#include <planner_core/backup_braking.hpp>
#include <navigation_planning/planning_limits.hpp>
#include <traj_opt/config.hpp>
#include <utils/header/yaml_loader.hpp>

#include <cmath>
#include <optional>
#include <stdexcept>

namespace navigation_planning_backend {
    using namespace traj_opt;
    using std::cout;
    using std::endl;

    using DynamicLimits = navigation_planning::DynamicLimits;

    class Config {
    public:
        enum YawMode{
            YAW_TO_VEL = 1,
            YAW_TO_GOAL = 2
        };

        traj_opt::Config exp_traj_cfg, back_traj_cfg;

        // Bool Params
        bool visualization_en{true};
        bool detailed_log_en{false};
        bool backup_traj_en;
        bool use_fov_cut, print_log;
        bool goal_vel_en,goal_yaw_en;
        bool visual_process;
        bool frontend_in_known_free;

        double resolution;
        double planning_horizon;
        double receding_dis;
        double safe_corridor_line_max_length;
        double safe_corridor_line_nominal_length;
        // for fov cut
        double sensing_horizon;

        // Planning Params
        int obs_skip_num;
        double corridor_bound_dis, corridor_line_max_length;
        double replan_forward_dt;
        double astar_search_time_limit_s;
        double astar_total_time_limit_s;
        double solve_deadline_s;
        double sample_traj_dt;
        double robot_r;
        double vehicle_radius_m;
        double tracking_error_budget_m;
        double localization_error_budget_m;
        double mapping_error_budget_m;
        double planning_margin_m;
        int iris_iter_num;

        int mpc_horizon{};

        double yaw_dot_max;
        // Yaw mode: 1 heading to velocity, 2 heading to goal
        int yaw_mode = YAW_TO_VEL;

        navigation_math::vec_E<navigation_math::Vec3i> seed_line_neighbour;


        Config() = default;
        explicit Config(
                const std::string &cfg_path,
                const std::optional<DynamicLimits> &mission_limits = std::nullopt) {
            yaml_loader::YamlLoader loader(cfg_path);
            exp_traj_cfg = traj_opt::Config(cfg_path, "exp_traj");
            back_traj_cfg = traj_opt::Config(cfg_path, "backup_traj");
            if (mission_limits.has_value()) {
                const auto &limits = *mission_limits;
                if (!std::isfinite(limits.max_velocity_mps) ||
                    !std::isfinite(limits.max_acceleration_mps2) ||
                    !std::isfinite(limits.max_jerk_mps3) ||
                    limits.max_velocity_mps <= 0.0 ||
                    limits.max_acceleration_mps2 <= 0.0 ||
                    limits.max_jerk_mps3 <= 0.0) {
                    throw std::invalid_argument(
                        "mission dynamic limits must be finite and positive");
                }
                if (limits.max_velocity_mps > exp_traj_cfg.max_vel + 1.0e-9 ||
                    limits.max_acceleration_mps2 > exp_traj_cfg.max_acc + 1.0e-9 ||
                    limits.max_jerk_mps3 > exp_traj_cfg.max_jerk + 1.0e-9 ||
                    limits.max_velocity_mps > back_traj_cfg.max_vel + 1.0e-9 ||
                    limits.max_acceleration_mps2 > back_traj_cfg.max_acc + 1.0e-9 ||
                    limits.max_jerk_mps3 > back_traj_cfg.max_jerk + 1.0e-9) {
                    throw std::invalid_argument(
                        "mission dynamic limits exceed the product envelope");
                }
                exp_traj_cfg.max_vel = limits.max_velocity_mps;
                exp_traj_cfg.max_acc = limits.max_acceleration_mps2;
                exp_traj_cfg.max_jerk = limits.max_jerk_mps3;
                back_traj_cfg.max_vel = limits.max_velocity_mps;
                back_traj_cfg.max_acc = limits.max_acceleration_mps2;
                back_traj_cfg.max_jerk = limits.max_jerk_mps3;
            }
            loader.LoadParam("planner/print_log", print_log, false);
            loader.LoadParam("planner/detailed_log_en", detailed_log_en, false);
            loader.LoadParam("planner/visualization_en", visualization_en, false);
            loader.LoadParam("planner/backup_traj_en", backup_traj_en, false);
            loader.LoadParam("planner/goal_vel_en", goal_vel_en, false);
            loader.LoadParam("planner/goal_yaw_en", goal_yaw_en, false);
            loader.LoadParam("planner/visual_process", visual_process, false);
            loader.LoadParam("planner/use_fov_cut", use_fov_cut, false);
            loader.LoadParam("planner/frontend_in_known_free", frontend_in_known_free, false);
            loader.LoadParam("planner/safe_corridor_line_max_length", safe_corridor_line_max_length, 3.0);
            loader.LoadParam("planner/safe_corridor_line_nominal_length",
                             safe_corridor_line_nominal_length,
                             safe_corridor_line_max_length);
            loader.LoadParam("planner/sensing_horizon", sensing_horizon, 3.0);
            loader.LoadParam("planner/obs_skip_num", obs_skip_num, 1);
            loader.LoadParam("planner/replan_forward_dt", replan_forward_dt, 0.3);
            loader.LoadParam("astar/search_time_limit_s", astar_search_time_limit_s, 0.1);
            loader.LoadParam("astar/total_time_limit_s", astar_total_time_limit_s,
                             2.0 * astar_search_time_limit_s);
            loader.LoadParam("planner/solve_deadline_s", solve_deadline_s,
                             0.9 * replan_forward_dt);
            loader.LoadParam("planner/corridor_bound_dis", corridor_bound_dis, 3.0);
            loader.LoadParam("planner/corridor_line_max_length", corridor_line_max_length, 3.0);
            loader.LoadParam("planner/planning_horizon", planning_horizon, 10.0);
            loader.LoadParam("planner/receding_dis", receding_dis, 5.0);
            loader.LoadParam("planner/robot_r", robot_r, 0.3);
            loader.LoadParam("planner/vehicle_radius_m", vehicle_radius_m, 0.35);
            loader.LoadParam("planner/tracking_error_budget_m", tracking_error_budget_m, 0.25);
            loader.LoadParam("planner/localization_error_budget_m", localization_error_budget_m, 0.05);
            loader.LoadParam("planner/mapping_error_budget_m", mapping_error_budget_m, 0.10);
            loader.LoadParam("planner/planning_margin_m", planning_margin_m, 0.05);
            loader.LoadParam("planner/iris_iter_num", iris_iter_num, 1);
            loader.LoadParam("planner/yaw_mode", yaw_mode, 1);
            loader.LoadParam("planner/mpc_horizon", mpc_horizon, 1);
            loader.LoadParam("planner/yaw_dot_max", yaw_dot_max, 3.14);

            loader.LoadParam("rog_map/resolution", resolution, 0.01, true);

            const double configured_visibility_cap = safe_corridor_line_max_length;
            const double required_safety_horizon =
                jerkLimitedStopDistance(exp_traj_cfg.max_vel,
                                        back_traj_cfg.max_acc,
                                        back_traj_cfg.max_jerk) +
                2.0 * exp_traj_cfg.max_vel * replan_forward_dt + robot_r;
            safe_corridor_line_max_length =
                std::max(safe_corridor_line_nominal_length, required_safety_horizon);
            const bool sensing_horizon_enabled = sensing_horizon > 0.0;
            if (!std::isfinite(resolution) || resolution <= 0.0 ||
                !std::isfinite(robot_r) || robot_r <= 0.0 ||
                !std::isfinite(vehicle_radius_m) || vehicle_radius_m <= 0.0 ||
                !std::isfinite(tracking_error_budget_m) || tracking_error_budget_m < 0.0 ||
                !std::isfinite(localization_error_budget_m) || localization_error_budget_m < 0.0 ||
                !std::isfinite(mapping_error_budget_m) || mapping_error_budget_m < 0.0 ||
                !std::isfinite(planning_margin_m) || planning_margin_m < 0.0 ||
                !std::isfinite(safe_corridor_line_max_length) ||
                safe_corridor_line_max_length <= 0.0 ||
                !std::isfinite(safe_corridor_line_nominal_length) ||
                safe_corridor_line_nominal_length <= 0.0 ||
                safe_corridor_line_nominal_length > configured_visibility_cap ||
                safe_corridor_line_max_length > configured_visibility_cap + 1.0e-9 ||
                !std::isfinite(sensing_horizon) ||
                (sensing_horizon_enabled && sensing_horizon > safe_corridor_line_max_length)) {
                throw std::invalid_argument(
                    "planner safety geometry requires finite positive robot_r and "
                    "safe_corridor_line_max_length; a positive sensing_horizon must not exceed "
                    "the safe corridor length");
            }
            const double required_robot_radius = vehicle_radius_m + tracking_error_budget_m +
                                                 localization_error_budget_m + mapping_error_budget_m +
                                                 planning_margin_m;
            if (std::abs(robot_r - required_robot_radius) > 1.0e-6) {
                throw std::invalid_argument(
                    "SUPER robot_r must equal the configured vehicle/tracking/localization/"
                    "mapping/planning safety envelope");
            }
            if (!std::isfinite(replan_forward_dt) || replan_forward_dt <= 0.0 ||
                !std::isfinite(astar_search_time_limit_s) || astar_search_time_limit_s <= 0.0 ||
                !std::isfinite(astar_total_time_limit_s) ||
                astar_total_time_limit_s < astar_search_time_limit_s ||
                !std::isfinite(solve_deadline_s) || solve_deadline_s <= 0.0 ||
                astar_total_time_limit_s >= solve_deadline_s ||
                solve_deadline_s > replan_forward_dt + 1.0e-9) {
                throw std::invalid_argument(
                    "SUPER deadlines require 0 < A* attempt <= A* total < solve <= "
                    "replan_forward_dt");
            }
            if (!std::isfinite(exp_traj_cfg.max_vel) || exp_traj_cfg.max_vel <= 0.0 ||
                !std::isfinite(exp_traj_cfg.max_acc) || exp_traj_cfg.max_acc <= 0.0 ||
                !std::isfinite(exp_traj_cfg.max_jerk) || exp_traj_cfg.max_jerk <= 0.0 ||
                !std::isfinite(back_traj_cfg.max_vel) || back_traj_cfg.max_vel <= 0.0 ||
                !std::isfinite(back_traj_cfg.max_acc) || back_traj_cfg.max_acc <= 0.0 ||
                !std::isfinite(back_traj_cfg.max_jerk) || back_traj_cfg.max_jerk <= 0.0 ||
                !std::isfinite(exp_traj_cfg.penna_jerk) ||
                !std::isfinite(back_traj_cfg.penna_jerk)) {
                throw std::invalid_argument(
                    "SUPER dynamic limits must be finite and positive; penalty weights must be finite");
            }
            if (back_traj_cfg.max_vel + 1.0e-9 < exp_traj_cfg.max_vel ||
                back_traj_cfg.max_acc + 1.0e-9 < exp_traj_cfg.max_acc ||
                back_traj_cfg.max_jerk + 1.0e-9 < exp_traj_cfg.max_jerk) {
                throw std::invalid_argument(
                    "SUPER backup limits must cover every inherited EXP boundary state");
            }
            const double visibility_horizon = sensing_horizon_enabled
                ? std::min(sensing_horizon, safe_corridor_line_max_length)
                : safe_corridor_line_max_length;
            if (visibility_horizon + 1.0e-9 < required_safety_horizon) {
                throw std::invalid_argument(
                    "SUPER backup visibility horizon is shorter than braking plus replan and "
                    "safety margin");
            }

            sample_traj_dt = resolution / exp_traj_cfg.max_vel;

            int step = ceil(robot_r / resolution);
            for (int x = -step; x <= step; x++) {
                for (int y = -step; y <= step; y++) {
                    for (int z = -step; z <= step; z++) {
                        if (x * x + y * y + z * z <= step * step) {
                            seed_line_neighbour.push_back({x, y, z});
                        }
                    }
                }
            }
            std::sort(seed_line_neighbour.begin(), seed_line_neighbour.end(),
                      [](const auto& a, const auto& b) {
                          return a[0] * a[0] + a[1] * a[1] + a[2] * a[2] < b[0] * b[0] + b[1] * b[1] + b[2] * b[2];
                      });
        }


    };
}

#endif
