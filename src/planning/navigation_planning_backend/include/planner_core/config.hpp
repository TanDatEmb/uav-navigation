/*
 * Product-owned navigation implementation.
 * Algorithmic provenance and external attributions are documented in the
 * package documentation; they are not part of the runtime API or behaviour.
 */


#ifndef PLANNER_CONFIG_HPP
#define PLANNER_CONFIG_HPP

#include <planner_core/backup_braking.hpp>
#include <navigation_planning/planning_limits.hpp>
#include <navigation_world_model/world_model_view.hpp>
#include <path_search/config.hpp>
#include <traj_opt/config.hpp>
#include <utils/header/yaml_loader.hpp>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <optional>
#include <stdexcept>

namespace navigation_planning_backend {
    using namespace traj_opt;
    using std::cout;
    using std::endl;

    class Config {
    public:
        traj_opt::Config exp_traj_cfg, back_traj_cfg;
        path_search::PathSearchConfig astar_cfg;

        // Bool Params
        bool visualization_en{true};
        bool use_fov_cut{false};
        bool print_log{false};
        bool goal_vel_en{false};
        bool goal_yaw_en{false};
        bool preserve_backup_altitude{true};
        navigation_world_model::UnknownPolicy unknown_space_policy{
            navigation_world_model::UnknownPolicy::kRequireKnownFree};

        // Bound to the immutable world model after YAML loading. Map
        // resolution has one owner: the world-model snapshot.
        double resolution{0.0};
        double planning_horizon_m{0.0};
        double receding_distance_m{0.0};
        double visibility_horizon_cap_m{0.0};
        double visibility_horizon_floor_m{0.0};
        double visibility_horizon_m{0.0};
        // Zero disables the optional FOV cut; the unit is explicit.
        double sensing_horizon_m{0.0};

        // Planning Params
        double corridor_bound_distance_m{0.0};
        double corridor_segment_max_length_m{0.0};
        double replan_forward_dt_s{0.0};
        double astar_search_time_limit_s{0.0};
        double astar_total_time_limit_s{0.0};
        double solve_deadline_s{0.0};
        double sample_traj_dt_s{0.0};
        double robot_r{0.0};
        double vehicle_radius_m{0.0};
        double tracking_error_budget_m{0.0};
        double localization_error_budget_m{0.0};
        double mapping_error_budget_m{0.0};
        double planning_margin_m{0.0};
        int iris_iter_num{0};

        double yaw_rate_max_rad_s{0.0};

        navigation_math::vec_E<navigation_math::Vec3i> seed_line_neighbour;


        Config() = default;
        explicit Config(
                const std::string &cfg_path,
                const std::optional<navigation_planning::DynamicLimits> &mission_limits = std::nullopt) {
            yaml_loader::YamlLoader loader(cfg_path);
            exp_traj_cfg = traj_opt::Config(loader, "exp_traj");
            back_traj_cfg = traj_opt::Config(loader, "backup_traj");
            astar_cfg = path_search::PathSearchConfig(loader);
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
            loader.LoadParam("planner/visualization_en", visualization_en, false);
            loader.LoadParam("planner/goal_vel_en", goal_vel_en, false);
            loader.LoadParam("planner/goal_yaw_en", goal_yaw_en, false);
            loader.LoadParam("planner/preserve_backup_altitude",
                             preserve_backup_altitude, true);
            loader.LoadParam("planner/use_fov_cut", use_fov_cut, false);
            if (mission_limits.has_value()) {
                unknown_space_policy =
                    mission_limits->unknown_space_policy ==
                            navigation_planning::UnknownSpacePolicy::kRequireKnownFree
                        ? navigation_world_model::UnknownPolicy::kRequireKnownFree
                        : navigation_world_model::UnknownPolicy::kAllowUnknown;
            }
            loader.LoadParam("planner/visibility_horizon_cap_m",
                             visibility_horizon_cap_m, 3.0);
            loader.LoadParam("planner/visibility_horizon_floor_m",
                             visibility_horizon_floor_m,
                             visibility_horizon_cap_m);
            loader.LoadParam("planner/sensing_horizon_m", sensing_horizon_m, 0.0);
            loader.LoadParam("planner/replan_forward_dt_s", replan_forward_dt_s, 0.3);
            loader.LoadParam("astar/search_time_limit_s", astar_search_time_limit_s, 0.1);
            loader.LoadParam("astar/total_time_limit_s", astar_total_time_limit_s,
                             2.0 * astar_search_time_limit_s);
            loader.LoadParam("planner/solve_deadline_s", solve_deadline_s,
                             0.9 * replan_forward_dt_s);
            loader.LoadParam("planner/corridor_bound_distance_m", corridor_bound_distance_m, 3.0);
            loader.LoadParam("planner/corridor_segment_max_length_m",
                             corridor_segment_max_length_m, 3.0);
            loader.LoadParam("planner/planning_horizon_m", planning_horizon_m, 10.0);
            loader.LoadParam("planner/receding_distance_m", receding_distance_m, 5.0);
            loader.LoadParam("planner/vehicle_radius_m", vehicle_radius_m, 0.35);
            loader.LoadParam("planner/tracking_error_budget_m", tracking_error_budget_m, 0.25);
            loader.LoadParam("planner/localization_error_budget_m", localization_error_budget_m, 0.05);
            loader.LoadParam("planner/mapping_error_budget_m", mapping_error_budget_m, 0.10);
            loader.LoadParam("planner/planning_margin_m", planning_margin_m, 0.05);
            loader.LoadParam("planner/iris_iter_num", iris_iter_num, 1);
            loader.LoadParam("planner/yaw_rate_max_rad_s", yaw_rate_max_rad_s, 3.14);

            // The collision radius is a derived safety envelope. There is no
            // independent YAML value that can silently drift from its owners.
            robot_r = vehicle_radius_m + tracking_error_budget_m +
                      localization_error_budget_m + mapping_error_budget_m +
                      planning_margin_m;

            const double required_safety_horizon =
                jerkLimitedStopDistance(exp_traj_cfg.max_vel,
                                        back_traj_cfg.max_acc,
                                        back_traj_cfg.max_jerk) +
                2.0 * exp_traj_cfg.max_vel * replan_forward_dt_s + robot_r;
            visibility_horizon_m =
                std::max(visibility_horizon_floor_m, required_safety_horizon);
            const bool sensing_horizon_enabled = sensing_horizon_m > 0.0;
            if (!std::isfinite(robot_r) || robot_r <= 0.0 ||
                !std::isfinite(vehicle_radius_m) || vehicle_radius_m <= 0.0 ||
                !std::isfinite(tracking_error_budget_m) || tracking_error_budget_m < 0.0 ||
                !std::isfinite(localization_error_budget_m) || localization_error_budget_m < 0.0 ||
                !std::isfinite(mapping_error_budget_m) || mapping_error_budget_m < 0.0 ||
                !std::isfinite(planning_margin_m) || planning_margin_m < 0.0 ||
                !std::isfinite(visibility_horizon_cap_m) ||
                visibility_horizon_cap_m <= 0.0 ||
                !std::isfinite(visibility_horizon_floor_m) ||
                visibility_horizon_floor_m <= 0.0 ||
                visibility_horizon_floor_m > visibility_horizon_cap_m ||
                !std::isfinite(visibility_horizon_m) ||
                visibility_horizon_m <= 0.0 ||
                visibility_horizon_m > visibility_horizon_cap_m + 1.0e-9 ||
                !std::isfinite(sensing_horizon_m) || sensing_horizon_m < 0.0 ||
                (sensing_horizon_enabled && sensing_horizon_m > visibility_horizon_m)) {
                throw std::invalid_argument(
                    "planner safety geometry requires finite collision and visibility horizons; "
                    "sensing_horizon_m must not exceed the effective visibility horizon");
            }
            const double required_robot_radius = vehicle_radius_m + tracking_error_budget_m +
                                                 localization_error_budget_m + mapping_error_budget_m +
                                                 planning_margin_m;
            if (std::abs(robot_r - required_robot_radius) > 1.0e-6) {
                throw std::invalid_argument(
                    "planner robot radius must equal the configured vehicle/tracking/localization/"
                    "mapping/planning safety envelope");
            }
            if (!std::isfinite(replan_forward_dt_s) || replan_forward_dt_s <= 0.0 ||
                !std::isfinite(astar_search_time_limit_s) || astar_search_time_limit_s <= 0.0 ||
                !std::isfinite(astar_total_time_limit_s) ||
                astar_total_time_limit_s < astar_search_time_limit_s ||
                !std::isfinite(solve_deadline_s) || solve_deadline_s <= 0.0 ||
                astar_total_time_limit_s >= solve_deadline_s ||
                solve_deadline_s > replan_forward_dt_s + 1.0e-9) {
                throw std::invalid_argument(
                    "navigation planner deadlines require 0 < A* attempt <= A* total < solve <= "
                    "replan_forward_dt_s");
            }
            if (!std::isfinite(planning_horizon_m) || planning_horizon_m <= 0.0 ||
                !std::isfinite(receding_distance_m) || receding_distance_m <= 0.0 ||
                receding_distance_m >= planning_horizon_m ||
                !std::isfinite(corridor_bound_distance_m) || corridor_bound_distance_m <= 0.0 ||
                !std::isfinite(corridor_segment_max_length_m) ||
                corridor_segment_max_length_m <= 0.0 ||
                iris_iter_num <= 0 || !std::isfinite(yaw_rate_max_rad_s) ||
                yaw_rate_max_rad_s <= 0.0) {
                throw std::invalid_argument(
                    "planner geometric horizon, corridor, IRIS, and yaw parameters are invalid");
            }
            if (!std::isfinite(exp_traj_cfg.max_vel) || exp_traj_cfg.max_vel <= 0.0 ||
                !std::isfinite(exp_traj_cfg.max_acc) || exp_traj_cfg.max_acc <= 0.0 ||
                !std::isfinite(exp_traj_cfg.max_jerk) || exp_traj_cfg.max_jerk <= 0.0 ||
                !std::isfinite(back_traj_cfg.max_vel) || back_traj_cfg.max_vel <= 0.0 ||
                !std::isfinite(back_traj_cfg.max_acc) || back_traj_cfg.max_acc <= 0.0 ||
                !std::isfinite(back_traj_cfg.max_jerk) || back_traj_cfg.max_jerk <= 0.0 ||
                !std::isfinite(exp_traj_cfg.jerk_penalty_weight) ||
                !std::isfinite(back_traj_cfg.jerk_penalty_weight)) {
                throw std::invalid_argument(
                    "navigation planner dynamic limits must be finite and positive; penalty weights must be finite");
            }
            if (back_traj_cfg.max_vel + 1.0e-9 < exp_traj_cfg.max_vel ||
                back_traj_cfg.max_acc + 1.0e-9 < exp_traj_cfg.max_acc ||
                back_traj_cfg.max_jerk + 1.0e-9 < exp_traj_cfg.max_jerk) {
                throw std::invalid_argument(
                    "navigation planner backup limits must cover every inherited EXP boundary state");
            }
            const double visibility_horizon = sensing_horizon_enabled
                ? std::min(sensing_horizon_m, visibility_horizon_m)
                : visibility_horizon_m;
            if (visibility_horizon + 1.0e-9 < required_safety_horizon) {
                throw std::invalid_argument(
                    "navigation planner backup visibility horizon is shorter than braking plus replan and "
                    "safety margin");
            }

        }

        void bindWorldGeometry(
                const navigation_world_model::WorldGeometry &world_geometry) {
            const double world_resolution = world_geometry.evidence_resolution_m;
            if (!std::isfinite(world_resolution) || world_resolution <= 0.0) {
                throw std::invalid_argument(
                    "world geometry evidence resolution must be finite and positive");
            }
            if (!std::isfinite(world_geometry.inflated_resolution_m) ||
                world_geometry.inflated_resolution_m <= 0.0 ||
                !std::isfinite(world_geometry.occupied_inflation_radius_m) ||
                world_geometry.occupied_inflation_radius_m + 1.0e-9 < robot_r ||
                !world_geometry.local_center_m.allFinite() ||
                !world_geometry.local_size_m.allFinite() ||
                (world_geometry.local_size_m.array() <= 0.0).any() ||
                !std::isfinite(world_geometry.effective_virtual_ground_m) ||
                !std::isfinite(world_geometry.effective_virtual_ceiling_m) ||
                world_geometry.effective_virtual_ground_m >=
                    world_geometry.effective_virtual_ceiling_m) {
                throw std::invalid_argument(
                    "world geometry must provide a finite map, ordered vertical bounds, and "
                    "inflation at least as large as the planner safety envelope");
            }
            const double minimum_map_extent = 2.0 * robot_r + world_resolution;
            // The local evidence map is intentionally allowed to be
            // anisotropic. Route-specific support is checked at runtime from
            // the measured start toward the requested route direction; doing
            // that here would either reject valid short goals or require an
            // unbenchmarked square map. Do not use maxCoeff(X,Y) as a claim
            // that the planning horizon is available in every direction.
            if (world_geometry.local_size_m.minCoeff() < minimum_map_extent - 1.0e-9) {
                throw std::invalid_argument(
                    "world map extent is insufficient for the planner safety envelope");
            }
            if (corridor_bound_distance_m + 1.0e-9 < robot_r ||
                corridor_segment_max_length_m + 1.0e-9 < world_resolution) {
                throw std::invalid_argument(
                    "corridor geometry must cover the planner safety envelope and at least one "
                    "world cell");
            }
            resolution = world_resolution;
            sample_traj_dt_s = resolution / exp_traj_cfg.max_vel;
            if (!std::isfinite(sample_traj_dt_s) || sample_traj_dt_s <= 0.0) {
                throw std::invalid_argument(
                    "planner trajectory sample period is not finite and positive");
            }

            const double step_real = std::ceil(robot_r / resolution);
            if (!std::isfinite(step_real) || step_real < 1.0 ||
                step_real > static_cast<double>(std::numeric_limits<int>::max() - 1)) {
                throw std::invalid_argument(
                    "planner safety envelope cannot be represented on the world grid");
            }
            const int step = static_cast<int>(step_real);
            seed_line_neighbour.clear();
            for (int x = -step; x <= step; ++x) {
                for (int y = -step; y <= step; ++y) {
                    for (int z = -step; z <= step; ++z) {
                        const std::int64_t distance_squared =
                            static_cast<std::int64_t>(x) * x +
                            static_cast<std::int64_t>(y) * y +
                            static_cast<std::int64_t>(z) * z;
                        if (distance_squared <=
                            static_cast<std::int64_t>(step) * step) {
                            seed_line_neighbour.push_back({x, y, z});
                        }
                    }
                }
            }
            std::sort(seed_line_neighbour.begin(), seed_line_neighbour.end(),
                      [](const auto &a, const auto &b) {
                          const std::int64_t a_squared =
                              static_cast<std::int64_t>(a[0]) * a[0] +
                              static_cast<std::int64_t>(a[1]) * a[1] +
                              static_cast<std::int64_t>(a[2]) * a[2];
                          const std::int64_t b_squared =
                              static_cast<std::int64_t>(b[0]) * b[0] +
                              static_cast<std::int64_t>(b[1]) * b[1] +
                              static_cast<std::int64_t>(b[2]) * b[2];
                          return a_squared < b_squared;
                      });
        }


    };
}

#endif
