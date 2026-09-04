/*
 * Product-owned navigation implementation.
 * Algorithmic provenance and external attributions are documented in the
 * package documentation; they are not part of the runtime API or behaviour.
 */


#ifndef PLANNER_CONFIG_HPP
#define PLANNER_CONFIG_HPP

#include <planner_core/backup_braking.hpp>
#include <planner_core/route_yaw_reference.hpp>
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
        inline static const Eigen::Vector3d kProductMapSizeM{50.0, 50.0, 8.0};

        traj_opt::Config exp_traj_cfg, back_traj_cfg;
        path_search::PathSearchConfig astar_cfg;

        // Bool Params
        bool visualization_en{true};
        bool use_fov_cut{false};
        bool print_log{false};
        bool preserve_backup_altitude{true};
        // The deterministic minimum-snap BACKUP seed is the release safety
        // artifact. Optional L-BFGS refinement is opt-in for experiments and
        // can never be required for a complete candidate.
        bool backup_refinement_enabled{false};
        navigation_world_model::UnknownPolicy unknown_space_policy{
            navigation_world_model::UnknownPolicy::kRequireKnownFree};

        // Bound to the immutable world model after YAML loading. Map
        // resolution has one owner: the world-model snapshot.
        double resolution{0.0};
        double local_window_m{0.0};
        double receding_distance_m{0.0};
        double visibility_horizon_cap_m{0.0};
        double visibility_horizon_floor_m{0.0};
        double visibility_horizon_m{0.0};
        // Mission intent is a requested cruise speed, not a physical model
        // limit. The immutable vehicle model remains in `DynamicLimits`.
        double requested_cruise_speed_mps{0.0};
        // Product-owned nominal closed-loop capability. This is applied only
        // to the MAIN optimizer; BACKUP/EMERGENCY retain the hard physical
        // limits loaded from traj_opt/boundary.
        navigation_planning::VehicleControlEnvelope control_envelope{};
        double effective_cruise_speed_mps{0.0};
        // Zero disables the optional FOV cut; the unit is explicit.
        double sensing_horizon_m{0.0};

        // Planning Params
        double corridor_bound_distance_m{0.0};
        double corridor_segment_max_length_m{0.0};
        double replan_forward_dt_s{0.0};
        double astar_search_time_limit_s{0.0};
        double astar_total_time_limit_s{0.0};
        double solve_deadline_s{0.0};
        double finalization_reserve_s{0.04};
        double sample_traj_dt_s{0.0};
        double robot_r{0.0};
        double vehicle_radius_m{0.0};
        double tracking_error_budget_m{0.0};
        double localization_error_budget_m{0.0};
        double mapping_error_budget_m{0.0};
        double planning_margin_m{0.0};
        int iris_iter_num{0};

        double yaw_rate_max_rad_s{0.0};
        double yaw_acceleration_max_rad_s2{0.0};
        RouteYawConfig route_yaw_config{};
        // Recovery trigger for a committed yaw state that the vehicle did
        // not track. This is a continuity/rebase envelope, not a waypoint or
        // safety acceptance gate.
        double yaw_tracking_error_budget_rad{0.0};

        navigation_math::vec_E<navigation_math::Vec3i> seed_line_neighbour;


        Config() = default;
        explicit Config(
                const std::string &cfg_path,
                const std::optional<navigation_planning::DynamicLimits> &mission_limits = std::nullopt) {
            yaml_loader::YamlLoader loader(cfg_path);
            exp_traj_cfg = traj_opt::Config(loader, "exp_traj");
            back_traj_cfg = traj_opt::Config(loader, "backup_traj");
            astar_cfg = path_search::PathSearchConfig(loader);
            loader.LoadParam("planner/control_envelope/maximum_velocity_mps",
                             control_envelope.maximum_velocity_mps, 0.0);
            loader.LoadParam("planner/control_envelope/maximum_acceleration_mps2",
                             control_envelope.maximum_acceleration_mps2, 0.0);
            loader.LoadParam("planner/control_envelope/maximum_jerk_mps3",
                             control_envelope.maximum_jerk_mps3, 0.0);
            if (mission_limits.has_value()) {
                const auto &limits = *mission_limits;
                if (!limits.valid()) {
                    throw std::invalid_argument(
                        "mission dynamic limits or unknown-space policy are invalid");
                }
                // The mission owns intent only. Its requested speed may be
                // above the nominal closed-loop envelope, but must still fit
                // the hard physical model loaded from the planner config.
                if (limits.intent.requested_cruise_speed_mps >
                        exp_traj_cfg.max_vel + 1.0e-9 ||
                    limits.vehicle.maximum_acceleration_mps2 > exp_traj_cfg.max_acc + 1.0e-9 ||
                    limits.vehicle.maximum_jerk_mps3 > exp_traj_cfg.max_jerk + 1.0e-9 ||
                    limits.vehicle.maximum_acceleration_mps2 > back_traj_cfg.max_acc + 1.0e-9 ||
                    limits.vehicle.maximum_jerk_mps3 > back_traj_cfg.max_jerk + 1.0e-9) {
                    throw std::invalid_argument(
                        "mission requested speed or vehicle model exceeds the physical envelope");
                }
                requested_cruise_speed_mps = limits.intent.requested_cruise_speed_mps;
            }
            loader.LoadParam("planner/print_log", print_log, false);
            loader.LoadParam("planner/visualization_en", visualization_en, false);
            const YAML::Node planner_node = loader.document()["planner"];
            if (planner_node.IsDefined() &&
                planner_node["goal_vel_en"].IsDefined()) {
                throw std::invalid_argument(
                    "planner/goal_vel_en was removed; terminal state is owned by waypoint behavior");
            }
            loader.LoadParam("planner/preserve_backup_altitude",
                             preserve_backup_altitude, true);
            loader.LoadParam("planner/backup_refinement_enabled",
                             backup_refinement_enabled, false);
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
            loader.LoadParam("planner/replan_forward_dt_s", replan_forward_dt_s, 0.4);
            loader.LoadParam("astar/search_time_limit_s", astar_search_time_limit_s, 0.1);
            loader.LoadParam("astar/total_time_limit_s", astar_total_time_limit_s,
                             2.0 * astar_search_time_limit_s);
            loader.LoadParam("planner/solve_deadline_s", solve_deadline_s,
                             0.9 * replan_forward_dt_s);
            loader.LoadParam("planner/finalization_reserve_s",
                             finalization_reserve_s, 0.04);
            loader.LoadParam("planner/corridor_bound_distance_m", corridor_bound_distance_m, 3.0);
            loader.LoadParam("planner/corridor_segment_max_length_m",
                             corridor_segment_max_length_m, 3.0);
            const bool has_local_window = loader.LoadParam(
                "planner/local_window_m", local_window_m, 20.0);
            double legacy_planning_horizon_m = 0.0;
            const bool has_legacy_planning_horizon = loader.LoadParam(
                "planner/planning_horizon_m", legacy_planning_horizon_m, 0.0);
            if (has_local_window && has_legacy_planning_horizon) {
                throw std::invalid_argument(
                    "planner/planning_horizon_m is not accepted when planner/local_window_m is present");
            }
            if (!has_local_window) {
                if (!has_legacy_planning_horizon ||
                    !std::isfinite(legacy_planning_horizon_m) ||
                    std::abs(legacy_planning_horizon_m - 20.0) > 1.0e-9) {
                    throw std::invalid_argument(
                        "planner/local_window_m is required; legacy planning_horizon_m is accepted only at 20.0");
                }
                local_window_m = legacy_planning_horizon_m;
                std::cerr
                    << "DEPRECATED planner/planning_horizon_m=20.0; use planner/local_window_m"
                    << std::endl;
            }
            loader.LoadParam("planner/receding_distance_m", receding_distance_m, 5.0);
            loader.LoadParam("planner/vehicle_radius_m", vehicle_radius_m, 0.35);
            loader.LoadParam("planner/tracking_error_budget_m", tracking_error_budget_m, 0.25);
            loader.LoadParam("planner/localization_error_budget_m", localization_error_budget_m, 0.05);
            loader.LoadParam("planner/mapping_error_budget_m", mapping_error_budget_m, 0.10);
            loader.LoadParam("planner/planning_margin_m", planning_margin_m, 0.05);
            loader.LoadParam("planner/iris_iter_num", iris_iter_num, 1);
            loader.LoadParam("planner/yaw_rate_max_rad_s", yaw_rate_max_rad_s, 3.14);
            loader.LoadParam("planner/yaw_acceleration_max_rad_s2",
                             yaw_acceleration_max_rad_s2, 0.3);
            const navigation_planning::VehicleDynamicModel exp_physical_model{
                exp_traj_cfg.max_vel, exp_traj_cfg.max_acc, exp_traj_cfg.max_jerk,
                exp_traj_cfg.max_omg, exp_traj_cfg.max_omg,
                yaw_acceleration_max_rad_s2, exp_traj_cfg.min_acc_thr * exp_traj_cfg.mass,
                exp_traj_cfg.max_acc_thr * exp_traj_cfg.mass, exp_traj_cfg.mass};
            const navigation_planning::VehicleDynamicModel backup_physical_model{
                back_traj_cfg.max_vel, back_traj_cfg.max_acc, back_traj_cfg.max_jerk,
                back_traj_cfg.max_omg, back_traj_cfg.max_omg,
                yaw_acceleration_max_rad_s2, back_traj_cfg.min_acc_thr * back_traj_cfg.mass,
                back_traj_cfg.max_acc_thr * back_traj_cfg.mass, back_traj_cfg.mass};
            if (!control_envelope.valid(exp_physical_model) ||
                !control_envelope.valid(backup_physical_model)) {
                throw std::invalid_argument(
                    "planner control envelope must be finite, positive, and no greater than "
                    "the physical traj_opt boundary");
            }
            if (!std::isfinite(requested_cruise_speed_mps) ||
                requested_cruise_speed_mps <= 0.0) {
                requested_cruise_speed_mps = exp_physical_model.maximum_velocity_mps;
            }
            effective_cruise_speed_mps = std::min(
                requested_cruise_speed_mps,
                control_envelope.maximum_velocity_mps);
            if (!std::isfinite(effective_cruise_speed_mps) ||
                effective_cruise_speed_mps <= 0.0) {
                throw std::invalid_argument(
                    "planner effective cruise speed must be finite and positive");
            }
            // MAIN owns the nominal closed-loop envelope. The separately
            // loaded BACKUP limits are intentionally not overwritten here.
            exp_traj_cfg.max_vel = effective_cruise_speed_mps;
            exp_traj_cfg.max_acc = control_envelope.maximum_acceleration_mps2;
            exp_traj_cfg.max_jerk = control_envelope.maximum_jerk_mps3;
            loader.LoadParam("planner/route_yaw/minimum_lookahead_m",
                             route_yaw_config.minimum_lookahead_m, 2.0);
            loader.LoadParam("planner/route_yaw/maximum_lookahead_m",
                             route_yaw_config.maximum_lookahead_m, 12.0);
            loader.LoadParam("planner/route_yaw/lookahead_time_s",
                             route_yaw_config.lookahead_time_s, 1.5);
            loader.LoadParam("planner/route_yaw/minimum_horizontal_speed_mps",
                             route_yaw_config.minimum_horizontal_speed_mps, 0.3);
            loader.LoadParam("planner/route_yaw/minimum_horizontal_support_m",
                             route_yaw_config.minimum_horizontal_support_m, 0.5);
            loader.LoadParam("planner/route_yaw/reversal_threshold_rad",
                             route_yaw_config.reversal_threshold_rad,
                             2.6179938779914944);
            loader.LoadParam("planner/yaw_tracking_error_budget_rad",
                             yaw_tracking_error_budget_rad, 0.35);

            // The collision radius is a derived safety envelope. There is no
            // independent YAML value that can silently drift from its owners.
            robot_r = vehicle_radius_m + tracking_error_budget_m +
                      localization_error_budget_m + mapping_error_budget_m +
                      planning_margin_m;

            const double required_safety_horizon =
                jerkLimitedStopDistance(effective_cruise_speed_mps,
                        exp_traj_cfg.max_acc,
                        exp_traj_cfg.max_jerk) +
                2.0 * effective_cruise_speed_mps * replan_forward_dt_s + robot_r;
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
                !std::isfinite(finalization_reserve_s) ||
                finalization_reserve_s < 0.04 ||
                finalization_reserve_s >= solve_deadline_s ||
                astar_total_time_limit_s >= solve_deadline_s ||
                solve_deadline_s > replan_forward_dt_s + 1.0e-9) {
                throw std::invalid_argument(
                    "navigation planner deadlines require 0 < A* attempt <= A* total < solve <= "
                    "replan_forward_dt_s and 0.04 <= finalization reserve < solve");
            }
            if (!std::isfinite(local_window_m) ||
                std::abs(local_window_m - 20.0) > 1.0e-9 ||
                !std::isfinite(receding_distance_m) || receding_distance_m <= 0.0 ||
                receding_distance_m >= local_window_m ||
                !std::isfinite(corridor_bound_distance_m) || corridor_bound_distance_m <= 0.0 ||
                !std::isfinite(corridor_segment_max_length_m) ||
                corridor_segment_max_length_m <= 0.0 ||
                iris_iter_num <= 0 || !std::isfinite(yaw_rate_max_rad_s) ||
                yaw_rate_max_rad_s <= 0.0 ||
                !std::isfinite(yaw_acceleration_max_rad_s2) ||
                yaw_acceleration_max_rad_s2 <= 0.0 ||
                !route_yaw_config.valid() ||
                !std::isfinite(yaw_tracking_error_budget_rad) ||
                yaw_tracking_error_budget_rad <= 0.0 ||
                yaw_tracking_error_budget_rad > M_PI) {
                throw std::invalid_argument(
                    "planner local window must equal 20 m; corridor, IRIS, and yaw parameters must be valid");
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

        [[nodiscard]] navigation_planning::VehicleDynamicModel physical_model() const {
            return navigation_planning::VehicleDynamicModel{
                back_traj_cfg.max_vel, back_traj_cfg.max_acc, back_traj_cfg.max_jerk,
                back_traj_cfg.max_omg, back_traj_cfg.max_omg,
                yaw_acceleration_max_rad_s2, back_traj_cfg.min_acc_thr * back_traj_cfg.mass,
                back_traj_cfg.max_acc_thr * back_traj_cfg.mass, back_traj_cfg.mass};
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
            // ROG-Map rounds each requested dimension upward by at most one
            // evidence cell. Validate every axis independently so excess
            // support on one axis cannot hide insufficient or drifted support
            // on another axis.
            for (int axis = 0; axis < 3; ++axis) {
                const double actual_extent_m = world_geometry.local_size_m(axis);
                const double requested_extent_m = kProductMapSizeM(axis);
                if (actual_extent_m + 1.0e-9 < requested_extent_m ||
                    actual_extent_m > requested_extent_m + world_resolution + 1.0e-9) {
                    throw std::invalid_argument(
                        "world map extent does not match the locked 50 x 50 x 8 m product geometry on every axis");
                }
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

            constexpr long double kMaximumNeighbourStep = 64.0L;
            constexpr long double kMaximumNeighbourCells = 2'000'000.0L;
            const long double step_real = std::ceil(
                static_cast<long double>(robot_r) /
                static_cast<long double>(resolution));
            if (!std::isfinite(step_real) || step_real < 1.0L ||
                step_real > kMaximumNeighbourStep) {
                throw std::invalid_argument(
                    "planner safety envelope cannot be represented on the world grid");
            }
            const long double neighbour_side = 2.0L * step_real + 1.0L;
            if (neighbour_side * neighbour_side * neighbour_side >
                kMaximumNeighbourCells) {
                throw std::invalid_argument(
                    "planner safety-neighbor generation exceeds the bounded resource envelope");
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
