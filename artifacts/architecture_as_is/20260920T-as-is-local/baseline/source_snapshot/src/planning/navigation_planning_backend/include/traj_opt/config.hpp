/*
 * Product-owned navigation implementation.
 * Algorithmic provenance and external attributions are documented in the
 * package documentation; they are not part of the runtime API or behaviour.
 */

#pragma once

#include <cmath>
#include <array>
#include <stdexcept>
#include <string>
#include <utility>
#include <utils/geometry/quadrotor_flatness.hpp>
#include <utils/header/yaml_loader.hpp>
#define NAVIGATION_PLANNER_DEBUG_FILE_DIR(name) (string(string(ROOT_DIR) + "log/"+(name)))

namespace traj_opt {
    using std::string;
    using std::vector;

    enum PosConstrainType {
        WAYPOINT = 1,
        CORRIDOR = 2,
    };

    class Config {
    public:
        static constexpr int kMaximumIntegralResolution = 1000;
        static constexpr int kMaximumFeasibilityRetryIterations = 256;

        bool uniform_time_en{false};

        flatness::FlatnessMap quadrotor_flatness;

        bool print_optimizer_log{false};

        /// Param for flatness
        double mass{0.0};
        double dh{0.0};
        double dv{0.0};
        double grav{0.0};
        double cp{0.0};
        double v_eps{0.0};

        // if save the optimization problem to log
        bool save_log_en{false};

        int pos_constraint_type{CORRIDOR};
        // Set to true for only min time.
        bool block_energy_cost{false};
        // Limit conditions.
        double max_vel{0}, max_acc{0}, max_jerk{0}, max_omg{0}, max_acc_thr{0}, min_acc_thr{0};
        // Objective terms. These weights shape the numerical search only; hard
        // geometric and dynamic certificates remain independent.
        double velocity_penalty_weight{0};
        double acceleration_penalty_weight{0};
        double jerk_penalty_weight{0};
        double angular_rate_penalty_weight{0};
        double thrust_penalty_weight{0};
        double time_weight{0};
        double position_penalty_weight{0};
        double waypoint_attraction_weight{0};
        double terminal_time_weight{0};
        // for backup traj piece num
        int piece_num{0};

        double smooth_eps{0};
        // Numerical smoothing and geometric certificates have independent
        // ownership. Changing one must never silently relax the other.
        double corridor_plane_tolerance_m{0};
        // Route-reference quality terms are independent from corridor
        // feasibility penalties. They shape the nominal path only; corridor
        // and dynamic hard gates remain authoritative.
        double route_reference_lateral_weight{0};
        double route_reference_vertical_weight{0};
        double route_reference_lateral_deadband_m{0};
        double route_reference_vertical_deadband_m{0};
        // Keep the nominal optimizer's soft dynamic target inside the
        // physical mission envelope. This is a search reserve only; the
        // independent V/A/J certificate still uses max_vel/max_acc/max_jerk.
        double optimization_dynamic_reserve_ratio{1.0};
        int integral_reso{0};
        double opt_accuracy{0};
        int feasibility_retry_max_iterations{64};
        // L-BFGS history length is a bounded numerical-performance knob. It
        // changes neither the physical limits nor any candidate certificate.
        int lbfgs_memory_size{32};

        Config() = default;

        // Validate the complete optimizer contract at every direct API
        // boundary. The YAML loader is not the only owner of Config: tests,
        // embedders and planner factories can construct or mutate it directly.
        // `require_piece_count` is true only for the fixed-size backup
        // optimizer; nominal pieces are derived from the corridor.
        void validate(const bool require_piece_count = false) const {
            const auto finiteNonNegative = [](const double value) {
                return std::isfinite(value) && value >= 0.0;
            };
            if ((pos_constraint_type != WAYPOINT && pos_constraint_type != CORRIDOR) ||
                !std::isfinite(mass) || mass <= 0.0 || !std::isfinite(grav) || grav <= 0.0 ||
                !finiteNonNegative(dh) || !finiteNonNegative(dv) ||
                !finiteNonNegative(cp) || !std::isfinite(v_eps) || v_eps <= 0.0 ||
                !std::isfinite(max_vel) || max_vel <= 0.0 ||
                !std::isfinite(max_acc) || max_acc <= 0.0 ||
                !std::isfinite(max_jerk) || max_jerk <= 0.0 ||
                !std::isfinite(max_omg) || max_omg <= 0.0 ||
                !std::isfinite(min_acc_thr) || min_acc_thr < 0.0 ||
                !std::isfinite(max_acc_thr) || max_acc_thr <= 0.0 ||
                min_acc_thr > max_acc_thr ||
                !std::isfinite(min_acc_thr * mass) ||
                !std::isfinite(max_acc_thr * mass) || integral_reso <= 0 ||
                !std::isfinite(smooth_eps) || smooth_eps <= 0.0 ||
                !std::isfinite(opt_accuracy) || opt_accuracy <= 0.0 ||
                piece_num < 0 || (require_piece_count && piece_num <= 0) ||
                !std::isfinite(corridor_plane_tolerance_m) ||
                corridor_plane_tolerance_m < 0.0 ||
                !finiteNonNegative(velocity_penalty_weight) ||
                !finiteNonNegative(acceleration_penalty_weight) ||
                !finiteNonNegative(jerk_penalty_weight) ||
                !finiteNonNegative(angular_rate_penalty_weight) ||
                !finiteNonNegative(thrust_penalty_weight) ||
                !finiteNonNegative(time_weight) ||
                !finiteNonNegative(position_penalty_weight) ||
                !finiteNonNegative(waypoint_attraction_weight) ||
                !finiteNonNegative(terminal_time_weight) ||
                !finiteNonNegative(route_reference_lateral_weight) ||
                !finiteNonNegative(route_reference_vertical_weight) ||
                !finiteNonNegative(route_reference_lateral_deadband_m) ||
                !finiteNonNegative(route_reference_vertical_deadband_m) ||
                !std::isfinite(optimization_dynamic_reserve_ratio) ||
                optimization_dynamic_reserve_ratio <= 0.0 ||
                optimization_dynamic_reserve_ratio > 1.0 ||
                feasibility_retry_max_iterations <= 0 ||
                feasibility_retry_max_iterations > kMaximumFeasibilityRetryIterations ||
                integral_reso > kMaximumIntegralResolution ||
                lbfgs_memory_size < 3 || lbfgs_memory_size > 256) {
                throw std::invalid_argument("invalid trajectory optimizer configuration");
            }
        }

        Config(const std::string & cfg_path, string ns)
            : Config(yaml_loader::YamlLoader(cfg_path), std::move(ns)) {}

        Config(const yaml_loader::YamlLoader& loader, string ns) {
            const YAML::Node traj_opt_node = loader.document()["traj_opt"];
            if (traj_opt_node.IsDefined() && traj_opt_node.IsMap()) {
                const YAML::Node boundary_node = traj_opt_node["boundary"];
                if (boundary_node.IsDefined() && boundary_node.IsMap() &&
                    boundary_node["dynamic_limit_tolerance_ratio"].IsDefined()) {
                    throw std::invalid_argument(
                        "traj_opt/boundary/dynamic_limit_tolerance_ratio was removed; "
                        "physical V/A/J certificates use fixed floating-point ULP accounting only");
                }
            }
            if (ns.empty()) {
                ns = "/";
            }
            else {
                ns = "/" + ns + "/";
            }

            loader.LoadParam("traj_opt/switch/print_optimizer_log", print_optimizer_log, false);
            /// Load Param for Flatness
            loader.LoadParam("traj_opt/flatness/mass", mass, 1.0);
            loader.LoadParam("traj_opt/flatness/dh", dh, 0.7);
            loader.LoadParam("traj_opt/flatness/dv", dv, 0.8);
            loader.LoadParam("traj_opt/flatness/grav", grav, 1.0);
            loader.LoadParam("traj_opt/flatness/cp", cp, 0.01);
            loader.LoadParam("traj_opt/flatness/v_eps", v_eps, 0.0001);

            loader.LoadParam("traj_opt/switch/save_log_en", save_log_en, false);
            loader.LoadParam("traj_opt" + ns + "pos_constraint_type", pos_constraint_type, 2);
            // Only the emergency trajectory owns a fixed piece count and
            // uniform-time parameterization. The nominal optimizer derives
            // its pieces from the generated corridor and guide path; loading
            // these keys for that profile creates configuration that has no
            // consumer and can mislead tuning.
            const bool nominal_profile = ns == "/exp_traj/";
            const bool backup_profile = ns == "/backup_traj/";
            if (backup_profile) {
                loader.LoadParam("traj_opt" + ns + "piece_num", piece_num, 1);
                loader.LoadParam("traj_opt" + ns + "uniform_time_en", uniform_time_en, false);
            }
            loader.LoadParam("traj_opt" + ns + "block_energy_cost", block_energy_cost, false);
            loader.LoadParam("traj_opt" + ns + "opt_accuracy", opt_accuracy, 1.0e-5);
            loader.LoadParam("traj_opt" + ns + "integral_reso", integral_reso, 10);
            loader.LoadParam("traj_opt" + ns + "lbfgs_memory_size", lbfgs_memory_size, 32);
            loader.LoadParam("traj_opt" + ns + "smooth_eps", smooth_eps, 0.01);
            loader.LoadParam("traj_opt" + ns + "corridor_plane_tolerance_m",
                             corridor_plane_tolerance_m, 0.01);
            if (nominal_profile) {
                loader.LoadParam("traj_opt" + ns + "feasibility_retry_max_iterations",
                                 feasibility_retry_max_iterations, 64);
                loader.LoadParam("traj_opt" + ns + "route_reference/lateral_weight",
                                 route_reference_lateral_weight, 0.0);
                loader.LoadParam("traj_opt" + ns + "route_reference/vertical_weight",
                                 route_reference_vertical_weight, 0.0);
                loader.LoadParam("traj_opt" + ns + "route_reference/lateral_deadband_m",
                                 route_reference_lateral_deadband_m, 0.0);
                loader.LoadParam("traj_opt" + ns + "route_reference/vertical_deadband_m",
                                 route_reference_vertical_deadband_m, 0.0);
                loader.LoadParam("traj_opt" + ns + "optimization_dynamic_reserve_ratio",
                                 optimization_dynamic_reserve_ratio, 1.0);
            }
            // Missing physical limits remain invalid and are rejected by the
            // planner contract; zero is the neutral loader default rather
            // than a negative sentinel with an overloaded meaning.
            loader.LoadParam("traj_opt/boundary/max_vel", max_vel, 0.0);
            loader.LoadParam("traj_opt/boundary/max_acc", max_acc, 0.0);
            loader.LoadParam("traj_opt/boundary/max_jerk", max_jerk, 0.0);
            loader.LoadParam("traj_opt/boundary/max_omg", max_omg, 0.0);
            loader.LoadParam("traj_opt/boundary/max_acc_thr", max_acc_thr, 0.0);
            loader.LoadParam("traj_opt/boundary/min_acc_thr", min_acc_thr, 0.0);

            loader.LoadParam("traj_opt" + ns + "objective/time_weight",
                             time_weight, 0.0);
            if (backup_profile) {
                loader.LoadParam("traj_opt" + ns + "objective/terminal_time_weight",
                                 terminal_time_weight, 0.0);
            }
            loader.LoadParam("traj_opt" + ns + "objective/position_penalty_weight",
                             position_penalty_weight, 0.0);
            loader.LoadParam("traj_opt" + ns + "objective/velocity_penalty_weight",
                             velocity_penalty_weight, 0.0);
            loader.LoadParam("traj_opt" + ns + "objective/acceleration_penalty_weight",
                             acceleration_penalty_weight, 0.0);
            loader.LoadParam("traj_opt" + ns + "objective/jerk_penalty_weight",
                             jerk_penalty_weight, 0.0);
            if (nominal_profile) {
                loader.LoadParam("traj_opt" + ns + "objective/waypoint_attraction_weight",
                                 waypoint_attraction_weight, 0.0);
            }
            loader.LoadParam("traj_opt" + ns + "objective/angular_rate_penalty_weight",
                             angular_rate_penalty_weight, 0.0);
            loader.LoadParam("traj_opt" + ns + "objective/thrust_penalty_weight",
                             thrust_penalty_weight, 0.0);

            const std::array<double, 9> objective_weights{
                time_weight, terminal_time_weight, position_penalty_weight,
                velocity_penalty_weight, acceleration_penalty_weight,
                jerk_penalty_weight, waypoint_attraction_weight,
                angular_rate_penalty_weight, thrust_penalty_weight};
            for (const double weight : objective_weights) {
                if (!std::isfinite(weight) || weight < 0.0) {
                    throw std::invalid_argument(
                        "trajectory objective weights must be finite and non-negative; "
                        "use zero to disable an objective term");
                }
            }
            if (!std::isfinite(smooth_eps) || smooth_eps <= 0.0 ||
                !std::isfinite(corridor_plane_tolerance_m) ||
                corridor_plane_tolerance_m < 0.0 ||
                !std::isfinite(route_reference_lateral_weight) ||
                route_reference_lateral_weight < 0.0 ||
                !std::isfinite(route_reference_vertical_weight) ||
                route_reference_vertical_weight < 0.0 ||
                !std::isfinite(route_reference_lateral_deadband_m) ||
                route_reference_lateral_deadband_m < 0.0 ||
                !std::isfinite(route_reference_vertical_deadband_m) ||
                route_reference_vertical_deadband_m < 0.0 ||
                !std::isfinite(optimization_dynamic_reserve_ratio) ||
                optimization_dynamic_reserve_ratio <= 0.0 ||
                optimization_dynamic_reserve_ratio > 1.0 ||
                !std::isfinite(opt_accuracy) ||
                opt_accuracy <= 0.0 ||
                integral_reso <= 0 ||
                integral_reso > kMaximumIntegralResolution ||
                feasibility_retry_max_iterations <= 0 ||
                feasibility_retry_max_iterations > kMaximumFeasibilityRetryIterations ||
                lbfgs_memory_size < 3 || lbfgs_memory_size > 256) {
                throw std::invalid_argument(
                    "trajectory smoothing and geometric tolerances must be finite; "
                    "smoothing must be positive, certificate tolerances non-negative, "
                    "retry iterations positive, and L-BFGS memory must be within [3, 256]");
            }

            quadrotor_flatness.reset(mass, grav, dh, dv, cp, v_eps);
            validate(backup_profile);
        }
    };
}
