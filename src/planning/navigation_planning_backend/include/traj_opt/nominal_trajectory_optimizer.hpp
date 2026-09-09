/*
 * Product-owned navigation implementation.
 * Algorithmic provenance and external attributions are documented in the
 * package documentation; they are not part of the runtime API or behaviour.
 */

#ifndef NAVIGATION_EXP_TRAJ_OPT_H
#define NAVIGATION_EXP_TRAJ_OPT_H

#include <iostream>
#include <atomic>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <limits>
#include <optional>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include <traj_opt/config.hpp>
#include <traj_opt/minco.h>


#include <data_structure/base/polytope.h>
#include <data_structure/base/trajectory.h>

#include <utils/header/scope_timer.hpp>
#include <utils/header/type_utils.hpp>
#include <utils/optimization/optimization_utils.h>
#include <utils/geometry/geometry_utils.h>

#include <planner_runtime_context/planner_runtime_context.hpp>
#include <navigation_world_model/world_model_view.hpp>

namespace traj_opt {

    using navigation_math::MatD3f;
    using navigation_math::Mat3Df;
    using navigation_math::VecDi;
    using navigation_math::VecDf;
    using navigation_math::PolyhedraH;
    using navigation_math::PolyhedraV;

    // Diagnostic-only EXP optimizer evidence.  These fields describe the
    // bounded L-BFGS/retry path for one optimize() invocation; they do not
    // participate in candidate selection, hard gates, or cancellation.
    struct ExpOptimizationDiagnostics {
        int lbfgs_attempt_count{0};
        int lbfgs_evaluation_count{0};
        int lbfgs_first_attempt_evaluation_count{0};
        int lbfgs_last_attempt_evaluation_count{0};
        int retry_count{0};
        int retry_violation_mask{0};  // bit 0=velocity, bit 1=acceleration, bit 2=jerk
        int retry_stop_reason{0};     // 0=none, 1=non-finite scale, 2=cancelled,
                                      // 3=LBFGS failure, 4=corridor rejection,
                                      // 5=no progress, 6=retry limit
        int first_lbfgs_return_code{-1};
        int last_lbfgs_return_code{-1};
        bool cancelled{false};
        bool hard_deadline_observed{false};
        bool valid{false};
        bool used_certified_seed{false};
        bool baseline_fallback_to_optimizer{false};
        int certified_seed_failure_stage{0};
        int corridor_seed_build_failure_stage{0};
        int corridor_seed_retry_attempt_count{0};
        int corridor_seed_retry_build_valid_count{0};
        int corridor_seed_retry_last_certificate_stage{0};
        int corridor_seed_selected_mode{0};  // 0=none, 1=initial, 2=piece, 3=uniform/certified stretch
        double corridor_seed_selected_max_duration_scale{
            std::numeric_limits<double>::quiet_NaN()};
        double initial_normalized_dynamic_violation{
            std::numeric_limits<double>::quiet_NaN()};
        double best_normalized_dynamic_violation{
            std::numeric_limits<double>::quiet_NaN()};
        double final_normalized_dynamic_violation{
            std::numeric_limits<double>::quiet_NaN()};
        double last_candidate_maximum_velocity_mps{
            std::numeric_limits<double>::quiet_NaN()};
        double last_candidate_maximum_acceleration_mps2{
            std::numeric_limits<double>::quiet_NaN()};
        double last_candidate_maximum_jerk_mps3{
            std::numeric_limits<double>::quiet_NaN()};
        double certified_seed_maximum_velocity_mps{
            std::numeric_limits<double>::quiet_NaN()};
        double certified_seed_maximum_acceleration_mps2{
            std::numeric_limits<double>::quiet_NaN()};
        double certified_seed_maximum_jerk_mps3{
            std::numeric_limits<double>::quiet_NaN()};
        double initial_duration_s{std::numeric_limits<double>::quiet_NaN()};
        double initial_minimum_piece_duration_s{
            std::numeric_limits<double>::quiet_NaN()};
        double initial_maximum_piece_duration_s{
            std::numeric_limits<double>::quiet_NaN()};
        double final_duration_s{std::numeric_limits<double>::quiet_NaN()};
        double retry_duration_lower_bound_min_s{
                std::numeric_limits<double>::quiet_NaN()};
        double retry_duration_lower_bound_max_s{
                std::numeric_limits<double>::quiet_NaN()};
        double retry_free_duration_seed_min_s{
                std::numeric_limits<double>::quiet_NaN()};
        double retry_free_duration_seed_max_s{
                std::numeric_limits<double>::quiet_NaN()};
        std::int64_t retry_budget_remaining_us{-1};
        // Remaining steady-clock budget when MINCO enters. This distinguishes
        // an absolute solve deadline from an optional-refinement cutoff in
        // runtime evidence; it never participates in candidate admission.
        std::int64_t refinement_budget_at_entry_us{-1};
        int nonfinite_evaluation_count{0};
        int first_nonfinite_stage{0}; // 1=input, 2=duration, 3=points, 4=MINCO, 5=objective, 6=gradient
        int first_nonfinite_value_mask{0};
        int first_nonfinite_attempt{0};
        int first_nonfinite_iteration{0};
        double first_nonfinite_min_duration_s{
            std::numeric_limits<double>::quiet_NaN()};
        double first_nonfinite_max_duration_s{
            std::numeric_limits<double>::quiet_NaN()};
        double first_nonfinite_cost{std::numeric_limits<double>::quiet_NaN()};
        double first_nonfinite_gradient_norm{
                std::numeric_limits<double>::quiet_NaN()};
        // Setup diagnostics identify a pre-MINCO rejection without changing
        // the production admission decision.  In particular, an overlap
        // vertex-enumeration failure must not be reported merely as
        // "main_minco".
        int setup_failure_stage{0};
        int setup_failure_index{-1};
        int setup_failure_vertex_count{-1};
        double setup_failure_metric{
                std::numeric_limits<double>::quiet_NaN()};
    };

    // Provenance supplied by Planner for an opt-in nominal-problem capture.
    // These fields are observational only; they never participate in solve
    // admission, candidate selection, or any hard certificate.
    struct NominalProblemProvenance {
        std::string source_identity{"uav-navigation::ExpTrajOpt"};
        std::string source_revision{"not-provided"};
        std::string source_diff_sha256{"not-provided"};
        std::string workspace{"not-provided"};
        std::string build_manifest_path{"not-provided"};
        navigation_world_model::WorldSnapshotIdentity world_identity{};
        navigation_world_model::WorldSnapshotIdentity execution_world_identity{};
        std::uint64_t execution_bundle_generation{0U};
        std::uint64_t execution_localization_epoch{0U};
        std::uint64_t execution_goal_epoch{0U};
        std::uint64_t execution_request_id{0U};
        std::uint64_t request_localization_epoch{0U};
        std::uint64_t request_goal_epoch{0U};
        std::uint64_t request_id{0U};
        // Runtime-owned identifiers for correlating this immutable snapshot
        // with the planner timeline. They are diagnostic provenance only and
        // never participate in request admission or candidate validity.
        std::uint64_t solve_generation{0U};
        std::uint64_t planner_cycle{0U};
        std::uint64_t route_revision{0U};
        std::uint32_t waypoint_index{0U};
        std::string mission_id;
        std::string start_mode{"unknown"};
        std::string planner_mode{"nominal"};
        std::string recovery_state{"not-provided"};
        std::int64_t request_stamp_ns{0};
        std::int64_t anchor_stamp_ns{0};
        std::int64_t activation_stamp_ns{0};
        double solve_start_wall_time_s{std::numeric_limits<double>::quiet_NaN()};
        std::optional<navigation_world_model::WorldModelDiagnosticSnapshot>
                diagnostic_world_snapshot;
    };

    struct NominalOptimizerConfigSnapshot {
        bool uniform_time_en{false};
        bool print_optimizer_log{false};
        bool save_log_en{false};
        bool block_energy_cost{false};
        int pos_constraint_type{0};
        int piece_num{0};
        int integral_reso{0};
        int feasibility_retry_max_iterations{0};
        int lbfgs_memory_size{0};
        double mass{0.0};
        double dh{0.0};
        double dv{0.0};
        double grav{0.0};
        double cp{0.0};
        double v_eps{0.0};
        double max_vel{0.0};
        double max_acc{0.0};
        double max_jerk{0.0};
        double max_omg{0.0};
        double max_acc_thr{0.0};
        double min_acc_thr{0.0};
        double velocity_penalty_weight{0.0};
        double acceleration_penalty_weight{0.0};
        double jerk_penalty_weight{0.0};
        double angular_rate_penalty_weight{0.0};
        double thrust_penalty_weight{0.0};
        double time_weight{0.0};
        double position_penalty_weight{0.0};
        double waypoint_attraction_weight{0.0};
        double terminal_time_weight{0.0};
        double smooth_eps{0.0};
        double corridor_plane_tolerance_m{0.0};
        double route_reference_lateral_weight{0.0};
        double route_reference_vertical_weight{0.0};
        double route_reference_lateral_deadband_m{0.0};
        double route_reference_vertical_deadband_m{0.0};
        double optimization_dynamic_reserve_ratio{0.0};
        double opt_accuracy{0.0};
    };

    struct NominalDurationRetrySnapshot {
        int retry_type{0};
        VecDf duration_s;
        VecDf duration_scale;
        bool build_valid{false};
        int failure_stage{0};
        int certificate_failure_stage{0};
        int failing_piece_index{-1};
        int failing_control_index{-1};
        int failing_plane_index{-1};
        double maximum_plane_violation_m{
            std::numeric_limits<double>::quiet_NaN()};
        double minimum_internal_derivative_scale{
            std::numeric_limits<double>::quiet_NaN()};
        Eigen::Vector3d failing_control_point{
            Eigen::Vector3d::Constant(std::numeric_limits<double>::quiet_NaN())};
        Eigen::Vector4d failing_plane{
            Eigen::Vector4d::Constant(std::numeric_limits<double>::quiet_NaN())};
    };

    // Complete immutable nominal input plus the deterministic duration retry
    // evidence. This object is created only when
    // UAV_NAVIGATION_NOMINAL_SNAPSHOT_DIR is set, and is serialized only
    // after the optimizer has returned. It is deliberately not part of the
    // product planning or ROS message API.
    struct NominalProblemSnapshot {
        std::uint32_t schema_version{1U};
        std::string snapshot_kind{"nominal"};
        bool recovery_request{false};
        bool target_failure_signature{false};
        NominalProblemProvenance provenance{};
        NominalOptimizerConfigSnapshot config{};
        navigation_math::StatePVAJ head_pvaj{navigation_math::StatePVAJ::Zero()};
        navigation_math::StatePVAJ tail_pvaj{navigation_math::StatePVAJ::Zero()};
        vec_E<Vec3f> guide_path;
        vector<double> guide_stamp;
        Mat3Df junction_positions;
        Mat3Df initial_spatial_variables;
        Mat3Df initial_route_reference_points;
        Mat3Df initial_waypoint_attractor;
        Mat3Df initial_path;
        VecDf initial_durations_s;
        VecDf initial_duration_lower_bound;
        VecDf initial_waypoint_attractor_dead_d;
        vec_Vec3f initial_points;
        VecDf initial_times;
        VecDi piece_idx;
        VecDi v_poly_idx;
        VecDi h_poly_idx;
        PolyhedraV v_polytopes;
        // Keep the optimizer-boundary corridor separate from the
        // post-SimplifySFC view stored in h_polytopes. This is diagnostic-only
        // and lets offline replay attribute a degenerate overlap to corridor
        // generation versus simplification.
        PolyhedraH pre_simplify_h_polytopes;
        // Preserve PRE route-boundary semantics separately. The legacy
        // route_boundary_* fields are replaced with the POST view after
        // SimplifySFC/setup completes.
        std::vector<unsigned char> pre_simplify_route_boundary_gates;
        std::vector<Vec3f> pre_simplify_route_boundary_points;
        vector<double> pre_simplify_route_boundary_radii;
        PolyhedraH h_polytopes;
        PolyhedraH h_overlap_polytopes;
        std::vector<unsigned char> route_boundary_gates;
        std::vector<Vec3f> route_boundary_points;
        vector<double> route_boundary_radii;
        std::optional<navigation_world_model::WorldModelDiagnosticSnapshot>
                diagnostic_world_snapshot;
        std::int64_t refinement_deadline_ns{0};
        std::int64_t hard_deadline_ns{0};
        bool hard_deadline_observed{false};
        bool hard_deadline_exceeded_at_signature{false};
        bool baseline_only{false};
        bool suppress_optional_refinement{false};
        bool setup_completed{false};
        // True only after the current request has populated opt_vars from its
        // own post-SimplifySFC chain. This prevents a rejected successor from
        // inheriting POST geometry or route metadata from the previous solve.
        bool post_setup_input_bound{false};
        int setup_failure_stage{0};
        int setup_failure_index{-1};
        int setup_failure_vertex_count{-1};
        double setup_failure_metric{
                std::numeric_limits<double>::quiet_NaN()};
        VecDf effective_magnitude_bounds;
        VecDf initial_penalty_weights;
        bool initial_corridor_seed_build_valid{false};
        int initial_corridor_seed_failure_stage{0};
        int initial_corridor_seed_failing_piece_index{-1};
        int initial_corridor_seed_failing_control_index{-1};
        int initial_corridor_seed_failing_plane_index{-1};
        double initial_corridor_seed_maximum_plane_violation_m{
            std::numeric_limits<double>::quiet_NaN()};
        double initial_corridor_seed_minimum_internal_derivative_scale{
            std::numeric_limits<double>::quiet_NaN()};
        Eigen::Vector3d initial_corridor_seed_failing_control_point{
            Eigen::Vector3d::Constant(std::numeric_limits<double>::quiet_NaN())};
        Eigen::Vector4d initial_corridor_seed_failing_plane{
            Eigen::Vector4d::Constant(std::numeric_limits<double>::quiet_NaN())};
        bool initial_deterministic_certificate_evaluated{false};
        bool initial_deterministic_certificate_valid{false};
        int initial_deterministic_failure_stage{0};
        double initial_deterministic_maximum_velocity_mps{
            std::numeric_limits<double>::quiet_NaN()};
        double initial_deterministic_maximum_acceleration_mps2{
            std::numeric_limits<double>::quiet_NaN()};
        double initial_deterministic_maximum_jerk_mps3{
            std::numeric_limits<double>::quiet_NaN()};
        std::vector<NominalDurationRetrySnapshot> duration_retries;
    };

    // Recovery uses the ordinary nominal optimizer entry point.  The alias is
    // intentional: it gives recovery evidence an explicit name without
    // creating a second planner or optimizer data model.
    using RecoveryProblemSnapshot = NominalProblemSnapshot;

    [[nodiscard]] std::string writeNominalProblemSnapshotJson(
        const NominalProblemSnapshot& snapshot,
        const std::string& directory);

    enum class NominalSolveStatus : std::uint8_t {
        kRefined,
        kCertifiedSeed,
        kFailed,
    };

    struct NominalSolveResult {
        NominalSolveStatus status{NominalSolveStatus::kFailed};
        bool cancellation_observed{false};
        bool deadline_observed{false};

        [[nodiscard]] bool candidateAvailable() const noexcept {
            return status == NominalSolveStatus::kRefined ||
                   status == NominalSolveStatus::kCertifiedSeed;
        }
    };

    [[nodiscard]] inline NominalSolveResult classifyNominalSolveResult(
            bool optimizer_success,
            const ExpOptimizationDiagnostics& diagnostics,
            bool deadline_observed) noexcept {
        NominalSolveResult result;
        result.cancellation_observed = diagnostics.cancelled;
        result.deadline_observed = deadline_observed;
        if (!optimizer_success) return result;
        result.status = diagnostics.used_certified_seed
            ? NominalSolveStatus::kCertifiedSeed
            : NominalSolveStatus::kRefined;
        return result;
    }

    // A zero nominal objective weight means "do not shape this quantity"
    // during ordinary optimization; it cannot mean "provide no gradient"
    // after the independent hard gate has already found that exact quantity
    // infeasible. Reuse the strongest configured translational dynamic weight
    // only for the bounded feasibility retry. If all dynamic penalties are
    // disabled, return NaN so the caller keeps failing closed.
    inline double feasibilityRetryPenaltyWeight(
            const VecDf& nominal_penalty_weights,
            const int dynamic_penalty_index) noexcept {
        if (nominal_penalty_weights.size() < 4 ||
            dynamic_penalty_index < 1 || dynamic_penalty_index > 3) {
            return std::numeric_limits<double>::quiet_NaN();
        }
        const double requested = nominal_penalty_weights(dynamic_penalty_index);
        if (std::isfinite(requested) && requested > 0.0) return requested;
        double fallback = 0.0;
        for (int index = 1; index <= 3; ++index) {
            const double candidate = nominal_penalty_weights(index);
            if (std::isfinite(candidate) && candidate > fallback) {
                fallback = candidate;
            }
        }
        return fallback > 0.0
            ? fallback
            : std::numeric_limits<double>::quiet_NaN();
    }


    class ExpTrajOpt {
        ExpOptimizationDiagnostics diagnostics_{};
        std::optional<NominalProblemSnapshot> nominal_problem_snapshot_{};
        NominalProblemProvenance diagnostic_provenance_{};
        traj_opt::Config cfg_;
        std::ofstream failed_traj_log;
        std::ofstream penalty_log;
        navigation_planner_context::PlannerRuntimeContext::Ptr planner_context_;

        struct OptimizationVariables {
            double rho;
            int iter_num{0};
            int pos_constraint_type;
            bool block_energy_cost;
            double smooth_eps;
            int integral_res;
            flatness::FlatnessMap quadrotor_flatness;

            Mat3Df gradByPoints;
            VecDf gradByTimes;
            MatD3f partialGradByCoeffs;
            VecDf partialGradByTimes;
            bool default_init{true};
            bool given_init_ts_and_ps{false};
            int piece_num;
            Mat3Df points;
            VecDf times;
            VecDf duration_lower_bound;
            Mat3Df route_reference_points;
            double route_reference_lateral_weight{0.0};
            double route_reference_vertical_weight{0.0};
            double route_reference_lateral_deadband_m{0.0};
            double route_reference_vertical_deadband_m{0.0};
            // One entry per corridor cell.  A marked cell is a hard mission
            // boundary inserted for a pass-through waypoint. Its adjacent
            // junction must enter the recorded acceptance ball; the cell
            // itself remains a collision-certified free-space region so a
            // smooth turn is not forced into a sub-metre box.
            std::vector<unsigned char> route_boundary_gates;
            std::vector<Vec3f> route_boundary_points;
            std::vector<double> route_boundary_radii;
            VecDf magnitudeBounds, penaltyWeights;

            PolyhedraV vPolytopes; // the original sfc and intersecting sfc
            PolyhedraH hPolytopes; // the original sfc
            PolyhedraH hOverlapPolytopes;
            Mat3Df init_path;
            VecDf init_ts;
            vec_Vec3f init_ps;
            Mat3Df waypoint_attractor;
            VecDf waypoint_attractor_dead_d;

            VecDi pieceIdx;
            VecDi vPolyIdx;
            VecDi hPolyIdx;

            MINCO_S4NU minco;

            StatePVAJ headPVAJ;
            StatePVAJ tailPVAJ;
            vec_E<Vec3f> guide_path;
            vector<double> guide_t;

            int temporalDim, spatialDim;

            VecDf penalty_log;

            std::atomic_bool* solve_cancelled{nullptr};
            std::int64_t steady_deadline_ns{0};
            std::int64_t refinement_deadline_ns{0};
            std::int64_t hard_deadline_ns{0};
            int solver_attempt{0};
            int nonfinite_evaluation_count{0};
            int first_nonfinite_stage{0};
            int first_nonfinite_value_mask{0};
            int first_nonfinite_attempt{0};
            int first_nonfinite_iteration{0};
            double first_nonfinite_min_duration_s{
                std::numeric_limits<double>::quiet_NaN()};
            double first_nonfinite_max_duration_s{
                std::numeric_limits<double>::quiet_NaN()};
            double first_nonfinite_cost{std::numeric_limits<double>::quiet_NaN()};
            double first_nonfinite_gradient_norm{
                std::numeric_limits<double>::quiet_NaN()};
        } opt_vars{};

        static double costFunctional(void *ptr,
                                     const VecDf &x,
                                     VecDf &g);

        static void constraintsFunctional(const VecDf &T,
                                          const MatD3f &coeffs,
                                          const VecDi &hIdx,
                                          const PolyhedraH &hPolys,
                                          const Mat3Df &waypoint_attractor,
                                          const VecDf &waypoint_attractor_dead_d,
                                          const Mat3Df &route_reference_points,
                                          const std::vector<Vec3f> &route_boundary_points,
                                          const std::vector<double> &route_boundary_radii,
                                          const Vec3f &route_reference_head,
                                          const Vec3f &route_reference_tail,
                                          const double route_reference_lateral_weight,
                                          const double route_reference_vertical_weight,
                                          const double route_reference_lateral_deadband_m,
                                          const double route_reference_vertical_deadband_m,
                                          const double &smoothFactor,
                                          const int &integralResolution,
                                          const VecDf &magnitudeBounds,
                                          const VecDf &penaltyWeights,
                                          flatness::FlatnessMap &flatMap,
                                          double &cost,
                                          VecDf &gradT,
                                          MatD3f &gradC,
                                          VecDf &penalty_log);

        bool processCorridorWithGuideTraj();

        bool setupProblemAndCheck();

        bool setInitPsAndTs(const vec_Vec3f &init_ps, const vector<double> &init_ts);

        double optimize(Trajectory &traj, const double &relCostTol,
                        bool suppress_optional_refinement);

        static int monitorProgress(void *instance,
                                   const VecDf &x,
                                   const VecDf &g,
                                   double fx,
                                   double step,
                                   int k,
                                   int ls);

        void resetTransientDiagnostics() noexcept {
            diagnostics_ = ExpOptimizationDiagnostics{};
        }

        void beginNominalProblemSnapshot(const StatePVAJ& headPVAJ,
                                         const StatePVAJ& tailPVAJ,
                                         const vec_E<Vec3f>& guide_path,
                                         const vector<double>& guide_t,
                                         const PolytopeVec& sfcs,
                                         bool baseline_only,
                                         bool suppress_optional_refinement);

        void recordNominalProblemSetupResult(bool setup_completed);

    public:
        typedef std::shared_ptr<ExpTrajOpt> Ptr;

        ExpTrajOpt(const traj_opt::Config &cfg, const navigation_planner_context::PlannerRuntimeContext::Ptr & planner_context);

        ~ExpTrajOpt();

#ifdef UAV_NAVIGATION_DIAGNOSTIC_REPLAY
        // Diagnostic-only seam for replaying a frozen corridor through the
        // production pre-MINCO setup path.  This deliberately bypasses
        // SimplifySFC so a replay can compare the captured PRE and POST
        // chains under the exact same downstream representation contract.
        // It is compiled only into the offline replay executable and is not
        // part of the normal runtime library API.
        bool diagnosticSetupFromFrozenCorridor(
                const StatePVAJ& headPVAJ,
                const StatePVAJ& tailPVAJ,
                const vec_E<Vec3f>& guide_path,
                const vector<double>& guide_t,
                const PolytopeVec& sfcs) {
            resetDiagnostics();
            baseline_only_ = false;
            if (guide_path.empty() || guide_path.size() != guide_t.size() ||
                sfcs.empty()) {
                return false;
            }
            opt_vars.default_init = false;
            opt_vars.given_init_ts_and_ps = false;
            opt_vars.headPVAJ = headPVAJ;
            opt_vars.tailPVAJ = tailPVAJ;
            opt_vars.guide_path = guide_path;
            opt_vars.guide_t = guide_t;
            opt_vars.route_boundary_gates.assign(sfcs.size(), 0U);
            opt_vars.route_boundary_points.assign(
                sfcs.size(), Vec3f::Constant(
                    std::numeric_limits<float>::quiet_NaN()));
            opt_vars.route_boundary_radii.assign(
                sfcs.size(), std::numeric_limits<double>::quiet_NaN());
            opt_vars.hPolytopes.resize(sfcs.size());
            for (std::size_t index = 0; index < sfcs.size(); ++index) {
                opt_vars.hPolytopes[index] = sfcs[index].GetPlanes();
                if (sfcs[index].IsRouteBoundaryGate()) {
                    opt_vars.route_boundary_gates[index] = 1U;
                    opt_vars.route_boundary_points[index] =
                        sfcs[index].GetRouteBoundaryPoint();
                    opt_vars.route_boundary_radii[index] =
                        sfcs[index].GetRouteBoundaryRadius();
                }
            }
            return setupProblemAndCheck();
        }
#endif

        void setSolveBudget(std::atomic_bool* solve_cancelled,
                            std::int64_t refinement_deadline_ns,
                            std::int64_t hard_deadline_ns = 0) noexcept {
            opt_vars.solve_cancelled = solve_cancelled;
            opt_vars.refinement_deadline_ns = refinement_deadline_ns;
            opt_vars.hard_deadline_ns = hard_deadline_ns > 0
                    ? hard_deadline_ns : refinement_deadline_ns;
            opt_vars.steady_deadline_ns = refinement_deadline_ns;
        }

        // Planning-thread-only recovery envelope. This lowers the velocity
        // bound while leaving acceleration, jerk and all hard certificates
        // unchanged.
        void setMaximumVelocity(double maximum_velocity_mps) noexcept {
            if (std::isfinite(maximum_velocity_mps) && maximum_velocity_mps > 0.0) {
                cfg_.max_vel = maximum_velocity_mps;
            }
        }

        void resetDiagnostics() noexcept {
            diagnostics_ = ExpOptimizationDiagnostics{};
            nominal_problem_snapshot_.reset();
        }

        ExpOptimizationDiagnostics diagnostics() const noexcept { return diagnostics_; }

        void setNominalProblemProvenance(NominalProblemProvenance provenance) {
            diagnostic_provenance_ = std::move(provenance);
        }

        // Attach the optional, potentially large world materialization only
        // after solve() has returned.  This keeps diagnostic capture outside
        // the optimizer's deadline-critical section.
        void setNominalProblemDiagnosticWorldSnapshot(
                std::optional<navigation_world_model::WorldModelDiagnosticSnapshot>
                    snapshot) {
            if (nominal_problem_snapshot_) {
                nominal_problem_snapshot_->diagnostic_world_snapshot =
                    std::move(snapshot);
            }
        }

        [[nodiscard]] bool nominalProblemSetupFailed() const noexcept {
            return nominal_problem_snapshot_.has_value() &&
                   !nominal_problem_snapshot_->setup_completed;
        }

        [[nodiscard]] bool nominalProblemTargetFailureSignature() const noexcept {
            return nominal_problem_snapshot_.has_value() &&
                   nominal_problem_snapshot_->target_failure_signature;
        }

        [[nodiscard]] bool nominalProblemRecoveryRequest() const noexcept {
            return nominal_problem_snapshot_.has_value() &&
                   nominal_problem_snapshot_->recovery_request;
        }

        [[nodiscard]] std::optional<NominalProblemSnapshot>
        takeNominalProblemSnapshot() noexcept {
            if (nominal_problem_snapshot_.has_value()) {
                nominal_problem_snapshot_->hard_deadline_observed =
                    diagnostics_.hard_deadline_observed;
            }
            auto snapshot = std::move(nominal_problem_snapshot_);
            nominal_problem_snapshot_.reset();
            return snapshot;
        }

        bool optimize(const StatePVAJ &headPVAJ, const StatePVAJ &tailPVAJ,
                      PolytopeVec &sfcs,
                      Trajectory &out_traj);

        bool optimize(const StatePVAJ &headPVAJ, const StatePVAJ &tailPVAJ,
                      const vec_E<Vec3f> &guide_path, const vector<double> &guide_t,
                      PolytopeVec &sfcs,
                      Trajectory &out_traj,
                      bool baseline_only = false,
                      bool suppress_optional_refinement = false);

        NominalSolveResult solve(
                      const StatePVAJ &headPVAJ, const StatePVAJ &tailPVAJ,
                      const vec_E<Vec3f> &guide_path, const vector<double> &guide_t,
                      PolytopeVec &sfcs, Trajectory &out_traj,
                      bool deadline_observed = false,
                      bool baseline_only = false,
                      bool suppress_optional_refinement = false);

        void getInitValue(VecDf &ts, vec_Vec3f &ps) const {
            ts = opt_vars.init_ts;
            ps = opt_vars.init_ps;
        }

        bool optimize(const StatePVAJ &headPVAJ, const StatePVAJ &tailPVAJ,
                      PolytopeVec &sfcs,
                      const vec_Vec3f & init_ps,
                      const VecDf & init_ts,
                      Trajectory &out_traj);

        bool baseline_only_{false};

    };
}

#endif
