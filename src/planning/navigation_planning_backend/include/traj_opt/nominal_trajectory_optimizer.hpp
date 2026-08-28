/*
 * Product-owned navigation implementation.
 * Algorithmic provenance and external attributions are documented in the
 * package documentation; they are not part of the runtime API or behaviour.
 */

#ifndef NAVIGATION_EXP_TRAJ_OPT_H
#define NAVIGATION_EXP_TRAJ_OPT_H

#include <iostream>
#include <atomic>
#include <cstdint>
#include <limits>
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
        int retry_count{0};
        int retry_violation_mask{0};  // bit 0=velocity, bit 1=acceleration, bit 2=jerk
        int retry_stop_reason{0};     // 0=none, 1=non-finite scale, 2=cancelled,
                                      // 3=LBFGS failure, 4=corridor rejection,
                                      // 5=no progress, 6=retry limit
        int first_lbfgs_return_code{-1};
        int last_lbfgs_return_code{-1};
        bool cancelled{false};
        bool valid{false};
        double initial_normalized_dynamic_violation{
            std::numeric_limits<double>::quiet_NaN()};
        double best_normalized_dynamic_violation{
            std::numeric_limits<double>::quiet_NaN()};
        double final_normalized_dynamic_violation{
            std::numeric_limits<double>::quiet_NaN()};
        double initial_duration_s{std::numeric_limits<double>::quiet_NaN()};
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
    };


    class ExpTrajOpt {
        ExpOptimizationDiagnostics diagnostics_{};
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
            // boundary inserted for a pass-through waypoint; its outgoing
            // overlap must consume a post-boundary guide sample so the hot
            // seed does not create a near-zero-duration turn piece.
            std::vector<unsigned char> route_boundary_gates;
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
        } opt_vars;

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

        double optimize(Trajectory &traj, const double &relCostTol);

        static int monitorProgress(void *instance,
                                   const VecDf &x,
                                   const VecDf &g,
                                   double fx,
                                   double step,
                                   int k,
                                   int ls);

    public:
        typedef std::shared_ptr<ExpTrajOpt> Ptr;

        ExpTrajOpt(const traj_opt::Config &cfg, const navigation_planner_context::PlannerRuntimeContext::Ptr & planner_context);

        ~ExpTrajOpt();

        void setSolveBudget(std::atomic_bool* solve_cancelled,
                            std::int64_t steady_deadline_ns) noexcept {
            opt_vars.solve_cancelled = solve_cancelled;
            opt_vars.steady_deadline_ns = steady_deadline_ns;
        }

        void resetDiagnostics() noexcept { diagnostics_ = ExpOptimizationDiagnostics{}; }

        ExpOptimizationDiagnostics diagnostics() const noexcept { return diagnostics_; }

        bool optimize(const StatePVAJ &headPVAJ, const StatePVAJ &tailPVAJ,
                      PolytopeVec &sfcs,
                      Trajectory &out_traj);

        bool optimize(const StatePVAJ &headPVAJ, const StatePVAJ &tailPVAJ,
                      const vec_E<Vec3f> &guide_path, const vector<double> &guide_t,
                      PolytopeVec &sfcs,
                      Trajectory &out_traj);

        void getInitValue(VecDf &ts, vec_Vec3f &ps) const {
            ts = opt_vars.init_ts;
            ps = opt_vars.init_ps;
        }

        bool optimize(const StatePVAJ &headPVAJ, const StatePVAJ &tailPVAJ,
                      PolytopeVec &sfcs,
                      const vec_Vec3f & init_ps,
                      const VecDf & init_ts,
                      Trajectory &out_traj);

    };
}

#endif
