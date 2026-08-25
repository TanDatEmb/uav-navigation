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
        std::int64_t retry_budget_remaining_us{-1};
        int nonfinite_evaluation_count{0};
        int first_nonfinite_stage{0}; // 1=input, 2=duration, 3=points, 4=MINCO, 5=objective, 6=gradient
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
            VecDf minimum_time_floor;
            Mat3Df feasibility_reference_points;
            double feasibility_point_weight{0.0};
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
                                          const Mat3Df &feasibility_reference_points,
                                          const double feasibility_reference_head_z,
                                          const double feasibility_reference_tail_z,
                                          const double feasibility_point_weight,
                                          const double &smoothFactor,
                                          const int &integralResolution,
                                          const VecDf &magnitudeBounds,
                                          const VecDf &penaltyWeights,
                                          flatness::FlatnessMap &flatMap,
                                          double &cost,
                                          VecDf &gradT,
                                          MatD3f &gradC,
                                          VecDf &penalty_log);

        bool processCorridor();

        bool processCorridorWithGuideTraj();

        void defaultInitialization();

        bool setupProblemAndCheck();

        bool processCorridorWithGuideTraj2() {
            using namespace traj_opt;
            using namespace color_text;
            using namespace navigation_math;
            using namespace math_utils;
            using namespace optimization_utils;
            // * 1) allocate memory for vertex
            const int sizeCorridor = static_cast<int>(opt_vars.hPolytopes.size() - 1);

            opt_vars.vPolytopes.clear();
            opt_vars.vPolytopes.reserve(2 * sizeCorridor + 1);

            long nv;
            PolyhedronH curIH;
            PolyhedronV curIV, curIOB;
            opt_vars.waypoint_attractor.resize(3, sizeCorridor);
            opt_vars.hOverlapPolytopes.resize(sizeCorridor);
            opt_vars.waypoint_attractor_dead_d.resize(sizeCorridor);
            // * 2) Process the corridor
            for (int i = 0; i < sizeCorridor; i++) {
                // * 2.1) Get current vertex
                if (!geometry_utils::enumerateVs(opt_vars.hPolytopes[i], curIV)) {
                    cout << YELLOW << " -- [planner] in [ GcopterExpS4::processCorridor]: Failed to enumerate corridor Vs."
                         << RESET << endl;

                    return false;
                }
                // * 2.3) Conver the vertex to the frame of the first point
                nv = curIV.cols();
                curIOB.resize(3, nv);
                // *    Save the position of the first point
                curIOB.col(0) = curIV.col(0);
                // *    Use the relative position of the rest vertex.
                curIOB.rightCols(nv - 1) = curIV.rightCols(nv - 1).colwise() - curIV.col(0);
                // *    save the i-th corridor's vertex
                opt_vars.vPolytopes.push_back(curIOB);

                // * 2.4) Find the overlap corridor
                curIH.resize(opt_vars.hPolytopes[i].rows() + opt_vars.hPolytopes[i + 1].rows(), 4);
                curIH.topRows(opt_vars.hPolytopes[i].rows()) = opt_vars.hPolytopes[i];
                curIH.bottomRows(opt_vars.hPolytopes[i + 1].rows()) = opt_vars.hPolytopes[i + 1];
                opt_vars.hOverlapPolytopes[i] = curIH;
                Vec3f interior;

                const double dis = geometry_utils::findInteriorDist(curIH, interior) / 2;
                if (dis < 0.0 || std::isinf(dis)) {

                    cout << YELLOW << " -- [planner] in [ GcopterExpS4::processCorridor]: Failed findInteriorDist Vs." <<
                         RESET << endl;
                    return false;
                }
                geometry_utils::enumerateVs(curIH, interior, curIV);
                const double test_sum = curIV.sum();
                if (std::isnan(test_sum) || std::isinf(test_sum)) {
                    return false;
                }
                opt_vars.waypoint_attractor.col(i) = curIV.rowwise().mean();
                opt_vars.waypoint_attractor_dead_d(i) = dis;
                nv = curIV.cols();
                curIOB.resize(3, nv);
                curIOB.col(0) = curIV.col(0);
                curIOB.rightCols(nv - 1) = curIV.rightCols(nv - 1).colwise() - curIV.col(0);
                opt_vars.vPolytopes.push_back(curIOB);
            }

            // * 3) Time and waypoint allocation for hot initialization
            VecDf min_dis(opt_vars.waypoint_attractor.cols());
            VecDi min_id(opt_vars.waypoint_attractor.cols());
            VecDf time_stamps(opt_vars.waypoint_attractor.cols() + 2);
            time_stamps(0) = 0.0;
            time_stamps(opt_vars.waypoint_attractor.cols() + 1) = opt_vars.guide_t.back();
            min_id.setConstant(0);
            min_dis.setConstant(std::numeric_limits<double>::max());
            for (int i = 0; i < opt_vars.guide_path.size(); i++) {
                for (int j = 0; j < opt_vars.waypoint_attractor.cols(); j++) {
                    const double dis = (opt_vars.guide_path[i] - opt_vars.waypoint_attractor.col(j)).norm();
                    if (dis < min_dis[j]) {
                        min_dis[j] = dis;
                        min_id[j] = i;
                        opt_vars.points.col(j) = opt_vars.guide_path[i];
                        time_stamps(j + 1) = opt_vars.guide_t[i];
                    }
                }
            }

            for (int i = 1; i < time_stamps.size(); i++) {
                opt_vars.times(i - 1) = time_stamps(i) - time_stamps(i - 1);
                opt_vars.times(i - 1) = std::max(0.01, opt_vars.times(i - 1));
            }

            if (!geometry_utils::enumerateVs(opt_vars.hPolytopes.back(), curIV)) {
                return false;
            }
            nv = curIV.cols();
            curIOB.resize(3, nv);
            curIOB.col(0) = curIV.col(0);
            curIOB.rightCols(nv - 1) = curIV.rightCols(nv - 1).colwise() - curIV.col(0);
            opt_vars.vPolytopes.push_back(curIOB);
            return true;
        }

        bool setupProblemAndCheck2() {
            // init internal variables size;
            opt_vars.piece_num = static_cast<int>(opt_vars.hPolytopes.size());
            opt_vars.times.resize(opt_vars.piece_num);
            opt_vars.points.resize(3, opt_vars.piece_num - 1);


            // Check corridor and init points
            if (opt_vars.default_init) {
                if (!processCorridor()) {
                    return false;
                }
            } else {
                if (!processCorridorWithGuideTraj2()) {
                    return false;
                }
            }
            opt_vars.init_path.resize(3, opt_vars.piece_num + 1);
            for (long i = 0; i < opt_vars.piece_num - 1; i++) {
                opt_vars.init_path.col(i + 1) = opt_vars.waypoint_attractor.col(i);
            }
            opt_vars.init_path.col(0) = opt_vars.headPVAJ.col(0);
            opt_vars.init_path.rightCols(1) = opt_vars.tailPVAJ.col(0);
            if (opt_vars.default_init) {
                defaultInitialization();
            } else {
                opt_vars.times *= 0.8;
            }

            if (std::isnan(opt_vars.times.sum())) {
//                cout << YELLOW << " -- [ExpOpt] Init times and point failed." << RESET << endl;
                return false;
            }

            const Mat3Df deltas = opt_vars.init_path.rightCols(opt_vars.piece_num)
                                  - opt_vars.init_path.leftCols(opt_vars.piece_num);
            opt_vars.pieceIdx = (deltas.colwise().norm() / INFINITY).cast<int>().transpose();
            opt_vars.pieceIdx.array() += 1;

            opt_vars.temporalDim = opt_vars.piece_num;
            opt_vars.spatialDim = 0;
            opt_vars.vPolyIdx.resize(opt_vars.piece_num - 1);
            opt_vars.hPolyIdx.resize(opt_vars.piece_num);

            switch (cfg_.pos_constraint_type) {
                case 1: {
                    for (int i = 0, j = 0, k; i < opt_vars.piece_num; i++) {
                        k = opt_vars.pieceIdx(i);
                        for (int l = 0; l < k; l++, j++) {
                            if (l < k - 1) {
                                opt_vars.vPolyIdx(j) = 2 * i;
                            } else if (i < opt_vars.piece_num - 1) {
                                opt_vars.vPolyIdx(j) = 2 * i + 1;
                            }
                            opt_vars.hPolyIdx(j) = i;
                        }
                    }
                    opt_vars.spatialDim = 3 * (opt_vars.piece_num - 1);
                    break;
                }
                default: {
                    for (int i = 0, j = 0, k; i < opt_vars.piece_num; i++) {
                        k = opt_vars.pieceIdx(i);
                        for (int l = 0; l < k; l++, j++) {
                            if (l < k - 1) {
                                opt_vars.vPolyIdx(j) = 2 * i;
                                opt_vars.spatialDim += static_cast<int>(opt_vars.vPolytopes[2 * i].cols());
                            } else if (i < opt_vars.piece_num - 1) {
                                opt_vars.vPolyIdx(j) = 2 * i + 1;
                                opt_vars.spatialDim += static_cast<int>(opt_vars.vPolytopes[2 * i + 1].cols());
                            }
                            opt_vars.hPolyIdx(j) = i;
                        }
                    }
                }
            }

            opt_vars.minco.setConditions(opt_vars.headPVAJ, opt_vars.tailPVAJ, opt_vars.piece_num);
            opt_vars.gradByPoints.resize(3, opt_vars.piece_num - 1);
            opt_vars.gradByTimes.resize(opt_vars.piece_num);
            opt_vars.partialGradByCoeffs.resize(8 * opt_vars.piece_num, 3);
            opt_vars.partialGradByTimes.resize(opt_vars.piece_num);
            return true;
        }

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
