/*
 * Product-owned navigation implementation.
 * Algorithmic provenance and external attributions are documented in the
 * package documentation; they are not part of the runtime API or behaviour.
 */

#include <traj_opt/nominal_trajectory_optimizer.hpp>
#include <algorithm>
#include <chrono>
#include <utility>
#include <navigation_planning/planning_limits.hpp>
#include <traj_opt/trajectory_dynamics.hpp>
#include <planner_core/corridor_plane_validation.hpp>
#include <planner_core/deterministic_nominal_seed.hpp>
#include <planner_core/corridor_bezier_seed.hpp>
#include <planner_core/boundary_velocity_recovery.hpp>
#include <utils/optimization/lbfgs.h>
#include <planner_runtime_context/planner_runtime_context.hpp>
#include <planner_core/route_boundary_timing.hpp>

#define POS_IDX 1
#define VEL_IDX 2
#define ACC_IDX 3
#define JER_IDX 4
#define ATT_IDX 5
#define OMG_IDX 6
#define THR_IDX 7

using namespace traj_opt;
using namespace color_text;
using namespace navigation_math;
using namespace math_utils;
using namespace optimization_utils;

using Vec8f = Eigen::Matrix<double, 8, 1>;
using Mat83f = Eigen::Matrix<double, 8, 3>;

namespace {

}  // namespace

int ExpTrajOpt::monitorProgress(void *instance,
                               const VecDf &, const VecDf &, double,
                               double, int, int) {
    auto *vars = static_cast<OptimizationVariables *>(instance);
    if (!vars) return 1;
    if (vars->solve_cancelled &&
        vars->solve_cancelled->load(std::memory_order_relaxed)) return 1;
    if (vars->steady_deadline_ns > 0) {
        const auto now_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::steady_clock::now().time_since_epoch()).count();
        if (now_ns >= vars->steady_deadline_ns) return 1;
    }
    return 0;
}

void ExpTrajOpt::constraintsFunctional(const VecDf &T,
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
        // outputs
                                       double &cost,
                                       VecDf &gradT,
                                       MatD3f &gradC,
                                       VecDf &pena_log) {
    /* 1) define some varible alias*/
    const auto &vmax = magnitudeBounds[0];
    const auto &amax = magnitudeBounds[1];
    const auto &jmax = magnitudeBounds[2];
    const auto &omgmax = magnitudeBounds[3];
    const auto &accthrmin = magnitudeBounds[4];
    const auto &accthrmax = magnitudeBounds[5];

    const auto &vmaxSqr = vmax * vmax;
    const auto &amaxSqr = amax * amax;
    const auto &jmaxSqr = jmax * jmax;
    const auto &omgmaxSqr = omgmax * omgmax;

    const auto &thrustMean = 0.5 * (accthrmax + accthrmin);
    const auto &thrustRadi = 0.5 * std::abs(accthrmax - accthrmin);
    const auto &thrustSqrRadi = thrustRadi * thrustRadi;

    const auto &weightPos = penaltyWeights[0];
    const auto &weightVel = penaltyWeights[1];
    const auto &weightAcc = penaltyWeights[2];
    const auto &weightJer = penaltyWeights[3];
    const auto &weightAtt = penaltyWeights[4];
    const auto &weightOmg = penaltyWeights[5];
    const auto &weightAccThr = penaltyWeights[6];

    const auto &piece_num = T.size();

    const double integralFrac = 1.0 / integralResolution;
    VecDf max_pena(8);
    max_pena.setZero();

    /* 2) add integral cost */

    for (int i = 0; i < piece_num; i++) {
        const Mat83f &c = coeffs.block<8, 3>(i * 8, 0);
        const auto &step = T(i) * integralFrac;
        for (int j = 0; j <= integralResolution; j++) {
            double s1 = j * step;
            double s2 = s1 * s1;
            double s3 = s2 * s1;
            double s4 = s2 * s2;
            double s5 = s4 * s1;
            double s6 = s4 * s2;
            double s7 = s4 * s3;
            Vec8f beta0, beta1, beta2, beta3, beta4;
            beta0 << 1.0, s1, s2, s3, s4, s5, s6, s7;
            beta1 << 0.0, 1.0, 2.0 * s1, 3.0 * s2, 4.0 * s3, 5.0 * s4, 6.0 * s5, 7.0 * s6;
            beta2 << 0.0, 0.0, 2.0, 6.0 * s1, 12.0 * s2, 20.0 * s3, 30.0 * s4, 42.0 * s5;
            beta3 << 0.0, 0.0, 0.0, 6.0, 24.0 * s1, 60.0 * s2, 120.0 * s3, 210.0 * s4;
            beta4 << 0.0, 0.0, 0.0, 0.0, 24.0, 120.0 * s1, 360.0 * s2, 840.0 * s3;
            //beta5 << 0.0, 0.0, 0.0, 0., 0.0, 120.0, 720.0 * s1, 2520.0 * s2;

            const Vec3f pos = c.transpose() * beta0;
            const Vec3f vel = c.transpose() * beta1;
            const Vec3f acc = c.transpose() * beta2;
            const Vec3f jer = c.transpose() * beta3;
            const Vec3f sna = c.transpose() * beta4;

            double tmp_cost{0.0};
            Vec3f gradPos{0, 0, 0}, gradVel{0, 0, 0}, gradAcc{0, 0, 0}, gradJer{0, 0, 0};

            /* 2.1  For position cost */
            const auto &L = hIdx(i);
            const auto &K = hPolys[L].rows();
            if (weightPos > 0) {
                for (int k = 0; k < K; k++) {
                    const Vec3f outerNormal = hPolys[L].block<1, 3>(k, 0);
                    const double violaPos = outerNormal.dot(pos) + hPolys[L](k, 3);
                    if (violaPos > max_pena(POS_IDX)) max_pena(POS_IDX) = violaPos;
                    double violaPosPena, violaPosPenaD;
                    if (gcopter::smoothedL1(violaPos, smoothFactor, violaPosPena, violaPosPenaD)) {
                        gradPos += weightPos * violaPosPenaD * outerNormal;
                        tmp_cost += weightPos * violaPosPena;
                    }
                }
            }

            /* 2.2  For attract point cost  */
            if (weightAtt > 0.0) {
                const auto is_waypoint = (j == 0) && (i != 0);
                const auto is_end = ((j == integralResolution) && (i != piece_num - 1));
                const auto idx = is_end ? i : i - 1;

                if (is_waypoint || is_end) {
                    Vec3f p_a = pos - waypoint_attractor.col(idx);
                    const auto &violaAtt =
                            p_a.squaredNorm() - waypoint_attractor_dead_d(idx) * waypoint_attractor_dead_d(idx);
                    double violaAttPena, violaAttPenaD;
                    if (violaAtt > max_pena(ATT_IDX)) max_pena(ATT_IDX) = violaAtt;
                    if (gcopter::smoothedL1(violaAtt, smoothFactor, violaAttPena, violaAttPenaD)) {
                        gradPos += weightAtt * violaAttPenaD * 2.0 * p_a;
                        tmp_cost += weightAtt * violaAttPena;
                    }
                }
            }

            // Route-boundary acceptance is a junction contract, not a
            // corridor-center preference. Penalize only an excursion of the
            // junction outside the mission acceptance ball so the initialized
            // guide point remains a valid zero-penalty seed while the
            // independent post-solve check remains authoritative.
            const bool is_internal_junction =
                (j == integralResolution) && (i + 1 < piece_num);
            const int boundary_cell_index = i + 1;
            if (is_internal_junction &&
                boundary_cell_index >= 0 &&
                boundary_cell_index < static_cast<int>(route_boundary_points.size()) &&
                boundary_cell_index < static_cast<int>(route_boundary_radii.size()) &&
                route_boundary_points[static_cast<std::size_t>(boundary_cell_index)].allFinite() &&
                std::isfinite(route_boundary_radii[static_cast<std::size_t>(boundary_cell_index)]) &&
                route_boundary_radii[static_cast<std::size_t>(boundary_cell_index)] > 0.0) {
                const Vec3f boundary_error = pos -
                    route_boundary_points[static_cast<std::size_t>(boundary_cell_index)];
                const double boundary_violation = boundary_error.squaredNorm() -
                    std::pow(route_boundary_radii[static_cast<std::size_t>(boundary_cell_index)], 2.0);
                double boundary_penalty = 0.0;
                double boundary_penalty_derivative = 0.0;
                if (gcopter::smoothedL1(
                        boundary_violation, smoothFactor,
                        boundary_penalty, boundary_penalty_derivative)) {
                    const double boundary_weight = std::max(weightPos, 1.0);
                    gradPos += boundary_weight * boundary_penalty_derivative *
                        2.0 * boundary_error;
                    tmp_cost += boundary_weight * boundary_penalty;
                }
            }

            // Preserve the vertical shape of the collision-checked guide over
            // the complete polynomial, not only at MINCO junctions. High-order
            // minimum-snap pieces can otherwise bow far below two perfectly
            // valid guide points while all waypoint variables remain anchored.
            // A piecewise-linear z reference still permits deliberate climb
            // and descent encoded by A* and by the fixed boundary PVAJ.
            // Reference timing is intentionally piece-local: each guide
            // segment is matched to the corresponding polynomial segment.
            // The guide time allocation remains the independent duration seed.
            if ((route_reference_lateral_weight > 0.0 ||
                 route_reference_vertical_weight > 0.0) &&
                route_reference_points.rows() == 3 &&
                route_reference_points.cols() == piece_num - 1 &&
                route_reference_points.allFinite() &&
                route_reference_head.allFinite() &&
                route_reference_tail.allFinite()) {
                const Vec3f reference_start = i == 0
                        ? route_reference_head
                        : route_reference_points.col(i - 1);
                const Vec3f reference_end = i == piece_num - 1
                        ? route_reference_tail
                        : route_reference_points.col(i);
                const double reference_alpha = j * integralFrac;
                const Vec3f reference_point =
                        (1.0 - reference_alpha) * reference_start +
                        reference_alpha * reference_end;
                const Eigen::Vector2d lateral_error =
                        pos.head<2>() - reference_point.head<2>();
                const double lateral_distance = lateral_error.norm();
                if (route_reference_lateral_weight > 0.0 &&
                    lateral_distance > route_reference_lateral_deadband_m) {
                    const double excess = lateral_distance - route_reference_lateral_deadband_m;
                    gradPos.head<2>() +=
                        2.0 * route_reference_lateral_weight * excess *
                        lateral_error / lateral_distance;
                    tmp_cost += route_reference_lateral_weight * excess * excess;
                }
                const double vertical_error = pos.z() - reference_point.z();
                const double vertical_excess =
                    std::max(0.0, std::abs(vertical_error) - route_reference_vertical_deadband_m);
                if (route_reference_vertical_weight > 0.0 && vertical_excess > 0.0) {
                    gradPos.z() += 2.0 * route_reference_vertical_weight * vertical_excess *
                        (vertical_error >= 0.0 ? 1.0 : -1.0);
                    tmp_cost += route_reference_vertical_weight * vertical_excess * vertical_excess;
                }
            }

            /* 2.3 For vel cost  */
            const auto &violaVel = vel.squaredNorm() - vmaxSqr;
            double violaVelPena, violaVelPenaD;
            if (weightVel > 0 && gcopter::smoothedL1(violaVel, smoothFactor, violaVelPena, violaVelPenaD)) {
                gradVel += weightVel * violaVelPenaD * 2.0 * vel;
                tmp_cost += weightVel * violaVelPena;
                if (violaVel > max_pena(VEL_IDX)) max_pena(VEL_IDX) = violaVel;
            }

            /* 2.4 For acc cost  */
            const auto &violaAcc = acc.squaredNorm() - amaxSqr;
            double violaAccPena, violaAccPenaD;
            if (weightAcc > 0 && gcopter::smoothedL1(violaAcc, smoothFactor, violaAccPena, violaAccPenaD)) {
                gradAcc += weightAcc * violaAccPenaD * 2.0 * acc;
                tmp_cost += weightAcc * violaAccPena;
                if (violaAcc > max_pena(ACC_IDX)) max_pena(ACC_IDX) = violaAcc;
            }

            /* 2.5 For acc cost  */
            const auto &violaJer = jer.squaredNorm() - jmaxSqr;
            double violaJerPena, violaJerPenaD;
            if (weightJer > 0 && gcopter::smoothedL1(violaJer, smoothFactor, violaJerPena, violaJerPenaD)) {
                gradJer += weightJer * violaJerPenaD * 2.0 * jer;
                tmp_cost += weightJer * violaJerPena;
                if (violaJer > max_pena(JER_IDX)) max_pena(JER_IDX) = violaJer;
            }

            Vec3f totalGradPos{0.0, 0.0, 0.0}, totalGradVel{0.0, 0.0, 0.0},
                    totalGradAcc{0.0, 0.0, 0.0}, totalGradJer{0.0, 0.0, 0.0};

            /* 2.6  For omg amd thr cost  */
            if (weightOmg > 0 || weightAccThr > 0) {
                double thr;
                Vec4f quat;
                Vec3f omg;
                flatMap.forward(vel, acc, jer, 0.0, 0.0, thr, quat, omg);
                const auto &violaOmg = omg.squaredNorm() - omgmaxSqr;
                const auto &violaThrust = (thr - thrustMean) * (thr - thrustMean) - thrustSqrRadi;

                /* 2.6.1  For omg cost  */
                double violaOmgPena, violaOmgPenaD;
                Vec3f gradOmg{0, 0, 0};
                if (weightOmg > 0 && gcopter::smoothedL1(violaOmg, smoothFactor, violaOmgPena, violaOmgPenaD)) {
                    gradOmg += weightOmg * violaOmgPenaD * 2.0 * omg;
                    tmp_cost += weightOmg * violaOmgPena;
                    if (violaOmg > max_pena(OMG_IDX)) max_pena(OMG_IDX) = violaOmg;
                }

                /* 2.6.2  For thr cost  */
                double violaThrustPena, violaThrustPenaD;
                double gradThr{0.0};
                if (weightAccThr > 0 &&
                    gcopter::smoothedL1(violaThrust, smoothFactor, violaThrustPena, violaThrustPenaD)) {
                    gradThr += weightAccThr * violaThrustPenaD * 2.0 * (thr - thrustMean);
                    tmp_cost += weightAccThr * violaThrustPena;
                    if (violaThrust > max_pena(THR_IDX)) max_pena(THR_IDX) = violaThrust;
                }
                double totalGradPsi{0.0}, totalGradPsiD{0.0};
                flatMap.backward(gradPos, gradVel, gradAcc, gradJer, gradThr, Vec4f(0, 0, 0, 0), gradOmg,
                                 totalGradPos, totalGradVel, totalGradAcc, totalGradJer,
                                 totalGradPsi, totalGradPsiD);
            } else {
                totalGradPos = gradPos;
                totalGradVel = gradVel;
                totalGradAcc = gradAcc;
                totalGradJer = gradJer;
            }

            const auto node = (j == 0 || j == integralResolution) ? 0.5 : 1.0;
            const double alpha = j * integralFrac;
            gradC.block<8, 3>(i * 8, 0) += (beta0 * totalGradPos.transpose() +
                                            beta1 * totalGradVel.transpose() +
                                            beta2 * totalGradAcc.transpose() +
                                            beta3 * totalGradJer.transpose()) *
                                           node * step;
            gradT(i) += (totalGradPos.dot(vel) +
                         totalGradVel.dot(acc) +
                         totalGradAcc.dot(jer) +
                         totalGradJer.dot(sna)) *
                        alpha * node * step +
                        node * integralFrac * tmp_cost;
            cost += node * step * tmp_cost;
        }
    }

    /* 3) log all violations */
    pena_log.tail(7) = max_pena.tail(7);
}


/*
 * @ brief: This is the callback function of the L-BFGS solver
 *
 */
double ExpTrajOpt::costFunctional(void *ptr,
                                  const VecDf &x,
                                  VecDf &g) {
    /* 1) Decode the pointer */
    OptimizationVariables &obj = *static_cast<OptimizationVariables *>(ptr);
    const auto &dimTau = obj.temporalDim;
    const auto &dimXi = obj.spatialDim;
    const auto &weightT = obj.rho;
    const auto &vPolyIdx = obj.vPolyIdx;
    const auto &vPolytopes = obj.vPolytopes;
    const auto &hPolyIdx = obj.hPolyIdx;
    const auto &hPolytopes = obj.hPolytopes;
    const auto &waypoint_attractor = obj.waypoint_attractor;
    const auto &waypoint_attractor_dead_d = obj.waypoint_attractor_dead_d;
    const auto &smooth_eps = obj.smooth_eps;
    const auto &integral_res = obj.integral_res;
    const auto &magnitudeBounds = obj.magnitudeBounds;
    const auto &penaltyWeights = obj.penaltyWeights;
    const auto &block_energy_cost = obj.block_energy_cost;
    const auto &route_reference_lateral_weight = obj.route_reference_lateral_weight;
    const auto &route_reference_vertical_weight = obj.route_reference_vertical_weight;
    const auto &route_reference_lateral_deadband_m = obj.route_reference_lateral_deadband_m;
    const auto &route_reference_vertical_deadband_m = obj.route_reference_vertical_deadband_m;

    auto &quadrotor_flatness = obj.quadrotor_flatness;

    obj.iter_num++;
    const auto &pos_constraint_type = obj.pos_constraint_type;

    const Eigen::Map<const VecDf> tau(x.data(), dimTau);
    const Eigen::Map<const VecDf> xi(x.data() + dimTau, dimXi);
    Eigen::Map<VecDf> gradTau(g.data(), dimTau);
    Eigen::Map<VecDf> gradXi(g.data() + dimTau, dimXi);

    const auto recordNonFinite = [&](const int stage, const int value_mask,
                                     const double cost, const double gradient_norm,
                                     const VecDf *durations = nullptr) {
        ++obj.nonfinite_evaluation_count;
        if (obj.first_nonfinite_stage != 0) return;
        obj.first_nonfinite_stage = stage;
        obj.first_nonfinite_value_mask = value_mask;
        obj.first_nonfinite_attempt = obj.solver_attempt;
        obj.first_nonfinite_iteration = obj.iter_num;
        obj.first_nonfinite_cost = cost;
        obj.first_nonfinite_gradient_norm = gradient_norm;
        const VecDf &snapshot = durations != nullptr ? *durations : obj.times;
        if (snapshot.size() > 0) {
            obj.first_nonfinite_min_duration_s = snapshot.minCoeff();
            obj.first_nonfinite_max_duration_s = snapshot.maxCoeff();
        }
    };

    if (!x.allFinite()) {
        recordNonFinite(1, 1, std::numeric_limits<double>::quiet_NaN(),
                        std::numeric_limits<double>::quiet_NaN());
        return INFINITY;
    }

    /* 2) Reconstruct the optimization variables */

    Mat3Df points;
    VecDf times;
    gcopter::forwardMapTauToT(tau, times);
    if (obj.duration_lower_bound.size() != 0 &&
        obj.duration_lower_bound.size() != times.size()) {
        recordNonFinite(2, 2, std::numeric_limits<double>::quiet_NaN(),
                        std::numeric_limits<double>::quiet_NaN(), &times);
        return INFINITY;
    }
    if (obj.duration_lower_bound.size() == times.size()) {
        times += obj.duration_lower_bound;
    }
    if (!times.allFinite() || times.size() == 0 || times.minCoeff() <= 0.0) {
        recordNonFinite(2, 1, std::numeric_limits<double>::quiet_NaN(),
                        std::numeric_limits<double>::quiet_NaN(), &times);
        return INFINITY;
    }
    switch (pos_constraint_type) {
        case 1: {
            VecDf xi_e = xi;
            points = Eigen::Map<Eigen::Matrix<double, 3, Eigen::Dynamic>>(xi_e.data(), 3, xi_e.size() / 3);
            break;
        }
        default: {
            gcopter::forwardP(xi, vPolyIdx, vPolytopes, points);
            break;
        }
    }
    // Route-boundary junctions are initialized at the measured mission
    // waypoint, but remain optimization variables inside the corridor. The
    // acceptance ball penalty below and the independent post-solve check own
    // the boundary. Re-pinning every evaluation to the ball centre makes a
    // non-zero-speed C3 corner geometrically identical to a point turn.
    if (!points.allFinite()) {
        recordNonFinite(3, 1, std::numeric_limits<double>::quiet_NaN(),
                        std::numeric_limits<double>::quiet_NaN(), &times);
        return INFINITY;
    }

    /* 3) Compute the energy const and gradient */
    double cost{0};
    obj.minco.setParameters(points, times);
    MatD3f partialGradByCoeffs(8 * times.size(), 3);
    VecDf partialGradByTimes(times.size());
    partialGradByCoeffs.setZero();
    partialGradByTimes.setZero();
    if (!block_energy_cost) {
        obj.minco.getEnergy(cost);
        obj.minco.getEnergyPartialGradByCoeffs(partialGradByCoeffs);
        obj.minco.getEnergyPartialGradByTimes(partialGradByTimes);
    }
    obj.penalty_log(0) = cost;
    const int minco_value_mask =
            (!std::isfinite(cost) ? 1 : 0) |
            (!partialGradByCoeffs.allFinite() ? 2 : 0) |
            (!partialGradByTimes.allFinite() ? 4 : 0) |
            (!obj.minco.getCoeffs().allFinite() ? 8 : 0);
    if (minco_value_mask != 0) {
        recordNonFinite(4, minco_value_mask, cost,
                        std::numeric_limits<double>::quiet_NaN(), &times);
        return INFINITY;
    }

    /* 4) Compute the constrain cost and gradient  */
    constraintsFunctional(times, obj.minco.getCoeffs(),
                          hPolyIdx, hPolytopes,
                          waypoint_attractor, waypoint_attractor_dead_d,
                          obj.route_reference_points,
                          obj.route_boundary_points, obj.route_boundary_radii,
                          obj.headPVAJ.col(0), obj.tailPVAJ.col(0),
                          route_reference_lateral_weight,
                          route_reference_vertical_weight,
                          route_reference_lateral_deadband_m,
                          route_reference_vertical_deadband_m,
                          smooth_eps, integral_res,
                          magnitudeBounds, penaltyWeights,
                          quadrotor_flatness,
                          cost, partialGradByTimes, partialGradByCoeffs, obj.penalty_log);
    const int objective_value_mask =
            (!std::isfinite(cost) ? 1 : 0) |
            (!partialGradByCoeffs.allFinite() ? 2 : 0) |
            (!partialGradByTimes.allFinite() ? 4 : 0) |
            (!obj.penalty_log.allFinite() ? 8 : 0);
    if (objective_value_mask != 0) {
        recordNonFinite(5, objective_value_mask, cost,
                        std::numeric_limits<double>::quiet_NaN(), &times);
        return INFINITY;
    }

    /* 5) Propagate the gradient from CT to PT */
    Mat3Df gradByPoints;
    VecDf gradByTimes;
    obj.minco.propogateGrad(partialGradByCoeffs, partialGradByTimes,
                            gradByPoints, gradByTimes);
    cost += weightT * times.sum();
    gradByTimes.array() += weightT;

    /* 6) Propagate the gradient from PT to optimization variables. */
    gcopter::propagateGradientTToTau(tau, gradByTimes, gradTau);
    switch (pos_constraint_type) {
        case 1: {
            MatDf gp = gradByPoints;
            gradXi = Eigen::Map<VecDf>(gp.data(), gp.size());
            break;
        }
        default: {
            gcopter::backwardGradP(xi, vPolyIdx, vPolytopes, gradByPoints, gradXi);
            gcopter::normRetrictionLayer(xi, vPolyIdx, vPolytopes, cost, gradXi);
            break;
        }
    }
    const int gradient_value_mask =
            (!std::isfinite(cost) ? 1 : 0) |
            (!gradTau.allFinite() ? 2 : 0) |
            (!gradXi.allFinite() ? 4 : 0);
    if (gradient_value_mask != 0) {
        recordNonFinite(6, gradient_value_mask, cost, g.norm(), &times);
        return INFINITY;
    }
    return cost;
}

static void truncateToSixDecimals(double &num) {
    num = std::trunc(num * 1e6) / 1e6; // 直接截断，无四舍五入
}

bool ExpTrajOpt::processCorridorWithGuideTraj() {
    if (opt_vars.hPolytopes.empty() ||
        opt_vars.guide_path.size() < 2U ||
        opt_vars.guide_path.size() != opt_vars.guide_t.size()) {
        planner_context_->warn(
                " -- [ExpOpt] corridor/guide input is empty or inconsistent");
        return false;
    }
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
    opt_vars.route_reference_points.resize(3, sizeCorridor);
    // * 2) Process the corridor
    for (int i = 0; i < sizeCorridor; i++) {
        // * 2.1) Get current vertex
        if (!geometry_utils::enumerateVs(opt_vars.hPolytopes[i], curIV)) {
            cout << YELLOW << " -- [planner] in [ GcopterExpS4::processCorridor]: Failed to enumerate corridor Vs."
                 << RESET << endl;

            return false;
        }
        if (curIV.cols() <= 0) {
            planner_context_->warn(
                    " -- [ExpOpt] corridor {} has no finite vertices", i);
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
        if (!std::isfinite(dis) || dis < 0.0) {

            cout << YELLOW << " -- [planner] in [ GcopterExpS4::processCorridor]: Failed findInteriorDist Vs." <<
                 RESET << endl;
            return false;
        }
        curIV.resize(3, 0);
        geometry_utils::enumerateVs(curIH, interior, curIV);
        if (curIV.cols() == 0) {
            planner_context_->warn(
                    " -- [ExpOpt] Corridor overlap {} cannot be vertex-enumerated "
                    "(corridors={}, overlap_depth={})",
                    i, opt_vars.hPolytopes.size(), 2.0 * dis);
            return false;
        }
        const double test_sum = curIV.sum();
        if (std::isnan(test_sum) || std::isinf(test_sum)) {
            planner_context_->warn(
                    " -- [ExpOpt] Corridor overlap {} produced non-finite vertices "
                    "(corridors={}, overlap_depth={})",
                    i, opt_vars.hPolytopes.size(), 2.0 * dis);
            return false;
        }
        opt_vars.waypoint_attractor.col(i) = interior;
        opt_vars.waypoint_attractor_dead_d(i) = dis;
        // Safe default when no guide sample belongs to this overlap.
        opt_vars.points.col(i) = interior;
        nv = curIV.cols();
        curIOB.resize(3, nv);
        curIOB.col(0) = curIV.col(0);
        curIOB.rightCols(nv - 1) = curIV.rightCols(nv - 1).colwise() - curIV.col(0);
        opt_vars.vPolytopes.push_back(curIOB);
    }

    // * 3) Time and waypoint allocation for hot initialization
    VecDi min_id(opt_vars.waypoint_attractor.cols());
    VecDf time_stamps(opt_vars.waypoint_attractor.cols() + 2);
    time_stamps(0) = 0.0;
    time_stamps(opt_vars.waypoint_attractor.cols() + 1) = opt_vars.guide_t.back();
    min_id.setConstant(0);
    int first_guide_index = 0;
    for (int j = 0; j < opt_vars.waypoint_attractor.cols(); j++) {
        const Vec3f interior = opt_vars.waypoint_attractor.col(j);
        int nearest_index = first_guide_index;
        double nearest_distance = std::numeric_limits<double>::max();
        // A route-boundary gate occupies one hard corridor cell.  The overlap
        // after that cell is a distinct temporal junction, even when both
        // neighbouring overlap interiors are closest to the same mission
        // waypoint.  Advance to the next guide sample for that outgoing
        // junction so MINCO receives an executable turn duration instead of
        // the historical 0.01 s clamp.
        const bool outgoing_from_route_gate =
                j < static_cast<int>(opt_vars.route_boundary_gates.size()) &&
                opt_vars.route_boundary_gates[static_cast<std::size_t>(j)] != 0U;
        const int guide_search_start = outgoing_from_route_gate &&
                first_guide_index + 1 < static_cast<int>(opt_vars.guide_path.size())
                ? first_guide_index + 1
                : first_guide_index;
        for (int i = guide_search_start; i < static_cast<int>(opt_vars.guide_path.size()); i++) {
            const double distance = (opt_vars.guide_path[i] - interior).norm();
            if (distance < nearest_distance) {
                nearest_distance = distance;
                nearest_index = i;
            }
        }

        min_id[j] = nearest_index;
        first_guide_index = nearest_index;
        const Vec3f &guide_point = opt_vars.guide_path[nearest_index];
        // Keep the collision-checked guide as a distinct reference even when
        // its nearest sample is not inside the overlap and the MINCO junction
        // itself must initialize at the overlap interior.
        opt_vars.route_reference_points.col(j) = guide_point;
        const VecDf guide_plane_values =
            opt_vars.hOverlapPolytopes[j].leftCols(3) * guide_point +
            opt_vars.hOverlapPolytopes[j].col(3);
        // Initialize on the collision-checked A* route when the corresponding
        // sample belongs to this convex overlap. The guide-plane test above
        // guarantees feasibility. A centre-dominant blend creates lateral and
        // vertical drift in open space; the receding prefix then feeds that
        // drift into every later solve.
        opt_vars.points.col(j) = guide_plane_values.maxCoeff() <= 1.0e-6
                ? guide_point
                : interior;
        const int boundary_cell_index = j + 1;
        if (boundary_cell_index < static_cast<int>(opt_vars.route_boundary_points.size()) &&
            boundary_cell_index < static_cast<int>(opt_vars.route_boundary_radii.size()) &&
            opt_vars.route_boundary_points[static_cast<std::size_t>(boundary_cell_index)].allFinite() &&
            std::isfinite(opt_vars.route_boundary_radii[static_cast<std::size_t>(boundary_cell_index)]) &&
            opt_vars.route_boundary_radii[static_cast<std::size_t>(boundary_cell_index)] > 0.0) {
            const Vec3f &boundary_point =
                opt_vars.route_boundary_points[static_cast<std::size_t>(boundary_cell_index)];
            const VecDf boundary_plane_values =
                opt_vars.hOverlapPolytopes[j].leftCols(3) * boundary_point +
                opt_vars.hOverlapPolytopes[j].col(3);
            if (!boundary_plane_values.allFinite() ||
                boundary_plane_values.maxCoeff() > cfg_.corridor_plane_tolerance_m) {
                planner_context_->warn(
                    " -- [ExpOpt] route-boundary point {} is not in its incoming overlap "
                    "(violation={})",
                    boundary_cell_index, boundary_plane_values.maxCoeff());
                return false;
            }
            // Seed the junction at the mission-owned boundary. The objective
            // retains a soft outside-ball penalty, while the final junction
            // check below is the hard acceptance authority.
            opt_vars.points.col(j) = boundary_point;
            opt_vars.route_reference_points.col(j) = boundary_point;
        }
        time_stamps(j + 1) = navigation_planning_backend::routeBoundaryJunctionTime(
                outgoing_from_route_gate, nearest_index, opt_vars.guide_t.size(), j,
                time_stamps(j), opt_vars.guide_t.back(), opt_vars.guide_t[nearest_index]);
    }

    std::vector<double> monotonic_time_stamps(
            time_stamps.data(), time_stamps.data() + time_stamps.size());
    if (!navigation_planning_backend::spreadRepeatedGuideJunctionTimes(
            monotonic_time_stamps)) {
        planner_context_->warn(
                " -- [ExpOpt] guide timestamp allocation is not strictly monotonic");
        return false;
    }
    for (Eigen::Index i = 1; i < time_stamps.size(); ++i) {
        opt_vars.times(i - 1) =
                monotonic_time_stamps[static_cast<std::size_t>(i)] -
                monotonic_time_stamps[static_cast<std::size_t>(i - 1)];
        if (!std::isfinite(opt_vars.times(i - 1)) ||
            opt_vars.times(i - 1) <= 0.0) {
            planner_context_->warn(
                    " -- [ExpOpt] guide duration allocation produced invalid segment {}: {}",
                    i - 1, opt_vars.times(i - 1));
            return false;
        }
    }

    if (!geometry_utils::enumerateVs(opt_vars.hPolytopes.back(), curIV)) {
        planner_context_->warn(" -- [ExpOpt] Final corridor {} cannot be vertex-enumerated",
                       opt_vars.hPolytopes.size() - 1);
        return false;
    }
    if (curIV.cols() <= 0) {
        planner_context_->warn(" -- [ExpOpt] final corridor has no finite vertices");
        return false;
    }
    nv = curIV.cols();
    curIOB.resize(3, nv);
    curIOB.col(0) = curIV.col(0);
    curIOB.rightCols(nv - 1) = curIV.rightCols(nv - 1).colwise() - curIV.col(0);
    opt_vars.vPolytopes.push_back(curIOB);
    return true;
}

bool ExpTrajOpt::setupProblemAndCheck() {
    if (opt_vars.hPolytopes.empty() ||
        opt_vars.hPolytopes.size() >
            static_cast<std::size_t>(std::numeric_limits<int>::max())) {
        return false;
    }
    // init internal variables size;
    opt_vars.piece_num = static_cast<int>(opt_vars.hPolytopes.size());
    opt_vars.times.resize(opt_vars.piece_num);
    opt_vars.points.resize(3, opt_vars.piece_num - 1);


    // Check corridor and init points
    if (opt_vars.default_init) {
        throw std::runtime_error("Not support default init in this version.");
    } else {
        if (!processCorridorWithGuideTraj()) {
            return false;
        }
    }
    opt_vars.init_path.resize(3, opt_vars.piece_num + 1);
    for (long i = 0; i < opt_vars.piece_num - 1; i++) {
        opt_vars.init_path.col(i + 1) = opt_vars.waypoint_attractor.col(i);
    }
    opt_vars.init_path.col(0) = opt_vars.headPVAJ.col(0);
    opt_vars.init_path.rightCols(1) = opt_vars.tailPVAJ.col(0);
    // The guide branch already populated times from its configured guide
    // allocation. Preserve that seed; it is an initialization, not a
    // dynamic certificate, and hard feasibility gates remain authoritative.
    if (!opt_vars.times.allFinite() || opt_vars.times.minCoeff() <= 0.0) {
        cout << YELLOW << " -- [ExpOpt] Init times and point failed: non-positive or non-finite duration." << RESET << endl;
        return false;
    }

    const Mat3Df deltas = opt_vars.init_path.rightCols(opt_vars.piece_num)
                          - opt_vars.init_path.leftCols(opt_vars.piece_num);
    if (!deltas.allFinite()) {
        cout << YELLOW << " -- [ExpOpt] Initial path contains non-finite edge." << RESET << endl;
        return false;
    }
    // CorridorGenerator already subdivides every collision-certified edge to
    // cfg_.corridor_segment_max_length_m. Keep one MINCO piece per certified
    // cell explicitly; the old division by INFINITY was an opaque placeholder
    // that silently collapsed all finite distances to zero and made malformed
    // floating-point inputs reach an integer cast.
    opt_vars.pieceIdx = VecDi::Ones(opt_vars.piece_num);

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

bool ExpTrajOpt::setInitPsAndTs(const vec_Vec3f &init_ps, const vector<double> &init_ts) {
    opt_vars.default_init = false;
    if (opt_vars.times.size() == 0 || init_ts.empty() ||
        opt_vars.times.size() != static_cast<Eigen::Index>(init_ts.size())) {
        return false;
    }
    if (opt_vars.points.cols() != static_cast<Eigen::Index>(init_ps.size())) {
        return false;
    }
    for (std::size_t i = 0; i < init_ts.size(); ++i) {
        if (!std::isfinite(init_ts[i]) || init_ts[i] <= 0.0) {
            return false;
        }
    }
    if (!std::all_of(init_ps.begin(), init_ps.end(),
                     [](const Vec3f &point) { return point.allFinite(); })) {
        return false;
    }

    for (long i = 0; i < opt_vars.points.cols(); i++) {
        opt_vars.times[i] = init_ts[i];
        opt_vars.points.col(i) = init_ps[i];
    }
    opt_vars.times[opt_vars.times.size() - 1] = init_ts.back();
    return true;
}

double ExpTrajOpt::optimize(Trajectory &traj, const double &relCostTol) {
    resetDiagnostics();
    diagnostics_.valid = true;

    if (!std::isfinite(relCostTol) || relCostTol <= 0.0 ||
        opt_vars.times.size() <= 0) {
        planner_context_->warn(
                " -- [ExpOpt] invalid relative cost tolerance or empty durations");
        traj.clear();
        return INFINITY;
    }

    /* 1) Allocate the vector for optimization variables. */
    VecDf x(opt_vars.temporalDim + opt_vars.spatialDim);
    /*    creat map for the opt_var vector */
    Eigen::Map<VecDf> tau(x.data(), opt_vars.temporalDim);
    Eigen::Map<VecDf> xi(x.data() + opt_vars.temporalDim, opt_vars.spatialDim);

    opt_vars.penalty_log.resize(8);
    opt_vars.penalty_log.setZero();
    opt_vars.solver_attempt = 0;
    opt_vars.nonfinite_evaluation_count = 0;
    opt_vars.first_nonfinite_stage = 0;
    opt_vars.first_nonfinite_value_mask = 0;
    opt_vars.first_nonfinite_attempt = 0;
    opt_vars.first_nonfinite_iteration = 0;
    opt_vars.first_nonfinite_min_duration_s =
            std::numeric_limits<double>::quiet_NaN();
    opt_vars.first_nonfinite_max_duration_s =
            std::numeric_limits<double>::quiet_NaN();
    opt_vars.first_nonfinite_cost = std::numeric_limits<double>::quiet_NaN();
    opt_vars.first_nonfinite_gradient_norm =
            std::numeric_limits<double>::quiet_NaN();

    /* 2) Check the initial value of the optimization variables. */
    if (!opt_vars.times.allFinite() || opt_vars.times.minCoeff() < 1e-3) {
        cout << YELLOW << " -- [TrajOpt] Error, the init times have zero, force return." << RESET << endl;
        cout << " -- Head PVAJ: " << endl;
        cout << opt_vars.headPVAJ << endl;
        cout << " -- Head PVAJ: " << endl;
        cout << opt_vars.tailPVAJ << endl;
        cout << " -- Times: " << endl;
        cout << opt_vars.times.transpose() << endl;
        return INFINITY;
    }

    if (opt_vars.given_init_ts_and_ps) {
        opt_vars.times = opt_vars.init_ts;
        for (std::size_t i = 0; i < opt_vars.init_ps.size(); ++i) {
            opt_vars.points.col(static_cast<Eigen::Index>(i)) = opt_vars.init_ps[i];
        }
    }
    // Use a conservative interior target for the optimizer's soft dynamic
    // penalties. The hard certificate below remains tied to cfg_ limits, and
    // boundary states are never penalized merely for being above the interior
    // target when they are already physically valid.
    const double reserve = cfg_.optimization_dynamic_reserve_ratio;
    if (!std::isfinite(reserve) || reserve <= 0.0 || reserve > 1.0) {
        planner_context_->warn(
                " -- [ExpOpt] invalid dynamic optimization reserve ratio={}", reserve);
        traj.clear();
        return INFINITY;
    }
    opt_vars.magnitudeBounds[0] = std::max(
            cfg_.max_vel * reserve, opt_vars.headPVAJ.col(1).norm());
    opt_vars.magnitudeBounds[1] = std::max(
            cfg_.max_acc * reserve, opt_vars.headPVAJ.col(2).norm());
    opt_vars.magnitudeBounds[2] = std::max(
            cfg_.max_jerk * reserve, opt_vars.headPVAJ.col(3).norm());
    if (!opt_vars.magnitudeBounds.allFinite() ||
        (opt_vars.magnitudeBounds.array() <= 0.0).any()) {
        planner_context_->warn(
                " -- [ExpOpt] invalid dynamic optimization bounds after reserve");
        traj.clear();
        return INFINITY;
    }
    diagnostics_.initial_duration_s = opt_vars.times.sum();
    diagnostics_.initial_minimum_piece_duration_s = opt_vars.times.minCoeff();
    diagnostics_.initial_maximum_piece_duration_s = opt_vars.times.maxCoeff();

    /* 3) Construct the initial guess of the optimization variables. */
    opt_vars.duration_lower_bound.resize(0);
    // Keep the collision-checked guide available to the route-reference
    // quality objective on every solve. Its weights are explicit quality
    // parameters; a malformed reference must never silently disable the
    // objective by becoming a self-reference.
    if (opt_vars.route_reference_points.rows() != opt_vars.points.rows() ||
        opt_vars.route_reference_points.cols() != opt_vars.points.cols() ||
        !opt_vars.route_reference_points.allFinite() ||
        !opt_vars.headPVAJ.col(0).allFinite() ||
        !opt_vars.tailPVAJ.col(0).allFinite()) {
        planner_context_->warn(
            " -- [ExpOpt] route reference shape or values are invalid");
        return INFINITY;
    }
    gcopter::backwardMapTToTau(opt_vars.times, tau);
    switch (opt_vars.pos_constraint_type) {
        case 1: {
            MatDf p_e = opt_vars.points;
            xi = Eigen::Map<const VecDf>(p_e.data(), p_e.size());
            break;
        }
        default: {
            gcopter::backwardP(opt_vars.points, opt_vars.vPolyIdx, opt_vars.vPolytopes, xi);
            break;
        }
    }

    /* 4) setup the optimizer's parameters*/
    opt_vars.iter_num = 0;
    double minCostFunctional{0};
    lbfgs::lbfgs_parameter_t lbfgs_params;
    lbfgs_params.mem_size = cfg_.lbfgs_memory_size;
    lbfgs_params.past = 3;
    lbfgs_params.min_step = 1.0e-32;
    lbfgs_params.g_epsilon = 0.0;
    lbfgs_params.delta = relCostTol;
    opt_vars.init_ts = opt_vars.times;
    opt_vars.init_ps.clear();
    for (int col = 0; col < opt_vars.points.cols(); col++) {
        opt_vars.init_ps.emplace_back(opt_vars.points.col(col));
    }
    const Mat3Df route_reference_points = opt_vars.route_reference_points;

    // keep fixed accuracy for
    for (int i = 0; i < opt_vars.waypoint_attractor_dead_d.size(); i++) {
        truncateToSixDecimals(opt_vars.waypoint_attractor_dead_d(i));
        truncateToSixDecimals(opt_vars.waypoint_attractor(0, i));
        truncateToSixDecimals(opt_vars.waypoint_attractor(1, i));
        truncateToSixDecimals(opt_vars.waypoint_attractor(2, i));
    }

    // Freeze the complete seed before L-BFGS can mutate x, MINCO parameters,
    // durations, or spatial points. A later fallback may copy only this exact
    // object and only when this independent certificate already passed.
    Trajectory immutable_nominal_seed;
    opt_vars.minco.setParameters(opt_vars.points, opt_vars.times);
    opt_vars.minco.getTrajectory(immutable_nominal_seed);
    auto corridor_seed_result =
            navigation_planning_backend::buildCorridorContainedBezierSeed(
                opt_vars.headPVAJ, opt_vars.tailPVAJ, opt_vars.points,
                opt_vars.times, opt_vars.hPolytopes, opt_vars.hPolyIdx,
                cfg_.max_vel * cfg_.optimization_dynamic_reserve_ratio,
                cfg_.corridor_plane_tolerance_m);
    diagnostics_.corridor_seed_build_failure_stage =
        static_cast<int>(corridor_seed_result.failure_stage);
    if (!corridor_seed_result.valid &&
        corridor_seed_result.failing_piece_index >= 0) {
        planner_context_->warn(
                " -- [ExpOpt] corridor-contained seed failed: stage={} piece={} "
                "control={} plane={} violation_m={}",
                static_cast<int>(corridor_seed_result.failure_stage),
                corridor_seed_result.failing_piece_index,
                corridor_seed_result.failing_control_index,
                corridor_seed_result.failing_plane_index,
                corridor_seed_result.maximum_plane_violation_m);
    }
    Trajectory deterministic_nominal_seed = immutable_nominal_seed;
    navigation_planning_backend::DeterministicNominalSeedCertificate
            deterministic_seed_certificate;
    bool deterministic_seed_uses_corridor_bezier = false;
    double deterministic_seed_duration_scale = 1.0;
    if (corridor_seed_result.valid) {
        deterministic_seed_certificate =
                navigation_planning_backend::certifyDeterministicNominalSeed(
                    corridor_seed_result.trajectory,
                    opt_vars.hPolytopes,
                    opt_vars.hPolyIdx,
                    opt_vars.route_boundary_gates,
                    opt_vars.route_boundary_points,
                    opt_vars.route_boundary_radii,
                    opt_vars.headPVAJ,
                    opt_vars.tailPVAJ,
                    cfg_);
        if (deterministic_seed_certificate.valid) {
            deterministic_nominal_seed = corridor_seed_result.trajectory;
            deterministic_seed_uses_corridor_bezier = true;
            diagnostics_.corridor_seed_selected_mode = 1;
            diagnostics_.corridor_seed_selected_max_duration_scale = 1.0;
        } else {
            // Corridor construction has already established geometry.  If the
            // only failure is V/A/J, rebuild it with a finite set of uniform
            // duration reserves.  Endpoint P/V/A/J remain immutable, so every
            // rebuilt candidate still requires the complete certificate.
            std::vector<std::pair<int, VecDf>> retry_duration_candidates;
            const VecDf piece_scales = navigation_planning_backend::
                    boundedPieceDurationRetryScales(
                        corridor_seed_result.trajectory, cfg_);
            if (piece_scales.size() == opt_vars.times.size() &&
                piece_scales.allFinite() && piece_scales.maxCoeff() > 1.0) {
                retry_duration_candidates.emplace_back(
                    2, opt_vars.times.cwiseProduct(piece_scales));
            }
            const auto retry_scales = navigation_planning_backend::
                    boundedDynamicDurationRetryScales(
                        deterministic_seed_certificate, cfg_);
            for (const double duration_scale : retry_scales) {
                retry_duration_candidates.emplace_back(
                    3, opt_vars.times * duration_scale);
            }
            for (const auto& [retry_mode, retry_times] : retry_duration_candidates) {
                ++diagnostics_.corridor_seed_retry_attempt_count;
                auto retry_seed = navigation_planning_backend::
                        buildCorridorContainedBezierSeed(
                            opt_vars.headPVAJ, opt_vars.tailPVAJ,
                            opt_vars.points, retry_times,
                            opt_vars.hPolytopes, opt_vars.hPolyIdx,
                            cfg_.max_vel *
                                cfg_.optimization_dynamic_reserve_ratio,
                            cfg_.corridor_plane_tolerance_m);
                if (!retry_seed.valid) continue;
                ++diagnostics_.corridor_seed_retry_build_valid_count;
                const auto retry_certificate = navigation_planning_backend::
                        certifyDeterministicNominalSeed(
                            retry_seed.trajectory,
                            opt_vars.hPolytopes,
                            opt_vars.hPolyIdx,
                            opt_vars.route_boundary_gates,
                            opt_vars.route_boundary_points,
                            opt_vars.route_boundary_radii,
                            opt_vars.headPVAJ,
                            opt_vars.tailPVAJ,
                            cfg_);
                diagnostics_.corridor_seed_retry_last_certificate_stage =
                    static_cast<int>(retry_certificate.failure_stage);
                if (!retry_certificate.valid) continue;
                corridor_seed_result = std::move(retry_seed);
                deterministic_nominal_seed = corridor_seed_result.trajectory;
                deterministic_seed_certificate = retry_certificate;
                deterministic_seed_uses_corridor_bezier = true;
                deterministic_seed_duration_scale =
                    (retry_times.array() / opt_vars.times.array()).maxCoeff();
                diagnostics_.corridor_seed_selected_mode = retry_mode;
                diagnostics_.corridor_seed_selected_max_duration_scale =
                    deterministic_seed_duration_scale;
                break;
            }
        }
    }
    // Preserve the historical exact MINCO interpolation only as a secondary
    // fallback. Avoid running its expensive continuous certificate when the
    // convex-hull-contained baseline already passed every hard gate.
    if (!deterministic_seed_certificate.valid) {
        const auto immutable_minco_seed_certificate =
                navigation_planning_backend::certifyDeterministicNominalSeed(
                    immutable_nominal_seed,
                    opt_vars.hPolytopes,
                    opt_vars.hPolyIdx,
                    opt_vars.route_boundary_gates,
                    opt_vars.route_boundary_points,
                    opt_vars.route_boundary_radii,
                    opt_vars.headPVAJ,
                    opt_vars.tailPVAJ,
                    cfg_);
        if (immutable_minco_seed_certificate.valid ||
            !corridor_seed_result.valid) {
            deterministic_nominal_seed = immutable_nominal_seed;
            deterministic_seed_certificate = immutable_minco_seed_certificate;
            deterministic_seed_uses_corridor_bezier = false;
        } else {
            deterministic_nominal_seed = corridor_seed_result.trajectory;
            deterministic_seed_uses_corridor_bezier = true;
        }
    }
    diagnostics_.certified_seed_failure_stage =
            static_cast<int>(deterministic_seed_certificate.failure_stage);
    diagnostics_.certified_seed_maximum_velocity_mps =
            deterministic_seed_certificate.maximum_velocity_mps;
    diagnostics_.certified_seed_maximum_acceleration_mps2 =
            deterministic_seed_certificate.maximum_acceleration_mps2;
    diagnostics_.certified_seed_maximum_jerk_mps3 =
            deterministic_seed_certificate.maximum_jerk_mps3;

    const auto run_lbfgs = [&](const bool feasibility_retry) {
        ++diagnostics_.lbfgs_attempt_count;
        opt_vars.solver_attempt = diagnostics_.lbfgs_attempt_count;
        auto attempt_params = lbfgs_params;
        if (feasibility_retry) {
            // A cost plateau is not a feasibility certificate.  Retry mode
            // stops only at cancellation, the solve deadline, a line-search
            // result, or the explicit iteration bound below.
            attempt_params.past = 0;
            attempt_params.max_iterations =
                    cfg_.feasibility_retry_max_iterations;
        }
        const int result = lbfgs::lbfgs_optimize(x,
                                                  minCostFunctional,
                                                  &ExpTrajOpt::costFunctional,
                                                  nullptr,
                                                  &ExpTrajOpt::monitorProgress,
                                                  &this->opt_vars,
                                                  attempt_params);
        const int attempt_evaluation_count = std::max(0, opt_vars.iter_num);
        diagnostics_.lbfgs_evaluation_count += attempt_evaluation_count;
        if (diagnostics_.lbfgs_attempt_count == 1) {
            diagnostics_.lbfgs_first_attempt_evaluation_count =
                    attempt_evaluation_count;
        }
        diagnostics_.lbfgs_last_attempt_evaluation_count =
                attempt_evaluation_count;
        if (diagnostics_.lbfgs_attempt_count == 1) {
            diagnostics_.first_lbfgs_return_code = result;
        }
        diagnostics_.last_lbfgs_return_code = result;
        diagnostics_.cancelled = result == lbfgs::LBFGS_CANCELED;
        return result;
    };

    cout << std::fixed << std::setprecision(15);
    // only for debug
//    cout << " -- [ExpOpt] Start optimization." << x.transpose() << endl;
//    cout << " -- [ExpOpt] minCostFunctional: " << minCostFunctional << endl;
//    cout << " -- [ExpOpt] relCostTol: " << relCostTol << endl;
//    cout << " -- [ExpOpt] weightAtt: " << opt_vars.penaltyWeights(4) << endl;
//    cout << " -- [ExpOpt] waypoint_attractor: " << opt_vars.waypoint_attractor << endl;
//    cout << " -- [ExpOpt] waypoint_attractor_dead_d: " << opt_vars.waypoint_attractor_dead_d.transpose() << endl;
    // TimeConsuming ttt(" -- [ExpTrajOpt]", false);
    opt_vars.iter_num = 0;
    int ret = run_lbfgs(false);
    if (ret == lbfgs::LBFGS_CANCELED) {
        diagnostics_.retry_stop_reason = 2;
        diagnostics_.final_normalized_dynamic_violation =
                std::numeric_limits<double>::infinity();
        traj.clear();
        return INFINITY;
    }
    // double dt = ttt.stop();
    const auto rebuild_candidate = [&]() {
        gcopter::forwardMapTauToT(tau, opt_vars.times);
        if (opt_vars.duration_lower_bound.size() != 0 &&
            opt_vars.duration_lower_bound.size() != opt_vars.times.size()) {
            planner_context_->warn(
                    " -- [ExpOpt] duration lower-bound size mismatch: bound={} segments={}",
                    opt_vars.duration_lower_bound.size(), opt_vars.times.size());
            traj.clear();
            return;
        }
        if (opt_vars.duration_lower_bound.size() == opt_vars.times.size()) {
            opt_vars.times += opt_vars.duration_lower_bound;
        }
        switch (opt_vars.pos_constraint_type) {
            case 1: {
                VecDf xi_e = xi;
                opt_vars.points = Eigen::Map<Eigen::Matrix<double, 3, Eigen::Dynamic>>(xi_e.data(), 3, xi_e.size() / 3);
                break;
            }
            default: {
                gcopter::forwardP(xi, opt_vars.vPolyIdx,
                                  opt_vars.vPolytopes, opt_vars.points);
                break;
            }
        }
        if (!opt_vars.points.allFinite()) {
            traj.clear();
            return;
        }
        //        opt_vars.minco.setConditions(opt_vars.headPVAJ, opt_vars.tailPVAJ, opt_vars.temporalDim);
        opt_vars.minco.setParameters(opt_vars.points, opt_vars.times);
        traj.clear();
        opt_vars.minco.getTrajectory(traj);
    };
    const auto print_optimizer_result = [&]() {
        if (!cfg_.print_optimizer_log) return;
        cout << " -- [ExpOpt] Opt finish, with iter num: " << opt_vars.iter_num << "\n";
        cout << "\tEnergy: " << opt_vars.penalty_log(0) << endl;
        cout << "\tPos: " << opt_vars.penalty_log(1) << endl;
        cout << "\tVel: " << opt_vars.penalty_log(2) << endl;
        cout << "\tAcc: " << opt_vars.penalty_log(3) << endl;
        cout << "\tJerk: " << opt_vars.penalty_log(4) << endl;
        cout << "\tAttract: " << opt_vars.penalty_log(5) << endl;
        cout << "\tOmg: " << opt_vars.penalty_log(6) << endl;
        cout << "\tThr: " << opt_vars.penalty_log(7) << endl;
        cout << "\tOptimized Time: " << opt_vars.times.transpose() << endl;
    };
    const auto corridor_plane_violation = [&]() {
        if (traj.empty() || opt_vars.hPolyIdx.size() != traj.getPieceNum() ||
            opt_vars.hPolytopes.empty()) {
            return std::numeric_limits<double>::infinity();
        }
        double maximum_violation = -std::numeric_limits<double>::infinity();
        for (int piece_index = 0; piece_index < traj.getPieceNum(); ++piece_index) {
            const int polytope_index = opt_vars.hPolyIdx(piece_index);
            if (polytope_index < 0 ||
                polytope_index >= static_cast<int>(opt_vars.hPolytopes.size())) {
                return std::numeric_limits<double>::infinity();
            }
            const auto &polytope = opt_vars.hPolytopes[polytope_index];
            maximum_violation = std::max(
                maximum_violation,
                navigation_planning_backend::maximumContinuousCorridorPlaneViolation(
                    traj[piece_index], polytope));
        }
        return maximum_violation;
    };
    const auto route_boundary_satisfied = [&]() {
        if (opt_vars.route_boundary_gates.size() != opt_vars.hPolytopes.size() ||
            opt_vars.route_boundary_points.size() != opt_vars.hPolytopes.size() ||
            opt_vars.route_boundary_radii.size() != opt_vars.hPolytopes.size()) {
            return false;
        }
        const int piece_count = traj.getPieceNum();
        if (piece_count <= 0) return false;
        for (std::size_t gate_index = 0;
             gate_index < opt_vars.route_boundary_gates.size(); ++gate_index) {
            if (opt_vars.route_boundary_gates[gate_index] == 0U) continue;
            const Vec3f &boundary_point = opt_vars.route_boundary_points[gate_index];
            const double boundary_radius = opt_vars.route_boundary_radii[gate_index];
            if (!boundary_point.allFinite() || !std::isfinite(boundary_radius) ||
                boundary_radius <= 0.0) {
                return false;
            }
            bool reached = false;
            const int gate_cell = static_cast<int>(gate_index);
            const int first_junction = gate_cell - 1;
            const int last_junction = gate_cell;
            for (const int junction_index : {first_junction, last_junction}) {
                if (junction_index < 0 || junction_index >= piece_count) continue;
                const Eigen::Vector3d junction = traj.getJuncPos(junction_index);
                const double distance =
                    (junction - boundary_point.cast<double>()).norm();
                if (std::isfinite(distance) && distance <= boundary_radius + 1.0e-6) {
                    reached = true;
                    break;
                }
            }
            if (!reached) return false;
        }
        return true;
    };
    const auto position_constraint_satisfied = [&]() {
        // Corridor planes are a hard safety contract.  The integrated L-BFGS
        // penalty is only a search aid and must not authorize an excursion
        // outside an SFC. Evaluate the certificate independently so disabling
        // The objective position penalty cannot disable the gate.
        const double corridor_plane_tolerance = cfg_.corridor_plane_tolerance_m;
        const double maximum_violation = corridor_plane_violation();
        return std::isfinite(maximum_violation) &&
               maximum_violation <= corridor_plane_tolerance &&
               route_boundary_satisfied();
    };
    double maximum_velocity = std::numeric_limits<double>::infinity();
    double maximum_acceleration = std::numeric_limits<double>::infinity();
    double maximum_jerk = std::numeric_limits<double>::infinity();
    const auto update_dynamic_extrema = [&]() {
        maximum_velocity = traj.empty() ? std::numeric_limits<double>::infinity()
                                        : traj.getMaxVelRate();
        maximum_acceleration = traj.empty() ? std::numeric_limits<double>::infinity()
                                            : traj.getMaxAccRate();
        maximum_jerk = traj.empty() ? std::numeric_limits<double>::infinity()
                                    : traj.getMaxJerRate();
        diagnostics_.last_candidate_maximum_velocity_mps = maximum_velocity;
        diagnostics_.last_candidate_maximum_acceleration_mps2 = maximum_acceleration;
        diagnostics_.last_candidate_maximum_jerk_mps3 = maximum_jerk;
    };
    const auto normalized_dynamic_violation = [&]() {
        if (!std::isfinite(maximum_velocity) || !std::isfinite(maximum_acceleration) ||
            !std::isfinite(maximum_jerk)) {
            return std::numeric_limits<double>::infinity();
        }
        const auto velocity_recovery =
                navigation_planning_backend::certifyBoundaryVelocityRecovery(
                    traj, cfg_.max_vel, cfg_.max_acc, cfg_.max_jerk);
        const double peak_velocity_ratio = velocity_recovery.finite
                ? maximum_velocity / velocity_recovery.allowed_peak_speed_mps
                : std::numeric_limits<double>::infinity();
        const double recovered_velocity_ratio = velocity_recovery.initial_overspeed
                ? velocity_recovery.suffix_maximum_speed_mps / cfg_.max_vel
                : maximum_velocity / cfg_.max_vel;
        return std::max({peak_velocity_ratio, recovered_velocity_ratio,
                         maximum_acceleration / cfg_.max_acc,
                         maximum_jerk / cfg_.max_jerk});
    };
    const auto dynamic_gate_satisfied = [&]() {
        const auto velocity_recovery =
                navigation_planning_backend::certifyBoundaryVelocityRecovery(
                    traj, cfg_.max_vel, cfg_.max_acc, cfg_.max_jerk);
        return velocity_recovery.satisfied &&
               std::isfinite(maximum_velocity) &&
               std::isfinite(maximum_acceleration) &&
               std::isfinite(maximum_jerk) &&
               navigation_planning::withinNumericalDynamicLimit(
                   maximum_acceleration, cfg_.max_acc) &&
               navigation_planning::withinNumericalDynamicLimit(
                   maximum_jerk, cfg_.max_jerk);
    };
    const auto acceptFiniteLineSearchCandidate = [&]() {
        // L-BFGS can exhaust its line-search trials after leaving a finite,
        // already usable iterate.  Solver convergence is not a safety
        // certificate, so accept this path only after rebuilding the
        // candidate and re-running the independent corridor and V/A/J gates.
        if (ret != lbfgs::LBFGSERR_MAXIMUMLINESEARCH || !x.allFinite()) {
            return false;
        }
        rebuild_candidate();
        update_dynamic_extrema();
        if (!position_constraint_satisfied() || !dynamic_gate_satisfied()) {
            traj.clear();
            return false;
        }
        planner_context_->warn(
                " -- [ExpOpt] accepted finite candidate after line-search stop: "
                "vel={}/{} acc={}/{} jerk={}/{}",
                maximum_velocity, cfg_.max_vel,
                maximum_acceleration, cfg_.max_acc,
                maximum_jerk, cfg_.max_jerk);
        ret = lbfgs::LBFGS_STOP;
        return true;
    };

    struct CandidateSnapshot {
        VecDf x;
        VecDf duration_lower_bound;
        VecDf penalty_weights;
        VecDf penalty_log;
        double objective{std::numeric_limits<double>::infinity()};
        int iteration{0};
        bool valid{false};
    };

    if (ret >= 0) {
        rebuild_candidate();
        update_dynamic_extrema();
    } else {
        // A line-search stop is a numerical termination, not proof that the
        // last finite iterate is unusable.  The helper still requires the
        // independent hard gates before changing the solver result to a
        // successful bounded stop.
        (void)acceptFiniteLineSearchCandidate();
    }
    const VecDf nominal_duration_s = opt_vars.times;
    print_optimizer_result();

    if (ret >= 0 && !position_constraint_satisfied()) {
        const double rejected_candidate_violation = corridor_plane_violation();
        planner_context_->warn(
                " -- [ExpOpt] hard corridor gate rejected trajectory: cost={} limit={} lbfgs_ret={}",
                rejected_candidate_violation,
                cfg_.corridor_plane_tolerance_m, ret);
        const auto selection = navigation_planning_backend::selectNominalCandidate(
                traj, false, deterministic_nominal_seed,
                deterministic_seed_certificate, traj);
        if (selection == navigation_planning_backend::
                NominalCandidateSelection::kCertifiedSeed) {
            update_dynamic_extrema();
            planner_context_->warn(
                    " -- [ExpOpt] selected exact pre-LBFGS certified seed: "
                    "optimized_corridor_violation={} seed_corridor_violation={} "
                    "seed_boundary_residual={} seed_boundary_roundoff_bound={}",
                    rejected_candidate_violation,
                    deterministic_seed_certificate.maximum_corridor_violation_m,
                    deterministic_seed_certificate.maximum_boundary_residual,
                    deterministic_seed_certificate.maximum_boundary_roundoff_bound);
            ret = lbfgs::LBFGS_STOP;
        } else {
            const auto &flatness = deterministic_seed_certificate.flatness_report;
            planner_context_->warn(
                    " -- [ExpOpt] immutable pre-LBFGS seed unavailable: "
                    "stage={} corridor_violation={} boundary_residual={} "
                    "boundary_roundoff_bound={} boundary_failure={}/{}/{}/{} "
                    "boundary_value={}/{} "
                    "vel={}/{} acc={}/{} jerk={}/{} flatness_finite={} "
                    "body_rate={}/{} thrust=[{},{}]/[{},{}]",
                    static_cast<int>(deterministic_seed_certificate.failure_stage),
                    deterministic_seed_certificate.maximum_corridor_violation_m,
                    deterministic_seed_certificate.maximum_boundary_residual,
                    deterministic_seed_certificate.maximum_boundary_roundoff_bound,
                    deterministic_seed_certificate.boundary_failure_location,
                    deterministic_seed_certificate.boundary_failure_piece_index,
                    deterministic_seed_certificate.boundary_failure_axis,
                    deterministic_seed_certificate.boundary_failure_derivative,
                    deterministic_seed_certificate.boundary_failure_residual,
                    deterministic_seed_certificate.boundary_failure_roundoff_bound,
                    deterministic_seed_certificate.maximum_velocity_mps, cfg_.max_vel,
                    deterministic_seed_certificate.maximum_acceleration_mps2, cfg_.max_acc,
                    deterministic_seed_certificate.maximum_jerk_mps3, cfg_.max_jerk,
                    flatness.finite,
                    flatness.maximum_body_rate_rad_s, cfg_.max_omg,
                    flatness.minimum_thrust_n, flatness.maximum_thrust_n,
                    cfg_.min_acc_thr * cfg_.mass,
                    cfg_.max_acc_thr * cfg_.mass);
            ret = -1;
        }
    }

    // A fixed-point MINCO rebuild is not a valid time projection when the
    // boundary PVAJ is non-zero: increasing time can increase acceleration or
    // jerk. Re-run the full optimization so both spatial variables and segment
    // times can move inside the same corridor. The hard gate remains the only
    // authority and the retry budget is deliberately finite.
    constexpr int kMaximumFeasibilityRetries = 2;
    constexpr double kFeasibilityPenaltyGrowth = 10.0;
    constexpr double kFeasibilityTimeReserve = 1.05;
    double best_normalized_violation = normalized_dynamic_violation();
    diagnostics_.initial_normalized_dynamic_violation = best_normalized_violation;
    diagnostics_.best_normalized_dynamic_violation = best_normalized_violation;
    CandidateSnapshot best_candidate;
    const auto capture_best_candidate = [&]() {
        best_candidate.x = x;
        best_candidate.duration_lower_bound = opt_vars.duration_lower_bound;
        best_candidate.penalty_weights = opt_vars.penaltyWeights;
        best_candidate.penalty_log = opt_vars.penalty_log;
        best_candidate.objective = minCostFunctional;
        best_candidate.iteration = opt_vars.iter_num;
        best_candidate.valid = true;
    };
    const auto restore_best_candidate = [&]() {
        if (!best_candidate.valid) {
            return;
        }
        x = best_candidate.x;
        opt_vars.duration_lower_bound = best_candidate.duration_lower_bound;
        opt_vars.penaltyWeights = best_candidate.penalty_weights;
        opt_vars.penalty_log = best_candidate.penalty_log;
        minCostFunctional = best_candidate.objective;
        opt_vars.iter_num = best_candidate.iteration;
        rebuild_candidate();
        update_dynamic_extrema();
    };
    capture_best_candidate();
    // Keep the first finite candidate separately from later feasibility
    // retries. A retry is allowed to move spatial variables, and if that
    // move leaves the corridor we still need the last corridor-valid seed for
    // the bounded temporal fallback below.
    const CandidateSnapshot feasibility_seed_candidate = best_candidate;
    const auto rebuildInitialCandidate = [&]() {
        if (opt_vars.init_ts.size() == 0 ||
            opt_vars.init_ts.size() != opt_vars.times.size() ||
            opt_vars.init_ps.size() != static_cast<std::size_t>(opt_vars.points.cols())) {
            return false;
        }
        opt_vars.duration_lower_bound.resize(0);
        gcopter::backwardMapTToTau(opt_vars.init_ts, tau);
        Mat3Df initial_points(3, opt_vars.init_ps.size());
        for (std::size_t index = 0; index < opt_vars.init_ps.size(); ++index) {
            if (!opt_vars.init_ps[index].allFinite()) return false;
            initial_points.col(static_cast<Eigen::Index>(index)) = opt_vars.init_ps[index];
        }
        switch (opt_vars.pos_constraint_type) {
            case 1: {
                xi = Eigen::Map<const VecDf>(initial_points.data(), initial_points.size());
                break;
            }
            default: {
                gcopter::backwardP(initial_points, opt_vars.vPolyIdx,
                                   opt_vars.vPolytopes, xi);
                break;
            }
        }
        if (!x.allFinite()) return false;
        rebuild_candidate();
        update_dynamic_extrema();
        return !traj.empty();
    };
    const auto rebuildFeasibilitySeedCandidate = [&]() {
        if (!feasibility_seed_candidate.valid ||
            !feasibility_seed_candidate.x.allFinite()) {
            return false;
        }
        x = feasibility_seed_candidate.x;
        opt_vars.duration_lower_bound =
                feasibility_seed_candidate.duration_lower_bound;
        rebuild_candidate();
        update_dynamic_extrema();
        return !traj.empty() && position_constraint_satisfied();
    };
    const auto tryBoundedTimeStretch = [&]() {
        // If the guide seed is geometrically valid but too fast, preserve its
        // collision-checked spatial path and stretch only its durations. This
        // is deterministic and bounded; it avoids a feasibility retry moving
        // the points outside the already certified corridor before the dynamic
        // gate can be evaluated.
        // Prefer the last corridor-valid optimized seed. The original guide
        // remains a fallback for a numerical line-search stop that did not
        // produce a corridor-valid optimized iterate.
        if (!rebuildFeasibilitySeedCandidate() &&
            (!rebuildInitialCandidate() || !position_constraint_satisfied())) {
            planner_context_->warn(
                    " -- [ExpOpt] bounded time stretch skipped: no corridor-valid finite seed");
            return false;
        }
        const double velocity_scale = maximum_velocity / cfg_.max_vel;
        const double acceleration_scale = std::sqrt(maximum_acceleration / cfg_.max_acc);
        const double jerk_scale = std::cbrt(maximum_jerk / cfg_.max_jerk);
        const double required_scale = std::max({
                1.0, velocity_scale, acceleration_scale, jerk_scale});
        if (!std::isfinite(required_scale) || required_scale <= 1.0 ||
            !nominal_duration_s.allFinite() || nominal_duration_s.size() == 0 ||
            nominal_duration_s.minCoeff() <= 0.0) {
            return false;
        }
        const double initial_duration_reserve_scale = std::min(
                4.0, std::max(kFeasibilityTimeReserve, required_scale * 1.05));
        if (!std::isfinite(initial_duration_reserve_scale) ||
            initial_duration_reserve_scale <= 1.0) {
            planner_context_->warn(
                    " -- [ExpOpt] bounded time stretch skipped: invalid reserve scale={} "
                    "required_scale={}",
                    initial_duration_reserve_scale, required_scale);
            return false;
        }
        // A single scale derived from the initial peak can still leave a
        // fixed-boundary MINCO polynomial marginally outside the envelope.
        // Try only two larger, deterministic reserves; never relax the hard
        // certificate and never allow an unbounded retry loop.
        const std::array<double, 3> duration_reserve_scales = {
                initial_duration_reserve_scale,
                std::min(4.0, initial_duration_reserve_scale * 1.5),
                4.0};
        for (const double duration_reserve_scale : duration_reserve_scales) {
            const VecDf reserved_duration_s = nominal_duration_s * duration_reserve_scale;
            const VecDf free_duration_seed_s = nominal_duration_s *
                    (duration_reserve_scale - 1.0);
            if (!reserved_duration_s.allFinite() || reserved_duration_s.size() == 0 ||
                reserved_duration_s.minCoeff() <= 0.0 || !free_duration_seed_s.allFinite() ||
                free_duration_seed_s.size() == 0 || free_duration_seed_s.minCoeff() <= 0.0) {
                continue;
            }
            if (!rebuildFeasibilitySeedCandidate() &&
                (!rebuildInitialCandidate() || !position_constraint_satisfied())) {
                planner_context_->warn(
                        " -- [ExpOpt] bounded time stretch candidate={} skipped: "
                        "seed left corridor",
                        duration_reserve_scale);
                continue;
            }
            // This branch accepts the already-certified deterministic guide
            // directly; the requested reserve is therefore the complete
            // duration, not an additional lower bound plus another reserve.
            opt_vars.duration_lower_bound.resize(0);
            gcopter::backwardMapTToTau(reserved_duration_s, tau);
            if (!x.allFinite()) continue;
            rebuild_candidate();
            update_dynamic_extrema();
            if (!position_constraint_satisfied()) {
                planner_context_->warn(
                        " -- [ExpOpt] bounded time stretch candidate={} rejected: "
                        "corridor_violation={}",
                        duration_reserve_scale, corridor_plane_violation());
                continue;
            }
            if (!dynamic_gate_satisfied()) {
                planner_context_->warn(
                        " -- [ExpOpt] bounded time stretch candidate={} rejected: "
                        "vel={} acc={} jerk={} limits={}/{}/{}",
                        duration_reserve_scale, maximum_velocity,
                        maximum_acceleration, maximum_jerk,
                        cfg_.max_vel,
                        cfg_.max_acc,
                        cfg_.max_jerk);
                continue;
            }
            diagnostics_.retry_duration_lower_bound_min_s = reserved_duration_s.minCoeff();
            diagnostics_.retry_duration_lower_bound_max_s = reserved_duration_s.maxCoeff();
            diagnostics_.retry_free_duration_seed_min_s = free_duration_seed_s.minCoeff();
            diagnostics_.retry_free_duration_seed_max_s = free_duration_seed_s.maxCoeff();
            planner_context_->warn(
                    " -- [ExpOpt] accepted bounded time-stretched guide: "
                    "scale={} vel={}/{} acc={}/{} jerk={}/{}",
                    duration_reserve_scale,
                    maximum_velocity, cfg_.max_vel,
                    maximum_acceleration, cfg_.max_acc,
                    maximum_jerk, cfg_.max_jerk);
            ret = lbfgs::LBFGS_STOP;
            return true;
        }
        return false;
    };
    const bool initial_bounded_solver_stop =
            ret == lbfgs::LBFGSERR_MAXIMUMLINESEARCH ||
            ret == lbfgs::LBFGSERR_MAXIMUMITERATION;
    if (!dynamic_gate_satisfied() &&
        (ret >= 0 || initial_bounded_solver_stop)) {
        if (tryBoundedTimeStretch()) {
            capture_best_candidate();
        } else {
            restore_best_candidate();
        }
    }
    const VecDf nominal_penalty_weights = opt_vars.penaltyWeights;
    for (int retry = 0;
         ret >= 0 && !dynamic_gate_satisfied() &&
         retry < kMaximumFeasibilityRetries; ++retry) {
        const double velocity_scale = maximum_velocity / cfg_.max_vel;
        const double acceleration_scale = std::sqrt(maximum_acceleration / cfg_.max_acc);
        const double jerk_scale = std::cbrt(maximum_jerk / cfg_.max_jerk);
        const double required_scale = std::max({
                1.0, velocity_scale, acceleration_scale, jerk_scale});
        if (!std::isfinite(required_scale)) {
            diagnostics_.retry_stop_reason = 1;
            break;
        }
        const bool velocity_violated =
                maximum_velocity > cfg_.max_vel;
        const bool acceleration_violated =
                maximum_acceleration > cfg_.max_acc;
        const bool jerk_violated =
                maximum_jerk > cfg_.max_jerk;
        diagnostics_.retry_violation_mask |=
                (velocity_violated ? 1 : 0) |
                (acceleration_violated ? 2 : 0) |
                (jerk_violated ? 4 : 0);
        ++diagnostics_.retry_count;
        if (opt_vars.steady_deadline_ns > 0) {
            const auto now_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
                    std::chrono::steady_clock::now().time_since_epoch()).count();
            diagnostics_.retry_budget_remaining_us = std::max<std::int64_t>(
                    0, (opt_vars.steady_deadline_ns - now_ns) / 1000);
        }

        const double duration_reserve_scale = std::min(
                4.0, std::max(kFeasibilityTimeReserve, required_scale * 1.05));
        if (!nominal_duration_s.allFinite() || nominal_duration_s.size() == 0 ||
            nominal_duration_s.minCoeff() <= 0.0 ||
            !std::isfinite(duration_reserve_scale) ||
            duration_reserve_scale <= 1.0) {
            diagnostics_.retry_stop_reason = 1;
            break;
        }
        // Apply the reserve to the actual lower bound, not only to the initial
        // free-duration seed.  The objective contains a time term and is
        // allowed to drive the free duration back toward zero; keeping the
        // old nominal lower bound therefore allowed the retry to return the
        // same slightly overspeed trajectory.  The fixed nominal reference
        // prevents compounding reserves across retries while the hard gate
        // below remains authoritative for every dynamic component.
        const VecDf reserved_duration_s = nominal_duration_s * duration_reserve_scale;
        if (!reserved_duration_s.allFinite() || reserved_duration_s.size() == 0 ||
            reserved_duration_s.minCoeff() <= 0.0) {
            diagnostics_.retry_stop_reason = 1;
            break;
        }
        opt_vars.duration_lower_bound = reserved_duration_s;
        // forwardMapTauToT() returns free duration and the objective adds the
        // lower bound.  Seed only a tiny positive slack so the actual initial
        // duration remains the requested reserve instead of adding the
        // reserve a second time.
        const VecDf free_duration_seed_s = nominal_duration_s.unaryExpr(
                [](const double duration_s) {
                    return std::max(1.0e-9, duration_s * 1.0e-6);
                });
        if (!free_duration_seed_s.allFinite() || free_duration_seed_s.size() == 0 ||
            free_duration_seed_s.minCoeff() <= 0.0) {
            diagnostics_.retry_stop_reason = 1;
            break;
        }
        diagnostics_.retry_duration_lower_bound_min_s =
                reserved_duration_s.minCoeff();
        diagnostics_.retry_duration_lower_bound_max_s =
                reserved_duration_s.maxCoeff();
        diagnostics_.retry_free_duration_seed_min_s =
                free_duration_seed_s.minCoeff();
        diagnostics_.retry_free_duration_seed_max_s =
                free_duration_seed_s.maxCoeff();
        gcopter::backwardMapTToTau(free_duration_seed_s, tau);
        // Continue the soft solve toward the hard-feasible set. A disabled
        // nominal shaping penalty remains disabled during ordinary solves,
        // but a violated hard constraint needs a gradient during this bounded
        // feasibility-only retry. Reuse the configured translational dynamic
        // scale rather than introducing another flight-tuned number.
        // The first retry is primarily a bounded time/point re-optimization.
        // Preserve the nominal objective so a tiny hard-gate violation does
        // not immediately turn into a much slower trajectory.  Only the
        // second and final retry strengthens the violated configured penalty.
        const double penalty_scale = std::pow(kFeasibilityPenaltyGrowth, retry);
        const std::array<bool, 3> violated = {
                velocity_violated, acceleration_violated, jerk_violated};
        for (int index = 1; index <= 3; ++index) {
            if (violated[static_cast<std::size_t>(index - 1)]) {
                const double feasibility_weight = feasibilityRetryPenaltyWeight(
                        nominal_penalty_weights, index);
                opt_vars.penaltyWeights(index) =
                        std::isfinite(feasibility_weight)
                        ? feasibility_weight * penalty_scale
                        : nominal_penalty_weights(index);
            } else {
                opt_vars.penaltyWeights(index) = nominal_penalty_weights(index);
            }
        }
        opt_vars.route_reference_points = route_reference_points;
        opt_vars.penalty_log.setZero();
        opt_vars.iter_num = 0;
        const int retry_result = run_lbfgs(true);
        if (retry_result == lbfgs::LBFGS_CANCELED) {
            diagnostics_.retry_stop_reason = 2;
            diagnostics_.final_normalized_dynamic_violation =
                    std::numeric_limits<double>::infinity();
            restore_best_candidate();
            traj.clear();
            return INFINITY;
        }
        // Hitting the explicit iteration bound yields a finite candidate that
        // still requires every hard gate.  Keep the raw solver result in
        // diagnostics, but let the validation path inspect that candidate.
        ret = retry_result == lbfgs::LBFGSERR_MAXIMUMITERATION
                ? lbfgs::LBFGS_STOP
                : retry_result;
        if (ret < 0) {
            if (retry_result == lbfgs::LBFGSERR_MAXIMUMLINESEARCH &&
                x.allFinite()) {
                ret = retry_result;
                if (acceptFiniteLineSearchCandidate()) {
                    capture_best_candidate();
                    break;
                }
            }
            diagnostics_.retry_stop_reason = 3;
            restore_best_candidate();
            break;
        }

        rebuild_candidate();
        update_dynamic_extrema();
        if (!position_constraint_satisfied()) {
            planner_context_->warn(
                    " -- [ExpOpt] feasibility retry left corridor: attempt={} cost={}",
                    retry + 1, corridor_plane_violation());
            diagnostics_.retry_stop_reason = 4;
            restore_best_candidate();
            ret = -1;
            break;
        }
        const double retry_violation = normalized_dynamic_violation();
        planner_context_->info(
                " -- [ExpOpt] bounded feasibility retry attempt={} violation={} "
                "penalty_scale={} vel={} acc={} jerk={}",
                retry + 1, retry_violation,
                penalty_scale,
                maximum_velocity, maximum_acceleration, maximum_jerk);
        const bool candidate_is_feasible = dynamic_gate_satisfied();
        const bool candidate_made_progress = std::isfinite(retry_violation) &&
                                             retry_violation < best_normalized_violation;
        if (!candidate_is_feasible && !candidate_made_progress) {
            planner_context_->warn(
                    " -- [ExpOpt] feasibility retry made no progress: previous={} current={}",
                    best_normalized_violation, retry_violation);
            diagnostics_.retry_stop_reason = 5;
            restore_best_candidate();
            break;
        }
        best_normalized_violation = retry_violation;
        diagnostics_.best_normalized_dynamic_violation = best_normalized_violation;
        capture_best_candidate();
        if (candidate_is_feasible) {
            break;
        }
    }
    if (ret >= 0 && !dynamic_gate_satisfied() &&
        diagnostics_.retry_count >= kMaximumFeasibilityRetries &&
        diagnostics_.retry_stop_reason == 0) {
        diagnostics_.retry_stop_reason = 6;
    }
    diagnostics_.final_normalized_dynamic_violation =
            normalized_dynamic_violation();
    diagnostics_.nonfinite_evaluation_count = opt_vars.nonfinite_evaluation_count;
    diagnostics_.first_nonfinite_stage = opt_vars.first_nonfinite_stage;
    diagnostics_.first_nonfinite_value_mask =
            opt_vars.first_nonfinite_value_mask;
    diagnostics_.first_nonfinite_attempt = opt_vars.first_nonfinite_attempt;
    diagnostics_.first_nonfinite_iteration = opt_vars.first_nonfinite_iteration;
    diagnostics_.first_nonfinite_min_duration_s =
            opt_vars.first_nonfinite_min_duration_s;
    diagnostics_.first_nonfinite_max_duration_s =
            opt_vars.first_nonfinite_max_duration_s;
    diagnostics_.first_nonfinite_cost = opt_vars.first_nonfinite_cost;
    diagnostics_.first_nonfinite_gradient_norm =
            opt_vars.first_nonfinite_gradient_norm;
    diagnostics_.final_duration_s = std::numeric_limits<double>::quiet_NaN();
    opt_vars.penaltyWeights = nominal_penalty_weights;

    if (ret >= 0) {
        // Mission V/A/J values are command limits, not soft optimizer
        // penalties. The hard gate is the physical mission/product envelope.
        const auto velocity_recovery =
                navigation_planning_backend::certifyBoundaryVelocityRecovery(
                    traj, cfg_.max_vel, cfg_.max_acc, cfg_.max_jerk);
        if (!velocity_recovery.satisfied ||
            !std::isfinite(maximum_velocity) || !std::isfinite(maximum_acceleration) ||
            !std::isfinite(maximum_jerk) ||
            !navigation_planning::withinNumericalDynamicLimit(
                maximum_acceleration, cfg_.max_acc) ||
            !navigation_planning::withinNumericalDynamicLimit(
                maximum_jerk, cfg_.max_jerk)) {
            planner_context_->warn(
                    " -- [ExpOpt] physical hard gate rejected trajectory: "
                    "vel={}/{} initial_vel={} allowed_peak={} recovery_deadline={} "
                    "recovery_suffix_max={} acc={}/{} jerk={}/{}",
                    maximum_velocity, cfg_.max_vel,
                    velocity_recovery.initial_speed_mps,
                    velocity_recovery.allowed_peak_speed_mps,
                    velocity_recovery.recovery_deadline_s,
                    velocity_recovery.suffix_maximum_speed_mps,
                    maximum_acceleration, cfg_.max_acc,
                    maximum_jerk, cfg_.max_jerk);
            traj.clear();
            ret = -1;
            minCostFunctional = INFINITY;
        }
        if (ret >= 0) {
            TrajectoryDynamicReport dynamic_report;
            if (!trajectorySatisfiesFlatnessEnvelope(traj, cfg_, &dynamic_report)) {
                planner_context_->warn(
                        " -- [ExpOpt] flatness hard gate rejected trajectory: "
                        "finite={} body_rate={}/{} thrust=[{},{}]/[{},{}]",
                        dynamic_report.finite,
                        dynamic_report.maximum_body_rate_rad_s,
                        cfg_.max_omg,
                        dynamic_report.minimum_thrust_n,
                        dynamic_report.maximum_thrust_n,
                        cfg_.min_acc_thr * cfg_.mass,
                        cfg_.max_acc_thr * cfg_.mass);
                traj.clear();
                ret = -1;
                minCostFunctional = INFINITY;
            }
        }
    }

    // MINCO is a quality refinement, not the sole owner of nominal command
    // availability. If it terminates numerically or its final hard gates
    // reject the optimized iterate, copy only the immutable pre-LBFGS seed
    // that was independently certified before any optimizer mutation.
    if (ret < 0 && !diagnostics_.cancelled) {
        const auto selection = navigation_planning_backend::selectNominalCandidate(
                traj, false, deterministic_nominal_seed,
                deterministic_seed_certificate, traj);
        if (selection == navigation_planning_backend::
                NominalCandidateSelection::kCertifiedSeed) {
            update_dynamic_extrema();
            diagnostics_.used_certified_seed = true;
            diagnostics_.final_normalized_dynamic_violation =
                    normalized_dynamic_violation();
            diagnostics_.final_duration_s = traj.getTotalDuration();
            minCostFunctional = 0.0;
            ret = lbfgs::LBFGS_STOP;
            planner_context_->warn(
                    " -- [ExpOpt] MINCO refinement unavailable; selected exact "
                    "pre-LBFGS certified seed: source={} duration={} "
                    "duration_scale={} corridor_violation={} "
                    "boundary_residual={} boundary_roundoff_bound={} vel={}/{} "
                    "acc={}/{} jerk={}/{}",
                    deterministic_seed_uses_corridor_bezier
                        ? "corridor_bezier" : "minco_interpolation",
                    diagnostics_.final_duration_s,
                    deterministic_seed_duration_scale,
                    deterministic_seed_certificate.maximum_corridor_violation_m,
                    deterministic_seed_certificate.maximum_boundary_residual,
                    deterministic_seed_certificate.maximum_boundary_roundoff_bound,
                    maximum_velocity, cfg_.max_vel,
                    maximum_acceleration, cfg_.max_acc,
                    maximum_jerk, cfg_.max_jerk);
        }
    }

    if (ret >= 0 && !traj.empty()) {
        diagnostics_.final_duration_s = traj.getTotalDuration();
    } else {
        traj.clear();
        minCostFunctional = INFINITY;
        const auto &flatness = deterministic_seed_certificate.flatness_report;
        planner_context_->warn(
                " -- [ExpOpt] MINCO and immutable seed unavailable: "
                "solver={} seed_stage={} corridor_violation={} "
                "boundary_residual={} boundary_roundoff_bound={} "
                "boundary_failure={}/{}/{}/{} boundary_value={}/{} "
                "vel={}/{} acc={}/{} jerk={}/{} "
                "flatness_finite={} body_rate={}/{} thrust=[{},{}]/[{},{}]",
                ret,
                static_cast<int>(deterministic_seed_certificate.failure_stage),
                deterministic_seed_certificate.maximum_corridor_violation_m,
                deterministic_seed_certificate.maximum_boundary_residual,
                deterministic_seed_certificate.maximum_boundary_roundoff_bound,
                deterministic_seed_certificate.boundary_failure_location,
                deterministic_seed_certificate.boundary_failure_piece_index,
                deterministic_seed_certificate.boundary_failure_axis,
                deterministic_seed_certificate.boundary_failure_derivative,
                deterministic_seed_certificate.boundary_failure_residual,
                deterministic_seed_certificate.boundary_failure_roundoff_bound,
                deterministic_seed_certificate.maximum_velocity_mps, cfg_.max_vel,
                deterministic_seed_certificate.maximum_acceleration_mps2, cfg_.max_acc,
                deterministic_seed_certificate.maximum_jerk_mps3, cfg_.max_jerk,
                flatness.finite,
                flatness.maximum_body_rate_rad_s, cfg_.max_omg,
                flatness.minimum_thrust_n, flatness.maximum_thrust_n,
                cfg_.min_acc_thr * cfg_.mass,
                cfg_.max_acc_thr * cfg_.mass);
        cout << YELLOW << " -- [MINCO] TrajOpt failed, " << lbfgs::lbfgs_strerror(ret) << RESET << endl;
//        cout << "Init times: " << times_init.transpose() << endl;
    }
    return minCostFunctional + ret;
}

ExpTrajOpt::ExpTrajOpt(const traj_opt::Config &cfg, const navigation_planner_context::PlannerRuntimeContext::Ptr &planner_context) :
        cfg_(cfg),
        planner_context_(planner_context) {
    cfg_.validate(false);
    if (!planner_context_) {
        throw std::invalid_argument("ExpTrajOpt requires a planner runtime context");
    }
    /// Use time as log file name
    //    auto now = std::chrono::system_clock::now();
    //    std::time_t t = std::chrono::system_clock::to_time_t(now);
    //    std::tm tm = *std::localtime(&t);
    //    std::stringstream ss;
    //    ss << std::put_time(&tm, "%Y-%m-%d_%H-%M-%S");
    //    std::string filename = ss.str() + "_exp_opt_log.csv";
    if(cfg_.save_log_en){
        std::string filename = "exp_opt_log.csv";
        failed_traj_log.open(NAVIGATION_PLANNER_DEBUG_FILE_DIR(filename), std::ios::out | std::ios::trunc);
        penalty_log.open(NAVIGATION_PLANNER_DEBUG_FILE_DIR("nominal_opt_objective.csv"), std::ios::out | std::ios::trunc);
    }

    opt_vars.magnitudeBounds.resize(6);
    opt_vars.penaltyWeights.resize(7);
    opt_vars.magnitudeBounds << cfg_.max_vel, cfg_.max_acc, cfg_.max_jerk,
            cfg_.max_omg, cfg_.min_acc_thr * cfg_.mass, cfg_.max_acc_thr * cfg_.mass;
    opt_vars.penaltyWeights << cfg_.position_penalty_weight,
            cfg_.velocity_penalty_weight, cfg_.acceleration_penalty_weight,
            cfg_.jerk_penalty_weight, cfg_.waypoint_attraction_weight,
            cfg_.angular_rate_penalty_weight, cfg_.thrust_penalty_weight;
    opt_vars.rho = cfg_.time_weight;
    opt_vars.pos_constraint_type = cfg_.pos_constraint_type;
    opt_vars.block_energy_cost = cfg_.block_energy_cost;
    opt_vars.smooth_eps = cfg_.smooth_eps;
    opt_vars.integral_res = cfg_.integral_reso;
    opt_vars.quadrotor_flatness = cfg_.quadrotor_flatness;
    opt_vars.route_reference_lateral_weight = cfg_.route_reference_lateral_weight;
    opt_vars.route_reference_vertical_weight = cfg_.route_reference_vertical_weight;
    opt_vars.route_reference_lateral_deadband_m = cfg_.route_reference_lateral_deadband_m;
    opt_vars.route_reference_vertical_deadband_m = cfg_.route_reference_vertical_deadband_m;
}

ExpTrajOpt::~ExpTrajOpt() {
    failed_traj_log.close();
    penalty_log.close();
}


//bool ExpTrajOpt::optimize(const StatePVAJ &headPVAJ, const StatePVAJ &tailPVAJ,
//                          PolytopeVec &sfcs,
//                          Trajectory &out_traj) {
//    /// Check if SFC is valid
//    if (sfcs.empty()) {
//        cout << YELLOW << " -- [TrajOpt] Error, the SFC is empty." << RESET << endl;
//        return false;
//    }
//
//    if (!SimplifySFC(headPVAJ.col(0), tailPVAJ.col(0), sfcs)) {
//        cout << YELLOW << " -- [TrajOpt] Cannot simplify sfcs." << RESET << endl;
//        //        VisualUtils::VisualizePoint(mkr_pub_, headPVAJ.col(0),Color::Pink(),"ill_start",0.5,1);
//        //        VisualUtils::VisualizePoint(mkr_pub_, tailPVAJ.col(0),Color::Pink(),"ill_end",0.5,2);
//        //        cout << "headPVAJ: " << headPVAJ.col(0).transpose() << endl;
//        //        cout << "tailPVAJ: " << tailPVAJ.col(0).transpose() << endl;
//        //        cout << YELLOW << "Killing the node." << RESET << endl;
//        //        exit(-1);
//        return false;
//    }
//
//    for (const auto &poly: sfcs) {
//        if (std::isnan(poly.GetPlanes().sum())) {
//            cout << YELLOW << " -- [TrajOpt] Error, the SFC containes NaN." << RESET << endl;
//            return false;
//        }
//    }
//
//    bool success{true};
//
//    /// Setup optimization problems
//    opt_vars.default_init = true;
//    opt_vars.given_init_ts_and_ps = false;
//    opt_vars.headPVAJ = headPVAJ;
//    opt_vars.tailPVAJ = tailPVAJ;
//    opt_vars.guide_path.clear();
//    opt_vars.guide_t.clear();
//    opt_vars.hPolytopes.resize(sfcs.size());
//    for (long i = 0; i < sfcs.size(); i++) {
//        opt_vars.hPolytopes[i] = sfcs[i].GetPlanes();
//    }
//
//    if (!setupProblemAndCheck()) {
//        cout << YELLOW << " -- [planner] Minco corridor preprocess error." << RESET << endl;
//        success = false;
//    }
//
//    if (success && std::isinf(optimize(out_traj, cfg_.opt_accuracy))) {
//        std::cout << YELLOW << " -- [planner] in [ExpTrajOpt::optimize]: Optimization failed." << RESET << std::endl;
//        success = false;
//    }
//
//    if(success){
//        out_traj.start_WT = planner_context_->getSimTime();
//    }
//
//    if (!success && cfg_.save_log_en) {
//        failed_traj_log << 990419 << endl;
//        failed_traj_log << headPVAJ << endl;
//        failed_traj_log << tailPVAJ << endl;
//        for (long i = 0; i < sfcs.size(); i++) {
//            failed_traj_log << i << endl;
//            failed_traj_log << sfcs[i].GetPlanes() << endl;
//        }
//    }
//
//    return success;
//}

bool ExpTrajOpt::optimize(const StatePVAJ &headPVAJ, const StatePVAJ &tailPVAJ,
                          const vec_E<Vec3f> &guide_path, const vector<double> &guide_t,
                          PolytopeVec &sfcs,
                          Trajectory &out_traj) {
    /// Check if hot init is valid
    if (guide_path.empty() || guide_path.size() != guide_t.size()) {
        cout << YELLOW << " -- [TrajOpt] Error, the guide trajectory has wrong path and time stamp." << RESET
             << endl;
        return false;
    }
    for (std::size_t i = 0; i < guide_path.size(); ++i) {
        if (!guide_path[i].allFinite() || !std::isfinite(guide_t[i]) ||
            guide_t[i] < 0.0 || (i > 0 && guide_t[i] < guide_t[i - 1])) {
            cout << YELLOW << " -- [TrajOpt] Error, the guide trajectory contains non-finite or "
                 << "non-monotonic samples." << RESET << endl;
            return false;
        }
    }
    /// Check if SFC is valid
    if (sfcs.empty()) {
        cout << YELLOW << " -- [TrajOpt] Error, the SFC is empty." << RESET << endl;
        return false;
    }

    for (std::size_t i = 0; i < sfcs.size(); ++i) {
        auto planes = sfcs[i].GetPlanes();
        if (!navigation_planning_backend::normalizeCorridorPlanes(planes)) {
            planner_context_->warn(
                " -- [ExpOpt] corridor {} has invalid half-space planes; rejecting before geometry",
                i);
            return false;
        }
        const bool route_boundary_gate = sfcs[i].IsRouteBoundaryGate();
        const auto route_boundary_point = sfcs[i].GetRouteBoundaryPoint();
        const double route_boundary_radius = sfcs[i].GetRouteBoundaryRadius();
        if (route_boundary_gate &&
            (!route_boundary_point.allFinite() ||
             !std::isfinite(route_boundary_radius) || route_boundary_radius <= 0.0)) {
            planner_context_->warn(
                " -- [ExpOpt] corridor {} has invalid route-boundary metadata", i);
            return false;
        }
        sfcs[i].SetPlanes(std::move(planes));
        if (route_boundary_gate) {
            sfcs[i].SetRouteBoundaryContract(route_boundary_point, route_boundary_radius);
        }
    }

    if (!SimplifySFC(headPVAJ.col(0), tailPVAJ.col(0), sfcs)) {
        cout << YELLOW << " -- [TrajOpt] Cannot simplify sfcs." << RESET << endl;
        return false;
    }

    bool success{true};

    /// Setup optimization problems
    opt_vars.default_init = false;
    opt_vars.given_init_ts_and_ps = false;
    opt_vars.headPVAJ = headPVAJ;
    opt_vars.tailPVAJ = tailPVAJ;
    opt_vars.guide_path = guide_path;
    opt_vars.guide_t = guide_t;
    opt_vars.route_boundary_gates.assign(sfcs.size(), 0U);
    opt_vars.route_boundary_points.assign(
        sfcs.size(), Vec3f::Constant(std::numeric_limits<float>::quiet_NaN()));
    opt_vars.route_boundary_radii.assign(
        sfcs.size(), std::numeric_limits<double>::quiet_NaN());
    opt_vars.hPolytopes.resize(sfcs.size());

    for (std::size_t i = 0; i < sfcs.size(); ++i) {
        opt_vars.route_boundary_gates[i] =
                sfcs[i].IsRouteBoundaryGate() ? 1U : 0U;
        if (sfcs[i].IsRouteBoundaryGate()) {
            const auto &boundary_point = sfcs[i].GetRouteBoundaryPoint();
            const double boundary_radius = sfcs[i].GetRouteBoundaryRadius();
            if (!boundary_point.allFinite() || !std::isfinite(boundary_radius) ||
                boundary_radius <= 0.0) {
                planner_context_->warn(
                    " -- [ExpOpt] route-boundary gate {} has invalid point/radius",
                    i);
                return false;
            }
            opt_vars.route_boundary_points[i] =
                    boundary_point;
            opt_vars.route_boundary_radii[i] =
                    boundary_radius;
        }
        opt_vars.hPolytopes[i] = sfcs[i].GetPlanes();
        if (!navigation_planning_backend::normalizeCorridorPlanes(opt_vars.hPolytopes[i])) {
            planner_context_->warn(
                " -- [ExpOpt] corridor {} has invalid half-space planes; rejecting before MINCO",
                i);
            return false;
        }
    }

    if (!setupProblemAndCheck()) {
        cout << YELLOW << " -- [planner] Minco corridor preprocess error." << RESET << endl;
        success = false;
    }

    out_traj.clear();


    if (success && !std::isfinite(optimize(out_traj, cfg_.opt_accuracy))) {
        cout << YELLOW << " -- [planner] Minco exp_traj opt failed." << RESET << endl;
        success = false;
    }

    penalty_log << opt_vars.penalty_log.transpose() << endl;

    if (success) {
        out_traj.start_WT = planner_context_->getSimTime();
    }

    if (!success && cfg_.save_log_en) {
        failed_traj_log << 990419 << endl;
        failed_traj_log << headPVAJ << endl;
        failed_traj_log << tailPVAJ << endl;
        for (double i: guide_t) {
            failed_traj_log << i << " ";
        }
        failed_traj_log << endl;
        for (const auto &i: guide_path) {
            failed_traj_log << i.transpose() << " ";
        }
        failed_traj_log << endl;
        for (std::size_t i = 0; i < sfcs.size(); ++i) {
            failed_traj_log << i << endl;
            failed_traj_log << sfcs[i].GetPlanes() << endl;
        }
    }
    return success;
}

bool ExpTrajOpt::optimize(const StatePVAJ &headPVAJ, const StatePVAJ &tailPVAJ,
                          PolytopeVec &sfcs,
                          const vec_Vec3f &init_ps,
                          const VecDf &init_ts,
                          Trajectory &out_traj) {
    vec_Vec3f guide_path;
    guide_path.emplace_back(headPVAJ.col(0));
    for (const auto &i: init_ps) {
        guide_path.emplace_back(i);
    }
    guide_path.emplace_back(tailPVAJ.col(0));
    vector<double> guide_t;
    guide_t.emplace_back(0);
    double accumulate_t = 0;
    for (Eigen::Index i = 0; i < init_ts.size(); ++i) {
        accumulate_t += init_ts[i];
        guide_t.emplace_back(accumulate_t);
    }
    /// Check if hot init is valid
    if (guide_path.size() != guide_t.size()) {
        cout << YELLOW << " -- [TrajOpt] Error, the guide trajectory has wrong path and time stamp." << RESET
             << endl;
        return false;
    }
    /// Check if SFC is valid
    if (sfcs.empty()) {
        cout << YELLOW << " -- [TrajOpt] Error, the SFC is empty." << RESET << endl;
        return false;
    }

    for (std::size_t i = 0; i < sfcs.size(); ++i) {
        auto planes = sfcs[i].GetPlanes();
        if (!navigation_planning_backend::normalizeCorridorPlanes(planes)) {
            planner_context_->warn(
                " -- [ExpOpt] corridor {} has invalid half-space planes; rejecting before geometry",
                i);
            return false;
        }
        const bool route_boundary_gate = sfcs[i].IsRouteBoundaryGate();
        const auto route_boundary_point = sfcs[i].GetRouteBoundaryPoint();
        const double route_boundary_radius = sfcs[i].GetRouteBoundaryRadius();
        if (route_boundary_gate &&
            (!route_boundary_point.allFinite() ||
             !std::isfinite(route_boundary_radius) || route_boundary_radius <= 0.0)) {
            planner_context_->warn(
                " -- [ExpOpt] corridor {} has invalid route-boundary metadata", i);
            return false;
        }
        sfcs[i].SetPlanes(std::move(planes));
        if (route_boundary_gate) {
            sfcs[i].SetRouteBoundaryContract(route_boundary_point, route_boundary_radius);
        }
    }

    if (!SimplifySFC(headPVAJ.col(0), tailPVAJ.col(0), sfcs)) {
        cout << YELLOW << " -- [TrajOpt] Cannot simplify sfcs." << RESET << endl;
        return false;
    }

    bool success{true};

    /// Setup optimization problems
    opt_vars.default_init = false;
    opt_vars.given_init_ts_and_ps = true;
    opt_vars.init_ts = init_ts;
    opt_vars.init_ps = init_ps;
    opt_vars.headPVAJ = headPVAJ;
    opt_vars.tailPVAJ = tailPVAJ;
    opt_vars.guide_path = guide_path;
    opt_vars.guide_t = guide_t;
    opt_vars.route_boundary_gates.assign(sfcs.size(), 0U);
    opt_vars.route_boundary_points.assign(
        sfcs.size(), Vec3f::Constant(std::numeric_limits<float>::quiet_NaN()));
    opt_vars.route_boundary_radii.assign(
        sfcs.size(), std::numeric_limits<double>::quiet_NaN());
    opt_vars.hPolytopes.resize(sfcs.size());

    for (std::size_t i = 0; i < sfcs.size(); ++i) {
        opt_vars.route_boundary_gates[i] =
                sfcs[i].IsRouteBoundaryGate() ? 1U : 0U;
        if (sfcs[i].IsRouteBoundaryGate()) {
            const auto &boundary_point = sfcs[i].GetRouteBoundaryPoint();
            const double boundary_radius = sfcs[i].GetRouteBoundaryRadius();
            if (!boundary_point.allFinite() || !std::isfinite(boundary_radius) ||
                boundary_radius <= 0.0) {
                planner_context_->warn(
                    " -- [ExpOpt] route-boundary gate {} has invalid point/radius",
                    i);
                return false;
            }
            opt_vars.route_boundary_points[i] =
                    boundary_point;
            opt_vars.route_boundary_radii[i] =
                    boundary_radius;
        }
        opt_vars.hPolytopes[i] = sfcs[i].GetPlanes();
        if (!navigation_planning_backend::normalizeCorridorPlanes(opt_vars.hPolytopes[i])) {
            planner_context_->warn(
                " -- [ExpOpt] corridor {} has invalid half-space planes; rejecting before MINCO",
                i);
            return false;
        }
    }

    if (!setupProblemAndCheck()) {
        cout << YELLOW << " -- [planner] Minco corridor preprocess error." << RESET << endl;
        success = false;
    }

    out_traj.clear();

    if (success && !std::isfinite(optimize(out_traj, cfg_.opt_accuracy))) {
        cout << YELLOW << " -- [planner] Minco exp_traj opt failed." << RESET << endl;
        success = false;
    }
    penalty_log << opt_vars.penalty_log.transpose() << endl;

    if (success) {
        out_traj.start_WT = planner_context_->getSimTime();
    }


    if (!success && cfg_.save_log_en) {
        failed_traj_log << 990419 << endl;
        failed_traj_log << headPVAJ << endl;
        failed_traj_log << tailPVAJ << endl;
        for (double i: guide_t) {
            failed_traj_log << i << " ";
        }
        failed_traj_log << endl;
        for (const auto &i: guide_path) {
            failed_traj_log << i.transpose() << " ";
        }
        failed_traj_log << endl;
        for (std::size_t i = 0; i < sfcs.size(); ++i) {
            failed_traj_log << i << endl;
            failed_traj_log << sfcs[i].GetPlanes() << endl;
        }
    }
    return success;
}
