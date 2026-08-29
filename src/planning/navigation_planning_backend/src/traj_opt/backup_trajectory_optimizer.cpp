/*
 * Product-owned navigation implementation.
 * Algorithmic provenance and external attributions are documented in the
 * package documentation; they are not part of the runtime API or behaviour.
 */

#include <traj_opt/backup_trajectory_optimizer.hpp>
#include <algorithm>
#include <chrono>
#include <navigation_planning/planning_limits.hpp>
#include <traj_opt/trajectory_dynamics.hpp>
#include <planner_core/backup_braking.hpp>
#include <planner_core/corridor_plane_validation.hpp>
#include <utils/header/color_msg_utils.hpp>

using namespace traj_opt;
using namespace color_text;
using namespace navigation_math;

namespace {
constexpr double kDurationSlackInitializationS = 1.0e-3;
constexpr double kDurationSlackInitializationRatio = 0.05;

void forwardDurationAboveFloor(const Eigen::VectorXd &tau,
                               const Eigen::VectorXd &floor,
                               Eigen::VectorXd &duration) {
    gcopter::forwardMapTauToT(tau, duration);
    duration += floor;
}

bool validReferenceTrajectory(const Trajectory &trajectory, const double query_time) {
    if (trajectory.empty() || trajectory.getPieceNum() <= 0 ||
        !std::isfinite(query_time) || query_time < 0.0) {
        return false;
    }
    const double total_duration = trajectory.getTotalDuration();
    if (!std::isfinite(total_duration) || total_duration <= 0.0 ||
        query_time > total_duration) {
        return false;
    }
    const Eigen::VectorXd durations = trajectory.getDurations();
    if (durations.size() != trajectory.getPieceNum() || !durations.allFinite() ||
        (durations.array() <= 0.0).any()) {
        return false;
    }
    for (int piece_index = 0; piece_index < trajectory.getPieceNum(); ++piece_index) {
        const auto &piece = trajectory[piece_index];
        const auto &coefficients = piece.getCoeffMat();
        if (coefficients.rows() != 3 || coefficients.cols() != piece.getDegree() + 1 ||
            !coefficients.allFinite()) {
            return false;
        }
    }
    StatePVAJ state;
    return trajectory.getState(query_time, state);
}

bool validBackupBoundaryInputs(const Trajectory &exp_traj,
                               const double t_0,
                               const double t_e,
                               const double heu_ts,
                               const VecDf &heu_end_pt,
                               const double heu_dur,
                               const Polytope &sfc) {
    return validReferenceTrajectory(exp_traj, heu_ts) &&
           std::isfinite(t_0) && std::isfinite(t_e) && std::isfinite(heu_ts) &&
           std::isfinite(heu_dur) && t_0 >= 0.0 && t_e > t_0 &&
           heu_ts >= t_0 && heu_ts <= t_e && heu_dur > 0.0 &&
           heu_end_pt.size() == 3 && heu_end_pt.allFinite() && !sfc.empty();
}

}  // namespace

int BackupTrajOpt::monitorProgress(void *instance,
                                   const Eigen::VectorXd &, const Eigen::VectorXd &,
                                   double, double, int, int) {
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

//========================================================================
void BackupTrajOpt::constraintsFunctional(const Eigen::VectorXd &T,
                                          const Eigen::MatrixX3d &coeffs,
                                          const PolyhedronH &hPoly,
                                          const double &smoothFactor,
                                          const int &integralResolution,
                                          const Eigen::VectorXd &magnitudeBounds,
                                          const Eigen::VectorXd &penaltyWeights,
                                          flatness::FlatnessMap &flatMap,
                                          double &cost,
                                          Eigen::VectorXd &gradT,
                                          Eigen::MatrixX3d &gradC,
                                          VecDf &pena_log) {
    // The shared penalty vector reserves slot 4 for the nominal waypoint
    // objective. Backup has no waypoint-attraction term.
    const double vmax = magnitudeBounds[0];
    const double amax = magnitudeBounds[1];
    const double jmax = magnitudeBounds[2];
    const double omgmax = magnitudeBounds[3];
    const double accthrmin = magnitudeBounds[4];
    const double accthrmax = magnitudeBounds[5];

    const double vmaxSqr = vmax * vmax;
    const double amaxSqr = amax * amax;
    const double jmaxSqr = jmax * jmax;
    const double omgmaxSqr = omgmax * omgmax;

    const double thrustMean = 0.5 * (accthrmax + accthrmin);
    const double thrustRadi = 0.5 * std::abs(accthrmax - accthrmin);
    const double thrustSqrRadi = thrustRadi * thrustRadi;

    const double weightPos = penaltyWeights[0];
    const double weightVel = penaltyWeights[1];
    const double weightAcc = penaltyWeights[2];
    const double weightJer = penaltyWeights[3];
    const double weightOmg = penaltyWeights[5];
    const double weightAccThr = penaltyWeights[6];


    Eigen::Vector3d pos, vel, acc, jer, sna;
    Eigen::Vector3d totalGradPos, totalGradVel, totalGradAcc, totalGradJer;
    double totalGradPsi, totalGradPsiD;
    double thr;
    Eigen::Vector4d quat;
    Eigen::Vector3d omg;
    double gradThr;
    Eigen::Vector3d gradPos, gradVel, gradAcc, gradJer, gradOmg;

    double step, alpha;
    double s1, s2, s3, s4, s5, s6, s7;
    Eigen::Matrix<double, 8, 1> beta0, beta1, beta2, beta3, beta4;
    Eigen::Vector3d outerNormal;

    double violaPos, violaVel, violaAcc, violaJer, violaOmg, violaThrust;
    double violaPosPenaD, violaVelPenaD, violaAccPenaD, violaJerPenaD, violaOmgPenaD, violaThrustPenaD;
    double violaPosPena, violaVelPena, violaAccPena, violaJerPena, violaOmgPena, violaThrustPena;
    double node, pena;
    const auto pieceNum = T.size();
    const double integralFrac = 1.0 / integralResolution;
    double position_penalty_log = 0.0;
    double max_pos_viola_log = 0.0;
    double velocity_penalty_log = 0.0;
    double max_vel_viola_log = 0.0;
    double acceleration_penalty_log = 0.0;
    double max_acc_viola_log = 0.0;
    double jerk_penalty_log = 0.0;
    double max_jer_viola_log = 0.0;
    double angular_rate_penalty_log = 0.0;
    double max_omg_viola_log = 0.0;
    double thrust_penalty_log = 0.0;
    double max_thr_viola_log = 0.0;

    for (int i = 0; i < pieceNum; i++) {
        const Eigen::Matrix<double, 8, 3> &c = coeffs.block<8, 3>(i * 8, 0);

        step = T(i) * integralFrac;
        for (int j = 0; j <= integralResolution; j++) {
            s1 = j * step;
            s2 = s1 * s1;
            s3 = s2 * s1;
            s4 = s2 * s2;
            s5 = s4 * s1;
            s6 = s4 * s2;
            s7 = s4 * s3;
            beta0 << 1.0, s1, s2, s3, s4, s5, s6, s7;
            beta1 << 0.0, 1.0, 2.0 * s1, 3.0 * s2, 4.0 * s3, 5.0 * s4, 6.0 * s5, 7.0 * s6;
            beta2 << 0.0, 0.0, 2.0, 6.0 * s1, 12.0 * s2, 20.0 * s3, 30.0 * s4, 42.0 * s5;
            beta3 << 0.0, 0.0, 0.0, 6.0, 24.0 * s1, 60.0 * s2, 120.0 * s3, 210.0 * s4;
            beta4 << 0.0, 0.0, 0.0, 0.0, 24.0, 120.0 * s1, 360.0 * s2, 840.0 * s3;
//            beta5 << 0.0, 0.0, 0.0, 0., 0.0, 120.0, 720.0 * s1, 2520.0 * s2;
            pos = c.transpose() * beta0;
            vel = c.transpose() * beta1;
            acc = c.transpose() * beta2;
            jer = c.transpose() * beta3;
            sna = c.transpose() * beta4;

            const auto K = hPoly.rows();

            violaVel = vel.squaredNorm() - vmaxSqr;
            violaAcc = acc.squaredNorm() - amaxSqr;
            violaJer = jer.squaredNorm() - jmaxSqr;
            gradThr = 0.0;
//            gradQuat.setZero();
            gradPos << 0, 0, 0;
            gradVel << 0, 0, 0;
            gradAcc << 0, 0, 0;
            gradJer << 0, 0, 0;
            gradOmg << 0, 0, 0;
            pena = 0.0;
            violaPos = 0.0;
            violaOmg = 0.0;
            violaThrust = 0.0;

            for (int k = 0; k < K; k++) {
                outerNormal = hPoly.block<1, 3>(k, 0);
                violaPos = outerNormal.dot(pos) + hPoly(k, 3);
                max_pos_viola_log = std::max(max_pos_viola_log, violaPos);
                if (weightPos > 0) {
                    if (gcopter::smoothedL1(violaPos, smoothFactor, violaPosPena, violaPosPenaD)) {
                        gradPos += weightPos * violaPosPenaD * outerNormal;
                        pena += weightPos * violaPosPena;
                        position_penalty_log += weightPos * violaPosPena;
                    }
                }
            }

            if (weightVel > 0 && gcopter::smoothedL1(violaVel, smoothFactor, violaVelPena, violaVelPenaD)) {
                gradVel += weightVel * violaVelPenaD * 2.0 * vel;
                pena += weightVel * violaVelPena;
                velocity_penalty_log += weightVel * violaVelPena;
            }

            if (weightAcc > 0 && gcopter::smoothedL1(violaAcc, smoothFactor, violaAccPena, violaAccPenaD)) {
                gradAcc += weightAcc * violaAccPenaD * 2.0 * acc;
                pena += weightAcc * violaAccPena;
                acceleration_penalty_log += weightAcc * violaAccPena;
            }

            if (weightJer > 0 && gcopter::smoothedL1(violaJer, smoothFactor, violaJerPena, violaJerPenaD)) {
                gradJer += weightJer * violaJerPenaD * 2.0 * jer;
                pena += weightJer * violaJerPena;
                jerk_penalty_log += weightJer * violaJerPena;
            }

            if (weightOmg > 0 || weightAccThr > 0) {
                flatMap.forward(vel, acc, jer, 0.0, 0.0, thr, quat, omg);
                violaOmg = omg.squaredNorm() - omgmaxSqr;
                violaThrust = (thr - thrustMean) * (thr - thrustMean) - thrustSqrRadi;

                if (weightOmg > 0 &&
                    gcopter::smoothedL1(violaOmg, smoothFactor, violaOmgPena, violaOmgPenaD)) {
                    gradOmg += weightOmg * violaOmgPenaD * 2.0 * omg;
                    pena += weightOmg * violaOmgPena;
                    angular_rate_penalty_log += weightOmg * violaOmgPena;
                }

                if (weightAccThr > 0 &&
                    gcopter::smoothedL1(violaThrust, smoothFactor, violaThrustPena, violaThrustPenaD)) {
                    gradThr += weightAccThr * violaThrustPenaD * 2.0 * (thr - thrustMean);
                    pena += weightAccThr * violaThrustPena;
                    thrust_penalty_log += weightAccThr * violaThrustPena;
                }

//            if (smoothedL1(violaTheta, smoothFactor, violaThetaPena, violaThetaPenaD))
//            {
//                gradQuat += weightTheta * violaThetaPenaD /
//                            sqrt(1.0 - cos_theta * cos_theta) * 4.0 *
//                            Eigen::Vector4d(0.0, quat(1), quat(2), 0.0);
//                pena += weightTheta * violaThetaPena;
//            }


                flatMap.backward(gradPos, gradVel, gradAcc, gradJer, gradThr, Vec4f(0, 0, 0, 0), gradOmg,
                                 totalGradPos, totalGradVel, totalGradAcc, totalGradJer,
                                 totalGradPsi, totalGradPsiD);

            } else {
                totalGradPos = gradPos;
                totalGradVel = gradVel;
                totalGradAcc = gradAcc;
                totalGradJer = gradJer;
            }

            {
                // log the max violation
                max_vel_viola_log = std::max(max_vel_viola_log, violaVel);
                max_acc_viola_log = std::max(max_acc_viola_log, violaAcc);
                max_jer_viola_log = std::max(max_jer_viola_log, violaJer);
                max_omg_viola_log = std::max(max_omg_viola_log, violaOmg);
                max_thr_viola_log = std::max(max_thr_viola_log, violaThrust);
            }

            node = (j == 0 || j == integralResolution) ? 0.5 : 1.0;
            alpha = j * integralFrac;
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
                        node * integralFrac * pena;
            cost += node * step * pena;
        }
    }

//    pena_log(1) = position_penalty_log;
//    pena_log(2) = velocity_penalty_log;
//    pena_log(3) = acceleration_penalty_log;
//    pena_log(4) = jerk_penalty_log;
//    pena_log(5) = attraction_penalty_log;
//    pena_log(6) = angular_rate_penalty_log;
//    pena_log(7) = thrust_penalty_log;
    pena_log(1) = max_pos_viola_log;
    pena_log(2) = max_vel_viola_log;
    pena_log(3) = max_acc_viola_log;
    pena_log(4) = max_jer_viola_log;
    pena_log(5) = 0.0;
    pena_log(6) = max_omg_viola_log;
    pena_log(7) = max_thr_viola_log;
}


double BackupTrajOpt::costFunctional(void *ptr, const Eigen::VectorXd &x, Eigen::VectorXd &g) {
//            TimeConsuming t_("cost functional");
    auto &obj = *static_cast<OptimizationVariables *>(ptr);
    obj.iter_num++;
    const long dimTau = obj.temporalDim;
    const long dimXi = obj.spatialDim;
    const double weightT = obj.rho;
    const double weight_ts = obj.weight_ts;
    Eigen::Map<const Eigen::VectorXd> tau(x.data(), dimTau);
    Eigen::Map<const Eigen::VectorXd> xi(x.data() + dimTau, dimXi);
    double tau_s = x(x.size() - 1);
    Eigen::Map<Eigen::VectorXd> gradTau(g.data(), dimTau);
    Eigen::Map<Eigen::VectorXd> gradXi(g.data() + dimTau, dimXi);
    double gradTaus = g(g.size() - 1);

    if (obj.uniform_time_en) {
        Eigen::VectorXd total_floor(1);
        total_floor(0) = obj.minimum_time_floor.sum();
        forwardDurationAboveFloor(tau, total_floor, obj.total_time);
        obj.times.setConstant(obj.total_time(0) / obj.times.size());
    } else {
        forwardDurationAboveFloor(tau, obj.minimum_time_floor, obj.times);
    }

    switch (obj.pos_constraint_type) {
        case 1: {
            VecDf xi_e = xi;
            obj.points = Eigen::Map<Eigen::Matrix<double, 3, Eigen::Dynamic>>(xi_e.data(), 3, xi_e.size() / 3);
            break;
        }
        default: {
            gcopter::forwardP(xi, obj.vPolytope, obj.points);
            break;
        }
    }


    if (obj.weight_ts > 0) {
        gcopter::mapInfToInterval(obj.min_ts, obj.max_ts, tau_s, obj.ts);
    }

    StatePVAJ headPVAJ, tailPVAJ;
    tailPVAJ.setZero();
    headPVAJ = obj.exp_traj.getState(obj.ts);
    tailPVAJ.col(0) = obj.points.rightCols(1);
    obj.minco.setConditions(headPVAJ, tailPVAJ);
    // points在这里是没有用的
    obj.minco.setParameters(obj.points.leftCols(obj.piece_num - 1), obj.times);
    double cost = 0;
    obj.partialGradByCoeffs.setZero();
    obj.partialGradByTimes.setZero();
    if (!obj.block_energy_cost) {
        obj.minco.getEnergy(cost);
        obj.minco.getEnergyPartialGradByCoeffs(obj.partialGradByCoeffs);
        obj.minco.getEnergyPartialGradByTimes(obj.partialGradByTimes);
    }
    obj.penalty_log.setZero();
    obj.penalty_log(0) = cost;
    constraintsFunctional(obj.times, obj.minco.getCoeffs(), obj.hPolytope,
                          obj.smooth_eps, obj.integral_res,
                          obj.magnitudeBounds, obj.penaltyWeights,
                          obj.quadrotor_flatness,
                          cost, obj.partialGradByTimes, obj.partialGradByCoeffs,
                          obj.penalty_log);

    StatePVAJ partGradOfHeadPVAJ, partGradOfTailPVAJ;
    Mat3Df partGradOfWaypts;
    obj.minco.propagateGradOfWayptsAndState(obj.partialGradByCoeffs, obj.partialGradByTimes,
                                            obj.gradByTimes,
                                            partGradOfHeadPVAJ,
                                            partGradOfWaypts,
                                            partGradOfTailPVAJ);
    cost += weightT * obj.times.sum();
    obj.gradByTimes.array() += weightT;
    obj.gradByPoints.leftCols(obj.piece_num - 1) = partGradOfWaypts;
    obj.gradByPoints.rightCols(1) = partGradOfTailPVAJ.col(0);

    if (obj.uniform_time_en) {
        obj.gradByTotalT(0) = obj.gradByTimes.sum() / obj.times.size();
        gcopter::propagateGradientTToTau(tau, obj.gradByTotalT, gradTau);
    } else {
        gcopter::propagateGradientTToTau(tau, obj.gradByTimes, gradTau);
    }

    switch (obj.pos_constraint_type) {
        case 1: {
            MatDf gp = obj.gradByPoints;
            gradXi = Eigen::Map<Eigen::VectorXd>(gp.data(), gp.size());
            break;
        }
        default: {
            gcopter::backwardGradP(xi, obj.vPolytope, obj.gradByPoints, gradXi);
            gcopter::normRetrictionLayer(xi, obj.vPolytope, cost, gradXi);
            break;
        }
    }
    // Add ts cost and gradient;
    if (obj.weight_ts > 0) {
        // square cost
//        cost += weight_ts * pow(obj.bod_.t_e - obj.ts, 2);
//        obj.gradTs =
//                partGradOfHeadPVAJ.col(0).dot(obj.bod_.exp_traj.getVel(obj.ts)) +
//                partGradOfHeadPVAJ.col(1).dot(obj.bod_.exp_traj.getAcc(obj.ts)) +
//                partGradOfHeadPVAJ.col(2).dot(obj.bod_.exp_traj.getJer(obj.ts)) +
//                partGradOfHeadPVAJ.col(3).dot(obj.bod_.exp_traj.getSnap(obj.ts)) +
//                -weight_ts * 2 * (obj.bod_.t_e - obj.ts);
//        gcopter::propagateGradIntervalToInf(obj.bod_.t_0, obj.bod_.t_e, tau_s, obj.gradTs, gradTaus);
//        g(g.size() - 1) = gradTaus;
        // linear cost
//        double vioTs = (obj.bod_.t_e - obj.ts);
//        double vioTsPen, vioTsPenD;
//        gcopter::smoothedL1(vioTs, obj.smoothEps, vioTsPen, vioTsPenD);
//        cost += weight_ts * vioTsPen;
//        double gradTs = partGradOfHeadPVAJ.col(0).dot(obj.bod_.exp_traj.getVel(obj.ts)) +
//                        partGradOfHeadPVAJ.col(1).dot(obj.bod_.exp_traj.getAcc(obj.ts)) +
//                        partGradOfHeadPVAJ.col(2).dot(obj.bod_.exp_traj.getJer(obj.ts)) +
//                        partGradOfHeadPVAJ.col(3).dot(obj.bod_.exp_traj.getSnap(obj.ts)) +
//                        -weight_ts * vioTsPenD;
//        gcopter::propagateGradIntervalToInf(obj.bod_.t_0, obj.bod_.t_e, tau_s, gradTs, gradTaus);
//        g(g.size() - 1) = gradTaus;

        cost += weight_ts * (obj.max_ts - obj.ts);
        obj.gradTs =
                partGradOfHeadPVAJ.col(0).dot(obj.exp_traj.getVel(obj.ts)) +
                partGradOfHeadPVAJ.col(1).dot(obj.exp_traj.getAcc(obj.ts)) +
                partGradOfHeadPVAJ.col(2).dot(obj.exp_traj.getJer(obj.ts)) +
                partGradOfHeadPVAJ.col(3).dot(obj.exp_traj.getSnap(obj.ts)) +
                -weight_ts;

        gcopter::propagateGradIntervalToInf(obj.min_ts, obj.max_ts, tau_s, obj.gradTs, gradTaus);
        g(g.size() - 1) = gradTaus;

    } else {
        g(g.size() - 1) = 0;
    }
    return cost;
}

bool BackupTrajOpt::processCorridor() {
    PolyhedronV curIV, curIOB; // 走廊的顶点
    if (!geometry_utils::enumerateVs(opt_vars.hPolytope, curIV)) {
        std::cout << YELLOW << " -- [MINCO] enumerateVs failed." << RESET << std::endl;
        return false;
    }
    const Eigen::Index nv = curIV.cols();
    if (nv <= 0 || !curIV.allFinite()) {
        std::cout << YELLOW << " -- [MINCO] enumerateVs returned no finite vertices."
                  << RESET << std::endl;
        return false;
    }
    curIOB.resize(3, nv);
    // 第一个点存储第一个顶点
    curIOB.col(0) = curIV.col(0);
    // 后面的点都归一化到第一个点坐标系下
    curIOB.rightCols(nv - 1) = curIV.rightCols(nv - 1).colwise() - curIV.col(0);
    opt_vars.vPolytope = curIOB;
    return true;
}

bool BackupTrajOpt::setupProblemAndCheck() {
    // 1. Validate and normalize before any corridor geometry or optimizer use.
    if (!navigation_planning_backend::normalizeCorridorPlanes(opt_vars.hPolytope)) {
        std::cout << YELLOW << " -- [BackTrajOpt] Invalid corridor planes." << RESET << std::endl;
        return false;
    }

    if (!processCorridor()) {
        std::cout << YELLOW << " -- [processCorridor] Failed to get Overlap enumerateVs ." << RESET << std::endl;
        return false;
    }

    // 2. Reset the problem dimension
    if (opt_vars.uniform_time_en) {
        opt_vars.temporalDim = 1;
    } else {
        opt_vars.temporalDim = opt_vars.piece_num;
    }
    switch (opt_vars.pos_constraint_type) {
        case 1: {
            opt_vars.spatialDim = 3 * opt_vars.piece_num;
            break;
        }
        default: {
            opt_vars.spatialDim = opt_vars.vPolytope.cols() * opt_vars.piece_num;
        }
    }

    opt_vars.minco.setConditions(opt_vars.headPVAJ, opt_vars.tailPVAJ, opt_vars.piece_num);
    opt_vars.points.resize(3, opt_vars.piece_num);
    opt_vars.gradByPoints.resize(3, opt_vars.piece_num);
    opt_vars.partialGradByCoeffs.resize(8 * opt_vars.piece_num, 3);
    opt_vars.partialGradByTimes.resize(opt_vars.piece_num);
    return true;
}

double BackupTrajOpt::optimize(Trajectory &traj, const double &relCostTol) {
    // 1. Initialize the trajectory
    //      the optimization variables include time allocation [1] * pieceN + tail waypoints [vPoly_size] * pieceN + split time [1]
    Eigen::VectorXd x(opt_vars.temporalDim + opt_vars.spatialDim + 1);
    Eigen::Map<Eigen::VectorXd> tau(x.data(), opt_vars.temporalDim);
    Eigen::Map<Eigen::VectorXd> xi(x.data() + opt_vars.temporalDim, opt_vars.spatialDim);

    // 2. Initialize the optimization problem
    // 初始化均匀时间分配，均匀waypoint分配
    Vec3f step = (opt_vars.tailPVAJ.col(0) - opt_vars.headPVAJ.col(0)) / opt_vars.piece_num;
    for (int i = 0; i < opt_vars.piece_num - 1; i++) {
        opt_vars.points.col(i) = step * (i + 1) + opt_vars.headPVAJ.col(0);
    }
    opt_vars.points.rightCols(1) = opt_vars.tailPVAJ.col(0);

    if(opt_vars.given_init_ts_and_ps){
        opt_vars.times = opt_vars.given_init_t_vec;
        for (std::size_t i = 0; i < opt_vars.given_init_ps.size(); ++i) {
            opt_vars.points.col(i) = opt_vars.given_init_ps[i];
        }
        opt_vars.ts = opt_vars.given_init_ts;
    }

    if (opt_vars.uniform_time_en) {
        Eigen::VectorXd total_floor(1);
        total_floor(0) = opt_vars.minimum_time_floor.sum();
        const Eigen::VectorXd initial_slack =
                (total_floor * kDurationSlackInitializationRatio)
                        .cwiseMax(kDurationSlackInitializationS);
        Eigen::VectorXd slack = (opt_vars.total_time - total_floor)
                                        .cwiseMax(initial_slack);
        gcopter::backwardMapTToTau(slack, tau);
    } else {
        const Eigen::VectorXd initial_slack =
                (opt_vars.minimum_time_floor * kDurationSlackInitializationRatio)
                        .cwiseMax(kDurationSlackInitializationS);
        Eigen::VectorXd slack = (opt_vars.times - opt_vars.minimum_time_floor)
                                        .cwiseMax(initial_slack);
        gcopter::backwardMapTToTau(slack, tau);
    }

    switch (opt_vars.pos_constraint_type) {
        case 1: {
            MatDf p_e = opt_vars.points;
            xi = Eigen::Map<const VecDf>(p_e.data(), p_e.size());
            break;
        }
        default: {
            gcopter::backwardP(opt_vars.points, opt_vars.vPolytope, xi);
            break;
        }
    }

    double tau_s;
    gcopter::mapIntervalToInf(opt_vars.min_ts, opt_vars.max_ts, opt_vars.ts, tau_s);
    x(x.size() - 1) = tau_s;
    double minCostFunctional;
    lbfgs::lbfgs_parameter_t lbfgs_params;
    lbfgs_params.mem_size = cfg_.lbfgs_memory_size;
    lbfgs_params.past = 3;
    lbfgs_params.min_step = 1.0e-32;
    lbfgs_params.g_epsilon = 0.0;
    lbfgs_params.delta = relCostTol;
    int ret;
    opt_vars.penalty_log.resize(8);
    opt_vars.penalty_log.setZero();
    opt_vars.iter_num = 0;

    opt_vars.init_ts = opt_vars.ts;
    opt_vars.init_t_vec = opt_vars.times;
    opt_vars.init_ps.clear();
    for (int col = 0; col < opt_vars.points.cols(); col++) {
        opt_vars.init_ps.emplace_back(opt_vars.points.col(col));
    }

    TimeConsuming ttt(" -- [BackupTrajOpt]", false);
    if (opt_vars.debug_en) {
        throw std::runtime_error(" -- [BackupTrajOpt] Debug mode is not supported yet.");
    } else {
        ret = lbfgs::lbfgs_optimize(x,
                                    minCostFunctional,
                                    &BackupTrajOpt::costFunctional,
                                    nullptr,
                                    &BackupTrajOpt::monitorProgress,
                                    &this->opt_vars,
                                    lbfgs_params);

    }
    if (ret == lbfgs::LBFGS_CANCELED) {
        traj.clear();
        return -1.0;
    }
    using namespace std;
    if (cfg_.print_optimizer_log) {
        cout << " -- [BackupOpt] Optimization finished, with iter num: " << opt_vars.iter_num << "\n";
        cout << "\tEnergy: " << opt_vars.penalty_log(0) << endl;
        cout << "\tPos: " << opt_vars.penalty_log(1) << endl;
        cout << "\tVel: " << opt_vars.penalty_log(2) << endl;
        cout << "\tAcc: " << opt_vars.penalty_log(3) << endl;
        cout << "\tJerk: " << opt_vars.penalty_log(4) << endl;
        cout << "\tAttract: " << opt_vars.penalty_log(5) << endl;
        cout << "\tOmg: " << opt_vars.penalty_log(6) << endl;
        cout << "\tThr: " << opt_vars.penalty_log(7) << endl;
        cout << "\tTs: " << opt_vars.ts << endl;
    }
    if (ret >= 0) {
        if (opt_vars.uniform_time_en) {
            Eigen::VectorXd total_floor(1);
            total_floor(0) = opt_vars.minimum_time_floor.sum();
            forwardDurationAboveFloor(tau, total_floor, opt_vars.total_time);
            opt_vars.times.setConstant(opt_vars.total_time(0) / opt_vars.times.size());
        } else {
            forwardDurationAboveFloor(tau, opt_vars.minimum_time_floor, opt_vars.times);
        }
        switch (opt_vars.pos_constraint_type) {
            case 1: {
                VecDf xi_e = xi;
                opt_vars.points = Eigen::Map<Eigen::Matrix<double, 3, Eigen::Dynamic>>(xi_e.data(), 3, xi_e.size() / 3);
                break;
            }
            default: {
                gcopter::forwardP(xi, opt_vars.vPolytope, opt_vars.points);
                break;
            }
        }

        opt_vars.tailPVAJ.setZero();
        opt_vars.headPVAJ = opt_vars.exp_traj.getState(opt_vars.ts);
        opt_vars.tailPVAJ.col(0) = opt_vars.points.rightCols(1);
        opt_vars.minco.setConditions(opt_vars.headPVAJ, opt_vars.tailPVAJ, opt_vars.piece_num);
        opt_vars.minco.setEndPosition(opt_vars.points.rightCols(1));
        opt_vars.minco.setParameters(opt_vars.points.leftCols(opt_vars.piece_num - 1), opt_vars.times);
        opt_vars.minco.getTrajectory(traj);
    } else {
        traj.clear();
        minCostFunctional = INFINITY;
        std::cout << YELLOW << " -- [MINCO] TrajOpt failed, " << lbfgs::lbfgs_strerror(ret) << RESET << std::endl;
    }
    return minCostFunctional;
}

BackupTrajOpt::BackupTrajOpt(const traj_opt::Config &cfg, const navigation_planner_context::PlannerRuntimeContext::Ptr &planner_context)
        : cfg_(cfg), planner_context_(planner_context) {
    using namespace std;

    cfg_ = cfg;
    cfg_.validate(true);
    if (!planner_context_) {
        throw std::invalid_argument("BackupTrajOpt requires a planner runtime context");
    }
    std::string filename = "back_opt_log.csv";
    if(cfg_.save_log_en){
        failed_traj_log.open(NAVIGATION_PLANNER_DEBUG_FILE_DIR(filename), std::ios::out | std::ios::trunc);
        penalty_log.open(NAVIGATION_PLANNER_DEBUG_FILE_DIR("backup_opt_objective.csv"), std::ios::out | std::ios::trunc);
    }
    opt_vars.magnitudeBounds.resize(6);
    opt_vars.penaltyWeights.resize(7);
    opt_vars.magnitudeBounds << cfg_.max_vel, cfg_.max_acc, cfg_.max_jerk,
            cfg_.max_omg, cfg_.min_acc_thr * cfg_.mass, cfg_.max_acc_thr * cfg_.mass;
    opt_vars.penaltyWeights << cfg_.position_penalty_weight,
            cfg_.velocity_penalty_weight, cfg_.acceleration_penalty_weight,
            cfg_.jerk_penalty_weight, 0.0,
            cfg_.angular_rate_penalty_weight, cfg_.thrust_penalty_weight;
    opt_vars.rho = cfg_.time_weight;
    opt_vars.pos_constraint_type = cfg_.pos_constraint_type;
    opt_vars.block_energy_cost = cfg_.block_energy_cost;
    opt_vars.smooth_eps = cfg_.smooth_eps;
    opt_vars.integral_res = cfg_.integral_reso;
    opt_vars.quadrotor_flatness = cfg_.quadrotor_flatness;
    opt_vars.weight_ts = cfg_.terminal_time_weight;
    opt_vars.uniform_time_en = cfg_.uniform_time_en;
    opt_vars.piece_num = cfg_.piece_num;
    opt_vars.total_time.resize(1);
    opt_vars.gradByTotalT.resize(1);
}

bool BackupTrajOpt::checkTrajMagnitudeBound(Trajectory &out_traj) {
    if (out_traj.empty() || !std::isfinite(out_traj.getTotalDuration()) ||
        out_traj.getTotalDuration() <= 0.0) {
        std::cout << YELLOW << " -- [TrajOpt] Minco backup produced an empty or invalid trajectory."
                  << RESET << std::endl;
        return false;
    }
    const double maximum_jerk = out_traj.getMaxJerRate();
    const double jerk_gate = cfg_.max_jerk;
    if (!navigation_planning::withinNumericalDynamicLimit(
            maximum_jerk, jerk_gate)) {
        std::cout << YELLOW << " -- [TrajOpt] Minco backup hard jerk gate failed: "
                  << maximum_jerk << " > " << jerk_gate << RESET << std::endl;
        return false;
    }
    if (!navigation_planning::withinNumericalDynamicLimit(
            out_traj.getMaxVelRate(), cfg_.max_vel)) {
        std::cout << YELLOW << " -- [TrajOpt] Minco backup opt failed." << RESET << std::endl;
        std::cout << YELLOW << "\t\tBackend Max vel:\t" << out_traj.getMaxVelRate() << " m/s" << RESET
                  << std::endl;
        return false;
    }
    if (!navigation_planning::withinNumericalDynamicLimit(
            out_traj.getMaxAccRate(), cfg_.max_acc)) {
        std::cout << YELLOW << " -- [TrajOpt] Minco backup opt failed." << RESET << std::endl;
        std::cout << YELLOW << "\t\tBackend Max Acc:\t" << out_traj.getMaxAccRate() << " m/s" << RESET
                  << std::endl;
        return false;
    }
    TrajectoryDynamicReport dynamic_report;
    if (!trajectorySatisfiesFlatnessEnvelope(out_traj, cfg_, &dynamic_report)) {
        std::cout << YELLOW << " -- [TrajOpt] Minco backup flatness hard gate failed: "
                  << "finite=" << dynamic_report.finite
                  << " body_rate=" << dynamic_report.maximum_body_rate_rad_s
                  << "/" << cfg_.max_omg
                  << " thrust=[" << dynamic_report.minimum_thrust_n << ","
                  << dynamic_report.maximum_thrust_n << "]/["
                  << cfg_.min_acc_thr * cfg_.mass << ","
                  << cfg_.max_acc_thr * cfg_.mass << "]"
                  << RESET << std::endl;
        return false;
    }
    return true;
}

bool
BackupTrajOpt::optimize(const Trajectory &exp_traj,
                        const double &t_0,
                        const double &t_e,
                        const double &heu_ts,
                        const VecDf &heu_end_pt,
                        double &heu_dur,
                        const Polytope &sfc,
                        Trajectory &out_traj,
                        double &out_ts,
                        const bool &debug) {
    if (!validBackupBoundaryInputs(exp_traj, t_0, t_e, heu_ts, heu_end_pt,
                                   heu_dur, sfc)) {
        out_traj.clear();
        out_ts = std::numeric_limits<double>::quiet_NaN();
        return false;
    }
    opt_vars.hPolytope = sfc.GetPlanes();

    opt_vars.debug_en = debug;
    /// Setup optimization problems
    opt_vars.default_init = true;
    opt_vars.given_init_ts_and_ps = false;
    opt_vars.headPVAJ = exp_traj.getState(heu_ts);
    opt_vars.tailPVAJ.setZero();
    opt_vars.guide_path.clear();
    opt_vars.guide_t.clear();
    opt_vars.exp_traj = exp_traj;
    opt_vars.piece_num = cfg_.piece_num;
    opt_vars.max_ts = t_e;
    opt_vars.min_ts = t_0;
    opt_vars.tailPVAJ.col(0) = heu_end_pt;
    opt_vars.times.resize(opt_vars.piece_num);
    opt_vars.times.setConstant(heu_dur / opt_vars.piece_num);
    opt_vars.minimum_time_floor = opt_vars.times;
    opt_vars.ts = heu_ts;

    if (opt_vars.uniform_time_en) {
        opt_vars.total_time(0) = heu_dur;
    }


    out_traj.clear();
    bool success{true};

    if (!setupProblemAndCheck()) {
        std::cout << YELLOW << " -- [TrajOpt] Minco corridor preprocess error." << RESET << std::endl;
        success = false;
    }

    if (success && !std::isfinite(optimize(out_traj, cfg_.opt_accuracy))) {
        std::cout << YELLOW << " -- [planner] Minco backup_traj opt failed." << RESET << std::endl;
        success = false;
    }

    const double maximum_corridor_violation =
        navigation_planning_backend::maximumContinuousCorridorPlaneViolation(
            out_traj, opt_vars.hPolytope);
    if (success &&
        (!std::isfinite(maximum_corridor_violation) ||
         maximum_corridor_violation > cfg_.corridor_plane_tolerance_m)) {
        std::cout << YELLOW << " -- [planner] Minco backup_traj out of corridor." << RESET << std::endl;
        success = false;
    }
    out_ts = opt_vars.ts;

    // Preserve the first causal failure.  When setup/optimization already
    // failed, out_traj is intentionally empty and its extrema are -inf;
    // reporting a dynamic-gate failure would hide the corridor/root cause.
    if (success && !checkTrajMagnitudeBound(out_traj)) {
        success = false;
    }
    if (success && !navigation_planning_backend::refinementDurationRespectsCertifiedFloor(
            out_traj.getTotalDuration(), opt_vars.minimum_time_floor.sum())) {
        planner_context_->warn(
                " -- [BackOpt] refinement duration {} is shorter than certified seed {}",
                out_traj.getTotalDuration(), opt_vars.minimum_time_floor.sum());
        success = false;
    }

    if (!success && cfg_.save_log_en) {
        // log the optimization problem
        failed_traj_log << 123321 << std::endl;
        failed_traj_log << t_0 << std::endl;
        failed_traj_log << t_e << std::endl;
        failed_traj_log << heu_ts << std::endl;
        failed_traj_log << heu_end_pt.transpose() << std::endl;
        failed_traj_log << heu_dur << std::endl;
        failed_traj_log << 0 << std::endl;
        failed_traj_log << sfc.GetPlanes() << std::endl;
        failed_traj_log << exp_traj.getPieceNum() << std::endl;
        failed_traj_log << exp_traj.getDurations().transpose() << std::endl;
        for (int i = 0; i < exp_traj.getPieceNum(); i++) {
            failed_traj_log << exp_traj[i].getCoeffMat() << std::endl;
        }
        out_ts = heu_ts;
    }

    return success;
}

bool
BackupTrajOpt::optimize(const Trajectory &exp_traj,
                        const double &t_0,
                        const double &t_e,
                        const double &heu_ts,
                        const Polytope &sfc,
                        const VecDf & init_t_vec,
                        const vec_Vec3f &init_ps,
                        Trajectory &out_traj,
                        double & out_ts) {
    // This path is called from the realtime planner and must fail closed on
    // malformed warm-start data.  In particular, init_ps.back() used to be
    // reachable with an empty vector, turning a bad candidate into UB.
    if (!validReferenceTrajectory(exp_traj, heu_ts) ||
        !std::isfinite(t_0) || !std::isfinite(t_e) || !std::isfinite(heu_ts) ||
        t_0 < 0.0 || t_e <= t_0 || heu_ts < t_0 || heu_ts > t_e || sfc.empty() ||
        cfg_.piece_num <= 0 ||
        init_t_vec.size() != static_cast<Eigen::Index>(cfg_.piece_num) ||
        init_ps.size() != static_cast<std::size_t>(cfg_.piece_num) ||
        init_t_vec.size() == 0 || !init_t_vec.allFinite() ||
        (init_t_vec.array() <= 0.0).any() ||
        !std::all_of(init_ps.begin(), init_ps.end(),
                     [](const Vec3f &point) { return point.allFinite(); })) {
        std::cout << YELLOW << " -- [BackTrajOpt] Invalid warm-start dimensions or values."
                  << RESET << std::endl;
        out_traj.clear();
        return false;
    }
    const double initial_duration = init_t_vec.sum();
    if (!std::isfinite(initial_duration) || initial_duration <= 0.0) {
        out_traj.clear();
        return false;
    }
    opt_vars.hPolytope = sfc.GetPlanes();

    opt_vars.debug_en = false;
    /// Setup optimization problems
    opt_vars.default_init = true;

    opt_vars.headPVAJ = exp_traj.getState(heu_ts);
    opt_vars.tailPVAJ.setZero();
    opt_vars.guide_path.clear();
    opt_vars.guide_t.clear();
    opt_vars.exp_traj = exp_traj;
    opt_vars.piece_num = cfg_.piece_num;
    opt_vars.max_ts = t_e;
    opt_vars.min_ts = t_0;
    opt_vars.tailPVAJ.col(0) = init_ps.back();
    opt_vars.times.resize(opt_vars.piece_num);
    const double heu_dur = initial_duration;
    opt_vars.times.setConstant(heu_dur / opt_vars.piece_num);
    opt_vars.minimum_time_floor = init_t_vec;
    opt_vars.ts = heu_ts;
    if (opt_vars.uniform_time_en) {
        opt_vars.total_time(0) = heu_dur;
    }

    opt_vars.given_init_ts_and_ps = true;
    opt_vars.given_init_t_vec = init_t_vec;
    opt_vars.given_init_ps = init_ps;
    opt_vars.given_init_ts = heu_ts;

    out_traj.clear();
    bool success{true};

    if (!setupProblemAndCheck()) {
        std::cout << YELLOW << " -- [TrajOpt] Minco corridor preprocess error." << RESET << std::endl;
        success = false;
    }

    if (success && !std::isfinite(optimize(out_traj, cfg_.opt_accuracy))) {
        std::cout << YELLOW << " -- [planner] Minco backup_traj opt failed." << RESET << std::endl;
        success = false;
    }

    const double maximum_corridor_violation =
        navigation_planning_backend::maximumContinuousCorridorPlaneViolation(
            out_traj, opt_vars.hPolytope);
    if (success &&
        (!std::isfinite(maximum_corridor_violation) ||
         maximum_corridor_violation > cfg_.corridor_plane_tolerance_m)) {
        std::cout << YELLOW << " -- [planner] Minco backup_traj out of corridor." << RESET << std::endl;
        success = false;
    }
    out_ts = opt_vars.ts;

    // Preserve the first causal failure; do not evaluate extrema on an empty
    // output after corridor preprocessing or optimization has failed.
    if (success && !checkTrajMagnitudeBound(out_traj)) {
        success = false;
    }
    if (success && !navigation_planning_backend::refinementDurationRespectsCertifiedFloor(
            out_traj.getTotalDuration(), opt_vars.minimum_time_floor.sum())) {
        planner_context_->warn(
                " -- [BackOpt] refinement duration {} is shorter than certified seed {}",
                out_traj.getTotalDuration(), opt_vars.minimum_time_floor.sum());
        success = false;
    }


    return success;
}
