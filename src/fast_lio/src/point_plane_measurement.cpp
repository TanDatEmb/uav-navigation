// Copyright (c) 2026, LeTanDat
// SPDX-License-Identifier: BSD-3-Clause

#include "fast_lio/point_plane_measurement.hpp"

namespace fast_lio {

PointPlaneMeasurement evaluatePointPlaneMeasurement(
    const State15& state,
    const Eigen::Vector3d& point_L,
    const Eigen::Vector3d& plane_normal_W,
    const Eigen::Vector3d& plane_point_W) {
    PointPlaneMeasurement result;

    // Transform point: LiDAR -> IMU -> World
    // p_I = T_I_L * p_L = R_I_L * p_L + t_I_L
    result.p_I = state.T_I_L * point_L;

    // p_W = R_WI * p_I + p_WI
    result.p_W = state.R_wb * result.p_I + state.p_w;

    // Point-to-plane residual: r = n^T * (p_W - q)
    result.residual = plane_normal_W.dot(result.p_W - plane_point_W);
    result.distance = std::abs(result.residual);

    // Jacobian w.r.t. rotation (right perturbation: R' = R * Exp(δθ))
    // H_θ = -n^T * R_WI * [p_I]_x
    const Eigen::Matrix3d p_I_skew = skewSymmetric(result.p_I);
    const Eigen::Vector3d H_theta =
        -(plane_normal_W.transpose() * state.R_wb.matrix() * p_I_skew).transpose();

    // Jacobian w.r.t. position
    // H_p = n^T
    const Eigen::Vector3d H_p = plane_normal_W;

    // Assemble full Jacobian (1x15)
    result.H.setZero();
    result.H.segment<3>(0) = H_theta;   // δθ (rotation error)
    result.H.segment<3>(3) = H_p;         // δp (position error)
    // H.segment<3>(6) = 0 (velocity - no direct observation)
    // H.segment<3>(9) = 0 (accel bias)
    // H.segment<3>(12) = 0 (gyro bias)

    return result;
}

double computePointPlaneResidual(
    const State15& state,
    const Eigen::Vector3d& point_L,
    const Eigen::Vector3d& plane_normal_W,
    const Eigen::Vector3d& plane_point_W) {
    // p_I = T_I_L * p_L
    const Eigen::Vector3d p_I = state.T_I_L * point_L;

    // p_W = R_WI * p_I + p_WI
    const Eigen::Vector3d p_W = state.R_wb * p_I + state.p_w;

    // r = n^T * (p_W - q)
    return plane_normal_W.dot(p_W - plane_point_W);
}

}  // namespace fast_lio
