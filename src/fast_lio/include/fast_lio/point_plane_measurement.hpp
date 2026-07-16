// Copyright (c) 2026, LeTanDat
// SPDX-License-Identifier: BSD-3-Clause

#ifndef FAST_LIO_POINT_PLANE_MEASUREMENT_HPP_
#define FAST_LIO_POINT_PLANE_MEASUREMENT_HPP_

#include "fast_lio/commons.hpp"

namespace fast_lio {

/// @brief Point-to-plane measurement evaluation result.
struct PointPlaneMeasurement {
    /// Signed point-to-plane residual: r = n^T * (p_W - q)
    double residual = 0.0;

    /// Jacobian w.r.t. error state (1x15)
    /// H = [H_θ, H_p, 0, 0, 0] where:
    ///   H_θ = -n^T * R_WI * [p_I]_x  (3 elements)
    ///   H_p = n^T                     (3 elements)
    ///   remaining 9 elements = 0 (velocity, accel bias, gyro bias)
    Eigen::Matrix<double, 1, 15> H = Eigen::Matrix<double, 1, 15>::Zero();

    /// Point in world frame (for debugging)
    Eigen::Vector3d p_W = Eigen::Vector3d::Zero();

    /// Point in IMU frame (for debugging)
    Eigen::Vector3d p_I = Eigen::Vector3d::Zero();

    /// World-frame point-to-plane distance
    double distance = 0.0;

    /// Check if measurement is finite (no NaN/Inf)
    [[nodiscard]] bool isFinite() const {
        return std::isfinite(residual) && H.allFinite();
    }
};

/// @brief Evaluate point-to-plane measurement given state and plane.
///
/// Computes residual and Jacobian for a single point-to-plane correspondence.
/// Uses right perturbation convention: R' = R * Exp(δθ)
///
/// Formula:
///   p_I = T_I_L * p_L
///   p_W = R_WI * p_I + p_WI
///   r = n^T * (p_W - q)
///   H_θ = -n^T * R_WI * [p_I]_x
///   H_p = n^T
///
/// @param state Current state estimate (contains R_WI, p_WI, T_I_L)
/// @param point_L Point in LiDAR frame
/// @param plane_normal_W Plane normal in world frame (must be unit length)
/// @param plane_point_W Any point on the plane in world frame
/// @return Measurement with residual and Jacobian
PointPlaneMeasurement evaluatePointPlaneMeasurement(
    const State15& state,
    const Eigen::Vector3d& point_L,
    const Eigen::Vector3d& plane_normal_W,
    const Eigen::Vector3d& plane_point_W);

/// @brief Compute residual only (for finite-difference testing).
double computePointPlaneResidual(
    const State15& state,
    const Eigen::Vector3d& point_L,
    const Eigen::Vector3d& plane_normal_W,
    const Eigen::Vector3d& plane_point_W);

}  // namespace fast_lio

#endif  // FAST_LIO_POINT_PLANE_MEASUREMENT_HPP_
