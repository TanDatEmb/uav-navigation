// Copyright (c) 2026, LeTanDat
// SPDX-License-Identifier: BSD-3-Clause

#include <gtest/gtest.h>

#include <cmath>

#include "fast_lio/commons.hpp"
#include "fast_lio/point_plane_measurement.hpp"

namespace fast_lio {
namespace {

// Helper: Skew-symmetric matrix
Eigen::Matrix3d skewSymmetric(const Eigen::Vector3d& v) {
    Eigen::Matrix3d M;
    M << 0, -v(2), v(1), v(2), 0, -v(0), -v(1), v(0), 0;
    return M;
}

// J1: Full rotation + translation + full extrinsic
TEST(PointPlaneMeasurementTest, J1_FullRotationTranslationExtrinsic) {
    // General state (non-identity, non-axis-aligned)
    State15 state;
    state.R_wb = SO3d::exp(Eigen::Vector3d(0.21, -0.17, 0.13));
    state.p_w = Eigen::Vector3d(1.2, -0.7, 0.4);
    state.T_I_L = SE3d(SO3d::exp(Eigen::Vector3d(-0.08, 0.11, 0.19)),
                        Eigen::Vector3d(0.23, -0.04, 0.12));

    const Eigen::Vector3d point_L(1.3, -0.6, 2.1);
    const Eigen::Vector3d normal_W = Eigen::Vector3d(0.4, -0.3, 0.8).normalized();
    const Eigen::Vector3d plane_point_W(2.0, -1.1, 0.5);

    const auto measurement =
        evaluatePointPlaneMeasurement(state, point_L, normal_W, plane_point_W);

    // Verify residual consistency
    const double direct_residual =
        computePointPlaneResidual(state, point_L, normal_W, plane_point_W);
    EXPECT_NEAR(measurement.residual, direct_residual, 1e-12);

    // Finite-difference verification for rotation and position
    constexpr double epsilon = 1e-7;
    const double tolerance = 1e-5;

    for (int idx = 0; idx < 6; ++idx) {
        V15D delta = V15D::Zero();
        delta(idx) = epsilon;

        State15 plus = state;
        plus.update(delta);

        delta(idx) = -epsilon;
        State15 minus = state;
        minus.update(delta);

        const double residual_plus =
            computePointPlaneResidual(plus, point_L, normal_W, plane_point_W);
        const double residual_minus =
            computePointPlaneResidual(minus, point_L, normal_W, plane_point_W);
        const double numerical = (residual_plus - residual_minus) / (2.0 * epsilon);

        EXPECT_NEAR(numerical, measurement.H(0, idx), tolerance)
            << "Jacobian mismatch at index " << idx;
    }
}

// J2: Nonzero lever arm
TEST(PointPlaneMeasurementTest, J2_NonzeroLeverArm) {
    State15 state;
    state.R_wb = SO3d();
    state.p_w = Eigen::Vector3d::Zero();
    state.T_I_L = SE3d(SO3d(), Eigen::Vector3d(0.5, -0.3, 0.2));  // Pure translation extrinsic

    const Eigen::Vector3d point_L(1.0, 0.0, 0.0);
    const Eigen::Vector3d normal_W = Eigen::Vector3d::UnitY();
    const Eigen::Vector3d plane_point_W = Eigen::Vector3d::Zero();

    const auto measurement =
        evaluatePointPlaneMeasurement(state, point_L, normal_W, plane_point_W);

    // With lever arm, point in world is offset by t_I_L
    // p_I = point_L + t_I_L = (1.0, 0, 0) + (0.5, -0.3, 0.2) = (1.5, -0.3, 0.2)
    // r = n^T * p_W = [0,1,0] * [1.5, -0.3, 0.2] = -0.3
    EXPECT_NEAR(measurement.residual, -0.3, 1e-6);

    // Verify via direct computation
    const Eigen::Vector3d p_I = point_L + state.T_I_L.translation();
    const Eigen::Vector3d p_W = p_I;  // R = I, p_w = 0
    EXPECT_NEAR(normal_W.dot(p_W - plane_point_W), measurement.residual, 1e-9);
}

// J3: Rotational extrinsic
TEST(PointPlaneMeasurementTest, J3_RotationalExtrinsic) {
    State15 state;
    state.R_wb = SO3d();
    state.p_w = Eigen::Vector3d::Zero();
    state.T_I_L = SE3d(SO3d::exp(Eigen::Vector3d(0.0, 0.0, M_PI / 2)),
                        Eigen::Vector3d::Zero());  // 90° Z rotation

    const Eigen::Vector3d point_L(1.0, 0.0, 0.0);
    const Eigen::Vector3d normal_W = Eigen::Vector3d::UnitX();
    const Eigen::Vector3d plane_point_W = Eigen::Vector3d::Zero();

    const auto measurement =
        evaluatePointPlaneMeasurement(state, point_L, normal_W, plane_point_W);

    // Point (1,0,0) in LiDAR rotated 90° around Z becomes (0,1,0) in IMU
    // r = n^T * p_W = [1,0,0] * [0,1,0] = 0
    EXPECT_NEAR(std::abs(measurement.residual), 0.0, 1e-6);
}

// J4: Identity sanity case
TEST(PointPlaneMeasurementTest, J4_IdentitySanityCase) {
    State15 state;
    state.R_wb = SO3d();
    state.p_w = Eigen::Vector3d::Zero();
    state.T_I_L = SE3d();

    const Eigen::Vector3d point_L(1.0, 0.0, 0.0);
    const Eigen::Vector3d normal_W = Eigen::Vector3d::UnitX();
    const Eigen::Vector3d plane_point_W(2.0, 0.0, 0.0);

    const auto measurement =
        evaluatePointPlaneMeasurement(state, point_L, normal_W, plane_point_W);

    // Point at x=1, plane at x=2, residual = 1 - 2 = -1
    EXPECT_NEAR(measurement.residual, -1.0, 1e-9);
    EXPECT_NEAR(measurement.H(0, 3), 1.0, 1e-9);  // H_p(0) = n_x = 1
    EXPECT_NEAR(measurement.H(0, 4), 0.0, 1e-9);  // H_p(1) = n_y = 0
    EXPECT_NEAR(measurement.H(0, 5), 0.0, 1e-9);  // H_p(2) = n_z = 0
}

// J5: Velocity/bias columns exactly zero
TEST(PointPlaneMeasurementTest, J5_VelocityBiasColumnsZero) {
    State15 state;
    state.R_wb = SO3d::exp(Eigen::Vector3d(0.1, 0.2, 0.3));
    state.p_w = Eigen::Vector3d(1.0, 2.0, 3.0);
    state.T_I_L = SE3d(SO3d::exp(Eigen::Vector3d(0.05, -0.1, 0.15)),
                        Eigen::Vector3d(0.1, -0.2, 0.3));

    const Eigen::Vector3d point_L(0.5, -0.3, 1.2);
    const Eigen::Vector3d normal_W = Eigen::Vector3d(-0.2, 0.6, -0.4).normalized();
    const Eigen::Vector3d plane_point_W(0.1, 0.2, 0.3);

    const auto measurement =
        evaluatePointPlaneMeasurement(state, point_L, normal_W, plane_point_W);

    // Velocity columns (6-8) should be zero
    EXPECT_NEAR(measurement.H(0, 6), 0.0, 1e-12);
    EXPECT_NEAR(measurement.H(0, 7), 0.0, 1e-12);
    EXPECT_NEAR(measurement.H(0, 8), 0.0, 1e-12);

    // Accel bias columns (9-11) should be zero
    EXPECT_NEAR(measurement.H(0, 9), 0.0, 1e-12);
    EXPECT_NEAR(measurement.H(0, 10), 0.0, 1e-12);
    EXPECT_NEAR(measurement.H(0, 11), 0.0, 1e-12);

    // Gyro bias columns (12-14) should be zero
    EXPECT_NEAR(measurement.H(0, 12), 0.0, 1e-12);
    EXPECT_NEAR(measurement.H(0, 13), 0.0, 1e-12);
    EXPECT_NEAR(measurement.H(0, 14), 0.0, 1e-12);
}

// J6: Analytic residual equals direct SE(3) residual
TEST(PointPlaneMeasurementTest, J6_AnalyticEqualsDirect) {
    State15 state;
    state.R_wb = SO3d::exp(Eigen::Vector3d(0.15, -0.25, 0.35));
    state.p_w = Eigen::Vector3d(-0.5, 1.2, -0.8);
    state.T_I_L = SE3d(SO3d::exp(Eigen::Vector3d(0.12, 0.08, -0.18)),
                        Eigen::Vector3d(0.15, -0.25, 0.35));

    const Eigen::Vector3d point_L(0.8, -1.2, 0.4);
    const Eigen::Vector3d normal_W = Eigen::Vector3d(0.6, 0.5, -0.6).normalized();
    const Eigen::Vector3d plane_point_W(-1.0, 0.5, 2.0);

    const auto measurement =
        evaluatePointPlaneMeasurement(state, point_L, normal_W, plane_point_W);

    // Compute directly using SE(3) operations
    const Eigen::Vector3d p_I = state.T_I_L * point_L;
    const SE3d T_W_I(state.R_wb, state.p_w);
    const Eigen::Vector3d p_W = T_W_I * p_I;
    const double direct_residual = normal_W.dot(p_W - plane_point_W);

    EXPECT_NEAR(measurement.residual, direct_residual, 1e-12);
}

// J7: Multiple epsilon values for robust finite-difference
TEST(PointPlaneMeasurementTest, J7_MultipleEpsilonValues) {
    State15 state;
    state.R_wb = SO3d::exp(Eigen::Vector3d(0.3, -0.2, 0.1));
    state.p_w = Eigen::Vector3d(0.5, -0.4, 0.3);
    state.T_I_L = SE3d(SO3d::exp(Eigen::Vector3d(-0.15, 0.25, -0.05)),
                        Eigen::Vector3d(0.2, -0.1, 0.15));

    const Eigen::Vector3d point_L(1.1, -0.9, 1.5);
    const Eigen::Vector3d normal_W = Eigen::Vector3d(0.3, 0.7, -0.6).normalized();
    const Eigen::Vector3d plane_point_W(0.2, -0.3, 0.4);

    const auto measurement =
        evaluatePointPlaneMeasurement(state, point_L, normal_W, plane_point_W);

    // Test with multiple epsilon values
    const std::vector<double> epsilons = {1e-5, 1e-6, 1e-7};
    const double tolerance = 1e-4;

    for (double eps : epsilons) {
        for (int idx = 0; idx < 6; ++idx) {
            V15D delta = V15D::Zero();
            delta(idx) = eps;

            State15 plus = state;
            plus.update(delta);

            delta(idx) = -eps;
            State15 minus = state;
            minus.update(delta);

            const double residual_plus =
                computePointPlaneResidual(plus, point_L, normal_W, plane_point_W);
            const double residual_minus =
                computePointPlaneResidual(minus, point_L, normal_W, plane_point_W);
            const double numerical = (residual_plus - residual_minus) / (2.0 * eps);

            EXPECT_NEAR(numerical, measurement.H(0, idx), tolerance)
                << "Jacobian mismatch at index " << idx << " with epsilon " << eps;
        }
    }
}

}  // namespace
}  // namespace fast_lio
