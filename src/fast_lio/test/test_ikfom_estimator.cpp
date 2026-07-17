// Copyright 2026 TanDatEmb.

#include <gtest/gtest.h>

#include <Eigen/Cholesky>
#include <Eigen/Geometry>

#include "fast_lio/estimator/ikfom_estimator.hpp"

namespace fast_lio {
namespace {

TEST(IkfomEstimatorTest, ResetProducesFiniteStateAndCovariance) {
    IkfomEstimator estimator;

    const State15 state = estimator.getState();
    EXPECT_TRUE(state.p_w.allFinite());
    EXPECT_TRUE(state.v_w.allFinite());
    EXPECT_TRUE(state.R_wb.matrix().allFinite());
    EXPECT_TRUE(state.b_a.allFinite());
    EXPECT_TRUE(state.b_w.allFinite());

    const Eigen::Matrix<double, 15, 15> P = estimator.getCovariance();
    EXPECT_TRUE(P.allFinite());
    Eigen::LLT<Eigen::Matrix<double, 15, 15>> llt(P);
    EXPECT_EQ(llt.info(), Eigen::Success)
        << "Initial covariance must be symmetric positive-definite";
}

TEST(IkfomEstimatorTest, StateRoundTripPreservesValues) {
    IkfomEstimator estimator;

    State15 state_in;
    state_in.p_w = Eigen::Vector3d(1.0, 2.0, 3.0);
    state_in.v_w = Eigen::Vector3d(0.1, 0.2, 0.3);
    state_in.R_wb = SO3d(Eigen::AngleAxisd(0.5, Eigen::Vector3d::UnitZ()).toRotationMatrix());
    state_in.b_a = Eigen::Vector3d(0.01, 0.02, 0.03);
    state_in.b_w = Eigen::Vector3d(0.001, 0.002, 0.003);

    estimator.setState(state_in);
    const State15 state_out = estimator.getState();

    EXPECT_TRUE(state_out.p_w.isApprox(state_in.p_w, 1e-12));
    EXPECT_TRUE(state_out.v_w.isApprox(state_in.v_w, 1e-12));
    EXPECT_TRUE(state_out.R_wb.matrix().isApprox(state_in.R_wb.matrix(), 1e-12));
    EXPECT_TRUE(state_out.b_a.isApprox(state_in.b_a, 1e-12));
    EXPECT_TRUE(state_out.b_w.isApprox(state_in.b_w, 1e-12));
}

TEST(IkfomEstimatorTest, GravityInitializationAlignsZAxis) {
    IkfomEstimator estimator;

    // Sensor at rest with z-axis pointing down in a world with z-up gravity.
    const Eigen::Vector3d mean_acc(0.0, 0.0, 9.80665);
    estimator.initWithGravity(mean_acc);

    const State15 state = estimator.getState();
    // Body z-axis should align with world z-axis (gravity vector is -z in world).
    const Eigen::Vector3d body_z_in_world = state.R_wb * Eigen::Vector3d::UnitZ();
    EXPECT_TRUE(body_z_in_world.isApprox(Eigen::Vector3d::UnitZ(), 1e-6));
}

TEST(IkfomEstimatorTest, StationaryImuHasZeroNetAcceleration) {
    IkfomEstimator estimator;
    estimator.reset();

    // IMU reports exactly gravity when stationary and aligned with the world.
    // The net acceleration should be zero, so velocity remains unchanged.
    const IMUData imu(Eigen::Vector3d(0.0, 0.0, 9.80665), Eigen::Vector3d::Zero(), 0.0);
    const double dt = 0.01;

    estimator.predict(imu, dt);

    const State15 state = estimator.getState();
    EXPECT_TRUE(state.p_w.allFinite());
    EXPECT_TRUE(state.v_w.allFinite());

    // Biases should remain zero after a single zero-bias propagation.
    EXPECT_TRUE(state.b_a.isApprox(Eigen::Vector3d::Zero(), 1e-12));
    EXPECT_TRUE(state.b_w.isApprox(Eigen::Vector3d::Zero(), 1e-12));

    // Net acceleration is zero; velocity and position should not drift.
    EXPECT_NEAR(state.v_w.z(), 0.0, 1e-9);
    EXPECT_NEAR(state.p_w.z(), 0.0, 1e-12);
}

TEST(IkfomEstimatorTest, AcceleratingImuPropagatesKinematics) {
    IkfomEstimator estimator;
    estimator.reset();

    // Body reports 2g upward; net world acceleration is +g upward.
    const IMUData imu(Eigen::Vector3d(0.0, 0.0, 2.0 * 9.80665), Eigen::Vector3d::Zero(), 0.0);
    const double dt = 0.01;

    // First step from rest: IKFoM uses explicit Euler, so velocity updates
    // but position remains zero because the position derivative was computed
    // with zero velocity.
    estimator.predict(imu, dt);
    State15 state = estimator.getState();
    EXPECT_GT(state.v_w.z(), 0.09);
    EXPECT_LT(state.v_w.z(), 0.11);
    EXPECT_NEAR(state.p_w.z(), 0.0, 1e-12);

    // Second step: position now accumulates using the velocity from step 1.
    estimator.predict(imu, dt);
    state = estimator.getState();
    EXPECT_GT(state.v_w.z(), 0.19);
    EXPECT_LT(state.v_w.z(), 0.21);
    EXPECT_GT(state.p_w.z(), 9.0e-4);
    EXPECT_LT(state.p_w.z(), 1.1e-3);
}

TEST(IkfomEstimatorTest, ProcessNoiseScalesWithConfiguredNoise) {
    Config quiet_config;
    quiet_config.na = 0.0;
    quiet_config.ng = 0.0;
    quiet_config.nba = 0.0;
    quiet_config.nbg = 0.0;

    Config noisy_config = quiet_config;
    noisy_config.na = 1.0;
    noisy_config.ng = 1.0;
    noisy_config.nba = 1.0;
    noisy_config.nbg = 1.0;

    IkfomEstimator quiet_estimator;
    quiet_estimator.configure(quiet_config);

    IkfomEstimator noisy_estimator;
    noisy_estimator.configure(noisy_config);

    const IMUData imu(Eigen::Vector3d(0.0, 0.0, 9.80665), Eigen::Vector3d::Zero(), 0.0);
    const double dt = 0.1;

    quiet_estimator.predict(imu, dt);
    noisy_estimator.predict(imu, dt);

    const Eigen::Matrix<double, 15, 15> quiet_P = quiet_estimator.getCovariance();
    const Eigen::Matrix<double, 15, 15> noisy_P = noisy_estimator.getCovariance();

    EXPECT_TRUE((noisy_P.diagonal().array() >= quiet_P.diagonal().array()).all())
        << "Noisier configuration must not shrink diagonal covariance";
}

TEST(IkfomEstimatorTest, UpdateWithoutProviderReturnsNoMeasurements) {
    IkfomEstimator estimator;
    const auto result = estimator.update(3);
    EXPECT_EQ(result.status, IkfomUpdateStatus::kNoMeasurements);
    EXPECT_FALSE(result.success());
    EXPECT_EQ(result.measurements, 0);
}

TEST(IkfomEstimatorTest, UpdateWithPlaneMeasurementCorrectsPosition) {
    IkfomEstimator estimator;
    estimator.initWithGravity(Eigen::Vector3d(0.0, 0.0, 9.80665));

    // Start 0.1 m above the ground plane with a small roll error.
    State15 state = estimator.getState();
    state.p_w = Eigen::Vector3d(0.0, 0.0, 0.1);
    state.R_wb = SO3d::exp(Eigen::Vector3d(0.01, 0.0, 0.0)) * state.R_wb;
    estimator.setState(state);

    // Synthetic ground-plane measurement at the sensor origin.
    // Residual = height in world; Jacobian follows the IKFoM error order
    // [pos, rot, vel, bg, ba].
    estimator.setMeasurementProvider([](const State15& s, Eigen::MatrixXd& H,
                                        Eigen::VectorXd& residuals,
                                        Eigen::MatrixXd& R) -> bool {
        H.resize(1, 15);
        H.setZero();
        H.block<1, 3>(0, 0) = Eigen::Vector3d::UnitZ().transpose();
        residuals.resize(1);
        residuals(0) = s.p_w.z();
        R.resize(1, 1);
        R(0, 0) = 0.001;
        return true;
    });

    const auto result = estimator.update(3);
    EXPECT_TRUE(result.success()) << "status=" << static_cast<int>(result.status);
    EXPECT_EQ(result.measurements, 1);
    EXPECT_LT(estimator.getState().p_w.z(), 0.01);
}

}  // namespace
}  // namespace fast_lio
