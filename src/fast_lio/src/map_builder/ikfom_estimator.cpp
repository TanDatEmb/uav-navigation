// Copyright 2026 TanDatEmb.

#include "fast_lio/estimator/ikfom_estimator.hpp"

#include "fast_lio/estimator/ikfom_state.hpp"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <stdexcept>

namespace fast_lio {

namespace {

// Constant gravity in the z-up LIO world frame [m/s²].
constexpr double kGravityMps2 = 9.80665;
const Eigen::Vector3d kGravityWorld(0.0, 0.0, -kGravityMps2);

// Skew-symmetric matrix of a 3-vector (matches IESKF::skewSymmetric).
Eigen::Matrix3d SkewSymmetric(const Eigen::Vector3d& v) {
    Eigen::Matrix3d M;
    M << 0.0, -v.z(), v.y(), v.z(), 0.0, -v.x(), -v.y(), v.x(), 0.0;
    return M;
}

// 15-DOF process model: ẋ = f(x, u).
// State order in the flattened manifold: [pos, rot, vel, bg, ba].
Eigen::Matrix<double, IkfomState::DIM, 1> get_f(IkfomState& s, const IkfomInput& in) {
    Eigen::Matrix<double, IkfomState::DIM, 1> res = Eigen::Matrix<double, IkfomState::DIM, 1>::Zero();

    const Eigen::Vector3d omega = in.gyro - s.bg;
    const Eigen::Vector3d a_body = in.acc - s.ba;
    const Eigen::Vector3d a_world = s.rot * a_body + kGravityWorld;

    res.template segment<3>(0) = s.vel;     // ṗ = v
    res.template segment<3>(3) = omega;      // Ṙ = R * [ω^∧]
    res.template segment<3>(6) = a_world;    // v̇ = R*a + g
    res.template segment<3>(9).setZero();    // ḃg = 0
    res.template segment<3>(12).setZero();   // ḃa = 0

    return res;
}

// Jacobian of f w.r.t. the error state (15×15).
Eigen::Matrix<double, IkfomState::DIM, IkfomState::DOF> df_dx(IkfomState& s,
                                                              const IkfomInput& in) {
    Eigen::Matrix<double, IkfomState::DIM, IkfomState::DOF> jac =
        Eigen::Matrix<double, IkfomState::DIM, IkfomState::DOF>::Zero();

    const Eigen::Vector3d a_body = in.acc - s.ba;

    // d(ṗ)/d(v)
    jac.template block<3, 3>(0, 6) = Eigen::Matrix3d::Identity();

    // d(Ṙ)/d(bg)
    jac.template block<3, 3>(3, 9) = -Eigen::Matrix3d::Identity();

    // d(v̇)/d(rot) = -R * [a_body^∧]
    jac.template block<3, 3>(6, 3) = -s.rot.toRotationMatrix() * SkewSymmetric(a_body);

    // d(v̇)/d(ba) = -R
    jac.template block<3, 3>(6, 12) = -s.rot.toRotationMatrix();

    return jac;
}

// Jacobian of f w.r.t. process noise (15×12).
// Noise order: [ng, na, nbg, nba].
Eigen::Matrix<double, IkfomState::DIM, 12> df_dw(IkfomState& s, const IkfomInput& /*in*/) {
    Eigen::Matrix<double, IkfomState::DIM, 12> jac =
        Eigen::Matrix<double, IkfomState::DIM, 12>::Zero();

    // d(Ṙ)/d(ng)
    jac.template block<3, 3>(3, 0) = -Eigen::Matrix3d::Identity();

    // d(v̇)/d(na)
    jac.template block<3, 3>(6, 3) = -s.rot.toRotationMatrix();

    // d(ḃg)/d(nbg)
    jac.template block<3, 3>(9, 6) = Eigen::Matrix3d::Identity();

    // d(ḃa)/d(nba)
    jac.template block<3, 3>(12, 9) = Eigen::Matrix3d::Identity();

    return jac;
}

// Build continuous-time process-noise covariance from project-wide Config.
Eigen::Matrix<double, 12, 12> BuildProcessNoiseCovariance(const Config& config) {
    Eigen::Matrix<double, 12, 12> Q = Eigen::Matrix<double, 12, 12>::Zero();

    const double gyro_var = config.ng * config.ng;
    const double accel_var = config.na * config.na;
    const double gyro_bias_var = config.nbg * config.nbg;
    const double accel_bias_var = config.nba * config.nba;

    Q.block<3, 3>(0, 0) = gyro_var * Eigen::Matrix3d::Identity();
    Q.block<3, 3>(3, 3) = accel_var * Eigen::Matrix3d::Identity();
    Q.block<3, 3>(6, 6) = gyro_bias_var * Eigen::Matrix3d::Identity();
    Q.block<3, 3>(9, 9) = accel_bias_var * Eigen::Matrix3d::Identity();

    return Q;
}

}  // namespace

struct IkfomEstimatorImpl {
    esekfom::esekf<IkfomState, 12, IkfomInput> kf;
    Eigen::Matrix<double, 12, 12> process_noise_cov;
    double lidar_information = 200.0;

    IkfomEstimatorImpl() : kf(), process_noise_cov(Eigen::Matrix<double, 12, 12>::Zero()) {}

    void BindProcessModel() {
        double limit[15] = {};
        kf.init_dyn_runtime_share(get_f, df_dx, df_dw, 1, limit);
    }

    void ResetTo(const IkfomState& x, const Eigen::Matrix<double, 15, 15>& P) {
        kf = esekfom::esekf<IkfomState, 12, IkfomInput>(x, P);
        BindProcessModel();
    }
};

IkfomEstimator::IkfomEstimator() : impl_(std::make_unique<IkfomEstimatorImpl>()) {
    reset();
}

IkfomEstimator::~IkfomEstimator() = default;

void IkfomEstimator::configure(const Config& config) {
    impl_->lidar_information = std::isfinite(config.lidar_cov_inv) && config.lidar_cov_inv > 0.0
                                   ? config.lidar_cov_inv
                                   : 200.0;

    impl_->process_noise_cov = BuildProcessNoiseCovariance(config);
}

void IkfomEstimator::setState(const State15& state) {
    impl_->ResetTo(ToIkfomState(state), impl_->kf.get_P());
}

State15 IkfomEstimator::getState() const {
    return FromIkfomState(impl_->kf.get_x());
}

void IkfomEstimator::predict(const IMUData& imu, double dt) {
    if (dt <= 0.0 || dt > 0.1) {
        std::cerr << "[IkfomEstimator] Invalid dt: " << dt << std::endl;
        return;
    }

    const IkfomInput input = ToIkfomInput(imu);
    double dt_mut = dt;  // IKFoM predict takes a non-const reference.
    impl_->kf.predict(dt_mut, impl_->process_noise_cov, input);
}

IkfomUpdateResult IkfomEstimator::update(int /*max_iterations*/) {
    IkfomUpdateResult result;
    result.status = IkfomUpdateStatus::kNotImplemented;
    result.converged = false;
    result.iterations = 0;
    result.measurements = 0;
    return result;
}

void IkfomEstimator::reset() {
    Eigen::Matrix<double, 15, 15> P = Eigen::Matrix<double, 15, 15>::Identity();

    // Initial uncertainties match the custom IESKF defaults.
    P.block<3, 3>(0, 0) = std::pow(5.0 * M_PI / 180.0, 2) * Eigen::Matrix3d::Identity();  // orientation
    P.block<3, 3>(3, 3) = std::pow(0.1, 2) * Eigen::Matrix3d::Identity();                  // position
    P.block<3, 3>(6, 6) = std::pow(0.1, 2) * Eigen::Matrix3d::Identity();                  // velocity
    P.block<3, 3>(9, 9) = std::pow(0.01, 2) * Eigen::Matrix3d::Identity();                 // gyro bias
    P.block<3, 3>(12, 12) = std::pow(0.1, 2) * Eigen::Matrix3d::Identity();                // accel bias

    impl_->ResetTo(IkfomState(), P);
}

void IkfomEstimator::initWithGravity(const Eigen::Vector3d& mean_acc) {
    if (!mean_acc.allFinite() || mean_acc.norm() < 1e-6) {
        std::cerr << "[IkfomEstimator] Invalid mean acceleration for gravity alignment"
                  << std::endl;
        return;
    }

    const Eigen::Vector3d specific_force_body = mean_acc.normalized();
    const Eigen::Vector3d specific_force_world = -kGravityWorld.normalized();
    const double alignment = std::clamp(specific_force_body.dot(specific_force_world), -1.0, 1.0);

    Eigen::Quaterniond q_world_body;
    if (alignment > 1.0 - 1e-12) {
        q_world_body = Eigen::Quaterniond::Identity();
    } else if (alignment < -1.0 + 1e-12) {
        q_world_body = Eigen::Quaterniond(
            Eigen::AngleAxisd(std::acos(-1.0), specific_force_body.unitOrthogonal()));
    } else {
        const Eigen::Vector3d rotation_axis = specific_force_body.cross(specific_force_world);
        q_world_body = Eigen::Quaterniond(1.0 + alignment, rotation_axis.x(), rotation_axis.y(),
                                          rotation_axis.z())
                           .normalized();
    }

    IkfomState s = impl_->kf.get_x();
    s.rot = ikfom_so3(q_world_body.normalized());
    s.bg = Eigen::Vector3d::Zero();
    s.ba = Eigen::Vector3d::Zero();
    impl_->ResetTo(s, impl_->kf.get_P());
}

Eigen::Matrix<double, 15, 15> IkfomEstimator::getCovariance() const {
    return impl_->kf.get_P();
}

}  // namespace fast_lio
