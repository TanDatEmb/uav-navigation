// Copyright 2026 TanDatEmb.

#include "fast_lio/estimator/ikfom_estimator.hpp"

#include "fast_lio/estimator/ikfom_state.hpp"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <stdexcept>

namespace fast_lio {

namespace {

// IKFoM stores raw function pointers for its measurement callbacks, so the
// active estimator impl is passed through a thread-local pointer during update.
thread_local IkfomEstimatorImpl* g_current_impl = nullptr;

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

// Forward declaration of the IKFoM measurement callback installed in BindProcessModel().
static Eigen::Matrix<double, Eigen::Dynamic, 1> h_dyn_share_model(
    IkfomState& s, esekfom::dyn_share_datastruct<double>& dyn_share);

struct IkfomEstimatorImpl {
    esekfom::esekf<IkfomState, 12, IkfomInput> kf;
    Eigen::Matrix<double, 12, 12> process_noise_cov;
    double lidar_information = 200.0;
    IkfomMeasurementProvider measurement_provider;
    int last_measurement_count = 0;
    double limit[15] = {};

    IkfomEstimatorImpl() : kf(), process_noise_cov(Eigen::Matrix<double, 12, 12>::Zero()) {
        // Convergence thresholds for the error-state DOF order used by IKFoM:
        // [pos, rot, vel, bg, ba].
        for (int i = 0; i < 3; ++i) {
            limit[i] = 0.015;                     // position [m]
            limit[i + 3] = 0.01 * M_PI / 180.0;   // orientation [rad]
            limit[i + 6] = 1.0;                   // velocity
            limit[i + 9] = 1.0;                   // gyro bias
            limit[i + 12] = 1.0;                  // accel bias
        }
    }

    void BindProcessModel() {
        kf.init_dyn_share(get_f, df_dx, df_dw, &h_dyn_share_model, 3, limit);
    }

    void ResetTo(const IkfomState& x, const Eigen::Matrix<double, 15, 15>& P) {
        kf = esekfom::esekf<IkfomState, 12, IkfomInput>(x, P);
        BindProcessModel();
    }
};

static Eigen::Matrix<double, Eigen::Dynamic, 1> h_dyn_share_model(
    IkfomState& s, esekfom::dyn_share_datastruct<double>& dyn_share) {
    dyn_share.valid = false;
    if (!g_current_impl || !g_current_impl->measurement_provider) {
        return Eigen::Matrix<double, Eigen::Dynamic, 1>();
    }

    const State15 state = FromIkfomState(s);
    Eigen::MatrixXd H;
    Eigen::VectorXd residuals;
    Eigen::MatrixXd R;
    if (!g_current_impl->measurement_provider(state, H, residuals, R) || residuals.size() == 0) {
        return Eigen::Matrix<double, Eigen::Dynamic, 1>();
    }

    g_current_impl->last_measurement_count = static_cast<int>(residuals.size());
    dyn_share.valid = true;
    dyn_share.z = Eigen::Matrix<double, Eigen::Dynamic, 1>::Zero(residuals.size());
    dyn_share.h_x = H;
    dyn_share.h_v = Eigen::MatrixXd::Identity(residuals.size(), residuals.size());
    dyn_share.R = R;

    // The provider gives residuals (z - h). We return them as the predicted
    // measurement h so that IKFoM's innovation (z - h) becomes -residual,
    // which drives the residual to zero.
    return residuals;
}

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

void IkfomEstimator::setMeasurementProvider(IkfomMeasurementProvider provider) {
    impl_->measurement_provider = std::move(provider);
}

IkfomUpdateResult IkfomEstimator::update(int max_iterations) {
    IkfomUpdateResult result;
    result.status = IkfomUpdateStatus::kNoMeasurements;
    result.converged = false;
    result.iterations = 0;
    result.measurements = 0;

    if (!impl_->measurement_provider) {
        return result;
    }

    // The thread-local pointer lets the raw IKFoM callback reach the provider.
    g_current_impl = impl_.get();
    impl_->last_measurement_count = 0;

    const State15 state_before = getState();

    // Re-initialise with the requested iteration count. IKFoM keeps the prior
    // state/covariance and only updates function pointers and maximum_iter.
    impl_->kf.init_dyn_share(get_f, df_dx, df_dw, &h_dyn_share_model, max_iterations,
                             impl_->limit);
    impl_->kf.update_iterated_dyn_share();

    const State15 state_after = getState();
    g_current_impl = nullptr;

    result.measurements = impl_->last_measurement_count;

    if (result.measurements == 0) {
        result.status = IkfomUpdateStatus::kNoMeasurements;
        return result;
    }

    if (!state_after.p_w.allFinite() || !state_after.v_w.allFinite() ||
        !state_after.R_wb.matrix().allFinite()) {
        result.status = IkfomUpdateStatus::kInsufficientMeasurements;
        return result;
    }

    const Eigen::Vector3d dpos = state_after.p_w - state_before.p_w;
    const double dpos_norm = dpos.norm();
    const Eigen::Matrix3d dR =
        state_before.R_wb.matrix().transpose() * state_after.R_wb.matrix();
    const double drot_angle = Eigen::AngleAxisd(dR).angle();

    result.final_delta_position_m = dpos_norm;
    result.final_delta_rotation_rad = drot_angle;
    result.converged = (dpos_norm < 0.015) && (drot_angle < 0.01 * M_PI / 180.0);
    result.iterations = max_iterations;
    result.status = IkfomUpdateStatus::kSuccess;
    return result;
}

void IkfomEstimator::reset() {
    Eigen::Matrix<double, 15, 15> P = Eigen::Matrix<double, 15, 15>::Identity();

    // Initial uncertainties in the IKFoM error-state DOF order:
    // [pos, rot, vel, bg, ba]. These match the custom IESKF magnitudes but are
    // reordered because IKFoM places position first.
    P.block<3, 3>(0, 0) = std::pow(0.1, 2) * Eigen::Matrix3d::Identity();                  // position
    P.block<3, 3>(3, 3) = std::pow(5.0 * M_PI / 180.0, 2) * Eigen::Matrix3d::Identity();  // orientation
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
