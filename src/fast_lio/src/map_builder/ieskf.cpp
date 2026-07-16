#include "fast_lio/ieskf.hpp"

#include <algorithm>
#include <cmath>
#include <iostream>

namespace fast_lio {

IESKF::IESKF() {
    reset();
    configure(Config{});
}

void IESKF::configure(const Config& config) {
    noise_params_.gyro_noise = std::max(0.0, config.ng);
    noise_params_.accel_noise = std::max(0.0, config.na);
    noise_params_.gyro_bias_rw = std::max(0.0, config.nbg);
    noise_params_.accel_bias_rw = std::max(0.0, config.nba);
    lidar_information_ = std::isfinite(config.lidar_cov_inv) && config.lidar_cov_inv > 0.0
                             ? config.lidar_cov_inv
                             : 200.0;
}

void IESKF::reset() {
    x_ = State15();

    // Initialize covariance
    P_.setZero();

    // Initial uncertainties (diagonal)
    // δθ: orientation uncertainty (~5 degrees)
    P_.block<3, 3>(0, 0) = std::pow(5.0 * M_PI / 180.0, 2) * Eigen::Matrix3d::Identity();

    // δp: position uncertainty (~0.1m)
    P_.block<3, 3>(3, 3) = std::pow(0.1, 2) * Eigen::Matrix3d::Identity();

    // δv: velocity uncertainty (~0.1 m/s)
    P_.block<3, 3>(6, 6) = std::pow(0.1, 2) * Eigen::Matrix3d::Identity();

    // δb_a: accel bias uncertainty (~0.1 m/s²)
    P_.block<3, 3>(9, 9) = std::pow(0.1, 2) * Eigen::Matrix3d::Identity();

    // δb_ω: gyro bias uncertainty (~0.01 rad/s ≈ 0.57 deg/s)
    P_.block<3, 3>(12, 12) = std::pow(0.01, 2) * Eigen::Matrix3d::Identity();
}

void IESKF::initWithGravity(const Eigen::Vector3d& mean_acc) {
    // At rest, specific force points opposite world gravity. Reject a
    // degenerate mean instead of normalizing it into NaNs.
    if (!mean_acc.allFinite() || mean_acc.norm() < 1e-6) {
        std::cerr << "[IESKF] Invalid mean acceleration for gravity alignment" << std::endl;
        return;
    }

    const Eigen::Vector3d specific_force_body = mean_acc.normalized();
    const Eigen::Vector3d specific_force_world = -GRAVITY_WORLD_.normalized();
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
    x_.R_wb = SO3d(q_world_body.normalized().toRotationMatrix());

    // Reset biases; the stationary gyro mean is applied by MapBuilder.
    x_.b_a = Eigen::Vector3d::Zero();
    x_.b_w = Eigen::Vector3d::Zero();
}

void IESKF::predict(const IMUData& imu, double dt) {
    if (dt <= 0 || dt > 0.1) {
        std::cerr << "[IESKF] Invalid dt: " << dt << std::endl;
        return;
    }

    // Propagate state
    propagateState(imu, dt);

    // Propagate covariance (error-state EKF)
    auto Phi = computeStateTransition(imu, dt);
    auto Q = computeProcessNoise(dt);

    P_ = Phi * P_ * Phi.transpose() + Q;

    // Symmetrize
    P_ = 0.5 * (P_ + P_.transpose());

    // Clamp variances only. Element-wise clamping would corrupt valid negative
    // cross-covariances and can make the matrix inconsistent.
    for (int i = 0; i < P_.rows(); ++i) {
        P_(i, i) = std::clamp(P_(i, i), 1e-12, 1e6);
    }
}

void IESKF::propagateState(const IMUData& imu, double dt) {
    // Bias-corrected measurements
    Eigen::Vector3d a_unbiased = imu.acc - x_.b_a;
    Eigen::Vector3d w_unbiased = imu.gyro - x_.b_w;

    // Rotation propagation. SO3::exp handles a zero angular rate without the
    // undefined normalization performed by Eigen::AngleAxis on a zero vector.
    x_.R_wb = x_.R_wb * SO3d::exp(w_unbiased * dt);

    // Acceleration in world frame
    Eigen::Vector3d a_world = x_.R_wb * a_unbiased + GRAVITY_WORLD_;

    // Position and velocity (midpoint integration)
    x_.p_w += x_.v_w * dt + 0.5 * a_world * dt * dt;
    x_.v_w += a_world * dt;

    // Biases: random walk (no deterministic propagation)
    // x_.b_a unchanged
    // x_.b_w unchanged
}

Eigen::Matrix<double, 15, 15> IESKF::computeStateTransition(const IMUData& imu, double dt) const {
    // Error-state transition matrix Φ
    // δx_dot = F * δx + w

    Eigen::Matrix<double, 15, 15> Phi = Eigen::Matrix<double, 15, 15>::Identity();

    // Bias-corrected accel
    Eigen::Vector3d a_unbiased = imu.acc - x_.b_a;

    // Rotation error: δθ_dot = -[ω]× δθ - δb_ω
    Eigen::Vector3d w_unbiased = imu.gyro - x_.b_w;
    Phi.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity() - skewSymmetric(w_unbiased) * dt;
    Phi.block<3, 3>(0, 12) = -Eigen::Matrix3d::Identity() * dt;  // δb_ω → δθ

    // Position error: δp_dot = δv
    Phi.block<3, 3>(3, 6) = Eigen::Matrix3d::Identity() * dt;

    // Velocity error: δv_dot = -R * [a]× * δθ - R * δb_a
    Phi.block<3, 3>(6, 0) = -x_.R_wb.matrix() * skewSymmetric(a_unbiased) * dt;
    Phi.block<3, 3>(6, 9) = -x_.R_wb.matrix() * dt;  // δb_a → δv

    // Bias errors: no cross terms (random walk)

    return Phi;
}

Eigen::Matrix<double, 15, 15> IESKF::computeProcessNoise(double dt) const {
    // Discrete process noise Q_d
    Eigen::Matrix<double, 15, 15> Q = Eigen::Matrix<double, 15, 15>::Zero();

    // Gyro noise: affects rotation
    Q.block<3, 3>(0, 0) = std::pow(noise_params_.gyro_noise, 2) * dt * Eigen::Matrix3d::Identity();

    // Velocity noise: from accel
    Q.block<3, 3>(6, 6) = std::pow(noise_params_.accel_noise, 2) * dt * Eigen::Matrix3d::Identity();

    // Accel bias random walk
    Q.block<3, 3>(9, 9) =
        std::pow(noise_params_.accel_bias_rw, 2) * dt * Eigen::Matrix3d::Identity();

    // Gyro bias random walk
    Q.block<3, 3>(12, 12) =
        std::pow(noise_params_.gyro_bias_rw, 2) * dt * Eigen::Matrix3d::Identity();

    return Q;
}

IeskfUpdateResult IESKF::update(int max_iterations) {
    IeskfUpdateResult result;
    result.status = IeskfUpdateStatus::kNoMeasurements;
    
    if (!loss_func_) {
        std::cerr << "[IESKF] No loss function set!" << std::endl;
        return result;
    }
    if (max_iterations <= 0) {
        result.status = IeskfUpdateStatus::kNoMeasurements;
        return result;
    }

    using Matrix15 = Eigen::Matrix<double, 15, 15>;
    const Matrix15 identity = Matrix15::Identity();

    // Work in information form so the only factorization is 15x15.
    const Eigen::LDLT<Matrix15> prior_ldlt(P_);
    if (prior_ldlt.info() != Eigen::Success) {
        std::cerr << "[IESKF] Prior covariance factorization failed" << std::endl;
        result.status = IeskfUpdateStatus::kPriorFactorizationFailure;
        return result;
    }
    const Matrix15 prior_information = prior_ldlt.solve(identity);
    if (prior_ldlt.info() != Eigen::Success || !prior_information.allFinite()) {
        std::cerr << "[IESKF] Prior information solve failed" << std::endl;
        result.status = IeskfUpdateStatus::kPriorFactorizationFailure;
        return result;
    }

    State15 x_iter = x_;
    V15D accumulated_delta = V15D::Zero();

    for (int iter = 0; iter < max_iterations; ++iter) {
        SharedState15 shared;
        loss_func_(x_iter, shared);
        
        if (!shared.valid) {
            // Check specific validation status for diagnostics
            if (shared.validation_status == MeasurementValidationStatus::kInsufficientMeasurements) {
                std::cerr << "[IESKF] Insufficient measurements (" << shared.num_measurements
                          << " < minimum required)" << std::endl;
                result.status = IeskfUpdateStatus::kInsufficientMeasurements;
                result.measurements = shared.num_measurements;
                return result;
            } else {
                std::cerr << "[IESKF] No valid measurements!" << std::endl;
                result.status = IeskfUpdateStatus::kNoMeasurements;
                return result;
            }
        }

        const auto H = shared.H.topRows(shared.num_measurements);
        const auto residual = shared.b.head(shared.num_measurements);

        const Matrix15 information = prior_information + lidar_information_ * (H.transpose() * H);
        const V15D rhs = -lidar_information_ * (H.transpose() * residual) -
                         prior_information * accumulated_delta;

        Eigen::LDLT<Matrix15> information_ldlt(information);
        if (information_ldlt.info() != Eigen::Success) {
            std::cerr << "[IESKF] Measurement information factorization failed" << std::endl;
            result.status = IeskfUpdateStatus::kMeasurementFactorizationFailure;
            return result;
        }
        const V15D delta_x = information_ldlt.solve(rhs);
        if (information_ldlt.info() != Eigen::Success || !delta_x.allFinite()) {
            std::cerr << "[IESKF] Measurement information solve failed" << std::endl;
            result.status = IeskfUpdateStatus::kNonFiniteCorrection;
            return result;
        }

        x_iter.update(delta_x);
        accumulated_delta += delta_x;
        ++result.iterations;
        result.measurements = shared.num_measurements;

        const bool converged =
            convergence_check_ ? convergence_check_(delta_x) : defaultConvergenceCheck(delta_x);
        if (converged) {
            result.converged = true;
            break;
        }
    }

    // Final linearization for covariance
    SharedState15 shared_final;
    loss_func_(x_iter, shared_final);
    if (!shared_final.valid || shared_final.num_measurements == 0) {
        std::cerr << "[IESKF] Final covariance linearization has no valid measurements"
                  << std::endl;
        result.status = IeskfUpdateStatus::kFinalLinearizationFailure;
        return result;
    }

    const auto H = shared_final.H.topRows(shared_final.num_measurements);
    const Matrix15 posterior_information =
        prior_information + lidar_information_ * (H.transpose() * H);
    Eigen::LDLT<Matrix15> posterior_ldlt(posterior_information);
    if (posterior_ldlt.info() != Eigen::Success) {
        std::cerr << "[IESKF] Posterior covariance factorization failed" << std::endl;
        result.status = IeskfUpdateStatus::kFinalLinearizationFailure;
        return result;
    }

    Matrix15 posterior = posterior_ldlt.solve(identity);
    if (posterior_ldlt.info() != Eigen::Success || !posterior.allFinite()) {
        std::cerr << "[IESKF] Posterior covariance solve failed" << std::endl;
        result.status = IeskfUpdateStatus::kFinalLinearizationFailure;
        return result;
    }

    posterior = 0.5 * (posterior + posterior.transpose());
    for (int i = 0; i < posterior.rows(); ++i) {
        posterior(i, i) = std::clamp(posterior(i, i), 1e-12, 1e6);
    }

    x_ = x_iter;
    P_ = posterior;
    
    result.final_delta_rotation_rad = accumulated_delta.segment<3>(0).norm();
    result.final_delta_position_m = accumulated_delta.segment<3>(3).norm();
    result.status = IeskfUpdateStatus::kSuccess;
    
    return result;
}

bool IESKF::defaultConvergenceCheck(const V15D& delta_x) {
    // Convergence: ‖δx‖ < threshold
    // Rotation part: norm of angle in radians
    // Translation part: norm in meters

    double rot_norm = delta_x.segment<3>(0).norm() * 180.0 / M_PI;  // degrees
    double pos_norm = delta_x.segment<3>(3).norm();                 // meters

    // Threshold: 0.01 degrees rotation, 0.015 meters position
    return (rot_norm < 0.01) && (pos_norm < 0.015);
}

void IESKF::setLossFunction(std::function<void(const State15&, SharedState15&)> func) {
    loss_func_ = func;
}

void IESKF::setConvergenceCheck(std::function<bool(const V15D&)> func) {
    convergence_check_ = func;
}

Eigen::Matrix3d IESKF::skewSymmetric(const Eigen::Vector3d& v) {
    Eigen::Matrix3d M;
    M << 0, -v(2), v(1), v(2), 0, -v(0), -v(1), v(0), 0;
    return M;
}

}  // namespace fast_lio
