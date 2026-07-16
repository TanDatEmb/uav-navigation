// Copyright (c) 2026, LeTanDat
// SPDX-License-Identifier: BSD-3-Clause

#include "fast_lio/imu_processor.hpp"

#include <algorithm>
#include <cmath>

namespace fast_lio {

namespace {
constexpr double kGravityMagnitude = 9.80665;

/// Gravity in LIO world frame (z-up).
const Eigen::Vector3d kGravityW(0.0, 0.0, -kGravityMagnitude);
}  // namespace

IMUProcessor::IMUProcessor(const Config& config) : config_(config), initialized_(false) {
    mean_acc_ = Eigen::Vector3d::Zero();
    mean_gyro_ = Eigen::Vector3d::Zero();
}

bool IMUProcessor::initialize(const std::deque<IMUData>& imu_buffer) {
    if (initialized_) return true;

    const size_t required_samples = static_cast<size_t>(std::max(1, config_.imu_init_num));
    for (const auto& imu : imu_buffer) {
        if (!imu.acc.allFinite() || !imu.gyro.allFinite() || !std::isfinite(imu.time)) continue;
        init_window_.push_back(imu);
        while (init_window_.size() > required_samples) init_window_.pop_front();
    }

    if (init_window_.size() < required_samples) return false;

    Eigen::Vector3d candidate_mean_acc = Eigen::Vector3d::Zero();
    Eigen::Vector3d candidate_mean_gyro = Eigen::Vector3d::Zero();
    double gyro_square_sum = 0.0;
    for (const auto& imu : init_window_) {
        candidate_mean_acc += imu.acc;
        candidate_mean_gyro += imu.gyro;
        gyro_square_sum += imu.gyro.squaredNorm();
    }
    const double n = static_cast<double>(init_window_.size());
    candidate_mean_acc /= n;
    candidate_mean_gyro /= n;

    double accel_variance_sum = 0.0;
    for (const auto& imu : init_window_)
        accel_variance_sum += (imu.acc - candidate_mean_acc).squaredNorm();

    const double accel_std = std::sqrt(accel_variance_sum / n);
    const double gyro_rms = std::sqrt(gyro_square_sum / n);
    const double gravity_error = std::abs(candidate_mean_acc.norm() - kGravityMagnitude);

    if (!candidate_mean_acc.allFinite() || !candidate_mean_gyro.allFinite() ||
        candidate_mean_acc.norm() < 1e-6 ||
        accel_std > std::max(0.0, config_.imu_init_accel_std_max) ||
        gyro_rms > std::max(0.0, config_.imu_init_gyro_rms_max) ||
        gravity_error > std::max(0.0, config_.imu_init_gravity_tolerance))
        return false;

    mean_acc_ = candidate_mean_acc;
    mean_gyro_ = candidate_mean_gyro;
    accel_scale_ = kGravityMagnitude / mean_acc_.norm();
    initialized_ = true;
    return true;
}

ImuInitializationDiagnostics IMUProcessor::initializationDiagnostics() const {
    ImuInitializationDiagnostics diag;
    diag.required_samples = static_cast<std::size_t>(std::max(1, config_.imu_init_num));
    diag.collected_samples = init_window_.size();
    diag.initialized = initialized_;

    if (init_window_.empty()) {
        return diag;
    }

    Eigen::Vector3d mean_acc = Eigen::Vector3d::Zero();
    Eigen::Vector3d mean_gyro = Eigen::Vector3d::Zero();
    double gyro_square_sum = 0.0;
    for (const auto& imu : init_window_) {
        mean_acc += imu.acc;
        mean_gyro += imu.gyro;
        gyro_square_sum += imu.gyro.squaredNorm();
    }
    const double n = static_cast<double>(init_window_.size());
    mean_acc /= n;
    mean_gyro /= n;

    double accel_variance_sum = 0.0;
    for (const auto& imu : init_window_) {
        accel_variance_sum += (imu.acc - mean_acc).squaredNorm();
    }

    diag.mean_acceleration = mean_acc;
    diag.mean_gyro = mean_gyro;
    diag.acceleration_norm = mean_acc.norm();
    diag.gyro_norm = mean_gyro.norm();
    diag.accel_std = std::sqrt(accel_variance_sum / n);
    diag.gyro_rms = std::sqrt(gyro_square_sum / n);
    diag.gravity_error = std::abs(diag.acceleration_norm - kGravityMagnitude);

    const bool accel_ok = diag.accel_std <= std::max(0.0, config_.imu_init_accel_std_max);
    const bool gyro_ok = diag.gyro_rms <= std::max(0.0, config_.imu_init_gyro_rms_max);
    const bool gravity_ok = diag.gravity_error <= std::max(0.0, config_.imu_init_gravity_tolerance);
    diag.stationary = accel_ok && gyro_ok && gravity_ok && diag.acceleration_norm > 1e-6;

    return diag;
}

std::vector<IMUData> IMUProcessor::buildContinuousSequence(
    const std::deque<IMUData>& imus,
    const std::optional<IMUData>& imu_before,
    const std::optional<IMUData>& imu_after) const {
    std::vector<IMUData> seq;
    constexpr double kEps = 1e-9;

    auto appendIfNew = [&](const IMUData& s) {
        if (seq.empty() || std::abs(s.time - seq.back().time) > kEps) {
            seq.push_back(s);
        }
    };

    if (imu_before.has_value()) appendIfNew(*imu_before);
    for (const auto& imu : imus) appendIfNew(imu);
    if (imu_after.has_value()) appendIfNew(*imu_after);

    return seq;
}

IMUData IMUProcessor::interpolateImu(const IMUData& a, const IMUData& b, double time) const {
    const double dt = b.time - a.time;
    if (dt <= 1e-12) return a;
    const double alpha = (time - a.time) / dt;
    IMUData result;
    result.time = time;
    result.acc = (1.0 - alpha) * a.acc + alpha * b.acc;
    result.gyro = (1.0 - alpha) * a.gyro + alpha * b.gyro;
    return result;
}

ImuTrajectory IMUProcessor::propagate(std::shared_ptr<IESKF> kf,
                                       const std::deque<IMUData>& imus,
                                       double scan_start_time,
                                       double scan_end_time,
                                       const std::optional<IMUData>& imu_before,
                                       const std::optional<IMUData>& imu_after) {
    ImuTrajectory trajectory;

    if (!initialized_ || !kf) return trajectory;

    // Build continuous, deduplicated IMU sequence
    auto seq = buildContinuousSequence(imus, imu_before, imu_after);

    // First scan: initialize integration epoch
    if (!integration_initialized_) {
        if (!seq.empty()) {
            last_imu_ = seq.back();
        }
        integration_time_ = scan_end_time;
        integration_initialized_ = true;
        const auto& state = kf->getState();
        ImuTrajectoryKnot knot;
        knot.timestamp_s = scan_start_time;
        knot.R_W_I = state.R_wb;
        knot.p_W_I = state.p_w;
        knot.v_W_I = state.v_w;
        if (!seq.empty()) {
            knot.omega_I = seq.back().gyro - state.b_w;
            knot.a_W = state.R_wb * (seq.back().acc * accel_scale_ - state.b_a) + kGravityW;
        }
        trajectory.append(knot);
        return trajectory;
    }

    if (scan_end_time <= integration_time_) return trajectory;

    // If seq is empty, propagate with last_imu_ only
    if (seq.empty()) {
        const double dt = scan_end_time - integration_time_;
        if (dt > 0.0 && dt <= 0.1) {
            IMUData scaled = last_imu_;
            scaled.acc *= accel_scale_;
            kf->predict(scaled, dt);
        }
        last_imu_.time = scan_end_time;
        integration_time_ = scan_end_time;
        // Add knot at scan_end
        const auto& state = kf->getState();
        ImuTrajectoryKnot knot;
        knot.timestamp_s = scan_end_time;
        knot.R_W_I = state.R_wb;
        knot.p_W_I = state.p_w;
        knot.v_W_I = state.v_w;
        trajectory.append(knot);
        return trajectory;
    }

    // Add initial knot at scan_start using current state
    {
        const auto& state = kf->getState();
        ImuTrajectoryKnot knot;
        knot.timestamp_s = scan_start_time;
        knot.R_W_I = state.R_wb;
        knot.p_W_I = state.p_w;
        knot.v_W_I = state.v_w;
        // Use last_imu_ for omega/acc at scan_start
        knot.omega_I = last_imu_.gyro - state.b_w;
        knot.a_W = state.R_wb * (last_imu_.acc * accel_scale_ - state.b_a) + kGravityW;
        trajectory.append(knot);
    }

    // Forward propagation through IMU sequence
    double current_time = integration_time_;
    IMUData held_imu = last_imu_;

    for (const auto& imu : seq) {
        // Skip IMUs before current integration time
        if (imu.time <= current_time) {
            held_imu = imu;
            continue;
        }
        // Stop if past scan_end
        if (imu.time > scan_end_time) break;

        const double dt = imu.time - current_time;
        if (dt > 0.0 && dt <= 0.1) {
            // Midpoint integration
            IMUData midpoint;
            midpoint.acc = 0.5 * (held_imu.acc + imu.acc) * accel_scale_;
            midpoint.gyro = 0.5 * (held_imu.gyro + imu.gyro);
            midpoint.time = imu.time;
            kf->predict(midpoint, dt);

            // Save trajectory knot
            const auto& state = kf->getState();
            ImuTrajectoryKnot knot;
            knot.timestamp_s = imu.time;
            knot.R_W_I = state.R_wb;
            knot.p_W_I = state.p_w;
            knot.v_W_I = state.v_w;
            knot.omega_I = midpoint.gyro - state.b_w;
            knot.a_W = state.R_wb * (midpoint.acc - state.b_a) + kGravityW;
            trajectory.append(knot);
        }

        held_imu = imu;
        current_time = imu.time;
    }

    // Propagate to exact scan_end
    const double dt_remaining = scan_end_time - current_time;
    if (dt_remaining > 0.0 && dt_remaining <= 0.1) {
        IMUData u_end = held_imu;
        if (imu_after.has_value() && imu_after->time > current_time) {
            u_end = interpolateImu(held_imu, *imu_after, scan_end_time);
        }

        // Midpoint between held_imu and u_end
        IMUData u_mid;
        u_mid.acc = 0.5 * (held_imu.acc + u_end.acc) * accel_scale_;
        u_mid.gyro = 0.5 * (held_imu.gyro + u_end.gyro);

        kf->predict(u_mid, dt_remaining);

        // Save final knot at scan_end
        const auto& state = kf->getState();
        ImuTrajectoryKnot knot;
        knot.timestamp_s = scan_end_time;
        knot.R_W_I = state.R_wb;
        knot.p_W_I = state.p_w;
        knot.v_W_I = state.v_w;
        knot.omega_I = u_mid.gyro - state.b_w;
        knot.a_W = state.R_wb * (u_mid.acc - state.b_a) + kGravityW;
        trajectory.append(knot);
    }

    // Carry forward the IMU sample at scan_end. If we have an imu_after sample,
    // interpolate to the exact scan_end time; otherwise keep the latest held
    // sample with the scan_end timestamp for continuous next-scan integration.
    if (imu_after.has_value() && imu_after->time > current_time) {
        last_imu_ = interpolateImu(held_imu, *imu_after, scan_end_time);
    } else {
        last_imu_ = held_imu;
        last_imu_.time = scan_end_time;
    }
    integration_time_ = scan_end_time;

    return trajectory;
}

}  // namespace fast_lio