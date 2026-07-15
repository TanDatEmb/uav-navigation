#include "fast_lio/imu_processor.hpp"

#include <algorithm>
#include <cmath>

namespace fast_lio {

namespace {
constexpr double kGravityMagnitude = 9.81;
}

IMUProcessor::IMUProcessor(const Config& config) : config_(config), initialized_(false) {
    mean_acc_ = Eigen::Vector3d::Zero();
    mean_gyro_ = Eigen::Vector3d::Zero();
}

bool IMUProcessor::initialize(const std::deque<IMUData>& imu_buffer) {
    if (initialized_) {
        return true;
    }

    const size_t required_samples = static_cast<size_t>(std::max(1, config_.imu_init_num));
    for (const auto& imu : imu_buffer) {
        if (!imu.acc.allFinite() || !imu.gyro.allFinite() || !std::isfinite(imu.time)) {
            continue;
        }

        init_window_.push_back(imu);
        while (init_window_.size() > required_samples) {
            init_window_.pop_front();
        }
    }

    if (init_window_.size() < required_samples) {
        return false;
    }

    Eigen::Vector3d candidate_mean_acc = Eigen::Vector3d::Zero();
    Eigen::Vector3d candidate_mean_gyro = Eigen::Vector3d::Zero();
    double gyro_square_sum = 0.0;
    for (const auto& imu : init_window_) {
        candidate_mean_acc += imu.acc;
        candidate_mean_gyro += imu.gyro;
        gyro_square_sum += imu.gyro.squaredNorm();
    }
    const double sample_count = static_cast<double>(init_window_.size());
    candidate_mean_acc /= sample_count;
    candidate_mean_gyro /= sample_count;

    double accel_variance_sum = 0.0;
    for (const auto& imu : init_window_) {
        accel_variance_sum += (imu.acc - candidate_mean_acc).squaredNorm();
    }

    const double accel_std = std::sqrt(accel_variance_sum / sample_count);
    const double gyro_rms = std::sqrt(gyro_square_sum / sample_count);
    const double gravity_error = std::abs(candidate_mean_acc.norm() - kGravityMagnitude);
    const double max_accel_std = std::max(0.0, config_.imu_init_accel_std_max);
    const double max_gyro_rms = std::max(0.0, config_.imu_init_gyro_rms_max);
    const double gravity_tolerance = std::max(0.0, config_.imu_init_gravity_tolerance);

    if (!candidate_mean_acc.allFinite() || !candidate_mean_gyro.allFinite() ||
        candidate_mean_acc.norm() < 1e-6 || accel_std > max_accel_std || gyro_rms > max_gyro_rms ||
        gravity_error > gravity_tolerance) {
        return false;
    }

    mean_acc_ = candidate_mean_acc;
    mean_gyro_ = candidate_mean_gyro;
    // FAST-LIO normalizes acceleration magnitude from the accepted stationary
    // window. This prevents a fixed simulator/sensor scale error from becoming
    // a persistent world-frame acceleration after gravity alignment.
    accel_scale_ = kGravityMagnitude / mean_acc_.norm();
    initialized_ = true;
    return true;
}

void IMUProcessor::processIMU(std::shared_ptr<IESKF> kf, const std::deque<IMUData>& imus,
                              double end_time) {
    if (!initialized_ || !kf)
        return;

    // The first processed scan defines the filter epoch. Initialization IMUs are
    // used for gravity alignment but must not be replayed as motion history.
    if (!integration_initialized_) {
        if (!imus.empty()) {
            last_imu_ = imus.back();
        } else {
            last_imu_.acc = mean_acc_;
            last_imu_.gyro = mean_gyro_;
            last_imu_.time = end_time;
        }
        integration_time_ = end_time;
        integration_initialized_ = true;
        return;
    }

    if (end_time <= integration_time_) {
        return;
    }

    double current_time = integration_time_;
    IMUData held_imu = last_imu_;

    for (const auto& imu : imus) {
        if (imu.time <= current_time) {
            held_imu = imu;
            continue;
        }
        if (imu.time > end_time) {
            break;
        }

        const double dt = imu.time - current_time;
        if (dt > 0.0 && dt <= 0.1) {
            // Midpoint measurement reduces zero-order-hold integration error without
            // assigning artificial timestamps to the incoming IMU samples.
            IMUData midpoint;
            midpoint.acc = 0.5 * (held_imu.acc + imu.acc) * accel_scale_;
            midpoint.gyro = 0.5 * (held_imu.gyro + imu.gyro);
            midpoint.time = imu.time;
            kf->predict(midpoint, dt);
        }
        held_imu = imu;
        current_time = imu.time;
    }

    const double dt_remaining = end_time - current_time;
    if (dt_remaining > 0.0 && dt_remaining <= 0.1) {
        IMUData scaled_imu = held_imu;
        scaled_imu.acc *= accel_scale_;
        kf->predict(scaled_imu, dt_remaining);
    }

    held_imu.time = end_time;
    last_imu_ = held_imu;
    integration_time_ = end_time;
}

}  // namespace fast_lio
