#include "fast_lio_core/initialization/imu_initializer.hpp"

#include <algorithm>
#include <cmath>

namespace uav::nav::lio {
namespace {

[[nodiscard]] Eigen::Vector3d componentVariance(const std::deque<ImuSample>& samples,
                                                const Eigen::Vector3d& mean, bool gyro) {
  Eigen::Vector3d variance = Eigen::Vector3d::Zero();
  for (const auto& sample : samples) {
    const Eigen::Vector3d difference =
        (gyro ? sample.angular_velocity_imu_rad_s : sample.linear_acceleration_imu_m_s2) - mean;
    variance += difference.cwiseProduct(difference);
  }
  return variance / static_cast<double>(samples.size());
}

}  // namespace

ImuInitializer::ImuInitializer(ImuInitializerConfig config)
    : config_(config), timestamp_validator_(TimestampValidatorConfig{std::nullopt, true, false}) {}

Status ImuInitializer::addSample(const ImuSample& sample) {
  const Status sample_status = sample.validate();
  if (!sample_status.ok()) {
    return sample_status;
  }
  const Status time_status = timestamp_validator_.validate(sample.time);
  if (!time_status.ok()) {
    return time_status;
  }
  if (config_.maximum_imu_samples == 0) {
    return Status(StatusCode::kInvalidArgument, "IMU initializer maximum sample count is zero");
  }
  samples_.push_back(sample);
  while (samples_.size() > config_.maximum_imu_samples) {
    samples_.pop_front();
  }
  return Status::Ok();
}

InitializationQuality ImuInitializer::quality() const {
  InitializationQuality output;
  output.samples_collected = samples_.size();
  if (samples_.empty()) {
    return output;
  }
  for (const auto& sample : samples_) {
    output.gyro_mean_rad_s += sample.angular_velocity_imu_rad_s;
    output.accel_mean_m_s2 += sample.linear_acceleration_imu_m_s2;
  }
  const double denominator = static_cast<double>(samples_.size());
  output.gyro_mean_rad_s /= denominator;
  output.accel_mean_m_s2 /= denominator;
  output.gyro_variance_rad2_s2 = componentVariance(samples_, output.gyro_mean_rad_s, true);
  output.accel_variance_m2_s4 = componentVariance(samples_, output.accel_mean_m_s2, false);
  output.measured_gravity_norm_m_s2 = output.accel_mean_m_s2.norm();
  output.stationary =
      output.gyro_mean_rad_s.norm() <= config_.maximum_gyro_mean_norm_rad_s &&
      output.gyro_variance_rad2_s2.maxCoeff() <= config_.maximum_gyro_variance_rad2_s2 &&
      output.accel_variance_m2_s4.maxCoeff() <= config_.maximum_accel_variance_m2_s4 &&
      std::abs(output.measured_gravity_norm_m_s2 - config_.gravity_magnitude_m_s2) <=
          config_.maximum_gravity_norm_error_m_s2;
  return output;
}

Result<InitializationResult> ImuInitializer::tryInitialize() const {
  if (!hasEnoughSamples()) {
    return Status(StatusCode::kNotReady, "Not enough IMU samples for initialization");
  }
  const InitializationQuality initialization_quality = quality();
  if (config_.require_stationary && !initialization_quality.stationary) {
    return Status(StatusCode::kInitializationRejected, "IMU stationarity quality gate failed");
  }
  if (initialization_quality.measured_gravity_norm_m_s2 < 1e-6) {
    return Status(StatusCode::kNumericalFailure,
                  "Mean acceleration is too small to initialize gravity");
  }

  InitializationResult result;
  result.quality = initialization_quality;
  result.gyro_bias_rad_s = initialization_quality.gyro_mean_rad_s;
  result.gravity_odom_m_s2 = Eigen::Vector3d(0.0, 0.0, -config_.gravity_magnitude_m_s2);
  result.orientation_odom_imu = Eigen::Quaterniond::FromTwoVectors(
      initialization_quality.accel_mean_m_s2.normalized(), Eigen::Vector3d::UnitZ());
  result.orientation_odom_imu.normalize();
  const Eigen::Vector3d expected_specific_force_imu =
      result.orientation_odom_imu.conjugate() * (-result.gravity_odom_m_s2);
  result.accel_bias_m_s2 = initialization_quality.accel_mean_m_s2 - expected_specific_force_imu;
  return result;
}

std::size_t ImuInitializer::sampleCount() const noexcept { return samples_.size(); }

bool ImuInitializer::hasEnoughSamples() const noexcept {
  return samples_.size() >= config_.minimum_imu_samples;
}

void ImuInitializer::reset() {
  samples_.clear();
  timestamp_validator_.reset();
}

}  // namespace uav::nav::lio
