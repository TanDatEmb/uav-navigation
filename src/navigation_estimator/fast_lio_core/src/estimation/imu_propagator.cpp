#include "fast_lio_core/estimation/imu_propagator.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <vector>

namespace uav::nav::lio {
namespace {

[[nodiscard]] Result<ImuSample> interpolateSample(std::span<const ImuSample> samples,
                                                  const Timestamp& time) {
  const auto upper = std::lower_bound(samples.begin(), samples.end(), time.nanoseconds(),
                                      [](const ImuSample& sample, std::int64_t time_ns) {
                                        return sample.time.nanoseconds() < time_ns;
                                      });
  if (upper == samples.end()) {
    return Status(StatusCode::kInsufficientData, "No IMU sample at or after propagation boundary");
  }
  if (upper->time.nanoseconds() == time.nanoseconds()) {
    return *upper;
  }
  if (upper == samples.begin()) {
    return Status(StatusCode::kInsufficientData, "No IMU sample at or before propagation boundary");
  }
  const auto lower = std::prev(upper);
  const std::int64_t interval_ns = upper->time.nanoseconds() - lower->time.nanoseconds();
  if (interval_ns <= 0) {
    return Status(StatusCode::kTimestampRegression,
                  "IMU sample timestamps must be strictly increasing");
  }
  const double alpha = static_cast<double>(time.nanoseconds() - lower->time.nanoseconds()) /
                       static_cast<double>(interval_ns);
  ImuSample output;
  output.time = time;
  output.angular_velocity_imu_rad_s =
      (1.0 - alpha) * lower->angular_velocity_imu_rad_s + alpha * upper->angular_velocity_imu_rad_s;
  output.linear_acceleration_imu_m_s2 = (1.0 - alpha) * lower->linear_acceleration_imu_m_s2 +
                                        alpha * upper->linear_acceleration_imu_m_s2;
  return output;
}

[[nodiscard]] Eigen::Quaterniond rotationIncrement(const Eigen::Vector3d& angular_velocity_rad_s,
                                                   double dt_s) {
  const Eigen::Vector3d rotation_vector = angular_velocity_rad_s * dt_s;
  const double angle = rotation_vector.norm();
  if (angle < 1e-12) {
    return Eigen::Quaterniond::Identity();
  }
  return Eigen::Quaterniond(Eigen::AngleAxisd(angle, rotation_vector / angle));
}

[[nodiscard]] ImuTrajectoryState trajectoryState(const ManifoldState& state,
                                                 const Timestamp& time) {
  ImuTrajectoryState output;
  output.time = time;
  output.orientation_odom_imu = state.orientation_odom_imu();
  output.position_odom_imu_m = state.position_odom_imu_m();
  output.velocity_odom_imu_m_s = state.velocity_odom_imu_m_s();
  return output;
}

}  // namespace

ImuPropagator::ImuPropagator(ImuPropagatorConfig config) : config_(config) {}

Result<ImuTrajectory> ImuPropagator::propagate(ManifoldState& state,
                                               ManifoldState::Covariance& covariance,
                                               std::span<const ImuSample> samples,
                                               const Timestamp& start_time,
                                               const Timestamp& end_time) const {
  const auto interval = checkedDifference(end_time, start_time);
  if (!interval.ok()) {
    return interval.status();
  }
  if (interval.value().nanoseconds() < 0) {
    return Status(StatusCode::kTimestampRegression, "Propagation end precedes start");
  }
  if (!state.allFinite() || !covariance.allFinite()) {
    return Status(StatusCode::kNumericalFailure,
                  "Initial propagation state or covariance is not finite");
  }
  if (samples.empty()) {
    return Status(StatusCode::kInsufficientData, "No IMU samples supplied for propagation");
  }
  for (std::size_t index = 0; index < samples.size(); ++index) {
    const Status sample_status = samples[index].validate();
    if (!sample_status.ok()) {
      return sample_status;
    }
    if (!samples[index].time.sameClockDomain(start_time)) {
      return Status(StatusCode::kClockDomainMismatch,
                    "IMU and propagation interval use different clocks");
    }
    if (index > 0 && samples[index].time.nanoseconds() <= samples[index - 1].time.nanoseconds()) {
      return Status(StatusCode::kTimestampRegression, "IMU samples must be strictly increasing");
    }
  }
  if (config_.maximum_integration_step_ns <= 0) {
    return Status(StatusCode::kInvalidArgument, "Maximum IMU integration step must be positive");
  }

  const auto start_sample = interpolateSample(samples, start_time);
  if (!start_sample.ok()) {
    return start_sample.status();
  }
  const auto end_sample = interpolateSample(samples, end_time);
  if (!end_sample.ok()) {
    return end_sample.status();
  }

  ManifoldState propagated_state = state;
  ManifoldState::Covariance propagated_covariance = covariance;
  std::vector<ImuSample> integration_samples;
  integration_samples.reserve(samples.size() + 2);
  integration_samples.push_back(start_sample.value());
  for (const auto& sample : samples) {
    if (sample.time.nanoseconds() > start_time.nanoseconds() &&
        sample.time.nanoseconds() < end_time.nanoseconds()) {
      integration_samples.push_back(sample);
    }
  }
  if (end_time.nanoseconds() > start_time.nanoseconds()) {
    integration_samples.push_back(end_sample.value());
  }

  ImuTrajectory trajectory;
  Status trajectory_status = trajectory.addState(trajectoryState(propagated_state, start_time));
  if (!trajectory_status.ok()) {
    return trajectory_status;
  }
  for (std::size_t index = 1; index < integration_samples.size(); ++index) {
    const ImuSample& previous = integration_samples[index - 1];
    const ImuSample& current = integration_samples[index];
    const std::int64_t dt_ns = current.time.nanoseconds() - previous.time.nanoseconds();
    if (dt_ns <= 0 || dt_ns > config_.maximum_integration_step_ns) {
      return Status(StatusCode::kInsufficientData, "IMU integration step is invalid or too large");
    }
    const double dt_s = static_cast<double>(dt_ns) * 1e-9;
    const Eigen::Vector3d corrected_angular_velocity =
        0.5 * (previous.angular_velocity_imu_rad_s + current.angular_velocity_imu_rad_s) -
        propagated_state.gyro_bias_rad_s();
    const Eigen::Vector3d corrected_acceleration =
        0.5 * (previous.linear_acceleration_imu_m_s2 + current.linear_acceleration_imu_m_s2) -
        propagated_state.accel_bias_m_s2();

    const Eigen::Quaterniond orientation_before = propagated_state.orientation_odom_imu();
    Eigen::Quaterniond orientation_mid =
        orientation_before * rotationIncrement(corrected_angular_velocity, 0.5 * dt_s);
    orientation_mid.normalize();
    const Eigen::Vector3d acceleration_odom =
        orientation_mid * corrected_acceleration + propagated_state.gravity_odom_m_s2();
    const Eigen::Vector3d velocity_before = propagated_state.velocity_odom_imu_m_s();
    propagated_state.set_position_odom_imu_m(propagated_state.position_odom_imu_m() +
                                             velocity_before * dt_s +
                                             0.5 * acceleration_odom * dt_s * dt_s);
    propagated_state.set_velocity_odom_imu_m_s(velocity_before + acceleration_odom * dt_s);
    propagated_state.set_orientation_odom_imu(orientation_before *
                                              rotationIncrement(corrected_angular_velocity, dt_s));
    propagated_state.normalize();

    const Status covariance_status = ProcessModel::propagateCovariance(
        propagated_covariance, orientation_mid, corrected_acceleration, dt_s, config_.noise,
        propagated_state.gravityTangentBasis());
    if (!covariance_status.ok()) {
      return covariance_status;
    }
    if (!propagated_state.allFinite()) {
      return Status(StatusCode::kNumericalFailure, "IMU propagation produced a non-finite state");
    }
    trajectory_status = trajectory.addState(trajectoryState(propagated_state, current.time));
    if (!trajectory_status.ok()) {
      return trajectory_status;
    }
  }
  state = propagated_state;
  covariance = propagated_covariance;
  return trajectory;
}

}  // namespace uav::nav::lio
