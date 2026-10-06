#include "uavnav/lio/output_predictor.hpp"

#include <algorithm>
#include <cmath>
#include <numbers>

namespace uavnav::lio {

namespace {

// PX4 ekf2 OutputPredictor values, kept as they are (F35):
constexpr double kInitialImuDtAvgS = 0.005;   // _dt_update_states_avg initial value
constexpr double kInitialCorrDtAvgS = 0.010;  // _dt_correct_states_avg initial value
constexpr double kAvgKeep = 0.8;              // dt averages: avg = 0.8 * avg + 0.2 * dt
constexpr double kAvgNew = 0.2;
constexpr double kAttitudeGain = 0.5;         // att_gain = 0.5 * dt_imu_avg / delay (damping ~0.7)
constexpr double kIntegralGain = 0.1;         // PI integral term 0.1 * gain^2
constexpr double kVerticalVelGain = 1.1;      // vertical channel: vel term * 1.1 (5 % overshoot tuning)
constexpr double kMaxTauS = 10.0;             // gain = dt / constrain(tau, dt, 10)

// A quaternion with a norm below this carries no rotation that can be recovered by normalising.
constexpr double kMinQuaternionNorm = 1e-9;

double clamp_dt(double dt_s) noexcept {
  return std::clamp(dt_s, time::to_seconds(limits::kPredictorDtMin), time::to_seconds(limits::kPredictorDtMax));
}

bool usable_quaternion(const Eigen::Quaterniond& q) noexcept {
  return q.coeffs().allFinite() && q.norm() > kMinQuaternionNorm;
}

bool finite(const EstimatorSnapshot& s) noexcept {
  return usable_quaternion(s.q_world_imu) && s.v_world_mps.allFinite() && s.p_world_m.allFinite() &&
         s.gyro_bias.allFinite() && s.accel_bias.allFinite() && s.gravity_world.allFinite();
}

// Rotation-vector exponential: the quaternion of a rotation by |rv| about rv / |rv|.
Eigen::Quaterniond exp_rotation(const Eigen::Vector3d& rv) noexcept {
  const double angle = rv.norm();
  if (angle < 1e-12) return Eigen::Quaterniond(1.0, 0.5 * rv.x(), 0.5 * rv.y(), 0.5 * rv.z()).normalized();
  return Eigen::Quaterniond(Eigen::AngleAxisd(angle, rv / angle));
}

double yaw_of(const Eigen::Quaterniond& q) noexcept {
  const Eigen::Matrix3d r = q.toRotationMatrix();
  return std::atan2(r(1, 0), r(0, 0));
}

double wrap_pi(double a) noexcept { return std::remainder(a, 2.0 * std::numbers::pi); }

time::Duration seconds_to_duration(double s) noexcept {
  // Only called with clamped dts (<= kPredictorDtMax), so the conversion cannot overflow.
  return time::nanoseconds(static_cast<std::int64_t>(std::llround(s * 1e9)));
}

}  // namespace

OutputPredictor::OutputPredictor(const PredictorConfig& cfg) noexcept
    : cfg_(cfg),
      dt_imu_avg_s_(kInitialImuDtAvgS),
      dt_corr_avg_s_(kInitialCorrDtAvgS),
      imu_period_s_(time::to_seconds(limits::kPredictorDesignImuPeriod)) {}

void OutputPredictor::push(const Entry& e) noexcept {
  if (size_ < ring_.size()) {
    ring_[(head_ + size_) % ring_.size()] = e;
    ++size_;
  } else {
    ring_[head_] = e;  // overwrite the oldest
    head_ = (head_ + 1) % ring_.size();
  }
}

void OutputPredictor::restart(const EstimatorSnapshot& s) noexcept {
  head_ = 0;
  size_ = 0;
  const OutputSample out{s.t, s.q_world_imu.normalized(), s.v_world_mps, s.p_world_m, reset_counter_};
  push(Entry{out, s.v_world_mps.z(), s.p_world_m.z(), 0.0});

  gyro_bias_ = s.gyro_bias;
  accel_bias_ = s.accel_bias;
  gravity_world_ = s.gravity_world;

  dt_imu_avg_s_ = kInitialImuDtAvgS;
  dt_corr_avg_s_ = kInitialCorrDtAvgS;
  imu_period_s_ = time::to_seconds(limits::kPredictorDesignImuPeriod);
  last_correction_t_.reset();

  delta_angle_corr_.setZero();
  vel_err_integ_.setZero();
  pos_err_integ_.setZero();
  tracking_error_.setZero();
}

void OutputPredictor::align(const EstimatorSnapshot& s) noexcept {
  if (!finite(s)) return;
  restart(s);
}

std::optional<OutputSample> OutputPredictor::on_imu(const ImuDelta& d) noexcept {
  if (size_ == 0) return std::nullopt;
  if (!d.delta_angle_rad.allFinite() || !d.delta_velocity_mps.allFinite() || !std::isfinite(d.dt_s)) {
    return std::nullopt;
  }
  if (d.dt_s < 0.0 || d.dt_s > time::to_seconds(limits::kPredictorBufferSpan)) return std::nullopt;
  if (d.t <= newest().out.t) return std::nullopt;

  Entry e = newest();

  // PX4 calculateOutputStates: bias-corrected deltas plus the held attitude-tracking correction.
  const Eigen::Vector3d delta_angle = d.delta_angle_rad - gyro_bias_ * d.dt_s + delta_angle_corr_;
  const Eigen::Vector3d delta_velocity = d.delta_velocity_mps - accel_bias_ * d.dt_s;

  e.out.q_world_imu = (e.out.q_world_imu * exp_rotation(delta_angle)).normalized();
  // Specific force rotated by the attitude at the end of the step, plus gravity (see the header).
  const Eigen::Vector3d delta_vel_world = e.out.q_world_imu.toRotationMatrix() * delta_velocity + gravity_world_ * d.dt_s;
  const Eigen::Vector3d vel_last = e.out.v_world_mps;
  e.out.v_world_mps += delta_vel_world;
  const Eigen::Vector3d delta_pos = (e.out.v_world_mps + vel_last) * (0.5 * d.dt_s);  // trapezoidal
  e.out.p_world_m += delta_pos;

  e.vert_vel += delta_vel_world.z();
  e.vert_pos += delta_pos.z();
  e.dt = d.dt_s;

  if (!e.out.q_world_imu.coeffs().allFinite() || !e.out.v_world_mps.allFinite() || !e.out.p_world_m.allFinite() ||
      !std::isfinite(e.vert_vel) || !std::isfinite(e.vert_pos)) {
    return std::nullopt;  // finite but huge input overflowed: drop the sample, keep the last output
  }

  e.out.t = d.t;
  e.out.reset_counter = reset_counter_;

  const double dt_clamped = clamp_dt(d.dt_s);
  dt_imu_avg_s_ = kAvgKeep * dt_imu_avg_s_ + kAvgNew * dt_clamped;
  imu_period_s_ = dt_clamped;

  push(e);
  return e.out;
}

Result<std::size_t, PredictorReason> OutputPredictor::match(time::SensorTime t) const noexcept {
  const time::SensorTime newest_t = newest().out.t;
  if (t > newest_t) return std::unexpected(PredictorReason::kNewerThanOutput);
  if (t < at(0).out.t || t < newest_t - limits::kPredictorBufferSpan) {
    return std::unexpected(PredictorReason::kOlderThanBuffer);
  }
  const time::Duration tolerance = seconds_to_duration(imu_period_s_);
  for (std::size_t i = size_; i-- > 0;) {
    const time::SensorTime sample_t = at(i).out.t;
    if (sample_t <= t) {
      if (t - sample_t > tolerance) return std::unexpected(PredictorReason::kOlderThanBuffer);
      return i;
    }
  }
  return std::unexpected(PredictorReason::kOlderThanBuffer);  // unreachable: t >= oldest
}

Result<void, PredictorReason> OutputPredictor::on_correction(const EstimatorSnapshot& s) noexcept {
  if (size_ == 0) return std::unexpected(PredictorReason::kNotInitialized);
  if (!finite(s)) return std::unexpected(PredictorReason::kNonFinite);
  const auto matched = match(s.t);
  if (!matched) return std::unexpected(matched.error());
  const Entry delayed = at(*matched);  // copy: the buffer is modified below

  // Averaged, clamped interval between corrections (PX4 _dt_correct_states_avg).
  if (last_correction_t_) {
    const double dt = clamp_dt(time::to_seconds(s.t - *last_correction_t_));
    dt_corr_avg_s_ = kAvgKeep * dt_corr_avg_s_ + kAvgNew * dt;
  }
  last_correction_t_ = s.t;

  gyro_bias_ = s.gyro_bias;
  accel_bias_ = s.accel_bias;
  gravity_world_ = s.gravity_world;

  // Attitude: delta-angle correction held for the following IMU steps, gain adapted to the delay.
  const Eigen::Quaterniond q_state = s.q_world_imu.normalized();
  const Eigen::Quaterniond q_error = (q_state.conjugate() * delayed.out.q_world_imu).normalized();
  const double scalar = (q_error.w() >= 0.0) ? -2.0 : 2.0;
  const Eigen::Vector3d delta_ang_error = scalar * q_error.vec();
  const double delay = std::max(time::to_seconds(newest().out.t - s.t), dt_imu_avg_s_);
  delta_angle_corr_ = delta_ang_error * (kAttitudeGain * dt_imu_avg_s_ / delay);
  tracking_error_(0) = delta_ang_error.norm();

  // Complementary-filter gains.
  const double dtc = dt_corr_avg_s_;
  const double vel_gain = dtc / std::clamp(time::to_seconds(cfg_.tau_vel), dtc, kMaxTauS);
  const double pos_gain = dtc / std::clamp(time::to_seconds(cfg_.tau_pos), dtc, kMaxTauS);

  // Separate vertical channel (PX4 applyCorrectionToVerticalOutputBuffer): correct the vertical velocity
  // history, then re-integrate the vertical position forward from the oldest sample.
  const double vert_vel_err = s.v_world_mps.z() - delayed.vert_vel;
  const double vert_pos_err = s.p_world_m.z() - delayed.vert_pos;
  const double vert_vel_correction = vert_pos_err * pos_gain + vert_vel_err * vel_gain * kVerticalVelGain;
  at(0).vert_vel += vert_vel_correction;
  for (std::size_t i = 1; i < size_; ++i) {
    Entry& current = at(i - 1);
    Entry& next = at(i);
    next.vert_vel += vert_vel_correction;
    next.vert_pos = current.vert_pos + (current.vert_vel + next.vert_vel) * 0.5 * next.dt;
  }

  // Velocity and position: PI correction added to the whole history (PX4 applyCorrectionToOutputBuffer).
  const Eigen::Vector3d vel_err = s.v_world_mps - delayed.out.v_world_mps;
  const Eigen::Vector3d pos_err = s.p_world_m - delayed.out.p_world_m;
  tracking_error_(1) = vel_err.norm();
  tracking_error_(2) = pos_err.norm();

  vel_err_integ_ += vel_err;
  const Eigen::Vector3d vel_correction = vel_err * vel_gain + vel_err_integ_ * (vel_gain * vel_gain * kIntegralGain);
  pos_err_integ_ += pos_err;
  const Eigen::Vector3d pos_correction = pos_err * pos_gain + pos_err_integ_ * (pos_gain * pos_gain * kIntegralGain);

  for (std::size_t i = 0; i < size_; ++i) {
    at(i).out.v_world_mps += vel_correction;
    at(i).out.p_world_m += pos_correction;
  }
  return {};
}

ResetDelta OutputPredictor::reset_to(const EstimatorSnapshot& s) noexcept {
  const ResetDelta none{Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), 0.0};
  if (!finite(s)) return none;
  if (size_ == 0) {
    ++reset_counter_;
    restart(s);
    return none;
  }

  const OutputSample old = newest().out;
  const auto matched = match(s.t);
  ++reset_counter_;
  if (matched) {
    const Entry& m = at(*matched);
    const Eigen::Quaterniond q_delta = (s.q_world_imu.normalized() * m.out.q_world_imu.conjugate()).normalized();
    const Eigen::Vector3d vel_delta = s.v_world_mps - m.out.v_world_mps;
    const Eigen::Vector3d pos_delta = s.p_world_m - m.out.p_world_m;
    for (std::size_t i = 0; i < size_; ++i) {
      Entry& e = at(i);
      e.out.q_world_imu = (q_delta * e.out.q_world_imu).normalized();
      e.out.v_world_mps += vel_delta;
      e.out.p_world_m += pos_delta;
      e.out.reset_counter = reset_counter_;
      e.vert_vel += vel_delta.z();
      e.vert_pos += pos_delta.z();
    }
    gyro_bias_ = s.gyro_bias;
    accel_bias_ = s.accel_bias;
    gravity_world_ = s.gravity_world;
  } else {
    restart(s);
  }

  const OutputSample& now = newest().out;
  return ResetDelta{now.p_world_m - old.p_world_m, now.v_world_mps - old.v_world_mps,
                    wrap_pi(yaw_of(now.q_world_imu) - yaw_of(old.q_world_imu))};
}

double OutputPredictor::vertical_velocity_mps() const noexcept { return size_ == 0 ? 0.0 : newest().vert_vel; }

}  // namespace uavnav::lio
