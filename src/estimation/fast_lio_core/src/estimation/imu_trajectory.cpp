#include "fast_lio_core/estimation/imu_trajectory.hpp"

#include <algorithm>
#include <cmath>
#include <utility>

#include "fast_lio_core/geometry/pose_interpolator.hpp"

namespace uav::nav::lio {

bool ImuTrajectoryState::allFinite() const noexcept {
  return orientation_odom_imu.coeffs().allFinite() && position_odom_imu_m.allFinite() &&
         velocity_odom_imu_m_s.allFinite() &&
         std::isfinite(orientation_odom_imu.squaredNorm()) &&
         orientation_odom_imu.squaredNorm() > 1e-24;
}

ImuTrajectory::ImuTrajectory(FrameId odom_frame, FrameId imu_frame)
    : odom_frame_(std::move(odom_frame)), imu_frame_(std::move(imu_frame)) {}

Status ImuTrajectory::addState(ImuTrajectoryState state) {
  if (odom_frame_.empty() || imu_frame_.empty()) {
    return Status(StatusCode::kInvalidArgument, "IMU trajectory frame ids must not be empty");
  }
  if (!state.allFinite()) {
    return Status(StatusCode::kNumericalFailure, "IMU trajectory state is not finite");
  }
  if (!states_.empty()) {
    if (!states_.back().time.sameClockDomain(state.time)) {
      return Status(StatusCode::kClockDomainMismatch, "IMU trajectory clock domain changed");
    }
    if (state.time.nanoseconds() <= states_.back().time.nanoseconds()) {
      return Status(StatusCode::kTimestampRegression,
                    "IMU trajectory timestamps must be strictly increasing");
    }
  }
  state.orientation_odom_imu.normalize();
  states_.push_back(std::move(state));
  return Status::Ok();
}

Result<ImuTrajectoryState> ImuTrajectory::interpolate(const Timestamp& time) const {
  if (states_.empty()) {
    return Status(StatusCode::kInsufficientData, "Cannot interpolate an empty IMU trajectory");
  }
  if (!time.sameClockDomain(states_.front().time)) {
    return Status(StatusCode::kClockDomainMismatch,
                  "Trajectory query uses a different clock domain");
  }
  const auto upper = std::lower_bound(states_.begin(), states_.end(), time.nanoseconds(),
                                      [](const ImuTrajectoryState& state, std::int64_t time_ns) {
                                        return state.time.nanoseconds() < time_ns;
                                      });
  if (upper == states_.end()) {
    if (time == states_.back().time) {
      return states_.back();
    }
    return Status(StatusCode::kOutOfRange, "Trajectory query is after the final state");
  }
  if (upper->time == time) {
    return *upper;
  }
  if (upper == states_.begin()) {
    return Status(StatusCode::kOutOfRange, "Trajectory query is before the first state");
  }
  const auto lower = std::prev(upper);
  const auto numerator = checkedDifference(time, lower->time);
  const auto denominator = checkedDifference(upper->time, lower->time);
  if (!numerator.ok()) {
    return numerator.status();
  }
  if (!denominator.ok() || denominator.value().nanoseconds() <= 0) {
    return denominator.ok()
               ? Status(StatusCode::kTimestampRegression,
                        "IMU trajectory interpolation interval is not positive")
               : denominator.status();
  }
  const double alpha = static_cast<double>(numerator.value().nanoseconds()) /
                       static_cast<double>(denominator.value().nanoseconds());
  if (!std::isfinite(alpha) || alpha < 0.0 || alpha > 1.0) {
    return Status(StatusCode::kNumericalFailure,
                  "IMU trajectory interpolation ratio is not finite");
  }
  ImuTrajectoryState output;
  output.time = time;
  output.orientation_odom_imu =
      lower->orientation_odom_imu.slerp(alpha, upper->orientation_odom_imu);
  output.orientation_odom_imu.normalize();
  output.position_odom_imu_m =
      (1.0 - alpha) * lower->position_odom_imu_m + alpha * upper->position_odom_imu_m;
  output.velocity_odom_imu_m_s =
      (1.0 - alpha) * lower->velocity_odom_imu_m_s + alpha * upper->velocity_odom_imu_m_s;
  return output;
}

Result<RigidTransform> ImuTrajectory::pose(const Timestamp& time) const {
  const auto state = interpolate(time);
  if (!state.ok()) {
    return state.status();
  }
  return RigidTransform(odom_frame_, imu_frame_, state.value().orientation_odom_imu,
                        state.value().position_odom_imu_m);
}

const std::vector<ImuTrajectoryState>& ImuTrajectory::states() const noexcept { return states_; }

const FrameId& ImuTrajectory::odomFrameId() const noexcept { return odom_frame_; }

const FrameId& ImuTrajectory::imuFrameId() const noexcept { return imu_frame_; }

bool ImuTrajectory::empty() const noexcept { return states_.empty(); }

std::size_t ImuTrajectory::size() const noexcept { return states_.size(); }

const Timestamp* ImuTrajectory::startTime() const noexcept {
  return states_.empty() ? nullptr : &states_.front().time;
}

const Timestamp* ImuTrajectory::endTime() const noexcept {
  return states_.empty() ? nullptr : &states_.back().time;
}

}  // namespace uav::nav::lio
