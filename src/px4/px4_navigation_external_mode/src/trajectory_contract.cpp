#include "px4_navigation_external_mode/trajectory_contract.hpp"

#include <algorithm>
#include <cmath>

namespace px4_navigation_external_mode {
namespace {

bool finitePoint(const geometry_msgs::msg::Point& point) {
  return std::isfinite(point.x) && std::isfinite(point.y) && std::isfinite(point.z);
}

bool finiteVector(const geometry_msgs::msg::Vector3& vector) {
  return std::isfinite(vector.x) && std::isfinite(vector.y) && std::isfinite(vector.z);
}

Eigen::Vector3d pointToEigen(const geometry_msgs::msg::Point& point) {
  return {point.x, point.y, point.z};
}

Eigen::Vector3d vectorToEigen(const geometry_msgs::msg::Vector3& vector) {
  return {vector.x, vector.y, vector.z};
}

}  // namespace

TrajectoryValidation validateTrajectory(
    const navigation_interfaces::msg::PlannedTrajectory& trajectory,
    const std::string& expected_frame) {
  if (!trajectory.success) {
    return {TrajectoryInputFailure::NotSuccessful, "planner did not produce a trajectory"};
  }
  if (trajectory.header.frame_id != expected_frame) {
    return {TrajectoryInputFailure::WrongFrame, "trajectory frame does not match PX4 adapter input"};
  }
  const std::size_t count = trajectory.time_from_start.size();
  if (count == 0U) {
    return {TrajectoryInputFailure::Empty, "trajectory contains no samples"};
  }
  if (trajectory.position.size() != count || trajectory.velocity.size() != count ||
      trajectory.acceleration.size() != count) {
    return {TrajectoryInputFailure::SizeMismatch, "trajectory arrays have different lengths"};
  }
  if (!std::isfinite(trajectory.duration_s) || trajectory.duration_s < 0.0) {
    return {TrajectoryInputFailure::InvalidDuration, "trajectory duration is invalid"};
  }

  double previous_time = -1.0;
  for (std::size_t index = 0; index < count; ++index) {
    const double time = trajectory.time_from_start[index];
    if (!std::isfinite(time) || !finitePoint(trajectory.position[index]) ||
        !finiteVector(trajectory.velocity[index]) || !finiteVector(trajectory.acceleration[index])) {
      return {TrajectoryInputFailure::NonFinite, "trajectory contains a non-finite sample"};
    }
    if (time < 0.0 || time <= previous_time) {
      return {TrajectoryInputFailure::NonMonotonicTime, "trajectory sample times are not increasing"};
    }
    previous_time = time;
  }
  if (trajectory.duration_s + 1e-9 < previous_time) {
    return {TrajectoryInputFailure::InvalidDuration, "trajectory duration ends before its samples"};
  }
  return {};
}

TrajectorySample sampleTrajectory(
    const navigation_interfaces::msg::PlannedTrajectory& trajectory, double time_from_start_s) {
  const std::size_t last = trajectory.time_from_start.size() - 1U;
  const double query = std::clamp(time_from_start_s, trajectory.time_from_start.front(),
                                  trajectory.time_from_start.back());
  const auto upper = std::upper_bound(trajectory.time_from_start.begin(),
                                      trajectory.time_from_start.end(), query);
  const std::size_t upper_index = static_cast<std::size_t>(
      std::distance(trajectory.time_from_start.begin(), upper));
  const std::size_t right = std::min(upper_index, last);
  const std::size_t left = right == 0U ? 0U : right - 1U;
  const double left_time = trajectory.time_from_start[left];
  const double right_time = trajectory.time_from_start[right];
  const double alpha = right == left ? 0.0 : (query - left_time) / (right_time - left_time);

  if (right == left) {
    TrajectorySample sample;
    sample.position_enu = pointToEigen(trajectory.position[left]);
    sample.velocity_enu = vectorToEigen(trajectory.velocity[left]);
    sample.acceleration_enu = vectorToEigen(trajectory.acceleration[left]);
    return sample;
  }
  TrajectorySample sample;
  const auto interpolate = [alpha](double left_value, double right_value) {
    return (1.0 - alpha) * left_value + alpha * right_value;
  };
  const auto point_value = [](const auto& point, int axis) {
    return axis == 0 ? point.x : axis == 1 ? point.y : point.z;
  };
  for (int axis = 0; axis < 3; ++axis) {
    sample.position_enu[axis] = interpolate(point_value(trajectory.position[left], axis),
                                            point_value(trajectory.position[right], axis));
    sample.velocity_enu[axis] = interpolate(point_value(trajectory.velocity[left], axis),
                                             point_value(trajectory.velocity[right], axis));
    sample.acceleration_enu[axis] = interpolate(
        point_value(trajectory.acceleration[left], axis), point_value(trajectory.acceleration[right], axis));
  }
  return sample;
}

Eigen::Vector3f enuToNed(const Eigen::Vector3d& value_enu) {
  return {static_cast<float>(value_enu.y()), static_cast<float>(value_enu.x()),
          static_cast<float>(-value_enu.z())};
}

}  // namespace px4_navigation_external_mode
