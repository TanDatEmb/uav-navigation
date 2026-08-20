#pragma once

#include <cstddef>
#include <cstdint>
#include <string>

#include <Eigen/Core>
#include <navigation_interfaces/msg/planned_trajectory.hpp>

namespace px4_navigation_external_mode {

enum class TrajectoryInputFailure {
  None,
  NotSuccessful,
  WrongFrame,
  Empty,
  SizeMismatch,
  NonFinite,
  NonMonotonicTime,
  InvalidDuration,
  InvalidRole,
  InvalidSafetyTerminalState,
  InvalidValidFrom,
  InvalidTrajectoryId,
};

struct TrajectorySample {
  Eigen::Vector3d position_enu{Eigen::Vector3d::Zero()};
  Eigen::Vector3d velocity_enu{Eigen::Vector3d::Zero()};
  Eigen::Vector3d acceleration_enu{Eigen::Vector3d::Zero()};
};

struct TrajectoryValidation {
  TrajectoryInputFailure failure{TrajectoryInputFailure::None};
  std::string message;

  [[nodiscard]] bool valid() const noexcept { return failure == TrajectoryInputFailure::None; }
};

[[nodiscard]] TrajectoryValidation validateTrajectory(
    const navigation_interfaces::msg::PlannedTrajectory& trajectory,
    const std::string& expected_frame);

[[nodiscard]] TrajectorySample sampleTrajectory(
    const navigation_interfaces::msg::PlannedTrajectory& trajectory, double time_from_start_s);

[[nodiscard]] bool trajectoryMatchesGoal(
    const navigation_interfaces::msg::PlannedTrajectory& trajectory,
    const std::string& mission_id, std::uint32_t waypoint_index,
    std::uint64_t request_id) noexcept;

[[nodiscard]] bool trajectoryRevisionIsNotOlder(
    const navigation_interfaces::msg::PlannedTrajectory& trajectory,
    bool accepted_identity_valid, std::uint64_t accepted_generation,
    std::uint64_t accepted_revision) noexcept;

[[nodiscard]] bool trajectoryValidFromIsNotOlder(
    const navigation_interfaces::msg::PlannedTrajectory& trajectory,
    std::int64_t now_ns) noexcept;

// ROS navigation uses ENU/Z-up; PX4 local setpoints use NED/Z-down.
[[nodiscard]] Eigen::Vector3f enuToNed(const Eigen::Vector3d& value_enu);

}  // namespace px4_navigation_external_mode
