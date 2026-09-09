#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <string>

#include <Eigen/Core>

namespace px4_navigation_external_mode::velocity_only {

enum class Role : std::uint8_t { kMain = 0U, kBackup = 1U };

struct Identity final {
  std::string mission_id;
  std::uint32_t waypoint_index{0U};
  std::uint64_t request_id{0U};
  std::uint64_t bundle_generation{0U};
  Role role{Role::kMain};
};

struct Policy final {
  double maximum_velocity_mps{0.0};
  double maximum_acceleration_mps2{0.0};
  double maximum_jerk_mps3{0.0};
};

struct Previous final {
  Identity identity;
  Eigen::Vector3d velocity_enu{Eigen::Vector3d::Zero()};
  Eigen::Vector3d acceleration_enu{Eigen::Vector3d::Zero()};
  std::int64_t stamp_ns{0};
};

enum class Failure : std::uint8_t {
  kNone = 0U,
  kInvalidPolicy,
  kInvalidInput,
  kNonPositiveDelta,
  kIdentityBoundary,
  kAccelerationLimit,
  kJerkLimit,
  kVelocityLimit,
};

struct Result final {
  Eigen::Vector3d velocity_enu{Eigen::Vector3d::Zero()};
  Eigen::Vector3d acceleration_enu{Eigen::Vector3d::Zero()};
  bool limited{false};
  Failure failure{Failure::kNone};

  [[nodiscard]] bool success() const noexcept { return failure == Failure::kNone; }
};

[[nodiscard]] inline bool validPolicy(const Policy& policy) noexcept {
  return std::isfinite(policy.maximum_velocity_mps) && policy.maximum_velocity_mps > 0.0 &&
         std::isfinite(policy.maximum_acceleration_mps2) &&
         policy.maximum_acceleration_mps2 > 0.0 &&
         std::isfinite(policy.maximum_jerk_mps3) && policy.maximum_jerk_mps3 > 0.0;
}

[[nodiscard]] inline bool validIdentity(const Identity& identity) noexcept {
  return !identity.mission_id.empty() && identity.request_id > 0U &&
         identity.bundle_generation > 0U;
}

[[nodiscard]] inline bool sameContinuityOwner(const Identity& previous,
                                              const Identity& current) noexcept {
  if (previous.mission_id != current.mission_id ||
      previous.bundle_generation > current.bundle_generation) {
    return false;
  }
  const bool role_continuity =
      (previous.role == Role::kMain && current.role == Role::kMain) ||
      (previous.role == Role::kMain && current.role == Role::kBackup) ||
      (previous.role == Role::kBackup && current.role == Role::kBackup);
  if (!role_continuity) return false;
  const bool same_waypoint = previous.waypoint_index == current.waypoint_index &&
      current.request_id >= previous.request_id;
  const bool authorized_next_waypoint =
      previous.waypoint_index < std::numeric_limits<std::uint32_t>::max() &&
      current.waypoint_index == previous.waypoint_index + 1U &&
      current.request_id > previous.request_id;
  return (same_waypoint || authorized_next_waypoint) && current.request_id > 0U;
}

[[nodiscard]] inline Result limit(const Eigen::Vector3d& desired_velocity_enu,
                                  const std::int64_t now_ns,
                                  const Identity& identity,
                                  const Policy& policy,
                                  const Previous* previous) noexcept {
  Result result;
  if (!validPolicy(policy)) {
    result.failure = Failure::kInvalidPolicy;
    return result;
  }
  if (!validIdentity(identity) || !desired_velocity_enu.allFinite() || now_ns <= 0) {
    result.failure = Failure::kInvalidInput;
    return result;
  }
  if (desired_velocity_enu.norm() > policy.maximum_velocity_mps + 1.0e-12) {
    result.failure = Failure::kVelocityLimit;
    return result;
  }
  result.velocity_enu = desired_velocity_enu;
  if (previous == nullptr) return result;
  if (!validIdentity(previous->identity) || !previous->velocity_enu.allFinite() ||
      !previous->acceleration_enu.allFinite() || previous->stamp_ns <= 0) {
    result.failure = Failure::kInvalidInput;
    return result;
  }
  if (!sameContinuityOwner(previous->identity, identity)) {
    result.failure = Failure::kIdentityBoundary;
    return result;
  }
  if (previous->velocity_enu.norm() > policy.maximum_velocity_mps + 1.0e-12) {
    result.failure = Failure::kVelocityLimit;
    return result;
  }
  if (previous->acceleration_enu.norm() > policy.maximum_acceleration_mps2 + 1.0e-12) {
    result.failure = Failure::kAccelerationLimit;
    return result;
  }
  if (now_ns <= previous->stamp_ns) {
    result.failure = Failure::kNonPositiveDelta;
    return result;
  }
  const double dt_s = static_cast<double>(now_ns - previous->stamp_ns) * 1.0e-9;
  if (!std::isfinite(dt_s) || dt_s <= 0.0 || dt_s > 1.0) {
    result.failure = Failure::kNonPositiveDelta;
    return result;
  }
  const Eigen::Vector3d requested_acceleration =
      (desired_velocity_enu - previous->velocity_enu) / dt_s;
  if (!requested_acceleration.allFinite()) {
    result.failure = Failure::kInvalidInput;
    return result;
  }
  Eigen::Vector3d acceleration = requested_acceleration;
  if (acceleration.norm() > policy.maximum_acceleration_mps2) {
    acceleration *= policy.maximum_acceleration_mps2 / acceleration.norm();
    result.limited = true;
  }
  const Eigen::Vector3d jerk = (acceleration - previous->acceleration_enu) / dt_s;
  if (!jerk.allFinite()) {
    result.failure = Failure::kInvalidInput;
    return result;
  }
  if (jerk.norm() > policy.maximum_jerk_mps3) {
    acceleration = previous->acceleration_enu +
        jerk * (policy.maximum_jerk_mps3 / jerk.norm()) * dt_s;
    result.limited = true;
  }
  result.acceleration_enu = acceleration;
  result.velocity_enu = previous->velocity_enu + acceleration * dt_s;
  if (result.velocity_enu.norm() > policy.maximum_velocity_mps + 1.0e-12) {
    result.failure = Failure::kVelocityLimit;
    return result;
  }
  if (!result.velocity_enu.allFinite() || !result.acceleration_enu.allFinite()) {
    result.failure = Failure::kInvalidInput;
  }
  return result;
}

}  // namespace px4_navigation_external_mode::velocity_only
