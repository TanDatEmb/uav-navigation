#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <string>

#include <Eigen/Core>

namespace px4_navigation_external_mode::velocity_only {

enum class Role : std::uint8_t { kMain = 0U, kBackup = 1U, kEmergency = 2U };

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
  kVelocityViabilityLimit,
};

struct Result final {
  Eigen::Vector3d velocity_enu{Eigen::Vector3d::Zero()};
  Eigen::Vector3d acceleration_enu{Eigen::Vector3d::Zero()};
  Eigen::Vector3d requested_velocity_enu{Eigen::Vector3d::Zero()};
  Eigen::Vector3d previous_velocity_enu{Eigen::Vector3d::Zero()};
  Eigen::Vector3d previous_acceleration_enu{Eigen::Vector3d::Zero()};
  double delta_s{0.0};
  double velocity_residual_mps{0.0};
  double velocity_viability_residual_mps{0.0};
  double radial_acceleration_residual_mps2{0.0};
  double acceleration_residual_mps2{0.0};
  double jerk_residual_mps3{0.0};
  std::uint32_t projection_iterations{0U};
  bool projection_converged{false};
  bool braking_recovery{false};
  bool limited{false};
  Failure failure{Failure::kNone};

  [[nodiscard]] bool success() const noexcept { return failure == Failure::kNone; }
};

[[nodiscard]] inline const char* failureName(const Failure failure) noexcept {
  switch (failure) {
    case Failure::kNone: return "none";
    case Failure::kInvalidPolicy: return "invalid_policy";
    case Failure::kInvalidInput: return "invalid_input";
    case Failure::kNonPositiveDelta: return "non_positive_delta";
    case Failure::kIdentityBoundary: return "identity_boundary";
    case Failure::kAccelerationLimit: return "acceleration_limit";
    case Failure::kJerkLimit: return "jerk_limit";
    case Failure::kVelocityLimit: return "velocity_limit";
    case Failure::kVelocityViabilityLimit: return "velocity_viability_limit";
  }
  return "unknown";
}

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
      (previous.role == Role::kBackup && current.role == Role::kBackup) ||
      // A planner-certified emergency brake is an explicit safety transition
      // from the currently owned MAIN/BACKUP command. It is not a nominal
      // owner and cannot transition back through this helper.
      (current.role == Role::kEmergency &&
       (previous.role == Role::kMain || previous.role == Role::kBackup ||
        previous.role == Role::kEmergency));
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
  result.requested_velocity_enu = desired_velocity_enu;
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
  result.previous_velocity_enu = previous->velocity_enu;
  result.previous_acceleration_enu = previous->acceleration_enu;
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
  result.delta_s = dt_s;
  const Eigen::Vector3d requested_acceleration =
      (desired_velocity_enu - previous->velocity_enu) / dt_s;
  if (!requested_acceleration.allFinite()) {
    result.failure = Failure::kInvalidInput;
    return result;
  }
  // The next acceleration must lie in the intersection of three closed balls:
  // the acceleration limit, the jerk step around the previous acceleration,
  // and the velocity-cap ball after integrating over dt. Clamping A then J
  // independently can leave the integrated velocity just outside its cap even
  // when a joint feasible acceleration exists. Dykstra's projection preserves
  // the fail-closed limits while finding that joint step deterministically.
  const auto projectToBall = [](const Eigen::Vector3d& value,
                                const Eigen::Vector3d& center,
                                const double radius) -> Eigen::Vector3d {
    const Eigen::Vector3d offset = value - center;
    const double norm = offset.norm();
    if (!std::isfinite(norm) || norm <= radius) return value;
    return center + offset * (radius / norm);
  };
  const Eigen::Vector3d acceleration_center = Eigen::Vector3d::Zero();
  const Eigen::Vector3d jerk_center = previous->acceleration_enu;
  const double jerk_radius = policy.maximum_jerk_mps3 * dt_s;
  const Eigen::Vector3d velocity_center = -previous->velocity_enu / dt_s;
  const double velocity_radius = policy.maximum_velocity_mps / dt_s;
  // Keep one-step look-ahead feasibility. If the current sample reaches the
  // cap with outward acceleration, the next sample may be unable to brake
  // inside the same jerk ball. The fourth ball is a conservative certificate:
  // with v[k+1] = v[k] + a[k]dt and a[k+1] = a[k] - Jdt*d, it requires
  // v[k] + (2*a[k] - Jdt*d)dt to remain inside the cap. It never enlarges
  // any configured limit. The requested acceleration is shaped below using a
  // conservative radial braking envelope. It is deliberately a target
  // shaping term rather than another hard intersection constraint: if a
  // previous sample already carries more outward acceleration than the
  // envelope, the only physically valid recovery is to reduce it over several
  // jerk-bounded samples.
  Eigen::Vector3d viability_direction = previous->velocity_enu;
  if (viability_direction.norm() <= 1.0e-12) {
    viability_direction = requested_acceleration;
  }
  if (viability_direction.norm() <= 1.0e-12) {
    viability_direction = Eigen::Vector3d::UnitX();
  } else {
    viability_direction.normalize();
  }
  const Eigen::Vector3d viability_center =
      -previous->velocity_enu / (2.0 * dt_s) + viability_direction * (0.5 * jerk_radius);
  const double viability_radius = 0.5 * velocity_radius;
  Eigen::Vector3d radial_direction = previous->velocity_enu;
  if (radial_direction.norm() <= 1.0e-12) radial_direction = requested_acceleration;
  if (radial_direction.norm() <= 1.0e-12) radial_direction = Eigen::Vector3d::UnitX();
  radial_direction.normalize();
  const double speed_headroom = std::max(
      0.0, policy.maximum_velocity_mps - previous->velocity_enu.norm());
  // Solve a^2 + 2*J*dt*a - 2*J*headroom <= 0 so the envelope also accounts
  // for the velocity consumed by this sample before the next jerk-limited
  // braking step is available.
  const double acceleration_envelope_limit = std::max(
      0.0, -jerk_radius + std::sqrt(
          jerk_radius * jerk_radius + 2.0 * policy.maximum_jerk_mps3 * speed_headroom));
  Eigen::Vector3d shaped_requested_acceleration = requested_acceleration;
  const double requested_acceleration_norm = shaped_requested_acceleration.norm();
  if (std::isfinite(requested_acceleration_norm) &&
      requested_acceleration_norm > acceleration_envelope_limit &&
      requested_acceleration_norm > 1.0e-12) {
    shaped_requested_acceleration *= acceleration_envelope_limit / requested_acceleration_norm;
  }
  // This is used only to classify double-precision residuals after projection;
  // all projection radii remain the configured hard limits.
  constexpr double kProjectionNumericalTolerance = 1.0e-10;
  Eigen::Vector3d acceleration = shaped_requested_acceleration;
  Eigen::Vector3d acceleration_correction = Eigen::Vector3d::Zero();
  Eigen::Vector3d jerk_correction = Eigen::Vector3d::Zero();
  Eigen::Vector3d velocity_correction = Eigen::Vector3d::Zero();
  Eigen::Vector3d viability_correction = Eigen::Vector3d::Zero();
  for (int iteration = 0; iteration < 4096; ++iteration) {
    result.projection_iterations = static_cast<std::uint32_t>(iteration + 1);
    Eigen::Vector3d shifted = acceleration + acceleration_correction;
    Eigen::Vector3d projected = projectToBall(
        shifted, acceleration_center, policy.maximum_acceleration_mps2);
    acceleration_correction = shifted - projected;
    acceleration = projected;

    shifted = acceleration + jerk_correction;
    projected = projectToBall(shifted, jerk_center, jerk_radius);
    jerk_correction = shifted - projected;
    acceleration = projected;

    shifted = acceleration + velocity_correction;
    projected = projectToBall(shifted, velocity_center, velocity_radius);
    velocity_correction = shifted - projected;
    acceleration = projected;

    shifted = acceleration + viability_correction;
    projected = projectToBall(shifted, viability_center, viability_radius);
    viability_correction = shifted - projected;
    acceleration = projected;

    const double acceleration_error =
        std::max(0.0, acceleration.norm() - policy.maximum_acceleration_mps2);
    const double jerk_error = std::max(
        0.0, (acceleration - previous->acceleration_enu).norm() - jerk_radius);
    const double velocity_error = std::max(
        0.0, (previous->velocity_enu + acceleration * dt_s).norm() -
            policy.maximum_velocity_mps);
    const double viability_error = std::max(
        0.0, (previous->velocity_enu +
              (2.0 * acceleration - viability_direction * jerk_radius) * dt_s).norm() -
            policy.maximum_velocity_mps);
    if (std::max({acceleration_error, jerk_error, velocity_error, viability_error}) <=
        1.0e-13) {
      result.projection_converged = true;
      break;
    }
  }
  if (!result.projection_converged) {
    // The requested target can become infeasible after a direction change
    // even though a jerk-bounded braking step still fits inside the current
    // velocity cap. Continue with that physically reachable braking step;
    // fail closed only if the recovery step itself would exceed a hard limit.
    Eigen::Vector3d braking_acceleration =
        previous->acceleration_enu - radial_direction * jerk_radius;
    if (braking_acceleration.norm() > policy.maximum_acceleration_mps2 +
        kProjectionNumericalTolerance ||
        (braking_acceleration - previous->acceleration_enu).norm() / dt_s >
        policy.maximum_jerk_mps3 + kProjectionNumericalTolerance) {
      const double previous_acceleration_norm = previous->acceleration_enu.norm();
      if (previous_acceleration_norm > 1.0e-12) {
        braking_acceleration = previous->acceleration_enu * std::max(
            0.0, previous_acceleration_norm - jerk_radius) / previous_acceleration_norm;
      }
    }
    const Eigen::Vector3d braking_velocity =
        previous->velocity_enu + braking_acceleration * dt_s;
    const bool braking_step_valid = braking_acceleration.allFinite() &&
        braking_velocity.allFinite() &&
        braking_acceleration.norm() <= policy.maximum_acceleration_mps2 +
            kProjectionNumericalTolerance &&
        (braking_acceleration - previous->acceleration_enu).norm() / dt_s <=
            policy.maximum_jerk_mps3 + kProjectionNumericalTolerance &&
        braking_velocity.norm() <= policy.maximum_velocity_mps +
            kProjectionNumericalTolerance;
    if (!braking_step_valid) {
      result.acceleration_enu = acceleration;
      result.velocity_enu = previous->velocity_enu + acceleration * dt_s;
      result.jerk_residual_mps3 = std::max(
          0.0, (result.acceleration_enu - previous->acceleration_enu).norm() / dt_s -
              policy.maximum_jerk_mps3);
      result.failure = Failure::kJerkLimit;
      return result;
    }
    acceleration = braking_acceleration;
    result.braking_recovery = true;
  }
  if ((acceleration - requested_acceleration).norm() > 1.0e-12) {
    result.limited = true;
  }
  result.acceleration_enu = acceleration;
  result.velocity_enu = previous->velocity_enu + acceleration * dt_s;
  result.velocity_residual_mps = std::max(
      0.0, result.velocity_enu.norm() - policy.maximum_velocity_mps);
  result.acceleration_residual_mps2 = std::max(
      0.0, result.acceleration_enu.norm() - policy.maximum_acceleration_mps2);
  result.jerk_residual_mps3 = std::max(
      0.0, (result.acceleration_enu - previous->acceleration_enu).norm() / dt_s -
          policy.maximum_jerk_mps3);
  const double next_velocity_with_max_radial_brake =
      (previous->velocity_enu +
       (2.0 * result.acceleration_enu - viability_direction * jerk_radius) * dt_s).norm();
  const double viability_residual_mps = std::max(
      0.0, next_velocity_with_max_radial_brake - policy.maximum_velocity_mps);
  result.velocity_viability_residual_mps = viability_residual_mps;
  result.radial_acceleration_residual_mps2 = std::max(
      0.0, result.acceleration_enu.dot(radial_direction) - acceleration_envelope_limit);
  if (result.velocity_enu.norm() > policy.maximum_velocity_mps + 1.0e-12) {
    result.failure = Failure::kVelocityLimit;
    return result;
  }
  if (result.acceleration_enu.norm() >
      policy.maximum_acceleration_mps2 + kProjectionNumericalTolerance) {
    result.failure = Failure::kAccelerationLimit;
    return result;
  }
  if ((result.acceleration_enu - previous->acceleration_enu).norm() / dt_s >
      policy.maximum_jerk_mps3 + kProjectionNumericalTolerance) {
    result.failure = Failure::kJerkLimit;
    return result;
  }
  if (!result.braking_recovery && viability_residual_mps > kProjectionNumericalTolerance) {
    result.failure = Failure::kVelocityViabilityLimit;
    return result;
  }
  if (!result.velocity_enu.allFinite() || !result.acceleration_enu.allFinite()) {
    result.failure = Failure::kInvalidInput;
  }
  return result;
}

}  // namespace px4_navigation_external_mode::velocity_only
