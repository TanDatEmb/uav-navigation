#pragma once

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <limits>
#include <optional>
#include <utility>

#include "navigation_planning/planner.hpp"

namespace navigation_planning {

enum class VerificationFailureCode {
  None,
  WorldModelUnavailable,
  InvalidTrajectory,
  StartDiscontinuity,
  OutsideBounds,
  OccupiedSample,
  UnknownBeyondCommitment,
  SafetyUnknownSample,
  DynamicLimitsExceeded,
  WorldChanged,
};

struct TrajectoryVerificationConfig {
  double sample_dt_s{0.05};
  double commitment_horizon_s{1.0};
  double max_start_error_m{0.25};
  DynamicLimits limits{};
  // Simulation-only exception matching PlannerConfig::allow_unknown_start.
  // It applies only to the trusted current vehicle footprint; all other
  // samples remain known-free unless nominal commitment explicitly allows
  // them. A zero radius preserves exact-start-voxel behavior.
  bool allow_unknown_start{false};
  double unknown_start_radius_m{0.0};
  bool allow_nominal_unknown{false};
};

struct TrajectoryVerificationStatistics {
  std::uint64_t sampled_point_count{0};
  std::uint64_t unknown_sample_count{0};
  std::uint64_t occupied_sample_count{0};
  double first_unknown_time_s{std::numeric_limits<double>::infinity()};
  double maximum_velocity_mps{0.0};
  double maximum_acceleration_mps2{0.0};
  double maximum_deceleration_mps2{0.0};
  double maximum_jerk_mps3{0.0};
  std::int64_t verification_time_us{0};
};

struct TrajectoryVerificationResult {
  bool success{false};
  PlanRole role{PlanRole::Nominal};
  VerificationFailureCode failure_code{VerificationFailureCode::None};
  std::uint64_t world_generation{0};
  std::uint64_t world_revision{0};
  TrajectoryVerificationStatistics statistics{};

  [[nodiscard]] bool isCurrent(std::uint64_t generation,
                               std::uint64_t revision) const noexcept {
    return world_generation == generation && world_revision == revision;
  }
};

struct DualPlanVerificationResult {
  bool success{false};
  bool nominal_selected{false};
  PlanRole selected_role{PlanRole::Safety};
  VerificationFailureCode failure_code{VerificationFailureCode::None};
  TimeParameterizedTrajectory selected_trajectory{};
  TrajectoryVerificationResult nominal{};
  TrajectoryVerificationResult safety{};
};

namespace detail {

inline TrajectoryPoint interpolate(const TimeParameterizedTrajectory& trajectory,
                                   double time_s) {
  if (trajectory.points.size() == 1U) return trajectory.points.front();
  const auto& points = trajectory.points;
  const auto upper = std::lower_bound(
      points.begin(), points.end(), time_s,
      [](const TrajectoryPoint& point, double time) {
        return point.time_from_start_s < time;
      });
  if (upper == points.begin()) return *upper;
  if (upper == points.end()) return points.back();
  const auto& after = *upper;
  const auto& before = *(upper - 1);
  const double span = after.time_from_start_s - before.time_from_start_s;
  const double alpha = span > 0.0 ? (time_s - before.time_from_start_s) / span : 0.0;
  TrajectoryPoint result;
  result.time_from_start_s = time_s;
  result.position = before.position + alpha * (after.position - before.position);
  result.velocity = before.velocity + alpha * (after.velocity - before.velocity);
  result.acceleration = before.acceleration + alpha * (after.acceleration - before.acceleration);
  return result;
}

template <typename Model>
TrajectoryVerificationResult verifyModel(const TrajectoryVerificationConfig& config,
                                          const TimeParameterizedTrajectory& trajectory,
                                          const VehicleState& state, PlanRole role,
                                          const Model& world) {
  const auto started = std::chrono::steady_clock::now();
  TrajectoryVerificationResult result;
  result.role = role;
  const auto finish = [&]() {
    result.statistics.verification_time_us =
        std::chrono::duration_cast<std::chrono::microseconds>(
            std::chrono::steady_clock::now() - started)
            .count();
    return result;
  };

  if constexpr (requires { world.isReady(); }) {
    if (!world.isReady()) {
      result.failure_code = VerificationFailureCode::WorldModelUnavailable;
      return finish();
    }
  }
  result.world_generation = world.generation();
  if constexpr (requires { world.revision(); }) result.world_revision = world.revision();

  if (!trajectory.finiteAndMonotonic() || !state.position.allFinite() ||
      !std::isfinite(config.sample_dt_s) || config.sample_dt_s <= 0.0 ||
      !std::isfinite(config.max_start_error_m) || config.max_start_error_m < 0.0 ||
      !std::isfinite(config.limits.max_velocity_mps) || config.limits.max_velocity_mps <= 0.0 ||
      !std::isfinite(config.limits.max_acceleration_mps2) ||
      config.limits.max_acceleration_mps2 <= 0.0 ||
      !std::isfinite(config.limits.max_deceleration_mps2) ||
      config.limits.max_deceleration_mps2 <= 0.0 ||
      !std::isfinite(config.limits.max_jerk_mps3) ||
      config.limits.max_jerk_mps3 <= 0.0 ||
      (config.allow_unknown_start &&
       (!std::isfinite(config.unknown_start_radius_m) ||
        config.unknown_start_radius_m < 0.0))) {
    result.failure_code = VerificationFailureCode::InvalidTrajectory;
    return finish();
  }
  if ((trajectory.points.front().position - state.position).norm() > config.max_start_error_m) {
    result.failure_code = VerificationFailureCode::StartDiscontinuity;
    return finish();
  }
  if (role == PlanRole::Nominal &&
      (!std::isfinite(config.commitment_horizon_s) || config.commitment_horizon_s < 0.0)) {
    result.failure_code = VerificationFailureCode::InvalidTrajectory;
    return finish();
  }

  const auto layer = navigation_mapping::WorldLayer::Inflated;
  const auto bounds = world.bounds(layer);
  const auto start_index = world.worldToGrid(layer, state.position);
  const double resolution = world.resolution(layer);
  const double cell_half_diagonal =
      std::isfinite(resolution) && resolution > 0.0
          ? 0.5 * std::sqrt(3.0) * resolution
          : 0.0;
  const auto trustedUnknownStart = [&](const navigation_mapping::GridIndex3& index) {
    if (!config.allow_unknown_start) return false;
    if (index == start_index) return true;
    if (!std::isfinite(config.unknown_start_radius_m) ||
        config.unknown_start_radius_m <= 0.0) {
      return false;
    }
    return (world.gridToWorld(layer, index) - state.position).norm() <=
           config.unknown_start_radius_m + cell_half_diagonal + 1e-9;
  };
  const double epsilon = 1e-9;
  const double initial_speed = trajectory.points.front().velocity.norm();
  const bool overspeed_recovery = role == PlanRole::Safety &&
                                  initial_speed > config.limits.max_velocity_mps + epsilon;
  bool recovery_speed_seen = false;
  double previous_speed = initial_speed;
  std::optional<double> previous_sample_time;
  std::optional<navigation_mapping::Vec3> previous_acceleration;
  const auto sample = [&](const TrajectoryPoint& point) {
    ++result.statistics.sampled_point_count;
    if (!point.position.allFinite() || !point.velocity.allFinite() ||
        !point.acceleration.allFinite()) {
      result.failure_code = VerificationFailureCode::InvalidTrajectory;
      return false;
    }
    result.statistics.maximum_velocity_mps =
        std::max(result.statistics.maximum_velocity_mps, point.velocity.norm());
    result.statistics.maximum_acceleration_mps2 =
        std::max(result.statistics.maximum_acceleration_mps2, point.acceleration.norm());
    if (previous_sample_time.has_value() && previous_acceleration.has_value() &&
        point.velocity.norm() > 1e-6) {
      const double sample_dt = point.time_from_start_s - *previous_sample_time;
      if (sample_dt > epsilon) {
        const double jerk =
            (point.acceleration - *previous_acceleration).norm() / sample_dt;
        result.statistics.maximum_jerk_mps3 =
            std::max(result.statistics.maximum_jerk_mps3, jerk);
      }
    }
    previous_sample_time = point.time_from_start_s;
    previous_acceleration = point.acceleration;
    if (point.velocity.norm() > 1e-9) {
      result.statistics.maximum_deceleration_mps2 = std::max(
          result.statistics.maximum_deceleration_mps2,
          std::max(0.0, -point.acceleration.dot(point.velocity.normalized())));
    }
    // Acceleration and braking are directional limits. A configured braking
    // limit may be higher than the forward acceleration limit, so the norm
    // bound must admit either envelope while the tangential braking metric
    // below still enforces max_deceleration independently.
    const double maximum_allowed_acceleration =
        std::max(config.limits.max_acceleration_mps2, config.limits.max_deceleration_mps2);
    const double speed = point.velocity.norm();
    if (overspeed_recovery && speed > config.limits.max_velocity_mps + epsilon) {
      // Permit overspeed only while monotonically recovering from the measured
      // initial state. Once inside the normal envelope, never leave it again.
      if (speed > previous_speed + 1e-6) {
        result.failure_code = VerificationFailureCode::DynamicLimitsExceeded;
        return false;
      }
      recovery_speed_seen = true;
    } else if (overspeed_recovery && recovery_speed_seen &&
               speed > config.limits.max_velocity_mps + epsilon) {
      result.failure_code = VerificationFailureCode::DynamicLimitsExceeded;
      return false;
    }
    const bool velocity_exceeded =
        (!overspeed_recovery || !recovery_speed_seen) &&
        result.statistics.maximum_velocity_mps > config.limits.max_velocity_mps + 1e-9;
    if (velocity_exceeded ||
        result.statistics.maximum_acceleration_mps2 > maximum_allowed_acceleration + 1e-9 ||
        result.statistics.maximum_deceleration_mps2 >
            config.limits.max_deceleration_mps2 + 1e-9 ||
        result.statistics.maximum_jerk_mps3 > config.limits.max_jerk_mps3 + 1e-9) {
      result.failure_code = VerificationFailureCode::DynamicLimitsExceeded;
      return false;
    }
    const auto index = world.worldToGrid(layer, point.position);
    if (!bounds.contains(index)) {
      result.failure_code = VerificationFailureCode::OutsideBounds;
      return false;
    }
    const auto cell_state = world.cellState(layer, index);
    if (cell_state == navigation_mapping::CellState::Occupied) {
      ++result.statistics.occupied_sample_count;
      result.failure_code = VerificationFailureCode::OccupiedSample;
      return false;
    }
    if (cell_state == navigation_mapping::CellState::Unknown) {
      // Apply the same local KnownFree overlay used by search and corridor
      // fitting. These samples are not counted as unknown because the current
      // vehicle footprint is the authoritative free-space observation.
      if (trustedUnknownStart(index)) {
        previous_speed = speed;
        return true;
      }
      ++result.statistics.unknown_sample_count;
      result.statistics.first_unknown_time_s =
          std::min(result.statistics.first_unknown_time_s, point.time_from_start_s);
      const bool nominal_commitment =
          role == PlanRole::Nominal && config.allow_nominal_unknown &&
          point.time_from_start_s <= config.commitment_horizon_s + 1e-9;
      if (!nominal_commitment) {
        result.failure_code = role == PlanRole::Nominal
                                  ? VerificationFailureCode::UnknownBeyondCommitment
                                  : VerificationFailureCode::SafetyUnknownSample;
        return false;
      }
    }
    previous_speed = speed;
    return true;
  };

  // The planner already emits time-ordered points. Sample each emitted point
  // once and only add interior samples when a segment is wider than the
  // verifier interval. The previous implementation sampled a regular grid
  // and then every emitted point again, doubling map queries for the normal
  // planner output without improving coverage.
  const double spatial_dt = resolution > 0.0 && std::isfinite(resolution) &&
                                    config.limits.max_velocity_mps > 0.0
                                ? 0.5 * resolution / config.limits.max_velocity_mps
                                : config.sample_dt_s;
  const double dt = std::min(config.sample_dt_s, std::max(1e-4, spatial_dt));
  if (trajectory.points.front().time_from_start_s > epsilon &&
      !sample(interpolate(trajectory, 0.0))) {
    return finish();
  }
  for (std::size_t index = 0; index < trajectory.points.size(); ++index) {
    if (!sample(trajectory.points[index])) return finish();
    if (index + 1U == trajectory.points.size()) break;

    const double start_time = trajectory.points[index].time_from_start_s;
    const double end_time = trajectory.points[index + 1U].time_from_start_s;
    for (double time = start_time + dt; time < end_time - epsilon; time += dt) {
      if (!sample(interpolate(trajectory, time))) return finish();
    }
  }

  bool world_changed = world.generation() != result.world_generation;
  if constexpr (requires { world.revision(); }) {
    world_changed = world_changed || world.revision() != result.world_revision;
  }
  if (world_changed) {
    result.failure_code = VerificationFailureCode::WorldChanged;
    return finish();
  }
  result.success = true;
  return finish();
}

template <typename Model>
DualPlanVerificationResult verifyDualModel(const TrajectoryVerificationConfig& config,
                                            const TimeParameterizedTrajectory& nominal,
                                            const TimeParameterizedTrajectory& safety,
                                            const VehicleState& state, const Model& world) {
  DualPlanVerificationResult result;
  result.nominal = verifyModel(config, nominal, state, PlanRole::Nominal, world);
  result.safety = verifyModel(config, safety, state, PlanRole::Safety, world);
  if (!result.safety.success) {
    result.failure_code = result.safety.failure_code;
    return result;
  }
  if (result.nominal.success &&
      !result.nominal.isCurrent(result.safety.world_generation, result.safety.world_revision)) {
    result.failure_code = VerificationFailureCode::WorldChanged;
    return result;
  }
  result.success = true;
  if (result.nominal.success) {
    result.nominal_selected = true;
    result.selected_role = PlanRole::Nominal;
    result.selected_trajectory = nominal;
  } else {
    result.selected_role = PlanRole::Safety;
    result.selected_trajectory = safety;
  }
  return result;
}

}  // namespace detail

class TrajectoryVerifier final {
 public:
  explicit TrajectoryVerifier(TrajectoryVerificationConfig config = {})
      : config_(std::move(config)) {}

  [[nodiscard]] TrajectoryVerificationResult verify(
      const TimeParameterizedTrajectory& trajectory, const VehicleState& state,
      PlanRole role, const navigation_mapping::WorldModel& world) const;

  [[nodiscard]] DualPlanVerificationResult verifyDual(
      const TimeParameterizedTrajectory& nominal, const TimeParameterizedTrajectory& safety,
      const VehicleState& state, const navigation_mapping::WorldModel& world) const;

  template <typename Model>
  [[nodiscard]] TrajectoryVerificationResult verifyForTest(
      const TimeParameterizedTrajectory& trajectory, const VehicleState& state, PlanRole role,
      const Model& world) const {
    return detail::verifyModel(config_, trajectory, state, role, world);
  }

  template <typename Model>
  [[nodiscard]] DualPlanVerificationResult verifyDualForTest(
      const TimeParameterizedTrajectory& nominal, const TimeParameterizedTrajectory& safety,
      const VehicleState& state, const Model& world) const {
    return detail::verifyDualModel(config_, nominal, safety, state, world);
  }

 private:
  TrajectoryVerificationConfig config_;
};

}  // namespace navigation_planning
