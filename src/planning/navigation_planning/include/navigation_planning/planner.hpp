#pragma once

#include <cstdint>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "navigation_mapping/world_model.hpp"
#include "navigation_planning/a_star.hpp"
#include "navigation_planning/bspline_trajectory.hpp"

namespace navigation_planning {

struct VehicleState {
  navigation_mapping::Vec3 position{navigation_mapping::Vec3::Zero()};
  navigation_mapping::Vec3 velocity{navigation_mapping::Vec3::Zero()};
  navigation_mapping::Vec3 acceleration{navigation_mapping::Vec3::Zero()};
};

struct Goal {
  navigation_mapping::Vec3 position{navigation_mapping::Vec3::Zero()};
  // A mission waypoint is terminal by default. Receding-horizon local goals
  // set terminal=false and provide a bounded continuation tangent so the
  // vehicle does not stop at every local-map boundary before replanning.
  bool terminal{true};
  navigation_mapping::Vec3 terminal_velocity{navigation_mapping::Vec3::Zero()};
  // Optional online cap derived from observed free horizon and latency. Zero
  // means use the configured vehicle limit.
  double velocity_limit_mps{0.0};
};

struct DynamicLimits {
  double max_velocity_mps{2.0};
  double max_acceleration_mps2{3.0};
  double max_deceleration_mps2{3.0};
  // Jerk is kept in the planner contract so trajectory timing and the
  // verifier use the same envelope.  Existing callers remain source
  // compatible because the default is deliberately conservative.
  double max_jerk_mps3{6.0};
};

enum class TrajectoryGeneratorKind {
  Bspline,
  QuinticLegacy,
};

// The verifier enforces the mission limit as a hard boundary. New tangent
// values are generated inside a small design margin so ordinary replans do not
// spend their numerical tolerance at the boundary. The initial tangent is
// deliberately kept equal to measured state because it cannot be changed
// retroactively.
inline double designVelocityLimit(const DynamicLimits& limits) noexcept {
  return limits.max_velocity_mps * 0.995;
}

struct PlannerConfig {
  DynamicLimits limits{};
  TrajectoryGeneratorKind trajectory_generator{TrajectoryGeneratorKind::Bspline};
  double trajectory_sample_dt_s{0.05};
  double corridor_sample_spacing_m{0.0};
  // Keep enough retries for dynamic-limit recovery without making every
  // rolling replan spend most of its budget in rejected candidates.
  // B-spline derivatives scale as dt^-n.  Short rolling corridors with a
  // measured splice velocity can need more than five geometric retries before
  // both acceleration and jerk fit the envelope; failing at that arbitrary
  // cap produces a false corridor failure and an unnecessary braking stop.
  int maximum_time_scaling_iterations{8};
  int bspline_smoothing_iterations{12};
  double bspline_smoothing_step{0.04};
  double bspline_snap_weight{1.0};
  double bspline_path_length_weight{0.10};
  double bspline_reference_weight{0.35};
  // The initial B-spline duration is a design target, not a post-hoc stretch.
  double bspline_time_scale{1.10};
  double bspline_time_margin_s{0.25};
  // The final verifier remains independent; this is the cheaper optimizer
  // sampling rate used during repeated rolling-horizon candidates.
  double bspline_optimization_sample_dt_s{0.04};
  bool allow_unknown_start{false};
  // Radius of a virtual KnownFree overlay matching the trusted current vehicle
  // footprint. It converts only Unknown cells whose volumes intersect this
  // sphere; Occupied cells and Unknown cells beyond it remain blocked.
  double unknown_start_radius_m{0.0};
  // Explicit simulation-only gate. The runtime must still dual-verify this
  // candidate with a known-free safety trajectory before publishing it.
  bool allow_nominal_unknown{false};
  double nominal_commitment_horizon_s{1.0};
};

// A plan role is part of the product contract even while the runtime still
// executes the known-free baseline only. A safety plan is never allowed to
// rely on Unknown cells; a nominal plan may be introduced later, but must not
// be published to the vehicle without a committed safety fallback.
enum class PlanRole {
  Nominal,
  Safety,
  Committed,
};

enum class SafetyPlanKind {
  None,
  Route,
  BrakingStop,
};

enum class PlanFailureCode {
  None,
  WorldModelUnavailable,
  ClearanceUnavailable,
  InvalidState,
  StartOutsideBounds,
  GoalOutsideBounds,
  StartOccupied,
  GoalOccupied,
  NoPath,
  CorridorInfeasible,
  DynamicLimitsInfeasible,
  TrajectoryInvalid,
  WorldChanged,
  SafetyStopUnavailable,
};

struct TrajectoryPoint {
  double time_from_start_s{0.0};
  navigation_mapping::Vec3 position{navigation_mapping::Vec3::Zero()};
  navigation_mapping::Vec3 velocity{navigation_mapping::Vec3::Zero()};
  navigation_mapping::Vec3 acceleration{navigation_mapping::Vec3::Zero()};
  navigation_mapping::Vec3 jerk{navigation_mapping::Vec3::Zero()};
  navigation_mapping::Vec3 snap{navigation_mapping::Vec3::Zero()};
};

struct TimeParameterizedTrajectory {
  std::vector<TrajectoryPoint> points;
  double duration_s{0.0};

  [[nodiscard]] bool finiteAndMonotonic() const noexcept;
};

struct CorridorStatistics {
  std::uint64_t segment_count{0};
  std::uint64_t checked_sample_count{0};
  std::uint64_t blocked_sample_count{0};
  double minimum_clearance_radius_m{0.0};
  std::int64_t corridor_time_us{0};
};

struct TrajectoryOptimizationStatistics {
  std::uint64_t segment_count{0};
  std::uint64_t sampled_point_count{0};
  std::uint64_t iterations{0};
  double maximum_velocity_mps{0.0};
  double maximum_acceleration_mps2{0.0};
  double maximum_deceleration_mps2{0.0};
  double maximum_jerk_mps3{0.0};
  double integrated_squared_jerk{0.0};
  double integrated_squared_snap{0.0};
  double objective_cost{0.0};
  double c2_continuity_residual{0.0};
  double geometric_path_length_m{0.0};
  double trajectory_length_m{0.0};
  std::uint64_t raw_path_node_count{0};
  std::uint64_t simplified_path_node_count{0};
  std::uint64_t shortcut_count{0};
  std::uint64_t collision_check_failure_count{0};
  navigation_mapping::Vec3 first_collision_position{navigation_mapping::Vec3::Zero()};
  bool collision_free{false};
  bool dynamic_limits_satisfied{false};
  double duration_s{0.0};
  std::int64_t optimization_time_us{0};
};

struct PlannerStatistics {
  SearchStatistics search{};
  CorridorStatistics corridor{};
  TrajectoryOptimizationStatistics trajectory_optimization{};
  std::int64_t total_planning_time_us{0};
};

struct PlanResult {
  bool success{false};
  PlanRole role{PlanRole::Nominal};
  SafetyPlanKind safety_kind{SafetyPlanKind::None};
  PlanFailureCode failure_code{PlanFailureCode::None};
  TimeParameterizedTrajectory trajectory{};
  std::uint64_t world_generation{0};
  std::uint64_t world_revision{0};
  PlannerStatistics statistics{};

  [[nodiscard]] bool isCurrent(std::uint64_t generation,
                               std::uint64_t revision) const noexcept {
    return world_generation == generation && world_revision == revision;
  }
};

namespace detail {

template <typename Model>
PlanResult planSafetyStopModel(const PlannerConfig& config, const VehicleState& state,
                               const Model& world) {
  const auto planning_started = std::chrono::steady_clock::now();
  PlanResult result;
  result.role = PlanRole::Safety;
  result.safety_kind = SafetyPlanKind::BrakingStop;
  const auto finish = [&]() {
    result.statistics.total_planning_time_us =
        std::chrono::duration_cast<std::chrono::microseconds>(
            std::chrono::steady_clock::now() - planning_started)
            .count();
    return result;
  };

  if constexpr (requires { world.isReady(); }) {
    if (!world.isReady()) {
      result.failure_code = PlanFailureCode::SafetyStopUnavailable;
      return finish();
    }
  }
  result.world_generation = world.generation();
  if constexpr (requires { world.revision(); }) {
    result.world_revision = world.revision();
  }
  if (!state.position.allFinite() || !state.velocity.allFinite() ||
      !state.acceleration.allFinite() ||
      !std::isfinite(config.limits.max_velocity_mps) ||
      config.limits.max_velocity_mps <= 0.0 ||
      !std::isfinite(config.limits.max_acceleration_mps2) ||
      !std::isfinite(config.limits.max_deceleration_mps2) ||
      config.limits.max_acceleration_mps2 <= 0.0 ||
      config.limits.max_deceleration_mps2 <= 0.0 ||
      !std::isfinite(config.limits.max_jerk_mps3) ||
      config.limits.max_jerk_mps3 <= 0.0 ||
      (config.allow_unknown_start &&
       (!std::isfinite(config.unknown_start_radius_m) ||
        config.unknown_start_radius_m < 0.0)) ||
      !std::isfinite(config.trajectory_sample_dt_s) || config.trajectory_sample_dt_s <= 0.0 ||
      state.acceleration.norm() > 1e-6) {
    result.failure_code = PlanFailureCode::SafetyStopUnavailable;
    return finish();
  }
  // A braking trajectory is also the recovery path when measured speed
  // briefly overshoots the mission limit. Preserve that initial velocity so
  // the controller can actually decelerate from it.
  if (state.acceleration.norm() > config.limits.max_acceleration_mps2 + 1e-9) {
    result.failure_code = PlanFailureCode::SafetyStopUnavailable;
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
  const auto trustedUnknownStart = [&](const navigation_mapping::GridIndex3& index,
                                       const navigation_mapping::CellState cell_state) {
    if (!config.allow_unknown_start || cell_state != navigation_mapping::CellState::Unknown) {
      return false;
    }
    if (index == start_index) return true;
    if (!std::isfinite(config.unknown_start_radius_m) ||
        config.unknown_start_radius_m <= 0.0) {
      return false;
    }
    return (world.gridToWorld(layer, index) - state.position).norm() <=
           config.unknown_start_radius_m + cell_half_diagonal + 1e-9;
  };
  const auto isKnownFree = [&](const navigation_mapping::Vec3& position) {
    if (!position.allFinite()) return false;
    const auto index = world.worldToGrid(layer, position);
    if (!bounds.contains(index)) return false;
    const auto cell_state = world.cellState(layer, index);
    return cell_state == navigation_mapping::CellState::KnownFree ||
           trustedUnknownStart(index, cell_state);
  };

  // The propagated odometry can retain a small settling/noise velocity while
  // the vehicle is already held by PX4 after takeoff. Treating that value as
  // a spatial braking trajectory would require the sensor-shadow voxels ahead
  // of the vehicle to be KnownFree, defeating the current-pose exception. A
  // real motion envelope still uses the configured deceleration below this
  // deadband; only the stationary case emits a zero-velocity hold.
  constexpr double stationary_speed_epsilon_mps = 0.1;
  const double measured_speed = state.velocity.norm();
  const double speed = measured_speed > stationary_speed_epsilon_mps
                           ? measured_speed
                           : 0.0;
  // Keep the stop inside the same numerical design margin as generated
  // trajectories.  Using the exact configured limit here makes the sampled
  // braking envelope sit on the verifier boundary; tiny derivative/rounding
  // differences then reject the only fail-closed recovery path as
  // dynamic_limits_exceeded.
  const double acceleration_limit = 0.995 * config.limits.max_deceleration_mps2;
  const double jerk_limit = 0.995 * config.limits.max_jerk_mps3;
  // Use a jerk-limited S-curve instead of switching from zero acceleration to
  // full braking and then back to zero at the final sample. The latter looks
  // physically reasonable in a sparse trajectory, but the verifier samples
  // between emitted points and correctly sees an acceleration step as a
  // 30 m/s^3 jerk when max_jerk is 8 m/s^3.
  const double peak_acceleration = speed > 1e-9
                                        ? std::min(acceleration_limit,
                                                   std::sqrt(speed * jerk_limit))
                                        : 0.0;
  const double ramp_duration = peak_acceleration > 1e-9
                                   ? peak_acceleration / jerk_limit
                                   : 0.0;
  const double plateau_duration = peak_acceleration > 1e-9
                                      ? std::max(0.0, speed / peak_acceleration - ramp_duration)
                                      : 0.0;
  const double duration = 2.0 * ramp_duration + plateau_duration;
  const auto decelerationAt = [&](double time) {
    if (peak_acceleration <= 1e-9 || time <= 0.0) return 0.0;
    if (time < ramp_duration) return jerk_limit * time;
    if (time < ramp_duration + plateau_duration) return peak_acceleration;
    if (time < duration) {
      return std::max(0.0, peak_acceleration -
                               jerk_limit * (time - ramp_duration - plateau_duration));
    }
    return 0.0;
  };
  const auto speedAt = [&](double time) {
    if (speed <= 1e-9 || time <= 0.0) return speed;
    const double clamped_time = std::min(time, duration);
    if (clamped_time < ramp_duration) {
      return std::max(0.0, speed - 0.5 * jerk_limit * clamped_time * clamped_time);
    }
    const double speed_after_ramp = speed - 0.5 * peak_acceleration * ramp_duration;
    if (clamped_time < ramp_duration + plateau_duration) {
      const double local_time = clamped_time - ramp_duration;
      return std::max(0.0, speed_after_ramp - peak_acceleration * local_time);
    }
    const double speed_after_plateau =
        speed_after_ramp - peak_acceleration * plateau_duration;
    const double local_time = std::min(
        ramp_duration, std::max(0.0, clamped_time - ramp_duration - plateau_duration));
    return std::max(0.0, speed_after_plateau - peak_acceleration * local_time +
                             0.5 * jerk_limit * local_time * local_time);
  };
  const auto distanceAt = [&](double time) {
    if (speed <= 1e-9 || time <= 0.0) return 0.0;
    const double clamped_time = std::min(time, duration);
    const double ramp_distance = speed * ramp_duration -
                                 jerk_limit * ramp_duration * ramp_duration * ramp_duration / 6.0;
    if (clamped_time < ramp_duration) {
      return speed * clamped_time -
             jerk_limit * clamped_time * clamped_time * clamped_time / 6.0;
    }
    const double speed_after_ramp = speed - 0.5 * peak_acceleration * ramp_duration;
    const double plateau_distance = speed_after_ramp * plateau_duration -
                                    0.5 * peak_acceleration * plateau_duration * plateau_duration;
    if (clamped_time < ramp_duration + plateau_duration) {
      const double local_time = clamped_time - ramp_duration;
      return ramp_distance + speed_after_ramp * local_time -
             0.5 * peak_acceleration * local_time * local_time;
    }
    const double speed_after_plateau =
        speed_after_ramp - peak_acceleration * plateau_duration;
    const double local_time = std::min(
        ramp_duration, std::max(0.0, clamped_time - ramp_duration - plateau_duration));
    return ramp_distance + plateau_distance + speed_after_plateau * local_time -
           0.5 * peak_acceleration * local_time * local_time +
           jerk_limit * local_time * local_time * local_time / 6.0;
  };
  const double spatial_step = std::isfinite(resolution) && resolution > 0.0
                                  ? 0.5 * resolution
                                  : std::numeric_limits<double>::infinity();
  const double stopping_distance = distanceAt(duration);
  const int time_samples = std::max(
      1, static_cast<int>(std::ceil(duration / config.trajectory_sample_dt_s)));
  const int spatial_samples = std::isfinite(spatial_step)
                                  ? std::max(1, static_cast<int>(
                                                     std::ceil(stopping_distance / spatial_step)))
                                  : 1;
  const int sample_count = std::max(time_samples, spatial_samples);
  const auto motion_direction = speed > 1e-9 ? state.velocity.normalized()
                                             : navigation_mapping::Vec3::Zero();

  result.statistics.corridor.segment_count = duration > 0.0 ? 1U : 0U;
  result.statistics.corridor.minimum_clearance_radius_m = world.clearanceRadius();
  result.trajectory.points.reserve(static_cast<std::size_t>(sample_count + 1));
  const int last_sample = duration > 0.0 ? sample_count : 0;
  for (int sample = 0; sample <= last_sample; ++sample) {
    const double time = duration * static_cast<double>(sample) /
                        static_cast<double>(sample_count);
    const auto position = state.position + motion_direction * distanceAt(time);
    const auto velocity = motion_direction * speedAt(time);
    const auto acceleration = -motion_direction * decelerationAt(time);
    navigation_mapping::Vec3 clamped_velocity = velocity;
    if (sample == last_sample) {
      clamped_velocity = navigation_mapping::Vec3::Zero();
    }
    const navigation_mapping::Vec3 emitted_acceleration =
        sample == last_sample ? navigation_mapping::Vec3::Zero()
                              : acceleration.eval();
    ++result.statistics.corridor.checked_sample_count;
    if (!isKnownFree(position)) {
      ++result.statistics.corridor.blocked_sample_count;
      result.trajectory.points.clear();
      result.failure_code = PlanFailureCode::SafetyStopUnavailable;
      return finish();
    }
    result.trajectory.points.push_back(
        TrajectoryPoint{time, position, clamped_velocity, emitted_acceleration});
  }
  result.trajectory.duration_s = duration;
  result.statistics.trajectory_optimization.sampled_point_count =
      result.trajectory.points.size();
  result.statistics.trajectory_optimization.maximum_velocity_mps = speed;
  result.statistics.trajectory_optimization.maximum_acceleration_mps2 = peak_acceleration;
  result.statistics.trajectory_optimization.maximum_deceleration_mps2 = peak_acceleration;
  result.statistics.trajectory_optimization.maximum_jerk_mps3 = jerk_limit;
  result.statistics.trajectory_optimization.trajectory_length_m = stopping_distance;
  result.statistics.trajectory_optimization.collision_free = true;
  result.statistics.trajectory_optimization.dynamic_limits_satisfied = true;
  result.statistics.trajectory_optimization.duration_s = duration;
  bool world_changed = false;
  if constexpr (requires { world.generation(); }) {
    world_changed = world.generation() != result.world_generation;
  }
  if constexpr (requires { world.revision(); }) {
    world_changed = world_changed || world.revision() != result.world_revision;
  }
  if (!result.trajectory.finiteAndMonotonic() || world_changed) {
    result.trajectory.points.clear();
    result.failure_code = PlanFailureCode::SafetyStopUnavailable;
    return finish();
  }
  result.success = true;
  result.failure_code = PlanFailureCode::None;
  return finish();
}

}  // namespace detail

class Planner final {
 public:
  explicit Planner(PlannerConfig config = {});

  [[nodiscard]] PlanResult plan(const VehicleState& state, const Goal& goal,
                                const navigation_mapping::WorldModel& world) const;

  // Generate an optimistic candidate through Unknown cells. This method is
  // disabled unless the caller explicitly enables the simulation-only policy;
  // callers must pair it with planSafetyStop() and dual verification.
  [[nodiscard]] PlanResult planNominal(
      const VehicleState& state, const Goal& goal,
      const navigation_mapping::WorldModel& world) const;

  // Generate a braking-to-zero trajectory from the current state. The
  // trajectory is accepted only when every sampled point is inside the
  // inflated KnownFree layer, except the explicitly configured current
  // vehicle voxel. It is a safety fallback, not an unknown-space route.
  [[nodiscard]] PlanResult planSafetyStop(
      const VehicleState& state, const navigation_mapping::WorldModel& world) const;

  // Smooth known-free route to the active goal. It reuses the nominal
  // planner pipeline but never treats Unknown cells as traversable.
  [[nodiscard]] PlanResult planSafetyRoute(
      const VehicleState& state, const Goal& goal,
      const navigation_mapping::WorldModel& world) const;

  template <typename Model>
  [[nodiscard]] PlanResult planSafetyStopForTest(const VehicleState& state,
                                                  const Model& world) const {
    return detail::planSafetyStopModel(config_, state, world);
  }

  template <typename Model>
  [[nodiscard]] PlanResult planSafetyRouteForTest(const VehicleState& state,
                                                   const Goal& goal,
                                                   const Model& world) const {
    auto result = planModel(state, goal, world, UnknownPolicy::TreatUnknownAsBlocked,
                            PlanRole::Safety);
    result.safety_kind = SafetyPlanKind::Route;
    return result;
  }

  // Deterministic model hook for geometry and trajectory tests. Production
  // planning uses plan(WorldModel) and never exposes a ROS or vendor map type.
  template <typename Model>
  [[nodiscard]] PlanResult planForTest(const VehicleState& state, const Goal& goal,
                                       const Model& model) const {
    return planModel(state, goal, model, UnknownPolicy::TreatUnknownAsBlocked,
                     PlanRole::Nominal);
  }

  template <typename Model>
  [[nodiscard]] PlanResult planNominalForTest(const VehicleState& state, const Goal& goal,
                                              const Model& model) const {
    if (!config_.allow_nominal_unknown) {
      PlanResult result;
      result.role = PlanRole::Nominal;
      result.failure_code = PlanFailureCode::NoPath;
      return result;
    }
    return planModel(state, goal, model, UnknownPolicy::TreatUnknownAsTraversable,
                     PlanRole::Nominal);
  }

 private:
  template <typename Model>
  [[nodiscard]] PlanResult planModel(const VehicleState& state, const Goal& goal,
                                     const Model& world, UnknownPolicy unknown_policy,
                                     PlanRole role) const;

  PlannerConfig config_;
};

namespace detail {

inline PlanFailureCode mapSearchFailure(SearchFailureCode failure) {
  switch (failure) {
    case SearchFailureCode::WorldModelUnavailable:
      return PlanFailureCode::WorldModelUnavailable;
    case SearchFailureCode::StartOutsideBounds:
      return PlanFailureCode::StartOutsideBounds;
    case SearchFailureCode::GoalOutsideBounds:
      return PlanFailureCode::GoalOutsideBounds;
    case SearchFailureCode::StartOccupied:
      return PlanFailureCode::StartOccupied;
    case SearchFailureCode::GoalOccupied:
      return PlanFailureCode::GoalOccupied;
    case SearchFailureCode::NoPath:
      return PlanFailureCode::NoPath;
    case SearchFailureCode::None:
    default:
      return PlanFailureCode::TrajectoryInvalid;
  }
}

struct QuinticSegment {
  navigation_mapping::Vec3 coefficients[6]{};
  double duration_s{0.0};
};

inline QuinticSegment makeQuintic(const navigation_mapping::Vec3& p0,
                                  const navigation_mapping::Vec3& v0,
                                  const navigation_mapping::Vec3& a0,
                                  const navigation_mapping::Vec3& p1,
                                  const navigation_mapping::Vec3& v1,
                                  const navigation_mapping::Vec3& a1,
                                  double duration_s) {
  const double t = duration_s;
  const double t2 = t * t;
  const double t3 = t2 * t;
  const double t4 = t3 * t;
  const double t5 = t4 * t;
  const auto dp = p1 - p0;
  const auto c0 = p0;
  const auto c1 = v0;
  const auto c2 = 0.5 * a0;
  const auto c3 = (20.0 * dp - (8.0 * v1 + 12.0 * v0) * t -
                   (3.0 * a0 - a1) * t2) /
                  (2.0 * t3);
  const auto c4 = (-30.0 * dp + (14.0 * v1 + 16.0 * v0) * t +
                   (3.0 * a0 - 2.0 * a1) * t2) /
                  (2.0 * t4);
  const auto c5 = (12.0 * dp - (6.0 * v1 + 6.0 * v0) * t -
                   (a0 - a1) * t2) /
                  (2.0 * t5);
  return QuinticSegment{{c0, c1, c2, c3, c4, c5}, duration_s};
}

inline TrajectoryPoint evaluate(const QuinticSegment& segment, double time_s,
                                double absolute_time_s) {
  const double t = std::clamp(time_s, 0.0, segment.duration_s);
  const double t2 = t * t;
  const double t3 = t2 * t;
  const double t4 = t3 * t;
  const double position_scale[6] = {1.0, t, t2, t3, t4, t4 * t};
  TrajectoryPoint point;
  point.time_from_start_s = absolute_time_s;
  point.position = navigation_mapping::Vec3::Zero();
  for (int i = 0; i < 6; ++i) point.position += position_scale[i] * segment.coefficients[i];
  point.velocity = segment.coefficients[1] + 2.0 * t * segment.coefficients[2] +
                   3.0 * t2 * segment.coefficients[3] + 4.0 * t3 * segment.coefficients[4] +
                   5.0 * t4 * segment.coefficients[5];
  point.acceleration = 2.0 * segment.coefficients[2] +
                       6.0 * t * segment.coefficients[3] +
                       12.0 * t2 * segment.coefficients[4] +
                       20.0 * t3 * segment.coefficients[5];
  point.jerk = 6.0 * segment.coefficients[3] + 24.0 * t * segment.coefficients[4] +
               60.0 * t2 * segment.coefficients[5];
  point.snap = 24.0 * segment.coefficients[4] + 120.0 * t * segment.coefficients[5];
  return point;
}

inline navigation_mapping::Vec3 evaluateJerk(const QuinticSegment& segment, double time_s) {
  const double t = std::clamp(time_s, 0.0, segment.duration_s);
  return 6.0 * segment.coefficients[3] + 24.0 * t * segment.coefficients[4] +
         60.0 * t * t * segment.coefficients[5];
}

template <typename Model, typename Traversable>
std::vector<navigation_mapping::Vec3> simplifyPath(
    const std::vector<navigation_mapping::Vec3>& raw, double spacing,
    Traversable&& traversable, const Model& world, std::uint64_t& shortcut_count) {
  shortcut_count = 0;
  if (raw.size() <= 2U) return raw;

  // First remove strictly collinear voxel anchors.  Keeping the endpoints is
  // important because the first state and the goal carry boundary conditions.
  std::vector<navigation_mapping::Vec3> corner_reduced;
  corner_reduced.reserve(raw.size());
  corner_reduced.push_back(raw.front());
  for (std::size_t i = 1; i + 1U < raw.size(); ++i) {
    const auto before = raw[i] - raw[i - 1U];
    const auto after = raw[i + 1U] - raw[i];
    const double before_norm = before.norm();
    const double after_norm = after.norm();
    const bool collinear = before_norm > 1e-9 && after_norm > 1e-9 &&
                           (before.cross(after)).norm() <= 1e-9 * before_norm * after_norm &&
                           before.dot(after) > 0.0;
    if (!collinear) corner_reduced.push_back(raw[i]);
  }
  corner_reduced.push_back(raw.back());

  const auto lineOfSight = [&](const navigation_mapping::Vec3& start,
                               const navigation_mapping::Vec3& end) {
    const double length = (end - start).norm();
    const int samples = std::max(1, static_cast<int>(std::ceil(length / spacing)));
    for (int sample = 0; sample <= samples; ++sample) {
      const double alpha = static_cast<double>(sample) / static_cast<double>(samples);
      if (!traversable(start + alpha * (end - start))) return false;
      // Supercover the segment by probing the adjacent cell centres whenever
      // a sample lies on a voxel boundary.  This prevents diagonal corner
      // cutting through an inflated obstacle while retaining unknown policy.
      const auto index = world.worldToGrid(navigation_mapping::WorldLayer::Inflated,
                                           start + alpha * (end - start));
      const auto bounds = world.bounds(navigation_mapping::WorldLayer::Inflated);
      if (!bounds.contains(index)) return false;
      // Check the local supercover neighbourhood, not just the containing
      // cell.  This rejects a segment that threads exactly through the corner
      // of two occupied voxels (the classic diagonal corner-cut failure).
      for (int dx = -1; dx <= 1; ++dx) {
        for (int dy = -1; dy <= 1; ++dy) {
          for (int dz = -1; dz <= 1; ++dz) {
            const navigation_mapping::GridIndex3 neighbour{
                index.x + dx, index.y + dy, index.z + dz};
            if (!bounds.contains(neighbour)) continue;
            if (!traversable(world.gridToWorld(navigation_mapping::WorldLayer::Inflated,
                                               neighbour))) {
              // Only reject neighbours that can be touched by this sample's
              // voxel footprint; the inflated layer already encodes the
              // vehicle radius, so this conservative support check is safe.
              const auto neighbour_center =
                  world.gridToWorld(navigation_mapping::WorldLayer::Inflated, neighbour);
              if ((neighbour_center - (start + alpha * (end - start))).cwiseAbs().maxCoeff() <=
                  1.5 * world.resolution(navigation_mapping::WorldLayer::Inflated)) {
                return false;
              }
            }
          }
        }
      }
    }
    return true;
  };

  // The supercover test above is deliberately conservative: it rejects a
  // shortcut whenever an adjacent voxel could be touched at a grid corner.
  // That is the right first choice for safety, but it can leave a staircase
  // of 20--40 anchors around an inflated obstacle.  Time-parameterizing that
  // staircase forces a short jerk-limited segment for every voxel and can be
  // substantially slower than the actual geometric corridor.  Use a second,
  // still conservative point-sampled test as an optimization fallback.  The
  // resulting spline is checked again at 200 samples/segment and by the
  // runtime verifier; if it bows into an occupied voxel the planner restores
  // the raw A* corridor and fails closed.
  const auto pointwiseLineOfSight = [&](const navigation_mapping::Vec3& start,
                                        const navigation_mapping::Vec3& end) {
    const double length = (end - start).norm();
    const double point_spacing = std::max(0.05, 0.25 * world.resolution(
        navigation_mapping::WorldLayer::Inflated));
    const int samples = std::max(1, static_cast<int>(std::ceil(length / point_spacing)));
    for (int sample = 0; sample <= samples; ++sample) {
      const double alpha = static_cast<double>(sample) / static_cast<double>(samples);
      if (!traversable(start + alpha * (end - start))) return false;
    }
    return true;
  };

  std::vector<navigation_mapping::Vec3> simplified;
  simplified.reserve(corner_reduced.size());
  std::size_t anchor = 0;
  simplified.push_back(corner_reduced.front());
  while (anchor + 1U < corner_reduced.size()) {
    std::size_t farthest = anchor + 1U;
    for (std::size_t candidate = anchor + 2U; candidate < corner_reduced.size(); ++candidate) {
      if (lineOfSight(corner_reduced[anchor], corner_reduced[candidate]) ||
          pointwiseLineOfSight(corner_reduced[anchor], corner_reduced[candidate])) {
        farthest = candidate;
      }
    }
    if (farthest > anchor + 1U) ++shortcut_count;
    simplified.push_back(corner_reduced[farthest]);
    anchor = farthest;
  }
  return simplified;
}

template <typename Model>
PlanResult planModel(const PlannerConfig& config, const VehicleState& state,
                     const Goal& goal, const Model& world, UnknownPolicy unknown_policy,
                     PlanRole role) {
  const auto planning_started = std::chrono::steady_clock::now();
  PlanResult result;
  result.role = role;
  const auto finish = [&]() {
    result.statistics.total_planning_time_us =
        std::chrono::duration_cast<std::chrono::microseconds>(
            std::chrono::steady_clock::now() - planning_started)
            .count();
    return result;
  };

  if constexpr (requires { world.isReady(); }) {
    if (!world.isReady()) {
      result.failure_code = PlanFailureCode::WorldModelUnavailable;
      return finish();
    }
  }
  result.world_generation = world.generation();
  if constexpr (requires { world.revision(); }) {
    result.world_revision = world.revision();
  }
  double clearance_radius = std::numeric_limits<double>::quiet_NaN();
  if constexpr (requires { world.clearanceRadius(); }) {
    clearance_radius = world.clearanceRadius();
  }
  if (!std::isfinite(clearance_radius) || clearance_radius < 0.0) {
    result.failure_code = PlanFailureCode::ClearanceUnavailable;
    return finish();
  }
  if (!state.position.allFinite() || !state.velocity.allFinite() ||
      !state.acceleration.allFinite() || !goal.position.allFinite() ||
      !goal.terminal_velocity.allFinite() ||
      !std::isfinite(config.limits.max_velocity_mps) ||
      !std::isfinite(config.limits.max_acceleration_mps2) ||
      !std::isfinite(config.limits.max_deceleration_mps2) ||
      config.limits.max_velocity_mps <= 0.0 ||
      config.limits.max_acceleration_mps2 <= 0.0 ||
      config.limits.max_deceleration_mps2 <= 0.0 ||
      !std::isfinite(config.limits.max_jerk_mps3) ||
      config.limits.max_jerk_mps3 <= 0.0 ||
      (config.allow_unknown_start &&
       (!std::isfinite(config.unknown_start_radius_m) ||
        config.unknown_start_radius_m < 0.0)) ||
      !std::isfinite(config.trajectory_sample_dt_s) ||
      config.trajectory_sample_dt_s <= 0.0 || config.maximum_time_scaling_iterations < 0 ||
      !std::isfinite(config.bspline_time_scale) || config.bspline_time_scale <= 0.0 ||
      !std::isfinite(config.bspline_time_margin_s) || config.bspline_time_margin_s < 0.0 ||
      !std::isfinite(config.bspline_optimization_sample_dt_s) ||
      config.bspline_optimization_sample_dt_s <= 0.0) {
    result.failure_code = PlanFailureCode::InvalidState;
    return finish();
  }
  const double velocity_limit = goal.velocity_limit_mps > 0.0
                                    ? std::min(config.limits.max_velocity_mps,
                                               goal.velocity_limit_mps)
                                    : config.limits.max_velocity_mps;
  if (!std::isfinite(velocity_limit) || velocity_limit <= 0.0 ||
      (goal.velocity_limit_mps != 0.0 &&
       (!std::isfinite(goal.velocity_limit_mps) || goal.velocity_limit_mps < 0.0))) {
    result.failure_code = PlanFailureCode::InvalidState;
    return finish();
  }
  if (state.velocity.norm() > velocity_limit + 1e-9 ||
      state.acceleration.norm() > config.limits.max_acceleration_mps2 + 1e-9) {
    result.failure_code = PlanFailureCode::DynamicLimitsInfeasible;
    return finish();
  }

  SearchRequest request;
  request.layer = navigation_mapping::WorldLayer::Inflated;
  request.unknown_policy = unknown_policy;
  request.start_world = state.position;
  request.goal_world = goal.position;
  request.allow_unknown_start = config.allow_unknown_start;
  request.unknown_start_radius_m = config.unknown_start_radius_m;
  const auto search = AStar{}.searchModel(world, request);
  result.statistics.search = search.statistics;
  result.world_generation = search.world_generation;
  result.world_revision = search.world_revision;
  if (!search.success) {
    result.failure_code = mapSearchFailure(search.failure);
    return finish();
  }

  const auto corridor_started = std::chrono::steady_clock::now();
  std::vector<navigation_mapping::Vec3> raw_waypoints;
  raw_waypoints.reserve(search.path.size());
  for (const auto& index : search.path) {
    raw_waypoints.push_back(world.gridToWorld(navigation_mapping::WorldLayer::Inflated, index));
  }
  if (raw_waypoints.empty()) {
    result.failure_code = PlanFailureCode::CorridorInfeasible;
    return finish();
  }
  raw_waypoints.front() = state.position;
  raw_waypoints.back() = goal.position;
  if (raw_waypoints.size() == 1U &&
      (goal.position - state.position).norm() > 1e-9) {
    raw_waypoints.push_back(goal.position);
  }
  const double resolution = world.resolution(navigation_mapping::WorldLayer::Inflated);
  const double sample_spacing = config.corridor_sample_spacing_m > 0.0
                                    ? config.corridor_sample_spacing_m
                                    : 0.5 * resolution;
  if (!std::isfinite(sample_spacing) || sample_spacing <= 0.0) {
    result.failure_code = PlanFailureCode::CorridorInfeasible;
    return finish();
  }
  const auto start_index = world.worldToGrid(navigation_mapping::WorldLayer::Inflated,
                                             state.position);
  const double cell_half_diagonal = 0.5 * std::sqrt(3.0) * resolution;
  const auto trustedUnknownStart = [&](const navigation_mapping::GridIndex3& index,
                                       const navigation_mapping::CellState cell_state) {
    if (!config.allow_unknown_start || cell_state != navigation_mapping::CellState::Unknown) {
      return false;
    }
    if (index == start_index) return true;
    if (!std::isfinite(config.unknown_start_radius_m) ||
        config.unknown_start_radius_m <= 0.0) {
      return false;
    }
    return (world.gridToWorld(navigation_mapping::WorldLayer::Inflated, index) -
            state.position).norm() <=
           config.unknown_start_radius_m + cell_half_diagonal + 1e-9;
  };
  const auto traversable = [&](const navigation_mapping::Vec3& position) {
    if (!position.allFinite()) return false;
    const auto index = world.worldToGrid(navigation_mapping::WorldLayer::Inflated, position);
    if (!world.bounds(navigation_mapping::WorldLayer::Inflated).contains(index)) return false;
    const auto state = world.cellState(navigation_mapping::WorldLayer::Inflated, index);
    return state == navigation_mapping::CellState::KnownFree ||
           (unknown_policy == UnknownPolicy::TreatUnknownAsTraversable &&
            state == navigation_mapping::CellState::Unknown) ||
           trustedUnknownStart(index, state);
  };

  // A 26-neighbour search is allowed to use vertical cells when that is the
  // only collision-free topology.  In an otherwise open flight tube, however,
  // tie-breaking can select a harmless but needlessly changing z cell.  A
  // quintic through those anchors turns the harmless grid choice into a large
  // altitude command (especially when the map is revised during flight).
  // Prefer the requested flight level whenever the complete projected route is
  // known-free; retain the original 3-D path if any projection would be
  // occupied/unknown.  This is a geometry-preserving safety check, not an
  // assumption that obstacles are 2-D.
  bool level_projection_free = true;
  std::vector<navigation_mapping::Vec3> level_waypoints = raw_waypoints;
  for (std::size_t index = 1; index + 1U < level_waypoints.size(); ++index) {
    auto projected = level_waypoints[index];
    projected.z() = goal.position.z();
    if (!traversable(projected)) {
      level_projection_free = false;
      break;
    }
    level_waypoints[index] = projected;
  }
  if (level_projection_free && level_waypoints.size() > 1U) {
    level_waypoints.front() = state.position;
    level_waypoints.back() = goal.position;
    raw_waypoints = std::move(level_waypoints);
  }

  std::uint64_t shortcut_count = 0;
  auto waypoints = detail::simplifyPath(raw_waypoints, sample_spacing, traversable, world,
                                        shortcut_count);
  // A two-point route is already minimal.  For a route with corners the
  // shortcut pass may conservatively retain anchors when the supercover test
  // cannot prove the diagonal safe.
  result.statistics.trajectory_optimization.raw_path_node_count = raw_waypoints.size();
  result.statistics.trajectory_optimization.simplified_path_node_count = waypoints.size();
  result.statistics.trajectory_optimization.shortcut_count = shortcut_count;
  for (std::size_t i = 1; i < raw_waypoints.size(); ++i) {
    result.statistics.trajectory_optimization.geometric_path_length_m +=
        (raw_waypoints[i] - raw_waypoints[i - 1U]).norm();
  }
  for (std::size_t segment = 0; segment < waypoints.size(); ++segment) {
    const auto& start = waypoints[segment];
    const auto& end = segment + 1 < waypoints.size() ? waypoints[segment + 1] : start;
    const double length = (end - start).norm();
    const int sample_count = std::max(1, static_cast<int>(std::ceil(length / sample_spacing)));
    for (int sample = 0; sample <= sample_count; ++sample) {
      const double alpha = static_cast<double>(sample) / sample_count;
      ++result.statistics.corridor.checked_sample_count;
      if (!traversable(start + alpha * (end - start))) {
        ++result.statistics.corridor.blocked_sample_count;
      }
    }
  }
  result.statistics.corridor.segment_count = waypoints.size() > 1 ? waypoints.size() - 1 : 0;
  result.statistics.corridor.minimum_clearance_radius_m = clearance_radius;
  result.statistics.corridor.corridor_time_us =
      std::chrono::duration_cast<std::chrono::microseconds>(
          std::chrono::steady_clock::now() - corridor_started)
          .count();
  if (result.statistics.corridor.blocked_sample_count != 0) {
    result.failure_code = PlanFailureCode::CorridorInfeasible;
    return finish();
  }

  if (config.trajectory_generator == TrajectoryGeneratorKind::Bspline) {
    const auto optimization_started = std::chrono::steady_clock::now();
    const double path_length = result.statistics.trajectory_optimization.geometric_path_length_m;
    const double target_duration = std::max(
        0.6, config.bspline_time_scale * path_length / velocity_limit +
                 config.bspline_time_margin_s);
    const double approximate_span_count = std::max(
        2.0, std::ceil(path_length / 0.8));
    const double base_knot_dt = std::max(0.12, target_duration / approximate_span_count);
    const auto terminal_velocity = goal.terminal ? navigation_mapping::Vec3::Zero()
                                                 : goal.terminal_velocity;
    const auto terminal_acceleration = navigation_mapping::Vec3::Zero();
    bool generated = false;
    BsplineGenerationResult best_generation;
    bool best_collision_free = false;

    // First try the smooth projected fit. If it bows into a voxel corner, keep
    // the same B-spline representation but refit without the smoothing pass;
    // the dense collision verifier below remains the final authority.
    for (int smoothing_pass = 0; smoothing_pass < 2 && !generated; ++smoothing_pass) {
      for (int iteration = 0; iteration <= config.maximum_time_scaling_iterations;
           ++iteration) {
        BsplineGenerationConfig bspline_config;
        bspline_config.degree = 5;
        // A shorter initial duration is intentional for speed, so use a
        // slightly stronger retry expansion to recover dynamic feasibility in
        // one or two additional candidates instead of carrying a slow base
        // duration through every rolling replan.
        bspline_config.knot_dt_s = base_knot_dt * std::pow(1.35, iteration);
        bspline_config.sample_dt_s = config.bspline_optimization_sample_dt_s;
        bspline_config.smoothing_iterations = smoothing_pass == 0
                                                  ? config.bspline_smoothing_iterations
                                                  : 0;
        bspline_config.smoothing_step = config.bspline_smoothing_step;
        bspline_config.snap_weight = config.bspline_snap_weight;
        bspline_config.path_length_weight = config.bspline_path_length_weight;
        bspline_config.reference_weight = config.bspline_reference_weight;
        bspline_config.max_velocity_mps = velocity_limit;
        bspline_config.max_acceleration_mps2 = config.limits.max_acceleration_mps2;
        bspline_config.max_deceleration_mps2 = config.limits.max_deceleration_mps2;
        bspline_config.max_jerk_mps3 = config.limits.max_jerk_mps3;
        const auto generation = generateBsplineTrajectory(
            waypoints, state.velocity, state.acceleration, terminal_velocity,
            terminal_acceleration, bspline_config);
        if (!generation.success) {
          best_generation = generation;
          continue;
        }
        bool collision_free = true;
        // Uniform B-splines stay inside the convex hull of their active
        // control points. Check the control-point support first, then sample
        // the curve densely; this is the cheap corridor certificate that
        // prevents an optimizer update from creating an unverified bow.
        for (const auto& control_point : generation.trajectory.control_points) {
          if (!traversable(control_point)) {
            collision_free = false;
            ++result.statistics.trajectory_optimization.collision_check_failure_count;
            if (result.statistics.trajectory_optimization.collision_check_failure_count == 1U) {
              result.statistics.trajectory_optimization.first_collision_position = control_point;
            }
            break;
          }
        }
        const double duration = generation.trajectory.duration();
        const double collision_sample_dt = std::max(
            0.02, std::min(config.trajectory_sample_dt_s,
                           config.bspline_optimization_sample_dt_s));
        const int samples = std::max(
            2, static_cast<int>(std::ceil(duration / collision_sample_dt)));
        for (int sample = 0; collision_free && sample <= samples; ++sample) {
          const double time = duration * static_cast<double>(sample) /
                              static_cast<double>(samples);
          if (!traversable(generation.trajectory.evaluatePosition(time))) {
            collision_free = false;
            ++result.statistics.trajectory_optimization.collision_check_failure_count;
            if (result.statistics.trajectory_optimization.collision_check_failure_count == 1U) {
              result.statistics.trajectory_optimization.first_collision_position =
                  generation.trajectory.evaluatePosition(time);
            }
            break;
          }
        }
        best_generation = generation;
        best_collision_free = collision_free;
        if (collision_free) {
          generated = true;
          break;
        }
      }
    }

    result.statistics.trajectory_optimization.optimization_time_us =
        std::chrono::duration_cast<std::chrono::microseconds>(
            std::chrono::steady_clock::now() - optimization_started)
            .count();
    result.statistics.trajectory_optimization.segment_count =
        best_generation.trajectory.spanCount();
    result.statistics.trajectory_optimization.iterations =
        static_cast<std::uint64_t>(config.maximum_time_scaling_iterations + 1);
    result.statistics.trajectory_optimization.sampled_point_count =
        best_generation.sampled_point_count;
    result.statistics.trajectory_optimization.maximum_velocity_mps =
        best_generation.maximum_velocity_mps;
    result.statistics.trajectory_optimization.maximum_acceleration_mps2 =
        best_generation.maximum_acceleration_mps2;
    result.statistics.trajectory_optimization.maximum_deceleration_mps2 =
        best_generation.maximum_deceleration_mps2;
    result.statistics.trajectory_optimization.maximum_jerk_mps3 =
        best_generation.maximum_jerk_mps3;
    result.statistics.trajectory_optimization.integrated_squared_jerk =
        best_generation.integrated_squared_jerk;
    result.statistics.trajectory_optimization.integrated_squared_snap =
        best_generation.integrated_squared_snap;
    result.statistics.trajectory_optimization.objective_cost = best_generation.objective_cost;
    result.statistics.trajectory_optimization.trajectory_length_m =
        best_generation.geometric_length_m;
    result.statistics.trajectory_optimization.c2_continuity_residual = 0.0;
    result.statistics.trajectory_optimization.collision_free = generated && best_collision_free;
    result.statistics.trajectory_optimization.dynamic_limits_satisfied = best_generation.success;
    result.statistics.trajectory_optimization.duration_s = best_generation.trajectory.duration();
    if (!generated || !best_generation.trajectory.valid()) {
      result.failure_code = best_collision_free ? PlanFailureCode::DynamicLimitsInfeasible
                                                : PlanFailureCode::CorridorInfeasible;
      return finish();
    }

    const double duration = best_generation.trajectory.duration();
    // The published P/V/A/J/S trajectory remains dense enough for the
    // controller, while avoiding a needless 20 ms expansion of every long
    // rolling horizon. Collision verification above is independent.
    const double sample_dt = config.trajectory_sample_dt_s;
    const int samples = std::max(1, static_cast<int>(std::ceil(duration / sample_dt)));
    result.trajectory.points.reserve(static_cast<std::size_t>(samples + 1));
    for (int sample = 0; sample <= samples; ++sample) {
      const double time = duration * static_cast<double>(sample) /
                          static_cast<double>(samples);
      const auto value = best_generation.trajectory.evaluate(time);
      result.trajectory.points.push_back(
          TrajectoryPoint{time, value.position, value.velocity, value.acceleration, value.jerk,
                          value.snap});
    }
    result.trajectory.duration_s = duration;
    if (!result.trajectory.points.empty()) {
      result.trajectory.points.front().position = state.position;
      result.trajectory.points.front().velocity = state.velocity;
      result.trajectory.points.front().acceleration = state.acceleration;
      result.trajectory.points.back().position = goal.position;
      result.trajectory.points.back().velocity = terminal_velocity;
      result.trajectory.points.back().acceleration = terminal_acceleration;
    }
    result.statistics.trajectory_optimization.duration_s = duration;
    if (!result.trajectory.finiteAndMonotonic()) {
      result.failure_code = PlanFailureCode::TrajectoryInvalid;
      result.trajectory.points.clear();
      return finish();
    }
    bool world_changed = world.generation() != result.world_generation;
    if constexpr (requires { world.revision(); }) {
      world_changed = world_changed || world.revision() != result.world_revision;
    }
    if (world_changed) {
      result.failure_code = PlanFailureCode::WorldChanged;
      result.trajectory.points.clear();
      return finish();
    }
    result.success = true;
    result.failure_code = PlanFailureCode::None;
    return finish();
  }

  const auto optimization_started = std::chrono::steady_clock::now();
  std::vector<double> base_durations;
  if (waypoints.size() > 1) {
    base_durations.reserve(waypoints.size() - 1);
    for (std::size_t i = 1; i < waypoints.size(); ++i) {
      const double distance = (waypoints[i] - waypoints[i - 1]).norm();
      const double velocity_time = 1.2 * distance / velocity_limit;
      const double acceleration_time =
          std::sqrt(6.0 * std::max(distance, 1e-6) / config.limits.max_acceleration_mps2);
      const double deceleration_time =
          std::sqrt(6.0 * std::max(distance, 1e-6) / config.limits.max_deceleration_mps2);
      const double jerk_time = std::cbrt(
          60.0 * std::max(distance, 1e-6) / config.limits.max_jerk_mps3);
      base_durations.push_back(
          std::max({0.2, velocity_time, acceleration_time, deceleration_time, jerk_time}));
    }
  }

  std::vector<QuinticSegment> segments;
  std::vector<double> durations = base_durations;
  bool using_raw_repair = false;
  bool using_zero_tangent_repair = false;
  bool optimized = false;
  for (int iteration = 0; iteration <= config.maximum_time_scaling_iterations; ++iteration) {
    segments.clear();
    std::vector<navigation_mapping::Vec3> tangents(waypoints.size(),
                                                    navigation_mapping::Vec3::Zero());
    if (!using_zero_tangent_repair) {
      if (!waypoints.empty()) tangents.front() = state.velocity;
      for (std::size_t i = 1; i + 1U < waypoints.size(); ++i) {
        const double span = durations[i - 1U] + durations[i];
      if (span > 1e-9) {
          tangents[i] = (waypoints[i + 1U] - waypoints[i - 1U]) / span;
          const double norm = tangents[i].norm();
          const double design_limit = 0.995 * velocity_limit;
          if (norm > design_limit) {
            tangents[i] *= design_limit / norm;
          }
        }
      }
    } else if (waypoints.size() > 1U) {
      // A raw A* corridor is line-safe by construction.  If a moving-tangent
      // spline still bows through an inflated corner, refit that corridor with
      // zero interior tangents.  Quintic smooth-step segments remain C2 and
      // stay on each safe line segment; this is a bounded safety repair, not a
      // runtime legacy mode.
      const auto first_delta = waypoints[1] - waypoints.front();
      if (first_delta.norm() > 1e-9 && state.velocity.dot(first_delta) > 0.0) {
        tangents.front() = state.velocity;
      }
    }
    if (!goal.terminal && waypoints.size() > 1U) {
      // Intermediate receding-horizon targets must carry motion through the
      // local-map window. Clamp only the newly generated tangent; the measured
      // initial tangent remains untouched because it cannot be changed
      // retroactively. The collision/dynamic verifier still has final veto.
      tangents.back() = goal.terminal_velocity;
      const double terminal_norm = tangents.back().norm();
      const double design_limit = 0.995 * velocity_limit;
      if (terminal_norm > design_limit && terminal_norm > 1e-9) {
        tangents.back() *= design_limit / terminal_norm;
      }
    }
    for (std::size_t i = 1; i < waypoints.size(); ++i) {
      const auto v0 = tangents[i - 1U];
      const auto a0 = i == 1 ? state.acceleration : navigation_mapping::Vec3::Zero();
      segments.push_back(makeQuintic(waypoints[i - 1], v0, a0, waypoints[i],
                                     tangents[i],
                                     navigation_mapping::Vec3::Zero(),
                                     durations[i - 1]));
    }
    double maximum_velocity = 0.0;
    double maximum_acceleration = 0.0;
    double maximum_deceleration = 0.0;
    double maximum_jerk = 0.0;
    double integrated_squared_jerk = 0.0;
    double trajectory_length = 0.0;
    bool finite = true;
    bool collision_free = true;
    double absolute_time = 0.0;
    std::vector<bool> segment_bad(segments.size(), false);
    for (std::size_t segment_index = 0; segment_index < segments.size(); ++segment_index) {
      const auto& segment = segments[segment_index];
      double previous_jerk_sq = 0.0;
      navigation_mapping::Vec3 previous_position = navigation_mapping::Vec3::Zero();
      for (int sample = 0; sample <= 200; ++sample) {
        const double alpha = static_cast<double>(sample) / 200.0;
        const auto point = evaluate(segment, alpha * segment.duration_s, absolute_time +
                                                              alpha * segment.duration_s);
        const auto jerk = evaluateJerk(segment, alpha * segment.duration_s);
        finite = finite && point.position.allFinite() && point.velocity.allFinite() &&
                 point.acceleration.allFinite();
        maximum_velocity = std::max(maximum_velocity, point.velocity.norm());
        maximum_acceleration = std::max(maximum_acceleration, point.acceleration.norm());
        maximum_jerk = std::max(maximum_jerk, jerk.norm());
        if (sample > 0) {
          const double dt = segment.duration_s / 200.0;
          integrated_squared_jerk +=
              0.5 * (previous_jerk_sq + jerk.squaredNorm()) * dt;
          trajectory_length += (point.position - previous_position).norm();
        }
        previous_jerk_sq = jerk.squaredNorm();
        previous_position = point.position;
        if (point.velocity.norm() > 1e-9) {
          maximum_deceleration = std::max(
              maximum_deceleration,
              std::max(0.0, -point.acceleration.dot(point.velocity.normalized())));
        }
        if (!traversable(point.position)) {
          collision_free = false;
          segment_bad[segment_index] = true;
        }
      }
      if (segments[segment_index].duration_s > 0.0) {
        const double seg_velocity = maximum_velocity;
        (void)seg_velocity;
      }
      absolute_time += segment.duration_s;
    }
    result.statistics.trajectory_optimization.maximum_velocity_mps = maximum_velocity;
    result.statistics.trajectory_optimization.maximum_acceleration_mps2 = maximum_acceleration;
    result.statistics.trajectory_optimization.maximum_deceleration_mps2 = maximum_deceleration;
    result.statistics.trajectory_optimization.maximum_jerk_mps3 = maximum_jerk;
    result.statistics.trajectory_optimization.integrated_squared_jerk = integrated_squared_jerk;
    result.statistics.trajectory_optimization.trajectory_length_m = trajectory_length;
    result.statistics.trajectory_optimization.collision_free = collision_free;
    result.statistics.trajectory_optimization.dynamic_limits_satisfied =
        finite && maximum_velocity <= velocity_limit + 1e-9 &&
        maximum_acceleration <=
            std::max(config.limits.max_acceleration_mps2,
                     config.limits.max_deceleration_mps2) * 0.995 &&
        maximum_deceleration <= config.limits.max_deceleration_mps2 * 0.995 &&
        maximum_jerk <= config.limits.max_jerk_mps3 * 0.995;
    result.statistics.trajectory_optimization.iterations = iteration + 1;
    if (result.statistics.trajectory_optimization.dynamic_limits_satisfied &&
        collision_free) {
      optimized = true;
      break;
    }
    if (!collision_free && !using_raw_repair) {
      // A spline can bow outside a voxel-safe shortcut.  Reinsert the raw
      // A* anchors once, then refit; if that still fails the existing safety
      // stop path receives CorridorInfeasible.
      waypoints = raw_waypoints;
      durations.clear();
      durations.reserve(waypoints.size() > 1U ? waypoints.size() - 1U : 0U);
      for (std::size_t i = 1; i < waypoints.size(); ++i) {
        const double distance = (waypoints[i] - waypoints[i - 1U]).norm();
        durations.push_back(std::max({0.2, 1.2 * distance / velocity_limit,
                                      std::sqrt(6.0 * std::max(distance, 1e-6) /
                                                config.limits.max_acceleration_mps2),
                                      std::sqrt(6.0 * std::max(distance, 1e-6) /
                                                config.limits.max_deceleration_mps2),
                                      std::cbrt(60.0 * std::max(distance, 1e-6) /
                                                config.limits.max_jerk_mps3)}));
      }
      using_raw_repair = true;
      continue;
    }
    if (!collision_free && using_raw_repair && !using_zero_tangent_repair) {
      using_zero_tangent_repair = true;
      for (auto& duration : durations) duration *= 1.25;
      continue;
    }
    for (std::size_t i = 0; i < durations.size(); ++i) {
      if (segment_bad.empty() || i >= segment_bad.size() || segment_bad[i] ||
          !result.statistics.trajectory_optimization.dynamic_limits_satisfied) {
        durations[i] *= 1.25;
      }
    }
  }
  result.statistics.trajectory_optimization.optimization_time_us =
      std::chrono::duration_cast<std::chrono::microseconds>(
          std::chrono::steady_clock::now() - optimization_started)
          .count();
  result.statistics.trajectory_optimization.segment_count = segments.size();
  result.statistics.trajectory_optimization.simplified_path_node_count = waypoints.size();
  double c2_residual = 0.0;
  for (std::size_t i = 1; i < segments.size(); ++i) {
    const auto left_accel = evaluate(segments[i - 1U], segments[i - 1U].duration_s, 0.0).acceleration;
    const auto right_accel = evaluate(segments[i], 0.0, 0.0).acceleration;
    c2_residual = std::max(c2_residual, (left_accel - right_accel).norm());
  }
  result.statistics.trajectory_optimization.c2_continuity_residual = c2_residual;
  if (!optimized) {
    result.failure_code =
        result.statistics.trajectory_optimization.dynamic_limits_satisfied
            ? PlanFailureCode::CorridorInfeasible
            : PlanFailureCode::DynamicLimitsInfeasible;
    return finish();
  }

  double absolute_time = 0.0;
  result.trajectory.points.clear();
  if (segments.empty()) {
    result.trajectory.points.push_back(TrajectoryPoint{0.0, state.position, state.velocity,
                                                      state.acceleration});
  } else {
    for (std::size_t segment_index = 0; segment_index < segments.size(); ++segment_index) {
      const auto& segment = segments[segment_index];
      const int sample_count = std::max(
          1, static_cast<int>(std::ceil(segment.duration_s / config.trajectory_sample_dt_s)));
      const int first_sample = segment_index == 0 ? 0 : 1;
      for (int sample = first_sample; sample <= sample_count; ++sample) {
        const double alpha = static_cast<double>(sample) / sample_count;
        result.trajectory.points.push_back(
            evaluate(segment, alpha * segment.duration_s,
                     absolute_time + alpha * segment.duration_s));
      }
      absolute_time += segment.duration_s;
    }
  }
  result.trajectory.duration_s = absolute_time;
  // Preserve the mission contract exactly at the terminal knot.  Polynomial
  // evaluation can otherwise leave a few ulps of error, which is observable
  // in deterministic reports and can move an acceptance-radius check across
  // a voxel boundary.
  if (!result.trajectory.points.empty()) {
    result.trajectory.points.front().position = state.position;
    result.trajectory.points.back().position = goal.position;
    if (goal.terminal) {
      result.trajectory.points.back().velocity = navigation_mapping::Vec3::Zero();
    } else {
      auto endpoint_velocity = goal.terminal_velocity;
      const double endpoint_norm = endpoint_velocity.norm();
      const double design_limit = 0.995 * velocity_limit;
      if (endpoint_norm > design_limit && endpoint_norm > 1e-9) {
        endpoint_velocity *= design_limit / endpoint_norm;
      }
      result.trajectory.points.back().velocity = endpoint_velocity;
    }
    result.trajectory.points.back().acceleration = navigation_mapping::Vec3::Zero();
  }
  result.statistics.trajectory_optimization.sampled_point_count =
      result.trajectory.points.size();
  result.statistics.trajectory_optimization.duration_s = absolute_time;
  if (!result.trajectory.finiteAndMonotonic()) {
    result.failure_code = PlanFailureCode::TrajectoryInvalid;
    result.trajectory.points.clear();
    return finish();
  }
  bool world_changed = world.generation() != result.world_generation;
  if constexpr (requires { world.revision(); }) {
    world_changed = world_changed || world.revision() != result.world_revision;
  }
  if (world_changed) {
    result.failure_code = PlanFailureCode::WorldChanged;
    result.trajectory.points.clear();
    return finish();
  }
  result.success = true;
  result.failure_code = PlanFailureCode::None;
  return finish();
}

}  // namespace detail

template <typename Model>
PlanResult Planner::planModel(const VehicleState& state, const Goal& goal,
                              const Model& world, UnknownPolicy unknown_policy,
                              PlanRole role) const {
  return detail::planModel(config_, state, goal, world, unknown_policy, role);
}

}  // namespace navigation_planning
