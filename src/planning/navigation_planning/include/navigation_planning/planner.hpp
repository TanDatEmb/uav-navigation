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
  double trajectory_sample_dt_s{0.05};
  double corridor_sample_spacing_m{0.0};
  int maximum_time_scaling_iterations{8};
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
  double c2_continuity_residual{0.0};
  double geometric_path_length_m{0.0};
  double trajectory_length_m{0.0};
  std::uint64_t raw_path_node_count{0};
  std::uint64_t simplified_path_node_count{0};
  std::uint64_t shortcut_count{0};
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
  const double acceleration = config.limits.max_deceleration_mps2;
  const double duration = speed > 1e-9 ? speed / acceleration : 0.0;
  const double spatial_step = std::isfinite(resolution) && resolution > 0.0
                                  ? 0.5 * resolution
                                  : std::numeric_limits<double>::infinity();
  const double stopping_distance = 0.5 * speed * duration;
  const int time_samples = std::max(
      1, static_cast<int>(std::ceil(duration / config.trajectory_sample_dt_s)));
  const int spatial_samples = std::isfinite(spatial_step)
                                  ? std::max(1, static_cast<int>(
                                                     std::ceil(stopping_distance / spatial_step)))
                                  : 1;
  const int sample_count = std::max(time_samples, spatial_samples);
  navigation_mapping::Vec3 braking_acceleration = navigation_mapping::Vec3::Zero();
  if (speed > 1e-9) {
    braking_acceleration = -acceleration * state.velocity.normalized();
  }

  result.statistics.corridor.segment_count = duration > 0.0 ? 1U : 0U;
  result.statistics.corridor.minimum_clearance_radius_m = world.clearanceRadius();
  result.trajectory.points.reserve(static_cast<std::size_t>(sample_count + 1));
  const int last_sample = duration > 0.0 ? sample_count : 0;
  for (int sample = 0; sample <= last_sample; ++sample) {
    const double time = duration * static_cast<double>(sample) /
                        static_cast<double>(sample_count);
    const auto position = state.position + state.velocity * time +
                          0.5 * braking_acceleration * time * time;
    const auto velocity = state.velocity + braking_acceleration * time;
    navigation_mapping::Vec3 clamped_velocity = velocity;
    if (sample == last_sample) {
      clamped_velocity = navigation_mapping::Vec3::Zero();
    }
    ++result.statistics.corridor.checked_sample_count;
    if (!isKnownFree(position)) {
      ++result.statistics.corridor.blocked_sample_count;
      result.trajectory.points.clear();
      result.failure_code = PlanFailureCode::SafetyStopUnavailable;
      return finish();
    }
    result.trajectory.points.push_back(
        TrajectoryPoint{time, position, clamped_velocity, braking_acceleration});
  }
  result.trajectory.points.back().acceleration = navigation_mapping::Vec3::Zero();
  result.trajectory.duration_s = duration;
  result.statistics.trajectory_optimization.sampled_point_count =
      result.trajectory.points.size();
  result.statistics.trajectory_optimization.maximum_velocity_mps = speed;
  result.statistics.trajectory_optimization.maximum_acceleration_mps2 = acceleration;
  result.statistics.trajectory_optimization.maximum_deceleration_mps2 = acceleration;
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

  std::vector<navigation_mapping::Vec3> simplified;
  simplified.reserve(corner_reduced.size());
  std::size_t anchor = 0;
  simplified.push_back(corner_reduced.front());
  while (anchor + 1U < corner_reduced.size()) {
    std::size_t farthest = anchor + 1U;
    for (std::size_t candidate = anchor + 2U; candidate < corner_reduced.size(); ++candidate) {
      if (lineOfSight(corner_reduced[anchor], corner_reduced[candidate])) farthest = candidate;
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
      config.trajectory_sample_dt_s <= 0.0 || config.maximum_time_scaling_iterations < 0) {
    result.failure_code = PlanFailureCode::InvalidState;
    return finish();
  }
  if (state.velocity.norm() > config.limits.max_velocity_mps + 1e-9 ||
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

  const auto optimization_started = std::chrono::steady_clock::now();
  std::vector<double> base_durations;
  if (waypoints.size() > 1) {
    base_durations.reserve(waypoints.size() - 1);
    for (std::size_t i = 1; i < waypoints.size(); ++i) {
      const double distance = (waypoints[i] - waypoints[i - 1]).norm();
      const double velocity_time = 1.2 * distance / config.limits.max_velocity_mps;
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
          const double design_limit = designVelocityLimit(config.limits);
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
      const double design_limit = designVelocityLimit(config.limits);
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
        finite && maximum_velocity <= config.limits.max_velocity_mps + 1e-9 &&
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
        durations.push_back(std::max({0.2, 1.2 * distance / config.limits.max_velocity_mps,
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
      const double design_limit = designVelocityLimit(config.limits);
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
