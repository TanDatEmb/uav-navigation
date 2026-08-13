#pragma once

#include <cstdint>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <vector>

#include <Eigen/Core>

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
};

struct DynamicLimits {
  double max_velocity_mps{2.0};
  double max_acceleration_mps2{3.0};
};

struct PlannerConfig {
  DynamicLimits limits{};
  double trajectory_sample_dt_s{0.05};
  double corridor_sample_spacing_m{0.0};
  int maximum_time_scaling_iterations{8};
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

class Planner final {
 public:
  explicit Planner(PlannerConfig config = {});

  [[nodiscard]] PlanResult plan(const VehicleState& state, const Goal& goal,
                                const navigation_mapping::WorldModel& world) const;

  // Deterministic model hook for geometry and trajectory tests. Production
  // planning uses plan(WorldModel) and never exposes a ROS or vendor map type.
  template <typename Model>
  [[nodiscard]] PlanResult planForTest(const VehicleState& state, const Goal& goal,
                                       const Model& model) const {
    return planModel(state, goal, model);
  }

 private:
  template <typename Model>
  [[nodiscard]] PlanResult planModel(const VehicleState& state, const Goal& goal,
                                     const Model& world) const;

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

template <typename Model>
PlanResult planModel(const PlannerConfig& config, const VehicleState& state,
                     const Goal& goal, const Model& world) {
  const auto planning_started = std::chrono::steady_clock::now();
  PlanResult result;
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
      !std::isfinite(config.limits.max_velocity_mps) ||
      !std::isfinite(config.limits.max_acceleration_mps2) ||
      config.limits.max_velocity_mps <= 0.0 ||
      config.limits.max_acceleration_mps2 <= 0.0 ||
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
  request.unknown_policy = UnknownPolicy::TreatUnknownAsBlocked;
  request.start_world = state.position;
  request.goal_world = goal.position;
  const auto search = AStar{}.searchForTest(world, request);
  result.statistics.search = search.statistics;
  result.world_generation = search.world_generation;
  result.world_revision = search.world_revision;
  if (!search.success) {
    result.failure_code = mapSearchFailure(search.failure);
    return finish();
  }

  const auto corridor_started = std::chrono::steady_clock::now();
  std::vector<navigation_mapping::Vec3> waypoints;
  waypoints.reserve(search.path.size());
  for (const auto& index : search.path) {
    waypoints.push_back(world.gridToWorld(navigation_mapping::WorldLayer::Inflated, index));
  }
  if (waypoints.empty()) {
    result.failure_code = PlanFailureCode::CorridorInfeasible;
    return finish();
  }
  waypoints.front() = state.position;
  waypoints.back() = goal.position;
  const double resolution = world.resolution(navigation_mapping::WorldLayer::Inflated);
  const double sample_spacing = config.corridor_sample_spacing_m > 0.0
                                    ? config.corridor_sample_spacing_m
                                    : 0.5 * resolution;
  if (!std::isfinite(sample_spacing) || sample_spacing <= 0.0) {
    result.failure_code = PlanFailureCode::CorridorInfeasible;
    return finish();
  }
  const auto traversable = [&](const navigation_mapping::Vec3& position) {
    if (!position.allFinite()) return false;
    const auto index = world.worldToGrid(navigation_mapping::WorldLayer::Inflated, position);
    if (!world.bounds(navigation_mapping::WorldLayer::Inflated).contains(index)) return false;
    return world.cellState(navigation_mapping::WorldLayer::Inflated, index) ==
           navigation_mapping::CellState::KnownFree;
  };
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
      base_durations.push_back(std::max({0.2, velocity_time, acceleration_time}));
    }
  }

  std::vector<QuinticSegment> segments;
  bool optimized = false;
  for (int iteration = 0; iteration <= config.maximum_time_scaling_iterations; ++iteration) {
    const double scale = std::pow(1.5, iteration);
    segments.clear();
    for (std::size_t i = 1; i < waypoints.size(); ++i) {
      const auto v0 = i == 1 ? state.velocity : navigation_mapping::Vec3::Zero();
      const auto a0 = i == 1 ? state.acceleration : navigation_mapping::Vec3::Zero();
      segments.push_back(makeQuintic(waypoints[i - 1], v0, a0, waypoints[i],
                                     navigation_mapping::Vec3::Zero(),
                                     navigation_mapping::Vec3::Zero(),
                                     base_durations[i - 1] * scale));
    }
    double maximum_velocity = 0.0;
    double maximum_acceleration = 0.0;
    bool finite = true;
    bool collision_free = true;
    double absolute_time = 0.0;
    for (const auto& segment : segments) {
      for (int sample = 0; sample <= 200; ++sample) {
        const double alpha = static_cast<double>(sample) / 200.0;
        const auto point = evaluate(segment, alpha * segment.duration_s, absolute_time +
                                                              alpha * segment.duration_s);
        finite = finite && point.position.allFinite() && point.velocity.allFinite() &&
                 point.acceleration.allFinite();
        maximum_velocity = std::max(maximum_velocity, point.velocity.norm());
        maximum_acceleration = std::max(maximum_acceleration, point.acceleration.norm());
        collision_free = collision_free && traversable(point.position);
      }
      absolute_time += segment.duration_s;
    }
    result.statistics.trajectory_optimization.maximum_velocity_mps = maximum_velocity;
    result.statistics.trajectory_optimization.maximum_acceleration_mps2 = maximum_acceleration;
    result.statistics.trajectory_optimization.iterations = iteration + 1;
    if (finite && collision_free &&
        maximum_velocity <= config.limits.max_velocity_mps * 0.995 &&
        maximum_acceleration <= config.limits.max_acceleration_mps2 * 0.995) {
      optimized = true;
      break;
    }
  }
  result.statistics.trajectory_optimization.optimization_time_us =
      std::chrono::duration_cast<std::chrono::microseconds>(
          std::chrono::steady_clock::now() - optimization_started)
          .count();
  result.statistics.trajectory_optimization.segment_count = segments.size();
  if (!optimized) {
    result.failure_code = PlanFailureCode::DynamicLimitsInfeasible;
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
                              const Model& world) const {
  return detail::planModel(config_, state, goal, world);
}

}  // namespace navigation_planning
