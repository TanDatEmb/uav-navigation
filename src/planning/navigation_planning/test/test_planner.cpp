#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <vector>

#include "navigation_planning/planner.hpp"
#include "navigation_planning/trajectory_verifier.hpp"

namespace navigation_planning {
namespace {

class TestWorldModel {
 public:
  TestWorldModel(int size_x, int size_y, int size_z, double resolution = 0.5)
      : bounds_{{0, 0, 0}, {size_x - 1, size_y - 1, size_z - 1}},
        size_x_(size_x),
        size_y_(size_y),
        size_z_(size_z),
        resolution_(resolution),
        cells_(static_cast<std::size_t>(size_x * size_y * size_z),
               navigation_mapping::CellState::Unknown) {}

  void fill(navigation_mapping::CellState state) { std::fill(cells_.begin(), cells_.end(), state); }

  void set(const navigation_mapping::GridIndex3& index,
           navigation_mapping::CellState state) {
    cells_.at(offset(index)) = state;
  }

  [[nodiscard]] bool isReady() const noexcept { return true; }
  [[nodiscard]] navigation_mapping::CellState cellState(
      navigation_mapping::WorldLayer,
      const navigation_mapping::GridIndex3& index) const {
    return cells_.at(offset(index));
  }
  [[nodiscard]] navigation_mapping::GridIndex3 worldToGrid(
      navigation_mapping::WorldLayer, const navigation_mapping::Vec3& position) const {
    return {static_cast<int>(std::floor(position.x() / resolution_)),
            static_cast<int>(std::floor(position.y() / resolution_)),
            static_cast<int>(std::floor(position.z() / resolution_))};
  }
  [[nodiscard]] navigation_mapping::Vec3 gridToWorld(
      navigation_mapping::WorldLayer, const navigation_mapping::GridIndex3& index) const {
    return {(index.x + 0.5) * resolution_, (index.y + 0.5) * resolution_,
            (index.z + 0.5) * resolution_};
  }
  [[nodiscard]] double resolution(navigation_mapping::WorldLayer) const { return resolution_; }
  [[nodiscard]] navigation_mapping::GridBounds bounds(navigation_mapping::WorldLayer) const {
    return bounds_;
  }
  [[nodiscard]] std::uint64_t generation() const noexcept { return generation_; }
  [[nodiscard]] std::uint64_t revision() const noexcept { return revision_; }
  [[nodiscard]] double clearanceRadius() const noexcept { return 0.5; }

  void advanceEpoch() {
    ++generation_;
    ++revision_;
  }

 private:
  [[nodiscard]] std::size_t offset(const navigation_mapping::GridIndex3& index) const {
    return static_cast<std::size_t>((index.x * size_y_ + index.y) * size_z_ + index.z);
  }

  navigation_mapping::GridBounds bounds_;
  int size_x_;
  int size_y_;
  int size_z_;
  double resolution_;
  std::vector<navigation_mapping::CellState> cells_;
  std::uint64_t generation_{1};
  std::uint64_t revision_{1};
};

VehicleState stateAt(const TestWorldModel& world, const navigation_mapping::GridIndex3& index) {
  VehicleState state;
  state.position = world.gridToWorld(navigation_mapping::WorldLayer::Inflated, index);
  return state;
}

Goal goalAt(const TestWorldModel& world, const navigation_mapping::GridIndex3& index) {
  Goal goal;
  goal.position = world.gridToWorld(navigation_mapping::WorldLayer::Inflated, index);
  return goal;
}

TEST(PlannerTest, FreeSpaceProducesFiniteDynamicTrajectory) {
  TestWorldModel world(12, 5, 1);
  world.fill(navigation_mapping::CellState::KnownFree);
  const auto result = Planner{}.planForTest(stateAt(world, {0, 2, 0}), goalAt(world, {11, 2, 0}), world);

  ASSERT_TRUE(result.success);
  EXPECT_EQ(result.failure_code, PlanFailureCode::None);
  EXPECT_TRUE(result.trajectory.finiteAndMonotonic());
  EXPECT_GT(result.trajectory.points.size(), 2U);
  EXPECT_EQ(result.trajectory.points.front().position, stateAt(world, {0, 2, 0}).position);
  EXPECT_EQ(result.trajectory.points.back().position, goalAt(world, {11, 2, 0}).position);
  EXPECT_LE(result.statistics.trajectory_optimization.maximum_velocity_mps, 2.0);
  EXPECT_LE(result.statistics.trajectory_optimization.maximum_acceleration_mps2, 3.0);
  EXPECT_LE(result.statistics.trajectory_optimization.simplified_path_node_count,
            result.statistics.trajectory_optimization.raw_path_node_count);
  EXPECT_GT(result.statistics.trajectory_optimization.raw_path_node_count, 0U);
  EXPECT_LE(result.statistics.trajectory_optimization.maximum_jerk_mps3, 6.0);
  EXPECT_LT(result.statistics.trajectory_optimization.c2_continuity_residual, 1e-6);
  EXPECT_TRUE(std::any_of(
      result.trajectory.points.begin(), result.trajectory.points.end(),
      [](const TrajectoryPoint& point) { return point.velocity.norm() > 1e-6; }));
}

TEST(PlannerTest, RecedingHorizonGoalCarriesContinuationVelocity) {
  TestWorldModel world(20, 5, 1);
  world.fill(navigation_mapping::CellState::KnownFree);
  auto state = stateAt(world, {1, 2, 0});
  state.velocity = navigation_mapping::Vec3{0.8, 0.0, 0.0};
  auto goal = goalAt(world, {12, 2, 0});
  goal.terminal = false;
  goal.terminal_velocity = navigation_mapping::Vec3{0.8, 0.0, 0.0};

  const auto result = Planner{}.planForTest(state, goal, world);

  ASSERT_TRUE(result.success);
  ASSERT_FALSE(result.trajectory.points.empty());
  EXPECT_GT(result.trajectory.points.back().velocity.norm(), 0.1);
  EXPECT_NEAR(result.trajectory.points.back().velocity.x(), 0.8, 1e-6);
  EXPECT_NEAR(result.trajectory.points.back().velocity.y(), 0.0, 1e-6);
  EXPECT_NEAR(result.trajectory.points.back().velocity.z(), 0.0, 1e-6);
}

TEST(PlannerTest, SolidWallAndUnknownSpaceFailClosedWithoutTrajectory) {
  TestWorldModel wall(9, 5, 1);
  wall.fill(navigation_mapping::CellState::KnownFree);
  for (int y = 0; y < 5; ++y) wall.set({4, y, 0}, navigation_mapping::CellState::Occupied);
  const auto wall_result = Planner{}.planForTest(stateAt(wall, {0, 2, 0}), goalAt(wall, {8, 2, 0}), wall);
  EXPECT_FALSE(wall_result.success);
  EXPECT_TRUE(wall_result.trajectory.points.empty());
  EXPECT_EQ(wall_result.failure_code, PlanFailureCode::NoPath);

  TestWorldModel unknown(5, 1, 1);
  unknown.set({0, 0, 0}, navigation_mapping::CellState::KnownFree);
  unknown.set({4, 0, 0}, navigation_mapping::CellState::KnownFree);
  const auto unknown_result =
      Planner{}.planForTest(stateAt(unknown, {0, 0, 0}), goalAt(unknown, {4, 0, 0}), unknown);
  EXPECT_FALSE(unknown_result.success);
  EXPECT_EQ(unknown_result.failure_code, PlanFailureCode::NoPath);
}

TEST(PlannerTest, NominalPlannerIsExplicitlyOptInForUnknownCorridor) {
  TestWorldModel world(5, 1, 1);
  world.set({0, 0, 0}, navigation_mapping::CellState::KnownFree);
  world.set({1, 0, 0}, navigation_mapping::CellState::Unknown);
  world.set({2, 0, 0}, navigation_mapping::CellState::Unknown);
  world.set({3, 0, 0}, navigation_mapping::CellState::KnownFree);
  world.set({4, 0, 0}, navigation_mapping::CellState::KnownFree);
  const auto state = stateAt(world, {0, 0, 0});
  const auto goal = goalAt(world, {4, 0, 0});

  const auto disabled = Planner{}.planNominalForTest(state, goal, world);
  EXPECT_FALSE(disabled.success);

  auto config = PlannerConfig{};
  config.allow_nominal_unknown = true;
  const auto nominal = Planner(config).planNominalForTest(state, goal, world);
  ASSERT_TRUE(nominal.success);
  EXPECT_EQ(nominal.role, PlanRole::Nominal);
  EXPECT_TRUE(nominal.trajectory.finiteAndMonotonic());
}

TEST(PlannerTest, ConservativeOpeningPolicyDistinguishesNarrowAndWideOpenings) {
  TestWorldModel narrow(9, 5, 1);
  narrow.fill(navigation_mapping::CellState::KnownFree);
  for (int y = 0; y < 5; ++y) narrow.set({4, y, 0}, navigation_mapping::CellState::Occupied);
  // The inflated map has already rejected this sub-clearance opening.
  narrow.set({4, 2, 0}, navigation_mapping::CellState::Occupied);
  const auto narrow_result =
      Planner{}.planForTest(stateAt(narrow, {0, 2, 0}), goalAt(narrow, {8, 2, 0}), narrow);
  EXPECT_FALSE(narrow_result.success);

  TestWorldModel wide(9, 7, 1);
  wide.fill(navigation_mapping::CellState::KnownFree);
  for (int y = 0; y < 7; ++y) wide.set({4, y, 0}, navigation_mapping::CellState::Occupied);
  wide.set({4, 2, 0}, navigation_mapping::CellState::KnownFree);
  wide.set({4, 3, 0}, navigation_mapping::CellState::KnownFree);
  wide.set({4, 4, 0}, navigation_mapping::CellState::KnownFree);
  const auto wide_result =
      Planner{}.planForTest(stateAt(wide, {0, 3, 0}), goalAt(wide, {8, 3, 0}), wide);
  EXPECT_TRUE(wide_result.success);
}

TEST(PlannerTest, MissingClearanceAndStaleProvenanceAreExplicit) {
  TestWorldModel world(4, 1, 1);
  world.fill(navigation_mapping::CellState::KnownFree);
  auto result = Planner{}.planForTest(stateAt(world, {0, 0, 0}), goalAt(world, {3, 0, 0}), world);
  ASSERT_TRUE(result.success);
  EXPECT_TRUE(result.isCurrent(world.generation(), world.revision()));
  world.advanceEpoch();
  EXPECT_FALSE(result.isCurrent(world.generation(), world.revision()));
}

TEST(PlannerTest, UnknownStartExceptionDoesNotOpenUnknownCorridor) {
  TestWorldModel world(6, 1, 1);
  world.fill(navigation_mapping::CellState::KnownFree);
  world.set({0, 0, 0}, navigation_mapping::CellState::Unknown);
  world.set({2, 0, 0}, navigation_mapping::CellState::Unknown);

  auto config = PlannerConfig{};
  config.allow_unknown_start = true;
  const auto result = Planner(config).planForTest(
      stateAt(world, {0, 0, 0}), goalAt(world, {5, 0, 0}), world);

  EXPECT_FALSE(result.success);
  EXPECT_EQ(result.failure_code, PlanFailureCode::NoPath);
}

TEST(PlannerTest, UnknownStartFootprintConnectsOnlyAdjacentSensorShadow) {
  TestWorldModel world(7, 1, 1);
  world.fill(navigation_mapping::CellState::KnownFree);
  world.set({1, 0, 0}, navigation_mapping::CellState::Unknown);
  world.set({2, 0, 0}, navigation_mapping::CellState::Unknown);

  PlannerConfig config;
  config.allow_unknown_start = true;
  config.unknown_start_radius_m = 0.10;
  const auto connected = Planner(config).planForTest(
      stateAt(world, {1, 0, 0}), goalAt(world, {6, 0, 0}), world);
  ASSERT_TRUE(connected.success);
  EXPECT_EQ(connected.trajectory.points.front().position, stateAt(world, {1, 0, 0}).position);

  // A second unknown voxel beyond the intersecting footprint is still a
  // hard barrier. This prevents the current-pose exception becoming an
  // optimistic unknown corridor.
  world.set({3, 0, 0}, navigation_mapping::CellState::Unknown);
  const auto blocked = Planner(config).planForTest(
      stateAt(world, {1, 0, 0}), goalAt(world, {6, 0, 0}), world);
  EXPECT_FALSE(blocked.success);
  EXPECT_EQ(blocked.failure_code, PlanFailureCode::NoPath);
}

TEST(PlannerTest, SafetyStopStaysKnownFreeAndEndsAtZeroVelocity) {
  TestWorldModel world(12, 5, 1);
  world.fill(navigation_mapping::CellState::KnownFree);
  VehicleState state = stateAt(world, {1, 2, 0});
  state.velocity = navigation_mapping::Vec3{1.5, 0.0, 0.0};
  state.acceleration = navigation_mapping::Vec3::Zero();

  PlannerConfig config;
  config.limits.max_velocity_mps = 2.0;
  config.limits.max_acceleration_mps2 = 2.0;
  config.limits.max_deceleration_mps2 = 2.0;
  config.trajectory_sample_dt_s = 0.05;
  const auto result = Planner(config).planSafetyStopForTest(state, world);

  ASSERT_TRUE(result.success);
  EXPECT_EQ(result.role, PlanRole::Safety);
  EXPECT_EQ(result.failure_code, PlanFailureCode::None);
  ASSERT_FALSE(result.trajectory.points.empty());
  EXPECT_NEAR(result.trajectory.points.back().velocity.norm(), 0.0, 1e-12);
  EXPECT_TRUE(result.trajectory.finiteAndMonotonic());
  EXPECT_EQ(result.statistics.corridor.blocked_sample_count, 0U);
}

TEST(PlannerTest, SafetyStopRecoversFromMeasuredOverspeed) {
  TestWorldModel world(20, 5, 1);
  world.fill(navigation_mapping::CellState::KnownFree);
  VehicleState state = stateAt(world, {1, 2, 0});
  state.velocity = navigation_mapping::Vec3{1.5, 0.0, 0.0};

  PlannerConfig config;
  config.limits.max_velocity_mps = 1.0;
  config.limits.max_acceleration_mps2 = 2.0;
  config.limits.max_deceleration_mps2 = 2.0;
  const auto stop = Planner(config).planSafetyStopForTest(state, world);
  ASSERT_TRUE(stop.success);
  TrajectoryVerificationConfig verification_config;
  verification_config.limits = config.limits;
  verification_config.sample_dt_s = config.trajectory_sample_dt_s;
  TrajectoryVerifier verifier(verification_config);
  const auto verification = verifier.verifyForTest(
      stop.trajectory, state, PlanRole::Safety, world);
  EXPECT_TRUE(verification.success);
  EXPECT_NEAR(stop.trajectory.points.front().velocity.norm(), 1.5, 1e-12);
  EXPECT_NEAR(stop.trajectory.points.back().velocity.norm(), 0.0, 1e-12);
}

TEST(PlannerTest, SafetyStopUsesConfiguredDeceleration) {
  TestWorldModel world(20, 5, 1);
  world.fill(navigation_mapping::CellState::KnownFree);
  VehicleState state = stateAt(world, {1, 2, 0});
  state.velocity = navigation_mapping::Vec3{2.0, 0.0, 0.0};

  PlannerConfig config;
  config.limits.max_velocity_mps = 3.0;
  config.limits.max_acceleration_mps2 = 4.0;
  config.limits.max_deceleration_mps2 = 1.0;
  const auto result = Planner(config).planSafetyStopForTest(state, world);

  ASSERT_TRUE(result.success);
  EXPECT_NEAR(result.trajectory.duration_s, 2.0, 1e-9);
  EXPECT_NEAR(result.trajectory.points.back().velocity.norm(), 0.0, 1e-12);
  EXPECT_DOUBLE_EQ(result.statistics.trajectory_optimization.maximum_acceleration_mps2, 1.0);
}

TEST(PlannerTest, SafetyStopTreatsSmallSettlingVelocityAsStationary) {
  TestWorldModel world(5, 1, 1);
  VehicleState state = stateAt(world, {0, 0, 0});
  state.velocity = navigation_mapping::Vec3{0.05, 0.0, 0.0};
  world.set({0, 0, 0}, navigation_mapping::CellState::Unknown);
  PlannerConfig config;
  config.allow_unknown_start = true;
  config.limits.max_deceleration_mps2 = 1.0;
  Planner planner(config);

  const auto result = planner.planSafetyStopForTest(state, world);

  ASSERT_TRUE(result.success);
  ASSERT_EQ(result.safety_kind, navigation_planning::SafetyPlanKind::BrakingStop);
  ASSERT_EQ(result.trajectory.points.size(), 1U);
  EXPECT_DOUBLE_EQ(result.trajectory.duration_s, 0.0);
  EXPECT_TRUE(result.trajectory.points.front().velocity.isZero(1e-12));
}

TEST(PlannerTest, SafetyStopAllowsOnlyConfiguredUnknownCurrentVoxel) {
  TestWorldModel world(12, 5, 1);
  world.fill(navigation_mapping::CellState::KnownFree);
  world.set({1, 2, 0}, navigation_mapping::CellState::Unknown);
  VehicleState state = stateAt(world, {1, 2, 0});
  state.velocity = navigation_mapping::Vec3{1.0, 0.0, 0.0};

  PlannerConfig config;
  config.allow_unknown_start = true;
  config.limits.max_velocity_mps = 2.0;
  config.limits.max_acceleration_mps2 = 2.0;
  config.limits.max_deceleration_mps2 = 2.0;
  const auto result = Planner(config).planSafetyStopForTest(state, world);

  ASSERT_TRUE(result.success);
  ASSERT_FALSE(result.trajectory.points.empty());
  EXPECT_EQ(result.trajectory.points.front().position, state.position);
  EXPECT_NEAR(result.trajectory.points.back().velocity.norm(), 0.0, 1e-12);
}

TEST(PlannerTest, SafetyStopRejectsUnknownBeyondCurrentVoxel) {
  TestWorldModel world(12, 5, 1);
  world.fill(navigation_mapping::CellState::KnownFree);
  world.set({1, 2, 0}, navigation_mapping::CellState::Unknown);
  world.set({2, 2, 0}, navigation_mapping::CellState::Unknown);
  VehicleState state = stateAt(world, {1, 2, 0});
  state.velocity = navigation_mapping::Vec3{1.0, 0.0, 0.0};

  PlannerConfig config;
  config.allow_unknown_start = true;
  config.limits.max_velocity_mps = 2.0;
  config.limits.max_acceleration_mps2 = 2.0;
  config.limits.max_deceleration_mps2 = 2.0;
  const auto result = Planner(config).planSafetyStopForTest(state, world);

  EXPECT_FALSE(result.success);
  EXPECT_EQ(result.failure_code, PlanFailureCode::SafetyStopUnavailable);
  EXPECT_TRUE(result.trajectory.points.empty());
}

TEST(PlannerTest, SafetyRouteMayUseOnlyCurrentUnknownVoxel) {
  TestWorldModel world(8, 3, 1);
  world.fill(navigation_mapping::CellState::KnownFree);
  world.set({1, 1, 0}, navigation_mapping::CellState::Unknown);
  const auto state = stateAt(world, {1, 1, 0});
  const auto goal = goalAt(world, {7, 1, 0});

  PlannerConfig config;
  config.allow_unknown_start = true;
  const auto result = Planner(config).planSafetyRouteForTest(state, goal, world);

  ASSERT_TRUE(result.success);
  EXPECT_EQ(result.safety_kind, SafetyPlanKind::Route);
  EXPECT_EQ(result.trajectory.points.front().position, state.position);
  EXPECT_EQ(result.trajectory.points.back().position, goal.position);
}

TEST(PlannerTest, SafetyStopRejectsStoppingDistanceEnteringOccupiedSpace) {
  TestWorldModel world(8, 3, 1);
  world.fill(navigation_mapping::CellState::KnownFree);
  world.set({2, 1, 0}, navigation_mapping::CellState::Occupied);

  VehicleState state = stateAt(world, {0, 1, 0});
  state.velocity = navigation_mapping::Vec3{2.0, 0.0, 0.0};
  state.acceleration = navigation_mapping::Vec3::Zero();
  PlannerConfig config;
  config.limits.max_velocity_mps = 2.0;
  config.limits.max_acceleration_mps2 = 2.0;
  config.limits.max_deceleration_mps2 = 2.0;

  const auto result = Planner(config).planSafetyStopForTest(state, world);
  EXPECT_FALSE(result.success);
  EXPECT_EQ(result.role, PlanRole::Safety);
  EXPECT_EQ(result.failure_code, PlanFailureCode::SafetyStopUnavailable);
  EXPECT_TRUE(result.trajectory.points.empty());
}

TEST(PlannerTest, SafetyStopRejectsUnmodeledCurrentAcceleration) {
  TestWorldModel world(12, 5, 1);
  world.fill(navigation_mapping::CellState::KnownFree);
  VehicleState state = stateAt(world, {1, 2, 0});
  state.velocity = navigation_mapping::Vec3{1.0, 0.0, 0.0};
  state.acceleration = navigation_mapping::Vec3{0.1, 0.0, 0.0};

  const auto result = Planner{}.planSafetyStopForTest(state, world);
  EXPECT_FALSE(result.success);
  EXPECT_EQ(result.role, PlanRole::Safety);
  EXPECT_EQ(result.failure_code, PlanFailureCode::SafetyStopUnavailable);
}

TEST(PlannerTest, SafetyRouteReusesSmoothKnownFreePlanner) {
  TestWorldModel world(8, 3, 1);
  world.fill(navigation_mapping::CellState::KnownFree);
  const auto state = stateAt(world, {0, 1, 0});
  const auto goal = goalAt(world, {7, 1, 0});

  const auto result = Planner{}.planSafetyRouteForTest(state, goal, world);
  ASSERT_TRUE(result.success);
  EXPECT_EQ(result.role, PlanRole::Safety);
  EXPECT_EQ(result.safety_kind, SafetyPlanKind::Route);
  EXPECT_TRUE(result.trajectory.finiteAndMonotonic());
  EXPECT_EQ(result.trajectory.points.back().position, goal.position);
  EXPECT_NEAR(result.trajectory.points.back().velocity.norm(), 0.0, 1e-12);
}

TEST(PlannerTest, SafetyRouteRejectsUnknownAndBrakingStopRemainsSeparate) {
  TestWorldModel world(5, 1, 1);
  world.set({0, 0, 0}, navigation_mapping::CellState::KnownFree);
  world.set({1, 0, 0}, navigation_mapping::CellState::Unknown);
  world.set({2, 0, 0}, navigation_mapping::CellState::KnownFree);
  world.set({3, 0, 0}, navigation_mapping::CellState::KnownFree);
  world.set({4, 0, 0}, navigation_mapping::CellState::KnownFree);
  const auto state = stateAt(world, {0, 0, 0});
  const auto goal = goalAt(world, {4, 0, 0});

  const auto route = Planner{}.planSafetyRouteForTest(state, goal, world);
  EXPECT_FALSE(route.success);
  EXPECT_EQ(route.safety_kind, SafetyPlanKind::Route);

  const auto stop = Planner{}.planSafetyStopForTest(state, world);
  ASSERT_TRUE(stop.success);
  EXPECT_EQ(stop.safety_kind, SafetyPlanKind::BrakingStop);
}

TimeParameterizedTrajectory trajectoryThrough(
    const TestWorldModel& world, const std::vector<navigation_mapping::GridIndex3>& indices) {
  TimeParameterizedTrajectory trajectory;
  trajectory.duration_s = static_cast<double>(indices.size() - 1);
  for (std::size_t i = 0; i < indices.size(); ++i) {
    trajectory.points.push_back(TrajectoryPoint{
        static_cast<double>(i), world.gridToWorld(navigation_mapping::WorldLayer::Inflated,
                                                  indices[i]),
        navigation_mapping::Vec3::Zero(), navigation_mapping::Vec3::Zero()});
  }
  return trajectory;
}

TEST(PlannerTest, NominalUnknownIsBoundedByCommitmentHorizon) {
  TestWorldModel world(5, 1, 1);
  world.set({0, 0, 0}, navigation_mapping::CellState::KnownFree);
  world.set({1, 0, 0}, navigation_mapping::CellState::Unknown);
  world.set({2, 0, 0}, navigation_mapping::CellState::KnownFree);
  world.set({3, 0, 0}, navigation_mapping::CellState::KnownFree);
  world.set({4, 0, 0}, navigation_mapping::CellState::KnownFree);
  const auto state = stateAt(world, {0, 0, 0});
  const auto trajectory = trajectoryThrough(world, {{0, 0, 0}, {1, 0, 0}, {2, 0, 0}, {3, 0, 0}});

  TrajectoryVerificationConfig config;
  config.allow_nominal_unknown = true;
  config.commitment_horizon_s = 1.5;
  config.sample_dt_s = 0.1;
  const auto result = TrajectoryVerifier(config).verifyForTest(
      trajectory, state, PlanRole::Nominal, world);
  EXPECT_TRUE(result.success);
  EXPECT_GT(result.statistics.unknown_sample_count, 0U);

  config.commitment_horizon_s = 0.5;
  const auto bounded_result = TrajectoryVerifier(config).verifyForTest(
      trajectory, state, PlanRole::Nominal, world);
  EXPECT_FALSE(bounded_result.success);
  EXPECT_EQ(bounded_result.failure_code, VerificationFailureCode::UnknownBeyondCommitment);
}

TEST(PlannerTest, TrajectoryVerifierDoesNotDuplicateRegularAndEmittedSamples) {
  TestWorldModel world(4, 1, 1);
  world.fill(navigation_mapping::CellState::KnownFree);
  const auto state = stateAt(world, {0, 0, 0});
  const auto trajectory = trajectoryThrough(world, {{0, 0, 0}, {1, 0, 0}, {2, 0, 0}, {3, 0, 0}});

  TrajectoryVerificationConfig config;
  config.sample_dt_s = 0.1;
  const auto result = TrajectoryVerifier(config).verifyForTest(
      trajectory, state, PlanRole::Safety, world);

  ASSERT_TRUE(result.success);
  // Three one-second segments: each has nine interior samples plus its
  // endpoint, with the initial point counted once.
  EXPECT_EQ(result.statistics.sampled_point_count, 31U);
}

TEST(PlannerTest, TrajectoryVerifierAllowsHigherDirectionalBrakingLimit) {
  TestWorldModel world(4, 1, 1);
  world.fill(navigation_mapping::CellState::KnownFree);
  const auto state = stateAt(world, {0, 0, 0});
  auto trajectory = trajectoryThrough(world, {{0, 0, 0}, {1, 0, 0}});
  trajectory.points.front().velocity = navigation_mapping::Vec3{1.0, 0.0, 0.0};
  trajectory.points.front().acceleration = navigation_mapping::Vec3{-0.9, 0.0, 0.0};
  trajectory.points.back().velocity = navigation_mapping::Vec3{0.1, 0.0, 0.0};

  TrajectoryVerificationConfig config;
  config.limits.max_acceleration_mps2 = 0.6;
  config.limits.max_deceleration_mps2 = 0.9;

  const auto result = TrajectoryVerifier(config).verifyForTest(
      trajectory, state, PlanRole::Safety, world);

  EXPECT_TRUE(result.success);
  EXPECT_NEAR(result.statistics.maximum_deceleration_mps2, 0.9, 1e-9);
}

TEST(PlannerTest, TrajectoryVerifierUsesHardVelocityBoundaryWithoutDesignMargin) {
  TestWorldModel world(4, 1, 1);
  world.fill(navigation_mapping::CellState::KnownFree);
  const auto state = stateAt(world, {0, 0, 0});
  auto makeTrajectory = [&](double speed) {
    auto trajectory = trajectoryThrough(world, {{0, 0, 0}, {1, 0, 0}});
    trajectory.points.front().velocity = navigation_mapping::Vec3{speed, 0.0, 0.0};
    trajectory.points.back().velocity = navigation_mapping::Vec3{speed, 0.0, 0.0};
    return trajectory;
  };

  TrajectoryVerificationConfig config;
  config.limits.max_velocity_mps = 1.0;
  config.sample_dt_s = 0.1;
  const auto below = TrajectoryVerifier(config).verifyForTest(
      makeTrajectory(1.0 - 1e-6), state, PlanRole::Committed, world);
  const auto exactly = TrajectoryVerifier(config).verifyForTest(
      makeTrajectory(1.0), state, PlanRole::Committed, world);
  const auto above = TrajectoryVerifier(config).verifyForTest(
      makeTrajectory(1.0 + 1e-6), state, PlanRole::Committed, world);
  EXPECT_TRUE(below.success);
  EXPECT_TRUE(exactly.success);
  EXPECT_FALSE(above.success);
  EXPECT_EQ(above.failure_code, VerificationFailureCode::DynamicLimitsExceeded);
}

TEST(PlannerTest, UnknownStartExceptionIsLimitedToCurrentStartSample) {
  TestWorldModel world(3, 1, 1);
  world.set({0, 0, 0}, navigation_mapping::CellState::Unknown);
  world.set({1, 0, 0}, navigation_mapping::CellState::KnownFree);
  world.set({2, 0, 0}, navigation_mapping::CellState::KnownFree);
  const auto state = stateAt(world, {0, 0, 0});
  const auto trajectory = trajectoryThrough(world, {{0, 0, 0}, {1, 0, 0}, {2, 0, 0}});

  TrajectoryVerificationConfig config;
  config.allow_unknown_start = true;
  const auto start_result = TrajectoryVerifier(config).verifyForTest(
      trajectory, state, PlanRole::Committed, world);
  EXPECT_TRUE(start_result.success);

  world.set({1, 0, 0}, navigation_mapping::CellState::Unknown);
  const auto later_result = TrajectoryVerifier(config).verifyForTest(
      trajectory, state, PlanRole::Committed, world);
  EXPECT_FALSE(later_result.success);
  EXPECT_EQ(later_result.failure_code, VerificationFailureCode::SafetyUnknownSample);
}

TEST(PlannerTest, TrajectoryVerifierUsesSameUnknownStartFootprint) {
  TestWorldModel world(4, 1, 1);
  world.fill(navigation_mapping::CellState::KnownFree);
  world.set({0, 0, 0}, navigation_mapping::CellState::Unknown);
  world.set({1, 0, 0}, navigation_mapping::CellState::Unknown);
  const auto state = stateAt(world, {0, 0, 0});
  const auto trajectory = trajectoryThrough(
      world, {{0, 0, 0}, {1, 0, 0}, {2, 0, 0}, {3, 0, 0}});

  TrajectoryVerificationConfig config;
  config.allow_unknown_start = true;
  config.unknown_start_radius_m = 0.10;
  const auto connected = TrajectoryVerifier(config).verifyForTest(
      trajectory, state, PlanRole::Safety, world);
  EXPECT_TRUE(connected.success);

  world.set({2, 0, 0}, navigation_mapping::CellState::Unknown);
  const auto blocked = TrajectoryVerifier(config).verifyForTest(
      trajectory, state, PlanRole::Safety, world);
  EXPECT_FALSE(blocked.success);
  EXPECT_EQ(blocked.failure_code, VerificationFailureCode::SafetyUnknownSample);
}

TEST(PlannerTest, DualVerificationRequiresSafetyAndFallsBackFromNominal) {
  TestWorldModel world(4, 1, 1);
  world.set({0, 0, 0}, navigation_mapping::CellState::KnownFree);
  world.set({1, 0, 0}, navigation_mapping::CellState::Occupied);
  world.set({2, 0, 0}, navigation_mapping::CellState::Unknown);
  world.set({3, 0, 0}, navigation_mapping::CellState::KnownFree);
  const auto state = stateAt(world, {0, 0, 0});
  const auto nominal = trajectoryThrough(world, {{0, 0, 0}, {1, 0, 0}});
  const auto safety = trajectoryThrough(world, {{0, 0, 0}});

  TrajectoryVerificationConfig config;
  config.allow_nominal_unknown = true;
  const auto result = TrajectoryVerifier(config).verifyDualForTest(
      nominal, safety, state, world);
  ASSERT_TRUE(result.success);
  EXPECT_FALSE(result.nominal_selected);
  EXPECT_EQ(result.selected_role, PlanRole::Safety);
  EXPECT_EQ(result.nominal.failure_code, VerificationFailureCode::OccupiedSample);
  EXPECT_TRUE(result.safety.success);
}

TEST(PlannerTest, SafetyVerificationRejectsUnknownEvenWhenNominalAllowsIt) {
  TestWorldModel world(3, 1, 1);
  world.set({0, 0, 0}, navigation_mapping::CellState::KnownFree);
  world.set({1, 0, 0}, navigation_mapping::CellState::Unknown);
  world.set({2, 0, 0}, navigation_mapping::CellState::KnownFree);
  const auto state = stateAt(world, {0, 0, 0});
  const auto trajectory = trajectoryThrough(world, {{0, 0, 0}, {1, 0, 0}});

  TrajectoryVerificationConfig config;
  config.allow_nominal_unknown = true;
  config.commitment_horizon_s = 1.0;
  const auto nominal = TrajectoryVerifier(config).verifyForTest(
      trajectory, state, PlanRole::Nominal, world);
  const auto safety = TrajectoryVerifier(config).verifyForTest(
      trajectory, state, PlanRole::Safety, world);
  EXPECT_TRUE(nominal.success);
  EXPECT_FALSE(safety.success);
  EXPECT_EQ(safety.failure_code, VerificationFailureCode::SafetyUnknownSample);
}

TEST(PlannerTest, DualVerificationFailsClosedWithoutSafetyCandidate) {
  TestWorldModel world(3, 1, 1);
  world.set({0, 0, 0}, navigation_mapping::CellState::KnownFree);
  world.set({1, 0, 0}, navigation_mapping::CellState::Unknown);
  world.set({2, 0, 0}, navigation_mapping::CellState::KnownFree);
  const auto state = stateAt(world, {0, 0, 0});
  const auto nominal = trajectoryThrough(world, {{0, 0, 0}, {1, 0, 0}});
  const auto safety = trajectoryThrough(world, {{0, 0, 0}, {1, 0, 0}});

  TrajectoryVerificationConfig config;
  config.allow_nominal_unknown = true;
  config.commitment_horizon_s = 1.0;
  const auto result = TrajectoryVerifier(config).verifyDualForTest(
      nominal, safety, state, world);

  EXPECT_FALSE(result.success);
  EXPECT_FALSE(result.nominal_selected);
  EXPECT_EQ(result.failure_code, VerificationFailureCode::SafetyUnknownSample);
  EXPECT_FALSE(result.safety.success);
}

}  // namespace
}  // namespace navigation_planning
