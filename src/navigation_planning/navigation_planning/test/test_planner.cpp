#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <vector>

#include "navigation_planning/planner.hpp"

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

}  // namespace
}  // namespace navigation_planning
