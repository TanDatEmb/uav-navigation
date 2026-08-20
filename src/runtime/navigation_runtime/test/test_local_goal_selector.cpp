#include <cmath>

#include <gtest/gtest.h>

#include "navigation_runtime/local_goal_selector.hpp"

namespace navigation_runtime {
namespace {

class BoxWorld {
 public:
  bool isReady() const noexcept { return ready; }

  navigation_mapping::GridBounds bounds(navigation_mapping::WorldLayer) const {
    return {{0, 0, 0}, {10, 10, 10}};
  }

  navigation_mapping::Vec3 gridToWorld(navigation_mapping::WorldLayer,
                                       const navigation_mapping::GridIndex3& index) const {
    return navigation_mapping::Vec3{static_cast<double>(index.x) - 5.0,
                                    static_cast<double>(index.y) - 5.0,
                                    static_cast<double>(index.z) - 5.0};
  }

  navigation_mapping::GridIndex3 worldToGrid(navigation_mapping::WorldLayer,
                                             const navigation_mapping::Vec3& point) const {
    return {static_cast<int>(std::floor(point.x() + 5.0)),
            static_cast<int>(std::floor(point.y() + 5.0)),
            static_cast<int>(std::floor(point.z() + 5.0))};
  }

  double resolution(navigation_mapping::WorldLayer) const { return 1.0; }

  navigation_mapping::CellState cellState(navigation_mapping::WorldLayer,
                                          const navigation_mapping::GridIndex3& index) const {
    if (occupied && index == navigation_mapping::GridIndex3{7, 5, 5}) {
      return navigation_mapping::CellState::Occupied;
    }
    if (unknown && index == navigation_mapping::GridIndex3{7, 5, 5}) {
      return navigation_mapping::CellState::Unknown;
    }
    if (thin_free && (index.y != 5 || index.z != 5)) {
      return navigation_mapping::CellState::Unknown;
    }
    return all_unknown ? navigation_mapping::CellState::Unknown
                       : navigation_mapping::CellState::KnownFree;
  }

  bool ready{true};
  bool occupied{false};
  bool unknown{false};
  bool all_unknown{false};
  bool thin_free{false};
};

TEST(LocalGoalSelectorTest, KeepsGoalInsideCurrentWindowDirect) {
  const BoxWorld world;
  const auto result = selectLocalGoal(world, navigation_mapping::Vec3{0.0, 0.0, 0.0},
                                      navigation_mapping::Vec3{2.0, 1.0, 0.0}, 1.0);
  EXPECT_TRUE(result.success());
  EXPECT_FALSE(result.usesSubGoal());
  EXPECT_EQ(result.status, LocalGoalSelectionStatus::Direct);
  EXPECT_EQ(result.goal, (navigation_mapping::Vec3{2.0, 1.0, 0.0}));
}

TEST(LocalGoalSelectorTest, ProjectsFarGoalToInsetWindow) {
  const BoxWorld world;
  const auto result = selectLocalGoal(world, navigation_mapping::Vec3{0.0, 0.0, 0.0},
                                      navigation_mapping::Vec3{20.0, 0.0, 0.0}, 1.0);
  ASSERT_TRUE(result.success());
  EXPECT_TRUE(result.usesSubGoal());
  EXPECT_EQ(result.status, LocalGoalSelectionStatus::LocalSubGoal);
  EXPECT_NEAR(result.goal.x(), 4.0, 1e-9);
  EXPECT_NEAR(result.goal.y(), 0.0, 1e-9);
  EXPECT_NEAR(result.goal.z(), 0.0, 1e-9);
}

TEST(LocalGoalSelectorTest, UsesThinKnownFreeRayForFullHorizon) {
  BoxWorld world;
  world.thin_free = true;
  const auto result = selectPlanningHorizon(
      world, navigation_mapping::Vec3{0.0, 0.0, 0.0},
      navigation_mapping::Vec3{20.0, 0.0, 0.0}, 1.0, 5.0, std::nullopt,
      navigation_mapping::Vec3::UnitX());
  ASSERT_TRUE(result.success());
  EXPECT_TRUE(result.usesSubGoal());
  // The selector must keep moving along a thin observed ray; it must not
  // fail merely because adjacent horizontal voxels are still Unknown.
  EXPECT_GT(result.goal.x(), 1.5);
  EXPECT_GT(result.forward_projection_m, 1.5);
}

TEST(LocalGoalSelectorTest, UsesKnownFreePreferredHorizonAsSoftContinuityPrior) {
  const BoxWorld world;
  const auto result = selectLocalGoal(
      world, navigation_mapping::Vec3{0.0, 0.0, 0.0},
      navigation_mapping::Vec3{20.0, 0.0, 0.0}, 1.0, 15.0,
      navigation_mapping::Vec3{4.0, 1.0, 0.0});
  ASSERT_TRUE(result.success());
  EXPECT_TRUE(result.usesSubGoal());
  EXPECT_EQ(result.goal, (navigation_mapping::Vec3{4.0, 1.0, 0.0}));
}

TEST(LocalGoalSelectorTest, RollingHorizonFollowsIncomingTangentNotMissionRay) {
  const BoxWorld world;
  const auto result = selectPlanningHorizon(
      world, navigation_mapping::Vec3{0.0, 0.0, 0.0},
      navigation_mapping::Vec3{0.0, 20.0, 0.0}, 1.0, 5.0,
      navigation_mapping::Vec3{4.0, 1.0, 0.0},
      navigation_mapping::Vec3{1.0, 0.0, 0.0});
  ASSERT_TRUE(result.success());
  EXPECT_TRUE(result.usesSubGoal());
  EXPECT_GT(result.forward_projection_m, 0.0);
  EXPECT_GT(result.tangent.dot(navigation_mapping::Vec3::UnitX()), 0.8);
  // The rolling API treats the old anchor as a soft continuity cost; it does
  // not turn it into a completion endpoint on every planning tick.
  EXPECT_GT((result.goal - navigation_mapping::Vec3{4.0, 1.0, 0.0}).norm(), 0.05);
}

TEST(LocalGoalSelectorTest, RollingHorizonCarriesForwardDirectionAroundObstacle) {
  BoxWorld world;
  world.occupied = true;
  const auto result = selectPlanningHorizon(
      world, navigation_mapping::Vec3{0.0, 0.0, 0.0},
      navigation_mapping::Vec3{0.0, 20.0, 0.0}, 1.0, 5.0, std::nullopt,
      navigation_mapping::Vec3{1.0, 0.0, 0.0});
  ASSERT_TRUE(result.success());
  EXPECT_TRUE(result.usesSubGoal());
  EXPECT_GT(result.forward_projection_m, 0.0);
  EXPECT_GT(std::abs(result.goal.y()), 0.5);
  EXPECT_GT(result.tangent.dot(navigation_mapping::Vec3::UnitX()), 0.0);
}

TEST(LocalGoalSelectorTest, DoesNotAcceptUnknownMissionGoal) {
  BoxWorld world;
  world.unknown = true;
  const auto result = selectLocalGoal(world, navigation_mapping::Vec3{0.0, 0.0, 0.0},
                                      navigation_mapping::Vec3{2.0, 0.0, 0.0}, 1.0);
  ASSERT_TRUE(result.success());
  EXPECT_TRUE(result.usesSubGoal());
  EXPECT_NE(result.goal, (navigation_mapping::Vec3{2.0, 0.0, 0.0}));
  EXPECT_EQ(world.cellState(navigation_mapping::WorldLayer::Inflated,
                            world.worldToGrid(navigation_mapping::WorldLayer::Inflated,
                                              result.goal)),
            navigation_mapping::CellState::KnownFree);
}

TEST(LocalGoalSelectorTest, SelectsLateralDetourWhenRayIsOccupied) {
  BoxWorld world;
  world.occupied = true;
  const auto result = selectLocalGoal(world, navigation_mapping::Vec3{0.0, 0.0, 0.0},
                                      navigation_mapping::Vec3{20.0, 0.0, 0.0}, 1.0);
  ASSERT_TRUE(result.success());
  ASSERT_TRUE(result.usesSubGoal());
  // The direct x-axis ray contains an occupied voxel.  A usable sub-goal must
  // therefore carry lateral displacement so the next A* horizon can route
  // around the obstacle instead of repeatedly driving into it.
  EXPECT_GT(std::abs(result.goal.y()), 0.5);
  EXPECT_LE(result.goal.norm(), 5.0 + 1e-9);
  EXPECT_EQ(world.cellState(navigation_mapping::WorldLayer::Inflated,
                            world.worldToGrid(navigation_mapping::WorldLayer::Inflated,
                                              result.goal)),
            navigation_mapping::CellState::KnownFree);
}

TEST(LocalGoalSelectorTest, FailsClosedWithoutKnownFreeLocalGoal) {
  BoxWorld world;
  world.all_unknown = true;
  const auto result = selectLocalGoal(world, navigation_mapping::Vec3{0.0, 0.0, 0.0},
                                      navigation_mapping::Vec3{2.0, 0.0, 0.0}, 1.0);
  EXPECT_FALSE(result.success());
  EXPECT_EQ(result.status, LocalGoalSelectionStatus::NoUsableSubGoal);
}

TEST(LocalGoalSelectorTest, RejectsStartOutsideWindow) {
  const BoxWorld world;
  const auto result = selectLocalGoal(world, navigation_mapping::Vec3{6.0, 0.0, 0.0},
                                      navigation_mapping::Vec3{7.0, 0.0, 0.0}, 1.0);
  EXPECT_FALSE(result.success());
  EXPECT_EQ(result.status, LocalGoalSelectionStatus::StartOutsideBounds);
}

TEST(LocalGoalSelectorTest, RejectsMarginThatConsumesWindow) {
  const BoxWorld world;
  const auto result = selectLocalGoal(world, navigation_mapping::Vec3{0.0, 0.0, 0.0},
                                      navigation_mapping::Vec3{20.0, 0.0, 0.0}, 6.0);
  EXPECT_FALSE(result.success());
  EXPECT_EQ(result.status, LocalGoalSelectionStatus::NoUsableSubGoal);
}

}  // namespace
}  // namespace navigation_runtime
