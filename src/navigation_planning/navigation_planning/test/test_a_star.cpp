#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <vector>

#include "navigation_planning/a_star.hpp"

namespace navigation_planning {
namespace {

class TestGridModel {
 public:
  TestGridModel(int size_x, int size_y, int size_z, double resolution = 1.0)
      : bounds_{{0, 0, 0}, {size_x - 1, size_y - 1, size_z - 1}},
        size_x_(size_x),
        size_y_(size_y),
        size_z_(size_z),
        resolution_(resolution),
        probability_(static_cast<std::size_t>(size_x * size_y * size_z),
                     navigation_mapping::CellState::Unknown),
        inflated_(probability_) {}

  void fill(navigation_mapping::WorldLayer layer, navigation_mapping::CellState state) {
    auto& cells = layer == navigation_mapping::WorldLayer::Probability ? probability_ : inflated_;
    std::fill(cells.begin(), cells.end(), state);
  }

  void set(navigation_mapping::WorldLayer layer, const navigation_mapping::GridIndex3& index,
           navigation_mapping::CellState state) {
    cells(layer).at(offset(index)) = state;
  }

  [[nodiscard]] navigation_mapping::CellState cellState(
      navigation_mapping::WorldLayer layer,
      const navigation_mapping::GridIndex3& index) const {
    return cells(layer).at(offset(index));
  }

  [[nodiscard]] navigation_mapping::GridIndex3 worldToGrid(
      navigation_mapping::WorldLayer, const navigation_mapping::Vec3& position) const {
    return navigation_mapping::GridIndex3{
        static_cast<int>(std::floor(position.x() / resolution_)),
        static_cast<int>(std::floor(position.y() / resolution_)),
        static_cast<int>(std::floor(position.z() / resolution_))};
  }

  [[nodiscard]] navigation_mapping::Vec3 gridToWorld(
      navigation_mapping::WorldLayer, const navigation_mapping::GridIndex3& index) const {
    return navigation_mapping::Vec3{(index.x + 0.5) * resolution_,
                                    (index.y + 0.5) * resolution_,
                                    (index.z + 0.5) * resolution_};
  }

  [[nodiscard]] double resolution(navigation_mapping::WorldLayer) const { return resolution_; }
  [[nodiscard]] navigation_mapping::GridBounds bounds(navigation_mapping::WorldLayer) const {
    return bounds_;
  }
  [[nodiscard]] std::uint64_t generation() const noexcept { return 1; }

 private:
  [[nodiscard]] std::size_t offset(const navigation_mapping::GridIndex3& index) const {
    return static_cast<std::size_t>((index.x * size_y_ + index.y) * size_z_ + index.z);
  }

  [[nodiscard]] const std::vector<navigation_mapping::CellState>& cells(
      navigation_mapping::WorldLayer layer) const {
    return layer == navigation_mapping::WorldLayer::Probability ? probability_ : inflated_;
  }
  [[nodiscard]] std::vector<navigation_mapping::CellState>& cells(
      navigation_mapping::WorldLayer layer) {
    return layer == navigation_mapping::WorldLayer::Probability ? probability_ : inflated_;
  }

  navigation_mapping::GridBounds bounds_;
  int size_x_;
  int size_y_;
  int size_z_;
  double resolution_;
  std::vector<navigation_mapping::CellState> probability_;
  std::vector<navigation_mapping::CellState> inflated_;
};

SearchRequest request(const TestGridModel& model, navigation_mapping::WorldLayer layer,
                      UnknownPolicy policy, const navigation_mapping::GridIndex3& start,
                      const navigation_mapping::GridIndex3& goal) {
  return SearchRequest{layer, policy, model.gridToWorld(layer, start),
                       model.gridToWorld(layer, goal)};
}

TEST(AStarTest, EmptyFreeVolumeHasShortestPath) {
  TestGridModel model(5, 1, 1);
  model.fill(navigation_mapping::WorldLayer::Probability, navigation_mapping::CellState::KnownFree);
  const auto result = AStar{}.searchForTest(
      model, request(model, navigation_mapping::WorldLayer::Probability,
                     UnknownPolicy::TreatUnknownAsBlocked, {0, 0, 0}, {4, 0, 0}));
  ASSERT_TRUE(result.success);
  EXPECT_EQ(result.path.size(), 5U);
  EXPECT_DOUBLE_EQ(result.statistics.path_length_m, 4.0);
  EXPECT_EQ(result.statistics.path_node_count, 5U);
}

TEST(AStarTest, SolidWallHasNoPath) {
  TestGridModel model(5, 3, 1);
  model.fill(navigation_mapping::WorldLayer::Probability, navigation_mapping::CellState::KnownFree);
  for (int y = 0; y < 3; ++y) model.set(navigation_mapping::WorldLayer::Probability, {2, y, 0}, navigation_mapping::CellState::Occupied);
  const auto result = AStar{}.searchForTest(
      model, request(model, navigation_mapping::WorldLayer::Probability,
                     UnknownPolicy::TreatUnknownAsBlocked, {0, 1, 0}, {4, 1, 0}));
  EXPECT_FALSE(result.success);
  EXPECT_EQ(result.failure, SearchFailureCode::NoPath);
}

TEST(AStarTest, WallGapIsUsed) {
  TestGridModel model(5, 3, 1);
  model.fill(navigation_mapping::WorldLayer::Probability, navigation_mapping::CellState::KnownFree);
  model.set(navigation_mapping::WorldLayer::Probability, {2, 0, 0}, navigation_mapping::CellState::Occupied);
  model.set(navigation_mapping::WorldLayer::Probability, {2, 2, 0}, navigation_mapping::CellState::Occupied);
  const auto result = AStar{}.searchForTest(
      model, request(model, navigation_mapping::WorldLayer::Probability,
                     UnknownPolicy::TreatUnknownAsBlocked, {0, 1, 0}, {4, 1, 0}));
  ASSERT_TRUE(result.success);
  EXPECT_NE(std::find(result.path.begin(), result.path.end(), navigation_mapping::GridIndex3{2, 1, 0}),
            result.path.end());
}

TEST(AStarTest, UnknownPolicyControlsTraversal) {
  TestGridModel model(3, 1, 1);
  model.set(navigation_mapping::WorldLayer::Probability, {0, 0, 0}, navigation_mapping::CellState::KnownFree);
  model.set(navigation_mapping::WorldLayer::Probability, {2, 0, 0}, navigation_mapping::CellState::KnownFree);
  const auto blocked = AStar{}.searchForTest(
      model, request(model, navigation_mapping::WorldLayer::Probability,
                     UnknownPolicy::TreatUnknownAsBlocked, {0, 0, 0}, {2, 0, 0}));
  const auto traversable = AStar{}.searchForTest(
      model, request(model, navigation_mapping::WorldLayer::Probability,
                     UnknownPolicy::TreatUnknownAsTraversable, {0, 0, 0}, {2, 0, 0}));
  EXPECT_FALSE(blocked.success);
  EXPECT_TRUE(traversable.success);
}

TEST(AStarTest, InflatedLayerCanBlockAnOpenProbabilityGap) {
  TestGridModel model(3, 1, 1);
  model.fill(navigation_mapping::WorldLayer::Probability, navigation_mapping::CellState::KnownFree);
  model.fill(navigation_mapping::WorldLayer::Inflated, navigation_mapping::CellState::KnownFree);
  model.set(navigation_mapping::WorldLayer::Inflated, {1, 0, 0}, navigation_mapping::CellState::Occupied);
  const auto result = AStar{}.searchForTest(
      model, request(model, navigation_mapping::WorldLayer::Inflated,
                     UnknownPolicy::TreatUnknownAsBlocked, {0, 0, 0}, {2, 0, 0}));
  EXPECT_FALSE(result.success);
  EXPECT_EQ(result.failure, SearchFailureCode::NoPath);
}

TEST(AStarTest, OutsideBoundsFailsDeterministically) {
  TestGridModel model(3, 1, 1);
  model.fill(navigation_mapping::WorldLayer::Probability, navigation_mapping::CellState::KnownFree);
  const auto result = AStar{}.searchForTest(
      model, request(model, navigation_mapping::WorldLayer::Probability,
                     UnknownPolicy::TreatUnknownAsBlocked, {-1, 0, 0}, {2, 0, 0}));
  EXPECT_FALSE(result.success);
  EXPECT_EQ(result.failure, SearchFailureCode::StartOutsideBounds);
}

TEST(AStarTest, OccupiedEndpointsFailDeterministically) {
  TestGridModel model(3, 1, 1);
  model.fill(navigation_mapping::WorldLayer::Probability, navigation_mapping::CellState::KnownFree);
  model.set(navigation_mapping::WorldLayer::Probability, {0, 0, 0}, navigation_mapping::CellState::Occupied);
  const auto start_result = AStar{}.searchForTest(
      model, request(model, navigation_mapping::WorldLayer::Probability,
                     UnknownPolicy::TreatUnknownAsBlocked, {0, 0, 0}, {2, 0, 0}));
  model.set(navigation_mapping::WorldLayer::Probability, {0, 0, 0}, navigation_mapping::CellState::KnownFree);
  model.set(navigation_mapping::WorldLayer::Probability, {2, 0, 0}, navigation_mapping::CellState::Occupied);
  const auto goal_result = AStar{}.searchForTest(
      model, request(model, navigation_mapping::WorldLayer::Probability,
                     UnknownPolicy::TreatUnknownAsBlocked, {0, 0, 0}, {2, 0, 0}));
  EXPECT_EQ(start_result.failure, SearchFailureCode::StartOccupied);
  EXPECT_EQ(goal_result.failure, SearchFailureCode::GoalOccupied);
}

TEST(AStarTest, RepeatedSearchIsDeterministic) {
  TestGridModel model(5, 3, 1);
  model.fill(navigation_mapping::WorldLayer::Probability, navigation_mapping::CellState::KnownFree);
  model.set(navigation_mapping::WorldLayer::Probability, {2, 0, 0}, navigation_mapping::CellState::Occupied);
  model.set(navigation_mapping::WorldLayer::Probability, {2, 1, 0}, navigation_mapping::CellState::Occupied);
  const auto first = AStar{}.searchForTest(
      model, request(model, navigation_mapping::WorldLayer::Probability,
                     UnknownPolicy::TreatUnknownAsBlocked, {0, 1, 0}, {4, 1, 0}));
  const auto second = AStar{}.searchForTest(
      model, request(model, navigation_mapping::WorldLayer::Probability,
                     UnknownPolicy::TreatUnknownAsBlocked, {0, 1, 0}, {4, 1, 0}));
  EXPECT_EQ(first.success, second.success);
  EXPECT_EQ(first.path, second.path);
  EXPECT_EQ(first.statistics.expanded_nodes, second.statistics.expanded_nodes);
  EXPECT_EQ(first.statistics.cell_state_queries, second.statistics.cell_state_queries);
}

}  // namespace
}  // namespace navigation_planning
