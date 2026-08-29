#include "mapping_world_model_adapter.hpp"
#include <navigation_mapping/mapping_world_snapshot.hpp>
#include <navigation_math/type_utils.hpp>

#include <array>
#include <algorithm>
#include <limits>
#include <type_traits>

#include <gtest/gtest.h>

namespace {

static_assert(!std::is_copy_constructible_v<navigation_mapping::MappingWorldSnapshot>);
static_assert(!std::is_move_constructible_v<navigation_mapping::MappingWorldSnapshot>);

navigation_world_model::CellState expectedCell(rog_map::GridType cell) {
  using navigation_world_model::CellState;
  switch (cell) {
    case rog_map::GridType::UNKNOWN: return CellState::kUnknown;
    case rog_map::GridType::OUT_OF_MAP: return CellState::kOutOfMap;
    case rog_map::GridType::OCCUPIED: return CellState::kOccupied;
    case rog_map::GridType::KNOWN_FREE: return CellState::kKnownFree;
    case rog_map::GridType::FRONTIER: return CellState::kFrontier;
    case rog_map::GridType::UNDEFINED: return CellState::kUndefined;
  }
  return CellState::kUndefined;
}

navigation_mapping::PlanningGrid productGrid(rog_map::PlanningGridExport source) {
  const auto layout = [](const rog_map::PlanningGridLayoutExport& input) {
    return navigation_mapping::PlanningGridLayout{
        input.resolution_m,
        input.global_min_index,
        input.dimensions,
        input.local_center_m.cast<double>(),
        input.local_size_m.cast<double>(),
    };
  };
  navigation_mapping::PlanningGrid result;
  result.base_layout = layout(source.base_layout);
  result.inflated.layout = layout(source.inflated.layout);
  result.base_state = std::move(source.base_state);
  result.inflated.occupied = std::move(source.inflated.occupied);
  result.inflated.unknown = std::move(source.inflated.unknown);
  if (source.nearest_offsets) {
    auto offsets = std::make_shared<std::vector<navigation_world_model::GridIndex3>>();
    offsets->reserve(source.nearest_offsets->size());
    for (const auto& offset : *source.nearest_offsets) {
      offsets->emplace_back(offset.x(), offset.y(), offset.z());
    }
    result.nearest_offsets = std::move(offsets);
  }
  result.unknown_inflation_enabled = source.unknown_inflation_enabled;
  result.virtual_ground_ceiling_enabled = source.virtual_ground_ceiling_enabled;
  result.virtual_ground_m = source.virtual_ground_m;
  result.virtual_ceiling_m = source.virtual_ceiling_m;
  result.inflated_virtual_ground_m = source.inflated_virtual_ground_m;
  result.inflated_virtual_ceiling_m = source.inflated_virtual_ceiling_m;
  result.occupied_inflation_radius_m = source.occupied_inflation_radius_m;
  return result;
}

class MappingWorldModelTest : public ::testing::Test {
 protected:
  void SetUp() override {
    map = std::make_shared<navigation_mapping::internal::RuntimeMappingMap>([] { return 1.0; });
    map->loadConfigAndInit(NAVIGATION_PLANNER_CONFIG_PATH);
    view = std::make_shared<navigation_mapping::internal::MappingWorldModelView>(map);
    snapshot = std::make_shared<navigation_mapping::MappingWorldSnapshot>(
        productGrid(map->exportPlanningGrid()),
        navigation_world_model::WorldSnapshotIdentity{1, 1, 1, 123});
  }

  std::shared_ptr<navigation_mapping::internal::RuntimeMappingMap> map;
  std::shared_ptr<navigation_mapping::internal::MappingWorldModelView> view;
  std::shared_ptr<navigation_mapping::MappingWorldSnapshot> snapshot;
};

TEST_F(MappingWorldModelTest, GeometryAndIdentityAreProductOwned) {
  const auto geometry = view->geometry();
  const auto& config = map->getMapConfig();
  EXPECT_DOUBLE_EQ(geometry.evidence_resolution_m, map->getResolution());
  EXPECT_DOUBLE_EQ(geometry.inflated_resolution_m, map->getInfResolution());
  EXPECT_DOUBLE_EQ(geometry.occupied_inflation_radius_m,
                   config.inflation_resolution * config.inflation_step);
  EXPECT_FALSE(geometry.virtual_ground_ceiling_enabled);
  EXPECT_EQ(geometry.local_center_m, snapshot->geometry().local_center_m);
  EXPECT_EQ(geometry.local_size_m, snapshot->geometry().local_size_m);
  EXPECT_DOUBLE_EQ(geometry.effective_virtual_ground_m,
                   geometry.local_center_m.z() - 0.5 * geometry.local_size_m.z());
  EXPECT_DOUBLE_EQ(geometry.effective_virtual_ceiling_m,
                   geometry.local_center_m.z() + 0.5 * geometry.local_size_m.z());
  EXPECT_FALSE(snapshot->geometry().virtual_ground_ceiling_enabled);

  EXPECT_EQ(view->identity().revision, 0U);
  view->recordSuccessfulUpdate(123456789);
  EXPECT_EQ(view->identity().generation, 1U);
  EXPECT_EQ(view->identity().revision, 1U);
  EXPECT_EQ(view->identity().observation_stamp_ns, 123456789);
}

TEST_F(MappingWorldModelTest, InvalidLayerAndDistanceFailClosed) {
  const auto invalid_layer = static_cast<navigation_world_model::GridLayer>(255);
  const auto point = Eigen::Vector3d::Zero();

  EXPECT_EQ(view->classify(point, invalid_layer),
            navigation_world_model::CellState::kOutOfMap);
  EXPECT_EQ(view->positionToIndex(point, invalid_layer),
            navigation_world_model::GridIndex3::Zero());
  EXPECT_EQ(view->indexToPosition(navigation_world_model::GridIndex3::Zero(), invalid_layer),
            navigation_world_model::Point3::Zero());
  EXPECT_FALSE(view->nearestNotOccupied(point, invalid_layer, 1.0).has_value());
  EXPECT_FALSE(view->nearestNotOccupied(point,
                                        navigation_world_model::GridLayer::kEvidence,
                                        std::numeric_limits<double>::quiet_NaN())
                   .has_value());
  EXPECT_FALSE(view->isSegmentTraversable(
      point, point, navigation_world_model::GridLayer::kEvidence,
      static_cast<navigation_world_model::UnknownPolicy>(255)));
}

TEST_F(MappingWorldModelTest, SnapshotChangeHistoryOnlyExemptsDisjointRegions) {
  const auto identity = navigation_world_model::WorldSnapshotIdentity{1, 1, 2, 200};
  const navigation_world_model::WorldChangeRecord record{
      identity,
      navigation_world_model::AxisAlignedBox{
          Eigen::Vector3d{10.0, 10.0, 10.0},
          Eigen::Vector3d{11.0, 11.0, 11.0}},
      false};
  const auto history = std::make_shared<navigation_world_model::WorldChangeHistory>(
      navigation_world_model::WorldChangeHistory{{record}});
  navigation_mapping::MappingWorldSnapshot current(
      productGrid(map->exportPlanningGrid()), identity, history);

  EXPECT_FALSE(current.changedRegionIntersectsSince(
      navigation_world_model::WorldSnapshotIdentity{1, 1, 1, 100},
      navigation_world_model::AxisAlignedBox{
          Eigen::Vector3d{-1.0, -1.0, -1.0},
          Eigen::Vector3d{1.0, 1.0, 1.0}}));
  EXPECT_TRUE(current.changedRegionIntersectsSince(
      navigation_world_model::WorldSnapshotIdentity{1, 1, 1, 100},
      navigation_world_model::AxisAlignedBox{
          Eigen::Vector3d{10.5, 10.5, 10.5},
          Eigen::Vector3d{12.0, 12.0, 12.0}}));
  EXPECT_TRUE(current.changedRegionIntersectsSince(
      navigation_world_model::WorldSnapshotIdentity{1, 1, 0, 0},
      navigation_world_model::AxisAlignedBox{
          Eigen::Vector3d{-1.0, -1.0, -1.0},
          Eigen::Vector3d{1.0, 1.0, 1.0}}));
  EXPECT_TRUE(current.changedRegionIntersectsSince(
      navigation_world_model::WorldSnapshotIdentity{1, 1, 2, 199},
      navigation_world_model::AxisAlignedBox{
          Eigen::Vector3d{-1.0, -1.0, -1.0},
          Eigen::Vector3d{1.0, 1.0, 1.0}}));
}

TEST_F(MappingWorldModelTest, ChangeHistoryRetainsNewestRecordsAndFailsClosedWhenTruncated) {
  const auto region = [](double value) {
    return navigation_world_model::AxisAlignedBox{
        Eigen::Vector3d{value, value, value},
        Eigen::Vector3d{value + 0.1, value + 0.1, value + 0.1}};
  };
  const auto history = std::make_shared<navigation_world_model::WorldChangeHistory>();
  history->records = {
      navigation_world_model::WorldChangeRecord{
          navigation_world_model::WorldSnapshotIdentity{1, 1, 4, 400}, region(4.0), false},
      navigation_world_model::WorldChangeRecord{
          navigation_world_model::WorldSnapshotIdentity{1, 1, 3, 300}, region(3.0), false},
      navigation_world_model::WorldChangeRecord{
          navigation_world_model::WorldSnapshotIdentity{1, 1, 2, 200}, region(2.0), false},
  };
  navigation_mapping::MappingWorldSnapshot current(
      productGrid(map->exportPlanningGrid()),
      navigation_world_model::WorldSnapshotIdentity{1, 1, 4, 400}, history);

  EXPECT_FALSE(current.changedRegionIntersectsSince(
      navigation_world_model::WorldSnapshotIdentity{1, 1, 2, 200},
      navigation_world_model::AxisAlignedBox{
          Eigen::Vector3d{-1.0, -1.0, -1.0},
          Eigen::Vector3d{1.0, 1.0, 1.0}}));
  EXPECT_TRUE(current.changedRegionIntersectsSince(
      navigation_world_model::WorldSnapshotIdentity{1, 1, 0, 0},
      navigation_world_model::AxisAlignedBox{
          Eigen::Vector3d{-1.0, -1.0, -1.0},
          Eigen::Vector3d{1.0, 1.0, 1.0}}));
}

TEST_F(MappingWorldModelTest, ClassificationAndIndexConversionMatchBackend) {
  const auto geometry = view->geometry();
  const std::array<Eigen::Vector3d, 5> points{
      geometry.local_center_m,
      geometry.local_center_m + Eigen::Vector3d{0.5, -0.5, 0.5},
      geometry.local_center_m - 0.49 * geometry.local_size_m,
      geometry.local_center_m + 0.49 * geometry.local_size_m,
      geometry.local_center_m + geometry.local_size_m};

  for (const auto& point : points) {
    EXPECT_EQ(view->classify(point, navigation_world_model::GridLayer::kEvidence),
              expectedCell(map->getGridType(point)));
    EXPECT_EQ(view->classify(point, navigation_world_model::GridLayer::kInflated),
              expectedCell(map->getInfGridType(point)));
    EXPECT_EQ(snapshot->classify(point, navigation_world_model::GridLayer::kEvidence),
              view->classify(point, navigation_world_model::GridLayer::kEvidence));
    EXPECT_EQ(snapshot->classify(point, navigation_world_model::GridLayer::kInflated),
              view->classify(point, navigation_world_model::GridLayer::kInflated));
    for (const auto layer : {navigation_world_model::GridLayer::kEvidence,
                             navigation_world_model::GridLayer::kInflated}) {
      Eigen::Vector3i legacy_index;
      if (layer == navigation_world_model::GridLayer::kInflated) {
        map->infMapPosToGlobalIndex(point, legacy_index);
      } else {
        map->probMapPosToGlobalIndex(point, legacy_index);
      }
      EXPECT_EQ(view->positionToIndex(point, layer), legacy_index);
      EXPECT_EQ(snapshot->positionToIndex(point, layer), legacy_index);
      Eigen::Vector3d legacy_position;
      if (layer == navigation_world_model::GridLayer::kInflated) {
        map->infMapGlobalIndexToPos(legacy_index, legacy_position);
      } else {
        map->probMapGlobalIndexToPos(legacy_index, legacy_position);
      }
      EXPECT_EQ(view->indexToPosition(legacy_index, layer), legacy_position);
      EXPECT_EQ(snapshot->indexToPosition(legacy_index, layer), legacy_position);
    }
  }

  const Eigen::Vector3d nan_point{
      std::numeric_limits<double>::quiet_NaN(), 0.0, 0.0};
  EXPECT_EQ(view->classify(nan_point, navigation_world_model::GridLayer::kEvidence),
            navigation_world_model::CellState::kOutOfMap);
  EXPECT_FALSE(view->contains(nan_point));
}

TEST_F(MappingWorldModelTest, SegmentAndBoxQueriesPreserveBackendSemantics) {
  const auto center = view->geometry().local_center_m;
  const Eigen::Vector3d end = center + Eigen::Vector3d{0.3, 0.2, 0.1};
  for (const auto layer : {navigation_world_model::GridLayer::kEvidence,
                           navigation_world_model::GridLayer::kInflated}) {
    for (const auto policy : {navigation_world_model::UnknownPolicy::kAllowUnknown,
                              navigation_world_model::UnknownPolicy::kRequireKnownFree}) {
      EXPECT_EQ(view->isSegmentTraversable(center, end, layer, policy),
                map->isLineFree(
                    center, end,
                    layer == navigation_world_model::GridLayer::kInflated,
                    policy == navigation_world_model::UnknownPolicy::kRequireKnownFree));
      EXPECT_EQ(snapshot->isSegmentTraversable(center, end, layer, policy),
                view->isSegmentTraversable(center, end, layer, policy))
          << "layer=" << static_cast<int>(layer)
          << " policy=" << static_cast<int>(policy)
          << " center=" << center.transpose() << " end=" << end.transpose();
    }
  }

  navigation_world_model::AxisAlignedBox requested{
      center - Eigen::Vector3d::Constant(1000.0),
      center + Eigen::Vector3d::Constant(1000.0)};
  auto legacy_min = requested.minimum;
  auto legacy_max = requested.maximum;
  map->boundBoxByLocalMap(legacy_min, legacy_max);
  const auto bounded = view->clampToLocalBounds(requested);
  EXPECT_EQ(bounded.minimum, legacy_min);
  EXPECT_EQ(bounded.maximum, legacy_max);

  navigation_world_model::PointVector legacy_points;
  map->boxSearchObservedOccupied(legacy_min, legacy_max, legacy_points);
  const auto product_points = view->observedOccupiedPoints(bounded);
  EXPECT_EQ(product_points, legacy_points);
  EXPECT_EQ(snapshot->clampToLocalBounds(requested).minimum, bounded.minimum);
  EXPECT_EQ(snapshot->clampToLocalBounds(requested).maximum, bounded.maximum);
  EXPECT_EQ(snapshot->observedOccupiedPoints(bounded), legacy_points);
}

TEST_F(MappingWorldModelTest, VirtualPlaneClassificationAndRayBoundariesMatchBackend) {
  const auto geometry = view->geometry();
  const double resolution = geometry.inflated_resolution_m;
  const std::array<double, 10> heights{
      geometry.effective_virtual_ground_m - 1.0e-9,
      geometry.effective_virtual_ground_m,
      geometry.effective_virtual_ground_m + 1.0e-9,
      geometry.effective_virtual_ground_m + resolution,
      geometry.effective_virtual_ground_m + 5.0 * resolution,
      geometry.effective_virtual_ceiling_m - 5.0 * resolution,
      geometry.effective_virtual_ceiling_m - resolution,
      geometry.effective_virtual_ceiling_m - 1.0e-9,
      geometry.effective_virtual_ceiling_m,
      geometry.effective_virtual_ceiling_m + 1.0e-9};
  for (const double height : heights) {
    const Eigen::Vector3d start{geometry.local_center_m.x(), geometry.local_center_m.y(), height};
    const Eigen::Vector3d end = start + Eigen::Vector3d{0.05, 0.0, 0.0};
    for (const auto layer : {navigation_world_model::GridLayer::kEvidence,
                             navigation_world_model::GridLayer::kInflated}) {
      EXPECT_EQ(snapshot->classify(start, layer), view->classify(start, layer))
          << "height=" << height << " layer=" << static_cast<int>(layer);
      for (const auto policy : {navigation_world_model::UnknownPolicy::kAllowUnknown,
                                navigation_world_model::UnknownPolicy::kRequireKnownFree}) {
        EXPECT_EQ(snapshot->isSegmentTraversable(start, end, layer, policy),
                  view->isSegmentTraversable(start, end, layer, policy))
            << "height=" << height << " layer=" << static_cast<int>(layer)
            << " policy=" << static_cast<int>(policy);
      }
    }
  }
}

TEST_F(MappingWorldModelTest, VirtualPlaneBoundaryIsOccupiedForRayCertificates) {
  auto grid = productGrid(map->exportPlanningGrid());
  const auto center = grid.base_layout.local_center_m;
  grid.virtual_ground_ceiling_enabled = true;
  grid.virtual_ground_m = center.z() - 1.0;
  grid.virtual_ceiling_m = center.z() + 1.0;
  grid.inflated_virtual_ground_m = center.z() - 0.5;
  grid.inflated_virtual_ceiling_m = center.z() + 0.5;
  navigation_mapping::MappingWorldSnapshot guarded(
      std::move(grid), navigation_world_model::WorldSnapshotIdentity{1, 1, 2, 123});

  const std::array<std::pair<navigation_world_model::GridLayer, double>, 2> boundaries{
      std::pair{navigation_world_model::GridLayer::kEvidence, center.z() - 1.0},
      std::pair{navigation_world_model::GridLayer::kInflated, center.z() - 0.5}};
  for (const auto& [layer, height] : boundaries) {
    const Eigen::Vector3d start{center.x(), center.y(), height};
    const Eigen::Vector3d end = start + Eigen::Vector3d{0.05, 0.0, 0.0};
    EXPECT_EQ(guarded.classify(start, layer),
              navigation_world_model::CellState::kOccupied);
    EXPECT_FALSE(guarded.isSegmentTraversable(
        start, end, layer, navigation_world_model::UnknownPolicy::kAllowUnknown));
    EXPECT_FALSE(guarded.isSegmentTraversable(
        start, end, layer, navigation_world_model::UnknownPolicy::kRequireKnownFree));
  }
}

TEST_F(MappingWorldModelTest, SegmentCertificateIsSymmetricAtVoxelCorners) {
  auto grid = productGrid(map->exportPlanningGrid());
  std::fill(grid.base_state.begin(), grid.base_state.end(),
            static_cast<std::uint8_t>(navigation_world_model::CellState::kKnownFree));
  std::fill(grid.inflated.occupied.begin(), grid.inflated.occupied.end(), 0U);
  grid.unknown_inflation_enabled = false;
  grid.virtual_ground_ceiling_enabled = false;

  const auto start_index = grid.base_layout.global_min_index +
      navigation_world_model::GridIndex3{4, 4, 4};
  const auto side_obstacle_index = grid.base_layout.global_min_index +
      navigation_world_model::GridIndex3{4, 5, 4};
  const auto local = side_obstacle_index - grid.base_layout.global_min_index;
  const auto obstacle_offset =
      (static_cast<std::size_t>(local.x()) *
           static_cast<std::size_t>(grid.base_layout.dimensions.y()) +
       static_cast<std::size_t>(local.y())) *
          static_cast<std::size_t>(grid.base_layout.dimensions.z()) +
      static_cast<std::size_t>(local.z());
  ASSERT_LT(obstacle_offset, grid.base_state.size());
  grid.base_state[obstacle_offset] =
      static_cast<std::uint8_t>(navigation_world_model::CellState::kOccupied);

  navigation_mapping::MappingWorldSnapshot snapshot(
      std::move(grid), navigation_world_model::WorldSnapshotIdentity{1, 1, 2, 123});
  const auto end_index = start_index + navigation_world_model::GridIndex3{2, 2, 0};
  const auto start = snapshot.indexToPosition(
      start_index, navigation_world_model::GridLayer::kEvidence);
  const auto end = snapshot.indexToPosition(
      end_index, navigation_world_model::GridLayer::kEvidence);

  EXPECT_FALSE(snapshot.isSegmentTraversable(
      start, end, navigation_world_model::GridLayer::kEvidence,
      navigation_world_model::UnknownPolicy::kRequireKnownFree));
  EXPECT_FALSE(snapshot.isSegmentTraversable(
      end, start, navigation_world_model::GridLayer::kEvidence,
      navigation_world_model::UnknownPolicy::kRequireKnownFree));
}

TEST_F(MappingWorldModelTest, StationaryAxisDoesNotCreateSpuriousDdaTie) {
  auto grid = productGrid(map->exportPlanningGrid());
  std::fill(grid.base_state.begin(), grid.base_state.end(),
            static_cast<std::uint8_t>(navigation_world_model::CellState::kKnownFree));
  std::fill(grid.inflated.occupied.begin(), grid.inflated.occupied.end(), 0U);
  grid.unknown_inflation_enabled = false;
  grid.virtual_ground_ceiling_enabled = false;

  const auto start_index = grid.base_layout.global_min_index +
      navigation_world_model::GridIndex3{4, 4, 4};
  const auto spurious_diagonal = grid.base_layout.global_min_index +
      navigation_world_model::GridIndex3{5, 5, 4};
  const auto local = spurious_diagonal - grid.base_layout.global_min_index;
  const auto obstacle_offset =
      (static_cast<std::size_t>(local.x()) *
           static_cast<std::size_t>(grid.base_layout.dimensions.y()) +
       static_cast<std::size_t>(local.y())) *
          static_cast<std::size_t>(grid.base_layout.dimensions.z()) +
      static_cast<std::size_t>(local.z());
  ASSERT_LT(obstacle_offset, grid.base_state.size());
  grid.base_state[obstacle_offset] =
      static_cast<std::uint8_t>(navigation_world_model::CellState::kOccupied);

  navigation_mapping::MappingWorldSnapshot snapshot(
      std::move(grid), navigation_world_model::WorldSnapshotIdentity{1, 1, 2, 123});
  const auto end_index = start_index + navigation_world_model::GridIndex3{3, 1, 0};
  EXPECT_TRUE(snapshot.isSegmentTraversable(
      snapshot.indexToPosition(start_index, navigation_world_model::GridLayer::kEvidence),
      snapshot.indexToPosition(end_index, navigation_world_model::GridLayer::kEvidence),
      navigation_world_model::GridLayer::kEvidence,
      navigation_world_model::UnknownPolicy::kRequireKnownFree));
}

TEST_F(MappingWorldModelTest, InflatedSegmentRejectsObservedTubeBelowRobotRadius) {
  auto grid = productGrid(map->exportPlanningGrid());
  std::fill(grid.base_state.begin(), grid.base_state.end(),
            static_cast<std::uint8_t>(navigation_world_model::CellState::kKnownFree));
  std::fill(grid.inflated.occupied.begin(), grid.inflated.occupied.end(), 0U);
  grid.unknown_inflation_enabled = false;
  grid.virtual_ground_ceiling_enabled = false;

  const auto obstacle_index = grid.base_layout.global_min_index +
      navigation_world_model::GridIndex3{4, 8, 4};
  const auto local = obstacle_index - grid.base_layout.global_min_index;
  const auto obstacle_offset =
      (static_cast<std::size_t>(local.x()) *
           static_cast<std::size_t>(grid.base_layout.dimensions.y()) +
       static_cast<std::size_t>(local.y())) *
          static_cast<std::size_t>(grid.base_layout.dimensions.z()) +
      static_cast<std::size_t>(local.z());
  ASSERT_LT(obstacle_offset, grid.base_state.size());
  grid.base_state[obstacle_offset] =
      static_cast<std::uint8_t>(navigation_world_model::CellState::kOccupied);

  navigation_mapping::MappingWorldSnapshot obstacle_snapshot(
      std::move(grid), navigation_world_model::WorldSnapshotIdentity{1, 1, 2, 123});
  const auto start = obstacle_snapshot.indexToPosition(
      obstacle_index + navigation_world_model::GridIndex3{-4, -4, 0},
      navigation_world_model::GridLayer::kEvidence);
  const auto end = obstacle_snapshot.indexToPosition(
      obstacle_index + navigation_world_model::GridIndex3{4, -4, 0},
      navigation_world_model::GridLayer::kEvidence);

  EXPECT_FALSE(obstacle_snapshot.isSegmentTraversable(
      start, end, navigation_world_model::GridLayer::kInflated,
      navigation_world_model::UnknownPolicy::kAllowUnknown));
}

TEST_F(MappingWorldModelTest, NearestCellPreservesBaseGridOrderingForBothLayers) {
  const auto center = view->geometry().local_center_m;
  for (const auto layer : {navigation_world_model::GridLayer::kEvidence,
                           navigation_world_model::GridLayer::kInflated}) {
    const auto live = view->nearestNotOccupied(center, layer, 3.0);
    const auto immutable = snapshot->nearestNotOccupied(center, layer, 3.0);
    ASSERT_EQ(immutable.has_value(), live.has_value());
    if (live) EXPECT_EQ(*immutable, *live);
  }
}

TEST_F(MappingWorldModelTest, SnapshotIdentityAndQueriesStayImmutableAfterMapUpdate) {
  const auto identity = snapshot->identity();
  const Eigen::Vector3d obstacle{3.0, 0.0, 0.0};
  const auto before = snapshot->classify(
      obstacle, navigation_world_model::GridLayer::kEvidence);

  rog_map::PointCloud cloud;
  pcl::PointXYZI point;
  point.x = static_cast<float>(obstacle.x());
  point.y = static_cast<float>(obstacle.y());
  point.z = static_cast<float>(obstacle.z());
  point.intensity = 0.0F;
  cloud.push_back(point);
  const navigation_math::Pose pose{Eigen::Vector3d::Zero(), Eigen::Quaterniond::Identity()};
  for (int iteration = 0; iteration < 8; ++iteration) {
    EXPECT_EQ(map->updateMap(cloud, pose), rog_map::MapUpdateOutcome::UPDATED);
  }

  EXPECT_EQ(snapshot->identity().generation, identity.generation);
  EXPECT_EQ(snapshot->identity().revision, identity.revision);
  EXPECT_EQ(snapshot->identity().observation_stamp_ns, identity.observation_stamp_ns);
  EXPECT_EQ(snapshot->classify(obstacle, navigation_world_model::GridLayer::kEvidence), before);
  EXPECT_NE(view->classify(obstacle, navigation_world_model::GridLayer::kEvidence), before);
}

TEST_F(MappingWorldModelTest, SnapshotRejectsMalformedDetachedStorageBeforePublication) {
  auto malformed = productGrid(map->exportPlanningGrid());
  malformed.inflated.occupied.pop_back();
  EXPECT_THROW(
      navigation_mapping::MappingWorldSnapshot(
          std::move(malformed), navigation_world_model::WorldSnapshotIdentity{1, 1, 2, 123}),
      std::invalid_argument);

  auto invalid_identity_grid = productGrid(map->exportPlanningGrid());
  EXPECT_THROW(
      navigation_mapping::MappingWorldSnapshot(
          std::move(invalid_identity_grid),
          navigation_world_model::WorldSnapshotIdentity{1, 0, 2, 123}),
      std::invalid_argument);

  auto invalid_state_grid = productGrid(map->exportPlanningGrid());
  ASSERT_FALSE(invalid_state_grid.base_state.empty());
  invalid_state_grid.base_state.front() =
      static_cast<std::uint8_t>(navigation_world_model::CellState::kFrontier);
  EXPECT_THROW(
      navigation_mapping::MappingWorldSnapshot(
          std::move(invalid_state_grid),
          navigation_world_model::WorldSnapshotIdentity{1, 1, 2, 123}),
      std::invalid_argument);
}

TEST_F(MappingWorldModelTest, SnapshotRejectsNonFiniteInflationRadius) {
  auto malformed = productGrid(map->exportPlanningGrid());
  malformed.occupied_inflation_radius_m = std::numeric_limits<double>::quiet_NaN();
  EXPECT_THROW(
      navigation_mapping::MappingWorldSnapshot(
          std::move(malformed),
          navigation_world_model::WorldSnapshotIdentity{1, 1, 2, 123}),
      std::invalid_argument);
}

TEST_F(MappingWorldModelTest, InflatedQueriesUseInflatedLayerBounds) {
  auto shifted = productGrid(map->exportPlanningGrid());
  shifted.inflated.layout.global_min_index.x() += shifted.inflated.layout.dimensions.x();
  navigation_mapping::MappingWorldSnapshot detached(
      std::move(shifted), navigation_world_model::WorldSnapshotIdentity{1, 1, 2, 123});

  const auto center = view->geometry().local_center_m;
  EXPECT_TRUE(detached.contains(center));
  EXPECT_EQ(detached.classify(center, navigation_world_model::GridLayer::kInflated),
            navigation_world_model::CellState::kOutOfMap);
  EXPECT_FALSE(detached.isSegmentTraversable(
      center, center, navigation_world_model::GridLayer::kInflated,
      navigation_world_model::UnknownPolicy::kAllowUnknown));
}

TEST(WorldModelPolicy, TraversabilityIsTotalAndFailClosed) {
  using navigation_world_model::CellState;
  using navigation_world_model::UnknownPolicy;
  EXPECT_TRUE(navigation_world_model::isCellTraversable(
      CellState::kKnownFree, UnknownPolicy::kRequireKnownFree));
  EXPECT_TRUE(navigation_world_model::isCellTraversable(
      CellState::kUnknown, UnknownPolicy::kAllowUnknown));
  EXPECT_FALSE(navigation_world_model::isCellTraversable(
      CellState::kUnknown, UnknownPolicy::kRequireKnownFree));
  for (const auto state : {CellState::kUndefined, CellState::kOutOfMap,
                           CellState::kOccupied}) {
    EXPECT_FALSE(navigation_world_model::isCellTraversable(
        state, UnknownPolicy::kAllowUnknown));
  }
  EXPECT_TRUE(navigation_world_model::isCellTraversable(
      CellState::kFrontier, UnknownPolicy::kAllowUnknown));
  EXPECT_FALSE(navigation_world_model::isCellTraversable(
      static_cast<CellState>(255U), UnknownPolicy::kAllowUnknown));
  EXPECT_TRUE(navigation_world_model::isStoredCellState(CellState::kUnknown));
  EXPECT_FALSE(navigation_world_model::isStoredCellState(CellState::kFrontier));
}

}  // namespace
