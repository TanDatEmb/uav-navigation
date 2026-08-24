#include <navigation_runtime/rog_world_model_adapter.hpp>
#include <navigation_runtime/rog_world_snapshot.hpp>

#include <array>
#include <limits>
#include <type_traits>

#include <gtest/gtest.h>

namespace {

static_assert(!std::is_copy_constructible_v<navigation_runtime::RogWorldSnapshot>);
static_assert(!std::is_move_constructible_v<navigation_runtime::RogWorldSnapshot>);

navigation_world_model::CellState expectedCell(rog_map::GridType cell) {
  using navigation_world_model::CellState;
  switch (cell) {
    case super_utils::UNKNOWN: return CellState::kUnknown;
    case super_utils::OUT_OF_MAP: return CellState::kOutOfMap;
    case super_utils::OCCUPIED: return CellState::kOccupied;
    case super_utils::KNOWN_FREE: return CellState::kKnownFree;
    case super_utils::FRONTIER: return CellState::kFrontier;
    case super_utils::UNDEFINED: return CellState::kUndefined;
  }
  return CellState::kUndefined;
}

class RogWorldModelAdapterTest : public ::testing::Test {
 protected:
  void SetUp() override {
    map = std::make_shared<navigation_runtime::RuntimeRogMap>([] { return 1.0; });
    map->loadConfigAndInit(SUPER_PRODUCT_CONFIG_PATH);
    view = std::make_shared<navigation_runtime::RogWorldModelView>(map);
    snapshot = std::make_shared<navigation_runtime::RogWorldSnapshot>(
        map->exportPlanningGrid(),
        navigation_world_model::WorldSnapshotIdentity{1, 1, 123});
  }

  std::shared_ptr<navigation_runtime::RuntimeRogMap> map;
  std::shared_ptr<navigation_runtime::RogWorldModelView> view;
  std::shared_ptr<navigation_runtime::RogWorldSnapshot> snapshot;
};

TEST_F(RogWorldModelAdapterTest, GeometryAndIdentityAreProductOwned) {
  const auto geometry = view->geometry();
  const auto& config = map->getMapConfig();
  EXPECT_DOUBLE_EQ(geometry.evidence_resolution_m, map->getResolution());
  EXPECT_DOUBLE_EQ(geometry.inflated_resolution_m, map->getInfResolution());
  EXPECT_DOUBLE_EQ(geometry.occupied_inflation_radius_m,
                   config.inflation_resolution * config.inflation_step);
  EXPECT_DOUBLE_EQ(geometry.effective_virtual_ground_m, config.virtual_ground_height);
  EXPECT_DOUBLE_EQ(geometry.effective_virtual_ceiling_m, config.virtual_ceil_height);

  EXPECT_EQ(view->identity().revision, 0U);
  view->recordSuccessfulUpdate(123456789);
  EXPECT_EQ(view->identity().generation, 1U);
  EXPECT_EQ(view->identity().revision, 1U);
  EXPECT_EQ(view->identity().observation_stamp_ns, 123456789);
}

TEST_F(RogWorldModelAdapterTest, ClassificationAndIndexConversionMatchRog) {
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

TEST_F(RogWorldModelAdapterTest, SegmentAndBoxQueriesPreserveLegacySemantics) {
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

TEST_F(RogWorldModelAdapterTest, VirtualPlaneClassificationAndRayBoundariesMatchRog) {
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

TEST_F(RogWorldModelAdapterTest, NearestCellPreservesLegacyBaseGridOrderingForBothLayers) {
  const auto center = view->geometry().local_center_m;
  for (const auto layer : {navigation_world_model::GridLayer::kEvidence,
                           navigation_world_model::GridLayer::kInflated}) {
    const auto live = view->nearestNotOccupied(center, layer, 3.0);
    const auto immutable = snapshot->nearestNotOccupied(center, layer, 3.0);
    ASSERT_EQ(immutable.has_value(), live.has_value());
    if (live) EXPECT_EQ(*immutable, *live);
  }
}

TEST_F(RogWorldModelAdapterTest, SnapshotIdentityAndQueriesStayImmutableAfterMapUpdate) {
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
  const super_utils::Pose pose{Eigen::Vector3d::Zero(), Eigen::Quaterniond::Identity()};
  for (int iteration = 0; iteration < 8; ++iteration) map->updateMap(cloud, pose);

  EXPECT_EQ(snapshot->identity().generation, identity.generation);
  EXPECT_EQ(snapshot->identity().revision, identity.revision);
  EXPECT_EQ(snapshot->identity().observation_stamp_ns, identity.observation_stamp_ns);
  EXPECT_EQ(snapshot->classify(obstacle, navigation_world_model::GridLayer::kEvidence), before);
  EXPECT_NE(view->classify(obstacle, navigation_world_model::GridLayer::kEvidence), before);
}

TEST_F(RogWorldModelAdapterTest, SnapshotRejectsMalformedDetachedStorageBeforePublication) {
  auto malformed = map->exportPlanningGrid();
  malformed.inflated.occupied.pop_back();
  EXPECT_THROW(
      navigation_runtime::RogWorldSnapshot(
          std::move(malformed), navigation_world_model::WorldSnapshotIdentity{1, 2, 123}),
      std::invalid_argument);

  auto invalid_identity_grid = map->exportPlanningGrid();
  EXPECT_THROW(
      navigation_runtime::RogWorldSnapshot(
          std::move(invalid_identity_grid),
          navigation_world_model::WorldSnapshotIdentity{0, 2, 123}),
      std::invalid_argument);
}

}  // namespace
