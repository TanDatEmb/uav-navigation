#include <gtest/gtest.h>

#include <limits>

#include "rog_map_ros/semantic_visualization.hpp"

namespace uav::nav::rog {
namespace {

MapBounds testBounds() {
  return {{-2.0, -2.0, -2.0}, {2.0, 2.0, 2.0}};
}

VisualizationVoxelSet makeSet(std::initializer_list<VisualizationVoxelKey> keys) {
  return VisualizationVoxelSet(keys.begin(), keys.end());
}

TEST(SemanticVisualizationTest, ClearanceIsSubsetAndDisjointFromOccupied) {
  const auto occupied = makeSet({{0, 0, 0}, {1, 0, 0}});
  const auto full = makeSet({{0, 0, 0}, {1, 0, 0}, {2, 0, 0}});
  const auto clearance = deriveClearanceSurface(full, occupied);
  EXPECT_EQ(clearance, makeSet({{2, 0, 0}}));
  for (const auto& key : clearance) {
    EXPECT_TRUE(full.contains(key));
    EXPECT_FALSE(occupied.contains(key));
  }
}

TEST(SemanticVisualizationTest, SerializationIsLexicographicallyDeterministic) {
  const auto first = makeSet({{2, 0, 0}, {-1, 4, 0}, {0, -2, 1}});
  const auto second = makeSet({{0, -2, 1}, {2, 0, 0}, {-1, 4, 0}});
  const auto a = sortedCentersFromVisualizationSet(first, 1.0);
  const auto b = sortedCentersFromVisualizationSet(second, 1.0);
  ASSERT_EQ(a.size(), b.size());
  for (std::size_t i = 0; i < a.size(); ++i) EXPECT_EQ(a[i], b[i]);
  EXPECT_EQ(sortedVisualizationVoxelKeys(first),
            (std::vector<VisualizationVoxelKey>{{-1, 4, 0}, {0, -2, 1}, {2, 0, 0}}));
}

TEST(SemanticVisualizationTest, MarkerArrayHasFixedIdsFrameStampAndDisjointLayers) {
  const auto message = makeSemanticMarkerArray(
      "lio_odom", 4'200'000'123LL, makeSet({{0, 0, 0}}),
      makeSet({{0, 0, 0}, {1, 0, 0}}), testBounds(), 0.30, 0.90);
  ASSERT_EQ(message.markers.size(), 3U);
  for (const auto& marker : message.markers) {
    EXPECT_EQ(marker.ns, kSemanticMarkerNamespace);
    EXPECT_EQ(marker.header.frame_id, "lio_odom");
    EXPECT_EQ(marker.header.stamp.sec, 4);
    EXPECT_EQ(marker.header.stamp.nanosec, 200'000'123U);
    EXPECT_DOUBLE_EQ(marker.pose.orientation.x, 0.0);
    EXPECT_DOUBLE_EQ(marker.pose.orientation.y, 0.0);
    EXPECT_DOUBLE_EQ(marker.pose.orientation.z, 0.0);
    EXPECT_DOUBLE_EQ(marker.pose.orientation.w, 1.0);
    EXPECT_EQ(marker.lifetime.sec, 0);
    EXPECT_EQ(marker.lifetime.nanosec, 0U);
  }
  EXPECT_EQ(message.markers[0].id, kSemanticOccupiedMarkerId);
  EXPECT_EQ(message.markers[1].id, kSemanticClearanceMarkerId);
  EXPECT_EQ(message.markers[2].id, kSemanticBoundsMarkerId);
  EXPECT_EQ(message.markers[0].points.size(), 1U);
  EXPECT_EQ(message.markers[1].points.size(), 1U);
  EXPECT_EQ(message.markers[2].type, visualization_msgs::msg::Marker::LINE_LIST);
  EXPECT_FLOAT_EQ(message.markers[0].scale.x, 0.27F);
  EXPECT_FLOAT_EQ(message.markers[0].scale.y, 0.27F);
  EXPECT_FLOAT_EQ(message.markers[0].scale.z, 0.27F);
  EXPECT_FLOAT_EQ(message.markers[0].color.a, 1.0F);
  EXPECT_FLOAT_EQ(message.markers[1].color.a, 1.0F);
  EXPECT_FLOAT_EQ(message.markers[2].color.a, 0.90F);
}

TEST(SemanticVisualizationTest, InvalidCubeScaleRatioFailsFast) {
  EXPECT_THROW(validateCubeScaleRatio(0.0), std::invalid_argument);
  EXPECT_THROW(validateCubeScaleRatio(1.01), std::invalid_argument);
  EXPECT_THROW(validateCubeScaleRatio(std::numeric_limits<double>::quiet_NaN()), std::invalid_argument);
}

TEST(SemanticVisualizationTest, LifecycleDeleteCoversEveryCanonicalMarker) {
  const auto message = makeSemanticDeleteArray("lio_odom", 8);
  ASSERT_EQ(message.markers.size(), 3U);
  EXPECT_EQ(message.markers[0].action, visualization_msgs::msg::Marker::DELETE);
  EXPECT_EQ(message.markers[1].action, visualization_msgs::msg::Marker::DELETE);
  EXPECT_EQ(message.markers[2].action, visualization_msgs::msg::Marker::DELETE);
  EXPECT_EQ(message.markers[0].ns, kSemanticMarkerNamespace);
  EXPECT_EQ(message.markers[2].id, kSemanticBoundsMarkerId);
}

TEST(SemanticVisualizationTest, CanonicalSerializationDoesNotMutateSourceSets) {
  const auto occupied = makeSet({{0, 0, 0}});
  const auto full = makeSet({{0, 0, 0}, {1, 0, 0}});
  const auto occupied_before = occupied;
  const auto full_before = full;
  (void)makeSemanticMarkerArray("lio_odom", 1, occupied, full, testBounds(), 0.30, 0.90);
  EXPECT_EQ(occupied, occupied_before);
  EXPECT_EQ(full, full_before);
}

}  // namespace
}  // namespace uav::nav::rog
