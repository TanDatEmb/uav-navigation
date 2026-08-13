#include <gtest/gtest.h>

#include <cmath>
#include <limits>

#include "navigation_mapping/mapping_point_filter.hpp"

namespace navigation_mapping {
namespace {

TEST(MappingPointFilterTest, DownsamplesDensePointsToOneCentroidPerVoxel) {
  MappingPointFilterConfig config;
  config.voxel_size_m = 1.0;
  config.minimum_range_m = 0.0;
  config.maximum_range_m = 0.0;
  MappingPointFilter filter(config);

  std::vector<Point3f> points;
  for (int i = 0; i < 100; ++i) {
    points.emplace_back(0.01 * i, 0.0, 0.0);  // all inside voxel [0,1)
  }
  MappingPointFilterStats stats;
  const auto filtered = filter.filter(points, &stats);
  ASSERT_EQ(filtered.size(), 1U);
  EXPECT_TRUE(filtered.front().allFinite());
  EXPECT_NEAR(filtered.front().x(), 0.49499999996274707, 1e-12);
  EXPECT_NEAR(filtered.front().y(), 0.0, 1e-12);
  EXPECT_NEAR(filtered.front().z(), 0.0, 1e-12);
  EXPECT_EQ(stats.input_point_count, 100U);
  EXPECT_EQ(stats.post_filter_nonfinite_point_count, 0U);
  EXPECT_EQ(stats.output_point_count, 1U);
}

TEST(MappingPointFilterTest, ProducesFiniteCentroidsForMultipleAndNegativeVoxels) {
  MappingPointFilterConfig config;
  config.voxel_size_m = 1.0;
  MappingPointFilter filter(config);
  const std::vector<Point3f> points = {
      {-1.9, -0.1, 0.1}, {-1.1, -0.2, 0.2}, {-0.9, -0.3, 0.3},
      {0.1, 0.1, 0.1},  {0.9, 0.9, 0.9}};

  MappingPointFilterStats stats;
  const auto filtered = filter.filter(points, &stats);

  ASSERT_EQ(filtered.size(), 3U);
  EXPECT_EQ(stats.post_filter_nonfinite_point_count, 0U);
  for (const auto& point : filtered) {
    EXPECT_TRUE(point.allFinite());
  }
}

TEST(MappingPointFilterTest, RepeatedCallsAndDenseVoxelRemainFiniteAndExact) {
  MappingPointFilterConfig config;
  config.voxel_size_m = 0.5;
  MappingPointFilter filter(config);
  std::vector<Point3f> points;
  points.reserve(10000);
  for (int i = 0; i < 10000; ++i) {
    points.emplace_back(0.1 + (i % 100) * 1e-5, -0.2, 3.0);
  }

  MappingPointFilterStats first_stats;
  const auto first = filter.filter(points, &first_stats);
  MappingPointFilterStats second_stats;
  const auto second = filter.filter(points, &second_stats);

  ASSERT_EQ(first.size(), 1U);
  ASSERT_EQ(second.size(), 1U);
  EXPECT_TRUE(first.front().allFinite());
  EXPECT_TRUE(second.front().allFinite());
  EXPECT_NEAR(first.front().x(), 0.10049500003457069, 1e-12);
  EXPECT_NEAR(second.front().x(), first.front().x(), 1e-12);
  EXPECT_EQ(first_stats.post_filter_nonfinite_point_count, 0U);
  EXPECT_EQ(second_stats.post_filter_nonfinite_point_count, 0U);
}

TEST(MappingPointFilterTest, DropsNonFinitePoints) {
  MappingPointFilter filter;
  std::vector<Point3f> points = {
      Point3f(1.0, 0.0, 0.0),
      Point3f(std::numeric_limits<double>::quiet_NaN(), 0.0, 0.0),
  };
  MappingPointFilterStats stats;
  const auto filtered = filter.filter(points, &stats);
  EXPECT_EQ(stats.nonfinite_point_count, 1U);
  for (const auto& point : filtered) {
    EXPECT_TRUE(point.allFinite());
  }
}

TEST(MappingPointFilterTest, RangeGuardRemovesPointsOutsideBounds) {
  MappingPointFilterConfig config;
  config.voxel_size_m = 0.0;  // disable downsample to make counting exact
  config.minimum_range_m = 1.0;
  config.maximum_range_m = 5.0;
  MappingPointFilter filter(config);

  std::vector<Point3f> points = {
      Point3f(0.5, 0.0, 0.0),  // too close
      Point3f(2.0, 0.0, 0.0),  // in range
      Point3f(10.0, 0.0, 0.0),  // too far
  };
  const auto filtered = filter.filter(points);
  ASSERT_EQ(filtered.size(), 1U);
  EXPECT_NEAR(filtered.front().x(), 2.0, 1e-9);
}

TEST(MappingPointFilterTest, RangeGuardDisabledWhenBoundsAreZero) {
  MappingPointFilterConfig config;
  config.voxel_size_m = 0.0;
  config.minimum_range_m = 0.0;
  config.maximum_range_m = 0.0;
  MappingPointFilter filter(config);
  std::vector<Point3f> points = {Point3f(0.001, 0.0, 0.0),
                                 Point3f(1000.0, 0.0, 0.0)};
  EXPECT_EQ(filter.filter(points).size(), 2U);
}

}  // namespace
}  // namespace navigation_mapping
