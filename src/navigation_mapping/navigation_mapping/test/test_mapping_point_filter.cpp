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

  std::vector<Eigen::Vector3d> points;
  for (int i = 0; i < 100; ++i) {
    points.emplace_back(0.01 * i, 0.0, 0.0);  // all inside voxel [0,1)
  }
  MappingPointFilterStats stats;
  const auto filtered = filter.filter(points, &stats);
  EXPECT_EQ(filtered.size(), 1U);
  EXPECT_EQ(stats.input_point_count, 100U);
  EXPECT_EQ(stats.output_point_count, 1U);
}

TEST(MappingPointFilterTest, DropsNonFinitePoints) {
  MappingPointFilter filter;
  std::vector<Eigen::Vector3d> points = {
      Eigen::Vector3d(1.0, 0.0, 0.0),
      Eigen::Vector3d(std::numeric_limits<double>::quiet_NaN(), 0.0, 0.0),
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

  std::vector<Eigen::Vector3d> points = {
      Eigen::Vector3d(0.5, 0.0, 0.0),  // too close
      Eigen::Vector3d(2.0, 0.0, 0.0),  // in range
      Eigen::Vector3d(10.0, 0.0, 0.0),  // too far
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
  std::vector<Eigen::Vector3d> points = {Eigen::Vector3d(0.001, 0.0, 0.0),
                                         Eigen::Vector3d(1000.0, 0.0, 0.0)};
  EXPECT_EQ(filter.filter(points).size(), 2U);
}

}  // namespace
}  // namespace navigation_mapping
