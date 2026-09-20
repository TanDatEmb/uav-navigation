#include <gtest/gtest.h>

#include <limits>

#include "fast_lio_core/preprocessing/point_filter.hpp"

namespace uav::nav::lio {
namespace {

TEST(PointFilter, UsesStableRangeComparisonForLargeFiniteLimits) {
  PointFilterConfig config;
  config.minimum_range_m = 1.0e20;
  config.maximum_range_m = std::numeric_limits<double>::max();
  const PointFilter filter(config);

  LidarPoint point;
  point.position_lidar_m = Eigen::Vector3f{1.0e20F, 1.0e20F, 1.0e20F};
  EXPECT_TRUE(filter.accepts(point));
}

TEST(PointFilter, RejectsNonFiniteDerivedRange) {
  PointFilterConfig config;
  config.minimum_range_m = 0.0;
  config.maximum_range_m = std::numeric_limits<double>::max();
  const PointFilter filter(config);

  LidarPoint point;
  point.position_lidar_m = Eigen::Vector3f{
      std::numeric_limits<float>::max(),
      std::numeric_limits<float>::max(),
      std::numeric_limits<float>::max()};
  EXPECT_TRUE(filter.accepts(point));

  point.position_lidar_m.x() = std::numeric_limits<float>::infinity();
  EXPECT_FALSE(filter.accepts(point));
}

}  // namespace
}  // namespace uav::nav::lio
