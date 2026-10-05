#include <gtest/gtest.h>

#include <Eigen/Core>
#include <vector>

#include "fast_lio_core/registration/plane_estimator.hpp"

namespace uav::nav::lio {
namespace {

TEST(PlaneEstimatorTest, FitsDeterministicPlanarNeighborhood) {
  PlaneEstimator estimator;
  const std::vector<Eigen::Vector3d> points{
      {-1.0, -1.0, 2.0}, {0.0, -1.0, 2.0}, {1.0, -1.0, 2.0},
      {-1.0, 1.0, 2.0},  {0.0, 1.0, 2.0},  {1.0, 1.0, 2.0},
  };

  const std::optional<Plane> plane = estimator.estimate(points);

  ASSERT_TRUE(plane.has_value());
  EXPECT_NEAR(plane->centroid_odom_m.z(), 2.0, 1e-12);
  EXPECT_NEAR(plane->normal_odom.x(), 0.0, 1e-12);
  EXPECT_NEAR(plane->normal_odom.y(), 0.0, 1e-12);
  EXPECT_NEAR(plane->normal_odom.z(), 1.0, 1e-12);
  EXPECT_NEAR(plane->rms_error_m, 0.0, 1e-12);
  EXPECT_GT(plane->planarity, 0.99);
}

TEST(PlaneEstimatorTest, RejectsCollinearNeighborhood) {
  PlaneEstimator estimator;
  const std::vector<Eigen::Vector3d> points{
      {-2.0, 0.0, 0.0}, {-1.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {1.0, 0.0, 0.0}, {2.0, 0.0, 0.0},
  };

  EXPECT_FALSE(estimator.estimate(points).has_value());
}

TEST(PlaneEstimatorTest, RejectsNeighborhoodWithLargePlaneError) {
  PlaneEstimatorConfig config;
  config.maximum_rms_error_m = 0.02;
  config.maximum_point_error_m = 0.04;
  PlaneEstimator estimator(config);
  const std::vector<Eigen::Vector3d> points{
      {-1.0, -1.0, 0.0}, {1.0, -1.0, 0.0}, {-1.0, 1.0, 0.0}, {1.0, 1.0, 0.0}, {0.0, 0.0, 0.3},
  };

  EXPECT_FALSE(estimator.estimate(points).has_value());
}

}  // namespace
}  // namespace uav::nav::lio
