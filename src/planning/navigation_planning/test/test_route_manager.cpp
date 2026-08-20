#include <gtest/gtest.h>

#include "navigation_planning/route_manager.hpp"

namespace navigation_planning {
namespace {

using Vec3 = navigation_mapping::Vec3;

PolylineRoute straightRoute() {
  return *RouteManager::build({Vec3{0.0, 0.0, 0.0}, Vec3{10.0, 0.0, 0.0},
                               Vec3{20.0, 0.0, 0.0}});
}

TEST(RouteManager, ProjectsPositionAndReturnsTangent) {
  const auto route = straightRoute();
  const auto result = RouteManager::project(route, Vec3{4.0, 2.0, 0.0});
  ASSERT_TRUE(result.success);
  EXPECT_NEAR(result.arc_length_m, 4.0, 1e-9);
  EXPECT_NEAR(result.distance_m, 2.0, 1e-9);
  EXPECT_TRUE(result.tangent.isApprox(Vec3{1.0, 0.0, 0.0}, 1e-9));
}

TEST(RouteManager, RejectsBackwardProjection) {
  const auto route = straightRoute();
  const auto result = RouteManager::project(route, Vec3{4.0, 2.0, 0.0}, 8.0);
  EXPECT_FALSE(result.success);
}

TEST(RouteManager, SamplesAndTrimsConsumedPrefix) {
  const auto route = straightRoute();
  const auto sample = RouteManager::sample(route, 12.0);
  ASSERT_TRUE(sample.has_value());
  EXPECT_TRUE(sample->isApprox(Vec3{12.0, 0.0, 0.0}, 1e-9));

  const auto trimmed = RouteManager::trimConsumedPrefix(route, 12.0);
  ASSERT_TRUE(trimmed.has_value());
  ASSERT_TRUE(trimmed->valid());
  EXPECT_TRUE(trimmed->points.front().isApprox(Vec3{12.0, 0.0, 0.0}, 1e-9));
  EXPECT_NEAR(trimmed->length_m, 8.0, 1e-9);
}

TEST(RouteManager, RemovesDuplicatePointsAndRejectsNonFiniteInput) {
  const auto route = RouteManager::build({Vec3{0.0, 0.0, 0.0}, Vec3{0.0, 0.0, 0.0},
                                           Vec3{2.0, 0.0, 0.0}});
  ASSERT_TRUE(route.has_value());
  EXPECT_EQ(route->points.size(), 2U);
  EXPECT_FALSE(RouteManager::build({Vec3{0.0, 0.0, 0.0},
                                    Vec3{std::numeric_limits<double>::quiet_NaN(), 0.0, 0.0}}));
}

}  // namespace
}  // namespace navigation_planning
