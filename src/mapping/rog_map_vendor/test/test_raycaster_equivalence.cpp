#include <gtest/gtest.h>

#include <array>
#include <cmath>
#include <random>
#include <utility>
#include <vector>

#include <rog_map/rog_map_core/raycaster.h>

namespace {

using Index = std::array<int, 3>;
using Ray = std::pair<Eigen::Vector3d, Eigen::Vector3d>;

std::vector<Ray> makeRays() {
  std::vector<Ray> rays = {
      {{0.1, 0.2, 0.3}, {4.1, 0.2, 0.3}},
      {{4.1, 0.2, 0.3}, {0.1, 0.2, 0.3}},
      {{0.1, 0.2, 0.3}, {2.1, 3.2, 0.3}},
      {{0.1, 0.2, 0.3}, {2.1, 0.2, 3.3}},
      {{0.1, 0.2, 0.3}, {0.1, 3.2, 3.3}},
      {{-4.1, -3.2, -2.3}, {4.1, 3.2, 2.3}},
      {{-0.001, 0.001, -0.001}, {0.001, -0.001, 0.001}},
      {{0.999999, 1.0, -1.0}, {1.000001, 1.2, -0.8}},
      {{2.0, 2.0, 2.0}, {2.1, 2.1, 2.1}},
      {{-3.0, 0.0, 0.0}, {-3.0, 0.0, 3.0}},
  };

  std::mt19937 generator(0x5E1F);  // deterministic regression corpus
  std::uniform_real_distribution<double> distribution(-8.0, 8.0);
  for (int i = 0; i < 256; ++i) {
    rays.emplace_back(
        Eigen::Vector3d(distribution(generator), distribution(generator), distribution(generator)),
        Eigen::Vector3d(distribution(generator), distribution(generator), distribution(generator)));
  }
  return rays;
}

std::vector<Index> collectLegacy(const Ray& ray) {
  rog_map::raycaster::RayCaster caster(0.2);
  std::vector<Index> sequence;
  if (!caster.setInput(ray.first, ray.second)) {
    return sequence;
  }

  Eigen::Vector3d point;
  while (caster.step(point)) {
    int x;
    int y;
    int z;
    caster.posToIndex(point.x(), x);
    caster.posToIndex(point.y(), y);
    caster.posToIndex(point.z(), z);
    sequence.push_back({x, y, z});
  }
  return sequence;
}

std::vector<Index> collectIndexed(const Ray& ray) {
  rog_map::raycaster::RayCaster caster(0.2);
  std::vector<Index> sequence;
  if (!caster.setInput(ray.first, ray.second)) {
    return sequence;
  }

  rog_map::Vec3i voxel;
  while (caster.stepIndex(voxel)) {
    sequence.push_back({voxel.x(), voxel.y(), voxel.z()});
  }
  return sequence;
}

}  // namespace

TEST(RogMapRayCaster, IndexTraversalMatchesLegacyMetricTraversal) {
  for (const auto& ray : makeRays()) {
    EXPECT_EQ(collectIndexed(ray), collectLegacy(ray))
        << "start=" << ray.first.transpose() << " end=" << ray.second.transpose();
  }
}

TEST(RogMapRayCaster, TraversalIsBoundedAndNeverOvershootsAnEndpointAxis) {
  auto rays = makeRays();
  std::mt19937 generator(0xB0ADED);
  std::uniform_real_distribution<double> distribution(-30.0, 30.0);
  for (int i = 0; i < 4096; ++i) {
    rays.emplace_back(
        Eigen::Vector3d(distribution(generator), distribution(generator), distribution(generator)),
        Eigen::Vector3d(distribution(generator), distribution(generator), distribution(generator)));
  }

  for (const auto& ray : rays) {
    rog_map::raycaster::RayCaster caster(0.2);
    int sx, sy, sz, ex, ey, ez;
    caster.posToIndex(ray.first.x(), sx);
    caster.posToIndex(ray.first.y(), sy);
    caster.posToIndex(ray.first.z(), sz);
    caster.posToIndex(ray.second.x(), ex);
    caster.posToIndex(ray.second.y(), ey);
    caster.posToIndex(ray.second.z(), ez);
    const int expected_steps = std::abs(ex - sx) + std::abs(ey - sy) + std::abs(ez - sz);

    if (!caster.setInput(ray.first, ray.second)) {
      EXPECT_EQ(expected_steps, 0);
      continue;
    }

    rog_map::Vec3i voxel;
    int steps = 0;
    while (caster.stepIndex(voxel)) {
      ++steps;
      ASSERT_LE(steps, expected_steps)
          << "start=" << ray.first.transpose() << " end=" << ray.second.transpose();
      if (sx == ex) EXPECT_EQ(voxel.x(), ex);
      if (sy == ey) EXPECT_EQ(voxel.y(), ey);
      if (sz == ez) EXPECT_EQ(voxel.z(), ez);
    }
    EXPECT_EQ(steps, expected_steps)
        << "start=" << ray.first.transpose() << " end=" << ray.second.transpose();
  }
}
