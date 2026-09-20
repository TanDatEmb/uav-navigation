#include <cmath>
#include <limits>
#include <utility>

#include <gtest/gtest.h>

#include "data_structure/base/polytope.h"
#include "utils/geometry/geometry_utils.h"

namespace {

geometry_utils::Polytope makeBox(
    const double min_x, const double max_x,
    const double min_y = 0.0, const double max_y = 1.0,
    const double min_z = 0.0, const double max_z = 1.0) {
  navigation_math::MatD4f planes(6, 4);
  planes <<
      1.0, 0.0, 0.0, -max_x,
     -1.0, 0.0, 0.0,  min_x,
      0.0, 1.0, 0.0, -max_y,
      0.0,-1.0, 0.0,  min_y,
      0.0, 0.0, 1.0, -max_z,
      0.0, 0.0,-1.0,  min_z;
  return geometry_utils::Polytope(std::move(planes));
}

bool consumerTransitionRepresentable(
    const geometry_utils::Polytope& first,
    const geometry_utils::Polytope& second) {
  const auto overlap = first.CrossWith(second);
  Eigen::Vector3d interior;
  const double radius = geometry_utils::findInteriorDist(
      overlap.GetPlanes(), interior) / 2.0;
  if (!std::isfinite(radius) || radius < 0.0) return false;

  Eigen::Matrix3Xd vertices(3, 0);
  geometry_utils::enumerateVs(overlap.GetPlanes(), interior, vertices);
  return vertices.cols() > 0 && vertices.allFinite();
}

void expectConsumerDomain(const geometry_utils::PolytopeVec& corridors) {
  for (std::size_t index = 0; index + 1U < corridors.size(); ++index) {
    EXPECT_TRUE(consumerTransitionRepresentable(
        corridors[index], corridors[index + 1U]))
        << "transition index=" << index;
  }
}

geometry_utils::Polytope makeRedundantVerticalBox(
    const double min_x, const double max_x,
    const double min_z, const double max_z) {
  navigation_math::MatD4f planes(8, 4);
  planes <<
      1.0, 0.0, 0.0, -max_x,
      0.0, 1.0, 0.0, -6.0,
      0.0, 0.0, 1.0, -4.5,
     -1.0, 0.0, 0.0,  min_x,
      0.0,-1.0, 0.0,  3.0,
      0.0, 0.0,-1.0,  min_z,
      0.0, 0.0, 1.0, -max_z,
      0.0, 0.0,-1.0,  2.5;
  return geometry_utils::Polytope(std::move(planes));
}

void expectFaceOnlyInputIsRetained(
    geometry_utils::PolytopeVec corridors,
    const navigation_math::Vec3f& head = {0.5, 0.5, 0.5},
    const navigation_math::Vec3f& tail = {5.5, 0.5, 0.5}) {
  ASSERT_TRUE(consumerTransitionRepresentable(corridors[0], corridors[1]));
  ASSERT_TRUE(consumerTransitionRepresentable(corridors[1], corridors[2]));
  ASSERT_FALSE(consumerTransitionRepresentable(corridors[0], corridors[2]));

  ASSERT_TRUE(geometry_utils::SimplifySFC(head, tail, corridors));
  EXPECT_EQ(corridors.size(), 3U);
  expectConsumerDomain(corridors);
}

TEST(SimplifySfcConsumerContract,
     FaceOnlyNewAdjacencyMustNotBeCreated) {
  expectFaceOnlyInputIsRetained({
      makeRedundantVerticalBox(28.200000762939453, 34.0, 1.0, 4.0),
      makeRedundantVerticalBox(31.0, 37.0, 1.0, 4.0),
      makeRedundantVerticalBox(34.0, 40.0, 1.0, 4.0)},
      navigation_math::Vec3f{29.0, 4.5, 3.0},
      navigation_math::Vec3f{39.0, 4.5, 3.0});
}

TEST(SimplifySfcConsumerContract,
     RobustNewAdjacencyMayRemoveIntermediateCorridor) {
  geometry_utils::PolytopeVec corridors{
      makeBox(0.0, 4.0), makeBox(1.0, 3.0), makeBox(2.0, 6.0)};
  ASSERT_TRUE(consumerTransitionRepresentable(corridors[0], corridors[1]));
  ASSERT_TRUE(consumerTransitionRepresentable(corridors[1], corridors[2]));
  ASSERT_TRUE(consumerTransitionRepresentable(corridors[0], corridors[2]));

  ASSERT_TRUE(geometry_utils::SimplifySFC(
      navigation_math::Vec3f{0.5, 0.5, 0.5},
      navigation_math::Vec3f{5.5, 0.5, 0.5}, corridors));
  ASSERT_EQ(corridors.size(), 2U);
  expectConsumerDomain(corridors);
}

TEST(SimplifySfcConsumerContract,
     DisconnectedNewAdjacencyPreservesExistingSafeBehavior) {
  geometry_utils::PolytopeVec corridors{
      makeBox(0.0, 2.0), makeBox(1.0, 4.0), makeBox(3.0, 7.0)};
  ASSERT_TRUE(consumerTransitionRepresentable(corridors[0], corridors[1]));
  ASSERT_TRUE(consumerTransitionRepresentable(corridors[1], corridors[2]));
  ASSERT_FALSE(consumerTransitionRepresentable(corridors[0], corridors[2]));

  ASSERT_TRUE(geometry_utils::SimplifySFC(
      navigation_math::Vec3f{0.5, 0.5, 0.5},
      navigation_math::Vec3f{6.5, 0.5, 0.5}, corridors));
  ASSERT_EQ(corridors.size(), 3U);
  expectConsumerDomain(corridors);
}

TEST(SimplifySfcConsumerContract,
     ReconstructedHistoricalTopologyStaysInConsumerDomain) {
  expectFaceOnlyInputIsRetained({
      makeRedundantVerticalBox(18.200000762939453, 24.0,
                               1.2000000476837158, 3.2000000000000002),
      makeRedundantVerticalBox(21.0, 27.0, 1.5, 3.2000000000000002),
      makeRedundantVerticalBox(24.0, 30.0, 1.5, 3.2000000000000002)},
      navigation_math::Vec3f{19.0, 4.5, 2.75},
      navigation_math::Vec3f{29.0, 4.5, 2.75});
}

}  // namespace
