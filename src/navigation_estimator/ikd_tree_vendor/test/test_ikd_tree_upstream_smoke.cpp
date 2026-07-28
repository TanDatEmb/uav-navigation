#include <gtest/gtest.h>

#include <ikd_Tree.h>

TEST(IkdTreeUpstreamSmoke, BuildsAndQueriesActualKdTree) {
  auto tree = std::make_unique<KD_TREE<ikdTree_PointType>>();
  KD_TREE<ikdTree_PointType>::PointVector points{
      ikdTree_PointType{0.0F, 0.0F, 0.0F},
      ikdTree_PointType{1.0F, 0.0F, 0.0F},
      ikdTree_PointType{0.0F, 1.0F, 0.0F}};
  tree->Build(points);

  KD_TREE<ikdTree_PointType>::PointVector neighbors;
  std::vector<float> squared_distances;
  tree->Nearest_Search(ikdTree_PointType{0.1F, 0.0F, 0.0F}, 1, neighbors,
                       squared_distances, 1.0);

  ASSERT_EQ(neighbors.size(), 1U);
  ASSERT_EQ(squared_distances.size(), 1U);
  EXPECT_NEAR(neighbors.front().x, 0.0F, 1e-6F);
}
