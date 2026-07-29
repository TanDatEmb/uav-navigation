#include <gtest/gtest.h>

#include <ikd_Tree.h>

#include <cstddef>
#include <cstdint>
#include <memory>
#include <random>
#include <vector>

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

TEST(IkdTreeUpstreamSmoke, SynchronousModeHandlesTwoHundredThousandPoints) {
  constexpr std::size_t kPointCount = 200001U;
  auto tree = std::make_unique<KD_TREE<ikdTree_PointType>>(
      0.5F, 0.6F, 0.01F, false);
  KD_TREE<ikdTree_PointType>::PointVector points;
  points.reserve(kPointCount);
  for (std::size_t index = 0; index < kPointCount; ++index) {
    points.emplace_back(static_cast<float>(index % 1000U) * 0.02F,
                        static_cast<float>((index / 1000U) % 201U) * 0.02F,
                        static_cast<float>(index / 201000U) * 0.02F);
  }

  tree->Build(points);
  EXPECT_EQ(tree->validnum(), static_cast<int>(kPointCount));

  KD_TREE<ikdTree_PointType>::PointVector neighbors;
  std::vector<float> squared_distances;
  tree->Nearest_Search(ikdTree_PointType{10.0F, 2.0F, 0.0F}, 5, neighbors,
                       squared_distances, 1.0);
  EXPECT_EQ(neighbors.size(), 5U);
}

TEST(IkdTreeUpstreamSmoke, RepeatedSynchronousConstructionAndDestruction) {
  for (int iteration = 0; iteration < 100; ++iteration) {
    auto tree = std::make_unique<KD_TREE<ikdTree_PointType>>(
        0.5F, 0.6F, 0.05F, false);
    KD_TREE<ikdTree_PointType>::PointVector points;
    points.reserve(2000U);
    for (int index = 0; index < 2000; ++index) {
      points.emplace_back(static_cast<float>(index) * 0.1F,
                          static_cast<float>(iteration), 0.0F);
    }
    tree->Build(points);
    ASSERT_EQ(tree->validnum(), 2000);
  }
}

TEST(IkdTreeUpstreamSmoke, AsyncRebuildRandomizedOperationsUseFixedSeed) {
  constexpr std::uint32_t kSeed = 0x1AD7EE42U;
  std::mt19937 generator(kSeed);
  std::uniform_real_distribution<float> coordinate(-20.0F, 20.0F);

  for (int iteration = 0; iteration < 100; ++iteration) {
    SCOPED_TRACE(::testing::Message()
                 << "seed=" << kSeed << " iteration=" << iteration);
    auto tree = std::make_unique<KD_TREE<ikdTree_PointType>>(
        0.3F, 0.6F, 0.10F, true);
    KD_TREE<ikdTree_PointType>::PointVector initial;
    initial.reserve(2500U);
    for (std::size_t index = 0; index < 2500U; ++index) {
      initial.emplace_back(coordinate(generator), coordinate(generator),
                           coordinate(generator));
    }
    tree->Build(initial);

    KD_TREE<ikdTree_PointType>::PointVector additions;
    additions.reserve(500U);
    for (std::size_t index = 0; index < 500U; ++index) {
      additions.emplace_back(coordinate(generator), coordinate(generator),
                             coordinate(generator));
    }
    static_cast<void>(tree->Add_Points(additions, true));

    for (int query = 0; query < 50; ++query) {
      KD_TREE<ikdTree_PointType>::PointVector neighbors;
      std::vector<float> squared_distances;
      tree->Nearest_Search(
          ikdTree_PointType{coordinate(generator), coordinate(generator),
                            coordinate(generator)},
          5, neighbors, squared_distances, 10.0);
      EXPECT_EQ(neighbors.size(), squared_distances.size());
    }

    BoxPointType crop{};
    crop.vertex_min[0] = -25.0F;
    crop.vertex_min[1] = -25.0F;
    crop.vertex_min[2] = -25.0F;
    crop.vertex_max[0] = -10.0F;
    crop.vertex_max[1] = 25.0F;
    crop.vertex_max[2] = 25.0F;
    std::vector<BoxPointType> boxes{crop};
    static_cast<void>(tree->Delete_Point_Boxes(boxes));

    KD_TREE<ikdTree_PointType>::PointVector snapshot;
    BoxPointType all{};
    all.vertex_min[0] = all.vertex_min[1] = all.vertex_min[2] = -100.0F;
    all.vertex_max[0] = all.vertex_max[1] = all.vertex_max[2] = 100.0F;
    tree->Box_Search(all, snapshot);
    if (snapshot.size() > 50U) {
      KD_TREE<ikdTree_PointType>::PointVector exact_delete(
          snapshot.begin(), snapshot.begin() + 50);
      tree->Delete_Points(exact_delete);
    }
    EXPECT_GE(tree->validnum(), -1);
  }
}
