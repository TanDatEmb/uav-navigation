#include <gtest/gtest.h>

#include <ikd_Tree.h>

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <memory>
#include <random>
#include <thread>
#include <vector>

namespace {

constexpr int kFixedNeighborCount = 5;

struct FixedQueryResult {
  std::array<ikdTree_PointType, kFixedNeighborCount> neighbors{};
  std::array<float, kFixedNeighborCount> squared_distances{};
  int count{0};
};

FixedQueryResult queryInto(
    KD_TREE<ikdTree_PointType>& tree, const ikdTree_PointType& query,
    std::array<KD_TREE<ikdTree_PointType>::PointType_CMP,
               2 * kFixedNeighborCount>& heap_storage) {
  FixedQueryResult result;
  tree.Nearest_Search_Into(
      query, kFixedNeighborCount, result.neighbors.data(),
      result.squared_distances.data(),
      static_cast<int>(result.neighbors.size()), result.count,
      heap_storage.data(), static_cast<int>(heap_storage.size()), 1000.0);
  return result;
}

void expectConcurrentQueriesMatchSerial(
    KD_TREE<ikdTree_PointType>& serial_tree,
    KD_TREE<ikdTree_PointType>& concurrent_tree,
    const std::vector<ikdTree_PointType>& queries,
    const std::function<bool()>& prepare_concurrent_readers) {
  std::vector<FixedQueryResult> serial(queries.size());
  std::array<KD_TREE<ikdTree_PointType>::PointType_CMP,
             2 * kFixedNeighborCount>
      serial_heap{};
  for (std::size_t index = 0; index < queries.size(); ++index) {
    serial[index] = queryInto(serial_tree, queries[index], serial_heap);
  }

  std::vector<FixedQueryResult> concurrent(queries.size());
  constexpr std::size_t kThreadCount = 4U;
  std::array<std::thread, kThreadCount> workers;
  std::atomic<std::size_t> ready_count{0U};
  std::atomic<bool> start{false};
  for (std::size_t worker = 0; worker < kThreadCount; ++worker) {
    workers[worker] = std::thread([&, worker] {
      std::array<KD_TREE<ikdTree_PointType>::PointType_CMP,
                 2 * kFixedNeighborCount>
          heap_storage{};
      ready_count.fetch_add(1U, std::memory_order_release);
      while (!start.load(std::memory_order_acquire)) {
        std::this_thread::yield();
      }
      for (std::size_t index = worker; index < queries.size();
           index += kThreadCount) {
        concurrent[index] =
            queryInto(concurrent_tree, queries[index], heap_storage);
      }
    });
  }
  while (ready_count.load(std::memory_order_acquire) != kThreadCount) {
    std::this_thread::yield();
  }
  const bool preparation_succeeded = prepare_concurrent_readers();
  start.store(true, std::memory_order_release);
  for (std::thread& worker : workers) {
    worker.join();
  }
  ASSERT_TRUE(preparation_succeeded);

  for (std::size_t query = 0; query < queries.size(); ++query) {
    SCOPED_TRACE(::testing::Message() << "query=" << query);
    ASSERT_EQ(concurrent[query].count, serial[query].count);
    for (int neighbor = 0; neighbor < serial[query].count; ++neighbor) {
      const std::size_t index = static_cast<std::size_t>(neighbor);
      EXPECT_FLOAT_EQ(concurrent[query].squared_distances[index],
                      serial[query].squared_distances[index]);
      EXPECT_FLOAT_EQ(concurrent[query].neighbors[index].x,
                      serial[query].neighbors[index].x);
      EXPECT_FLOAT_EQ(concurrent[query].neighbors[index].y,
                      serial[query].neighbors[index].y);
      EXPECT_FLOAT_EQ(concurrent[query].neighbors[index].z,
                      serial[query].neighbors[index].z);
    }
  }
}

std::vector<ikdTree_PointType> deterministicQueries() {
  constexpr std::uint32_t kSeed = 0xC04C0A11U;
  std::mt19937 generator(kSeed);
  std::uniform_real_distribution<float> coordinate(-40.0F, 40.0F);
  std::vector<ikdTree_PointType> queries;
  queries.reserve(10000U);
  for (std::size_t index = 0; index < 10000U; ++index) {
    queries.emplace_back(coordinate(generator), coordinate(generator),
                         coordinate(generator));
  }
  return queries;
}

KD_TREE<ikdTree_PointType>::PointVector deterministicTreePoints(
    const std::size_t count, const std::uint32_t seed) {
  std::mt19937 generator(seed);
  std::uniform_real_distribution<float> coordinate(-50.0F, 50.0F);
  KD_TREE<ikdTree_PointType>::PointVector points;
  points.reserve(count);
  for (std::size_t index = 0; index < count; ++index) {
    points.emplace_back(coordinate(generator), coordinate(generator),
                        coordinate(generator));
  }
  return points;
}

}  // namespace

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

TEST(IkdTreeUpstreamSmoke, NearestSearchMatchesBruteForceWhenOneChildCanImprove) {
  auto tree = std::make_unique<KD_TREE<ikdTree_PointType>>(
      0.3F, 0.6F, 0.05F, false);
  KD_TREE<ikdTree_PointType>::PointVector points;
  for (int x = -12; x <= 12; ++x) {
    for (int y = -8; y <= 8; ++y) {
      for (int z = -2; z <= 2; ++z) {
        points.emplace_back(static_cast<float>(x) * 1.7F,
                            static_cast<float>(y) * 1.3F,
                            static_cast<float>(z) * 2.1F);
      }
    }
  }
  tree->Build(points);

  std::mt19937 generator(0x1AD5EEDU);
  std::uniform_real_distribution<float> coordinate(-25.0F, 25.0F);
  for (int query_index = 0; query_index < 200; ++query_index) {
    const ikdTree_PointType query{coordinate(generator), coordinate(generator),
                                   coordinate(generator)};
    KD_TREE<ikdTree_PointType>::PointVector neighbors;
    std::vector<float> squared_distances;
    tree->Nearest_Search(query, kFixedNeighborCount, neighbors,
                         squared_distances, 10000.0);

    std::vector<float> expected;
    expected.reserve(points.size());
    for (const auto& point : points) {
      const float dx = point.x - query.x;
      const float dy = point.y - query.y;
      const float dz = point.z - query.z;
      expected.push_back(dx * dx + dy * dy + dz * dz);
    }
    std::sort(expected.begin(), expected.end());
    std::sort(squared_distances.begin(), squared_distances.end());
    ASSERT_EQ(squared_distances.size(),
              static_cast<std::size_t>(kFixedNeighborCount));
    for (int neighbor = 0; neighbor < kFixedNeighborCount; ++neighbor) {
      EXPECT_NEAR(squared_distances[static_cast<std::size_t>(neighbor)],
                  expected[static_cast<std::size_t>(neighbor)], 1e-3F)
          << "query=" << query_index << " neighbor=" << neighbor;
    }
  }
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

TEST(IkdTreeUpstreamSmoke, AsyncRebuildSupportsAllocationFreeNearestSearch) {
  constexpr std::uint32_t kSeed = 0xA11C0C0AU;
  constexpr int kNeighborCount = 5;
  std::mt19937 generator(kSeed);
  std::uniform_real_distribution<float> coordinate(-30.0F, 30.0F);
  auto tree = std::make_unique<KD_TREE<ikdTree_PointType>>(
      0.3F, 0.6F, 0.05F, true);

  KD_TREE<ikdTree_PointType>::PointVector initial;
  initial.reserve(3000U);
  for (std::size_t index = 0; index < 3000U; ++index) {
    initial.emplace_back(coordinate(generator), coordinate(generator), coordinate(generator));
  }
  tree->Build(initial);

  for (int operation = 0; operation < 3000; ++operation) {
    SCOPED_TRACE(::testing::Message() << "seed=" << kSeed << " operation=" << operation);
    KD_TREE<ikdTree_PointType>::PointVector addition{
        ikdTree_PointType{coordinate(generator), coordinate(generator), coordinate(generator)}};
    static_cast<void>(tree->Add_Points(addition, true));

    if (operation % 11 == 0) {
      const float slab_min = -30.0F + static_cast<float>((operation / 11) % 20);
      BoxPointType slab{};
      slab.vertex_min[0] = slab_min;
      slab.vertex_min[1] = -35.0F;
      slab.vertex_min[2] = -35.0F;
      slab.vertex_max[0] = slab_min + 0.25F;
      slab.vertex_max[1] = 35.0F;
      slab.vertex_max[2] = 35.0F;
      std::vector<BoxPointType> boxes{slab};
      static_cast<void>(tree->Delete_Point_Boxes(boxes));
    }

    std::array<ikdTree_PointType, kNeighborCount> neighbors{};
    std::array<float, kNeighborCount> squared_distances{};
    std::array<KD_TREE<ikdTree_PointType>::PointType_CMP, 2 * kNeighborCount> heap_storage{};
    int result_count = 0;
    tree->Nearest_Search_Into(
        ikdTree_PointType{coordinate(generator), coordinate(generator), coordinate(generator)},
        kNeighborCount, neighbors.data(), squared_distances.data(),
        static_cast<int>(neighbors.size()), result_count, heap_storage.data(),
        static_cast<int>(heap_storage.size()), 100.0);
    ASSERT_GE(result_count, 0);
    ASSERT_LE(result_count, kNeighborCount);
    for (int index = 0; index < result_count; ++index) {
      EXPECT_GE(squared_distances[static_cast<std::size_t>(index)], 0.0F);
    }
  }
}

TEST(IkdTreeUpstreamSmoke, ConcurrentNearestSearchIntoMatchesSerialReference) {
  auto tree = std::make_unique<KD_TREE<ikdTree_PointType>>(
      0.3F, 0.6F, 0.05F, false);
  auto points = deterministicTreePoints(12000U, 0x51A1A11U);
  tree->Build(points);

  expectConcurrentQueriesMatchSerial(*tree, *tree, deterministicQueries(),
                                     [] { return true; });
}

TEST(IkdTreeUpstreamSmoke,
     ConcurrentNearestSearchIntoMatchesSerialDuringAsyncRebuild) {
  auto tree = std::make_unique<KD_TREE<ikdTree_PointType>>(
      0.3F, 0.6F, 0.05F, true);
  auto points = deterministicTreePoints(200000U, 0xA51AC11U);
  tree->Build(points);

  auto serial_tree = std::make_unique<KD_TREE<ikdTree_PointType>>(
      0.3F, 0.6F, 0.05F, false);
  serial_tree->Build(points);

  // Delete roughly 60% of a large root in one foreground operation. This
  // exceeds the vendor deletion criterion and leaves enough valid points for
  // a measurable asynchronous root rebuild.
  BoxPointType crop{};
  crop.vertex_min[0] = -60.0F;
  crop.vertex_min[1] = -60.0F;
  crop.vertex_min[2] = -60.0F;
  crop.vertex_max[0] = 10.0F;
  crop.vertex_max[1] = 60.0F;
  crop.vertex_max[2] = 60.0F;
  std::vector<BoxPointType> boxes{crop};
  static_cast<void>(serial_tree->Delete_Point_Boxes(boxes));

  expectConcurrentQueriesMatchSerial(
      *serial_tree, *tree, deterministicQueries(), [&] {
        static_cast<void>(tree->Delete_Point_Boxes(boxes));
        // Do not release readers unless a background rebuild was actually
        // observed; merely enabling async mode is insufficient coverage for
        // the search/rebuild interaction.
        for (std::size_t attempt = 0; attempt < 2000U; ++attempt) {
          if (tree->asynchronous_rebuild_in_progress()) {
            return true;
          }
          std::this_thread::sleep_for(std::chrono::microseconds(50));
        }
        return false;
      });
}
