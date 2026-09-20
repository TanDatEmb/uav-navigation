#include <planner_runtime_context/planner_runtime_context.hpp>
#include <path_search/astar.h>

#include <cmath>
#include <gtest/gtest.h>

namespace navigation_planning_backend {
namespace {

TEST(PlannerRuntimeContext, ReadsTheCurrentRuntimeClockOnEverySample) {
  double now_s = 10.0;
  navigation_planner_context::PlannerRuntimeContext clock{
      [&now_s]() { return now_s; }};

  EXPECT_DOUBLE_EQ(clock.getSimTime(), 10.0);
  now_s = 10.02;
  EXPECT_DOUBLE_EQ(clock.getSimTime(), 10.02);
}

TEST(Astar, GridNodeDefaultsUseInfiniteScores) {
  const path_search::GridNode node;
  EXPECT_TRUE(std::isinf(node.total_score));
  EXPECT_TRUE(std::isinf(node.distance_score));
}

TEST(Astar, OpenSetEntriesKeepEnqueuedScore) {
  path_search::GridNode first;
  path_search::GridNode second;
  first.total_score = 10.0;
  second.total_score = 20.0;

  using OpenSet = std::priority_queue<path_search::OpenSetEntry,
                                      std::vector<path_search::OpenSetEntry>,
                                      path_search::OpenSetComparator>;
  OpenSet open_set;
  open_set.push({&first, first.total_score, 1});
  first.total_score = 30.0;
  open_set.push({&second, second.total_score, 1});

  ASSERT_FALSE(open_set.empty());
  EXPECT_EQ(open_set.top().node, &first);
  EXPECT_DOUBLE_EQ(open_set.top().total_score, 10.0);
  open_set.pop();
  ASSERT_FALSE(open_set.empty());
  EXPECT_EQ(open_set.top().node, &second);
  EXPECT_DOUBLE_EQ(open_set.top().total_score, 20.0);
}

TEST(Astar, FrontierEntriesKeepEnqueuedScoreAndTieOrder) {
  path_search::GridNode first;
  path_search::GridNode second;
  first.distance_to_goal = 10.0;
  second.distance_to_goal = 20.0;

  using Frontier = std::priority_queue<path_search::FrontierEntry,
                                       std::vector<path_search::FrontierEntry>,
                                       path_search::FrontierComparator>;
  Frontier frontier;
  frontier.push({&first, first.distance_to_goal, 1U});
  first.distance_to_goal = 30.0;
  frontier.push({&second, second.distance_to_goal, 2U});

  ASSERT_FALSE(frontier.empty());
  EXPECT_EQ(frontier.top().node, &first);
  EXPECT_DOUBLE_EQ(frontier.top().distance_to_goal, 10.0);
  frontier.pop();
  ASSERT_FALSE(frontier.empty());
  EXPECT_EQ(frontier.top().node, &second);

  path_search::GridNode tied;
  frontier.push({&tied, 20.0, 0U});
  EXPECT_EQ(frontier.top().node, &tied);
}

}  // namespace
}  // namespace navigation_planning_backend
