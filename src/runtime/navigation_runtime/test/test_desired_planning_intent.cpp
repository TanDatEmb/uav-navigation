#include <gtest/gtest.h>

#include <array>
#include <cstdint>

#include <navigation_runtime/desired_planning_intent.hpp>

namespace navigation_runtime {
namespace {

using Goal = navigation_contracts::msg::NavigationGoal;

Goal makeGoal(
    const char* mission_id, const std::uint32_t waypoint_index,
    const std::uint64_t request_id, const std::uint64_t route_revision = 1U) {
  Goal goal;
  goal.mission_id = mission_id;
  goal.waypoint_index = waypoint_index;
  goal.request_id = request_id;
  goal.route.route_revision = route_revision;
  return goal;
}

TEST(DesiredPlanningIntent, InstallsAndConsumesNewIntent) {
  DesiredPlanningIntent intent;
  const auto goal = makeGoal("mission-a", 2U, 17U);

  EXPECT_FALSE(intent.goal().has_value());
  EXPECT_EQ(intent.revision(), 0U);
  EXPECT_FALSE(intent.hasTransition());

  ASSERT_EQ(intent.advanceRevision(), 1U);
  intent.install(goal, PlanningIntentTransition::kNewIntent);

  ASSERT_TRUE(intent.goal().has_value());
  EXPECT_EQ(intent.goal()->mission_id, goal.mission_id);
  EXPECT_TRUE(intent.isNewIntent());
  EXPECT_FALSE(intent.isHotRetarget());
  EXPECT_TRUE(intent.hasTransition());
  EXPECT_TRUE(intent.consumeNewIntent());
  EXPECT_FALSE(intent.consumeNewIntent());
  EXPECT_FALSE(intent.hasTransition());
}

TEST(DesiredPlanningIntent, InstallsAndConsumesHotRetarget) {
  DesiredPlanningIntent intent;
  const auto goal = makeGoal("mission-a", 3U, 18U);

  ASSERT_EQ(intent.advanceRevision(), 1U);
  intent.install(goal, PlanningIntentTransition::kHotRetarget);

  EXPECT_FALSE(intent.isNewIntent());
  EXPECT_TRUE(intent.isHotRetarget());
  EXPECT_FALSE(intent.consumeNewIntent());
  EXPECT_TRUE(intent.isHotRetarget());
  EXPECT_TRUE(intent.consumeHotRetarget());
  EXPECT_FALSE(intent.consumeHotRetarget());
  EXPECT_FALSE(intent.hasTransition());
}

TEST(DesiredPlanningIntent, TransitionEnumCannotRepresentBothFlags) {
  constexpr std::array transitions{
      PlanningIntentTransition::kNone,
      PlanningIntentTransition::kNewIntent,
      PlanningIntentTransition::kHotRetarget};

  for (const auto transition : transitions) {
    DesiredPlanningIntent intent;
    intent.install(makeGoal("mission-a", 0U, 1U), transition);
    EXPECT_FALSE(intent.isNewIntent() && intent.isHotRetarget());
  }
}

TEST(DesiredPlanningIntent, ReplaceGoalPreservesTransitionAndResetRearms) {
  DesiredPlanningIntent intent;
  const auto initial = makeGoal("mission-a", 1U, 10U);
  const auto updated_payload = makeGoal("mission-a", 1U, 10U, 2U);

  ASSERT_EQ(intent.advanceRevision(), 1U);
  intent.install(initial, PlanningIntentTransition::kHotRetarget);
  intent.install(updated_payload);
  EXPECT_TRUE(intent.isHotRetarget());
  EXPECT_EQ(intent.goal()->route.route_revision, 2U);

  intent.rearmAfterLocalizationReset();
  EXPECT_TRUE(intent.isNewIntent());
  EXPECT_FALSE(intent.isHotRetarget());

  intent.clearGoal();
  EXPECT_FALSE(intent.goal().has_value());
  EXPECT_FALSE(intent.hasTransition());
  intent.rearmAfterLocalizationReset();
  EXPECT_FALSE(intent.hasTransition());
}

TEST(DesiredPlanningIntent, RevisionIsMonotonicAndRejectsStaleIdentity) {
  DesiredPlanningIntent intent;
  const auto current = makeGoal("mission-a", 3U, 29U);
  const auto stale_goal = makeGoal("mission-a", 2U, 28U);

  ASSERT_EQ(intent.advanceRevision(), 1U);
  intent.install(current, PlanningIntentTransition::kNewIntent);
  ASSERT_EQ(intent.advanceRevision(), 2U);

  EXPECT_EQ(intent.revision(), 2U);
  EXPECT_TRUE(intent.matches(current, 2U));
  EXPECT_FALSE(intent.matches(current, 1U));
  EXPECT_FALSE(intent.matches(stale_goal, 2U));
}

}  // namespace
}  // namespace navigation_runtime
