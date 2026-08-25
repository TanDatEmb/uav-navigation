#include <planner_runtime_context/planner_runtime_context.hpp>

#include <gtest/gtest.h>

namespace navigation_runtime {
namespace {

TEST(PlannerRuntimeContext, ReadsTheCurrentRuntimeClockOnEverySample) {
  double now_s = 10.0;
  navigation_planner_context::PlannerRuntimeContext clock{[&now_s]() { return now_s; }};

  EXPECT_DOUBLE_EQ(clock.getSimTime(), 10.0);
  now_s = 10.02;
  EXPECT_DOUBLE_EQ(clock.getSimTime(), 10.02);
}

}  // namespace
}  // namespace navigation_runtime
