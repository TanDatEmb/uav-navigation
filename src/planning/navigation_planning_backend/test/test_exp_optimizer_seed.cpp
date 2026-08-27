#include <gtest/gtest.h>

#include <cmath>
#include <memory>
#include <utility>

#include <traj_opt/config.hpp>
#include <traj_opt/nominal_trajectory_optimizer.hpp>

namespace {

navigation_math::StatePVAJ makePositionState(const double x) {
  navigation_math::StatePVAJ state = navigation_math::StatePVAJ::Zero();
  state.col(0) << x, 0.0, 1.0;
  return state;
}

geometry_utils::Polytope makeConvexBox() {
  navigation_math::MatD4f planes(6, 4);
  planes <<
      1.0, 0.0, 0.0, -10.0,
     -1.0, 0.0, 0.0, -10.0,
      0.0, 1.0, 0.0, -10.0,
      0.0,-1.0, 0.0, -10.0,
      0.0, 0.0, 1.0, -10.0,
      0.0, 0.0,-1.0,   0.0;
  return geometry_utils::Polytope(std::move(planes));
}

TEST(ExpOptimizer, GuideTimeIsTheInitialDurationSeed) {
  const traj_opt::Config config(PLANNER_EXP_CONFIG_PATH, "exp_traj");
  const auto planner_context =
      std::make_shared<navigation_planner_context::PlannerRuntimeContext>(
          [] { return 12.0; });
  traj_opt::ExpTrajOpt optimizer(config, planner_context);

  const auto head = makePositionState(0.0);
  const auto tail = makePositionState(4.0);
  navigation_math::vec_E<navigation_math::Vec3f> guide_path;
  guide_path.emplace_back(head.col(0));
  guide_path.emplace_back(tail.col(0));
  const std::vector<double> guide_times{0.0, 2.0};
  geometry_utils::PolytopeVec corridors{makeConvexBox()};
  geometry_utils::Trajectory trajectory;

  ASSERT_TRUE(optimizer.optimize(
      head, tail, guide_path, guide_times, corridors, trajectory));
  const auto diagnostics = optimizer.diagnostics();
  ASSERT_TRUE(diagnostics.valid);
  EXPECT_DOUBLE_EQ(diagnostics.initial_duration_s, guide_times.back());
  EXPECT_TRUE(std::isfinite(diagnostics.final_duration_s));
  EXPECT_FALSE(trajectory.empty());
}

}  // namespace
