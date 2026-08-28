#include <gtest/gtest.h>

#include <cmath>
#include <memory>
#include <utility>

#include <planner_core/corridor_plane_validation.hpp>
#include <planner_core/deterministic_nominal_seed.hpp>
#include <planner_core/kinematic_state_boundary.hpp>
#include <traj_opt/config.hpp>
#include <traj_opt/nominal_trajectory_optimizer.hpp>

namespace {

navigation_math::StatePVAJ makePositionState(const double x) {
  navigation_math::StatePVAJ state = navigation_math::StatePVAJ::Zero();
  state.col(0) << x, 0.0, 1.0;
  return state;
}

navigation_math::StatePVAJ makeMovingPositionState(const double x,
                                                    const double velocity_x) {
  auto state = makePositionState(x);
  state.col(1) << velocity_x, 0.0, 0.0;
  return state;
}

geometry_utils::Polytope makeBox(const double min_x, const double max_x,
                                 const double min_y, const double max_y,
                                 const double min_z, const double max_z) {
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

geometry_utils::Polytope makeConvexBox() {
  return makeBox(-10.0, 10.0, -10.0, 10.0, 0.0, 10.0);
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

TEST(ExpOptimizer, HighSpeedCorridorSolveKeepsContinuousCertificate) {
  auto config = traj_opt::Config(PLANNER_EXP_CONFIG_PATH, "exp_traj");
  // This fixture intentionally starts at the exact physical velocity cap;
  // keep its search envelope at the cap while testing corridor/flatness
  // certification rather than interior reserve conditioning.
  config.optimization_dynamic_reserve_ratio = 1.0;
  config.max_vel = 8.0;
  config.max_acc = 2.0;
  config.max_jerk = 4.0;
  const auto planner_context =
      std::make_shared<navigation_planner_context::PlannerRuntimeContext>(
          [] { return 12.0; });
  traj_opt::ExpTrajOpt optimizer(config, planner_context);

  const auto head = makeMovingPositionState(0.0, 8.0);
  const auto tail = makePositionState(30.0);
  navigation_math::vec_E<navigation_math::Vec3f> guide_path;
  guide_path.emplace_back(head.col(0));
  guide_path.emplace_back(navigation_math::Vec3f(15.0, 0.0, 1.0));
  guide_path.emplace_back(tail.col(0));
  const std::vector<double> guide_times{0.0, 2.0, 5.0};

  navigation_math::MatD4f planes(6, 4);
  planes <<
     1.0, 0.0, 0.0, -31.0,
     -1.0, 0.0, 0.0, -1.0,
      0.0, 1.0, 0.0, -2.0,
      0.0,-1.0, 0.0, -2.0,
      0.0, 0.0, 1.0, -3.0,
      0.0, 0.0,-1.0,  0.0;
  geometry_utils::PolytopeVec corridors{
      geometry_utils::Polytope(std::move(planes))};
  geometry_utils::Trajectory trajectory;

  ASSERT_TRUE(optimizer.optimize(
      head, tail, guide_path, guide_times, corridors, trajectory));
  ASSERT_FALSE(trajectory.empty());
  EXPECT_EQ(config.pos_constraint_type, traj_opt::CORRIDOR);
  EXPECT_LE(
      navigation_planning_backend::maximumContinuousCorridorPlaneViolation(
          trajectory, corridors.front().GetPlanes()),
      config.corridor_plane_tolerance_m);
}

TEST(ExpOptimizer, PositiveJerkPenaltyKeepsHighSpeedCertificate) {
  auto config = traj_opt::Config(PLANNER_EXP_CONFIG_PATH, "exp_traj");
  // Characterize a product-style positive jerk objective without changing
  // the shipped default.  The analytic extrema gate below remains the sole
  // authority for acceptance.
  config.optimization_dynamic_reserve_ratio = 1.0;
  config.max_vel = 8.0;
  config.max_acc = 2.0;
  config.max_jerk = 4.0;
  config.jerk_penalty_weight = 1.0e5;
  const auto planner_context =
      std::make_shared<navigation_planner_context::PlannerRuntimeContext>(
          [] { return 12.0; });
  traj_opt::ExpTrajOpt optimizer(config, planner_context);

  const auto head = makeMovingPositionState(0.0, 8.0);
  const auto tail = makePositionState(30.0);
  navigation_math::vec_E<navigation_math::Vec3f> guide_path;
  guide_path.emplace_back(head.col(0));
  guide_path.emplace_back(navigation_math::Vec3f(15.0, 0.0, 1.0));
  guide_path.emplace_back(tail.col(0));
  const std::vector<double> guide_times{0.0, 2.0, 5.0};

  geometry_utils::PolytopeVec corridors{makeBox(-1.0, 31.0, -2.0, 2.0,
                                                  0.0, 3.0)};
  geometry_utils::Trajectory trajectory;

  ASSERT_TRUE(optimizer.optimize(
      head, tail, guide_path, guide_times, corridors, trajectory));
  ASSERT_FALSE(trajectory.empty());
  EXPECT_LE(trajectory.getMaxVelRate(), config.max_vel);
  EXPECT_LE(trajectory.getMaxAccRate(), config.max_acc);
  EXPECT_LE(trajectory.getMaxJerRate(), config.max_jerk);
  EXPECT_LE(
      navigation_planning_backend::maximumContinuousCorridorPlaneViolation(
          trajectory, corridors.front().GetPlanes()),
      config.corridor_plane_tolerance_m);
}

TEST(ExpOptimizer, HighSpeedMultiCorridorSolveKeepsEachPieceCertified) {
  auto config = traj_opt::Config(PLANNER_EXP_CONFIG_PATH, "exp_traj");
  // This fixture intentionally starts at the exact physical velocity cap;
  // keep its search envelope at the cap while testing corridor certification.
  config.optimization_dynamic_reserve_ratio = 1.0;
  config.max_vel = 8.0;
  config.max_acc = 2.0;
  config.max_jerk = 4.0;
  const auto planner_context =
      std::make_shared<navigation_planner_context::PlannerRuntimeContext>(
          [] { return 12.0; });
  traj_opt::ExpTrajOpt optimizer(config, planner_context);

  const auto head = makeMovingPositionState(0.0, 8.0);
  const auto tail = makePositionState(30.0);
  navigation_math::vec_E<navigation_math::Vec3f> guide_path;
  guide_path.emplace_back(head.col(0));
  guide_path.emplace_back(navigation_math::Vec3f(10.0, 0.0, 1.0));
  guide_path.emplace_back(navigation_math::Vec3f(20.0, 0.0, 1.0));
  guide_path.emplace_back(tail.col(0));
  const std::vector<double> guide_times{0.0, 1.8, 3.6, 5.6};

  geometry_utils::PolytopeVec corridors{
      makeBox(-1.0, 12.0, -2.0, 2.0, 0.0, 3.0),
      makeBox(8.0, 22.0, -2.0, 2.0, 0.0, 3.0),
      makeBox(18.0, 31.0, -2.0, 2.0, 0.0, 3.0)};
  geometry_utils::Trajectory trajectory;

  ASSERT_TRUE(optimizer.optimize(
      head, tail, guide_path, guide_times, corridors, trajectory));
  ASSERT_EQ(trajectory.getPieceNum(), corridors.size());
  ASSERT_FALSE(trajectory.empty());
  for (std::size_t piece = 0; piece < corridors.size(); ++piece) {
    EXPECT_LE(
        navigation_planning_backend::maximumContinuousCorridorPlaneViolation(
            trajectory[static_cast<int>(piece)], corridors[piece].GetPlanes()),
        config.corridor_plane_tolerance_m);
  }
}

TEST(ExpOptimizer, HighSpeedDetourCorridorSolveKeepsObstacleBypassCertified) {
  auto config = traj_opt::Config(PLANNER_EXP_CONFIG_PATH, "exp_traj");
  // This fixture intentionally starts at the exact physical velocity cap;
  // keep its search envelope at the cap while testing obstacle-bypass
  // certification.
  config.optimization_dynamic_reserve_ratio = 1.0;
  config.max_vel = 8.0;
  config.max_acc = 2.0;
  config.max_jerk = 4.0;
  const auto planner_context =
      std::make_shared<navigation_planner_context::PlannerRuntimeContext>(
          [] { return 12.0; });
  traj_opt::ExpTrajOpt optimizer(config, planner_context);

  const auto head = makeMovingPositionState(0.0, 8.0);
  const auto tail = makePositionState(70.0);
  navigation_math::vec_E<navigation_math::Vec3f> guide_path;
  guide_path.emplace_back(head.col(0));
  guide_path.emplace_back(navigation_math::Vec3f(25.0, 4.0, 1.0));
  guide_path.emplace_back(navigation_math::Vec3f(45.0, 4.0, 1.0));
  guide_path.emplace_back(tail.col(0));
  const std::vector<double> guide_times{0.0, 4.0, 7.0, 11.0};

  // The middle corridor represents the lateral bypass around a central
  // obstacle; adjacent overlaps remain wide enough to be a feasible high-
  // speed handover rather than an artificial one-box straight-line case.
  geometry_utils::PolytopeVec corridors{
      makeBox(-1.0, 30.0, -2.0, 6.0, 0.0, 3.0),
      makeBox(20.0, 50.0, 2.0, 6.0, 0.0, 3.0),
      makeBox(40.0, 71.0, -2.0, 6.0, 0.0, 3.0)};
  geometry_utils::Trajectory trajectory;

  ASSERT_TRUE(optimizer.optimize(
      head, tail, guide_path, guide_times, corridors, trajectory));
  ASSERT_EQ(trajectory.getPieceNum(), corridors.size());
  ASSERT_FALSE(trajectory.empty());
  for (std::size_t piece = 0; piece < corridors.size(); ++piece) {
    EXPECT_LE(
        navigation_planning_backend::maximumContinuousCorridorPlaneViolation(
            trajectory[static_cast<int>(piece)], corridors[piece].GetPlanes()),
        config.corridor_plane_tolerance_m);
  }
}

TEST(DeterministicNominalSeed, RequiresExactPieceCorridorMappingAndPvajContinuity) {
  auto config = traj_opt::Config(PLANNER_EXP_CONFIG_PATH, "exp_traj");
  config.max_vel = 5.0;
  config.max_acc = 5.0;
  config.max_jerk = 12.0;
  const auto initial = makePositionState(0.0);
  const auto middle = makePositionState(2.0);
  const auto terminal = makePositionState(4.0);
  const auto first = navigation_planning_backend::minimumSnapStateTransitionPiece(
      initial, middle, 3.0);
  const auto second = navigation_planning_backend::minimumSnapStateTransitionPiece(
      middle, terminal, 3.0);
  ASSERT_TRUE(first.has_value());
  ASSERT_TRUE(second.has_value());
  geometry_utils::Trajectory seed;
  seed.emplace_back(*first);
  seed.emplace_back(*second);

  navigation_math::PolyhedraH corridors{
      makeBox(-0.1, 2.1, -1.0, 1.0, 0.0, 2.0).GetPlanes(),
      makeBox(1.9, 4.1, -1.0, 1.0, 0.0, 2.0).GetPlanes()};
  navigation_math::VecDi mapping(2);
  mapping << 0, 1;
  const std::vector<unsigned char> gates(2, 0U);
  const std::vector<navigation_math::Vec3f> points(
      2, navigation_math::Vec3f::Zero());
  const std::vector<double> radii(
      2, std::numeric_limits<double>::quiet_NaN());

  const auto valid = navigation_planning_backend::certifyDeterministicNominalSeed(
      seed, corridors, mapping, gates, points, radii, initial, terminal, config);
  EXPECT_TRUE(valid.valid);
  EXPECT_LE(valid.maximum_corridor_violation_m,
            config.corridor_plane_tolerance_m);
  EXPECT_LE(valid.maximum_boundary_residual, 1.0e-8);

  mapping << 1, 0;
  EXPECT_FALSE(navigation_planning_backend::certifyDeterministicNominalSeed(
      seed, corridors, mapping, gates, points, radii, initial, terminal, config).valid);
}

TEST(DeterministicNominalSeed, RejectsAnyCorridorExcessWithoutToleranceRelaxation) {
  auto config = traj_opt::Config(PLANNER_EXP_CONFIG_PATH, "exp_traj");
  config.max_vel = 5.0;
  config.max_acc = 5.0;
  config.max_jerk = 12.0;
  const auto initial = makePositionState(0.0);
  const auto terminal = makePositionState(1.0);
  const auto piece = navigation_planning_backend::minimumSnapStateTransitionPiece(
      initial, terminal, 2.0);
  ASSERT_TRUE(piece.has_value());
  geometry_utils::Trajectory seed;
  seed.emplace_back(*piece);
  navigation_math::PolyhedraH corridors{
      makeBox(-0.1, 0.989999, -1.0, 1.0, 0.0, 2.0).GetPlanes()};
  navigation_math::VecDi mapping(1);
  mapping << 0;
  const std::vector<unsigned char> gates(1, 0U);
  const std::vector<navigation_math::Vec3f> points(
      1, navigation_math::Vec3f::Zero());
  const std::vector<double> radii(
      1, std::numeric_limits<double>::quiet_NaN());
  const auto rejected = navigation_planning_backend::certifyDeterministicNominalSeed(
      seed, corridors, mapping, gates, points, radii, initial, terminal, config);
  EXPECT_FALSE(rejected.valid);
  EXPECT_GT(rejected.maximum_corridor_violation_m,
            config.corridor_plane_tolerance_m);
}

}  // namespace
