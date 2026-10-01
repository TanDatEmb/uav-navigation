#include <gtest/gtest.h>

#include <array>
#include <atomic>
#include <cmath>
#include <chrono>
#include <cstdlib>
#include <memory>
#include <string>
#include <utility>

#include <planner_core/corridor_plane_validation.hpp>
#include <planner_core/deterministic_nominal_seed.hpp>
#include <planner_core/kinematic_state_boundary.hpp>
#include <planner_core/optimized_nominal_candidate.hpp>
#include <traj_opt/config.hpp>
#include <traj_opt/nominal_trajectory_optimizer.hpp>
#include <utils/optimization/lbfgs.h>

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

struct CapturedRenewal332Fixture {
  navigation_math::StatePVAJ head;
  navigation_math::StatePVAJ tail;
  navigation_math::vec_E<navigation_math::Vec3f> guide;
  std::vector<double> guide_t;
  geometry_utils::PolytopeVec corridors;
};

CapturedRenewal332Fixture makeCapturedRenewal332Fixture() {
  // Captured nominal-renewal-capture-nod2nN snapshot_2_2_16.json
  // (planner cycle 332, solve generation 54; source d3a801ce,
  // fingerprint 6e256...e092, manifest ba7a29...2e64). Setup-only diagnostic fixture;
  // it is not a complete executable bundle or flight-acceptance evidence.
  // The six PRE polytopes preserve ordinary optimize() setup input; passing
  // only four POST polytopes through SimplifySFC again is not setup parity.
  CapturedRenewal332Fixture out;
  out.head << 10.280708849609464, 1.9718184267288366, 3.460698148896486, 0.36018004354153277,
      2.2193448933127655, 0.29192781474471785, -0.9638361143636404, -0.2734847362885524,
      2.9507361208561975, 0.08378820616910174, 0.06933849624112493, -0.45554295333524086;
  out.tail << 26.1, 2.6119729524869952, 0.0, 0.0, -0.1, 0.0, 0.0, 0.0, 2.9000000000000004, 0.0, 0.0,
      0.0;
  out.guide = {{10.280708849609464, 2.2193448933127655, 2.9507361208561975},
               {10.449536131355035, 2.2395958462093768, 2.9576207179992955},
               {10.640377084874761, 2.2535977380861576, 2.9646974914659334},
               {10.852622821361608, 2.2613570914219778, 2.9716964699920609},
               {11.085200157373865, 2.2629637091775967, 2.978343974057533},
               {11.336643236151694, 2.2585757935827346, 2.9843808675700565},
               {11.605175464348974, 2.248403788662847, 2.9895801959617807},
               {11.888127323230881, 2.2327907530654736, 2.9937709333514384},
               {12.178342899642788, 2.2127366168803153, 2.9968931457641586},
               {12.464099413947309, 2.1899368256036773, 2.9990061013864349},
               {12.73215181428073, 2.1663707164253481, 3.0002532002639017},
               {12.970292234540088, 2.143950305728338, 3.0008266676254549},
               {13.169242430211037, 2.1242554176688326, 3.0009354024122028},
               {13.100000000000001, 2.1, 3.1},
               {16.1, 2.1, 3.1},
               {19.1, 2.1, 3.1},
               {21.900000000000002, 1.1, 3.1},
               {23.3, -0.1, 3.1},
               {24.1, -0.1, 2.9000000000000004},
               {26.1, -0.1, 2.9000000000000004}};
  out.guide_t = {0,
                 0.080000000000000071,
                 0.16000000000000014,
                 0.24000000000000021,
                 0.32000000000000028,
                 0.40000000000000036,
                 0.48000000000000043,
                 0.5600000000000005,
                 0.64000000000000057,
                 0.72000000000000064,
                 0.80000000000000071,
                 0.88000000000000078,
                 0.96000000000000085,
                 1.0288469835890202,
                 2.1411870105019872,
                 2.7444123228236168,
                 3.3390550727163566,
                 3.7078368510080724,
                 3.8727610760327789,
                 4.2727610760327792};
  // Exact captured PRE input; ordinary optimize() performs SimplifySFC.
  // All six PRE route-boundary gates are unset in this snapshot.
  const std::vector<std::vector<std::vector<double>>> raw = {
      {{-0.025459949605120526, -0.9971797570900652, -0.07059973807248704, 2.398783774188673},
       {0.11035703100197086, -0.9913410272676435, -0.07116384871803576, 0.7834514946722171},
       {-0.02622878251298125, -0.9930196313383998, 0.11499592490360507, 1.8429651543454864},
       {0.10860813008200541, -0.9875623791635054, 0.11368738426495488, 0.24084693216827222},
       {0, 2.7105054312137617e-20, -1, 1.4507361208561973},
       {0, -2.7105054312137617e-20, 1, -4.500935402412204},
       {-7.682588831596504e-19, 1, 0, -3.7193448933127655},
       {7.682588831596504e-19, -1, 0, 0.6242554176688326},
       {1, -2.504252908557339e-18, -1.734723475976807e-18, -14.669242430211037},
       {-1, 2.504252908557339e-18, 1.734723475976807e-18, 8.780708849609464},
       {0, 0, 1, -3.200935402412203},
       {0, 0, -1, 2.7507361208561973}},
      {{0, 0, 1, -4.6},
       {0, 1, 0, -3.5999999999999996},
       {0, -1, 0, 0.6000000000000004},
       {0, 0, -1, 1.6000000000000005},
       {-0.8029930424328082, -0.5959884007298317, 0, 11.332195418018667},
       {-0.796765826160455, -0.5925533230329181, 0.11851066460658351, 10.86109415046839},
       {-0.7785870217792048, -0.5826531539942719, -0.2330612615977085, 11.646969392331178},
       {1, 0, 0, -17.6},
       {-1, 0, 0, 11.600000000000001},
       {0, 0, 1, -3.3000000000000003},
       {0, 0, -1, 2.9}},
      {{1, 0, 0, -20.6},
       {0, 1, 0, -3.6},
       {0, 0, 1, -4.6},
       {-1, 0, 0, 14.600000000000001},
       {0, -1, 0, 0.6000000000000001},
       {0, 0, -1, 1.6},
       {0, 0, 1, -3.3000000000000003},
       {0, 0, -1, 2.9}},
      {{0, 0, 1, -4.6},
       {0, 0, -1, 1.6},
       {0.9676334875999666, 0.2523597306923695, 0, -21.787756358950602},
       {-1.3877787807814454e-17, 1, 0, -3.5999999999999996},
       {1.3877787807814454e-17, -1, 0, -0.3999999999999996},
       {-1, 2.7755575615628914e-17, 0, 17.6},
       {1, -2.7755575615628914e-17, 0, -23.400000000000002},
       {0, 0, 1, -3.3000000000000003},
       {0, 0, -1, 2.9}},
      {{0.7349748627481386, 0.678094352673988, 0, -17.070183878735513},
       {0.5633700667291068, 0.8262046767681975, 0, -13.287674317896293},
       {1, 0, 0, -24.8},
       {-1, 0, 0, 20.400000000000002},
       {0, 1, 0, -2.6},
       {0, -1, 0, -1.6},
       {0, 0, 1, -4.6},
       {0, 0, -1, 1.6000000000000005},
       {0, 0, 1, -3.3000000000000003},
       {0, 0, -1, 2.9}},
      {{-0.546163994158071, 0.8126672954922194, 0.20316682387305451, 12.65463519952688},
       {-0.5632942866920952, 0.8262563443508579, 0, 13.518645613140784},
       {-0.5221188934898598, 0.7918724323970695, -0.3167489729588283, 13.358382826705451},
       {-0.4213538031047716, 0.7776571964264097, -0.46659431785584576, 11.32837433728659},
       {1, 0, 0, -27.6},
       {-1, 0, 0, 22.6},
       {0, 1, 0, -1.4},
       {0, -1, 0, -1.6},
       {0, 0, 1, -4.4},
       {0, 0, -1, 1.4000000000000004},
       {0, 0, 1, -3.1000000000000005},
       {0, 0, -1, 2.7}}};
  for (const auto& rows : raw) {
    navigation_math::MatD4f planes(rows.size(), 4);
    for (std::size_t r = 0; r < rows.size(); ++r)
      for (std::size_t c = 0; c < 4; ++c) planes(r, c) = rows[r][c];
    out.corridors.emplace_back(std::move(planes));
  }
  return out;
}

class ScopedEnvironmentVariable {
 public:
  ScopedEnvironmentVariable(const char* name, const char* value)
      : name_(name), previous_(std::getenv(name)) {
    if (previous_ != nullptr) {
      previous_value_ = previous_;
    }
    EXPECT_EQ(setenv(name_.c_str(), value, 1), 0);
  }

  ~ScopedEnvironmentVariable() {
    if (previous_ != nullptr) {
      setenv(name_.c_str(), previous_value_.c_str(), 1);
    } else {
      unsetenv(name_.c_str());
    }
  }

 private:
  std::string name_;
  const char* previous_;
  std::string previous_value_;
};

TEST(ExpOptimizer, RouteGateDoesNotKeepZeroDisplacementMovingTailSeed) {
  const traj_opt::Config config(PLANNER_EXP_CONFIG_PATH, "exp_traj");
  const auto context =
      std::make_shared<navigation_planner_context::PlannerRuntimeContext>(
          [] { return 12.0; });
  traj_opt::ExpTrajOpt optimizer(config, context);
  const auto head = makePositionState(0.0);
  const auto tail = makeMovingPositionState(13.0, 2.0);
  auto gate = makeBox(4.8, 5.2, -1.0, 1.0, 0.0, 2.0);
  gate.SetRouteBoundaryContract({5.0, 0.0, 1.0}, 0.9);
  geometry_utils::PolytopeVec corridors{
      makeBox(-1.0, 6.0, -1.0, 1.0, 0.0, 2.0), gate,
      makeBox(4.0, 10.0, -1.0, 1.0, 0.0, 2.0),
      makeBox(9.0, 14.0, -1.0, 1.0, 0.0, 2.0),
      makeBox(11.0, 16.0, -1.0, 1.0, 0.0, 2.0)};
  const navigation_math::vec_Vec3f guide{
      head.col(0), {5.0, 0.0, 1.0}, {10.0, 0.0, 1.0}, tail.col(0)};
  const std::vector<double> times{0.0, 5.0, 10.0, 13.0};
  geometry_utils::Trajectory trajectory;
  const auto result = optimizer.solve(
      head, tail, guide, times, corridors, trajectory, false, true, false);
  ASSERT_TRUE(result.candidateAvailable());
  navigation_math::VecDf initial_times;
  navigation_math::vec_Vec3f initial_points;
  optimizer.getInitValue(initial_times, initial_points);
  ASSERT_EQ(initial_times.size(), 4);
  ASSERT_EQ(initial_points.size(), 3U);
  EXPECT_GT((initial_points.back() - tail.col(0)).norm(), 1.0);
  EXPECT_TRUE(corridors[1].IsRouteBoundaryGate());
  EXPECT_TRUE(initial_points.front().isApprox(gate.GetRouteBoundaryPoint(), 0.0));
  EXPECT_NEAR(initial_times.sum(), times.back(), 1.0e-12);
  EXPECT_GT(initial_times.minCoeff(), 0.0);
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

TEST(ExpOptimizer, WarmStartRejectsSeedDimensionsAfterCorridorSimplification) {
  const traj_opt::Config config(PLANNER_EXP_CONFIG_PATH, "exp_traj");
  const auto context =
      std::make_shared<navigation_planner_context::PlannerRuntimeContext>(
          [] { return 12.0; });
  traj_opt::ExpTrajOpt optimizer(config, context);
  const auto head = makePositionState(0.0);
  const auto tail = makePositionState(4.0);
  geometry_utils::PolytopeVec corridors{
      makeConvexBox(), makeConvexBox(), makeConvexBox()};
  const navigation_math::vec_Vec3f initial_points{
      {1.0, 0.0, 1.0}, {3.0, 0.0, 1.0}};
  const navigation_math::VecDf initial_times =
      navigation_math::VecDf::Constant(3, 1.0);
  geometry_utils::Trajectory trajectory;

  EXPECT_FALSE(optimizer.optimize(
      head, tail, corridors, initial_points, initial_times, trajectory));
  ASSERT_EQ(corridors.size(), 1U);
  EXPECT_TRUE(trajectory.empty());
  EXPECT_EQ(optimizer.diagnostics().lbfgs_evaluation_count, 0U);
}

TEST(ExpOptimizer, WarmStartRetainsMatchingPostSimplificationSeed) {
  const traj_opt::Config config(PLANNER_EXP_CONFIG_PATH, "exp_traj");
  const auto context =
      std::make_shared<navigation_planner_context::PlannerRuntimeContext>(
          [] { return 12.0; });
  traj_opt::ExpTrajOpt optimizer(config, context);
  geometry_utils::PolytopeVec corridors{makeConvexBox()};
  const navigation_math::vec_Vec3f initial_points;
  const navigation_math::VecDf initial_times =
      navigation_math::VecDf::Constant(1, 4.0);
  geometry_utils::Trajectory trajectory;

  ASSERT_TRUE(optimizer.optimize(
      makePositionState(0.0), makePositionState(4.0), corridors,
      initial_points, initial_times, trajectory));
  EXPECT_FALSE(trajectory.empty());
  EXPECT_EQ(trajectory.getPieceNum(), 1);
}

TEST(ExpOptimizer,
     SnapshotDoesNotReusePostSetupGeometryAfterSimplifyReject) {
  ScopedEnvironmentVariable capture(
      "UAV_NAVIGATION_NOMINAL_SNAPSHOT_DIR", "/tmp/uav-navigation-snapshot-test");
  ScopedEnvironmentVariable session(
      "UAV_NAVIGATION_NOMINAL_SNAPSHOT_SESSION_ID", "session-123");
  ScopedEnvironmentVariable source_commit(
      "UAV_NAVIGATION_SOURCE_COMMIT", "source-commit");
  ScopedEnvironmentVariable source_diff(
      "UAV_NAVIGATION_SOURCE_DIFF_SHA256", "source-diff");
  ScopedEnvironmentVariable source_fingerprint(
      "UAV_NAVIGATION_SOURCE_FINGERPRINT_SHA256", "source-fingerprint");
  ScopedEnvironmentVariable workspace(
      "UAV_NAVIGATION_WORKSPACE", "/workspace");
  ScopedEnvironmentVariable build_manifest(
      "UAV_NAVIGATION_BUILD_MANIFEST", "/workspace/manifest.json");
  ScopedEnvironmentVariable build_manifest_sha(
      "UAV_NAVIGATION_BUILD_MANIFEST_SHA256", "manifest-sha");
  const traj_opt::Config config(PLANNER_EXP_CONFIG_PATH, "exp_traj");
  const auto planner_context =
      std::make_shared<navigation_planner_context::PlannerRuntimeContext>(
          [] { return 12.0; });
  traj_opt::ExpTrajOpt optimizer(config, planner_context);

  const auto head = makePositionState(0.0);
  const auto tail = makePositionState(8.0);
  navigation_math::vec_E<navigation_math::Vec3f> guide_path;
  guide_path.emplace_back(head.col(0));
  guide_path.emplace_back(tail.col(0));
  const std::vector<double> guide_times{0.0, 2.0};
  geometry_utils::Trajectory trajectory;

  geometry_utils::PolytopeVec valid_corridors{makeConvexBox()};
  ASSERT_TRUE(optimizer.optimize(
      head, tail, guide_path, guide_times, valid_corridors, trajectory));
  const auto valid_snapshot = optimizer.takeNominalProblemSnapshot();
  ASSERT_TRUE(valid_snapshot.has_value());
  EXPECT_EQ(valid_snapshot->provenance.session_id, "session-123");
  EXPECT_EQ(valid_snapshot->provenance.source_revision, "source-commit");
  EXPECT_EQ(valid_snapshot->provenance.source_diff_sha256, "source-diff");
  EXPECT_EQ(
      valid_snapshot->provenance.source_fingerprint_sha256,
      "source-fingerprint");
  EXPECT_EQ(valid_snapshot->provenance.workspace, "/workspace");
  EXPECT_EQ(
      valid_snapshot->provenance.build_manifest_path,
      "/workspace/manifest.json");
  EXPECT_EQ(valid_snapshot->provenance.build_manifest_sha256, "manifest-sha");
  EXPECT_TRUE(valid_snapshot->setup_completed);
  EXPECT_TRUE(valid_snapshot->post_setup_input_bound);
  ASSERT_EQ(valid_snapshot->h_polytopes.size(), 1U);

  geometry_utils::PolytopeVec rejected_corridors{
      makeBox(1.0, 2.0, -10.0, 10.0, 0.0, 10.0),
      makeBox(3.0, 4.0, -10.0, 10.0, 0.0, 10.0),
      makeBox(6.0, 7.0, -10.0, 10.0, 0.0, 10.0)};
  trajectory = geometry_utils::Trajectory{};
  ASSERT_FALSE(optimizer.optimize(
      head, tail, guide_path, guide_times, rejected_corridors, trajectory));
  const auto rejected_snapshot = optimizer.takeNominalProblemSnapshot();
  ASSERT_TRUE(rejected_snapshot.has_value());
  EXPECT_FALSE(rejected_snapshot->setup_completed);
  EXPECT_FALSE(rejected_snapshot->post_setup_input_bound);
  EXPECT_EQ(rejected_snapshot->setup_failure_stage, 2);
  ASSERT_EQ(rejected_snapshot->pre_simplify_h_polytopes.size(), 3U);
  EXPECT_EQ(rejected_snapshot->h_polytopes.size(), 3U);
  EXPECT_TRUE(rejected_snapshot->h_overlap_polytopes.empty());
}

TEST(ExpOptimizer, RejectsMissingPlannerContext) {
  const traj_opt::Config config(PLANNER_EXP_CONFIG_PATH, "exp_traj");
  EXPECT_THROW(traj_opt::ExpTrajOpt(config, nullptr), std::invalid_argument);
}

TEST(PieceCertificate, IgnoresRoundoffOnlyLeadingPolynomialTerms) {
  constexpr double duration_s = 0.2;
  Eigen::MatrixXd coefficients = Eigen::MatrixXd::Zero(3, 6);
  // The leading t^5 term is LU roundoff from an analytically quartic stop.
  // It must not turn the extrema equation into an ill-conditioned higher-
  // degree polynomial and hide the 6 m/s2 interior acceleration peak.
  coefficients.row(0) << -6.6613381477509625e-13,
      50.000000000000327, -20.000000000000039, 0.0, 0.8, 0.0;
  const geometry_utils::Piece piece(duration_s, coefficients);

  EXPECT_NEAR(piece.getMaxVelRate(), 0.8, 1.0e-9);
  EXPECT_NEAR(piece.getMaxAccRate(), 6.0, 1.0e-8);
  EXPECT_NEAR(piece.getMaxJerRate(), 120.0, 1.0e-7);
  EXPECT_FALSE(piece.checkMaxAccRate(0.3));
}

TEST(DeterministicNominalSeed, RejectsMainCandidateAboveControlEnvelope) {
  auto config = traj_opt::Config(PLANNER_EXP_CONFIG_PATH, "exp_traj");
  config.max_vel = 0.5624988750005627;
  config.max_acc = 0.2165062314375;
  config.max_jerk = 0.2804066718750005;
  const auto initial = makePositionState(0.0);
  const auto terminal = makePositionState(1.0);
  const auto piece = navigation_planning_backend::minimumSnapStateTransitionPiece(
      initial, terminal, 0.2);
  ASSERT_TRUE(piece.has_value());
  geometry_utils::Trajectory seed;
  seed.emplace_back(*piece);

  navigation_math::PolyhedraH corridors{makeConvexBox().GetPlanes()};
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
  EXPECT_EQ(rejected.failure_stage,
            navigation_planning_backend::DeterministicNominalSeedFailureStage::kDynamics);
  EXPECT_GT(rejected.maximum_acceleration_mps2, config.max_acc);
}

TEST(OptimizedNominalCandidateCertificate, AcceptsCompleteHardFeasibleCandidate) {
  auto config = traj_opt::Config(PLANNER_EXP_CONFIG_PATH, "exp_traj");
  config.max_vel = 5.0;
  config.max_acc = 5.0;
  config.max_jerk = 20.0;
  const auto initial = makePositionState(0.0);
  const auto terminal = makePositionState(1.0);
  const auto piece = navigation_planning_backend::minimumSnapStateTransitionPiece(
      initial, terminal, 5.0);
  ASSERT_TRUE(piece.has_value());
  geometry_utils::Trajectory candidate;
  candidate.emplace_back(*piece);

  navigation_math::PolyhedraH corridors{makeConvexBox().GetPlanes()};
  navigation_math::VecDi mapping(1);
  mapping << 0;
  const std::vector<unsigned char> gates(1, 0U);
  const std::vector<navigation_math::Vec3f> points(
      1, navigation_math::Vec3f::Zero());
  const std::vector<double> radii(
      1, std::numeric_limits<double>::quiet_NaN());

  const auto certificate =
      navigation_planning_backend::certifyOptimizedNominalCandidate(
          candidate, corridors, mapping, gates, points, radii, config);
  EXPECT_TRUE(certificate.valid);
  EXPECT_EQ(certificate.failure_stage,
            navigation_planning_backend::
                OptimizedNominalCandidateFailureStage::kNone);
}

TEST(OptimizedNominalCandidateCertificate, RejectsEachExternalHardBoundary) {
  auto config = traj_opt::Config(PLANNER_EXP_CONFIG_PATH, "exp_traj");
  config.max_vel = 5.0;
  config.max_acc = 5.0;
  config.max_jerk = 20.0;
  const auto initial = makePositionState(0.0);
  const auto terminal = makePositionState(1.0);
  const auto piece = navigation_planning_backend::minimumSnapStateTransitionPiece(
      initial, terminal, 5.0);
  ASSERT_TRUE(piece.has_value());
  geometry_utils::Trajectory candidate;
  candidate.emplace_back(*piece);
  navigation_math::VecDi mapping(1);
  mapping << 0;
  const std::vector<unsigned char> no_gates(1, 0U);
  const std::vector<navigation_math::Vec3f> unused_points(
      1, navigation_math::Vec3f::Zero());
  const std::vector<double> unused_radii(
      1, std::numeric_limits<double>::quiet_NaN());

  const navigation_math::PolyhedraH disjoint_corridor{
      makeBox(2.0, 3.0, -1.0, 1.0, 0.0, 2.0).GetPlanes()};
  const auto corridor_reject =
      navigation_planning_backend::certifyOptimizedNominalCandidate(
          candidate, disjoint_corridor, mapping, no_gates, unused_points,
          unused_radii, config);
  EXPECT_FALSE(corridor_reject.valid);
  EXPECT_EQ(corridor_reject.failure_stage,
            navigation_planning_backend::
                OptimizedNominalCandidateFailureStage::kCorridor);

  const navigation_math::PolyhedraH corridor{makeConvexBox().GetPlanes()};
  const std::vector<unsigned char> route_gate(1, 1U);
  const std::vector<navigation_math::Vec3f> unreachable_point{
      navigation_math::Vec3f(8.0, 0.0, 1.0)};
  const std::vector<double> route_radius{0.25};
  const auto route_reject =
      navigation_planning_backend::certifyOptimizedNominalCandidate(
          candidate, corridor, mapping, route_gate, unreachable_point,
          route_radius, config);
  EXPECT_FALSE(route_reject.valid);
  EXPECT_EQ(route_reject.failure_stage,
            navigation_planning_backend::
                OptimizedNominalCandidateFailureStage::kRouteBoundary);

  config.max_vel = 0.01;
  const auto dynamics_reject =
      navigation_planning_backend::certifyOptimizedNominalCandidate(
          candidate, corridor, mapping, no_gates, unused_points,
          unused_radii, config);
  EXPECT_FALSE(dynamics_reject.valid);
  EXPECT_EQ(dynamics_reject.failure_stage,
            navigation_planning_backend::
                OptimizedNominalCandidateFailureStage::kDynamics);

  config.max_vel = 5.0;
  config.max_acc_thr = 9.0;
  const auto flatness_reject =
      navigation_planning_backend::certifyOptimizedNominalCandidate(
          candidate, corridor, mapping, no_gates, unused_points,
          unused_radii, config);
  EXPECT_FALSE(flatness_reject.valid);
  EXPECT_EQ(flatness_reject.failure_stage,
            navigation_planning_backend::
                OptimizedNominalCandidateFailureStage::kFlatness);
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

TEST(ExpOptimizer, MandatoryFeasibilityUsesHardDeadlineWhenNoCertifiedSeed) {
  auto config = traj_opt::Config(PLANNER_EXP_CONFIG_PATH, "exp_traj");
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

  const auto now = std::chrono::steady_clock::now();
  const auto refinement_deadline = std::chrono::duration_cast<
      std::chrono::nanoseconds>(now.time_since_epoch()).count();
  const auto hard_deadline = std::chrono::duration_cast<
      std::chrono::nanoseconds>((now + std::chrono::seconds(1)).time_since_epoch()).count();
  optimizer.setSolveBudget(nullptr, refinement_deadline, hard_deadline);

  const auto result = optimizer.solve(
      head, tail, guide_path, guide_times, corridors, trajectory,
      false, false, false);

  EXPECT_TRUE(result.candidateAvailable());
  EXPECT_FALSE(trajectory.empty());
  const auto diagnostics = optimizer.diagnostics();
  EXPECT_EQ(diagnostics.refinement_budget_at_entry_us, 0);
  EXPECT_FALSE(diagnostics.hard_deadline_observed);
  EXPECT_GT(diagnostics.lbfgs_attempt_count, 0);
  EXPECT_EQ(diagnostics.certified_seed_failure_stage, 5);
  EXPECT_TRUE(diagnostics.used_feasible_iterate_checkpoint);
  EXPECT_GT(diagnostics.feasible_iterate_checkpoint_attempt, 0);
  EXPECT_GT(diagnostics.feasible_iterate_checkpoint_iteration, 0);
  EXPECT_GT(diagnostics.feasible_iterate_certificate_count, 0);
  EXPECT_GE(diagnostics.feasible_iterate_certificate_time_us, 0);
}

TEST(NominalSolveRevocation, CancellationOverridesRemainingHardBudget) {
  std::atomic_bool cancelled{false};
  EXPECT_FALSE(traj_opt::nominalSolveRevoked(&cancelled, 100, 10));
  cancelled.store(true, std::memory_order_relaxed);
  EXPECT_TRUE(traj_opt::nominalSolveRevoked(&cancelled, 100, 11));
  EXPECT_TRUE(traj_opt::nominalSolveRevoked(&cancelled, 0, 11));
}

TEST(NominalSolveRevocation, HardExpiryIncludesExactBoundary) {
  EXPECT_FALSE(traj_opt::nominalSolveRevoked(nullptr, 100, 99));
  EXPECT_TRUE(traj_opt::nominalSolveRevoked(nullptr, 100, 100));
  EXPECT_TRUE(traj_opt::nominalSolveRevoked(nullptr, 100, 101));
  // Preserve the explicitly unbudgeted offline diagnostic contract.
  EXPECT_FALSE(traj_opt::nominalSolveRevoked(nullptr, 0, 101));
}

TEST(ExpOptimizer, CertifiedIncumbentSurvivesLaterOptimizerRejection) {
  auto fixture = makeCapturedRenewal332Fixture();
  auto config = traj_opt::Config(PLANNER_EXP_CONFIG_PATH, "exp_traj");
  config.max_vel = 5.0;
  config.max_acc = 5.0;
  config.max_jerk = 8.0;
  config.validate();
  const auto planner_context =
      std::make_shared<navigation_planner_context::PlannerRuntimeContext>([] { return 12.0; });
  traj_opt::ExpTrajOpt optimizer(config, planner_context);
  geometry_utils::Trajectory trajectory;

  // Keep both deadlines well clear of wall-clock expiry; this characterizes
  // checkpoint retention, not deadline scheduling.
  const auto now = std::chrono::steady_clock::now();
  const auto refinement_deadline = std::chrono::duration_cast<std::chrono::nanoseconds>(
                                       (now + std::chrono::hours(1)).time_since_epoch())
                                       .count();
  const auto hard_deadline = std::chrono::duration_cast<std::chrono::nanoseconds>(
                                 (now + std::chrono::hours(2)).time_since_epoch())
                                 .count();
  optimizer.setSolveBudget(nullptr, refinement_deadline, hard_deadline);

  const auto result = optimizer.solve(fixture.head, fixture.tail, fixture.guide, fixture.guide_t,
                                      fixture.corridors, trajectory, false, false, false);

  ASSERT_TRUE(result.candidateAvailable());
  ASSERT_FALSE(trajectory.empty());
  const auto diagnostics = optimizer.diagnostics();
  EXPECT_EQ(diagnostics.certified_seed_failure_stage, 5);
  EXPECT_TRUE(diagnostics.used_feasible_iterate_checkpoint);
  EXPECT_GT(diagnostics.feasible_iterate_certificate_count, 0);
  // Capturing an incumbent must not implement the withdrawn early-return
  // policy: this frozen job continues into later optimizer attempts.
  EXPECT_GT(diagnostics.lbfgs_attempt_count, diagnostics.feasible_iterate_checkpoint_attempt);
  EXPECT_LE(trajectory.getMaxVelRate(), config.max_vel);
  EXPECT_LE(trajectory.getMaxAccRate(), config.max_acc);
  EXPECT_LE(trajectory.getMaxJerRate(), config.max_jerk);
}

TEST(ExpOptimizer, MandatoryFeasibilityDoesNotPreemptOptionalRefinementWindow) {
  auto config = traj_opt::Config(PLANNER_EXP_CONFIG_PATH, "exp_traj");
  config.optimization_dynamic_reserve_ratio = 1.0;
  config.max_vel = 8.0;
  config.max_acc = 2.0;
  config.max_jerk = 4.0;
  config.jerk_penalty_weight = 0.0;
  const auto context =
      std::make_shared<navigation_planner_context::PlannerRuntimeContext>([] { return 12.0; });
  traj_opt::ExpTrajOpt optimizer(config, context);
  const auto head = makeMovingPositionState(0.0, 8.0);
  const auto tail = makePositionState(30.0);
  const navigation_math::vec_E<navigation_math::Vec3f> guide{
      head.col(0), {10.0, 0.0, 1.0}, {20.0, 0.0, 1.0}, tail.col(0)};
  const std::vector<double> times{0.0, 1.8, 3.6, 5.6};
  geometry_utils::PolytopeVec corridors{makeBox(-1.0, 12.0, -2.0, 2.0, 0.0, 3.0),
                                        makeBox(8.0, 22.0, -2.0, 2.0, 0.0, 3.0),
                                        makeBox(18.0, 31.0, -2.0, 2.0, 0.0, 3.0)};
  geometry_utils::Trajectory trajectory;
  // Keep the optional cutoff far in the future without sleeping or asserting
  // a machine-dependent runtime. A nominal-only checkpoint must not preempt
  // the reserved refinement window without complete-bundle readiness evidence.
  const auto now = std::chrono::steady_clock::now();
  const auto cutoff_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
                             (now + std::chrono::hours(1)).time_since_epoch())
                             .count();
  const auto hard_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
                           (now + std::chrono::hours(2)).time_since_epoch())
                           .count();
  optimizer.setSolveBudget(nullptr, cutoff_ns, hard_ns);
  const auto result =
      optimizer.solve(head, tail, guide, times, corridors, trajectory, false, false, false);

  ASSERT_TRUE(result.candidateAvailable());
  ASSERT_FALSE(trajectory.empty());
  const auto diagnostics = optimizer.diagnostics();
  EXPECT_EQ(diagnostics.certified_seed_failure_stage, 5);
  EXPECT_GT(diagnostics.refinement_budget_at_entry_us, 0);
  // Final rejection may legitimately select a certified incumbent after
  // refinement. Check raw solver stops instead of conflating that selection
  // with preemption; a canceled stop may otherwise be converted to STOP.
  EXPECT_FALSE(diagnostics.cancelled);
  EXPECT_NE(diagnostics.first_lbfgs_return_code, math_utils::lbfgs::LBFGS_CANCELED);
  EXPECT_NE(diagnostics.last_lbfgs_return_code, math_utils::lbfgs::LBFGS_CANCELED);
  if (diagnostics.used_feasible_iterate_checkpoint) {
    EXPECT_GT(diagnostics.feasible_iterate_checkpoint_attempt, 0);
    // This fixture continues into later attempts; this is not a universal
    // requirement for same-attempt refinement followed by final fallback.
    EXPECT_GT(diagnostics.lbfgs_attempt_count,
              diagnostics.feasible_iterate_checkpoint_attempt);
  }
  EXPECT_LT(std::chrono::duration_cast<std::chrono::nanoseconds>(
                std::chrono::steady_clock::now().time_since_epoch()).count(),
            cutoff_ns);
  EXPECT_FALSE(diagnostics.hard_deadline_observed);
  EXPECT_GT(diagnostics.feasible_iterate_certificate_count, 0);
  EXPECT_LE(trajectory.getMaxVelRate(), config.max_vel);
  EXPECT_LE(trajectory.getMaxAccRate(), config.max_acc);
  EXPECT_LE(trajectory.getMaxJerRate(), config.max_jerk);
}

TEST(ExpOptimizer, ExplicitCancellationStillStopsMandatoryFeasibility) {
  auto config = traj_opt::Config(PLANNER_EXP_CONFIG_PATH, "exp_traj");
  config.optimization_dynamic_reserve_ratio = 1.0;
  config.max_vel = 8.0;
  config.max_acc = 2.0;
  config.max_jerk = 4.0;
  const auto planner_context =
      std::make_shared<navigation_planner_context::PlannerRuntimeContext>(
          [] { return 12.0; });

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
  std::atomic_bool cancelled{true};
  const auto now = std::chrono::steady_clock::now();
  const auto expired_refinement = std::chrono::duration_cast<
      std::chrono::nanoseconds>(now.time_since_epoch()).count();
  const auto hard_deadline = std::chrono::duration_cast<
      std::chrono::nanoseconds>((now + std::chrono::seconds(1)).time_since_epoch()).count();
  traj_opt::ExpTrajOpt optimizer(config, planner_context);
  optimizer.setSolveBudget(&cancelled, expired_refinement, hard_deadline);

  const auto result = optimizer.solve(
      head, tail, guide_path, guide_times, corridors, trajectory,
      false, false, false);

  EXPECT_FALSE(result.candidateAvailable());
  EXPECT_TRUE(trajectory.empty());
  EXPECT_TRUE(optimizer.diagnostics().cancelled);
  EXPECT_FALSE(optimizer.diagnostics().hard_deadline_observed);
  EXPECT_FALSE(optimizer.diagnostics().used_feasible_iterate_checkpoint);
}

TEST(ExpOptimizer, MandatoryFeasibilityReportsExpiredHardDeadline) {
  auto config = traj_opt::Config(PLANNER_EXP_CONFIG_PATH, "exp_traj");
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

  const auto deadline = std::chrono::steady_clock::now();
  const auto expired_deadline = std::chrono::duration_cast<
      std::chrono::nanoseconds>(deadline.time_since_epoch()).count();
  optimizer.setSolveBudget(nullptr, expired_deadline, expired_deadline);

  const auto result = optimizer.solve(
      head, tail, guide_path, guide_times, corridors, trajectory,
      false, false, false);

  EXPECT_FALSE(result.candidateAvailable());
  EXPECT_TRUE(trajectory.empty());
  EXPECT_TRUE(optimizer.diagnostics().hard_deadline_observed);
  EXPECT_FALSE(optimizer.diagnostics().used_feasible_iterate_checkpoint);
}

TEST(ExpOptimizer, PassThroughJunctionRemainsInsideAcceptanceBall) {
  auto config = traj_opt::Config(PLANNER_EXP_CONFIG_PATH, "exp_traj");
  config.max_vel = 5.0;
  config.max_acc = 4.0;
  config.max_jerk = 12.0;
  const auto planner_context =
      std::make_shared<navigation_planner_context::PlannerRuntimeContext>(
          [] { return 12.0; });
  traj_opt::ExpTrajOpt optimizer(config, planner_context);

  const auto head = makeMovingPositionState(0.0, 2.0);
  auto tail = makePositionState(8.0);
  tail.col(0).y() = 4.0;
  tail.col(1) << 1.0, 1.0, 0.0;
  navigation_math::vec_E<navigation_math::Vec3f> guide_path;
  guide_path.emplace_back(head.col(0));
  guide_path.emplace_back(navigation_math::Vec3f(4.0, 0.0, 1.0));
  guide_path.emplace_back(navigation_math::Vec3f(5.0, 1.0, 1.0));
  guide_path.emplace_back(tail.col(0));
  const std::vector<double> guide_times{0.0, 2.0, 3.0, 6.0};

  geometry_utils::PolytopeVec corridors{
      makeBox(-1.0, 4.5, -1.0, 1.0, 0.0, 2.0),
      makeBox(3.0, 5.5, -1.0, 2.5, 0.0, 2.0),
      makeBox(4.0, 9.0, 0.0, 5.0, 0.0, 2.0)};
  const Eigen::Vector3d waypoint{4.0, 0.0, 1.0};
  constexpr double kAcceptanceRadiusM = 1.0;
  corridors[1].SetRouteBoundaryContract(
      waypoint, kAcceptanceRadiusM);
  geometry_utils::Trajectory trajectory;

  ASSERT_TRUE(optimizer.optimize(
      head, tail, guide_path, guide_times, corridors, trajectory));
  ASSERT_GE(trajectory.getPieceNum(), 2);
  double closest_junction_distance_m =
      std::numeric_limits<double>::infinity();
  for (int junction = 0; junction < trajectory.getPieceNum() - 1; ++junction) {
    closest_junction_distance_m = std::min(
        closest_junction_distance_m,
        (trajectory.getJuncPos(junction) - waypoint).norm());
  }
  EXPECT_LE(closest_junction_distance_m, kAcceptanceRadiusM + 1.0e-6);
}

TEST(ExpOptimizer, SparseStraightGuideKeepsJunctionGeometryAndTimeOnTheSameEdge) {
  ScopedEnvironmentVariable capture("UAV_NAVIGATION_NOMINAL_SNAPSHOT_DIR",
                                    "/unused/guide-junction-fixture");
  const traj_opt::Config config(PLANNER_EXP_CONFIG_PATH, "exp_traj");
  const auto context =
      std::make_shared<navigation_planner_context::PlannerRuntimeContext>(
          [] { return 12.0; });
  traj_opt::ExpTrajOpt optimizer(config, context);
  auto head = makePositionState(0.0);
  auto tail = makePositionState(13.8);
  head.col(0) << 0.0, -0.1, 3.0;
  tail.col(0) << 13.8, -0.1, 3.0;
  const navigation_math::vec_Vec3f guide{
      head.col(0), {2.7, -0.1, 3.0}, {5.7, -0.1, 3.0},
      {8.5, -0.1, 3.0}, {11.5, -0.1, 3.0}, tail.col(0)};
  const std::vector<double> guide_times{0.0, 0.8, 1.4, 2.0, 2.6, 3.2};
  // The middle overlap [7.0,7.2] contains no discrete guide sample, but
  // does contain its collision-checked straight edge. Its asymmetric box
  // interior is deliberately off that guide, as in the frozen 2WP input.
  geometry_utils::PolytopeVec corridors{
      makeBox(-1.6, 4.2, -1.6, 1.4, 2.7, 3.2),
      makeBox(1.2, 7.2, -1.6, 1.4, 2.8, 3.2),
      makeBox(7.0, 13.0, -1.6, 1.4, 2.8, 3.2),
      makeBox(10.7, 15.8, -1.6, 1.4, 2.8, 3.2)};
  geometry_utils::Trajectory trajectory;
  // Read immutable pre-optimization state regardless of solve success;
  // optimizer convergence cannot hide a defective initialization.
  optimizer.optimize(head, tail, guide, guide_times, corridors, trajectory, true);
  const auto snapshot = optimizer.takeNominalProblemSnapshot();
  ASSERT_TRUE(snapshot.has_value());
  ASSERT_TRUE(snapshot->setup_completed);
  ASSERT_EQ(snapshot->initial_spatial_variables.cols(), 3);
  double junction_time_s = 0.0;
  for (Eigen::Index junction = 0; junction < 3; ++junction) {
    const auto point = snapshot->initial_spatial_variables.col(junction);
    EXPECT_NEAR(point.y(), -0.1, 1.0e-12);
    EXPECT_NEAR(point.z(), 3.0, 1.0e-12);
    junction_time_s += snapshot->initial_durations_s(junction);
    const auto upper = std::upper_bound(
        guide.begin(), guide.end(), point.x(),
        [](double x, const auto& p) { return x < p.x(); });
    ASSERT_NE(upper, guide.begin());
    ASSERT_NE(upper, guide.end());
    const auto edge = static_cast<std::size_t>(upper - guide.begin() - 1);
    const double fraction = (point.x() - guide[edge].x()) /
        (guide[edge + 1U].x() - guide[edge].x());
    EXPECT_NEAR(junction_time_s, guide_times[edge] + fraction *
        (guide_times[edge + 1U] - guide_times[edge]), 1.0e-12);
  }
}

TEST(ExpOptimizer, RepeatedSparseGuideProjectionAvoidsTinyClampPieces) {
  auto config = traj_opt::Config(PLANNER_EXP_CONFIG_PATH, "exp_traj");
  config.max_vel = 20.0;
  config.max_acc = 20.0;
  config.max_jerk = 100.0;
  const auto planner_context =
      std::make_shared<navigation_planner_context::PlannerRuntimeContext>(
          [] { return 12.0; });
  traj_opt::ExpTrajOpt optimizer(config, planner_context);

  const auto head = makePositionState(0.0);
  const auto tail = makePositionState(20.0);
  navigation_math::vec_E<navigation_math::Vec3f> guide_path;
  guide_path.emplace_back(head.col(0));
  guide_path.emplace_back(navigation_math::Vec3f(10.0, 0.0, 1.0));
  guide_path.emplace_back(tail.col(0));
  const std::vector<double> guide_times{0.0, 2.0, 4.0};
  geometry_utils::PolytopeVec corridors{
      makeBox(-1.0, 8.0, -2.0, 2.0, 0.0, 2.0),
      makeBox(4.0, 12.0, -2.0, 2.0, 0.0, 2.0),
      makeBox(8.0, 16.0, -2.0, 2.0, 0.0, 2.0),
      makeBox(12.0, 21.0, -2.0, 2.0, 0.0, 2.0)};
  corridors[2].SetRouteBoundaryContract(
      Eigen::Vector3d{10.0, 0.0, 1.0}, 1.0);
  geometry_utils::Trajectory trajectory;

  ASSERT_TRUE(optimizer.optimize(
      head, tail, guide_path, guide_times, corridors, trajectory));
  const auto diagnostics = optimizer.diagnostics();
  ASSERT_TRUE(diagnostics.valid);
  EXPECT_GT(diagnostics.initial_minimum_piece_duration_s, 0.1);
  EXPECT_DOUBLE_EQ(diagnostics.initial_duration_s, guide_times.back());
  ASSERT_FALSE(trajectory.empty());
}

TEST(ExpOptimizer, GuideClockSweepKeepsSetupGeometryAndRouteReference) {
  ScopedEnvironmentVariable capture("UAV_NAVIGATION_NOMINAL_SNAPSHOT_DIR",
                                    "/unused/guide-clock-fixture");
  const traj_opt::Config captured_config(PLANNER_EXP_CONFIG_PATH, "exp_traj");
  const auto context =
      std::make_shared<navigation_planner_context::PlannerRuntimeContext>(
          [] { return 12.0; });
  const auto head = makePositionState(0.0);
  const auto tail = makePositionState(20.0);
  const navigation_math::vec_Vec3f guide{
      head.col(0), {10.0, 0.0, 1.0}, tail.col(0)};
  const std::vector<double> captured_times{0.0, 2.0, 4.0};
  const geometry_utils::PolytopeVec captured_corridors{
      makeBox(-1.0, 8.0, -2.0, 2.0, 0.0, 2.0),
      makeBox(4.0, 12.0, -2.0, 2.0, 0.0, 2.0),
      makeBox(8.0, 16.0, -2.0, 2.0, 0.0, 2.0),
      makeBox(12.0, 21.0, -2.0, 2.0, 0.0, 2.0)};
  std::optional<traj_opt::NominalProblemSnapshot> baseline;
  for (const int retry_cap : {
       captured_config.feasibility_retry_max_iterations,
       traj_opt::Config::kMaximumFeasibilityRetryIterations}) {
    for (const double scale : {1.0, 0.5, 2.0, 4.0}) {
      auto config = captured_config;
      config.feasibility_retry_max_iterations = retry_cap;
      traj_opt::ExpTrajOpt optimizer(config, context);
      auto corridors = captured_corridors;
      corridors[2].SetRouteBoundaryContract({10.0, 0.0, 1.0}, 1.0);
      auto times = captured_times;
      for (auto& stamp : times) stamp *= scale;
      geometry_utils::Trajectory trajectory;
      // Setup is the subject of this test, not solver convergence. A failed
      // numerical search must not hide a changed geometric problem.
      optimizer.optimize(head, tail, guide, times, corridors, trajectory, true);
      auto snapshot = optimizer.takeNominalProblemSnapshot();
      ASSERT_TRUE(snapshot.has_value());
      ASSERT_TRUE(snapshot->setup_completed);
      EXPECT_EQ(snapshot->config.feasibility_retry_max_iterations, retry_cap);
      if (!baseline) baseline = *snapshot;
      ASSERT_EQ(snapshot->h_polytopes.size(), baseline->h_polytopes.size());
      for (std::size_t index = 0; index < snapshot->h_polytopes.size(); ++index) {
        EXPECT_TRUE(snapshot->h_polytopes[index].isApprox(
            baseline->h_polytopes[index], 0.0));
      }
      EXPECT_TRUE(snapshot->initial_spatial_variables.isApprox(
          baseline->initial_spatial_variables, 0.0));
      EXPECT_TRUE(snapshot->initial_route_reference_points.isApprox(
          baseline->initial_route_reference_points, 0.0));
      EXPECT_TRUE(snapshot->h_poly_idx.isApprox(baseline->h_poly_idx, 0.0));
      EXPECT_EQ(snapshot->route_boundary_gates, baseline->route_boundary_gates);
      for (std::size_t index = 0; index < snapshot->route_boundary_gates.size();
           ++index) {
        if (snapshot->route_boundary_gates[index] == 0U) continue;
        EXPECT_TRUE(snapshot->route_boundary_points[index].isApprox(
            baseline->route_boundary_points[index], 0.0));
        EXPECT_DOUBLE_EQ(snapshot->route_boundary_radii[index],
                         baseline->route_boundary_radii[index]);
      }
      ASSERT_EQ(snapshot->initial_durations_s.size(),
                baseline->initial_durations_s.size());
      EXPECT_TRUE(snapshot->initial_durations_s.isApprox(
          baseline->initial_durations_s * scale, 1.0e-12));
    }
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

TEST(ExpOptimizer, BaselineUsesBoundedDurationRetryWhenSeedFails) {
  auto config = traj_opt::Config(PLANNER_EXP_CONFIG_PATH, "exp_traj");
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
  // Intentionally too short for the immutable seed's high-order dynamics;
  // the bounded production retry must stretch it before acceptance.
  const std::vector<double> guide_times{0.0, 0.25, 0.5};
  geometry_utils::PolytopeVec corridors{makeBox(-100.0, 100.0, -100.0,
                                                 100.0, 0.0, 10.0)};
  geometry_utils::Trajectory trajectory;

  ASSERT_TRUE(optimizer.optimize(
      head, tail, guide_path, guide_times, corridors, trajectory, true));
  const auto diagnostics = optimizer.diagnostics();
  EXPECT_FALSE(diagnostics.baseline_fallback_to_optimizer);
  EXPECT_TRUE(diagnostics.used_certified_seed);
  EXPECT_GT(diagnostics.corridor_seed_selected_max_duration_scale, 1.0);
  ASSERT_FALSE(trajectory.empty());
  EXPECT_LE(trajectory.getMaxVelRate(), config.max_vel);
  EXPECT_LE(trajectory.getMaxAccRate(), config.max_acc);
  EXPECT_LE(trajectory.getMaxJerRate(), config.max_jerk);
  EXPECT_LE(
      navigation_planning_backend::maximumContinuousCorridorPlaneViolation(
          trajectory, corridors.front().GetPlanes()),
      config.corridor_plane_tolerance_m);
}

TEST(ExpOptimizer, UrgentBaselineSuppressesOptionalRefinementForCertifiedSeed) {
  const traj_opt::Config config(PLANNER_EXP_CONFIG_PATH, "exp_traj");
  const auto planner_context =
      std::make_shared<navigation_planner_context::PlannerRuntimeContext>(
          [] { return 12.0; });
  traj_opt::ExpTrajOpt optimizer(config, planner_context);

  const auto head = makeMovingPositionState(0.0, 2.0);
  const auto tail = makePositionState(8.0);
  navigation_math::vec_E<navigation_math::Vec3f> guide_path;
  guide_path.emplace_back(head.col(0));
  guide_path.emplace_back(navigation_math::Vec3f(4.0, 0.0, 1.0));
  guide_path.emplace_back(tail.col(0));
  const std::vector<double> guide_times{0.0, 2.0, 4.0};
  geometry_utils::PolytopeVec corridors{makeConvexBox()};
  geometry_utils::Trajectory trajectory;

  // Model an already exhausted optional-refinement cutoff. A certified
  // deterministic seed must be returned without entering L-BFGS; this is
  // the urgent-baseline contract that prevents a mandatory fallback from
  // inheriting an expired optional-refinement deadline.
  const auto deadline = std::chrono::steady_clock::now();
  optimizer.setSolveBudget(nullptr,
      std::chrono::duration_cast<std::chrono::nanoseconds>(
          deadline.time_since_epoch()).count());
  const auto result = optimizer.solve(
      head, tail, guide_path, guide_times, corridors, trajectory,
      false, false, true);

  ASSERT_TRUE(result.candidateAvailable());
  EXPECT_EQ(optimizer.diagnostics().refinement_budget_at_entry_us, 0);
  EXPECT_EQ(optimizer.diagnostics().lbfgs_attempt_count, 0);
  EXPECT_TRUE(optimizer.diagnostics().used_certified_seed);
  ASSERT_FALSE(trajectory.empty());
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
  EXPECT_EQ(valid.failure_stage,
            navigation_planning_backend::DeterministicNominalSeedFailureStage::kNone);
  EXPECT_LE(valid.maximum_corridor_violation_m,
            config.corridor_plane_tolerance_m);
  EXPECT_LE(valid.maximum_boundary_residual, 1.0e-8);
  EXPECT_TRUE(std::isfinite(valid.maximum_boundary_roundoff_bound));

  mapping << 1, 0;
  EXPECT_FALSE(navigation_planning_backend::certifyDeterministicNominalSeed(
      seed, corridors, mapping, gates, points, radii, initial, terminal, config).valid);

  mapping << 0, 1;
  auto wrong_terminal = terminal;
  wrong_terminal(0, 0) += 1.0e-4;
  const auto physical_mismatch =
      navigation_planning_backend::certifyDeterministicNominalSeed(
          seed, corridors, mapping, gates, points, radii, initial,
          wrong_terminal, config);
  EXPECT_FALSE(physical_mismatch.valid);
  EXPECT_EQ(physical_mismatch.failure_stage,
            navigation_planning_backend::
                DeterministicNominalSeedFailureStage::kBoundary);
  EXPECT_EQ(physical_mismatch.boundary_failure_location, 2);
  EXPECT_EQ(physical_mismatch.boundary_failure_piece_index, 1);
  EXPECT_EQ(physical_mismatch.boundary_failure_axis, 0);
  EXPECT_EQ(physical_mismatch.boundary_failure_derivative, 0);
  EXPECT_GT(physical_mismatch.boundary_failure_residual,
            physical_mismatch.boundary_failure_roundoff_bound);
}

TEST(DeterministicNominalSeed, BoundaryRoundoffTracksPowerBasisConditioning) {
  constexpr int degree = 7;
  constexpr double root = 23.1;
  Eigen::MatrixXd coefficients = Eigen::MatrixXd::Zero(3, degree + 1);
  const std::array<double, degree + 1> binomial{1.0, 7.0, 21.0, 35.0,
                                               35.0, 21.0, 7.0, 1.0};
  for (int column = 0; column <= degree; ++column) {
    coefficients(0, column) =
        binomial[static_cast<std::size_t>(column)] *
        std::pow(-root, column);
  }
  const geometry_utils::Piece ill_conditioned(root, coefficients);
  const double represented_residual = std::abs(
      ill_conditioned.getPos(root).x());
  const auto roundoff = navigation_planning_backend::pieceStateRoundoffBound(
      ill_conditioned, root);
  EXPECT_GT(represented_residual, 1.0e-8);
  EXPECT_TRUE(roundoff.allFinite());
  EXPECT_LE(represented_residual, roundoff(0, 0));
  EXPECT_LT(roundoff(0, 0), 0.01);
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
  EXPECT_EQ(rejected.failure_stage,
            navigation_planning_backend::DeterministicNominalSeedFailureStage::kCorridor);
  EXPECT_GT(rejected.maximum_corridor_violation_m,
            config.corridor_plane_tolerance_m);
}

TEST(DeterministicNominalSeed, FallbackCopiesExactPreOptimizerTrajectory) {
  const auto initial = makePositionState(0.0);
  const auto terminal = makePositionState(2.0);
  const auto seed_piece =
      navigation_planning_backend::minimumSnapStateTransitionPiece(
          initial, terminal, 3.0);
  ASSERT_TRUE(seed_piece.has_value());
  geometry_utils::Trajectory immutable_seed;
  immutable_seed.emplace_back(*seed_piece);

  auto altered_terminal = terminal;
  altered_terminal(1, 0) = 0.25;
  const auto optimized_piece =
      navigation_planning_backend::minimumSnapStateTransitionPiece(
          initial, altered_terminal, 3.0);
  ASSERT_TRUE(optimized_piece.has_value());
  geometry_utils::Trajectory optimized;
  optimized.emplace_back(*optimized_piece);

  navigation_planning_backend::DeterministicNominalSeedCertificate certificate;
  certificate.valid = true;
  geometry_utils::Trajectory selected;
  EXPECT_EQ(navigation_planning_backend::selectNominalCandidate(
                optimized, false, immutable_seed, certificate, selected),
            navigation_planning_backend::NominalCandidateSelection::kCertifiedSeed);
  ASSERT_EQ(selected.getPieceNum(), immutable_seed.getPieceNum());
  EXPECT_DOUBLE_EQ(selected[0].getDuration(), immutable_seed[0].getDuration());
  EXPECT_TRUE(selected[0].getCoeffMat().isApprox(
      immutable_seed[0].getCoeffMat(), 0.0));

  certificate.valid = false;
  EXPECT_EQ(navigation_planning_backend::selectNominalCandidate(
                optimized, false, immutable_seed, certificate, selected),
            navigation_planning_backend::NominalCandidateSelection::kRejected);
  EXPECT_TRUE(selected.empty());
}

}  // namespace
