#include <algorithm>
#include <cmath>
#include <atomic>
#include <chrono>
#include <limits>
#include <thread>
#include <utility>
#include <vector>

#include <Eigen/Core>
#include <gtest/gtest.h>

#include "data_structure/base/trajectory.h"
#include <navigation_world_model/goal_contract.hpp>
#include "planner_core/planner.hpp"
#include "planner_core/fov_checker.h"
#include "path_search/astar.h"
#include "planner_core/absolute_deadline.hpp"
#include "planner_core/command_time.hpp"
#include "planner_core/ciri.h"
#include "planner_core/corridor_generator.h"
#include "planner_core/guide_endpoint.hpp"
#include "planner_core/guide_vertical_envelope.hpp"
#include "planner_core/kinematic_state_boundary.hpp"
#include "planner_core/replan_contract.hpp"
#include "planner_core/route_regression_certificate.hpp"
#include "planner_core/planning_stage.hpp"
#include "planner_core/trajectory_world_validator.hpp"
#include "traj_opt/trajectory_dynamics.hpp"
#include "traj_opt/nominal_trajectory_optimizer.hpp"
#include "traj_opt/yaw_traj_opt.h"
#include "data_structure/base/polytope.h"
#include "utils/geometry/geometry_utils.h"
#include "utils/geometry/quadrotor_flatness.hpp"
#include "utils/optimization/polynomial_interpolation.h"
#include "utils/optimization/root_finder.h"

namespace navigation_planning_backend {

struct CiriGeometryTestAccess {
  static bool findTangentPlaneOfSphere(
      const Eigen::Vector3d& center, double radius,
      const Eigen::Vector3d& pass_point, const Eigen::Vector3d& seed_point,
      Eigen::Vector4d& plane) {
    return CIRI::findTangentPlaneOfSphere(
        center, radius, pass_point, seed_point, plane);
  }
};

}  // namespace navigation_planning_backend

namespace {

TEST(PlannerTrajectory, FeasibilityRetryActivatesDisabledViolatedDynamicPenalty) {
  navigation_math::VecDf weights(7);
  weights << 5.0e8, 5.0e5, 5.0e5, 0.0, 0.0, 1.0e5, 1.0e5;
  EXPECT_DOUBLE_EQ(traj_opt::feasibilityRetryPenaltyWeight(weights, 3), 5.0e5);
  EXPECT_DOUBLE_EQ(traj_opt::feasibilityRetryPenaltyWeight(weights, 1), 5.0e5);

  weights.segment(1, 3).setZero();
  EXPECT_TRUE(std::isnan(
      traj_opt::feasibilityRetryPenaltyWeight(weights, 3)));
  EXPECT_TRUE(std::isnan(
      traj_opt::feasibilityRetryPenaltyWeight(weights, 0)));
}

class SweepWorld : public navigation_world_model::WorldModelView {
 public:
  double blocked_from_x{std::numeric_limits<double>::infinity()};
  bool endpoints_in_bounds{true};
  navigation_world_model::WorldGeometry geometry() const noexcept override {
    navigation_world_model::WorldGeometry value;
    value.inflated_resolution_m = 0.2;
    value.effective_virtual_ground_m = -10.0;
    value.effective_virtual_ceiling_m = 10.0;
    return value;
  }
  navigation_world_model::WorldSnapshotIdentity identity() const noexcept override {
    return {1, 1, 1, 1};
  }
  navigation_world_model::CellState classify(
      const navigation_world_model::Point3& p,
      navigation_world_model::GridLayer) const noexcept override {
    return p.x() >= blocked_from_x
        ? navigation_world_model::CellState::kOccupied
        : navigation_world_model::CellState::kUnknown;
  }
  bool contains(const navigation_world_model::Point3&) const noexcept override {
    return endpoints_in_bounds;
  }
  navigation_world_model::GridIndex3 positionToIndex(
      const navigation_world_model::Point3& p,
      navigation_world_model::GridLayer) const noexcept override {
    return (p.array() / 0.2).floor().cast<int>();
  }
  navigation_world_model::Point3 indexToPosition(
      const navigation_world_model::GridIndex3& index,
      navigation_world_model::GridLayer) const noexcept override {
    return (index.cast<double>().array() + 0.5).matrix() * 0.2;
  }
  std::optional<navigation_world_model::Point3> nearestNotOccupied(
      const navigation_world_model::Point3& p, navigation_world_model::GridLayer,
      double) const override { return p; }
  bool isSegmentTraversable(
      const navigation_world_model::Point3& a,
      const navigation_world_model::Point3& b,
      navigation_world_model::GridLayer,
      navigation_world_model::UnknownPolicy policy) const noexcept override {
    if (policy != navigation_world_model::UnknownPolicy::kAllowUnknown) return false;
    return std::max(a.x(), b.x()) < blocked_from_x;
  }
  navigation_world_model::AxisAlignedBox clampToLocalBounds(
      const navigation_world_model::AxisAlignedBox& box) const noexcept override { return box; }
  navigation_world_model::PointVector observedOccupiedPoints(
      const navigation_world_model::AxisAlignedBox&) const override { return {}; }
};

class CurvedCellWorld final : public SweepWorld {
 public:
  navigation_world_model::CellState classify(
      const navigation_world_model::Point3& point,
      navigation_world_model::GridLayer) const noexcept override {
    const navigation_world_model::Point3 occupied_cell_center(0.5, 0.3, 3.1);
    return (point - occupied_cell_center).norm() < 1.0e-3
        ? navigation_world_model::CellState::kOccupied
        : navigation_world_model::CellState::kUnknown;
  }
};

class DiagonalNeighborWorld final : public SweepWorld {
 public:
  explicit DiagonalNeighborWorld(
      const navigation_world_model::Point3& unknown_center)
      : unknown_center_(unknown_center) {}

  navigation_world_model::CellState classify(
      const navigation_world_model::Point3& point,
      navigation_world_model::GridLayer) const noexcept override {
    return (point - unknown_center_).norm() < 1.0e-9
        ? navigation_world_model::CellState::kUnknown
        : navigation_world_model::CellState::kKnownFree;
  }

 private:
  navigation_world_model::Point3 unknown_center_;
};

class CountingAstarWorld final : public SweepWorld {
 public:
  navigation_world_model::WorldGeometry geometry() const noexcept override {
    navigation_world_model::WorldGeometry value;
    value.evidence_resolution_m = 1.0;
    value.inflated_resolution_m = 1.0;
    value.local_center_m = Eigen::Vector3d{4.5, 4.5, 1.5};
    value.local_size_m = Eigen::Vector3d{20.0, 20.0, 3.0};
    value.evidence_bounds.global_min_index = Eigen::Vector3i{-5, -5, 0};
    value.evidence_bounds.dimensions = Eigen::Vector3i{20, 20, 3};
    value.inflated_bounds = value.evidence_bounds;
    return value;
  }

  navigation_world_model::CellState classify(
      const navigation_world_model::Point3& point,
      navigation_world_model::GridLayer) const noexcept override {
    return occupied(point)
        ? navigation_world_model::CellState::kOccupied
        : navigation_world_model::CellState::kKnownFree;
  }

  bool contains(
      const navigation_world_model::Point3& point) const noexcept override {
    return point.x() >= -5.0 && point.x() < 15.0 &&
           point.y() >= -5.0 && point.y() < 15.0 &&
           point.z() >= 0.0 && point.z() < 3.0;
  }

  navigation_world_model::GridIndex3 positionToIndex(
      const navigation_world_model::Point3& point,
      navigation_world_model::GridLayer) const noexcept override {
    return point.array().floor().cast<int>();
  }

  navigation_world_model::Point3 indexToPosition(
      const navigation_world_model::GridIndex3& index,
      navigation_world_model::GridLayer) const noexcept override {
    return index.cast<double>() + Eigen::Vector3d::Constant(0.5);
  }

  bool isSegmentTraversable(
      const navigation_world_model::Point3& start,
      const navigation_world_model::Point3& end,
      navigation_world_model::GridLayer layer,
      navigation_world_model::UnknownPolicy) const noexcept override {
    const bool identical_to_previous = previous_query_valid_ &&
        previous_layer_ == layer && previous_start_.isApprox(start, 1.0e-12) &&
        previous_end_.isApprox(end, 1.0e-12);
    if (identical_to_previous) {
      ++consecutive_duplicate_queries_;
    }
    previous_query_valid_ = true;
    previous_layer_ = layer;
    previous_start_ = start;
    previous_end_ = end;
    const bool repeated_undirected_edge = std::any_of(
        queried_edges_.begin(), queried_edges_.end(),
        [&](const auto& edge) {
          return edge.layer == layer &&
              ((edge.start.isApprox(start, 1.0e-12) &&
                edge.end.isApprox(end, 1.0e-12)) ||
               (edge.start.isApprox(end, 1.0e-12) &&
                edge.end.isApprox(start, 1.0e-12)));
        });
    if (repeated_undirected_edge) {
      ++repeated_undirected_edge_queries_;
    } else {
      queried_edges_.push_back({start, end, layer});
    }
    const int samples = std::max(
        1, static_cast<int>(std::ceil((end - start).norm() * 10.0)));
    for (int sample = 0; sample <= samples; ++sample) {
      const double ratio = static_cast<double>(sample) /
                           static_cast<double>(samples);
      if (occupied(start + ratio * (end - start))) {
        return false;
      }
    }
    return true;
  }

  [[nodiscard]] std::uint64_t consecutiveDuplicateQueries() const noexcept {
    return consecutive_duplicate_queries_;
  }

  [[nodiscard]] std::uint64_t repeatedUndirectedEdgeQueries() const noexcept {
    return repeated_undirected_edge_queries_;
  }

 private:
  struct QueriedEdge {
    navigation_world_model::Point3 start;
    navigation_world_model::Point3 end;
    navigation_world_model::GridLayer layer;
  };

  static bool occupied(const navigation_world_model::Point3& point) noexcept {
    return point.x() >= 2.0 && point.x() < 3.0 &&
           point.y() >= -0.5 && point.y() < 1.5;
  }

  mutable bool previous_query_valid_{false};
  mutable navigation_world_model::GridLayer previous_layer_{
      navigation_world_model::GridLayer::kEvidence};
  mutable navigation_world_model::Point3 previous_start_{
      navigation_world_model::Point3::Zero()};
  mutable navigation_world_model::Point3 previous_end_{
      navigation_world_model::Point3::Zero()};
  mutable std::uint64_t consecutive_duplicate_queries_{0U};
  mutable std::vector<QueriedEdge> queried_edges_;
  mutable std::uint64_t repeated_undirected_edge_queries_{0U};
};

geometry_utils::Trajectory linearTrajectory(double duration, double start_wall_time) {
  Eigen::MatrixXd coefficients = Eigen::MatrixXd::Zero(3, 8);
  coefficients(0, 6) = 1.0;
  coefficients(2, 7) = 3.0;
  geometry_utils::Trajectory result({duration}, {coefficients});
  result.start_WT = start_wall_time;
  return result;
}

geometry_utils::Trajectory stationaryTrajectory(double duration, double start_wall_time) {
  Eigen::MatrixXd coefficients = Eigen::MatrixXd::Zero(3, 8);
  coefficients(0, 7) = 2.0;
  coefficients(2, 7) = 3.0;
  geometry_utils::Trajectory result({duration}, {coefficients});
  result.start_WT = start_wall_time;
  return result;
}
}  // namespace

TEST(CiriGeometry, UsesNonCollinearSeedDirectionWithoutArtificialPerturbation) {
  Eigen::Vector4d plane;
  ASSERT_TRUE(navigation_planning_backend::CiriGeometryTestAccess::
                  findTangentPlaneOfSphere(
                      Eigen::Vector3d::Zero(), 1.0,
                      Eigen::Vector3d(2.0, 0.0, 0.0),
                      Eigen::Vector3d(0.0, 0.005, 0.0), plane));

  // The positive-y seed selects the first tangent branch. A fixed
  // pass_point - pass_point test would always perturb the seed by 0.01 m,
  // crossing this deliberately narrow branch boundary and flipping plane.y.
  EXPECT_NEAR(plane.x(), 0.5, 1e-12);
  EXPECT_NEAR(plane.y(), std::sqrt(3.0) / 2.0, 1e-12);
  EXPECT_NEAR(plane.z(), 0.0, 1e-12);
  EXPECT_NEAR(plane.w(), -1.0, 1e-12);
}

TEST(CiriGeometry, RejectsInvalidConfigurationBeforeNumericalWork) {
  navigation_planning_backend::CIRI ciri;
  EXPECT_THROW(ciri.setupParams(0.0, 1), std::invalid_argument);
  EXPECT_THROW(ciri.setupParams(0.5, 0), std::invalid_argument);
  EXPECT_THROW(ciri.setupParams(std::numeric_limits<double>::quiet_NaN(), 1),
               std::invalid_argument);
}

TEST(CiriGeometry, RejectsDegenerateSeedSegmentBeforeEllipsoidConstruction) {
  navigation_planning_backend::CIRI ciri;
  ciri.setupParams(0.35, 1);

  Eigen::MatrixX4d bounds(6, 4);
  bounds <<
      1.0, 0.0, 0.0, -2.0,
     -1.0, 0.0, 0.0, -2.0,
      0.0, 1.0, 0.0, -2.0,
      0.0, -1.0, 0.0, -2.0,
      0.0, 0.0, 1.0, -2.0,
      0.0, 0.0, -1.0, -2.0;
  Eigen::Matrix3Xd obstacles(3, 1);
  obstacles.col(0) = Eigen::Vector3d(1.5, 1.5, 1.5);

  EXPECT_EQ(ciri.comvexDecomposition(
                bounds, obstacles, Eigen::Vector3d::Zero(),
                Eigen::Vector3d::Zero()),
            navigation_math::INIT_ERROR);
}

TEST(PlannerTrajectory, PartialSlicePreservesPieceLocalTimeAndContinuity) {
  std::vector<double> durations{1.0, 1.0};
  std::vector<Eigen::MatrixXd> coefficients;
  for (double offset : {0.0, 1.0}) {
    Eigen::MatrixXd matrix = Eigen::MatrixXd::Zero(3, 6);
    matrix(0, 4) = 1.0;  // x(t) = t + offset
    matrix(0, 5) = offset;
    matrix(2, 5) = 3.0;
    coefficients.push_back(matrix);
  }

  geometry_utils::Trajectory source(durations, coefficients);
  geometry_utils::Trajectory partial;
  ASSERT_TRUE(source.getPartialTrajectoryByTime(0.25, 1.5, partial));
  ASSERT_EQ(partial.getPieceNum(), 2);
  EXPECT_NEAR(partial.getTotalDuration(), 1.25, 1e-12);
  EXPECT_NEAR(partial.getPos(0.0).x(), 0.25, 1e-12);
  EXPECT_NEAR(partial.getPos(0.75).x(), 1.0, 1e-12);
  EXPECT_NEAR(partial.getPos(partial.getTotalDuration()).x(), 1.5, 1e-12);
  EXPECT_NEAR(partial.getVel(0.0).x(), 1.0, 1e-12);
  EXPECT_NEAR(partial.getVel(partial.getTotalDuration()).x(), 1.0, 1e-12);
  EXPECT_TRUE(partial.getPos(0.0).allFinite());
}

TEST(PlannerTrajectory, PartialSliceUsesHalfOpenStartAtPieceBoundary) {
  std::vector<double> durations{1.0, 1.0};
  std::vector<Eigen::MatrixXd> coefficients;
  for (double offset : {0.0, 1.0}) {
    Eigen::MatrixXd matrix = Eigen::MatrixXd::Zero(3, 6);
    matrix(0, 4) = 1.0;
    matrix(0, 5) = offset;
    coefficients.push_back(matrix);
  }

  geometry_utils::Trajectory source(durations, coefficients);
  geometry_utils::Trajectory partial;
  ASSERT_TRUE(source.getPartialTrajectoryByTime(1.0, 1.5, partial));
  ASSERT_EQ(partial.getPieceNum(), 1);
  EXPECT_DOUBLE_EQ(partial[0].getDuration(), 0.5);
  EXPECT_DOUBLE_EQ(partial.start_WT, source.start_WT + 1.0);
  EXPECT_NEAR(partial.getPos(0.0).x(), 1.0, 1.0e-12);
  EXPECT_NEAR(partial.getPos(0.5).x(), 1.5, 1.0e-12);
  EXPECT_TRUE(partial.getState(0.0).allFinite());
}

TEST(PlannerTrajectory, PartialSliceEndingAtPieceBoundaryKeepsLocalDuration) {
  std::vector<double> durations{1.0, 1.0};
  std::vector<Eigen::MatrixXd> coefficients;
  for (double offset : {0.0, 1.0}) {
    Eigen::MatrixXd matrix = Eigen::MatrixXd::Zero(3, 6);
    matrix(0, 4) = 1.0;
    matrix(0, 5) = offset;
    coefficients.push_back(matrix);
  }

  geometry_utils::Trajectory source(durations, coefficients);
  geometry_utils::Trajectory partial;
  ASSERT_TRUE(source.getPartialTrajectoryByTime(0.0, 1.0, partial));
  ASSERT_EQ(partial.getPieceNum(), 1);
  EXPECT_DOUBLE_EQ(partial[0].getDuration(), 1.0);
  EXPECT_NEAR(partial.getPos(partial.getTotalDuration()).x(), 1.0, 1.0e-12);
  EXPECT_TRUE(partial.getState(partial.getTotalDuration()).allFinite());
}

TEST(PlannerTrajectory, PartialSliceMayCrossMixedDegreeCommandBoundary) {
  Eigen::MatrixXd main_coefficients = Eigen::MatrixXd::Zero(3, 8);
  main_coefficients(0, 6) = 1.0;
  Eigen::MatrixXd backup_coefficients = Eigen::MatrixXd::Zero(3, 6);
  backup_coefficients(0, 4) = 0.5;
  geometry_utils::Trajectory command(
      {0.243, 1.0}, {main_coefficients, backup_coefficients});
  command.start_WT = 10.0;

  geometry_utils::Trajectory prefix;
  ASSERT_TRUE(command.getPartialTrajectoryByTime(0.188, 0.388, prefix));
  ASSERT_EQ(prefix.getPieceNum(), 2);
  EXPECT_EQ(prefix[0].getDegree(), 7);
  EXPECT_EQ(prefix[1].getDegree(), 5);
  EXPECT_NEAR(prefix[0].getDuration(), 0.055, 1.0e-12);
  EXPECT_NEAR(prefix[1].getDuration(), 0.145, 1.0e-12);
  EXPECT_NEAR(prefix.getTotalDuration(), 0.2, 1.0e-12);
  EXPECT_TRUE(prefix.getState(0.0).isApprox(command.getState(0.188), 1.0e-12));
  EXPECT_TRUE(prefix.getState(0.2).isApprox(command.getState(0.388), 1.0e-12));
}

TEST(PlannerTrajectory, TrajectoryRejectsMismatchedDurationAndCoefficientInputs) {
  const std::vector<double> durations{1.0};
  const std::vector<Eigen::MatrixXd> coefficients;
  EXPECT_THROW((geometry_utils::Trajectory(durations, coefficients)),
               std::invalid_argument);
}

TEST(PlannerTrajectory, PieceAccessRejectsOutOfRangeIndices) {
  Eigen::MatrixXd coefficients = Eigen::MatrixXd::Zero(3, 6);
  geometry_utils::Trajectory trajectory({1.0}, {coefficients});

  EXPECT_THROW(static_cast<void>(trajectory[-1]), std::out_of_range);
  EXPECT_THROW(static_cast<void>(trajectory[1]), std::out_of_range);
}

TEST(PlannerTrajectory, CumulativeTimeOverflowFailsClosed) {
  Eigen::MatrixXd coefficients = Eigen::MatrixXd::Zero(3, 6);
  const double large_duration = std::numeric_limits<double>::max() * 0.75;
  geometry_utils::Trajectory trajectory(
      {large_duration, large_duration}, {coefficients, coefficients});

  EXPECT_FALSE(std::isfinite(trajectory.getTotalDuration()));
  EXPECT_FALSE(std::isfinite(trajectory.getWaypointTT(1)));

  trajectory.start_WT = std::numeric_limits<double>::max();
  geometry_utils::Trajectory partial;
  EXPECT_FALSE(trajectory.getPartialTrajectoryByID(1, -1, partial));
  EXPECT_TRUE(partial.empty());
}

TEST(PlannerTrajectory, PartialTimeSliceRejectsWallTimeOverflow) {
  Eigen::MatrixXd coefficients = Eigen::MatrixXd::Zero(3, 6);
  const double maximum = std::numeric_limits<double>::max();
  geometry_utils::Trajectory trajectory({maximum * 0.75}, {coefficients});
  trajectory.start_WT = maximum * 0.75;
  geometry_utils::Trajectory partial;

  EXPECT_FALSE(trajectory.getPartialTrajectoryByTime(
      maximum * 0.5, maximum * 0.6, partial));
  EXPECT_TRUE(partial.empty());
}

TEST(PlannerTrajectory, InvalidTrajectoryEvaluationFailsClosed) {
  geometry_utils::Trajectory empty;
  navigation_math::StatePVAJ state = navigation_math::StatePVAJ::Zero();
  EXPECT_FALSE(empty.getState(0.0, state));
  EXPECT_FALSE(empty.getPos(0.0).allFinite());
  EXPECT_FALSE(empty.getState(0.0).allFinite());
  EXPECT_FALSE(std::isfinite(empty.getWaypointTT(-1)));

  Eigen::MatrixXd coefficients = Eigen::MatrixXd::Zero(3, 6);
  geometry_utils::Trajectory malformed({0.0}, {coefficients});
  EXPECT_FALSE(malformed.getState(0.0, state));
  EXPECT_FALSE(malformed.getPos(0.0).allFinite());
}

TEST(PlannerTrajectory, PieceRejectsMalformedAndLowDegreeRateInputs) {
  const geometry_utils::Piece constant_piece(
      1.0, Eigen::MatrixXd::Zero(3, 1));
  EXPECT_TRUE(constant_piece.getPos(0.5).isApprox(Eigen::Vector3d::Zero()));
  EXPECT_DOUBLE_EQ(constant_piece.getMaxVelRate(), 0.0);
  EXPECT_DOUBLE_EQ(constant_piece.getMaxAccRate(), 0.0);
  EXPECT_DOUBLE_EQ(constant_piece.getMaxJerRate(), 0.0);
  EXPECT_TRUE(constant_piece.checkMaxVelRate(1.0));
  EXPECT_TRUE(constant_piece.checkMaxAccRate(1.0));
  EXPECT_FALSE(constant_piece.getPos(
      std::numeric_limits<double>::quiet_NaN()).allFinite());
  EXPECT_EQ(constant_piece.normalizeVelCoeffMat().cols(), 0);
  EXPECT_EQ(constant_piece.normalizeAccCoeffMat().cols(), 0);
  EXPECT_EQ(constant_piece.normalizeJerCoeffMat().cols(), 0);

  const geometry_utils::Piece malformed_piece(
      1.0, Eigen::MatrixXd::Zero(2, 6));
  EXPECT_FALSE(malformed_piece.getPos(0.0).allFinite());
  EXPECT_FALSE(std::isfinite(malformed_piece.getMaxVelRate()));
  EXPECT_FALSE(std::isfinite(malformed_piece.getMaxAccRate()));
  EXPECT_FALSE(std::isfinite(malformed_piece.getMaxJerRate()));
  EXPECT_FALSE(malformed_piece.checkMaxVelRate(1.0));
  EXPECT_FALSE(malformed_piece.checkMaxAccRate(1.0));
  EXPECT_FALSE(constant_piece.checkMaxVelRate(
      std::numeric_limits<double>::quiet_NaN()));
  EXPECT_FALSE(constant_piece.checkMaxAccRate(-1.0));
}

TEST(PlannerTrajectory, PieceRateCertificateBoundsRootSearchAtInitialBoundary) {
  Eigen::MatrixXd coefficients = Eigen::MatrixXd::Zero(3, 3);
  coefficients(0, 0) = 0.5;
  coefficients(0, 1) = 0.0625;
  const geometry_utils::Piece piece(1.0, coefficients);

  EXPECT_NEAR(piece.getMaxVelRate(), 1.0625, 1.0e-9);
}

TEST(PlannerTrajectory, PieceRateRootSearchExpandsAwayFromExecutionInterval) {
  // v(t) = t + 0.0625 has a squared-speed stationary point at the initial
  // auxiliary left bound. The search must move farther left, not converge
  // toward the legitimate t=0 execution boundary.
  Eigen::MatrixXd coefficients = Eigen::MatrixXd::Zero(3, 3);
  coefficients(0, 0) = 0.5;
  coefficients(0, 1) = 0.0625;
  const geometry_utils::Piece boundary_root_piece(1.0, coefficients);
  EXPECT_TRUE(std::isfinite(boundary_root_piece.getMaxVelRate()));

  // A normal stop starts with zero acceleration, so d|v|^2/dt is zero at
  // t=0. Extrema certification must remain finite for that common boundary.
  navigation_math::StatePVAJ initial = navigation_math::StatePVAJ::Zero();
  initial.col(1) = Eigen::Vector3d{1.0, 0.0, 0.0};
  const auto stop = navigation_planning_backend::minimumSnapStopPiece(initial, 1.0);
  EXPECT_TRUE(std::isfinite(stop.getMaxVelRate()));
  EXPECT_TRUE(std::isfinite(stop.getMaxAccRate()));
  EXPECT_TRUE(std::isfinite(stop.getMaxJerRate()));
}

TEST(PlannerTrajectory, RootFinderRejectsDegenerateNumericalInputs) {
  const Eigen::VectorXd empty;
  EXPECT_EQ(math_utils::RootFinder::polyConv(empty, Eigen::VectorXd::Ones(1)).size(), 0);
  EXPECT_EQ(math_utils::RootFinder::polySqr(empty).size(), 0);

  const Eigen::VectorXd non_finite = Eigen::VectorXd::Constant(
      2, std::numeric_limits<double>::quiet_NaN());
  EXPECT_TRUE(math_utils::RootFinder::solvePolynomial(
      non_finite, 0.0, 1.0, 1.0e-6).empty());
  EXPECT_EQ(math_utils::RootFinder::countRoots(non_finite, 0.0, 1.0), -1);
  EXPECT_TRUE(math_utils::RootFinder::solvePolynomial(
      Eigen::VectorXd::Ones(2), 1.0, 0.0, 1.0e-6).empty());
  EXPECT_EQ(math_utils::RootFinder::countRoots(
      Eigen::VectorXd::Ones(2), 1.0, 0.0), -1);
  EXPECT_EQ(math_utils::RootFinder::countRoots(Eigen::VectorXd::Ones(1), 0.0, 1.0), 0);

  // x^6 + 1 has no real roots.  The non-isolation companion-matrix path
  // must reject both signs of the complex imaginary component.
  Eigen::VectorXd complex_only = Eigen::VectorXd::Zero(7);
  complex_only(0) = 1.0;
  complex_only(6) = 1.0;
  EXPECT_TRUE(math_utils::RootFinder::solvePolynomial(
      complex_only, -2.0, 2.0, 1.0e-6, false).empty());
}

TEST(PlannerTrajectory, FovPlanesHaveNoUninitializedRowsAndAnglesArePerInstance) {
  Eigen::MatrixX4d planes;
  std::vector<Eigen::Matrix3d> points;
  geometry_utils::GetFovPlanes(Eigen::Matrix3d::Identity(), Eigen::Vector3d::Zero(),
                               planes, points);
  ASSERT_EQ(planes.rows(), 8);
  ASSERT_EQ(points.size(), 8U);
  EXPECT_TRUE(planes.allFinite());

  navigation_planning_backend::FOVChecker narrow(
      navigation_planning_backend::OMNI, 0.0, -5.0, 35.0);
  navigation_planning_backend::FOVChecker wide(
      navigation_planning_backend::OMNI, 0.0, -35.0, 35.0);
  Eigen::MatrixX4d narrow_planes;
  Eigen::MatrixX4d wide_planes;
  narrow.getFovCheckPlane(Eigen::Matrix3d::Identity(), Eigen::Vector3d::Zero(),
                          narrow_planes);
  wide.getFovCheckPlane(Eigen::Matrix3d::Identity(), Eigen::Vector3d::Zero(),
                        wide_planes);
  ASSERT_TRUE(narrow_planes.allFinite());
  ASSERT_TRUE(wide_planes.allFinite());
  EXPECT_GT((narrow_planes - wide_planes).norm(), 1.0e-3);
}

TEST(PlannerTrajectory, FlatnessMapHasSafeDefaultsAndRejectsInvalidReset) {
  flatness::FlatnessMap map;
  double thrust = 0.0;
  Eigen::Vector4d quaternion;
  Eigen::Vector3d angular_rate;
  map.forward(Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(),
              Eigen::Vector3d::Zero(), 0.0, 0.0, thrust, quaternion, angular_rate);
  EXPECT_TRUE(std::isfinite(thrust));
  EXPECT_TRUE(quaternion.allFinite());
  EXPECT_TRUE(angular_rate.allFinite());
  EXPECT_THROW(map.reset(0.0, 9.81, 0.0, 0.0, 0.0, 1.0e-4), std::invalid_argument);
}

TEST(PlannerTrajectory, PartialTrajectoryByIdAcceptsExclusiveEndSentinel) {
  Eigen::MatrixXd coefficients = Eigen::MatrixXd::Zero(3, 6);
  coefficients(0, 4) = 1.0;
  geometry_utils::Trajectory source({1.0, 2.0}, {coefficients, coefficients});
  source.start_WT = 10.0;
  geometry_utils::Trajectory partial;

  ASSERT_TRUE(source.getPartialTrajectoryByID(1, -1, partial));
  ASSERT_EQ(partial.getPieceNum(), 1);
  EXPECT_DOUBLE_EQ(partial.start_WT, 11.0);
  EXPECT_DOUBLE_EQ(partial.getTotalDuration(), 2.0);
  EXPECT_FALSE(source.getPartialTrajectoryByID(0, 3, partial));
}

TEST(PlannerTrajectory, SweepUsesNextPieceAtExactBoundary) {
  Eigen::MatrixXd coefficients = Eigen::MatrixXd::Zero(3, 8);
  coefficients(0, 6) = 1.0;
  coefficients(2, 7) = 3.0;
  geometry_utils::Trajectory position({0.5, 0.5}, {coefficients, coefficients});
  geometry_utils::Trajectory yaw({0.5, 0.5}, {coefficients, coefficients});
  position.start_WT = yaw.start_WT = 10.0;

  navigation_planning_backend::CandidateCommandBundle candidate;
  candidate.position = position;
  candidate.yaw = yaw;
  candidate.start_wall_time = 10.0;
  candidate.roles = {
      {0.0, 1.0, navigation_planning_backend::CandidateTrajectoryRole::MAIN},
  };

  const auto location = navigation_planning_backend::locatePieceForSweep(
      candidate.position, 0.5);
  ASSERT_TRUE(location);
  EXPECT_EQ(location->index, 1);
  EXPECT_DOUBLE_EQ(location->local_time, 0.0);
  EXPECT_TRUE(navigation_planning_backend::validateExecutableCandidate(
      SweepWorld{}, candidate, 10.5,
      navigation_world_model::UnknownPolicy::kAllowUnknown).valid);
}

TEST(PlannerTrajectory, SweepAdvancesAcrossNumericalPieceBoundary) {
  Eigen::MatrixXd coefficients = Eigen::MatrixXd::Zero(3, 8);
  coefficients(0, 6) = 1.0;
  coefficients(2, 7) = 3.0;
  geometry_utils::Trajectory position({0.2, 0.8}, {coefficients, coefficients});
  geometry_utils::Trajectory yaw({0.2, 0.8}, {coefficients, coefficients});
  position.start_WT = yaw.start_WT = 10.0;

  navigation_planning_backend::CandidateCommandBundle candidate;
  candidate.position = position;
  candidate.yaw = yaw;
  candidate.start_wall_time = 10.0;
  candidate.roles = {
      {0.0, 1.0, navigation_planning_backend::CandidateTrajectoryRole::MAIN},
  };

  const double just_before_boundary = 0.2 - 1.0e-15;
  const auto location = navigation_planning_backend::locatePieceForSweep(
      candidate.position, just_before_boundary);
  ASSERT_TRUE(location);
  EXPECT_EQ(location->index, 1);
  EXPECT_DOUBLE_EQ(location->local_time, 0.0);
  EXPECT_TRUE(navigation_planning_backend::validateExecutableCandidate(
      SweepWorld{}, candidate, 10.2 - 1.0e-15,
      navigation_world_model::UnknownPolicy::kAllowUnknown).valid);
}

TEST(PlannerTrajectory, CommandTrajectoryTimePreservesEstablishedSamplingSemantics) {
  const auto before = navigation_planning_backend::commandTrajectoryTime(9.75, 10.0, 2.0);
  EXPECT_FALSE(before.finished);
  EXPECT_DOUBLE_EQ(before.trajectory_time_s, -0.25);

  const auto start = navigation_planning_backend::commandTrajectoryTime(10.0, 10.0, 2.0);
  EXPECT_FALSE(start.finished);
  EXPECT_DOUBLE_EQ(start.trajectory_time_s, 0.0);

  const auto middle = navigation_planning_backend::commandTrajectoryTime(11.25, 10.0, 2.0);
  EXPECT_FALSE(middle.finished);
  EXPECT_DOUBLE_EQ(middle.trajectory_time_s, 1.25);

  const auto finished = navigation_planning_backend::commandTrajectoryTime(12.1, 10.0, 2.0);
  EXPECT_TRUE(finished.finished);
  EXPECT_DOUBLE_EQ(finished.trajectory_time_s, 2.0);
}

TEST(PlannerTrajectory, HotReplanUsesExecutableCommandClock) {
  constexpr double history_start_wall_time_s = 41.116;
  constexpr double command_start_wall_time_s = 43.520;
  constexpr double replan_start_wall_time_s = 43.720;
  constexpr double forward_time_s = 0.18;
  constexpr double emergency_duration_s = 1.468;

  const auto executable = navigation_planning_backend::hotReplanWindow(
      replan_start_wall_time_s, command_start_wall_time_s,
      forward_time_s, emergency_duration_s);
  ASSERT_TRUE(executable.valid);
  EXPECT_FALSE(executable.reaches_command_end);
  EXPECT_NEAR(executable.start_tt_s, 0.2, 1.0e-12);
  EXPECT_NEAR(executable.state_tt_s, 0.38, 1.0e-12);

  const auto stale_history = navigation_planning_backend::hotReplanWindow(
      replan_start_wall_time_s, history_start_wall_time_s,
      forward_time_s, emergency_duration_s);
  ASSERT_TRUE(stale_history.valid);
  EXPECT_TRUE(stale_history.reaches_command_end);
}

TEST(PlannerTrajectory, HotReplanClockFailsClosedAtInvalidBoundaries) {
  const double nan = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(navigation_planning_backend::hotReplanWindow(
      nan, 10.0, 0.18, 1.0).valid);
  EXPECT_FALSE(navigation_planning_backend::hotReplanWindow(
      9.9, 10.0, 0.18, 1.0).valid);
  EXPECT_FALSE(navigation_planning_backend::hotReplanWindow(
      10.1, 10.0, -0.18, 1.0).valid);

  const auto ended = navigation_planning_backend::hotReplanWindow(
      10.9, 10.0, 0.18, 1.0);
  ASSERT_TRUE(ended.valid);
  EXPECT_TRUE(ended.reaches_command_end);
}

TEST(PlannerTrajectory, InheritedBackupIntervalClipsToCandidateClock) {
  const auto window = navigation_planning_backend::hotReplanWindow(
      43.720, 43.520, 0.18, 1.468);
  ASSERT_TRUE(window.valid);

  const auto already_on_backup =
      navigation_planning_backend::inheritedBackupInterval(0.0, window, 2.0);
  ASSERT_TRUE(already_on_backup.valid);
  ASSERT_TRUE(already_on_backup.present);
  EXPECT_DOUBLE_EQ(already_on_backup.begin_tt_s, 0.0);
  EXPECT_NEAR(already_on_backup.end_tt_s, 0.18, 1.0e-12);

  const auto enters_backup =
      navigation_planning_backend::inheritedBackupInterval(0.3, window, 2.0);
  ASSERT_TRUE(enters_backup.valid);
  ASSERT_TRUE(enters_backup.present);
  EXPECT_NEAR(enters_backup.begin_tt_s, 0.1, 1.0e-12);
  EXPECT_NEAR(enters_backup.end_tt_s, 0.18, 1.0e-12);

  EXPECT_FALSE(navigation_planning_backend::inheritedBackupInterval(
      0.0, window, 0.1).valid);
}

TEST(PlannerTrajectory, StateTransitionPiecePreservesCompletePvajBoundary) {
  navigation_math::StatePVAJ initial = navigation_math::StatePVAJ::Zero();
  initial.col(0) = Eigen::Vector3d(1.0, -2.0, 3.0);
  initial.col(1) = Eigen::Vector3d(2.0, 0.5, -0.25);
  initial.col(2) = Eigen::Vector3d(-0.4, 0.2, 0.3);
  initial.col(3) = Eigen::Vector3d(0.1, -0.2, 0.05);

  navigation_math::StatePVAJ terminal = navigation_math::StatePVAJ::Zero();
  terminal.col(0) = Eigen::Vector3d(2.5, -1.0, 2.8);
  terminal.col(1) = Eigen::Vector3d(1.1, 0.8, 0.0);
  terminal.col(2) = Eigen::Vector3d(0.2, -0.1, -0.15);
  terminal.col(3) = Eigen::Vector3d(-0.05, 0.1, 0.02);

  const auto piece = navigation_planning_backend::minimumSnapStateTransitionPiece(
      initial, terminal, 0.8);
  ASSERT_TRUE(piece.has_value());
  EXPECT_TRUE(piece->getState(0.0).isApprox(initial, 1.0e-9));
  EXPECT_TRUE(piece->getState(0.8).isApprox(terminal, 1.0e-8));
}

TEST(PlannerTrajectory, StateTransitionPieceRejectsInvalidBoundary) {
  navigation_math::StatePVAJ finite = navigation_math::StatePVAJ::Zero();
  EXPECT_FALSE(navigation_planning_backend::minimumSnapStateTransitionPiece(
      finite, finite, 0.0).has_value());

  auto non_finite = finite;
  non_finite(0, 0) = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(navigation_planning_backend::minimumSnapStateTransitionPiece(
      non_finite, finite, 0.8).has_value());
}

TEST(PlannerTrajectory, YawHandoffRequiresRateAndAccelerationCertificate) {
  navigation_math::StatePVAJ initial = navigation_math::StatePVAJ::Zero();
  navigation_math::StatePVAJ terminal = navigation_math::StatePVAJ::Zero();
  terminal(0, 0) = 0.35;

  EXPECT_FALSE(navigation_planning_backend::
      minimumSnapStateTransitionPieceWithinRateAccelerationLimits(
          initial, terminal, 0.5, 1.0, 0.3));

  const auto certified = navigation_planning_backend::
      minimumSnapStateTransitionPieceWithinRateAccelerationLimits(
          initial, terminal, 5.0, 1.0, 0.3);
  ASSERT_TRUE(certified.has_value());
  EXPECT_LE(certified->getMaxVelRate(), 1.0 + 1.0e-6);
  EXPECT_LE(certified->getMaxAccRate(), 0.3 + 1.0e-6);
}

TEST(PlannerTrajectory, OnlySuccessfulExpResultMayBuildAndCommitNewCandidate) {
  EXPECT_TRUE(navigation_planning_backend::expResultMayBuildCommandCandidate(navigation_math::SUCCESS));
  EXPECT_FALSE(navigation_planning_backend::expResultMayBuildCommandCandidate(navigation_math::NO_NEED));
  EXPECT_FALSE(navigation_planning_backend::expResultMayBuildCommandCandidate(navigation_math::FAILED));
  EXPECT_FALSE(navigation_planning_backend::expResultMayBuildCommandCandidate(navigation_math::NEW_TRAJ));
  EXPECT_FALSE(navigation_planning_backend::expResultMayBuildCommandCandidate(navigation_math::EMER));

  EXPECT_TRUE(navigation_planning_backend::backupResultMayBuildCommandCandidate(navigation_math::SUCCESS));
  EXPECT_TRUE(navigation_planning_backend::backupResultMayBuildCommandCandidate(navigation_math::NO_NEED));
  EXPECT_TRUE(navigation_planning_backend::backupResultMayBuildCommandCandidate(navigation_math::FINISH));
  EXPECT_FALSE(navigation_planning_backend::backupResultMayBuildCommandCandidate(navigation_math::FAILED));
  EXPECT_FALSE(navigation_planning_backend::backupResultMayBuildCommandCandidate(navigation_math::OPT_FAILED));
  EXPECT_FALSE(navigation_planning_backend::backupResultMayBuildCommandCandidate(navigation_math::EMER));
}

TEST(PlannerTrajectory, VisibleReplacementDoesNotEraseFutureBackupSuffix) {
  EXPECT_TRUE(navigation_planning_backend::shouldRetainBackupCapableCommand(
      false, true));
  EXPECT_FALSE(navigation_planning_backend::shouldRetainBackupCapableCommand(
      true, true));
  EXPECT_FALSE(navigation_planning_backend::shouldRetainBackupCapableCommand(
      false, false));
}

TEST(PlannerTrajectory, FlatnessGateRejectsExcessBodyRateAndThrust) {
  traj_opt::Config config;
  config.mass = 1.0;
  config.grav = 9.81;
  config.dh = 0.0;
  config.dv = 0.0;
  config.cp = 0.0;
  config.v_eps = 1.0e-4;
  config.max_omg = 2.0;
  config.min_acc_thr = 6.0;
  config.max_acc_thr = 15.0;
  config.quadrotot_flatness.reset(config.mass, config.grav, config.dh, config.dv,
                                  config.cp, config.v_eps);

  Eigen::MatrixXd hover_coefficients = Eigen::MatrixXd::Zero(3, 6);
  hover_coefficients(2, 5) = 3.0;
  geometry_utils::Trajectory hover({1.0}, {hover_coefficients});
  traj_opt::TrajectoryDynamicReport hover_report;
  EXPECT_TRUE(traj_opt::trajectorySatisfiesFlatnessEnvelope(
      hover, config, &hover_report));
  EXPECT_NEAR(hover_report.minimum_thrust_n, 9.81, 1.0e-9);

  // For this representable duration, duration * intervals / intervals rounds
  // one ulp above duration. The hard gate must sample the exact endpoint.
  geometry_utils::Trajectory rounded_endpoint_hover(
      {0.8754185709044305}, {hover_coefficients});
  traj_opt::TrajectoryDynamicReport rounded_endpoint_report;
  EXPECT_TRUE(traj_opt::trajectorySatisfiesFlatnessEnvelope(
      rounded_endpoint_hover, config, &rounded_endpoint_report, 0.005));
  EXPECT_TRUE(rounded_endpoint_report.finite);
  EXPECT_EQ(rounded_endpoint_report.nonfinite_mask, 0U);

  // x(t)=10*t^3 creates 60 m/s^3 jerk at t=0, which maps to a body rate far
  // above the configured envelope even though thrust remains finite.
  Eigen::MatrixXd aggressive_coefficients = Eigen::MatrixXd::Zero(3, 6);
  aggressive_coefficients(0, 2) = 10.0;
  aggressive_coefficients(2, 5) = 3.0;
  geometry_utils::Trajectory aggressive({1.0}, {aggressive_coefficients});
  traj_opt::TrajectoryDynamicReport aggressive_report;
  EXPECT_FALSE(traj_opt::trajectorySatisfiesFlatnessEnvelope(
      aggressive, config, &aggressive_report));
  EXPECT_GT(aggressive_report.maximum_body_rate_rad_s, config.max_omg);

  Eigen::MatrixXd thrust_coefficients = Eigen::MatrixXd::Zero(3, 6);
  thrust_coefficients(2, 3) = 10.0;  // z(t)=10*t^2, constant 20 m/s^2 up.
  geometry_utils::Trajectory excessive_thrust({1.0}, {thrust_coefficients});
  traj_opt::TrajectoryDynamicReport thrust_report;
  EXPECT_FALSE(traj_opt::trajectorySatisfiesFlatnessEnvelope(
      excessive_thrust, config, &thrust_report));
  EXPECT_GT(thrust_report.maximum_thrust_n, config.max_acc_thr);

  Eigen::MatrixXd yaw_coefficients = Eigen::MatrixXd::Zero(3, 6);
  yaw_coefficients(0, 4) = 3.0;  // Constant 3 rad/s yaw rate.
  geometry_utils::Trajectory fast_yaw({1.0}, {yaw_coefficients});
  traj_opt::TrajectoryDynamicReport yaw_report;
  EXPECT_FALSE(traj_opt::trajectorySatisfiesFlatnessEnvelope(
      hover, config, &yaw_report, 0.01, &fast_yaw));
  EXPECT_GT(yaw_report.maximum_body_rate_rad_s, config.max_omg);
}

TEST(PlannerTrajectory, EmergencyBundleIsAtomicallyOwnedByBackup) {
  Eigen::MatrixXd position_coefficients = Eigen::MatrixXd::Zero(3, 8);
  position_coefficients(0, 6) = 1.0;
  position_coefficients(2, 7) = 3.0;
  geometry_utils::Trajectory position({1.0}, {position_coefficients});
  position.start_WT = 42.0;

  Eigen::MatrixXd yaw_coefficients = Eigen::MatrixXd::Zero(3, 8);
  yaw_coefficients(0, 7) = 0.4;
  geometry_utils::Trajectory yaw({1.0}, {yaw_coefficients});
  yaw.start_WT = 42.0;

  navigation_planning_backend::CmdTraj command;
  ASSERT_TRUE(command.setEmergencyBackup(position, yaw));
  EXPECT_FALSE(command.empty());
  EXPECT_TRUE(command.backupTrajAvilibale());
  EXPECT_DOUBLE_EQ(command.getBackupTrajStartTT(), 0.0);
  EXPECT_TRUE(command.isTTOnBackupTraj(0.01));
  EXPECT_NEAR(command.getPos(0.5).x(), 0.5, 1.0e-12);
  EXPECT_NEAR(command.getYaw(0.5).x(), 0.4, 1.0e-12);
}

TEST(PlannerTrajectory, FinalCommandYawRateCertificateCoversComposedBundle) {
  Eigen::MatrixXd slow_yaw_coefficients = Eigen::MatrixXd::Zero(3, 8);
  slow_yaw_coefficients(0, 6) = 1.0;
  geometry_utils::Trajectory slow_yaw({1.0}, {slow_yaw_coefficients});

  navigation_planning_backend::CandidateCommandBundle slow_candidate;
  slow_candidate.yaw = slow_yaw;
  EXPECT_TRUE(navigation_planning_backend::candidateYawRateWithinLimit(
      slow_candidate, 2.0));

  Eigen::MatrixXd fast_yaw_coefficients = Eigen::MatrixXd::Zero(3, 8);
  fast_yaw_coefficients(0, 6) = 4.0;
  geometry_utils::Trajectory fast_yaw({1.0}, {fast_yaw_coefficients});
  navigation_planning_backend::CandidateCommandBundle fast_candidate;
  fast_candidate.yaw = fast_yaw;
  EXPECT_FALSE(navigation_planning_backend::candidateYawRateWithinLimit(
      fast_candidate, 3.0));
  EXPECT_FALSE(navigation_planning_backend::candidateYawRateWithinLimit(
      fast_candidate, std::numeric_limits<double>::quiet_NaN()));
}

TEST(PlannerTrajectory, RobotStateHasDeterministicFiniteDefaults) {
  const navigation_math::RobotState state;
  EXPECT_TRUE(state.p.isZero());
  EXPECT_TRUE(state.v.isZero());
  EXPECT_TRUE(state.a.isZero());
  EXPECT_TRUE(state.j.isZero());
  EXPECT_TRUE(state.q.coeffs().allFinite());
  EXPECT_DOUBLE_EQ(state.q.norm(), 1.0);
  EXPECT_FALSE(state.rcv);
}

TEST(PlannerTrajectory, FailedCandidateLeavesEmergencyBundleUnchanged) {
  Eigen::MatrixXd coefficients = Eigen::MatrixXd::Zero(3, 8);
  coefficients(2, 7) = 3.0;
  geometry_utils::Trajectory position({1.0}, {coefficients});
  geometry_utils::Trajectory yaw({1.0}, {Eigen::MatrixXd::Zero(3, 8)});
  position.start_WT = yaw.start_WT = 7.0;
  navigation_planning_backend::CmdTraj command;
  ASSERT_TRUE(command.setEmergencyBackup(position, yaw));
  const auto generation = command.generation();
  const auto diagnostics_before = command.snapshot().diagnostics;
  const auto before = command.getPos(0.5);
  geometry_utils::Trajectory invalid_yaw;
  EXPECT_FALSE(command.setEmergencyBackup(position, invalid_yaw));
  EXPECT_EQ(command.generation(), generation);
  EXPECT_TRUE(command.getPos(0.5).isApprox(before));
  const auto diagnostics_after = command.snapshot().diagnostics;
  EXPECT_EQ(diagnostics_after.generation, diagnostics_before.generation);
  EXPECT_TRUE(diagnostics_after.candidate_start_pvaj.isApprox(
      diagnostics_before.candidate_start_pvaj));
}

TEST(PlannerTrajectory, CommitDiagnosticsDescribeExactOldToNewSplice) {
  auto first_position = linearTrajectory(1.0, 10.0);
  auto first_yaw = linearTrajectory(1.0, 10.0);
  navigation_planning_backend::CmdTraj command;
  ASSERT_TRUE(command.setEmergencyBackup(first_position, first_yaw));
  auto first = command.snapshot();
  EXPECT_EQ(first.generation, 1U);
  EXPECT_EQ(first.diagnostics.generation, first.generation);
  EXPECT_EQ(first.diagnostics.previous_generation, 0U);
  EXPECT_FALSE(first.diagnostics.previous_valid);
  EXPECT_TRUE(first.diagnostics.candidate_start_pvaj.isApprox(
      first_position.getState(0.0)));

  auto second_position = linearTrajectory(1.0, 10.5);
  auto second_yaw = linearTrajectory(1.0, 10.5);
  ASSERT_TRUE(command.setEmergencyBackup(second_position, second_yaw));
  const auto second = command.snapshot();
  ASSERT_EQ(second.generation, 2U);
  EXPECT_EQ(second.diagnostics.generation, second.generation);
  EXPECT_EQ(second.diagnostics.previous_generation, first.generation);
  EXPECT_TRUE(second.diagnostics.previous_valid);
  EXPECT_DOUBLE_EQ(second.diagnostics.previous_sample_tt, 0.5);
  EXPECT_TRUE(second.diagnostics.previous_pvaj.isApprox(
      first_position.getState(0.5)));
  EXPECT_TRUE(second.diagnostics.position_residual.isApprox(
      second_position.getPos(0.0) - first_position.getPos(0.5)));
  EXPECT_TRUE(second.diagnostics.velocity_residual.isApprox(
      second_position.getVel(0.0) - first_position.getVel(0.5)));
  EXPECT_TRUE(second.diagnostics.acceleration_residual.isApprox(
      second_position.getAcc(0.0) - first_position.getAcc(0.5)));
  EXPECT_TRUE(second.diagnostics.jerk_residual.isApprox(
      second_position.getJer(0.0) - first_position.getJer(0.5)));
  EXPECT_NEAR(second.diagnostics.yaw_residual, -0.5, 1.0e-12);
}

TEST(PlannerTrajectory, CommitDiagnosticsClampPriorSampleAtFinishedEnd) {
  navigation_planning_backend::CmdTraj command;
  auto first_position = linearTrajectory(1.0, 10.0);
  auto first_yaw = linearTrajectory(1.0, 10.0);
  ASSERT_TRUE(command.setEmergencyBackup(first_position, first_yaw));

  auto after_position = linearTrajectory(1.0, 20.0);
  auto after_yaw = linearTrajectory(1.0, 20.0);
  ASSERT_TRUE(command.setEmergencyBackup(after_position, after_yaw));
  EXPECT_DOUBLE_EQ(command.snapshot().diagnostics.previous_sample_tt, 1.0);
}

TEST(PlannerTrajectory, RegressedCandidateStartTimeCannotReplaceCommittedBundle) {
  navigation_planning_backend::CmdTraj command;
  auto current_position = linearTrajectory(1.0, 10.0);
  auto current_yaw = linearTrajectory(1.0, 10.0);
  ASSERT_TRUE(command.setEmergencyBackup(current_position, current_yaw));
  const auto before = command.snapshot();
  const bool before_role_at_start = command.isTTOnBackupTraj(0.0);
  const bool before_role_at_end = command.isTTOnBackupTraj(1.0);

  auto historical_position = linearTrajectory(1.0, 9.5);
  auto historical_yaw = linearTrajectory(1.0, 9.5);
  auto historical = navigation_planning_backend::CmdTraj::buildEmergencyCandidate(
      historical_position, historical_yaw);
  ASSERT_TRUE(historical);
  EXPECT_FALSE(command.commitCandidate(std::move(*historical), {}));

  const auto after = command.snapshot();
  EXPECT_EQ(after.generation, before.generation);
  EXPECT_DOUBLE_EQ(after.position.start_WT, before.position.start_WT);
  EXPECT_DOUBLE_EQ(after.yaw.start_WT, before.yaw.start_WT);
  EXPECT_TRUE(after.position.getState(0.25).isApprox(
      before.position.getState(0.25)));
  EXPECT_TRUE(after.yaw.getState(0.25).isApprox(before.yaw.getState(0.25)));
  EXPECT_EQ(after.empty, before.empty);
  EXPECT_EQ(after.backup_available, before.backup_available);
  EXPECT_DOUBLE_EQ(after.backup_start_tt, before.backup_start_tt);
  EXPECT_EQ(command.isTTOnBackupTraj(0.0), before_role_at_start);
  EXPECT_EQ(command.isTTOnBackupTraj(1.0), before_role_at_end);
  EXPECT_EQ(after.certificate.pinned_world.generation,
            before.certificate.pinned_world.generation);
  EXPECT_EQ(after.certificate.pinned_world.revision,
            before.certificate.pinned_world.revision);
  EXPECT_EQ(after.certificate.pinned_world.observation_stamp_ns,
            before.certificate.pinned_world.observation_stamp_ns);
  EXPECT_EQ(after.certificate.validated_world.generation,
            before.certificate.validated_world.generation);
  EXPECT_EQ(after.certificate.validated_world.revision,
            before.certificate.validated_world.revision);
  EXPECT_EQ(after.certificate.validated_world.observation_stamp_ns,
            before.certificate.validated_world.observation_stamp_ns);
  EXPECT_DOUBLE_EQ(after.certificate.validation_begin_tt,
                   before.certificate.validation_begin_tt);
  EXPECT_EQ(after.diagnostics.generation, before.diagnostics.generation);
  EXPECT_EQ(after.diagnostics.previous_generation,
            before.diagnostics.previous_generation);
  EXPECT_DOUBLE_EQ(after.diagnostics.candidate_start_wall_time,
                   before.diagnostics.candidate_start_wall_time);
  EXPECT_TRUE(after.diagnostics.candidate_start_pvaj.isApprox(
      before.diagnostics.candidate_start_pvaj));
  EXPECT_EQ(after.diagnostics.previous_valid, before.diagnostics.previous_valid);
  EXPECT_DOUBLE_EQ(after.diagnostics.previous_sample_tt,
                   before.diagnostics.previous_sample_tt);
  EXPECT_TRUE(after.diagnostics.previous_pvaj.isApprox(
      before.diagnostics.previous_pvaj));
  EXPECT_TRUE(after.diagnostics.position_residual.isApprox(
      before.diagnostics.position_residual));
  EXPECT_TRUE(after.diagnostics.velocity_residual.isApprox(
      before.diagnostics.velocity_residual));
  EXPECT_TRUE(after.diagnostics.acceleration_residual.isApprox(
      before.diagnostics.acceleration_residual));
  EXPECT_TRUE(after.diagnostics.jerk_residual.isApprox(
      before.diagnostics.jerk_residual));
  EXPECT_DOUBLE_EQ(after.diagnostics.candidate_start_yaw,
                   before.diagnostics.candidate_start_yaw);
  EXPECT_DOUBLE_EQ(after.diagnostics.candidate_start_yaw_rate,
                   before.diagnostics.candidate_start_yaw_rate);
  EXPECT_DOUBLE_EQ(after.diagnostics.previous_yaw,
                   before.diagnostics.previous_yaw);
  EXPECT_DOUBLE_EQ(after.diagnostics.previous_yaw_rate,
                   before.diagnostics.previous_yaw_rate);
  EXPECT_DOUBLE_EQ(after.diagnostics.yaw_residual,
                   before.diagnostics.yaw_residual);
  EXPECT_DOUBLE_EQ(after.diagnostics.yaw_rate_residual,
                   before.diagnostics.yaw_rate_residual);
}

TEST(PlannerTrajectory, EqualOrNewerCandidateStartTimeCanCommit) {
  navigation_planning_backend::CmdTraj command;
  auto first_position = linearTrajectory(1.0, 10.0);
  auto first_yaw = linearTrajectory(1.0, 10.0);
  ASSERT_TRUE(command.setEmergencyBackup(first_position, first_yaw));

  auto equal_position = linearTrajectory(1.0, 10.0);
  auto equal_yaw = linearTrajectory(1.0, 10.0);
  ASSERT_TRUE(command.setEmergencyBackup(equal_position, equal_yaw));
  auto newer_position = linearTrajectory(1.0, 10.1);
  auto newer_yaw = linearTrajectory(1.0, 10.1);
  ASSERT_TRUE(command.setEmergencyBackup(newer_position, newer_yaw));
  EXPECT_EQ(command.snapshot().generation, 3U);
}

TEST(PlannerTrajectory, CommitDiagnosticsWrapYawResidualAcrossPiBoundary) {
  auto position = linearTrajectory(1.0, 10.0);
  Eigen::MatrixXd first_yaw_coefficients = Eigen::MatrixXd::Zero(3, 8);
  first_yaw_coefficients(0, 7) = std::acos(-1.0) - 0.1;
  geometry_utils::Trajectory first_yaw({1.0}, {first_yaw_coefficients});
  first_yaw.start_WT = 10.0;
  navigation_planning_backend::CmdTraj command;
  ASSERT_TRUE(command.setEmergencyBackup(position, first_yaw));

  Eigen::MatrixXd second_yaw_coefficients = Eigen::MatrixXd::Zero(3, 8);
  second_yaw_coefficients(0, 7) = -std::acos(-1.0) + 0.1;
  geometry_utils::Trajectory second_yaw({1.0}, {second_yaw_coefficients});
  second_yaw.start_WT = 10.0;
  ASSERT_TRUE(command.setEmergencyBackup(position, second_yaw));
  EXPECT_NEAR(command.snapshot().diagnostics.yaw_residual, 0.2, 1.0e-12);
}

TEST(PlannerTrajectory, InheritedBackupPrefixSurvivesMainOnlyCommit) {
  Eigen::MatrixXd coefficients = Eigen::MatrixXd::Zero(3, 8);
  coefficients(2, 7) = 3.0;
  geometry_utils::Trajectory position({1.0}, {coefficients});
  geometry_utils::Trajectory yaw({1.0}, {Eigen::MatrixXd::Zero(3, 8)});
  navigation_planning_backend::ExpTraj exp;
  exp.setTrajectory(10.0, position, yaw, 0.0, 0.4);
  navigation_planning_backend::CmdTraj command;
  auto candidate = navigation_planning_backend::CmdTraj::buildCandidate(
      exp, nullptr, navigation_planning_backend::BackupDisposition::NO_NEED);
  ASSERT_TRUE(candidate);
  ASSERT_TRUE(command.commitCandidate(std::move(*candidate), {}));
  EXPECT_FALSE(command.backupTrajAvilibale());
  EXPECT_TRUE(command.isTTOnBackupTraj(0.0));
  EXPECT_TRUE(command.isTTOnBackupTraj(0.4));
  EXPECT_FALSE(command.isTTOnBackupTraj(0.5));
}

TEST(PlannerTrajectory, CandidateBuilderPreservesInheritedAndNewBackupRoles) {
  auto position = linearTrajectory(1.0, 10.0);
  auto yaw = linearTrajectory(1.0, 10.0);
  navigation_planning_backend::ExpTraj exp;
  exp.setTrajectory(10.0, position, yaw, 0.1, 0.3);
  navigation_planning_backend::BackupTraj backup;
  auto backup_position = linearTrajectory(0.5, 10.6);
  auto backup_yaw = linearTrajectory(0.5, 10.6);
  backup.setTrajectory(10.6, 0.6, backup_position, backup_yaw);

  auto candidate = navigation_planning_backend::CmdTraj::buildCandidate(
      exp, &backup, navigation_planning_backend::BackupDisposition::SUCCESS);
  ASSERT_TRUE(candidate);
  EXPECT_NEAR(candidate->position.getTotalDuration(), 1.1, 1.0e-12);
  ASSERT_EQ(candidate->roles.size(), 4U);
  EXPECT_EQ(candidate->roles[0].role, navigation_planning_backend::CandidateTrajectoryRole::MAIN);
  EXPECT_DOUBLE_EQ(candidate->roles[1].begin_tt, 0.1);
  EXPECT_DOUBLE_EQ(candidate->roles[1].end_tt, 0.3);
  EXPECT_EQ(candidate->roles[1].role, navigation_planning_backend::CandidateTrajectoryRole::BACKUP);
  EXPECT_DOUBLE_EQ(candidate->roles[3].begin_tt, 0.6);
  EXPECT_DOUBLE_EQ(candidate->roles[3].end_tt, 1.1);
  EXPECT_EQ(candidate->roles[3].role, navigation_planning_backend::CandidateTrajectoryRole::BACKUP);
  EXPECT_TRUE(candidate->backup_suffix_available);
}

TEST(PlannerTrajectory, CandidateBuilderDoesNotCutRequiredMainPrefix) {
  auto position = linearTrajectory(1.0, 10.0);
  auto yaw = linearTrajectory(1.0, 10.0);
  navigation_planning_backend::ExpTraj exp;
  exp.setTrajectory(10.0, position, yaw);
  ASSERT_TRUE(exp.setRequiredMainPrefixDuration(0.4));

  navigation_planning_backend::BackupTraj early_backup;
  early_backup.setTrajectory(
      10.3, 0.3, linearTrajectory(0.5, 10.3), linearTrajectory(0.5, 10.3));
  EXPECT_FALSE(navigation_planning_backend::CmdTraj::buildCandidate(
      exp, &early_backup,
      navigation_planning_backend::BackupDisposition::SUCCESS));

  navigation_planning_backend::BackupTraj delayed_backup;
  delayed_backup.setTrajectory(
      10.4, 0.4, linearTrajectory(0.5, 10.4), linearTrajectory(0.5, 10.4));
  auto candidate = navigation_planning_backend::CmdTraj::buildCandidate(
      exp, &delayed_backup,
      navigation_planning_backend::BackupDisposition::SUCCESS);
  ASSERT_TRUE(candidate);
  EXPECT_DOUBLE_EQ(candidate->backup_start_tt, 0.4);
  EXPECT_DOUBLE_EQ(candidate->position.getTotalDuration(), 0.9);
}

TEST(PlannerTrajectory, CandidateBuilderDoesNotAdvertiseZeroLengthBackupSuffix) {
  auto position = linearTrajectory(1.0, 10.0);
  auto yaw = linearTrajectory(1.0, 10.0);
  navigation_planning_backend::ExpTraj exp;
  exp.setTrajectory(10.0, position, yaw);

  navigation_planning_backend::BackupTraj backup;
  auto empty_position = linearTrajectory(0.0, 11.0);
  auto empty_yaw = linearTrajectory(0.0, 11.0);
  backup.setTrajectory(11.0, 1.0, empty_position, empty_yaw);

  // A non-null backup object is not sufficient evidence.  The executable
  // bundle must either contain a positive-duration final BACKUP interval or
  // reject construction before it can reach authorization.
  EXPECT_FALSE(navigation_planning_backend::CmdTraj::buildCandidate(
      exp, &backup, navigation_planning_backend::BackupDisposition::SUCCESS));
}

TEST(PlannerTrajectory, InheritedBackupIntersectionNeverCrossesNewSuffix) {
  auto position = linearTrajectory(1.0, 10.0);
  auto yaw = linearTrajectory(1.0, 10.0);
  auto backup_position = linearTrajectory(0.5, 10.6);
  auto backup_yaw = linearTrajectory(0.5, 10.6);
  navigation_planning_backend::BackupTraj backup;
  backup.setTrajectory(10.6, 0.6, backup_position, backup_yaw);

  for (const auto [begin, end] : std::vector<std::pair<double, double>>{
           {0.7, 0.9}, {0.5, 0.8}, {0.6, 0.6}, {0.2, 0.6}}) {
    navigation_planning_backend::ExpTraj exp;
    exp.setTrajectory(10.0, position, yaw, begin, end);
    auto candidate = navigation_planning_backend::CmdTraj::buildCandidate(
        exp, &backup, navigation_planning_backend::BackupDisposition::SUCCESS);
    ASSERT_TRUE(candidate);
    double previous_end = 0.0;
    for (const auto& interval : candidate->roles) {
      EXPECT_LE(interval.begin_tt, interval.end_tt);
      EXPECT_DOUBLE_EQ(interval.begin_tt, previous_end);
      previous_end = interval.end_tt;
    }
    EXPECT_DOUBLE_EQ(previous_end, candidate->position.getTotalDuration());
    EXPECT_TRUE(std::any_of(candidate->roles.begin(), candidate->roles.end(),
                            [](const auto& interval) {
                              return interval.role ==
                                         navigation_planning_backend::CandidateTrajectoryRole::BACKUP &&
                                     interval.begin_tt <= 0.6 && interval.end_tt >= 0.6;
                            }));
    if (begin > 0.6) {
      const auto first_backup = std::find_if(
          candidate->roles.begin(), candidate->roles.end(), [](const auto& interval) {
            return interval.role == navigation_planning_backend::CandidateTrajectoryRole::BACKUP;
          });
      ASSERT_NE(first_backup, candidate->roles.end());
      EXPECT_DOUBLE_EQ(first_backup->begin_tt, 0.6);
    }
  }
}

TEST(PlannerTrajectory, ExpOnlyDispositionsAndEmergencyPreserveProvenance) {
  auto position = stationaryTrajectory(1.0, 10.0);
  auto yaw = linearTrajectory(1.0, 10.0);
  navigation_planning_backend::ExpTraj exp;
  exp.setTrajectory(10.0, position, yaw, 0.0, 0.2);
  for (const auto disposition : {navigation_planning_backend::BackupDisposition::FINISH,
                                 navigation_planning_backend::BackupDisposition::NO_NEED}) {
    auto candidate = navigation_planning_backend::CmdTraj::buildCandidate(
        exp, nullptr, disposition);
    ASSERT_TRUE(candidate);
    EXPECT_EQ(candidate->backup_disposition, disposition);
    EXPECT_FALSE(candidate->backup_suffix_available);
    ASSERT_EQ(candidate->roles.size(), 2U);
    EXPECT_EQ(candidate->roles.front().role,
              navigation_planning_backend::CandidateTrajectoryRole::BACKUP);
    EXPECT_EQ(candidate->roles.back().role,
              navigation_planning_backend::CandidateTrajectoryRole::MAIN);
  }

  auto emergency = navigation_planning_backend::CmdTraj::buildEmergencyCandidate(position, yaw);
  ASSERT_TRUE(emergency);
  EXPECT_EQ(emergency->backup_disposition,
            navigation_planning_backend::BackupDisposition::EMERGENCY);
  ASSERT_EQ(emergency->roles.size(), 1U);
  EXPECT_EQ(emergency->roles.front().role,
            navigation_planning_backend::CandidateTrajectoryRole::BACKUP);
}

TEST(PlannerTrajectory, MainOnlyMovingTerminalRequiresBackupSuffix) {
  auto position = linearTrajectory(1.0, 10.0);
  auto yaw = stationaryTrajectory(1.0, 10.0);
  navigation_planning_backend::ExpTraj exp;
  exp.setTrajectory(10.0, position, yaw);

  for (const auto disposition : {navigation_planning_backend::BackupDisposition::FINISH,
                                 navigation_planning_backend::BackupDisposition::NO_NEED}) {
    EXPECT_FALSE(navigation_planning_backend::CmdTraj::buildCandidate(
        exp, nullptr, disposition));
  }
}

TEST(PlannerTrajectory, MainOnlyRestHasNoFlightTunedTerminalEpsilon) {
  auto position = stationaryTrajectory(1.0, 10.0);
  auto yaw = stationaryTrajectory(1.0, 10.0);
  navigation_planning_backend::ExpTraj exp;
  exp.setTrajectory(10.0, position, yaw);
  EXPECT_TRUE(navigation_planning_backend::CmdTraj::buildCandidate(
      exp, nullptr, navigation_planning_backend::BackupDisposition::FINISH));

  auto moving = position;
  auto coefficients = moving[0].getCoeffMat();
  coefficients(0, 6) = std::numeric_limits<double>::epsilon();
  geometry_utils::Trajectory nonzero_terminal({1.0}, {coefficients});
  nonzero_terminal.start_WT = 10.0;
  exp.setTrajectory(10.0, nonzero_terminal, yaw);
  EXPECT_FALSE(navigation_planning_backend::CmdTraj::buildCandidate(
      exp, nullptr, navigation_planning_backend::BackupDisposition::FINISH));
}

TEST(PlannerTrajectory, CandidateRejectsZeroDurationPiece) {
  auto position = linearTrajectory(1.0, 10.0);
  auto yaw = linearTrajectory(1.0, 10.0);
  geometry_utils::Trajectory malformed_position(
      {0.0, 1.0}, {position[0].getCoeffMat(), position[0].getCoeffMat()});
  geometry_utils::Trajectory malformed_yaw(
      {0.0, 1.0}, {yaw[0].getCoeffMat(), yaw[0].getCoeffMat()});

  EXPECT_FALSE(navigation_planning_backend::CmdTraj::buildEmergencyCandidate(
      malformed_position, malformed_yaw));
}

TEST(PlannerTrajectory, LatestWorldSweepAllowsUnknownAndRejectsFutureObstacle) {
  navigation_planning_backend::CandidateCommandBundle candidate;
  candidate.position = linearTrajectory(1.0, 10.0);
  candidate.yaw = linearTrajectory(1.0, 10.0);
  candidate.start_wall_time = 10.0;
  candidate.roles = {
      {0.0, 1.0, navigation_planning_backend::CandidateTrajectoryRole::MAIN},
  };
  SweepWorld world;
  EXPECT_TRUE(navigation_planning_backend::validateExecutableCandidate(
      world, candidate, 10.0, navigation_world_model::UnknownPolicy::kAllowUnknown).valid);
  EXPECT_FALSE(navigation_planning_backend::validateExecutableCandidate(
      world, candidate, 10.0,
      navigation_world_model::UnknownPolicy::kRequireKnownFree).valid);
  world.blocked_from_x = 0.7;
  const auto blocked = navigation_planning_backend::validateExecutableCandidate(
      world, candidate, 10.0, navigation_world_model::UnknownPolicy::kAllowUnknown);
  EXPECT_FALSE(blocked.valid);
  // The continuous tube certificate may reject at the first segment whose
  // conservative voxel tube reaches the obstacle boundary.
  EXPECT_GT(blocked.first_blocked_tt, 0.0);
}

TEST(PlannerTrajectory, ExpiredCandidateCannotBeValidatedAtItsTerminalPoint) {
  navigation_planning_backend::CandidateCommandBundle candidate;
  candidate.position = linearTrajectory(1.0, 10.0);
  candidate.yaw = linearTrajectory(1.0, 10.0);
  candidate.start_wall_time = 10.0;
  candidate.roles = {
      {0.0, 1.0, navigation_planning_backend::CandidateTrajectoryRole::MAIN},
  };

  SweepWorld world;
  EXPECT_TRUE(navigation_planning_backend::validateExecutableCandidate(
      world, candidate, 10.5,
      navigation_world_model::UnknownPolicy::kAllowUnknown).valid);
  // Validation must reject a candidate whose complete executable interval is
  // already in the past; clamping to t=duration must not turn it into a
  // terminal-point certificate.
  EXPECT_FALSE(navigation_planning_backend::validateExecutableCandidate(
      world, candidate, 11.01,
      navigation_world_model::UnknownPolicy::kAllowUnknown).valid);
}

TEST(PlannerTrajectory, ContinuousTubeRejectsCurvePassingThroughOccupiedCell) {
  constexpr double vertex_time = 0.5 / 0.98;
  Eigen::MatrixXd coefficients = Eigen::MatrixXd::Zero(3, 8);
  coefficients(0, 6) = 0.98;
  coefficients(1, 5) = -0.3 / (vertex_time * vertex_time);
  coefficients(1, 6) = 0.6 / vertex_time;
  coefficients(2, 7) = 3.0;

  navigation_planning_backend::CandidateCommandBundle candidate;
  candidate.position = geometry_utils::Trajectory({1.0}, {coefficients});
  candidate.yaw = linearTrajectory(1.0, 10.0);
  candidate.start_wall_time = 10.0;
  candidate.roles = {
      {0.0, 1.0, navigation_planning_backend::CandidateTrajectoryRole::MAIN},
  };

  CurvedCellWorld world;
  // The curve crosses the occupied voxel around t=0.5102 while its sampled
  // points remain 0.1 m below the voxel center. The tube certificate must
  // inspect the cell covered by the bounded curve deviation.
  EXPECT_FALSE(navigation_planning_backend::validateExecutableCandidate(
      world, candidate, 10.0,
      navigation_world_model::UnknownPolicy::kAllowUnknown).valid);
}

TEST(PlannerTrajectory, ContinuousTubeIgnoresOccupiedCellOutsideCurveTube) {
  navigation_planning_backend::CandidateCommandBundle candidate;
  candidate.position = linearTrajectory(1.0, 10.0);
  candidate.yaw = linearTrajectory(1.0, 10.0);
  candidate.start_wall_time = 10.0;
  candidate.roles = {
      {0.0, 1.0, navigation_planning_backend::CandidateTrajectoryRole::MAIN},
  };

  // This occupied cell is inside the segment's axis-aligned bounding box but
  // outside the actual swept tube. A box-only certificate would falsely
  // reject the candidate.
  CurvedCellWorld world;
  EXPECT_TRUE(navigation_planning_backend::validateExecutableCandidate(
      world, candidate, 10.0,
      navigation_world_model::UnknownPolicy::kAllowUnknown).valid);
}

TEST(PlannerTrajectory, ContinuousTubeDoesNotClassifyDiagonalCellOutsideTube) {
  const navigation_world_model::Point3 start(0.1, 0.21, 0.21);
  const navigation_world_model::Point3 end(1.1, 0.21, 0.21);
  DiagonalNeighborWorld world({0.1, 0.1, 0.1});

  // The old center-distance-plus-half-diagonal approximation included this
  // diagonal neighbour even though its voxel box is separated from the
  // zero-radius centerline by sqrt(0.01^2 + 0.01^2).
  EXPECT_TRUE(navigation_planning_backend::certificateTubeIsSafe(
      world, start, end, 0.0,
      navigation_world_model::UnknownPolicy::kRequireKnownFree, 0.2));
}

TEST(PlannerTrajectory, ContinuousTubeClassifiesCellTouchedByCenterline) {
  const navigation_world_model::Point3 start(0.1, 0.2, 0.2);
  const navigation_world_model::Point3 end(1.1, 0.2, 0.2);
  DiagonalNeighborWorld world({0.1, 0.1, 0.1});

  // The same neighbour is now touched at the voxel corner and must remain a
  // fail-closed UNKNOWN for a BACKUP certificate.
  EXPECT_FALSE(navigation_planning_backend::certificateTubeIsSafe(
      world, start, end, 0.0,
      navigation_world_model::UnknownPolicy::kRequireKnownFree, 0.2));
}

TEST(PlannerTrajectory, ContinuousTubeReportsActualBlockingCell) {
  const navigation_world_model::Point3 start(0.1, 0.2, 0.2);
  const navigation_world_model::Point3 end(1.1, 0.2, 0.2);
  const navigation_world_model::Point3 expected_blocker(0.1, 0.1, 0.1);
  DiagonalNeighborWorld world(expected_blocker);
  navigation_world_model::CellState blocked_state{
      navigation_world_model::CellState::kUndefined};
  navigation_world_model::Point3 blocked_position =
      navigation_world_model::Point3::Constant(
          std::numeric_limits<double>::quiet_NaN());

  EXPECT_FALSE(navigation_planning_backend::certificateTubeIsSafe(
      world, start, end, 0.0,
      navigation_world_model::UnknownPolicy::kRequireKnownFree, 0.2,
      &blocked_state, &blocked_position));
  EXPECT_EQ(blocked_state, navigation_world_model::CellState::kUnknown);
  EXPECT_TRUE(blocked_position.isApprox(expected_blocker, 1.0e-12));
}

TEST(PlannerTrajectory, CandidateValidationReportsActualTubeBlocker) {
  navigation_planning_backend::CandidateCommandBundle candidate;
  candidate.position = linearTrajectory(1.0, 10.0);
  candidate.yaw = linearTrajectory(1.0, 10.0);
  candidate.start_wall_time = 10.0;
  candidate.roles = {
      {0.0, 0.5, navigation_planning_backend::CandidateTrajectoryRole::MAIN},
      {0.5, 1.0, navigation_planning_backend::CandidateTrajectoryRole::BACKUP},
  };
  // The main prefix is allowed to cross UNKNOWN, while the backup interval
  // must report the first actual UNKNOWN voxel in its swept tube.
  SweepWorld world;

  const auto result = navigation_planning_backend::validateExecutableCandidate(
      world, candidate, 10.0,
      navigation_world_model::UnknownPolicy::kAllowUnknown);
  EXPECT_FALSE(result.valid);
  EXPECT_EQ(result.failure,
            navigation_planning_backend::SweptValidationResult::Failure::
                kCertificateTubeBlocked);
  EXPECT_EQ(result.blocked_cell_state,
            navigation_world_model::CellState::kUnknown);
  EXPECT_TRUE(result.blocked_position.allFinite());
  EXPECT_NEAR(result.blocked_position.x(), 0.5, 1.0e-12);
}

TEST(PlannerTrajectory, BackupRoleRequiresKnownFreeEvidence) {
  navigation_planning_backend::CandidateCommandBundle candidate;
  candidate.position = linearTrajectory(1.0, 10.0);
  candidate.yaw = linearTrajectory(1.0, 10.0);
  candidate.start_wall_time = 10.0;
  candidate.roles = {
      {0.0, 0.5, navigation_planning_backend::CandidateTrajectoryRole::MAIN},
      {0.5, 1.0, navigation_planning_backend::CandidateTrajectoryRole::BACKUP},
  };

  SweepWorld world;
  EXPECT_FALSE(navigation_planning_backend::validateExecutableCandidate(
      world, candidate, 10.0,
      navigation_world_model::UnknownPolicy::kAllowUnknown).valid);
  EXPECT_TRUE(navigation_planning_backend::candidateHasBackupSuffix(candidate));

  candidate.roles.pop_back();
  EXPECT_FALSE(navigation_planning_backend::candidateHasBackupSuffix(candidate));
}

TEST(PlannerTrajectory, MainOnlyAllowUnknownRequiresKnownFreeCertificate) {
  navigation_planning_backend::CandidateCommandBundle candidate;
  candidate.position = linearTrajectory(1.0, 10.0);
  candidate.yaw = linearTrajectory(1.0, 10.0);
  candidate.start_wall_time = 10.0;
  candidate.roles = {
      {0.0, 1.0, navigation_planning_backend::CandidateTrajectoryRole::MAIN},
  };

  EXPECT_EQ(
      navigation_planning_backend::candidateCertificatePolicy(
          candidate, navigation_world_model::UnknownPolicy::kAllowUnknown),
      navigation_world_model::UnknownPolicy::kRequireKnownFree);

  candidate.roles = {
      {0.0, 0.5, navigation_planning_backend::CandidateTrajectoryRole::MAIN},
      {0.5, 1.0, navigation_planning_backend::CandidateTrajectoryRole::BACKUP},
  };
  EXPECT_EQ(
      navigation_planning_backend::candidateCertificatePolicy(
          candidate, navigation_world_model::UnknownPolicy::kAllowUnknown),
      navigation_world_model::UnknownPolicy::kAllowUnknown);
}

TEST(PlannerTrajectory, MainOnlyRevalidationCannotReuseAllowUnknownPolicy) {
  navigation_planning_backend::CandidateCommandBundle candidate;
  candidate.position = linearTrajectory(1.0, 10.0);
  candidate.yaw = linearTrajectory(1.0, 10.0);
  candidate.start_wall_time = 10.0;
  candidate.roles = {
      {0.0, 1.0, navigation_planning_backend::CandidateTrajectoryRole::MAIN},
  };

  SweepWorld newer_world;
  const auto mission_policy = navigation_world_model::UnknownPolicy::kAllowUnknown;
  const auto certificate_policy =
      navigation_planning_backend::candidateCertificatePolicy(candidate, mission_policy);
  EXPECT_EQ(certificate_policy,
            navigation_world_model::UnknownPolicy::kRequireKnownFree);
  EXPECT_FALSE(navigation_planning_backend::validateExecutableCandidate(
                   newer_world, candidate, 10.0, certificate_policy)
                   .valid);
}

TEST(PlannerTrajectory, LatestWorldSweepIgnoresAlreadyExecutedBlockedPrefix) {
  navigation_planning_backend::CandidateCommandBundle candidate;
  candidate.position = linearTrajectory(1.0, 10.0);
  candidate.yaw = linearTrajectory(1.0, 10.0);
  candidate.start_wall_time = 10.0;
  class PrefixWorld final : public SweepWorld {
   public:
    navigation_world_model::CellState classify(
        const navigation_world_model::Point3& p,
        navigation_world_model::GridLayer) const noexcept override {
      return p.x() < 0.4 ? navigation_world_model::CellState::kOccupied
                         : navigation_world_model::CellState::kUnknown;
    }
    bool isSegmentTraversable(
        const navigation_world_model::Point3& a,
        const navigation_world_model::Point3&,
        navigation_world_model::GridLayer,
        navigation_world_model::UnknownPolicy) const noexcept override {
      return a.x() >= 0.4;
    }
  } prefix_world;
  candidate.roles = {
      {0.0, 1.0, navigation_planning_backend::CandidateTrajectoryRole::MAIN},
  };
  EXPECT_TRUE(navigation_planning_backend::validateExecutableCandidate(
      prefix_world, candidate, 10.5,
      navigation_world_model::UnknownPolicy::kAllowUnknown).valid);
}

TEST(PlannerTrajectory, NonFiniteYawCannotReplaceCommittedGeneration) {
  Eigen::MatrixXd position_coefficients = Eigen::MatrixXd::Zero(3, 8);
  position_coefficients(2, 7) = 3.0;
  geometry_utils::Trajectory position({1.0}, {position_coefficients});
  geometry_utils::Trajectory yaw({1.0}, {Eigen::MatrixXd::Zero(3, 8)});
  position.start_WT = yaw.start_WT = 5.0;
  navigation_planning_backend::CmdTraj command;
  ASSERT_TRUE(command.setEmergencyBackup(position, yaw));
  const auto generation = command.generation();

  Eigen::MatrixXd invalid_coefficients = Eigen::MatrixXd::Zero(3, 8);
  invalid_coefficients(0, 7) = std::numeric_limits<double>::quiet_NaN();
  geometry_utils::Trajectory invalid_yaw({1.0}, {invalid_coefficients});
  invalid_yaw.start_WT = 5.0;
  EXPECT_FALSE(command.setEmergencyBackup(position, invalid_yaw));
  EXPECT_EQ(command.generation(), generation);
}

TEST(PlannerTrajectory, ConcurrentCommitAndSnapshotNeverMixGenerations) {
  Eigen::MatrixXd position_coefficients = Eigen::MatrixXd::Zero(3, 8);
  position_coefficients(0, 6) = 1.0;
  position_coefficients(2, 7) = 3.0;
  geometry_utils::Trajectory position({1.0}, {position_coefficients});
  geometry_utils::Trajectory yaw({1.0}, {Eigen::MatrixXd::Zero(3, 8)});
  position.start_WT = yaw.start_WT = 1.0;
  navigation_planning_backend::CmdTraj command;
  ASSERT_TRUE(command.setEmergencyBackup(position, yaw));
  std::atomic_bool failed{false};
  std::thread writer([&] {
    for (int index = 0; index < 500; ++index) {
      if (!command.setEmergencyBackup(position, yaw)) failed.store(true);
    }
  });
  std::thread reader([&] {
    std::uint64_t previous_generation = 0;
    for (int index = 0; index < 500; ++index) {
      const auto snapshot = command.snapshot();
      const auto state = snapshot.position.getPos(0.5);
      if (snapshot.generation < previous_generation || !state.allFinite() ||
          snapshot.diagnostics.generation != snapshot.generation ||
          snapshot.diagnostics.previous_generation + 1U != snapshot.generation) {
        failed.store(true);
      }
      previous_generation = snapshot.generation;
    }
  });
  writer.join();
  reader.join();
  EXPECT_FALSE(failed.load());
}

TEST(PlannerTrajectory, SemanticYawTargetRespectsRateAndAccelerationEnvelope) {
  const std::vector<double> durations{5.0};
  Eigen::MatrixXd coefficients = Eigen::MatrixXd::Zero(3, 6);
  coefficients.col(4) = Eigen::Vector3d{1.0, 0.0, 0.0};
  geometry_utils::Trajectory position(durations, {coefficients});
  const navigation_math::Vec4f initial_yaw{0.0, 0.0, 0.1, 0.0};
  traj_opt::YawTrajOpt optimizer(1.0, 0.5);
  geometry_utils::Trajectory yaw;

  ASSERT_TRUE(optimizer.optimizeToTarget(initial_yaw, M_PI_2, position, yaw));
  EXPECT_NEAR(yaw.getTotalDuration(), position.getTotalDuration(), 1.0e-12);
  EXPECT_LE(yaw.getMaxVelRate(), 1.0 + 1.0e-6);
  EXPECT_LE(yaw.getMaxAccRate(), 0.5 + 1.0e-6);
  EXPECT_NEAR(yaw.getState(0.0)(0, 2), initial_yaw(2), 1.0e-9);
  EXPECT_NEAR(yaw.getState(yaw.getTotalDuration())(0, 2), 0.0, 1.0e-9);
  EXPECT_NEAR(yaw.getPos(yaw.getTotalDuration()).x(), M_PI_2, 1.0e-6);
}

TEST(PlannerTrajectory, SemanticYawExecutesCertifiedPartialTurnWhenTimeIsShort) {
  const std::vector<double> durations{0.5};
  Eigen::MatrixXd coefficients = Eigen::MatrixXd::Zero(3, 6);
  geometry_utils::Trajectory position(durations, {coefficients});
  const navigation_math::Vec4f initial_yaw{3.0, 0.0, 0.0, 0.0};
  traj_opt::YawTrajOpt optimizer(1.0, 0.5);
  geometry_utils::Trajectory yaw;

  ASSERT_TRUE(optimizer.optimizeToTarget(initial_yaw, -3.0, position, yaw));
  EXPECT_LE(yaw.getMaxVelRate(), 1.0 + 1.0e-6);
  EXPECT_LE(yaw.getMaxAccRate(), 0.5 + 1.0e-6);
  const double endpoint = yaw.getPos(yaw.getTotalDuration()).x();
  EXPECT_GT(endpoint, initial_yaw(0));
  EXPECT_LT(endpoint, initial_yaw(0) + std::remainder(-3.0 - 3.0, 2.0 * M_PI));
}

TEST(PlannerTrajectory, SemanticYawReportsInfeasibleShortSameHeadingStop) {
  const std::vector<double> durations{0.2};
  Eigen::MatrixXd coefficients = Eigen::MatrixXd::Zero(3, 6);
  geometry_utils::Trajectory position(durations, {coefficients});
  const navigation_math::Vec4f initial_yaw{0.0, 0.8, 0.0, 0.0};
  traj_opt::YawTrajOpt optimizer(1.0, 0.3);
  geometry_utils::Trajectory yaw;

  EXPECT_FALSE(optimizer.optimizeToTarget(initial_yaw, 0.0, position, yaw));
  const auto& diagnostics = optimizer.lastDiagnostics();
  EXPECT_EQ(diagnostics.failure,
            traj_opt::YawOptimizationFailure::kNoFeasibleHold);
  EXPECT_DOUBLE_EQ(diagnostics.duration_s, 0.2);
  EXPECT_DOUBLE_EQ(diagnostics.requested_delta_rad, 0.0);
  EXPECT_GT(diagnostics.hold_max_acceleration_rad_s2, 0.3);
  EXPECT_GT(diagnostics.stopping_max_acceleration_rad_s2, 0.3);
}

TEST(PlannerTrajectory, MinimumJerkForwardYawStopHasPhysicalAccelerationPeak) {
  const navigation_math::Vec3f initial{0.0, 0.8, 0.0};
  const navigation_math::Vec3f terminal{0.08, 0.0, 0.0};
  const navigation_math::VecDf times =
      navigation_math::VecDf::Constant(1, 0.2);
  const navigation_math::VecDf no_waypoints;
  const auto stopping = geometry_utils::poly_interpo::minimumJerkInterpolation<1>(
      initial, terminal, no_waypoints, times);

  ASSERT_FALSE(stopping.empty());
  SCOPED_TRACE(::testing::Message() << "coefficients:\n"
                                   << stopping[0].getCoeffMat());
  EXPECT_NEAR(stopping.getVel(0.0).x(), 0.8, 1.0e-12);
  EXPECT_NEAR(stopping.getPos(0.2).x(), 0.08, 1.0e-12);
  EXPECT_NEAR(stopping.getVel(0.2).x(), 0.0, 1.0e-12);
  EXPECT_NEAR(stopping.getAcc(0.2).x(), 0.0, 1.0e-12);
  EXPECT_NEAR(stopping.getMaxAccRate(), 6.0, 1.0e-8);
}

navigation_mission::ImmutableRouteSnapshot makeStraightActiveRouteSnapshot(
    const Eigen::Vector3d& start_position = Eigen::Vector3d{0.0, 0.0, 3.0},
    const Eigen::Vector3d& active_position = Eigen::Vector3d{20.0, 0.0, 3.0},
    const Eigen::Vector3d& measured_position = Eigen::Vector3d{5.0, 0.0, 3.0}) {
  navigation_mission::Mission mission;
  mission.id = "route-regression-test";
  mission.frame = "lio_odom";
  navigation_mission::MissionWaypoint start;
  start.id = "start";
  start.position_enu = start_position;
  navigation_mission::MissionWaypoint active;
  active.id = "active";
  active.position_enu = active_position;
  mission.waypoints = {start, active};
  navigation_mission::RouteProgress progress(mission);
  const auto measured = progress.update(measured_position);
  EXPECT_TRUE(measured.valid);
  return progress.snapshot(mission.id, mission.frame, 1U, 1U, 1U);
}

navigation_mission::ImmutableRouteSnapshot makeCornerActiveRouteSnapshot() {
  navigation_mission::Mission mission;
  mission.id = "route-regression-corner-test";
  mission.frame = "lio_odom";
  navigation_mission::MissionWaypoint start;
  start.id = "start";
  start.position_enu = Eigen::Vector3d{0.0, 0.0, 3.0};
  navigation_mission::MissionWaypoint corner;
  corner.id = "corner";
  corner.position_enu = Eigen::Vector3d{20.0, 0.0, 3.0};
  corner.acceptance_radius_m = 0.9;
  navigation_mission::MissionWaypoint outgoing;
  outgoing.id = "outgoing";
  outgoing.position_enu = Eigen::Vector3d{20.0, 10.0, 3.0};
  mission.waypoints = {start, corner, outgoing};
  navigation_mission::RouteProgress progress(mission);
  const auto measured = progress.update(Eigen::Vector3d{5.0, 0.0, 3.0});
  EXPECT_TRUE(measured.valid);
  return progress.snapshot(mission.id, mission.frame, 1U, 1U, 1U);
}

TEST(PlannerTrajectory, MainRouteRegressionCertificateRejectsFoldedNominal) {
  Eigen::MatrixXd coefficients = Eigen::MatrixXd::Zero(3, 3);
  // x(t)=5+8t-4t^2 advances to x=9 and folds back to x=5.
  coefficients.row(0) << -4.0, 8.0, 5.0;
  coefficients(2, 2) = 3.0;
  navigation_planning_backend::CandidateCommandBundle candidate;
  candidate.position = geometry_utils::Trajectory({2.0}, {coefficients});
  candidate.position.start_WT = 10.0;
  candidate.roles = {{0.0, 2.0,
                      navigation_planning_backend::CandidateTrajectoryRole::MAIN}};

  const auto certificate = navigation_planning_backend::certifyMainRouteRegression(
      candidate, makeStraightActiveRouteSnapshot(), 0.0, 0.5);
  EXPECT_TRUE(certificate.applicable);
  EXPECT_FALSE(certificate.valid);
  EXPECT_NEAR(certificate.maximum_regression_m, 4.0, 1.0e-9);
  EXPECT_NEAR(certificate.first_violation_time_s, 2.0, 1.0e-9);
}

TEST(PlannerTrajectory, MainRouteRegressionCertificateAcceptsForwardDetour) {
  Eigen::MatrixXd coefficients = Eigen::MatrixXd::Zero(3, 2);
  coefficients.col(0) << 4.0, 2.0, 0.0;
  coefficients.col(1) << 5.0, 0.0, 3.0;
  navigation_planning_backend::CandidateCommandBundle candidate;
  candidate.position = geometry_utils::Trajectory({2.0}, {coefficients});
  candidate.position.start_WT = 10.0;
  candidate.roles = {{0.0, 2.0,
                      navigation_planning_backend::CandidateTrajectoryRole::MAIN}};

  const auto certificate = navigation_planning_backend::certifyMainRouteRegression(
      candidate, makeStraightActiveRouteSnapshot(), 0.0, 0.5);
  EXPECT_TRUE(certificate.applicable);
  EXPECT_TRUE(certificate.valid);
  EXPECT_NEAR(certificate.maximum_regression_m, 0.0, 1.0e-12);
}

TEST(PlannerTrajectory, MainRouteRegressionCertificateDoesNotGateBackupBrake) {
  Eigen::MatrixXd coefficients = Eigen::MatrixXd::Zero(3, 2);
  coefficients.col(0) << -1.0, 0.0, 0.0;
  coefficients.col(1) << 5.0, 0.0, 3.0;
  navigation_planning_backend::CandidateCommandBundle candidate;
  candidate.position = geometry_utils::Trajectory({1.0}, {coefficients});
  candidate.position.start_WT = 10.0;
  candidate.roles = {{0.0, 1.0,
                      navigation_planning_backend::CandidateTrajectoryRole::BACKUP}};

  const auto certificate = navigation_planning_backend::certifyMainRouteRegression(
      candidate, makeStraightActiveRouteSnapshot(), 0.0, 0.5);
  EXPECT_FALSE(certificate.applicable);
}

TEST(PlannerTrajectory, MainRouteRegressionCertificateAllowsNegativeEnuDirection) {
  Eigen::MatrixXd coefficients = Eigen::MatrixXd::Zero(3, 2);
  coefficients.col(0) << -4.0, 0.0, 0.0;
  coefficients.col(1) << 5.0, 0.0, 3.0;
  navigation_planning_backend::CandidateCommandBundle candidate;
  candidate.position = geometry_utils::Trajectory({1.0}, {coefficients});
  candidate.position.start_WT = 10.0;
  candidate.roles = {{0.0, 1.0,
                      navigation_planning_backend::CandidateTrajectoryRole::MAIN}};
  const auto reverse_enu_route = makeStraightActiveRouteSnapshot(
      Eigen::Vector3d{10.0, 0.0, 3.0}, Eigen::Vector3d{0.0, 0.0, 3.0},
      Eigen::Vector3d{5.0, 0.0, 3.0});

  const auto certificate = navigation_planning_backend::certifyMainRouteRegression(
      candidate, reverse_enu_route, 0.0, 0.5);
  EXPECT_TRUE(certificate.applicable);
  EXPECT_TRUE(certificate.valid);
  EXPECT_NEAR(certificate.maximum_regression_m, 0.0, 1.0e-12);
}

TEST(PlannerTrajectory, MainRouteRegressionCertificateChangesTangentInsideAcceptanceBall) {
  Eigen::MatrixXd incoming = Eigen::MatrixXd::Zero(3, 2);
  incoming.row(0) << 14.4, 5.0;
  incoming(2, 1) = 3.0;
  Eigen::MatrixXd outgoing = Eigen::MatrixXd::Zero(3, 3);
  // The outgoing piece bows in X while progressing monotonically in Y. A
  // single incoming-tangent certificate sees a 1 m X regression at its tail;
  // route-arc progress correctly remains monotonic after the pinned junction.
  outgoing.row(0) << -1.0, 1.0, 19.4;
  outgoing.row(1) << 0.0, 7.0, 0.0;
  outgoing(2, 2) = 3.0;
  navigation_planning_backend::CandidateCommandBundle candidate;
  candidate.position = geometry_utils::Trajectory(
      {1.0, 1.0}, {incoming, outgoing});
  candidate.roles = {{0.0, 2.0,
                      navigation_planning_backend::CandidateTrajectoryRole::MAIN}};

  const auto certificate = navigation_planning_backend::certifyMainRouteRegression(
      candidate, makeCornerActiveRouteSnapshot(), 0.0, 0.5);
  EXPECT_TRUE(certificate.applicable);
  EXPECT_TRUE(certificate.valid);
  EXPECT_NEAR(certificate.maximum_regression_m, 0.0, 1.0e-12);
}

TEST(PlannerTrajectory, MainRouteRegressionCertificateRejectsFoldAfterCorner) {
  Eigen::MatrixXd incoming = Eigen::MatrixXd::Zero(3, 2);
  incoming.row(0) << 14.4, 5.0;
  incoming(2, 1) = 3.0;
  Eigen::MatrixXd outgoing = Eigen::MatrixXd::Zero(3, 3);
  outgoing(0, 2) = 19.4;
  // y(t)=-4t^2+8t advances 4 m, then returns to the corner at t=2.
  outgoing.row(1) << -4.0, 8.0, 0.0;
  outgoing(2, 2) = 3.0;
  navigation_planning_backend::CandidateCommandBundle candidate;
  candidate.position = geometry_utils::Trajectory(
      {1.0, 2.0}, {incoming, outgoing});
  candidate.roles = {{0.0, 3.0,
                      navigation_planning_backend::CandidateTrajectoryRole::MAIN}};

  const auto certificate = navigation_planning_backend::certifyMainRouteRegression(
      candidate, makeCornerActiveRouteSnapshot(), 0.0, 0.5);
  EXPECT_TRUE(certificate.applicable);
  EXPECT_FALSE(certificate.valid);
  EXPECT_NEAR(certificate.maximum_regression_m, 4.0, 1.0e-9);
}

TEST(PlannerTrajectory, SemanticYawUsesForwardStoppingDisplacementForBackupHold) {
  const std::vector<double> durations{3.0};
  Eigen::MatrixXd coefficients = Eigen::MatrixXd::Zero(3, 6);
  geometry_utils::Trajectory position(durations, {coefficients});
  const navigation_math::Vec4f initial_yaw{
      0.516842123820765, 0.24656354241796274,
      -0.06843350325595021, 0.0};
  traj_opt::YawTrajOpt optimizer(1.0, 0.3);
  geometry_utils::Trajectory yaw;

  ASSERT_TRUE(optimizer.optimizeToTarget(
      initial_yaw, initial_yaw(0), position, yaw));
  const auto& diagnostics = optimizer.lastDiagnostics();
  EXPECT_TRUE(diagnostics.used_stopping_displacement);
  const double expected_displacement =
      0.5 * initial_yaw(1) * 3.0 + initial_yaw(2) * 9.0 / 12.0;
  EXPECT_NEAR(diagnostics.stopping_displacement_rad,
              expected_displacement, 1.0e-12);
  EXPECT_NEAR(yaw.getPos(3.0).x(),
              initial_yaw(0) + expected_displacement, 1.0e-9);
  EXPECT_NEAR(yaw.getVel(3.0).x(), 0.0, 1.0e-9);
  EXPECT_NEAR(yaw.getAcc(3.0).x(), 0.0, 1.0e-9);
  EXPECT_LE(yaw.getMaxVelRate(), 1.0 + 1.0e-6);
  EXPECT_LE(yaw.getMaxAccRate(), 0.3 + 1.0e-6);
}

TEST(PlannerTrajectory, YawNormalizationRecoversNonFiniteTarget) {
  double yaw = std::numeric_limits<double>::infinity();
  geometry_utils::normalizeNextYaw(1.2, yaw);
  EXPECT_DOUBLE_EQ(yaw, 1.2);

  yaw = -std::numeric_limits<double>::infinity();
  geometry_utils::normalizeNextYaw(std::numeric_limits<double>::quiet_NaN(), yaw);
  EXPECT_DOUBLE_EQ(yaw, 0.0);
}

TEST(PlannerTrajectory, SearchFallbacksShareOneAbsoluteDeadline) {
  const navigation_planning_backend::AbsoluteDeadline deadline(100.0, 0.04);
  EXPECT_NEAR(deadline.remaining(100.01), 0.03, 1.0e-12);
  EXPECT_NEAR(deadline.remaining(100.039), 0.001, 1.0e-12);
  EXPECT_DOUBLE_EQ(deadline.remaining(100.04), 0.0);
  EXPECT_TRUE(deadline.expired(100.05));
  EXPECT_DOUBLE_EQ(deadline.remaining(std::numeric_limits<double>::quiet_NaN()), 0.0);
  const auto now_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::steady_clock::now().time_since_epoch()).count();
  EXPECT_GT(deadline.steadyDeadlineNanoseconds(), now_ns);
  EXPECT_FALSE(deadline.steadyExpired());
  EXPECT_THROW((navigation_planning_backend::AbsoluteDeadline{0.0, 0.0}), std::invalid_argument);
  EXPECT_THROW((navigation_planning_backend::AbsoluteDeadline{0.0, -1.0}), std::invalid_argument);
  EXPECT_THROW((navigation_planning_backend::AbsoluteDeadline{0.0, std::numeric_limits<double>::quiet_NaN()}),
               std::invalid_argument);
  EXPECT_THROW((navigation_planning_backend::AbsoluteDeadline{0.0, std::numeric_limits<double>::infinity()}),
               std::invalid_argument);
  EXPECT_THROW((navigation_planning_backend::AbsoluteDeadline{0.0, std::numeric_limits<double>::max()}),
               std::invalid_argument);

  // Simulation time may remain frozen while the optimizer is still executing.
  // The monotonic budget must expire independently of the ROS-time contract.
  const navigation_planning_backend::AbsoluteDeadline frozen_sim_time(42.0, 0.001);
  std::this_thread::sleep_for(std::chrono::milliseconds(3));
  EXPECT_TRUE(frozen_sim_time.steadyExpired());
  EXPECT_NEAR(frozen_sim_time.remaining(42.0), 0.001, 1.0e-12);
}

TEST(PlannerTrajectory, ConnectedGoalIsResolvedBeforeCorridorConstruction) {
  const navigation_math::Vec3f guide_endpoint(1.0, 2.0, 3.0);
  const navigation_math::Vec3f goal(1.1, 2.0, 3.0);
  const auto result = navigation_planning_backend::resolveGuideEndpoint(
      guide_endpoint, goal, 0.4);
  EXPECT_TRUE(result.goal_connected);
  EXPECT_TRUE(result.position.isApprox(goal));
}

TEST(PlannerTrajectory, SimplifySfcPreservesRouteBoundaryGate) {
  using geometry_utils::MatD4f;
  using geometry_utils::Polytope;
  using geometry_utils::PolytopeVec;
  using navigation_math::Vec3f;

  const auto make_box = [](double min_x, double max_x,
                           double min_y, double max_y,
                           double min_z, double max_z) {
    MatD4f planes(6, 4);
    planes <<
        1.0, 0.0, 0.0, -max_x,
       -1.0, 0.0, 0.0, min_x,
        0.0, 1.0, 0.0, -max_y,
        0.0,-1.0, 0.0, min_y,
        0.0, 0.0, 1.0, -max_z,
        0.0, 0.0,-1.0, min_z;
    return Polytope(planes);
  };

  Polytope gate = make_box(0.8, 1.2, -0.2, 0.2, 2.8, 3.2);
  gate.SetRouteBoundaryContract(Vec3f{1.0F, 0.0F, 3.0F}, 0.9);
  PolytopeVec sfcs{
      make_box(-1.0, 2.0, -1.0, 1.0, 2.0, 4.0),
      gate,
      make_box(0.0, 3.0, -1.0, 1.0, 2.0, 4.0)};

  ASSERT_TRUE(geometry_utils::SimplifySFC(
      Vec3f(0.0, 0.0, 3.0), Vec3f(2.5, 0.0, 3.0), sfcs));
  ASSERT_EQ(sfcs.size(), 3U);
  EXPECT_TRUE(sfcs[1].IsRouteBoundaryGate());
  EXPECT_TRUE(sfcs[1].GetRouteBoundaryPoint().isApprox(
      Vec3f{1.0F, 0.0F, 3.0F}));
  EXPECT_DOUBLE_EQ(sfcs[1].GetRouteBoundaryRadius(), 0.9);
}

TEST(PlannerTrajectory, GoalPoliciesRemainNamedAndShareProvisionalValue) {
  EXPECT_DOUBLE_EQ(navigation_world_model::kGoalConnectionToleranceM,
                   navigation_world_model::kGoalCompletionToleranceM);
  EXPECT_DOUBLE_EQ(navigation_world_model::kNearGoalShortcutToleranceM,
                   navigation_world_model::kGoalCompletionToleranceM);
}

TEST(PlannerTrajectory, NearGoalSegmentRejectsOccupiedAndOutOfMapEndpoints) {
  SweepWorld world;
  const navigation_world_model::Point3 start(0.0, 0.0, 0.0);
  const navigation_world_model::Point3 goal(1.0, 0.0, 0.0);
  EXPECT_TRUE(navigation_world_model::isGoalSegmentTraversable(world, start, goal));

  world.blocked_from_x = 0.5;
  EXPECT_FALSE(navigation_world_model::isGoalSegmentTraversable(world, start, goal));

  world.blocked_from_x = std::numeric_limits<double>::infinity();
  world.endpoints_in_bounds = false;
  EXPECT_FALSE(navigation_world_model::isGoalSegmentTraversable(world, start, goal));
}

TEST(PlannerTrajectory, TimeAllocatorHandlesSwitchingDistanceRoundoff) {
  double elapsed = std::numeric_limits<double>::quiet_NaN();
  double velocity = std::numeric_limits<double>::quiet_NaN();
  constexpr double distance = 5.608921464952064;
  geometry_utils::simplePMTimeAllocator(
      3.0, 2.0, 0.0, distance, distance, elapsed, velocity);
  EXPECT_TRUE(std::isfinite(elapsed));
  EXPECT_GE(elapsed, 0.0);
  EXPECT_TRUE(std::isfinite(velocity));
}

TEST(PlannerTrajectory, GuideTimeAllocationUsesPointAlignedTravelledDistance) {
  const navigation_math::Vec3f start(0.0, 0.0, 0.0);
  const navigation_math::vec_E<navigation_math::Vec3f> nonuniform_path{
      start,
      navigation_math::Vec3f(0.3, 0.0, 0.0),
      navigation_math::Vec3f(4.7, 0.0, 0.0),
      navigation_math::Vec3f(30.0, 0.0, 0.0),
  };

  double previous_total_time = std::numeric_limits<double>::infinity();
  for (const double maximum_velocity : {2.0, 5.0, 6.0, 10.0}) {
    geometry_utils::GuideTimeAllocation allocation;
    ASSERT_TRUE(geometry_utils::allocateGuideElapsedTimes(
        5.0, maximum_velocity, 0.0, start, nonuniform_path, allocation));
    ASSERT_EQ(allocation.points.size(), 3U);
    ASSERT_EQ(allocation.elapsed_s.size(), allocation.points.size());
    EXPECT_TRUE(allocation.points.front().isApprox(nonuniform_path[1]));
    EXPECT_TRUE(allocation.points.back().isApprox(nonuniform_path.back()));
    EXPECT_NEAR(allocation.path_length_m, 30.0, 1.0e-12);
    EXPECT_GT(allocation.elapsed_s.front(), 0.0);
    for (std::size_t index = 1; index < allocation.elapsed_s.size(); ++index) {
      EXPECT_GT(allocation.elapsed_s[index], allocation.elapsed_s[index - 1]);
    }

    double expected_total_time = std::numeric_limits<double>::quiet_NaN();
    double terminal_velocity = std::numeric_limits<double>::quiet_NaN();
    geometry_utils::simplePMTimeAllocator(
        5.0, maximum_velocity, 0.0, 30.0, 30.0,
        expected_total_time, terminal_velocity);
    EXPECT_NEAR(allocation.elapsed_s.back(), expected_total_time, 1.0e-12);
    EXPECT_LE(allocation.elapsed_s.back(), previous_total_time);
    previous_total_time = allocation.elapsed_s.back();
  }
}

TEST(PlannerTrajectory, ContinuationGuideDoesNotManufactureFrontierStop) {
  const navigation_math::Vec3f start(0.0, 0.0, 0.0);
  const navigation_math::vec_E<navigation_math::Vec3f> path{
      navigation_math::Vec3f(4.0, 0.0, 0.0),
      navigation_math::Vec3f(14.0, 0.0, 0.0),
  };
  geometry_utils::GuideTimeAllocation allocation;
  ASSERT_TRUE(geometry_utils::allocateGuideContinuationElapsedTimes(
      2.0, 4.0, 5.0, 0.0, start, path, allocation));
  ASSERT_EQ(allocation.elapsed_s.size(), 2U);
  EXPECT_GT(allocation.elapsed_s.front(), 0.0);
  EXPECT_GT(allocation.elapsed_s.back(), allocation.elapsed_s.front());
  EXPECT_NEAR(allocation.path_length_m, 14.0, 1.0e-12);
  EXPECT_NEAR(allocation.terminal_velocity_mps, 5.0, 1.0e-12);
  EXPECT_NEAR(allocation.elapsed_s.back(), 4.3, 1.0e-12);
}

TEST(PlannerTrajectory, ShortContinuationGuideEndsWithBoundedPositiveSpeed) {
  geometry_utils::JerkLimitedContinuationProfile profile;
  ASSERT_TRUE(geometry_utils::makeJerkLimitedContinuationProfile(
      0.5, 0.0, 5.0, 2.0, 4.0, profile));
  EXPECT_GT(profile.terminal_velocity_mps, 0.0);
  EXPECT_LT(profile.terminal_velocity_mps, 5.0);
  EXPECT_LE(profile.maximum_jerk_mps3, 4.0);
  EXPECT_LE(4.0 * profile.jerk_phase_s, 2.0 + 1.0e-12);
  EXPECT_NEAR(profile.distanceAtTime(profile.total_duration_s), 0.5, 1.0e-12);
}

TEST(PlannerTrajectory, ContinuationGuideRejectsOverspeedAndInvalidLimits) {
  geometry_utils::JerkLimitedContinuationProfile profile;
  EXPECT_FALSE(geometry_utils::makeJerkLimitedContinuationProfile(
      10.0, 5.1, 5.0, 2.0, 4.0, profile));
  EXPECT_FALSE(geometry_utils::makeJerkLimitedContinuationProfile(
      10.0, 0.0, 5.0, 2.0, 0.0, profile));
}

TEST(PlannerTrajectory, SubdividesEverySparseGuideEdgeForCorridorSeeds) {
  const navigation_math::vec_Vec3f path{
      navigation_math::Vec3f{44.0F, 5.0F, 3.0F},
      navigation_math::Vec3f{50.0F, 5.0F, 3.0F},
      navigation_math::Vec3f{50.0F, -0.7F, 3.0F}};
  navigation_math::vec_Vec3f subdivided;
  ASSERT_TRUE(geometry_utils::subdividePathByMaximumSegmentLength(
      path, 3.0, subdivided));
  ASSERT_EQ(subdivided.front(), path.front());
  ASSERT_EQ(subdivided.back(), path.back());
  ASSERT_EQ(subdivided.size(), 5U);
  for (std::size_t index = 1U; index < subdivided.size(); ++index) {
    EXPECT_LE((subdivided[index] - subdivided[index - 1U]).norm(), 3.0 + 1.0e-6);
  }
  EXPECT_TRUE(subdivided[2].isApprox(navigation_math::Vec3f{50.0F, 5.0F, 3.0F}));
}

TEST(PlannerTrajectory, TruncatesRemoteRouteToBoundedCertifiedPrefix) {
  const navigation_math::vec_Vec3f path{
      navigation_math::Vec3f{0.0F, 0.0F, 3.0F},
      navigation_math::Vec3f{4.0F, 0.0F, 3.0F},
      navigation_math::Vec3f{4.0F, 4.0F, 3.0F},
      navigation_math::Vec3f{12.0F, 4.0F, 3.0F}};
  navigation_math::vec_Vec3f prefix;
  bool truncated = false;
  ASSERT_TRUE(geometry_utils::truncatePathAtDistance(
      path, 7.0, prefix, truncated));
  ASSERT_TRUE(truncated);
  ASSERT_EQ(prefix.size(), 3U);
  EXPECT_TRUE(prefix.front().isApprox(path.front()));
  EXPECT_TRUE(prefix[1].isApprox(path[1]));
  EXPECT_NEAR((prefix.back() - path[1]).norm(), 3.0, 1.0e-6);
  EXPECT_NEAR(
      (prefix[1] - prefix[0]).norm() + (prefix[2] - prefix[1]).norm(),
      7.0, 1.0e-6);
}

TEST(PlannerTrajectory, BoundsRemoteAstarBeforeExpandingPastVisibility) {
  EXPECT_DOUBLE_EQ(geometry_utils::localRouteSearchHorizon(45.0, 14.0), 14.0);
  EXPECT_DOUBLE_EQ(geometry_utils::localRouteSearchHorizon(8.0, 14.0), 8.0);
  EXPECT_TRUE(std::isnan(
      geometry_utils::localRouteSearchHorizon(45.0, 0.0)));
  EXPECT_TRUE(std::isnan(
      geometry_utils::localRouteSearchHorizon(
          std::numeric_limits<double>::infinity(), 14.0)));
}

TEST(PlannerTrajectory, InflatedAstarDoesNotRepeatIdenticalEdgeCertificates) {
  auto world = std::make_shared<CountingAstarWorld>();
  path_search::PathSearchConfig config;
  config.allow_diag = true;
  config.heu_type = 2;
  config.heuristic_weight = 1.5;
  auto context =
      std::make_shared<navigation_planner_context::PlannerRuntimeContext>();
  path_search::Astar astar(config, context, world, 1.0);
  navigation_math::vec_Vec3f path;
  const int flags = path_search::ON_INF_MAP |
                    path_search::UNKNOWN_AS_FREE |
                    path_search::DONT_USE_INF_NEIGHBOR;

  const auto result = astar.pointToPointPathSearch(
      navigation_math::Vec3f{0.5F, 0.5F, 1.5F},
      navigation_math::Vec3f{5.5F, 0.5F, 1.5F},
      flags, 20.0, path, 1.0, true);

  EXPECT_TRUE(result == navigation_math::REACH_GOAL ||
              result == navigation_math::REACH_HORIZON);
  EXPECT_GT(path.size(), 2U);
  EXPECT_EQ(world->consecutiveDuplicateQueries(), 0U);
  EXPECT_EQ(world->repeatedUndirectedEdgeQueries(), 0U);
}

TEST(PlannerTrajectory, DoesNotTruncateRouteWithinBound) {
  const navigation_math::vec_Vec3f path{
      navigation_math::Vec3f{0.0F, 0.0F, 3.0F},
      navigation_math::Vec3f{2.0F, 0.0F, 3.0F},
      navigation_math::Vec3f{2.0F, 2.0F, 3.0F}};
  navigation_math::vec_Vec3f prefix;
  bool truncated = true;
  ASSERT_TRUE(geometry_utils::truncatePathAtDistance(
      path, 10.0, prefix, truncated));
  EXPECT_FALSE(truncated);
  ASSERT_EQ(prefix.size(), path.size());
  for (std::size_t index = 0U; index < path.size(); ++index) {
    EXPECT_TRUE(prefix[index].isApprox(path[index]));
  }
  EXPECT_LE(geometry_utils::computePathLength(prefix), 10.0);
}

TEST(PlannerTrajectory, RejectsInvalidRoutePrefixBounds) {
  navigation_math::vec_Vec3f prefix;
  bool truncated = false;
  EXPECT_FALSE(geometry_utils::truncatePathAtDistance(
      navigation_math::vec_Vec3f{navigation_math::Vec3f::Zero()},
      5.0, prefix, truncated));
  EXPECT_FALSE(geometry_utils::truncatePathAtDistance(
      navigation_math::vec_Vec3f{
          navigation_math::Vec3f::Zero(),
          navigation_math::Vec3f{1.0F, 0.0F, 0.0F}},
      0.0, prefix, truncated));
}

TEST(PlannerTrajectory, SparseGuideSubdivisionRejectsInvalidInputs) {
  navigation_math::vec_Vec3f subdivided;
  EXPECT_FALSE(geometry_utils::subdividePathByMaximumSegmentLength(
      {}, 3.0, subdivided));
  EXPECT_FALSE(geometry_utils::subdividePathByMaximumSegmentLength(
      navigation_math::vec_Vec3f{navigation_math::Vec3f::Zero()}, 0.0, subdivided));
  EXPECT_FALSE(geometry_utils::subdividePathByMaximumSegmentLength(
      navigation_math::vec_Vec3f{
          navigation_math::Vec3f::Zero(),
          navigation_math::Vec3f{
              std::numeric_limits<float>::quiet_NaN(), 0.0F, 0.0F}},
      3.0, subdivided));
}

TEST(PlannerTrajectory, RouteBoundaryCannotCreateOverlongLineSeed) {
  auto world = std::make_shared<SweepWorld>();
  navigation_planner_context::PlannerRuntimeContext::Ptr context =
      std::make_shared<navigation_planner_context::PlannerRuntimeContext>();
  navigation_planning_backend::CorridorGenerator generator(
      context, world, 10.0, 3.0, 0.1, -10.0, 10.0, 0.35, 1,
      navigation_world_model::UnknownPolicy::kAllowUnknown);

  const navigation_math::vec_Vec3f path{
      navigation_math::Vec3f{0.0F, 0.0F, 3.0F},
      navigation_math::Vec3f{6.0F, 0.0F, 3.0F},
      navigation_math::Vec3f{6.0F, -5.0F, 3.0F}};
  navigation_planning_backend::CorridorGenerator::RouteBoundaryGate gate{
      navigation_math::Vec3f{6.0F, 0.0F, 3.0F}, 0.9};
  geometry_utils::PolytopeVec sfcs;
  navigation_math::Vec3f shifted_start;
  ASSERT_TRUE(generator.SearchPolytopeOnPath(path, sfcs, shifted_start, false,
                                             nullptr, gate));
  ASSERT_FALSE(sfcs.empty());
  const auto gate_polytope = std::find_if(
      sfcs.begin(), sfcs.end(), [](const auto &polytope) {
        return polytope.IsRouteBoundaryGate();
      });
  ASSERT_NE(gate_polytope, sfcs.end());
  const auto centre = gate.point;
  EXPECT_TRUE(gate_polytope->PointIsInside(centre, 1.0e-6));
  // The gate cell is an inner approximation, not a box circumscribing the
  // acceptance ball. A point 0.8 radius away on one axis remains inside the
  // sphere but must not be authorized by this convex cell.
  EXPECT_FALSE(gate_polytope->PointIsInside(
      centre + navigation_math::Vec3f{0.8 * gate.radius_m, 0.0, 0.0},
      1.0e-6));
  const auto vertical_envelope =
      navigation_planning_backend::deriveGuideVerticalEnvelope(path, 0.2);
  EXPECT_TRUE(navigation_planning_backend::applyGuideVerticalEnvelope(
      sfcs, vertical_envelope));
}

TEST(PlannerTrajectory, GuideTimeAllocationPreservesNonZeroInitialSpeedOnShortPath) {
  const navigation_math::Vec3f start(0.0, 0.0, 0.0);
  const navigation_math::vec_Vec3f short_path{
      start, navigation_math::Vec3f(1.0, 0.0, 0.0)};
  geometry_utils::GuideTimeAllocation allocation;
  ASSERT_TRUE(geometry_utils::allocateGuideElapsedTimes(
      2.0, 3.0, 2.4, start, short_path, allocation));
  ASSERT_EQ(allocation.points.size(), 1U);
  EXPECT_NEAR(allocation.elapsed_s.back(), 0.5367, 0.005);
  EXPECT_NEAR(allocation.terminal_velocity_mps, 1.3266, 0.005);
  EXPECT_GE(allocation.terminal_velocity_mps, 0.0);
}

TEST(PlannerTrajectory, GuideTimeAllocationRejectsDegenerateOrInvalidInputs) {
  const navigation_math::Vec3f start(0.0, 0.0, 0.0);
  const navigation_math::vec_E<navigation_math::Vec3f> duplicate_path{start, start};
  geometry_utils::GuideTimeAllocation allocation;
  EXPECT_FALSE(geometry_utils::allocateGuideElapsedTimes(
      5.0, 10.0, 0.0, start, duplicate_path, allocation));
  EXPECT_FALSE(geometry_utils::allocateGuideElapsedTimes(
      0.0, 10.0, 0.0, start,
      navigation_math::vec_E<navigation_math::Vec3f>{navigation_math::Vec3f(1.0, 0.0, 0.0)},
      allocation));

  double elapsed = 0.0;
  double velocity = 0.0;
  geometry_utils::simplePMTimeAllocator(
      std::numeric_limits<double>::quiet_NaN(), 10.0, 0.0, 1.0, 0.5,
      elapsed, velocity);
  EXPECT_FALSE(std::isfinite(elapsed));
  EXPECT_FALSE(std::isfinite(velocity));
  geometry_utils::simplePMTimeAllocator(5.0, 10.0, 0.0, 1.0, 1.5,
                                        elapsed, velocity);
  EXPECT_FALSE(std::isfinite(elapsed));
  EXPECT_FALSE(std::isfinite(velocity));
}

TEST(PlannerTrajectory, GoalConnectionUsesInclusiveBoundary) {
  const navigation_math::Vec3f guide_endpoint(0.0, 0.0, 0.0);
  const navigation_math::Vec3f goal(0.2, 0.0, 0.0);
  const auto result = navigation_planning_backend::resolveGuideEndpoint(
      guide_endpoint, goal, 0.2);
  EXPECT_TRUE(result.goal_connected);
  EXPECT_TRUE(result.position.isApprox(goal));
}

TEST(PlannerTrajectory, SolveFailureCodesRemainDistinct) {
  EXPECT_EQ(navigation_planning_backend::PlannerResultCode_STR(
                navigation_planning_backend::PLANNER_SOLVE_TIMEOUT),
            "Solve deadline exhausted before a candidate could be committed");
  EXPECT_EQ(navigation_planning_backend::PlannerResultCode_STR(
                navigation_planning_backend::PLANNER_SOLVE_CANCELLED),
            "Solve was cancelled before a candidate could be committed");
  EXPECT_NE(navigation_planning_backend::PLANNER_SOLVE_TIMEOUT,
            navigation_planning_backend::PLANNER_BACKUP_FAILED);
  EXPECT_NE(navigation_planning_backend::PLANNER_SOLVE_CANCELLED,
            navigation_planning_backend::PLANNER_BACKUP_FAILED);
  EXPECT_NE(navigation_planning_backend::PLANNER_EXP_FAILED,
            navigation_planning_backend::PLANNER_BACKUP_FAILED);
  EXPECT_EQ(navigation_planning_backend::PlannerResultCode_STR(
                navigation_planning_backend::PLANNER_CANDIDATE_REJECTED),
            "Generated candidate failed construction or world validation");
}

TEST(PlannerTrajectory, BackupFailureKeepsActionableCause) {
  EXPECT_EQ(navigation_planning_backend::classifyBackupResult(
                navigation_math::TIME_OUT),
            navigation_planning_backend::PLANNER_SOLVE_TIMEOUT);
  EXPECT_EQ(navigation_planning_backend::classifyBackupResult(
                navigation_math::OPT_FAILED),
            navigation_planning_backend::PLANNER_BACKUP_OPTIMIZATION_FAILED);
  EXPECT_EQ(navigation_planning_backend::classifyBackupResult(
                navigation_math::INIT_ERROR),
            navigation_planning_backend::PLANNER_BACKUP_INITIALIZATION_FAILED);
  EXPECT_EQ(navigation_planning_backend::classifyBackupResult(
                navigation_math::NO_PATH),
            navigation_planning_backend::PLANNER_BACKUP_NO_PATH);
  EXPECT_EQ(navigation_planning_backend::classifyBackupResult(
                navigation_math::FAILED),
            navigation_planning_backend::PLANNER_BACKUP_FAILED);
}

TEST(PlannerTrajectory, UnconnectedOrInvalidGoalDoesNotMoveGuideEndpoint) {
  const navigation_math::Vec3f guide_endpoint(1.0, 2.0, 3.0);
  const navigation_math::Vec3f goal(2.0, 2.0, 3.0);
  const auto unconnected = navigation_planning_backend::resolveGuideEndpoint(
      guide_endpoint, goal, 0.4);
  EXPECT_FALSE(unconnected.goal_connected);
  EXPECT_TRUE(unconnected.position.isApprox(guide_endpoint));

  navigation_math::Vec3f invalid_goal = goal;
  invalid_goal.x() = std::numeric_limits<double>::quiet_NaN();
  const auto invalid = navigation_planning_backend::resolveGuideEndpoint(
      guide_endpoint, invalid_goal, 0.4);
  EXPECT_FALSE(invalid.goal_connected);
  EXPECT_TRUE(invalid.position.isApprox(guide_endpoint));
}

TEST(PlannerTrajectory, SolveStagesHaveStableDecisionTraceNames) {
  EXPECT_EQ(navigation_planning_backend::solveStageName(0), "idle");
  EXPECT_EQ(navigation_planning_backend::solveStageName(2), "astar");
  EXPECT_EQ(navigation_planning_backend::solveStageName(4), "main_minco");
  EXPECT_EQ(navigation_planning_backend::solveStageName(5), "backup");
  EXPECT_EQ(navigation_planning_backend::solveStageName(33), "corridor_iris");
  EXPECT_EQ(navigation_planning_backend::solveStageName(999), "unknown");
}

TEST(PlannerTrajectory, UnknownReturnCodeHasDeterministicDiagnostic) {
  EXPECT_EQ(navigation_planning_backend::PlannerResultCode_STR(42),
            "Unknown planner return code (42)");
}
