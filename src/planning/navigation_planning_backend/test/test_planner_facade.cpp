#include <navigation_planning_backend/planner_facade.hpp>
#include <planner_core/route_yaw_reference.hpp>
#include <navigation_world_model/continuous_clearance.hpp>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <functional>
#include <memory>
#include <optional>

#include <gtest/gtest.h>

namespace {

class IdentityOnlyWorld final : public navigation_world_model::WorldModelView {
 public:
  navigation_world_model::PointVector occupied_points;

  navigation_world_model::WorldGeometry geometry() const noexcept override {
    navigation_world_model::WorldGeometry result;
    result.evidence_resolution_m = 0.2;
    result.inflated_resolution_m = 0.2;
    result.occupied_inflation_radius_m = 1.0;
    result.effective_virtual_ground_m = -1.0;
    result.effective_virtual_ceiling_m = 5.0;
    // Match the product-locked planner envelope. This fixture must not mask
    // per-axis geometry validation with the former flight-map dimensions.
    result.local_size_m = Eigen::Vector3d{50.0, 50.0, 8.0};
    result.evidence_bounds.global_min_index = Eigen::Vector3i{-125, -125, -20};
    result.evidence_bounds.dimensions = Eigen::Vector3i{250, 250, 40};
    result.inflated_bounds.global_min_index = Eigen::Vector3i{-125, -125, -20};
    result.inflated_bounds.dimensions = Eigen::Vector3i{250, 250, 40};
    return result;
  }

  navigation_world_model::WorldSnapshotIdentity identity() const noexcept override {
    return {1, 1, 1, 100};
  }

  navigation_world_model::CellState classify(
      const navigation_world_model::Point3&, navigation_world_model::GridLayer) const noexcept override {
    return navigation_world_model::CellState::kKnownFree;
  }
  bool contains(const navigation_world_model::Point3&) const noexcept override { return true; }
  navigation_world_model::GridIndex3 positionToIndex(
      const navigation_world_model::Point3&, navigation_world_model::GridLayer) const noexcept override {
    return navigation_world_model::GridIndex3::Zero();
  }
  navigation_world_model::Point3 indexToPosition(
      const navigation_world_model::GridIndex3&, navigation_world_model::GridLayer) const noexcept override {
    return navigation_world_model::Point3::Zero();
  }
  std::optional<navigation_world_model::Point3> nearestNotOccupied(
      const navigation_world_model::Point3& point,
      navigation_world_model::GridLayer, double) const override {
    return point;
  }
  bool isSegmentTraversable(
      const navigation_world_model::Point3&, const navigation_world_model::Point3&,
      navigation_world_model::GridLayer,
      navigation_world_model::UnknownPolicy) const noexcept override {
    return true;
  }
  navigation_world_model::AxisAlignedBox clampToLocalBounds(
      const navigation_world_model::AxisAlignedBox& box) const noexcept override {
    return box;
  }
  navigation_world_model::PointVector observedOccupiedPoints(
      const navigation_world_model::AxisAlignedBox& box) const override {
    navigation_world_model::PointVector result;
    for (const auto& point : occupied_points) {
      if ((point.array() >= box.minimum.array()).all() &&
          (point.array() <= box.maximum.array()).all()) {
        result.push_back(point);
      }
    }
    return result;
  }
};

TEST(WorldGeometryBoundaries, ContinuousClearanceRejectsOverflowingDerivedQueryBox) {
  const auto world = std::make_shared<IdentityOnlyWorld>();
  const auto limit = std::numeric_limits<double>::max();
  EXPECT_FALSE(navigation_world_model::observedOccupiedTubeIsClear(
      *world, Eigen::Vector3d{limit, 0.0, 0.0},
      Eigen::Vector3d{-limit, 0.0, 0.0}, limit));
}

TEST(WorldGeometryBoundaries,
     ContinuousClearanceRejectsToleranceBoundaryAndZeroRadiusCollision) {
  auto world = std::make_shared<IdentityOnlyWorld>();
  const double tolerance_boundary = 0.35 - 0.01;
  world->occupied_points.push_back(
      Eigen::Vector3d{0.0, tolerance_boundary, 0.0});
  EXPECT_FALSE(navigation_world_model::observedOccupiedTubeIsClear(
      *world, Eigen::Vector3d{-1.0, 0.0, 0.0},
      Eigen::Vector3d{1.0, 0.0, 0.0}, 0.35));

  world->occupied_points.clear();
  world->occupied_points.push_back(Eigen::Vector3d{0.0, 0.0, 0.0});
  EXPECT_FALSE(navigation_world_model::observedOccupiedTubeIsClear(
      *world, Eigen::Vector3d{-1.0, 0.0, 0.0},
      Eigen::Vector3d{1.0, 0.0, 0.0}, 0.0));
}

class TestCommitAuthorizer final : public navigation_world_model::WorldCommitAuthorizer {
 public:
  explicit TestCommitAuthorizer(navigation_world_model::WorldModelViewPtr world)
      : world_(std::move(world)) {}

  navigation_world_model::WorldValidationLease latest() const noexcept override {
    return {world_, world_->identity()};
  }

  navigation_world_model::WorldCommitDecision commitIfCurrent(
      const navigation_world_model::WorldSnapshotIdentity& identity,
      const std::function<bool()>& commit) override {
    if (!navigation_world_model::sameWorldSnapshotIdentity(identity, world_->identity())) {
      return navigation_world_model::WorldCommitDecision::kWorldAdvanced;
    }
    return commit() ? navigation_world_model::WorldCommitDecision::kCommitted
                    : navigation_world_model::WorldCommitDecision::kCancelled;
  }

 private:
  navigation_world_model::WorldModelViewPtr world_;
};

struct BoundaryEntrySample final {
  double trajectory_time_s{0.0};
  navigation_planning::CandidateRole role{navigation_planning::CandidateRole::kMain};
};

std::optional<BoundaryEntrySample> firstBoundaryEntry(
    const navigation_planning::CandidateBundle& candidate,
    const Eigen::Vector3d& boundary_center,
    const double acceptance_radius_m) {
  if (!candidate.evaluator || !boundary_center.allFinite() ||
      !std::isfinite(acceptance_radius_m) || acceptance_radius_m <= 0.0 ||
      !std::isfinite(candidate.duration_s) || candidate.duration_s < 0.0 ||
      candidate.declared_start_ns <= 0) {
    return std::nullopt;
  }
  const Eigen::Vector3d boundary_min = boundary_center -
      Eigen::Vector3d::Constant(acceptance_radius_m);
  const Eigen::Vector3d boundary_max = boundary_center +
      Eigen::Vector3d::Constant(acceptance_radius_m);
  const auto sample = [&](const double trajectory_time_s,
                          navigation_planning::TrajectoryPoint& point) {
    const auto offset_ns = static_cast<std::int64_t>(std::llround(
        trajectory_time_s * 1.0e9));
    return candidate.evaluator(candidate.declared_start_ns + offset_ns, point);
  };
  const auto inside = [&](const double trajectory_time_s,
                          navigation_planning::TrajectoryPoint& point) {
    return sample(trajectory_time_s, point) &&
        (point.position_world.array() >= boundary_min.array()).all() &&
        (point.position_world.array() <= boundary_max.array()).all();
  };

  constexpr double kProbeDtS = 0.005;
  const auto probe_count = static_cast<std::size_t>(std::ceil(
      candidate.duration_s / kProbeDtS));
  if (probe_count > 10000000U) return std::nullopt;
  navigation_planning::TrajectoryPoint previous_point;
  bool previous_inside = inside(0.0, previous_point);
  if (!previous_point.finite()) return std::nullopt;
  double previous_time_s = 0.0;
  for (std::size_t probe = 1U; probe <= probe_count; ++probe) {
    const double current_time_s = std::min(
        candidate.duration_s, static_cast<double>(probe) * kProbeDtS);
    navigation_planning::TrajectoryPoint current_point;
    const bool current_inside = inside(current_time_s, current_point);
    if (!current_point.finite()) return std::nullopt;
    if (current_inside && !previous_inside) {
      double lower = previous_time_s;
      double upper = current_time_s;
      for (int iteration = 0; iteration < 32; ++iteration) {
        const double middle = 0.5 * (lower + upper);
        navigation_planning::TrajectoryPoint middle_point;
        if (inside(middle, middle_point)) {
          upper = middle;
        } else {
          lower = middle;
        }
      }
      navigation_planning::TrajectoryPoint entry_point;
      if (!inside(upper, entry_point)) return std::nullopt;
      return BoundaryEntrySample{upper, entry_point.role};
    }
    previous_time_s = current_time_s;
    previous_inside = current_inside;
    if (current_time_s >= candidate.duration_s) break;
  }
  return std::nullopt;
}

TEST(PlannerFacade, ExposesOnlyProductStateBeforeFirstCommit) {
  auto world = std::make_shared<IdentityOnlyWorld>();
  TestCommitAuthorizer authorizer(world);
  navigation_planning_backend::PlannerFacade facade(
      PLANNER_FACADE_CONFIG_PATH, world, std::nullopt, authorizer, [] { return 10.0; });

  EXPECT_EQ(facade.solveStage(), 0);
  EXPECT_EQ(facade.solvePointCount(), 0U);
  EXPECT_DOUBLE_EQ(facade.trackingErrorBudgetMeters(), 0.25);
  const auto snapshot = facade.committedSnapshot();
  EXPECT_TRUE(snapshot.empty());
  EXPECT_EQ(snapshot.generation, 0U);
  EXPECT_EQ(snapshot.certificate.pinned_world.generation, 0U);
  EXPECT_EQ(snapshot.certificate.validated_world.generation, 0U);

  const auto diagnostics = facade.diagnostics();
  EXPECT_EQ(diagnostics.solve_stage, 0);
  EXPECT_EQ(diagnostics.solve_point_count, 0U);
  EXPECT_EQ(diagnostics.module_time_us[0], 0.0);
  EXPECT_EQ(diagnostics.route_yaw_source,
            static_cast<int>(navigation_planning_backend::RouteYawSource::kInvalidRoute));
  EXPECT_DOUBLE_EQ(diagnostics.yaw_rate_limit_rad_s, 1.0);
  EXPECT_DOUBLE_EQ(diagnostics.yaw_acceleration_limit_rad_s2, 0.3);
}

TEST(PlannerFacade, RequiresValidImmutableRouteBeforePlanning) {
  auto world = std::make_shared<IdentityOnlyWorld>();
  TestCommitAuthorizer authorizer(world);
  navigation_planning_backend::PlannerFacade facade(
      PLANNER_FACADE_CONFIG_PATH, world, std::nullopt, authorizer, [] { return 10.0; });

  navigation_mission::ImmutableRouteSnapshot invalid;
  EXPECT_FALSE(facade.setRouteSnapshot(invalid));

  navigation_mission::Mission mission;
  mission.id = "route-contract";
  mission.frame = "lio_odom";
  navigation_mission::MissionWaypoint first;
  first.id = "wp-0";
  first.position_enu = Eigen::Vector3d{0.0, 0.0, 2.0};
  first.acceptance_radius_m = 1.0;
  navigation_mission::MissionWaypoint second;
  second.id = "wp-1";
  second.position_enu = Eigen::Vector3d{10.0, 0.0, 2.0};
  second.acceptance_radius_m = 1.0;
  mission.waypoints = {first, second};
  navigation_mission::RouteProgress progress(mission);
  ASSERT_TRUE(progress.update(Eigen::Vector3d{1.0, 0.0, 2.0}).valid);
  const auto snapshot = progress.snapshot(
      mission.id, mission.frame, 1U, 4U, 0U);

  ASSERT_TRUE(snapshot.valid());
  EXPECT_TRUE(facade.setRouteSnapshot(snapshot));
}

TEST(PlannerFacade, PassThroughLookaheadExportsRouteBoundaryEvent) {
  auto world = std::make_shared<IdentityOnlyWorld>();
  TestCommitAuthorizer authorizer(world);
  navigation_planning_backend::PlannerFacade facade(
      PLANNER_FACADE_CONFIG_PATH, world, std::nullopt, authorizer, [] { return 10.0; });

  navigation_mission::Mission mission;
  mission.id = "lookahead-boundary-contract";
  mission.frame = "lio_odom";
  navigation_mission::MissionWaypoint active;
  active.id = "active";
  active.position_enu = Eigen::Vector3d{10.0, 0.0, 3.0};
  active.behavior = navigation_mission::MissionWaypoint::Behavior::PassThrough;
  active.acceptance_radius_m = 0.5;
  navigation_mission::MissionWaypoint next;
  next.id = "next";
  next.position_enu = Eigen::Vector3d{20.0, 0.0, 3.0};
  next.behavior = navigation_mission::MissionWaypoint::Behavior::Stop;
  next.acceptance_radius_m = 0.5;
  mission.waypoints = {active, next};
  navigation_mission::RouteProgress progress(mission);
  ASSERT_TRUE(progress.update(Eigen::Vector3d{0.0, 0.0, 3.0}).valid);
  const auto route = progress.snapshot(mission.id, mission.frame, 1U, 1U, 0U);
  ASSERT_TRUE(route.valid());
  ASSERT_TRUE(facade.setRouteSnapshot(route));

  navigation_planning::KinematicState state;
  state.position_world = Eigen::Vector3d{0.0, 0.0, 3.0};
  state.source_stamp_ns = 1;
  state.receive_stamp_ns = 1;
  state.localization_epoch = 1U;
  state.world_frame_id = "lio_odom";
  state.body_frame_id = "base_link";
  ASSERT_TRUE(facade.setState(state));
  facade.setCommandIdentity(1U, 1U, 1U);
  facade.setGoalAcceptanceRadius(active.acceptance_radius_m);

  const auto status = facade.planInitialFromStoppedState(active.position_enu, 0.0, true);
  ASSERT_EQ(status, navigation_planning::PlannerStatus::kSuccess);
  const auto candidate = facade.exportCommandCandidate(1U, 1U, 1U, 10000000000LL,
                                                       20000000000LL);
  ASSERT_TRUE(candidate);
  ASSERT_TRUE(candidate->route_boundary_constraint.has_value());
  ASSERT_TRUE(candidate->route_boundary_event.has_value());
  EXPECT_EQ(candidate->route_boundary_event->kind,
            navigation_planning::RouteBoundaryEventKind::kPassThrough);
  EXPECT_EQ(candidate->route_boundary_event->junction_index, 0U);
  EXPECT_TRUE(candidate->route_boundary_constraint->contains(
      candidate->route_boundary_event->position_world));
}

TEST(PlannerFacade, PassThroughLookaheadPrefixWithoutBoundaryEntryStaysValid) {
  auto world = std::make_shared<IdentityOnlyWorld>();
  TestCommitAuthorizer authorizer(world);
  navigation_planning_backend::PlannerFacade facade(
      PLANNER_FACADE_CONFIG_PATH, world, std::nullopt, authorizer, [] { return 10.0; });

  navigation_mission::Mission mission;
  mission.id = "lookahead-prefix-contract";
  mission.frame = "lio_odom";
  navigation_mission::MissionWaypoint active;
  active.id = "active";
  active.position_enu = Eigen::Vector3d{10.0, 0.0, 3.0};
  active.behavior = navigation_mission::MissionWaypoint::Behavior::PassThrough;
  active.acceptance_radius_m = 0.5;
  navigation_mission::MissionWaypoint next;
  next.id = "next";
  next.position_enu = Eigen::Vector3d{20.0, 0.0, 3.0};
  next.behavior = navigation_mission::MissionWaypoint::Behavior::Stop;
  next.acceptance_radius_m = 0.5;
  mission.waypoints = {active, next};
  navigation_mission::RouteProgress progress(mission);
  ASSERT_TRUE(progress.update(Eigen::Vector3d{0.0, 0.0, 3.0}).valid);
  ASSERT_TRUE(facade.setRouteSnapshot(progress.snapshot(
      mission.id, mission.frame, 1U, 1U, 0U)));

  navigation_planning::KinematicState state;
  state.position_world = Eigen::Vector3d{0.0, 0.0, 3.0};
  state.source_stamp_ns = 1;
  state.receive_stamp_ns = 1;
  state.localization_epoch = 1U;
  state.world_frame_id = "lio_odom";
  state.body_frame_id = "base_link";
  ASSERT_TRUE(facade.setState(state));
  facade.setCommandIdentity(1U, 1U, 1U);
  facade.setGoalAcceptanceRadius(active.acceptance_radius_m);

  ASSERT_EQ(facade.planInitialFromStoppedState(
                Eigen::Vector3d{5.0, 0.0, 3.0}, 0.0, true),
            navigation_planning::PlannerStatus::kSuccess);
  const auto candidate = facade.exportCommandCandidate(1U, 1U, 1U, 10000000000LL,
                                                       20000000000LL);
  ASSERT_TRUE(candidate);
  ASSERT_TRUE(candidate->valid());
  EXPECT_FALSE(candidate->route_boundary_event.has_value());
}

TEST(PlannerFacade, PassThroughEntryAfterBackupDoesNotAdvertiseBoundaryEvent) {
  auto world = std::make_shared<IdentityOnlyWorld>();
  TestCommitAuthorizer authorizer(world);
  navigation_planning_backend::PlannerFacade facade(
      PLANNER_FACADE_CONFIG_PATH, world, std::nullopt, authorizer, [] { return 10.0; });

  navigation_mission::Mission mission;
  mission.id = "backup-boundary-contract";
  mission.frame = "lio_odom";
  navigation_mission::MissionWaypoint active;
  active.id = "active";
  active.position_enu = Eigen::Vector3d{5.55, 0.0, 1.736};
  active.behavior = navigation_mission::MissionWaypoint::Behavior::PassThrough;
  active.acceptance_radius_m = 0.2;
  navigation_mission::MissionWaypoint next;
  next.id = "next";
  next.position_enu = Eigen::Vector3d{20.0, 0.0, 3.0};
  next.behavior = navigation_mission::MissionWaypoint::Behavior::Stop;
  next.acceptance_radius_m = 0.5;
  mission.waypoints = {active, next};
  navigation_mission::RouteProgress progress(mission);
  ASSERT_TRUE(progress.update(Eigen::Vector3d{0.0, 0.0, 3.0}).valid);
  ASSERT_TRUE(facade.setRouteSnapshot(progress.snapshot(
      mission.id, mission.frame, 1U, 1U, 0U)));

  navigation_planning::KinematicState state;
  state.position_world = Eigen::Vector3d{0.0, 0.0, 3.0};
  state.source_stamp_ns = 1;
  state.receive_stamp_ns = 1;
  state.localization_epoch = 1U;
  state.world_frame_id = "lio_odom";
  state.body_frame_id = "base_link";
  ASSERT_TRUE(facade.setState(state));
  facade.setCommandIdentity(1U, 1U, 1U);
  // Keep the planner's corridor envelope independent of the deliberately
  // tighter route-event volume used to place the witness in BACKUP.
  facade.setGoalAcceptanceRadius(0.5);

  ASSERT_EQ(facade.planInitialFromStoppedState(
                Eigen::Vector3d{10.0, 0.0, 3.0}, 0.0, true),
            navigation_planning::PlannerStatus::kSuccess);
  const auto candidate = facade.exportCommandCandidate(1U, 1U, 1U, 10000000000LL,
                                                       20000000000LL);
  ASSERT_TRUE(candidate);
  ASSERT_TRUE(candidate->valid());
  ASSERT_TRUE(candidate->backup_available);
  const auto entry = firstBoundaryEntry(
      *candidate, active.position_enu,
      std::max(navigation_world_model::kGoalCompletionToleranceM,
               active.acceptance_radius_m));
  ASSERT_TRUE(entry.has_value());
  EXPECT_GT(entry->trajectory_time_s, candidate->backup_start_time_s + 1.0e-6);
  EXPECT_EQ(entry->role, navigation_planning::CandidateRole::kBackup);
  EXPECT_FALSE(candidate->route_boundary_event.has_value());
  EXPECT_FALSE(candidate->route_boundary_constraint.has_value());
}

}  // namespace
