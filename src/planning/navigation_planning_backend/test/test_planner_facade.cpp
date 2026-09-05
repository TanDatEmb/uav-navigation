#include <navigation_planning_backend/planner_facade.hpp>
#include <navigation_mapping/current_body_support.hpp>
#include <navigation_mapping/mapping_actor.hpp>
#include <navigation_mapping/mapping_observation.hpp>
#include <planner_core/route_yaw_reference.hpp>
#include <navigation_world_model/continuous_clearance.hpp>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <chrono>
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

struct ActualMappingFixture final {
  std::unique_ptr<navigation_mapping::MappingActor> actor;
  navigation_world_model::WorldModelViewPtr snapshot;
  navigation_world_model::CurrentBodySupportPtr body_support;
  navigation_planning::KinematicState start_state;
  navigation_mission::ImmutableRouteSnapshot route;
};

ActualMappingFixture actualMappingWorldSnapshot() {
  ActualMappingFixture fixture;
  fixture.actor = std::make_unique<navigation_mapping::MappingActor>(
      PLANNER_FACADE_CONFIG_PATH);

  constexpr std::int64_t kStampNs = 20'000'000'000LL;
  // One raycast call coalesces repeated voxel operations. Advance the real
  // MappingActor through successive timestamps so its probability map reaches
  // sensor-free evidence while the measured body cell remains UNKNOWN.
  for (std::int64_t iteration = 1; iteration <= 20; ++iteration) {
    const auto stamp_ns = iteration * 1'000'000'000LL;
    auto cloud = std::make_unique<navigation_mapping::PointCloud>();
    cloud->push_back(navigation_mapping::PointXYZI{4.0F, 0.0F, 1.5F, 0.0F});
    auto free_endpoints = std::make_unique<navigation_mapping::PointCloud>();
    free_endpoints->push_back(
        navigation_mapping::PointXYZI{1.0F, 0.0F, 1.5F, 0.0F});
    nav_msgs::msg::Odometry odometry;
    odometry.header.stamp.sec = static_cast<std::int32_t>(iteration);
    odometry.header.stamp.nanosec = 0U;
    odometry.header.frame_id = "lio_odom";
    odometry.child_frame_id = "base_link";
    odometry.pose.pose.orientation.w = 1.0;
    odometry.pose.pose.position.z = 1.5;
    navigation_mapping::MappingObservation observation{
        std::move(cloud), std::move(odometry), 1U,
        static_cast<std::uint64_t>(iteration), stamp_ns, 0};
    observation.free_space_endpoints = std::move(free_endpoints);
    observation.sensor_origin_world = Eigen::Vector3d{0.35, 0.0, 1.5};
    observation.sensor_origin_localization_epoch = 1U;
    observation.sensor_origin_stamp_ns = stamp_ns;
    const auto update = fixture.actor->process(observation);
    fixture.snapshot = update.snapshot;
  }
  if (!fixture.snapshot) return fixture;

  const auto support = navigation_mapping::makeX500Mid360CurrentBodySupport(
      Eigen::Vector3d{0.01, 0.01, 1.5}, Eigen::Quaterniond::Identity(),
      fixture.snapshot->identity(), "lio_odom", "base_link", 1U,
      kStampNs);
  if (!support.valid) return fixture;
  fixture.body_support = std::make_shared<const navigation_world_model::CurrentBodySupport>(
      support);

  navigation_mission::Mission mission;
  mission.id = "current-body-production";
  mission.frame = "lio_odom";
  mission.planning.requested_cruise_speed_mps = 1.0;
  mission.waypoints = {
      navigation_mission::MissionWaypoint{
          "start", Eigen::Vector3d{0.01, 0.01, 1.5}, 0.2, 0.0,
          navigation_mission::MissionWaypoint::Behavior::PassThrough},
      navigation_mission::MissionWaypoint{
          "goal", Eigen::Vector3d{1.0, 0.0, 1.5}, 0.3, 0.0,
          navigation_mission::MissionWaypoint::Behavior::Stop},
  };
  navigation_mission::RouteProgress progress(mission);
  if (!progress.update(Eigen::Vector3d{0.01, 0.01, 1.5}).valid) return fixture;
  fixture.route = progress.snapshot(mission.id, mission.frame, 1U, 17U, 1U);

  fixture.start_state.position_world = Eigen::Vector3d{0.01, 0.01, 1.5};
  fixture.start_state.orientation_world_body = Eigen::Quaterniond::Identity();
  fixture.start_state.source_stamp_ns = kStampNs;
  fixture.start_state.receive_stamp_ns = kStampNs;
  fixture.start_state.localization_epoch = 1U;
  fixture.start_state.world_frame_id = "lio_odom";
  fixture.start_state.body_frame_id = "base_link";
  return fixture;
}

navigation_planning::PlanningRequest productionRequest(
    const ActualMappingFixture& fixture,
    const bool with_body_support) {
  navigation_planning::PlanningRequest request;
  const auto identity = fixture.snapshot->identity();
  request.key.localization_epoch = 1U;
  request.key.goal_epoch = 1U;
  request.key.request_id = 17U;
  request.key.route_revision = fixture.route.route_revision;
  request.key.pinned_world_generation = identity.generation;
  request.key.pinned_world_revision = identity.revision;
  request.key.start_mode = navigation_planning::PlanningStartMode::kStoppedMeasuredState;
  request.key.anchor_stamp_ns = fixture.start_state.source_stamp_ns;
  request.key.dynamics_hash = 1U;
  request.goal = navigation_planning::GoalIdentity{
      1U, 1U, fixture.route.mission_id, 1U, 17U};
  request.start_state = fixture.start_state;
  request.route_snapshot = fixture.route;
  request.world = fixture.snapshot;
  request.current_body_support = with_body_support ? fixture.body_support : nullptr;
  request.dynamics.intent.requested_cruise_speed_mps = 1.0;
  request.dynamics.unknown_space_policy = navigation_world_model::UnknownPolicy::kRequireKnownFree;
  request.budget.deadline = navigation_planning::PlanningBudget::Clock::now() +
      std::chrono::seconds(10);
  return request;
}

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

TEST(PlannerFacade, ProductionPlanUsesMappingSnapshotBodyAdmission) {
  auto fixture = actualMappingWorldSnapshot();
  ASSERT_TRUE(fixture.snapshot);
  ASSERT_TRUE(fixture.body_support);
  ASSERT_TRUE(fixture.route.valid());
  ASSERT_TRUE(fixture.start_state.finite());
  ASSERT_EQ(fixture.snapshot->classify(
                fixture.start_state.position_world,
                navigation_world_model::GridLayer::kInflated),
            navigation_world_model::CellState::kUnknown);
  ASSERT_TRUE(fixture.body_support->containsSegment(
      fixture.start_state.position_world, Eigen::Vector3d{0.11, 0.01, 1.5},
      fixture.snapshot->identity(), fixture.snapshot->identity().observation_stamp_ns));
  ASSERT_NEAR(fixture.body_support->contiguousBodyPrefixFraction(
      fixture.start_state.position_world, Eigen::Vector3d{0.11, 0.01, 1.5}),
      1.0, 1.0e-12);
  EXPECT_EQ(fixture.snapshot->classify(
                Eigen::Vector3d{0.1, 0.1, 1.5},
                navigation_world_model::GridLayer::kEvidence),
            navigation_world_model::CellState::kUnknown);
  ASSERT_TRUE(fixture.snapshot->isSegmentTraversableWithCurrentBodySupport(
      fixture.start_state.position_world, Eigen::Vector3d{0.11, 0.01, 1.5},
      navigation_world_model::GridLayer::kEvidence,
      navigation_world_model::UnknownPolicy::kRequireKnownFree,
      fixture.body_support));
  ASSERT_TRUE(fixture.snapshot->isSegmentTraversable(
      Eigen::Vector3d{0.5, 0.0, 1.5}, Eigen::Vector3d{0.5, 0.0, 1.5},
      navigation_world_model::GridLayer::kEvidence,
      navigation_world_model::UnknownPolicy::kRequireKnownFree));
  EXPECT_EQ(fixture.snapshot->classify(
                Eigen::Vector3d{0.5, 0.0, 1.5},
                navigation_world_model::GridLayer::kInflated),
            navigation_world_model::CellState::kKnownFree);
  ASSERT_TRUE(fixture.snapshot->isSegmentTraversable(
      Eigen::Vector3d{0.5, 0.0, 1.5}, Eigen::Vector3d{0.5, 0.0, 1.5},
      navigation_world_model::GridLayer::kInflated,
      navigation_world_model::UnknownPolicy::kRequireKnownFree));
  ASSERT_TRUE(fixture.snapshot->isSegmentTraversable(
      Eigen::Vector3d{0.35, 0.0, 1.5}, Eigen::Vector3d{1.0, 0.0, 1.5},
      navigation_world_model::GridLayer::kInflated,
      navigation_world_model::UnknownPolicy::kRequireKnownFree));
  ASSERT_TRUE(fixture.body_support->containsSegment(
      fixture.start_state.position_world, Eigen::Vector3d{0.1, 0.0, 1.5},
      fixture.snapshot->identity(), fixture.snapshot->identity().observation_stamp_ns));

  double ros_time_s = 1.0;
  TestCommitAuthorizer authorizer(fixture.snapshot);
  navigation_planning_backend::PlannerFacade facade(
      PLANNER_FACADE_CONFIG_PATH, fixture.snapshot, std::nullopt,
      authorizer, [&ros_time_s] { return ros_time_s; });
  const auto request = productionRequest(fixture, true);
  ASSERT_TRUE(request.valid());

  // This is the production request transaction: MappingActor snapshot ->
  // A* seed -> corridor -> nominal optimizer -> candidate -> initial
  // admission. Advance the authorization clock only after the solve so the
  // candidate is also checked through the normal temporal boundary.
  const auto outcome = facade.plan(request);
  ASSERT_TRUE(outcome.valid()) << static_cast<int>(outcome.failure_stage)
                              << ":" << static_cast<int>(outcome.failure_reason);
  ASSERT_TRUE(navigation_planning::completePlanningSucceeded(outcome.outcome));
  ASSERT_TRUE(outcome.candidate.has_value());
  ros_time_s = 1.2;
  EXPECT_TRUE(facade.validateCommittedTrajectory(
      fixture.snapshot, ros_time_s).valid);

  // The identical production map and route cannot admit the UNKNOWN measured
  // start without the request-local physical-body witness.
  TestCommitAuthorizer no_support_authorizer(fixture.snapshot);
  navigation_planning_backend::PlannerFacade no_support_facade(
      PLANNER_FACADE_CONFIG_PATH, fixture.snapshot, std::nullopt,
      no_support_authorizer, [&ros_time_s] { return ros_time_s; });
  const auto no_support_outcome = no_support_facade.plan(
      productionRequest(fixture, false));
  EXPECT_FALSE(no_support_outcome.candidate.has_value());
  EXPECT_FALSE(navigation_planning::completePlanningSucceeded(
      no_support_outcome.outcome));
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
