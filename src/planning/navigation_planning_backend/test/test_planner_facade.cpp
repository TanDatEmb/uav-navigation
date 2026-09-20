#include <navigation_planning_backend/planner_facade.hpp>
#include <navigation_mapping/current_body_support.hpp>
#include <navigation_mapping/mapping_actor.hpp>
#include <navigation_mapping/mapping_observation.hpp>
#include <planner_core/planner.hpp>
#include <planner_core/route_yaw_reference.hpp>
#include <planner_core/route_backbone.hpp>
#include <planner_core/planner_result.hpp>
#include <planner_core/backup_braking.hpp>
#include <planner_core/config.hpp>
#include <planner_core/corridor_plane_validation.hpp>
#include <planner_core/trajectory_world_validator.hpp>
#include <data_structure/cmd_traj.h>
#include <traj_opt/trajectory_dynamics.hpp>
#include <navigation_planning/planning_timing.hpp>
#include <navigation_world_model/continuous_clearance.hpp>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <chrono>
#include <filesystem>
#include <limits>
#include <functional>
#include <memory>
#include <optional>
#include <stop_token>
#include <stdexcept>
#include <string>

#include <yaml-cpp/yaml.h>

#include <gtest/gtest.h>

namespace {

class ScopedSnapshotDirectory final {
 public:
  ScopedSnapshotDirectory() {
    char pattern[] = "/tmp/uav-navigation-delayed-anchor-XXXXXX";
    const char* const created = mkdtemp(pattern);
    if (created == nullptr) throw std::runtime_error("mkdtemp failed");
    path_ = created;
    if (path_.parent_path() != "/tmp" ||
        path_.filename().string().rfind("uav-navigation-delayed-anchor-", 0U) != 0U) {
      throw std::runtime_error("invalid temporary snapshot directory");
    }
  }

  ~ScopedSnapshotDirectory() {
    std::error_code error;
    std::filesystem::remove_all(path_, error);
  }

  ScopedSnapshotDirectory(const ScopedSnapshotDirectory&) = delete;
  ScopedSnapshotDirectory& operator=(const ScopedSnapshotDirectory&) = delete;
  const std::filesystem::path& path() const noexcept { return path_; }

 private:
  std::filesystem::path path_;
};

class ScopedSnapshotEnvironment final {
 public:
  ScopedSnapshotEnvironment(const char* name, const std::string& value)
      : name_(name) {
    if (const char* previous = std::getenv(name)) previous_ = previous;
    if (setenv(name_.c_str(), value.c_str(), 1) != 0) {
      throw std::runtime_error("snapshot environment setup failed");
    }
  }

  ~ScopedSnapshotEnvironment() {
    if (previous_) setenv(name_.c_str(), previous_->c_str(), 1);
    else unsetenv(name_.c_str());
  }

  ScopedSnapshotEnvironment(const ScopedSnapshotEnvironment&) = delete;
  ScopedSnapshotEnvironment& operator=(const ScopedSnapshotEnvironment&) = delete;

 private:
  std::string name_;
  std::optional<std::string> previous_;
};

template <typename Value>
Value snapshotScalar(const YAML::Node& node, const std::string& field) {
  try {
    return node.as<Value>();
  } catch (const YAML::Exception& error) {
    throw std::runtime_error("snapshot conversion failed for " + field +
        ": " + error.what() + "; value=" + YAML::Dump(node));
  }
}

class IdentityOnlyWorld : public navigation_world_model::WorldModelView {
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
      const navigation_world_model::Point3& point, navigation_world_model::GridLayer) const noexcept override {
    return (point.array() / 0.2).floor().cast<int>();
  }
  navigation_world_model::Point3 indexToPosition(
      const navigation_world_model::GridIndex3& index, navigation_world_model::GridLayer) const noexcept override {
    return (index.cast<double>().array() + 0.5).matrix() * 0.2;
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

class PlannerBodySupportWorld final : public IdentityOnlyWorld {
 public:
  const navigation_world_model::Point3 measured_start{0.0, 0.0, 2.0};
  bool unknown_after_body_exit{false};
  bool allow_unknown_for_path_search{false};

  navigation_world_model::CellState classify(
      const navigation_world_model::Point3& point,
      navigation_world_model::GridLayer) const noexcept override {
    if (unknown_after_body_exit && point.x() >= 0.3 && point.x() < 4.5) {
      return navigation_world_model::CellState::kUnknown;
    }
    return (point - measured_start).norm() <= 0.05
        ? navigation_world_model::CellState::kUnknown
        : navigation_world_model::CellState::kKnownFree;
  }

  navigation_world_model::GridIndex3 positionToIndex(
      const navigation_world_model::Point3& point,
      navigation_world_model::GridLayer) const noexcept override {
    return (point.array() / 0.2).floor().cast<int>();
  }

  navigation_world_model::Point3 indexToPosition(
      const navigation_world_model::GridIndex3& index,
      navigation_world_model::GridLayer) const noexcept override {
    return (index.cast<double>().array() + 0.5).matrix() * 0.2;
  }

  bool isSegmentTraversable(
      const navigation_world_model::Point3& start,
      const navigation_world_model::Point3& end,
      navigation_world_model::GridLayer layer,
      navigation_world_model::UnknownPolicy policy) const noexcept override {
    if (policy == navigation_world_model::UnknownPolicy::kAllowUnknown) {
      return true;
    }
    for (int sample = 0; sample <= 64; ++sample) {
      const double fraction = static_cast<double>(sample) / 64.0;
      if (classify(start + fraction * (end - start), layer) !=
          navigation_world_model::CellState::kKnownFree) {
        return false;
      }
    }
    return true;
  }

  bool isSegmentTraversableWithCurrentBodySupport(
      const navigation_world_model::Point3& start,
      const navigation_world_model::Point3& end,
      navigation_world_model::GridLayer layer,
      navigation_world_model::UnknownPolicy policy,
      const navigation_world_model::CurrentBodySupportPtr& support) const noexcept override {
    if (allow_unknown_for_path_search) return true;
    if (policy == navigation_world_model::UnknownPolicy::kAllowUnknown ||
        !support || classify(start, layer) != navigation_world_model::CellState::kUnknown ||
        !support->matchesWorldSnapshot(identity(), support->source_stamp_ns) ||
        !support->contains(start, identity(), support->source_stamp_ns)) {
      return isSegmentTraversable(start, end, layer, policy);
    }
    const double prefix = support->contiguousBodyPrefixFraction(start, end);
    if (!std::isfinite(prefix) || prefix <= 0.0) return false;
    for (int sample = 0; sample <= 64; ++sample) {
      const double fraction = static_cast<double>(sample) / 64.0;
      const auto state = classify(start + fraction * (end - start), layer);
      if (state == navigation_world_model::CellState::kOccupied ||
          state == navigation_world_model::CellState::kOutOfMap ||
          state == navigation_world_model::CellState::kUndefined ||
          (state == navigation_world_model::CellState::kUnknown &&
           fraction > prefix + 1.0e-9)) {
        return false;
      }
    }
    return true;
  }
};

TEST(WorldGeometryBoundaries, ContinuousClearanceRejectsOverflowingDerivedQueryBox) {
  const auto world = std::make_shared<IdentityOnlyWorld>();
  const auto limit = std::numeric_limits<double>::max();
  EXPECT_FALSE(navigation_world_model::observedOccupiedTubeIsClear(
      *world, Eigen::Vector3d{limit, 0.0, 0.0},
      Eigen::Vector3d{-limit, 0.0, 0.0}, limit));
}

TEST(WorldGeometryBoundaries, SyntheticWorldPreservesItsDeclaredVoxelCoordinates) {
  const IdentityOnlyWorld world;
  const Eigen::Vector3d point{10.0, -0.3, 3.0};
  const auto layer = navigation_world_model::GridLayer::kInflated;
  const auto index = world.positionToIndex(point, layer);
  const auto center = world.indexToPosition(index, layer);
  EXPECT_EQ(world.positionToIndex(center, layer), index);
  EXPECT_LE((center - point).cwiseAbs().maxCoeff(), 0.1 + 1.0e-14);
  EXPECT_NEAR(center.z(), 3.1, 1.0e-14);
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
  // Keep the ray on a voxel-centre-aligned y coordinate and end the free-space
  // witness beyond the mission goal: the raycaster excludes its endpoint cell,
  // while the goal at x=1.0 must still be known-free.
  for (std::int64_t iteration = 1; iteration <= 20; ++iteration) {
    const auto stamp_ns = iteration * 1'000'000'000LL;
    auto cloud = std::make_unique<navigation_mapping::PointCloud>();
    cloud->push_back(navigation_mapping::PointXYZI{4.0F, 0.1F, 1.5F, 0.0F});
    auto free_endpoints = std::make_unique<navigation_mapping::PointCloud>();
    free_endpoints->push_back(
        navigation_mapping::PointXYZI{1.2F, 0.1F, 1.5F, 0.0F});
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
    observation.sensor_origin_world = Eigen::Vector3d{0.1, 0.1, 1.5};
    observation.sensor_origin_localization_epoch = 1U;
    observation.sensor_origin_stamp_ns = stamp_ns;
    const auto update = fixture.actor->process(observation);
    fixture.snapshot = update.snapshot;
  }
  if (!fixture.snapshot) return fixture;

  const auto support = navigation_mapping::makeX500Mid360CurrentBodySupport(
      Eigen::Vector3d{0.1, 0.1, 1.5}, Eigen::Quaterniond::Identity(),
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
          "start", Eigen::Vector3d{0.1, 0.1, 1.5}, 0.2, 0.0,
          navigation_mission::MissionWaypoint::Behavior::PassThrough},
      navigation_mission::MissionWaypoint{
          "goal", Eigen::Vector3d{1.0, 0.1, 1.5}, 0.3, 0.0,
          navigation_mission::MissionWaypoint::Behavior::Stop},
  };
  navigation_mission::RouteProgress progress(mission);
  if (!progress.update(Eigen::Vector3d{0.1, 0.1, 1.5}).valid) return fixture;
  fixture.route = progress.snapshot(mission.id, mission.frame, 1U, 17U, 1U);

  fixture.start_state.position_world = Eigen::Vector3d{0.1, 0.1, 1.5};
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

navigation_planning::PlanningRequest plannerBodySupportRequest(
    const navigation_world_model::WorldModelViewPtr& world,
    const navigation_world_model::CurrentBodySupportPtr& body_support,
    const Eigen::Vector3d& goal = Eigen::Vector3d{5.0, 0.0, 2.0}) {
  constexpr std::int64_t kStampNs = 100;
  constexpr std::uint64_t kRequestId = 31U;
  const Eigen::Vector3d start{0.0, 0.0, 2.0};

  navigation_mission::Mission mission;
  mission.id = "current-body-planner-level";
  mission.frame = "lio_odom";
  mission.planning.requested_cruise_speed_mps = 1.0;
  mission.waypoints = {
      navigation_mission::MissionWaypoint{
          "start", start, 0.2, 0.0,
          navigation_mission::MissionWaypoint::Behavior::PassThrough},
      navigation_mission::MissionWaypoint{
          "goal", goal, 0.3, 0.0,
          navigation_mission::MissionWaypoint::Behavior::Stop}};
  navigation_mission::RouteProgress progress(mission);
  EXPECT_TRUE(progress.update(start).valid);
  const auto route = progress.snapshot(
      mission.id, mission.frame, 1U, kRequestId, 1U);

  navigation_planning::KinematicState state;
  state.position_world = start;
  state.orientation_world_body = Eigen::Quaterniond::Identity();
  state.source_stamp_ns = kStampNs;
  state.receive_stamp_ns = kStampNs;
  state.localization_epoch = 1U;
  state.world_frame_id = "lio_odom";
  state.body_frame_id = "base_link";

  navigation_planning::PlanningRequest request;
  const auto identity = world->identity();
  request.key.localization_epoch = 1U;
  request.key.goal_epoch = 1U;
  request.key.request_id = kRequestId;
  request.key.route_revision = route.route_revision;
  request.key.pinned_world_generation = identity.generation;
  request.key.pinned_world_revision = identity.revision;
  request.key.start_mode = navigation_planning::PlanningStartMode::kStoppedMeasuredState;
  request.key.anchor_stamp_ns = kStampNs;
  request.key.dynamics_hash = 1U;
  request.goal = navigation_planning::GoalIdentity{
      1U, 1U, mission.id, 1U, kRequestId};
  request.start_state = state;
  request.route_snapshot = route;
  request.world = world;
  request.current_body_support = body_support;
  request.dynamics.intent.requested_cruise_speed_mps = 1.0;
  request.dynamics.unknown_space_policy =
      navigation_world_model::UnknownPolicy::kRequireKnownFree;
  request.budget.deadline = navigation_planning::PlanningBudget::Clock::now() +
      std::chrono::seconds(10);
  return request;
}

navigation_planning::PlanningRequest plannerRequestForRoute(
    const navigation_world_model::WorldModelViewPtr& world,
    const navigation_mission::ImmutableRouteSnapshot& route,
    const navigation_planning::KinematicState& state,
    const navigation_planning::DynamicLimits& dynamics,
    const std::uint64_t goal_epoch,
    const std::optional<Eigen::Vector3d>& mission_start = std::nullopt) {
  const auto& waypoint = route.waypoints.at(route.active_waypoint_index);
  auto request = plannerBodySupportRequest(world, nullptr, waypoint.position_enu);
  const auto identity = world->identity();
  request.key.localization_epoch = state.localization_epoch;
  request.key.goal_epoch = goal_epoch;
  request.key.request_id = route.request_id;
  request.key.route_revision = route.route_revision;
  request.key.pinned_world_generation = identity.generation;
  request.key.pinned_world_revision = identity.revision;
  request.key.start_mode = navigation_planning::PlanningStartMode::kStoppedMeasuredState;
  request.key.anchor_stamp_ns = state.source_stamp_ns;
  request.goal = {state.localization_epoch, goal_epoch, route.mission_id,
                  static_cast<std::uint32_t>(route.active_waypoint_index),
                  route.request_id};
  request.start_state = state;
  request.route_snapshot = route;
  request.world = world;
  request.current_body_support.reset();
  request.mission_start_position_world = mission_start;
  request.goal_acceptance_radius_m = waypoint.acceptance_radius_m;
  request.dynamics = dynamics;
  return request;
}

navigation_world_model::CurrentBodySupportPtr plannerBodySupport(
    const navigation_world_model::WorldSnapshotIdentity& identity) {
  navigation_world_model::CurrentBodySupport support;
  support.snapshot_identity = identity;
  support.body_position = Eigen::Vector3d{0.0, 0.0, 2.0};
  support.body_orientation = Eigen::Quaterniond::Identity();
  support.localization_epoch = 1U;
  support.source_stamp_ns = 100;
  support.world_frame_id = "lio_odom";
  support.body_frame_id = "base_link";
  support.geometry_provenance =
      "repo:test-model@sha256=0123456789abcdef;"
      "component=base_link_collision_0_main_obb_only";
  support.body_box = {
      Eigen::Vector3d::Zero(), Eigen::Vector3d{0.3, 0.3, 0.3},
      Eigen::Quaterniond::Identity()};
  support.valid = true;
  return std::make_shared<const navigation_world_model::CurrentBodySupport>(support);
}

// These tests exercise request/route/body-support and boundary-event
// semantics, not the product MAIN-envelope stress profile. Pin their
// requested speed to the pre-stress nominal value so a production envelope
// change cannot turn a semantic fixture into a numerical-feasibility test.
std::optional<navigation_planning::DynamicLimits> semanticFixtureMissionLimits() {
  navigation_planning::DynamicLimits limits;
  limits.intent.requested_cruise_speed_mps = 0.5624988750005627;
  return limits;
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
  EXPECT_DOUBLE_EQ(diagnostics.yaw_rate_limit_rad_s, 2.0);
  EXPECT_DOUBLE_EQ(diagnostics.yaw_acceleration_limit_rad_s2, 2.0);
}

TEST(PlannerFacade, WorldRevalidationPreservesOffCentreBlockingCellAfterActivation) {
  class OffCentreBlockedWorld final : public IdentityOnlyWorld {
   public:
    const Eigen::Vector3d blocker{0.1, -0.1, 2.1};

    navigation_world_model::WorldSnapshotIdentity identity() const noexcept override {
      return {1U, 1U, 2U, 200};
    }

    navigation_world_model::CellState classify(
        const navigation_world_model::Point3& point,
        navigation_world_model::GridLayer layer) const noexcept override {
      return positionToIndex(point, layer) == positionToIndex(blocker, layer)
          ? navigation_world_model::CellState::kOccupied
          : navigation_world_model::CellState::kKnownFree;
    }
  };

  auto original_world = std::make_shared<IdentityOnlyWorld>();
  TestCommitAuthorizer authorizer(original_world);
  navigation_planning_backend::PlannerFacade facade(
      PLANNER_FACADE_CONFIG_PATH, original_world, semanticFixtureMissionLimits(),
      authorizer, [] { return 10.0; });
  const auto outcome = facade.plan(plannerBodySupportRequest(original_world, {}));
  ASSERT_TRUE(outcome.candidate.has_value());
  ASSERT_TRUE(outcome.candidate->valid());
  const auto generation = outcome.candidate->bundle_generation;
  const auto blocked_world = std::make_shared<OffCentreBlockedWorld>();

  // The initial centreline is known-free, but its certificate tube touches
  // an off-centre occupied voxel. Recertification must report that voxel,
  // not a known-free polynomial sample substituted by the facade.
  EXPECT_EQ(blocked_world->classify(Eigen::Vector3d{0.0, 0.0, 2.0},
                                  navigation_world_model::GridLayer::kInflated),
            navigation_world_model::CellState::kKnownFree);
  const auto staged = facade.validateStagedCommandCandidate(blocked_world, 10.0, generation);
  ASSERT_FALSE(staged.valid);
  ASSERT_EQ(staged.failure_code,
            static_cast<int>(navigation_planning_backend::SweptValidationResult::Failure::
                                 kCertificateTubeBlocked));
  EXPECT_TRUE(staged.first_blocked_position.isApprox(blocked_world->blocker, 1.0e-12));
  EXPECT_TRUE(staged.blocking_cell_observed);
  EXPECT_EQ(staged.tube_failure_code,
            static_cast<int>(navigation_planning_backend::CertificateTubeFailure::
                                 kNonTraversableCell));
  EXPECT_EQ(staged.evaluated_generation, generation);
  EXPECT_EQ(staged.first_blocked_cell_state,
            static_cast<int>(navigation_world_model::CellState::kOccupied));

  facade.onExecutionTimelineActivated(generation);
  ASSERT_EQ(facade.committedSnapshot().generation, generation);
  const auto committed = facade.validateCommittedTrajectory(
      *outcome.candidate, blocked_world, 10.0);
  ASSERT_FALSE(committed.valid);
  EXPECT_EQ(committed.failure_code, staged.failure_code);
  EXPECT_TRUE(committed.first_blocked_position.isApprox(blocked_world->blocker, 1.0e-12));
  EXPECT_TRUE(committed.blocking_cell_observed);
  EXPECT_EQ(committed.tube_failure_code, staged.tube_failure_code);
  EXPECT_EQ(committed.evaluated_generation, generation);
  EXPECT_EQ(committed.evaluated_unknown_policy, staged.evaluated_unknown_policy);
  EXPECT_DOUBLE_EQ(committed.unsafe_interval_end_time_s, staged.unsafe_interval_end_time_s);
  EXPECT_DOUBLE_EQ(committed.curve_deviation_bound_m, staged.curve_deviation_bound_m);
  EXPECT_DOUBLE_EQ(committed.curve_deviation_tolerance_m, staged.curve_deviation_tolerance_m);
  EXPECT_EQ(committed.first_blocked_cell_state, staged.first_blocked_cell_state);
  EXPECT_TRUE(navigation_world_model::sameWorldSnapshotIdentity(
      committed.validated_world, blocked_world->identity()));
  auto wrong_generation_bundle = *outcome.candidate;
  ++wrong_generation_bundle.bundle_generation;
  const auto wrong_generation = facade.validateCommittedTrajectory(
      wrong_generation_bundle, blocked_world, 10.0);
  EXPECT_FALSE(wrong_generation.valid);
  EXPECT_EQ(wrong_generation.evaluated_generation, 0U);
  EXPECT_FALSE(wrong_generation.blocking_cell_observed);
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
      fixture.start_state.position_world, Eigen::Vector3d{0.11, 0.1, 1.5},
      fixture.snapshot->identity(), fixture.snapshot->identity().observation_stamp_ns));
  ASSERT_NEAR(fixture.body_support->contiguousBodyPrefixFraction(
      fixture.start_state.position_world, Eigen::Vector3d{0.11, 0.1, 1.5}),
      1.0, 1.0e-12);
  EXPECT_EQ(fixture.snapshot->classify(
                Eigen::Vector3d{0.1, 0.1, 1.5},
                navigation_world_model::GridLayer::kEvidence),
            navigation_world_model::CellState::kUnknown);
  ASSERT_TRUE(fixture.snapshot->isSegmentTraversableWithCurrentBodySupport(
      fixture.start_state.position_world, Eigen::Vector3d{0.11, 0.1, 1.5},
      navigation_world_model::GridLayer::kEvidence,
      navigation_world_model::UnknownPolicy::kRequireKnownFree,
      fixture.body_support));
  ASSERT_TRUE(fixture.snapshot->isSegmentTraversable(
      Eigen::Vector3d{0.5, 0.1, 1.5}, Eigen::Vector3d{0.5, 0.1, 1.5},
      navigation_world_model::GridLayer::kEvidence,
      navigation_world_model::UnknownPolicy::kRequireKnownFree));
  EXPECT_EQ(fixture.snapshot->classify(
                Eigen::Vector3d{0.5, 0.1, 1.5},
                navigation_world_model::GridLayer::kInflated),
            navigation_world_model::CellState::kKnownFree);
  ASSERT_TRUE(fixture.snapshot->isSegmentTraversable(
      Eigen::Vector3d{0.5, 0.1, 1.5}, Eigen::Vector3d{0.5, 0.1, 1.5},
      navigation_world_model::GridLayer::kInflated,
      navigation_world_model::UnknownPolicy::kRequireKnownFree));
  ASSERT_TRUE(fixture.snapshot->isSegmentTraversable(
      Eigen::Vector3d{0.35, 0.1, 1.5}, Eigen::Vector3d{1.0, 0.1, 1.5},
      navigation_world_model::GridLayer::kInflated,
      navigation_world_model::UnknownPolicy::kRequireKnownFree));
  ASSERT_TRUE(fixture.body_support->containsSegment(
      fixture.start_state.position_world, Eigen::Vector3d{0.1, 0.1, 1.5},
      fixture.snapshot->identity(), fixture.snapshot->identity().observation_stamp_ns));

  double ros_time_s = 20.0;
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
  // plan() leaves the candidate staged until execution accepts its generation.
  // Validate after activation and after the trajectory has entered the
  // sensor-known-free corridor; committed-future validation receives no body
  // exception.
  facade.onExecutionTimelineActivated(outcome.candidate->bundle_generation);
  const auto committed = facade.committedSnapshot();
  ASSERT_FALSE(committed.empty());
  navigation_planning::TrajectoryPoint terminal;
  ASSERT_TRUE(committed.position.sample(committed.position.duration_s, terminal));
  ASSERT_EQ(fixture.snapshot->classify(
                terminal.position_world,
                navigation_world_model::GridLayer::kInflated),
            navigation_world_model::CellState::kKnownFree);
  ros_time_s = committed.position.start_wall_time_s +
      committed.position.duration_s;
  EXPECT_TRUE(facade.validateCommittedTrajectory(
      *outcome.candidate, fixture.snapshot, ros_time_s).valid);

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

TEST(PlannerFacade, CurrentBodySupportCrossesPlannerLayersAndIsRequestLocal) {
  auto world = std::make_shared<PlannerBodySupportWorld>();
  const auto support = plannerBodySupport(world->identity());
  ASSERT_TRUE(support);
  ASSERT_TRUE(support->matchesMeasuredState(
      world->measured_start, Eigen::Quaterniond::Identity(), 1U, 100,
      "lio_odom", "base_link"));

  TestCommitAuthorizer authorizer(world);
  double ros_time_s = 10.0;
  navigation_planning_backend::PlannerFacade facade(
      PLANNER_FACADE_CONFIG_PATH, world, semanticFixtureMissionLimits(), authorizer,
      [&ros_time_s] { return ros_time_s; });

  // The synthetic world has UNKNOWN only at the measured pose and KNOWN_FREE
  // immediately beyond it. The production request must therefore exercise
  // A*'s measured-start admission, corridor generation, nominal trajectory
  // construction, and final executable-candidate validation in one solve.
  auto request = plannerBodySupportRequest(world, support);
  ASSERT_TRUE(request.valid());
  const auto with_support = facade.plan(request);
  ASSERT_TRUE(with_support.valid())
      << static_cast<int>(with_support.failure_stage) << ":"
      << static_cast<int>(with_support.failure_reason);
  ASSERT_TRUE(with_support.candidate.has_value());
  EXPECT_TRUE(navigation_planning::completePlanningSucceeded(
      with_support.outcome));
  EXPECT_TRUE(with_support.candidate->valid());

  // The same facade receives a second stopped-state request after the first
  // transaction. Removing the witness must make the UNKNOWN measured start
  // fail closed; a stale mutable member must not authorize request B.
  request.current_body_support.reset();
  const auto without_support = facade.plan(request);
  EXPECT_FALSE(without_support.candidate.has_value());
  EXPECT_FALSE(navigation_planning::completePlanningSucceeded(
      without_support.outcome));

  // A second full planner transaction has UNKNOWN spanning the route. The
  // synthetic support keeps A* and corridor construction admissible, but it
  // does not contribute known-free distance; MAIN must therefore reject the
  // route at its braking-evidence gate rather than reach commit recertification.
  auto blocked_world = std::make_shared<PlannerBodySupportWorld>();
  blocked_world->unknown_after_body_exit = true;
  blocked_world->allow_unknown_for_path_search = true;
  // Keep the path-search and corridor layers admissible through the synthetic
  // UNKNOWN region, while making the strict known-free support calculation
  // observe zero post-body evidence.  The deliberately oversized x extent is
  // test-only; it prevents the A* invariant from becoming the failure under
  // test before MAIN's braking-evidence gate runs.
  const auto blocked_support_seed = plannerBodySupport(blocked_world->identity());
  auto blocked_support_value =
      std::make_shared<navigation_world_model::CurrentBodySupport>(
          *blocked_support_seed);
  blocked_support_value->body_box.half_extent.x() = 5.0;
  const navigation_world_model::CurrentBodySupportPtr blocked_support =
      blocked_support_value;
  TestCommitAuthorizer blocked_authorizer(blocked_world);
  navigation_planning_backend::PlannerFacade blocked_facade(
      PLANNER_FACADE_CONFIG_PATH, blocked_world, semanticFixtureMissionLimits(),
      blocked_authorizer, [] { return 10.0; });
  const auto blocked = blocked_facade.plan(
      plannerBodySupportRequest(blocked_world, blocked_support));
  EXPECT_FALSE(blocked.candidate.has_value());
  EXPECT_FALSE(navigation_planning::completePlanningSucceeded(blocked.outcome));
  EXPECT_EQ(blocked_facade.diagnostics().replan_return_code,
            navigation_planning_backend::PLANNER_MAIN_KNOWN_FREE_INSUFFICIENT);
  EXPECT_EQ(blocked.failure_stage,
            navigation_planning::PlanningFailureStage::kNominalSeed);
  EXPECT_EQ(blocked.failure_reason,
            navigation_planning::PlanningFailureReason::kMainKnownFreeInsufficient);
}

TEST(PlannerFacade, RejectsInvalidCommittedFutureRequestBeforeSolve) {
  auto world = std::make_shared<IdentityOnlyWorld>();
  TestCommitAuthorizer authorizer(world);
  navigation_planning_backend::PlannerFacade facade(
      PLANNER_FACADE_CONFIG_PATH, world, std::nullopt, authorizer,
      [] { return 10.0; });

  auto request = plannerBodySupportRequest(world, nullptr);
  request.key.start_mode =
      navigation_planning::PlanningStartMode::kCommittedFutureState;
  request.key.committed_bundle_generation = 11U;
  request.current_body_support.reset();
  request.anchor.reset();
  request.activation_stamp_ns = 0;

  EXPECT_FALSE(request.startModeContractValid());
  EXPECT_FALSE(request.valid());
  const auto outcome = facade.plan(request);
  EXPECT_EQ(outcome.outcome,
            navigation_planning::CompletePlanningOutcome::kInvalidRequest);
  EXPECT_EQ(outcome.failure_stage,
            navigation_planning::PlanningFailureStage::kInput);
  EXPECT_EQ(outcome.failure_reason,
            navigation_planning::PlanningFailureReason::kInvalidInput);
  EXPECT_EQ(facade.solveStage(), 0);
  EXPECT_FALSE(facade.hasStagedCommandCandidate());
  const auto timeline = facade.diagnostics().timeline;
  EXPECT_GT(timeline.request_received_steady_ns, 0);
  EXPECT_EQ(timeline.solve_started_steady_ns,
            timeline.request_received_steady_ns);
  EXPECT_GT(timeline.solve_finished_steady_ns,
            timeline.solve_started_steady_ns);
  EXPECT_EQ(timeline.hard_deadline_steady_ns, 0);
  EXPECT_EQ(timeline.remaining_hard_budget_us_at_finish, -1);
}

TEST(PlannerFacade, ExpiredPlanReportsLatestTimeoutAtProductBoundary) {
  auto world = std::make_shared<IdentityOnlyWorld>();
  TestCommitAuthorizer authorizer(world);
  navigation_planning_backend::PlannerFacade facade(
      PLANNER_FACADE_CONFIG_PATH, world, std::nullopt, authorizer,
      [] { return 10.0; });
  auto request = plannerBodySupportRequest(world, nullptr);
  request.budget.deadline = navigation_planning::PlanningBudget::Clock::now() +
      std::chrono::seconds(1);
  request.budget.steady_deadline_ns =
      std::chrono::duration_cast<std::chrono::nanoseconds>(
          navigation_planning::PlanningBudget::Clock::now().time_since_epoch() +
          std::chrono::nanoseconds(1)).count();

  const auto outcome = facade.plan(request);
  EXPECT_EQ(facade.diagnostics().replan_return_code,
            navigation_planning_backend::PLANNER_SOLVE_TIMEOUT);
  EXPECT_EQ(outcome.failure_stage,
            navigation_planning::PlanningFailureStage::kDeadline);
  EXPECT_EQ(outcome.failure_reason,
            navigation_planning::PlanningFailureReason::kNoCompleteBundleAtDeadline);
  const auto timeline = facade.diagnostics().timeline;
  EXPECT_EQ(timeline.hard_deadline_steady_ns, request.budget.steady_deadline_ns);
  EXPECT_GT(timeline.solve_finished_steady_ns,
            timeline.solve_started_steady_ns);
  EXPECT_LE(timeline.remaining_hard_budget_us_at_finish, 0);
}

class InterruptBackupWorld final : public IdentityOnlyWorld {
 public:
  std::function<int()> current_stage;
  std::function<void()> interrupt;
  mutable std::size_t queries_after_interrupt{0U};
  mutable bool interrupted{false};

  navigation_world_model::PointVector observedOccupiedPoints(
      const navigation_world_model::AxisAlignedBox& box) const override {
    if (interrupt && current_stage && current_stage() == 5) {
      ++queries_after_interrupt;
      if (!interrupted) {
        interrupted = true;
        interrupt();
      }
    }
    return IdentityOnlyWorld::observedOccupiedPoints(box);
  }
};

void expectBackupInterruptRecordsFailure(const bool cancel,
                                        const bool install_active = false,
                                        const bool expect_bounded = false) {
  auto world = std::make_shared<InterruptBackupWorld>();
  TestCommitAuthorizer authorizer(world);
  double ros_time_s = 10.0;
  const auto limits = semanticFixtureMissionLimits();
  navigation_planning_backend::PlannerFacade facade(
      PLANNER_FACADE_CONFIG_PATH, world, limits, authorizer,
      [&ros_time_s] { return ros_time_s; });

  navigation_mission::Mission mission;
  mission.id = "backup-interruption-contract";
  mission.frame = "lio_odom";
  mission.planning.requested_cruise_speed_mps =
      limits->intent.requested_cruise_speed_mps;
  navigation_mission::MissionWaypoint active;
  active.id = "active";
  active.position_enu = Eigen::Vector3d{10.0, 0.0, 3.0};
  active.behavior = navigation_mission::MissionWaypoint::Behavior::PassThrough;
  active.acceptance_radius_m = 0.5;
  auto next = active;
  next.id = "next";
  next.position_enu.x() = 20.0;
  next.behavior = navigation_mission::MissionWaypoint::Behavior::Stop;
  mission.waypoints = {active, next};
  navigation_mission::RouteProgress progress(mission);
  ASSERT_TRUE(progress.update(Eigen::Vector3d{0.0, 0.0, 3.0}).valid);
  const auto route = progress.snapshot(mission.id, mission.frame, 1U, 1U, 0U);
  ASSERT_TRUE(route.valid());
  navigation_planning::KinematicState state;
  state.position_world = Eigen::Vector3d{0.0, 0.0, 3.0};
  state.source_stamp_ns = 10'000'000'000LL;
  state.receive_stamp_ns = state.source_stamp_ns;
  state.localization_epoch = 1U;
  state.world_frame_id = "lio_odom";
  state.body_frame_id = "base_link";
  auto request = plannerBodySupportRequest(world, nullptr, active.position_enu);
  request.key.request_id = route.request_id;
  request.key.route_revision = route.route_revision;
  request.key.anchor_stamp_ns = state.source_stamp_ns;
  request.goal = {state.localization_epoch, 1U, route.mission_id,
                  static_cast<std::uint32_t>(route.active_waypoint_index),
                  route.request_id};
  request.start_state = state;
  request.route_snapshot = route;
  request.dynamics = *limits;
  request.goal_acceptance_radius_m = active.acceptance_radius_m;
  std::stop_source cancellation;
  if (cancel) request.budget.cancellation = cancellation.get_token();
  ASSERT_TRUE(request.valid());

  if (install_active) {
    const auto initial_outcome = facade.plan(request);
    ASSERT_TRUE(initial_outcome.valid());
    const auto initial = initial_outcome.candidate;
    ASSERT_TRUE(initial);
    ASSERT_TRUE(initial->valid());
    ASSERT_TRUE(initial->backup_available);
    facade.onExecutionTimelineActivated(initial->bundle_generation);
  }
  const auto previous = facade.committedSnapshot();
  if (install_active) ASSERT_FALSE(previous.empty());
  else ASSERT_EQ(previous.generation, 0U);

  // Interrupt only when the actual BACKUP corridor queries its world. MAIN
  // therefore completes first, and the source clock is not globally advanced
  // before reaching the phase under test. No sleep or wall-time threshold.
  world->current_stage = [&facade] { return facade.solveStage(); };
  world->interrupt = [&] {
    if (cancel) cancellation.request_stop();
    else ros_time_s = 1000.0;
  };
  const auto failed_outcome = facade.plan(request);
  EXPECT_FALSE(failed_outcome.candidate.has_value());
  EXPECT_FALSE(navigation_planning::completePlanningSucceeded(
      failed_outcome.outcome));
  ASSERT_TRUE(world->interrupted);
  const auto diagnostics = facade.diagnostics();
  EXPECT_TRUE(diagnostics.backup_certificate.attempted);
  EXPECT_GT(diagnostics.module_time_us[2], 0.0);
  EXPECT_FALSE(diagnostics.backup_certificate.selected);
  if (expect_bounded) {
    EXPECT_LE(diagnostics.backup_certificate.switch_candidate_count, 1U);
    EXPECT_LE(world->queries_after_interrupt, 1U);
  }
  EXPECT_EQ(diagnostics.replan_return_code,
            cancel ? navigation_planning_backend::PLANNER_SOLVE_CANCELLED
                   : navigation_planning_backend::PLANNER_SOLVE_TIMEOUT);
  EXPECT_FALSE(facade.hasStagedCommandCandidate());
  EXPECT_EQ(facade.committedGeneration(), previous.generation);
  const auto after = facade.committedSnapshot();
  EXPECT_EQ(after.generation, previous.generation);
  EXPECT_EQ(after.position.duration_s, previous.position.duration_s);
  if (install_active) {
    for (const double time_s : {0.0, previous.position.duration_s * 0.5,
                               previous.position.duration_s}) {
      navigation_planning::TrajectoryPoint old_point;
      navigation_planning::TrajectoryPoint retained_point;
      ASSERT_TRUE(previous.position.sample(time_s, old_point));
      ASSERT_TRUE(after.position.sample(time_s, retained_point));
      EXPECT_TRUE(old_point.position_world.isApprox(retained_point.position_world));
      EXPECT_TRUE(old_point.velocity_world.isApprox(retained_point.velocity_world));
      EXPECT_TRUE(old_point.acceleration_world.isApprox(retained_point.acceleration_world));
      EXPECT_TRUE(old_point.jerk_world.isApprox(retained_point.jerk_world));
    }
  }

  if (cancel) {
    // Cancellation is scoped to this solve. A new request with a distinct
    // route/goal identity must not inherit the old token or its solve inputs.
    const auto retry_route = progress.snapshot(
        mission.id, mission.frame, 1U, 2U, 0U);
    ASSERT_TRUE(retry_route.valid());
    auto retry = plannerRequestForRoute(
        world, retry_route, state, *limits, 2U);
    ASSERT_TRUE(retry.valid());
    const auto retried = facade.plan(retry);
    ASSERT_TRUE(retried.valid())
        << static_cast<int>(retried.failure_stage) << ":"
        << static_cast<int>(retried.failure_reason);
    ASSERT_TRUE(retried.candidate)
        << static_cast<int>(retried.failure_stage) << ":"
        << static_cast<int>(retried.failure_reason);
    EXPECT_EQ(retried.candidate->request_id, retry.key.request_id);
    EXPECT_EQ(retried.candidate->goal_epoch, retry.key.goal_epoch);
    EXPECT_EQ(retried.candidate->localization_epoch,
              retry.key.localization_epoch);
    EXPECT_EQ(retried.candidate->world_identity.revision,
              retry.key.pinned_world_revision);
  }
}

TEST(PlannerFacade, BackupInterruptedFirstAttemptAccountsFrontend) {
  expectBackupInterruptRecordsFailure(false);
  expectBackupInterruptRecordsFailure(true);
}

TEST(PlannerFacade, BackupSourceExpiryStopsEnumerationAndPreservesActive) {
  expectBackupInterruptRecordsFailure(false, true, true);
}

TEST(PlannerFacade, BackupCancellationStopsEnumerationAndPreservesActive) {
  expectBackupInterruptRecordsFailure(true, true, true);
}

TEST(PlannerFacade, ExportsCommittedFutureCandidateAtRequestedActivation) {
  auto world = std::make_shared<IdentityOnlyWorld>();
  TestCommitAuthorizer authorizer(world);
  double ros_time_s = 10.0;
  navigation_planning_backend::PlannerFacade facade(
      PLANNER_FACADE_CONFIG_PATH, world, std::nullopt, authorizer,
      [&ros_time_s] { return ros_time_s; });

  auto initial_request = plannerBodySupportRequest(world, nullptr);
  // The legacy planner call used the configured nominal cruise when no
  // mission-specific limits were supplied. Carry that same intent explicitly.
  initial_request.dynamics.intent.requested_cruise_speed_mps = 5.0;
  initial_request.key.anchor_stamp_ns = 10'000'000'000LL;
  initial_request.start_state.source_stamp_ns = initial_request.key.anchor_stamp_ns;
  initial_request.start_state.receive_stamp_ns = initial_request.key.anchor_stamp_ns;
  ASSERT_TRUE(initial_request.valid());
  const auto initial = facade.plan(initial_request);
  ASSERT_TRUE(initial.valid())
      << static_cast<int>(initial.failure_stage) << ":"
      << static_cast<int>(initial.failure_reason);
  ASSERT_TRUE(initial.candidate.has_value());
  ASSERT_TRUE(initial.candidate->valid());
  facade.onExecutionTimelineActivated(initial.candidate->bundle_generation);

  const auto& committed = *initial.candidate;
  const double main_end_time_s = committed.backup_available
      ? committed.backup_start_time_s
      : committed.duration_s;
  ASSERT_TRUE(std::isfinite(main_end_time_s));
  ASSERT_GT(main_end_time_s, 0.0);
  const double activation_offset_s = std::min(0.1, main_end_time_s * 0.5);
  ASSERT_GT(activation_offset_s, 0.0);
  const auto activation_stamp_ns = committed.declared_start_ns +
      static_cast<std::int64_t>(std::llround(activation_offset_s * 1.0e9));
  ASSERT_GT(activation_stamp_ns, initial_request.key.anchor_stamp_ns);
  ASSERT_LT(activation_stamp_ns,
            committed.declared_start_ns +
                static_cast<std::int64_t>(std::llround(main_end_time_s * 1.0e9)));

  const auto anchor_sample = committed.sampleAtDeclaredStamp(activation_stamp_ns);
  ASSERT_TRUE(anchor_sample.has_value());

  auto successor_request = initial_request;
  successor_request.key.start_mode =
      navigation_planning::PlanningStartMode::kCommittedFutureState;
  successor_request.key.committed_bundle_generation =
      committed.bundle_generation;
  successor_request.current_body_support.reset();
  successor_request.activation_stamp_ns = activation_stamp_ns;
  navigation_planning::ExecutionAnchor anchor;
  anchor.active_bundle_generation = committed.bundle_generation;
  anchor.execution_lineage_version = 1U;
  anchor.localization_epoch = successor_request.key.localization_epoch;
  anchor.goal_epoch = successor_request.key.goal_epoch;
  anchor.request_id = successor_request.key.request_id;
  anchor.request_stamp_ns = successor_request.key.anchor_stamp_ns;
  anchor.activation_stamp_ns = activation_stamp_ns;
  anchor.state = *anchor_sample;
  anchor.active_role = anchor_sample->role;
  anchor.active_main_end_ns = committed.declared_start_ns +
      static_cast<std::int64_t>(std::llround(main_end_time_s * 1.0e9));
  anchor.active_bundle_end_ns = committed.declared_end_ns;
  anchor.command_world = committed.world_identity;
  successor_request.anchor = anchor;
  successor_request.history.previous_bundle_generation =
      committed.bundle_generation;
  successor_request.history.previous_bundle =
      std::make_shared<const navigation_planning::CandidateBundle>(committed);
  successor_request.history.previous_velocity_world =
      anchor_sample->velocity_world;

  ASSERT_TRUE(successor_request.startModeContractValid());
  ASSERT_TRUE(successor_request.valid());
  auto stale_history = successor_request;
  ++stale_history.history.previous_bundle_generation;
  EXPECT_FALSE(stale_history.predecessorContractValid());
  const auto successor = facade.plan(successor_request);
  ASSERT_TRUE(successor.valid())
      << static_cast<int>(successor.failure_stage) << ":"
      << static_cast<int>(successor.failure_reason);
  ASSERT_TRUE(successor.candidate.has_value());
  ASSERT_TRUE(successor.candidate->valid());
  EXPECT_EQ(successor.candidate->localization_epoch,
            successor_request.key.localization_epoch);
  EXPECT_EQ(successor.candidate->goal_epoch, successor_request.key.goal_epoch);
  EXPECT_EQ(successor.candidate->request_id, successor_request.key.request_id);
  EXPECT_EQ(successor.candidate->activation_stamp_ns,
            successor_request.activation_stamp_ns);
  EXPECT_EQ(successor.candidate->valid_from_ns,
            successor_request.activation_stamp_ns);
  EXPECT_NE(successor.candidate->activation_stamp_ns,
            successor_request.key.anchor_stamp_ns);
}

void expectRequestOwnedGuideOrigin(const std::int64_t backend_delay_ns,
                                  const bool activation_expired = false) {
  const ScopedSnapshotDirectory directory;
  const ScopedSnapshotEnvironment capture(
      "UAV_NAVIGATION_NOMINAL_SNAPSHOT_DIR", directory.path().string());
  const ScopedSnapshotEnvironment failure_only(
      "UAV_NAVIGATION_NOMINAL_SNAPSHOT_FAILURE_ONLY", "0");
  const ScopedSnapshotEnvironment include_world(
      "UAV_NAVIGATION_NOMINAL_SNAPSHOT_INCLUDE_WORLD", "0");
  auto world = std::make_shared<IdentityOnlyWorld>();
  TestCommitAuthorizer authorizer(world);
  navigation_planning_backend::Config config(PLANNER_FACADE_CONFIG_PATH);
  config.bindWorldGeometry(world->geometry());
  std::optional<navigation_planning::CandidateBundle> predecessor;
  navigation_planning::TrajectoryPoint anchor_point;
  std::int64_t activation_ns = 0;
  double backend_entry_s = 0.0;
  {
    double ros_time_s = 10.0;
    navigation_planning_backend::PlannerFacade facade(
        PLANNER_FACADE_CONFIG_PATH, world, std::nullopt, authorizer,
        [&ros_time_s] { return ros_time_s; });
    auto request = plannerBodySupportRequest(world, nullptr);
    request.dynamics.intent.requested_cruise_speed_mps = 5.0;
    request.key.anchor_stamp_ns = 10'000'000'000LL;
    request.start_state.source_stamp_ns = request.key.anchor_stamp_ns;
    request.start_state.receive_stamp_ns = request.key.anchor_stamp_ns;
    ASSERT_TRUE(request.valid());
    const auto initial = facade.plan(request);
    ASSERT_TRUE(initial.valid()) << static_cast<int>(initial.failure_stage)
                                 << ":" << static_cast<int>(initial.failure_reason);
    ASSERT_TRUE(initial.candidate);
    predecessor = initial.candidate;
    facade.onExecutionTimelineActivated(predecessor->bundle_generation);
    const double main_end_s = predecessor->backup_available
        ? predecessor->backup_start_time_s : predecessor->duration_s;
    const double request_offset_s = std::min(0.5, main_end_s * 0.25);
    const auto request_ns = predecessor->declared_start_ns +
        static_cast<std::int64_t>(std::llround(request_offset_s * 1.0e9));
    activation_ns = request_ns + 400'000'000LL;
    ASSERT_LT(activation_ns, predecessor->declared_start_ns +
        static_cast<std::int64_t>(std::llround(main_end_s * 1.0e9)));
    const auto measured = predecessor->sampleAtDeclaredStamp(request_ns);
    const auto anchor_sample = predecessor->sampleAtDeclaredStamp(activation_ns);
    ASSERT_TRUE(measured);
    ASSERT_TRUE(anchor_sample);
    anchor_point = *anchor_sample;
    ASSERT_EQ(anchor_point.role, navigation_planning::CandidateRole::kMain);
    ASSERT_GT(anchor_point.velocity_world.norm(), 0.0);
    request.key.start_mode = navigation_planning::PlanningStartMode::kCommittedFutureState;
    request.key.committed_bundle_generation = predecessor->bundle_generation;
    request.key.anchor_stamp_ns = request_ns;
    request.start_state.position_world = measured->position_world;
    request.start_state.velocity_world = measured->velocity_world;
    request.start_state.acceleration_world = measured->acceleration_world;
    request.start_state.jerk_world = measured->jerk_world;
    request.start_state.source_stamp_ns = request_ns;
    request.start_state.receive_stamp_ns = request_ns;
    request.activation_stamp_ns = activation_ns;
    navigation_planning::ExecutionAnchor anchor;
    anchor.active_bundle_generation = predecessor->bundle_generation;
    anchor.execution_lineage_version = 1U;
    anchor.localization_epoch = request.key.localization_epoch;
    anchor.goal_epoch = request.key.goal_epoch;
    anchor.request_id = request.key.request_id;
    anchor.request_stamp_ns = request_ns;
    anchor.activation_stamp_ns = activation_ns;
    anchor.state = anchor_point;
    anchor.active_role = anchor_point.role;
    anchor.active_main_end_ns = predecessor->declared_start_ns +
        static_cast<std::int64_t>(std::llround(main_end_s * 1.0e9));
    anchor.active_bundle_end_ns = predecessor->declared_end_ns;
    anchor.command_world = predecessor->world_identity;
    request.anchor = anchor;
    request.history.previous_bundle_generation = predecessor->bundle_generation;
    request.history.previous_velocity_world = measured->velocity_world;
    request.current_body_support.reset();
    request.budget.deadline = navigation_planning::PlanningBudget::Clock::now() +
        std::chrono::duration_cast<navigation_planning::PlanningBudget::Clock::duration>(
            std::chrono::duration<double>(
                navigation_planning::PlanningTimingContract::kSolveDeadlineS));
    request.budget.steady_deadline_ns = std::chrono::duration_cast<
        std::chrono::nanoseconds>(request.budget.deadline.time_since_epoch()).count();
    ASSERT_TRUE(request.valid());
    // The request and anchor are immutable before the worker's source clock
    // advances. No sleep or wall-time scheduling assumption is involved.
    ros_time_s = static_cast<double>(activation_expired
        ? activation_ns + backend_delay_ns : request_ns + backend_delay_ns) * 1.0e-9;
    backend_entry_s = ros_time_s;
    const auto successor = facade.plan(request);
    if (activation_expired) {
      EXPECT_FALSE(successor.candidate);
      EXPECT_FALSE(navigation_planning::completePlanningSucceeded(successor.outcome));
      EXPECT_FALSE(facade.hasStagedCommandCandidate());
      EXPECT_EQ(facade.committedGeneration(), predecessor->bundle_generation);
      return;
    }
    ASSERT_TRUE(successor.valid()) << static_cast<int>(successor.failure_stage)
                                   << ":" << static_cast<int>(successor.failure_reason);
    ASSERT_TRUE(successor.candidate);
    EXPECT_EQ(successor.candidate->activation_stamp_ns, activation_ns);
    EXPECT_EQ(successor.candidate->declared_start_ns, activation_ns);
    const auto output_head = successor.candidate->sampleAtDeclaredStamp(activation_ns);
    ASSERT_TRUE(output_head);
    EXPECT_LE((output_head->position_world - anchor_point.position_world).norm(), 1.0e-8);
    EXPECT_LE((output_head->velocity_world - anchor_point.velocity_world).norm(), 1.0e-8);
    EXPECT_LE((output_head->acceleration_world - anchor_point.acceleration_world).norm(), 1.0e-8);
    EXPECT_LE((output_head->jerk_world - anchor_point.jerk_world).norm(), 1.0e-8);
  }  // Destruction joins the existing capture writer and drains both snapshots.

  const auto accounting = YAML::LoadFile(
      (directory.path() / "nominal_problem_snapshot_capture.json").string());
  ASSERT_TRUE(snapshotScalar<bool>(accounting["capture_complete"], "capture.capture_complete"));
  ASSERT_EQ(snapshotScalar<std::uint64_t>(accounting["dropped_records"],
                                        "capture.dropped_records"), 0U);
  ASSERT_EQ(snapshotScalar<std::uint64_t>(accounting["write_error_count"],
                                        "capture.write_error_count"), 0U);
  YAML::Node snapshot;
  std::size_t matching_snapshots = 0U;
  for (const auto& entry : std::filesystem::directory_iterator(directory.path())) {
    if (entry.path().filename().string().rfind("nominal_problem_snapshot_", 0U) != 0U ||
        entry.path().filename() == "nominal_problem_snapshot_capture.json" ||
        entry.path().extension() != ".json") continue;
    SCOPED_TRACE(entry.path().string());
    const auto candidate = YAML::LoadFile(entry.path().string());
    const auto candidate_activation = candidate["provenance"]["activation_stamp_ns"];
    if (!candidate_activation || candidate_activation.IsNull() ||
        !candidate_activation.IsScalar()) continue;
    if (snapshotScalar<std::int64_t>(candidate_activation,
                                    "provenance.activation_stamp_ns") == activation_ns) {
      snapshot = candidate;
      ++matching_snapshots;
    }
  }
  ASSERT_EQ(matching_snapshots, 1U);
  ASSERT_TRUE(snapshot["provenance"]["anchor_stamp_ns"].IsScalar())
      << "successor snapshot anchor_stamp_ns is missing/non-scalar";
  EXPECT_EQ(snapshotScalar<std::int64_t>(snapshot["provenance"]["anchor_stamp_ns"],
                                        "provenance.anchor_stamp_ns"), activation_ns);
  ASSERT_TRUE(snapshot["provenance"]["solve_start_wall_time_s"].IsScalar())
      << "successor snapshot solve_start_wall_time_s is missing/non-scalar";
  EXPECT_DOUBLE_EQ(snapshotScalar<double>(snapshot["provenance"]["solve_start_wall_time_s"],
                                         "provenance.solve_start_wall_time_s"), backend_entry_s);
  const auto head = snapshot["problem"]["head_pvaj"];
  const Eigen::Vector3d derivatives[]{anchor_point.position_world,
      anchor_point.velocity_world, anchor_point.acceleration_world, anchor_point.jerk_world};
  ASSERT_TRUE(head.IsSequence());
  ASSERT_EQ(head.size(), 3U);
  for (std::size_t axis = 0U; axis < 3U; ++axis) {
    ASSERT_TRUE(head[axis].IsSequence());
    ASSERT_EQ(head[axis].size(), 4U);
    for (std::size_t derivative = 0U; derivative < 4U; ++derivative) {
      ASSERT_TRUE(head[axis][derivative].IsScalar())
          << "head_pvaj axis=" << axis << " derivative=" << derivative;
      EXPECT_DOUBLE_EQ(snapshotScalar<double>(head[axis][derivative],
          "problem.head_pvaj[" + std::to_string(axis) + "][" +
          std::to_string(derivative) + "]"), derivatives[derivative][axis]);
    }
  }
  const auto points = snapshot["problem"]["guide_path"];
  const auto times = snapshot["problem"]["guide_stamp"];
  ASSERT_TRUE(points.IsSequence());
  ASSERT_TRUE(times.IsSequence());
  ASSERT_EQ(points.size(), times.size());
  ASSERT_GT(points.size(), 1U);
  ASSERT_TRUE(times[0U].IsScalar()) << "problem.guide_stamp[0]";
  EXPECT_DOUBLE_EQ(snapshotScalar<double>(times[0U], "problem.guide_stamp[0]"), 0.0);
  // Identify the actual retained prefix by the product's sample cadence and
  // spatial window, not by accepting arbitrary A* points as predecessor data.
  const double activation_tt_s = static_cast<double>(
      activation_ns - predecessor->declared_start_ns) * 1.0e-9;
  auto last_sample = anchor_point.position_world;
  std::size_t retained_count = 0U;
  for (double sample_tt_s = activation_tt_s + config.sample_traj_dt_s;
       sample_tt_s < predecessor->duration_s; sample_tt_s += config.sample_traj_dt_s) {
    const auto sample_ns = predecessor->declared_start_ns +
        static_cast<std::int64_t>(std::llround(sample_tt_s * 1.0e9));
    const auto sample = predecessor->sampleAtDeclaredStamp(sample_ns);
    ASSERT_TRUE(sample);
    if ((sample->position_world - last_sample).norm() < config.resolution * 0.8) continue;
    last_sample = sample->position_world;
    if ((sample->position_world - anchor_point.position_world).norm() >
        config.receding_distance_m) break;
    ++retained_count;
    ASSERT_LT(retained_count, points.size());
    ASSERT_TRUE(times[retained_count].IsScalar())
        << "retained guide_stamp index=" << retained_count;
    const double elapsed_s = snapshotScalar<double>(times[retained_count],
        "problem.guide_stamp[" + std::to_string(retained_count) + "]");
    EXPECT_GT(elapsed_s, 0.0);
    EXPECT_NEAR(elapsed_s, sample_tt_s - activation_tt_s, 1.0e-9);
    const auto corresponding = predecessor->sampleAtDeclaredStamp(activation_ns +
        static_cast<std::int64_t>(std::llround(elapsed_s * 1.0e9)));
    ASSERT_TRUE(corresponding);
    ASSERT_TRUE(points[retained_count].IsSequence());
    ASSERT_EQ(points[retained_count].size(), 3U);
    for (std::size_t axis = 0U; axis < 3U; ++axis) {
      // Vec3f uses the existing matrix serializer: three rows, one column.
      ASSERT_TRUE(points[retained_count][axis].IsSequence());
      ASSERT_EQ(points[retained_count][axis].size(), 1U);
      ASSERT_TRUE(points[retained_count][axis][0U].IsScalar())
          << "problem.guide_path[" << retained_count << "][" << axis << "]";
      EXPECT_NEAR(snapshotScalar<double>(points[retained_count][axis][0U],
                      "problem.guide_path[" + std::to_string(retained_count) + "][" +
                      std::to_string(axis) + "][0]"),
                  corresponding->position_world[axis], 1.0e-8);
    }
  }
  ASSERT_GT(retained_count, 0U);
}

TEST(PlannerFacade, DelayedBackendKeepsRequestOwnedGuideOrigin) {
  for (const std::int64_t delay_ns : {0LL, 20'000'000LL, 60'000'000LL}) {
    SCOPED_TRACE(delay_ns);
    expectRequestOwnedGuideOrigin(delay_ns);
  }
}

TEST(PlannerFacade, SequentialRequestsDoNotReusePriorRouteCruiseOrMissionStart) {
  auto world = std::make_shared<IdentityOnlyWorld>();
  TestCommitAuthorizer authorizer(world);
  navigation_planning_backend::PlannerFacade facade(
      PLANNER_FACADE_CONFIG_PATH, world, std::nullopt, authorizer,
      [] { return 10.0; });

  navigation_planning::KinematicState measured;
  measured.position_world = Eigen::Vector3d{0.0, 0.0, 3.0};
  measured.source_stamp_ns = 10'000'000'000LL;
  measured.receive_stamp_ns = measured.source_stamp_ns;
  measured.localization_epoch = 1U;
  measured.world_frame_id = "lio_odom";
  measured.body_frame_id = "base_link";

  navigation_mission::Mission route_a;
  route_a.id = "request-context";
  route_a.frame = "lio_odom";
  route_a.waypoints = {{
      "east-stop", Eigen::Vector3d{6.0, 0.0, 3.0}, 0.5, 0.0,
      navigation_mission::MissionWaypoint::Behavior::Stop}};
  navigation_mission::RouteProgress progress_a(route_a);
  ASSERT_TRUE(progress_a.update(measured.position_world).valid);
  const auto snapshot_a = progress_a.snapshot(
      route_a.id, route_a.frame, 1U, 1U, 0U);
  ASSERT_TRUE(snapshot_a.valid());

  auto limits_a = *semanticFixtureMissionLimits();
  limits_a.intent.requested_cruise_speed_mps = 4.0;
  const Eigen::Vector3d mission_start_a{0.0, 0.0, 3.0};
  const auto request_a = plannerRequestForRoute(
      world, snapshot_a, measured, limits_a, 1U, mission_start_a);
  ASSERT_TRUE(request_a.valid());
  const auto outcome_a = facade.plan(request_a);
  ASSERT_TRUE(outcome_a.valid())
      << static_cast<int>(outcome_a.failure_stage) << ":"
      << static_cast<int>(outcome_a.failure_reason);
  ASSERT_TRUE(outcome_a.candidate);
  facade.onExecutionTimelineActivated(outcome_a.candidate->bundle_generation);

  navigation_mission::Mission route_b;
  route_b.id = route_a.id;
  route_b.frame = route_a.frame;
  route_b.waypoints = {{
      "north-stop", Eigen::Vector3d{0.0, 6.0, 3.0}, 0.35, 0.0,
      navigation_mission::MissionWaypoint::Behavior::Stop}};
  navigation_mission::RouteProgress progress_b(route_b);
  ASSERT_TRUE(progress_b.update(measured.position_world).valid);
  const auto snapshot_b = progress_b.snapshot(
      route_b.id, route_b.frame, 2U, 2U, 0U);
  ASSERT_TRUE(snapshot_b.valid());

  auto limits_b = limits_a;
  limits_b.intent.requested_cruise_speed_mps = 2.0;
  const Eigen::Vector3d mission_start_b{0.0, 1.0, 3.0};
  auto request_b = plannerRequestForRoute(
      world, snapshot_b, measured, limits_b, 2U, mission_start_b);
  request_b.history.previous_bundle =
      std::make_shared<const navigation_planning::CandidateBundle>(
          *outcome_a.candidate);
  request_b.history.previous_bundle_generation =
      outcome_a.candidate->bundle_generation;
  request_b.history.previous_velocity_world = measured.velocity_world;
  ASSERT_TRUE(request_b.valid());

  const auto outcome_b = facade.plan(request_b);
  ASSERT_TRUE(outcome_b.valid())
      << static_cast<int>(outcome_b.failure_stage) << ":"
      << static_cast<int>(outcome_b.failure_reason);
  ASSERT_TRUE(outcome_b.candidate);
  EXPECT_EQ(outcome_b.candidate->request_id, request_b.key.request_id);
  EXPECT_EQ(outcome_b.candidate->goal_epoch, request_b.key.goal_epoch);
  EXPECT_EQ(outcome_b.candidate->localization_epoch,
            request_b.key.localization_epoch);
  EXPECT_TRUE(facade.diagnostics().requested_goal.isApprox(
      Eigen::Vector3d{0.0, 6.0, 3.0}, 1.0e-6));
  EXPECT_NEAR(facade.diagnostics().requested_cruise_speed_mps, 2.0, 1.0e-9);
  EXPECT_NEAR(facade.diagnostics().effective_cruise_speed_mps, 2.0, 1.0e-9);
  EXPECT_NEAR(facade.diagnostics().route_yaw_target_rad, M_PI_2, 1.0e-6);
}

TEST(PlannerFacade, ExpiredCommittedActivationDoesNotSlideOrReplacePredecessor) {
  expectRequestOwnedGuideOrigin(0LL, true);
  expectRequestOwnedGuideOrigin(1'000'000LL, true);
}

void expectMovingFrontierBehavior(const bool backup_allow_unknown) {
  // Exercise the real product transaction, with a predecessor produced and
  // activated by this same facade. No moving state is labelled stopped, no
  // execution anchor is invented, and the immutable world never changes.
  class FrontierWorld final : public IdentityOnlyWorld {
   public:
    navigation_world_model::WorldGeometry geometry() const noexcept override {
      auto value = IdentityOnlyWorld::geometry();
      // Same product dimensions, with the synthetic local window centred at
      // x=10 m. All authorized FAST samples must remain inside this window.
      value.evidence_bounds.global_min_index = Eigen::Vector3i{-75, -125, -5};
      value.inflated_bounds.global_min_index = value.evidence_bounds.global_min_index;
      return value;
    }
    bool contains(const navigation_world_model::Point3& point) const noexcept override {
      return point.allFinite() && point.x() >= -15.0 && point.x() < 35.0 &&
             point.y() >= -25.0 && point.y() < 25.0 &&
             point.z() >= -1.0 && point.z() < 5.0;
    }
    navigation_world_model::WorldSnapshotIdentity identity() const noexcept override {
      return {1U, 1U, 1U, 10'000'000'000LL};
    }
    navigation_world_model::CellState classify(
        const navigation_world_model::Point3& point,
        navigation_world_model::GridLayer) const noexcept override {
      if (!contains(point)) return navigation_world_model::CellState::kOutOfMap;
      return point.x() < 15.0 ? navigation_world_model::CellState::kKnownFree
                             : navigation_world_model::CellState::kUnknown;
    }
    bool isSegmentTraversable(
        const navigation_world_model::Point3& start,
        const navigation_world_model::Point3& end,
        navigation_world_model::GridLayer,
        navigation_world_model::UnknownPolicy policy) const noexcept override {
      return contains(start) && contains(end) &&
             (policy == navigation_world_model::UnknownPolicy::kAllowUnknown ||
              std::max(start.x(), end.x()) < 15.0);
    }
    navigation_world_model::AxisAlignedBox clampToLocalBounds(
        const navigation_world_model::AxisAlignedBox& box) const noexcept override {
      return {box.minimum.cwiseMax(Eigen::Vector3d{-15.0, -25.0, -1.0}),
              box.maximum.cwiseMin(Eigen::Vector3d{35.0, 25.0, 5.0})};
    }
  };
  auto world = std::make_shared<const FrontierWorld>();
  TestCommitAuthorizer authorizer(world);
  double ros_time_s = 10.0;
  navigation_planning::DynamicLimits limits;
  limits.intent.requested_cruise_speed_mps = 5.0;
  limits.unknown_space_policy = navigation_world_model::UnknownPolicy::kAllowUnknown;
  const char* config_path = backup_allow_unknown
      ? PLANNER_FACADE_FAST_CONFIG_PATH : PLANNER_FACADE_CONFIG_PATH;
  navigation_planning_backend::PlannerFacade facade(
      config_path, world, limits, authorizer,
      [&ros_time_s] { return ros_time_s; });
  auto request = plannerBodySupportRequest(
      world, nullptr, Eigen::Vector3d{32.0, 0.0, 2.0});
  request.start_state.source_stamp_ns = 10'000'000'000LL;
  request.start_state.receive_stamp_ns = request.start_state.source_stamp_ns;
  request.key.anchor_stamp_ns = request.start_state.source_stamp_ns;
  request.dynamics = limits;
  const auto set_product_budget = [&] {
    request.budget.deadline = navigation_planning::PlanningBudget::Clock::now() +
        std::chrono::milliseconds(80);
    request.budget.steady_deadline_ns =
        std::chrono::duration_cast<std::chrono::nanoseconds>(
            request.budget.deadline.time_since_epoch()).count();
  };
  set_product_budget();
  ASSERT_TRUE(request.valid());
  const auto initial = facade.plan(request);
  ASSERT_TRUE(initial.valid()) << static_cast<int>(initial.failure_stage) << ":"
                              << static_cast<int>(initial.failure_reason);
  ASSERT_TRUE(initial.candidate);
  ASSERT_TRUE(initial.candidate->backup_available);
  const auto& predecessor = *initial.candidate;
  facade.onExecutionTimelineActivated(predecessor.bundle_generation);

  // Place activation 4.25 m before the known-free frontier, on actual MAIN.
  // Binary search samples only this already-certified immutable predecessor.
  const auto stamp_at = [&](const double time_s) {
    return predecessor.declared_start_ns +
        static_cast<std::int64_t>(std::llround(time_s * 1.0e9));
  };
  const auto final_main = predecessor.sampleAtDeclaredStamp(
      stamp_at(predecessor.backup_start_time_s - 1.0e-5));
  ASSERT_TRUE(final_main);
  ASSERT_GT(final_main->position_world.x(), 10.75);
  double lower_s = 0.0;
  double upper_s = predecessor.backup_start_time_s - 1.0e-5;
  for (int iteration = 0; iteration < 60; ++iteration) {
    const double middle_s = 0.5 * (lower_s + upper_s);
    const auto sample = predecessor.sampleAtDeclaredStamp(stamp_at(middle_s));
    ASSERT_TRUE(sample);
    if (sample->position_world.x() < 10.75) lower_s = middle_s;
    else upper_s = middle_s;
  }
  const auto activation_stamp_ns = stamp_at(upper_s);
  const auto anchor_sample = predecessor.sampleAtDeclaredStamp(activation_stamp_ns);
  ASSERT_TRUE(anchor_sample);
  ASSERT_EQ(anchor_sample->role, navigation_planning::CandidateRole::kMain);
  ASSERT_GT(anchor_sample->velocity_world.norm(), 4.0);
  request.key.anchor_stamp_ns = activation_stamp_ns - 400'000'000LL;
  const auto measured = predecessor.sampleAtDeclaredStamp(request.key.anchor_stamp_ns);
  ASSERT_TRUE(measured);
  ros_time_s = static_cast<double>(request.key.anchor_stamp_ns) * 1.0e-9;
  request.start_state.position_world = measured->position_world;
  request.start_state.velocity_world = measured->velocity_world;
  request.start_state.acceleration_world = measured->acceleration_world;
  request.start_state.jerk_world = measured->jerk_world;
  request.start_state.source_stamp_ns = request.key.anchor_stamp_ns;
  request.start_state.receive_stamp_ns = request.key.anchor_stamp_ns;
  request.key.start_mode = navigation_planning::PlanningStartMode::kCommittedFutureState;
  request.key.committed_bundle_generation = predecessor.bundle_generation;
  request.activation_stamp_ns = activation_stamp_ns;
  navigation_planning::ExecutionAnchor anchor;
  anchor.active_bundle_generation = predecessor.bundle_generation;
  anchor.execution_lineage_version = 1U;
  anchor.localization_epoch = request.key.localization_epoch;
  anchor.goal_epoch = request.key.goal_epoch;
  anchor.request_id = request.key.request_id;
  anchor.request_stamp_ns = request.key.anchor_stamp_ns;
  anchor.activation_stamp_ns = activation_stamp_ns;
  anchor.state = *anchor_sample;
  anchor.active_role = anchor_sample->role;
  anchor.active_main_end_ns = stamp_at(predecessor.backup_start_time_s);
  anchor.active_bundle_end_ns = predecessor.declared_end_ns;
  anchor.command_world = predecessor.world_identity;
  request.anchor = anchor;
  request.history.previous_bundle_generation = predecessor.bundle_generation;
  request.history.previous_velocity_world = anchor_sample->velocity_world;
  set_product_budget();
  ASSERT_TRUE(request.valid());
  const auto successor = facade.plan(request);
  const auto diagnostics = facade.diagnostics();
  ::testing::Test::RecordProperty("backup_policy", backup_allow_unknown ? "allow_unknown" : "require_known_free");
  ::testing::Test::RecordProperty("anchor_speed_mps", std::to_string(anchor_sample->velocity_world.norm()));
  ::testing::Test::RecordProperty("anchor_acceleration_mps2", std::to_string(anchor_sample->acceleration_world.x()));
  ::testing::Test::RecordProperty("anchor_jerk_mps3", std::to_string(anchor_sample->jerk_world.x()));
  ::testing::Test::RecordProperty("frontier_support_m", std::to_string(15.0 - anchor_sample->position_world.x()));
  ::testing::Test::RecordProperty("failure_stage", std::to_string(static_cast<int>(successor.failure_stage)));
  ::testing::Test::RecordProperty("failure_reason", std::to_string(static_cast<int>(successor.failure_reason)));
  ::testing::Test::RecordProperty("backup_reject_stage", std::to_string(diagnostics.backup_certificate.last_reject_stage));
  ::testing::Test::RecordProperty("chosen_main_backup_stop_x_m", std::to_string(
      diagnostics.backup_certificate.last_seed_endpoint.x()));
  ::testing::Test::RecordProperty("solve_elapsed_ms", std::to_string(
      static_cast<double>(successor.trace.elapsed_steady_ns) * 1.0e-6));
  if (backup_allow_unknown) {
    ASSERT_TRUE(successor.valid());
    ASSERT_TRUE(navigation_planning::completePlanningSucceeded(successor.outcome));
    ASSERT_TRUE(successor.candidate);
    EXPECT_TRUE(successor.candidate->backup_available);
    EXPECT_GT(successor.candidate->bundle_generation, predecessor.bundle_generation);
    EXPECT_TRUE(diagnostics.backup_certificate.selected);
    EXPECT_TRUE(facade.hasStagedCommandCandidate());
    EXPECT_FALSE(facade.discardRetainedPositionHeadingCandidate(
        successor.candidate->bundle_generation));
    EXPECT_TRUE(facade.hasStagedCommandCandidate());
    EXPECT_TRUE(facade.validateStagedCommandCandidate(
        world, ros_time_s, successor.candidate->bundle_generation).valid);
    EXPECT_EQ(facade.committedGeneration(), predecessor.bundle_generation);
    return;  // Staging is not activation or measured waypoint acceptance.
  }
  EXPECT_TRUE(successor.valid());  // A well-formed failure is not executable.
  EXPECT_EQ(successor.outcome,
            navigation_planning::CompletePlanningOutcome::kNoCompleteBundle);
  EXPECT_EQ(successor.failure_stage,
            navigation_planning::PlanningFailureStage::kBackupSeed);
  EXPECT_EQ(successor.failure_reason,
            navigation_planning::PlanningFailureReason::kBackupKnownFreeInsufficient);
  EXPECT_FALSE(successor.candidate);
  EXPECT_TRUE(diagnostics.backup_certificate.attempted);
  EXPECT_FALSE(diagnostics.backup_certificate.selected);
  EXPECT_GT(diagnostics.backup_certificate.feasible_seed_count, 0U);
  EXPECT_GT(diagnostics.backup_certificate.aligned_hull_pass_count, 0U);
  EXPECT_GT(diagnostics.backup_certificate.known_free_check_count, 0U);
  EXPECT_EQ(diagnostics.backup_certificate.known_free_pass_count, 0U);
  EXPECT_EQ(diagnostics.backup_certificate.last_known_free_cell_state,
            static_cast<int>(navigation_world_model::CellState::kUnknown));
  EXPECT_FALSE(facade.hasStagedCommandCandidate());
  EXPECT_EQ(facade.committedGeneration(), predecessor.bundle_generation);

  // Independent construction on the exact same actual anchor. This is a
  // search-completeness discriminator, NOT a second online authority path or
  // a claim that the frontend/CIRI can select this profile on a real map.
  navigation_planning_backend::Config config(PLANNER_FACADE_CONFIG_PATH, limits);
  config.bindWorldGeometry(world->geometry());
  navigation_math::StatePVAJ actual_initial = navigation_math::StatePVAJ::Zero();
  actual_initial.col(0) = anchor_sample->position_world;
  actual_initial.col(1) = anchor_sample->velocity_world;
  actual_initial.col(2) = anchor_sample->acceleration_world;
  actual_initial.col(3) = anchor_sample->jerk_world;
  constexpr double kJerkRampS = 0.1;
  const double reserve_s =
      navigation_planning::PlanningTimingContract::kMinimumMainReserveS;
  const Eigen::Vector3d target_jerk =
      -config.exp_traj_cfg.max_jerk *
      config.exp_traj_cfg.optimization_dynamic_reserve_ratio *
      anchor_sample->velocity_world.normalized();
  Eigen::MatrixXd ramp = Eigen::MatrixXd::Zero(3, 8);
  ramp.col(7) = actual_initial.col(0);
  ramp.col(6) = actual_initial.col(1);
  ramp.col(5) = actual_initial.col(2) / 2.0;
  ramp.col(4) = actual_initial.col(3) / 6.0;
  ramp.col(3) = (target_jerk - actual_initial.col(3)) / (24.0 * kJerkRampS);
  geometry_utils::Trajectory decelerating({kJerkRampS}, {ramp});
  const auto ramp_end = decelerating.getState(kJerkRampS);
  Eigen::MatrixXd hold = Eigen::MatrixXd::Zero(3, 8);
  hold.col(7) = ramp_end.col(0);
  hold.col(6) = ramp_end.col(1);
  hold.col(5) = ramp_end.col(2) / 2.0;
  hold.col(4) = ramp_end.col(3) / 6.0;
  decelerating.emplace_back(reserve_s - kJerkRampS, hold);
  ASSERT_TRUE(decelerating.getState(0.0).isApprox(actual_initial, 1.0e-12));
  ASSERT_TRUE(decelerating[1].getState(0.0).isApprox(ramp_end, 1.0e-12));
  ASSERT_LE(decelerating.getMaxVelRate(), config.exp_traj_cfg.max_vel);
  ASSERT_LE(decelerating.getMaxAccRate(), config.exp_traj_cfg.max_acc);
  ASSERT_LE(decelerating.getMaxJerRate(), config.exp_traj_cfg.max_jerk);
  const auto switch_state = decelerating.getState(reserve_s);
  const auto seed = navigation_planning_backend::makeBackupBrakingSeed(
      reserve_s, switch_state, config.back_traj_cfg.max_vel,
      config.back_traj_cfg.max_acc, config.back_traj_cfg.max_jerk,
      config.sample_traj_dt_s, 0.0);
  ASSERT_TRUE(seed.feasible);
  geometry_utils::Trajectory stop;
  stop.emplace_back(navigation_planning_backend::minimumSnapStopPiece(
      switch_state, seed.duration_s));
  ASSERT_TRUE(stop.getState(0.0).isApprox(switch_state, 1.0e-12));
  const Eigen::MatrixXd yaw_coefficients = [&] {
    Eigen::MatrixXd value = Eigen::MatrixXd::Zero(3, 8);
    value(0, 7) = anchor_sample->yaw;
    return value;
  }();
  ASSERT_NEAR(anchor_sample->yaw_rate, 0.0, 1.0e-12);
  geometry_utils::Trajectory main_yaw({reserve_s}, {yaw_coefficients});
  geometry_utils::Trajectory backup_yaw({seed.duration_s}, {yaw_coefficients});
  ASSERT_TRUE(traj_opt::trajectorySatisfiesFlatnessEnvelope(
      decelerating, config.exp_traj_cfg, nullptr, 0.01, &main_yaw));
  ASSERT_TRUE(traj_opt::trajectorySatisfiesFlatnessEnvelope(
      stop, config.back_traj_cfg, nullptr, 0.01, &backup_yaw));
  navigation_planning_backend::ExpTraj alternate_main;
  const double activation_s = static_cast<double>(activation_stamp_ns) * 1.0e-9;
  alternate_main.setTrajectory(activation_s, decelerating, main_yaw);
  ASSERT_TRUE(alternate_main.setRequiredMainPrefixDuration(reserve_s));
  navigation_planning_backend::BackupTraj alternate_backup;
  alternate_backup.setTrajectory(activation_s + reserve_s, reserve_s,
                                 stop, backup_yaw);
  const auto alternate = navigation_planning_backend::CmdTraj::buildCandidate(
      alternate_main, &alternate_backup,
      navigation_planning_backend::BackupDisposition::SUCCESS);
  ASSERT_TRUE(alternate);
  ASSERT_DOUBLE_EQ(alternate->backup_start_tt, reserve_s);
  // Independently supplied convex corridor wholly inside the synthetic
  // world's known-free half-space. This does not bypass product CIRI.
  navigation_math::MatD4f corridor(6, 4);
  corridor << 1.0, 0.0, 0.0, -14.8,
             -1.0, 0.0, 0.0, 10.5,
              0.0, 1.0, 0.0, -1.0,
              0.0,-1.0, 0.0, -1.0,
              0.0, 0.0, 1.0, -3.0,
              0.0, 0.0,-1.0,  1.0;
  for (int piece = 0; piece < alternate->position.getPieceNum(); ++piece) {
    ASSERT_LE(navigation_planning_backend::maximumContinuousCorridorPlaneViolation(
                  alternate->position[piece], corridor),
              config.exp_traj_cfg.corridor_plane_tolerance_m);
  }
  ASSERT_TRUE(navigation_planning_backend::validateExecutableCandidate(
      *world, *alternate, activation_s,
      navigation_world_model::UnknownPolicy::kAllowUnknown, {}, false,
      navigation_world_model::UnknownPolicy::kRequireKnownFree).valid);
  ::testing::Test::RecordProperty("decelerating_stop_x_m", std::to_string(
      alternate->position.getPos(alternate->position.getTotalDuration()).x()));
  EXPECT_FALSE(facade.hasStagedCommandCandidate());
  EXPECT_EQ(facade.committedGeneration(), predecessor.bundle_generation);
}

TEST(PlannerFacade, SafeMovingFrontierRejectsChosenMainWhileDecelerationPassesCertificates) {
  expectMovingFrontierBehavior(false);
}

TEST(PlannerFacade, FastMovingFrontierAdmitsBackupUnknownWithoutActivatingSuccessor) {
  expectMovingFrontierBehavior(true);
}

TEST(PlannerFacade, SupersededUnactivatedProposalsNeverAliasGeneration) {
  auto world = std::make_shared<IdentityOnlyWorld>();
  TestCommitAuthorizer authorizer(world);
  double ros_time_s = 10.0;
  navigation_planning_backend::PlannerFacade facade(
      PLANNER_FACADE_CONFIG_PATH, world, std::nullopt, authorizer,
      [&ros_time_s] { return ros_time_s; });

  auto request = plannerBodySupportRequest(world, nullptr);
  request.dynamics.intent.requested_cruise_speed_mps = 5.0;
  ASSERT_TRUE(request.valid());
  const auto first = facade.plan(request);
  ASSERT_TRUE(first.valid());
  ASSERT_TRUE(first.candidate.has_value());

  // Do not activate the first proposal. A new solve supersedes the backend's
  // staged value while execution may still hold the old pointer as pending.
  const auto replacement = facade.plan(request);
  ASSERT_TRUE(replacement.valid());
  ASSERT_TRUE(replacement.candidate.has_value());
  EXPECT_GT(replacement.candidate->bundle_generation,
            first.candidate->bundle_generation);
  EXPECT_FALSE(facade.validateStagedCommandCandidate(
      world, ros_time_s, first.candidate->bundle_generation).valid);
  EXPECT_TRUE(facade.validateStagedCommandCandidate(
      world, ros_time_s, replacement.candidate->bundle_generation).valid);
}

TEST(PlannerFacade,
     StagesTerminalStopHoldWhenUnknownMeasuredStateIsAlreadyAccepted) {
  auto world = std::make_shared<PlannerBodySupportWorld>();
  const auto support = plannerBodySupport(world->identity());
  ASSERT_TRUE(support);

  TestCommitAuthorizer authorizer(world);
  navigation_planning_backend::PlannerFacade facade(
      PLANNER_FACADE_CONFIG_PATH, world, std::nullopt, authorizer,
      [] { return 10.0; });
  const auto request = plannerBodySupportRequest(
      world, support, Eigen::Vector3d{0.01, 0.0, 2.0});
  ASSERT_TRUE(request.valid());

  const auto outcome = facade.plan(request);
  ASSERT_TRUE(outcome.valid())
      << static_cast<int>(outcome.failure_stage) << ":"
      << static_cast<int>(outcome.failure_reason);
  ASSERT_TRUE(outcome.candidate.has_value());
  EXPECT_TRUE(navigation_planning::completePlanningSucceeded(outcome.outcome));
  EXPECT_TRUE(outcome.candidate->terminal_stop);

  navigation_planning::TrajectoryPoint initial;
  ASSERT_TRUE(outcome.candidate->evaluator(
      outcome.candidate->declared_start_ns, initial));
  EXPECT_EQ(initial.role, navigation_planning::CandidateRole::kMain);
  EXPECT_LE((initial.position_world - Eigen::Vector3d{0.0, 0.0, 2.0}).norm(),
            1.0e-9);
  EXPECT_LE(initial.velocity_world.norm(),
            navigation_planning::PlanningTimingContract::kStationarySpeedMps +
                1.0e-9);

  navigation_planning::TrajectoryPoint terminal;
  ASSERT_TRUE(outcome.candidate->evaluator(
      outcome.candidate->declared_end_ns, terminal));
  EXPECT_LE((terminal.position_world - Eigen::Vector3d{0.01, 0.0, 2.0}).norm(),
            0.3 + 1.0e-9);
  EXPECT_NEAR(terminal.velocity_world.norm(), 0.0, 1.0e-9);
}

TEST(PlannerFacade, RequiresValidImmutableRouteBeforePlanning) {
  auto world = std::make_shared<IdentityOnlyWorld>();
  TestCommitAuthorizer authorizer(world);
  navigation_planning_backend::PlannerFacade facade(
      PLANNER_FACADE_CONFIG_PATH, world, std::nullopt, authorizer, [] { return 10.0; });

  navigation_mission::ImmutableRouteSnapshot invalid;
  EXPECT_FALSE(invalid.valid());

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
  EXPECT_TRUE(snapshot.valid());
}

TEST(PlannerFacade, ImmediateHeadingRebindRetainsPositionAndUsesNewActiveLeg) {
  auto world = std::make_shared<IdentityOnlyWorld>();
  TestCommitAuthorizer authorizer(world);
  double ros_time_s = 10.0;
  navigation_planning_backend::PlannerFacade facade(
      PLANNER_FACADE_CONFIG_PATH, world, semanticFixtureMissionLimits(), authorizer,
      [&] { return ros_time_s; });

  navigation_mission::Mission mission;
  mission.id = "immediate-heading-rebind";
  mission.frame = "lio_odom";
  navigation_mission::MissionWaypoint start;
  start.id = "start";
  start.position_enu = Eigen::Vector3d{0.0, 0.0, 3.0};
  start.acceptance_radius_m = 0.5;
  navigation_mission::MissionWaypoint first;
  first.id = "first";
  first.position_enu = Eigen::Vector3d{8.0, 0.0, 3.0};
  first.behavior = navigation_mission::MissionWaypoint::Behavior::PassThrough;
  first.acceptance_radius_m = 0.5;
  navigation_mission::MissionWaypoint second;
  second.id = "second";
  second.position_enu = Eigen::Vector3d{8.0, 8.0, 3.0};
  second.behavior = navigation_mission::MissionWaypoint::Behavior::PassThrough;
  second.acceptance_radius_m = 0.5;
  navigation_mission::MissionWaypoint terminal;
  terminal.id = "terminal";
  terminal.position_enu = Eigen::Vector3d{0.0, 8.0, 3.0};
  terminal.acceptance_radius_m = 0.5;
  mission.waypoints = {first, second, terminal};

  navigation_mission::RouteProgress progress(mission);
  ASSERT_TRUE(progress.update(start.position_enu).valid);
  const auto first_route = progress.snapshot(mission.id, mission.frame, 1U, 1U, 0U);
  ASSERT_TRUE(first_route.valid());
  ASSERT_EQ(first_route.active_waypoint_index, 0U);

  navigation_planning::KinematicState state;
  state.position_world = start.position_enu;
  state.source_stamp_ns = 1;
  state.receive_stamp_ns = 1;
  state.localization_epoch = 1U;
  state.world_frame_id = "lio_odom";
  state.body_frame_id = "base_link";
  const auto initial_outcome = facade.plan(plannerRequestForRoute(
      world, first_route, state, *semanticFixtureMissionLimits(), 1U,
      start.position_enu));
  ASSERT_TRUE(initial_outcome.valid())
      << static_cast<int>(initial_outcome.failure_stage) << ":"
      << static_cast<int>(initial_outcome.failure_reason);
  const auto initial = initial_outcome.candidate;
  ASSERT_TRUE(initial);
  ASSERT_GT(initial->bundle_generation, 0U);
  facade.onExecutionTimelineActivated(initial->bundle_generation);

  ASSERT_TRUE(progress.update(first.position_enu).valid);
  const auto second_route = progress.snapshot(mission.id, mission.frame, 1U, 2U, 1U);
  ASSERT_TRUE(second_route.valid());
  ASSERT_EQ(second_route.active_waypoint_index, 1U);
  state.position_world = Eigen::Vector3d{7.8, 0.0, 3.0};

  const auto out_of_band = facade.buildImmediateHeadingRebindCandidate(
      world, second_route, state.position_world, state.velocity_world,
      state.yaw_rad, start.position_enu, 10.5, 1U, 2U, 2U,
      10500000000LL, 30000000000LL);
  ASSERT_TRUE(out_of_band);
  ASSERT_TRUE(out_of_band->valid());
  EXPECT_EQ(out_of_band->source, navigation_planning::CandidateSource::kRetained);
  EXPECT_TRUE(out_of_band->certificates.world);
  EXPECT_TRUE(out_of_band->certificates.flatness);
  EXPECT_TRUE(out_of_band->protected_region.valid());
  EXPECT_EQ(out_of_band->world_identity.generation, world->identity().generation);

  // A position job may already have passed its activation-ACK boundary when
  // the independent heading worker exports this exact generation. Neither
  // reapplying the same request nor starting a newer desired request retires
  // that exported owner: the timeline can still activate it and ACK it later.
  EXPECT_FALSE(facade.hasStagedCommandCandidate());
  EXPECT_FALSE(facade.hasStagedCommandCandidate());
  facade.onExecutionTimelineActivated(initial->bundle_generation);
  EXPECT_EQ(facade.committedGeneration(), initial->bundle_generation);
  EXPECT_FALSE(facade.discardRetainedPositionHeadingCandidate(0U));
  EXPECT_FALSE(facade.discardRetainedPositionHeadingCandidate(
      out_of_band->bundle_generation + 1U));
  EXPECT_TRUE(facade.validateStagedCommandCandidate(
      world, 10.5, out_of_band->bundle_generation).valid);
  // The independent export must not make a failed nominal job look ready or
  // get exported as that job's result. Its immutable original identity stays.
  EXPECT_EQ(out_of_band->goal_epoch, 2U);
  EXPECT_EQ(out_of_band->request_id, 2U);
  EXPECT_FALSE(facade.hasStagedCommandCandidate());

  // A retired canonical pending heading must not block a fresh measured
  // emergency proposal. This is backend ownership, not an assertion that
  // runtime dispatches/adopts a brake or that a waypoint was accepted.
  navigation_planning::TrajectoryPoint measured_brake;
  measured_brake.position_world = state.position_world;
  measured_brake.velocity_world = Eigen::Vector3d{0.5, 0.0, 0.0};
  measured_brake.yaw = state.yaw_rad;
  const auto emergency_route =
      progress.snapshot(mission.id, mission.frame, 1U, 3U, 1U);
  auto emergency_request = plannerRequestForRoute(
      world, emergency_route, state, *semanticFixtureMissionLimits(), 3U,
      start.position_enu);
  emergency_request.key.start_mode =
      navigation_planning::PlanningStartMode::kMeasuredEmergencyBrake;
  emergency_request.key.committed_bundle_generation = initial->bundle_generation;
  emergency_request.start_state.velocity_world = measured_brake.velocity_world;
  emergency_request.history.previous_bundle =
      std::make_shared<const navigation_planning::CandidateBundle>(*initial);
  emergency_request.history.previous_bundle_generation =
      initial->bundle_generation;
  emergency_request.history.previous_velocity_world = initial->sampleAtDeclaredEnd()
      ? initial->sampleAtDeclaredEnd()->velocity_world
      : Eigen::Vector3d::Zero();
  ASSERT_TRUE(emergency_request.valid());
  const auto emergency_outcome = facade.plan(emergency_request);
  ASSERT_TRUE(emergency_outcome.valid())
      << static_cast<int>(emergency_outcome.failure_stage) << ":"
      << static_cast<int>(emergency_outcome.failure_reason);
  const auto emergency = emergency_outcome.candidate;
  ASSERT_TRUE(emergency);
  ASSERT_GT(emergency->bundle_generation, out_of_band->bundle_generation);
  EXPECT_TRUE(facade.hasStagedCommandCandidate());
  EXPECT_TRUE(facade.validateStagedCommandCandidate(
      world, 10.6, emergency->bundle_generation).valid);
  EXPECT_TRUE(facade.validateStagedCommandCandidate(
      world, 10.5, out_of_band->bundle_generation).valid);

  // The out-of-band builder also registers the exact internal candidate
  // generation. Activation must promote that same retained position/yaw
  // bundle; a later nominal solve cannot substitute another candidate. The
  // generic nominal discard path must not erase this reserved owner.
  facade.onExecutionTimelineActivated(out_of_band->bundle_generation);
  EXPECT_TRUE(facade.hasStagedCommandCandidate());
  EXPECT_TRUE(facade.validateStagedCommandCandidate(
      world, 10.6, emergency->bundle_generation).valid);
  facade.discardCommandCandidate();
  EXPECT_FALSE(facade.hasStagedCommandCandidate());
  // The nominal discard cannot erase a heading export still waiting for ACK.
  const auto waiting = facade.buildImmediateHeadingRebindCandidate(
      world, second_route, state.position_world, state.velocity_world,
      state.yaw_rad, start.position_enu, 20.0, 1U, 3U, 3U,
      20000000000LL, 30000000000LL);
  ASSERT_TRUE(waiting);
  facade.discardCommandCandidate();
  EXPECT_TRUE(facade.validateStagedCommandCandidate(
      world, 20.0, waiting->bundle_generation).valid);
  EXPECT_TRUE(facade.discardRetainedPositionHeadingCandidate(
      waiting->bundle_generation));
  EXPECT_EQ(facade.committedGeneration(), out_of_band->bundle_generation);
  const auto& rebound = *out_of_band;
  EXPECT_FALSE(rebound.route_boundary_event.has_value());
  navigation_planning::TrajectoryPoint retained_at_activation;
  navigation_planning::TrajectoryPoint retained_after_turn;
  navigation_planning::TrajectoryPoint before_turn;
  navigation_planning::TrajectoryPoint after_turn;
  ASSERT_TRUE(initial->evaluator(rebound.declared_start_ns, retained_at_activation));
  ASSERT_TRUE(initial->evaluator(rebound.declared_start_ns + 1000000000LL,
                                 retained_after_turn));
  ASSERT_TRUE(rebound.evaluator(rebound.declared_start_ns, before_turn));
  ASSERT_TRUE(rebound.evaluator(rebound.declared_start_ns + 1000000000LL,
                                 after_turn));
  EXPECT_NEAR(before_turn.position_world.x(), retained_at_activation.position_world.x(), 1.0e-9);
  EXPECT_NEAR(before_turn.position_world.y(), retained_at_activation.position_world.y(), 1.0e-9);
  EXPECT_NEAR(after_turn.position_world.x(), retained_after_turn.position_world.x(), 1.0e-9);
  EXPECT_NEAR(after_turn.position_world.y(), retained_after_turn.position_world.y(), 1.0e-9);
  EXPECT_GT(after_turn.yaw, before_turn.yaw + 1.0e-3);
  EXPECT_LE(std::abs(after_turn.yaw_rate), facade.yawRateLimitRadS() + 1.0e-6);

  // Use the already-settled yaw plateau of the long fixture trajectory, so
  // owner retirement is tested independently of a second turning transient.
  const auto unadmitted = facade.buildImmediateHeadingRebindCandidate(
      world, second_route, state.position_world, state.velocity_world,
      state.yaw_rad, start.position_enu, 20.0, 1U, 3U, 3U,
      20000000000LL, 30000000000LL);
  ASSERT_TRUE(unadmitted);
  ASSERT_GT(unadmitted->bundle_generation, rebound.bundle_generation);
  // A delayed retirement/ACK for the previous owner cannot clear this one.
  EXPECT_FALSE(facade.discardRetainedPositionHeadingCandidate(
      rebound.bundle_generation));
  facade.onExecutionTimelineActivated(rebound.bundle_generation);
  EXPECT_TRUE(facade.validateStagedCommandCandidate(
      world, 20.0, unadmitted->bundle_generation).valid);
  EXPECT_TRUE(facade.discardRetainedPositionHeadingCandidate(
      unadmitted->bundle_generation));
  EXPECT_FALSE(facade.hasStagedCommandCandidate());
  EXPECT_EQ(facade.committedGeneration(), rebound.bundle_generation);
  // No ghost retained slot blocks a subsequent desired heading export.
  const auto replacement = facade.buildImmediateHeadingRebindCandidate(
      world, second_route, state.position_world, state.velocity_world,
      state.yaw_rad, start.position_enu, 20.1, 1U, 4U, 4U,
      20100000000LL, 30000000000LL);
  ASSERT_TRUE(replacement);
  EXPECT_GT(replacement->bundle_generation, unadmitted->bundle_generation);
  // Model goal C canceling pending G before activation: no ACK G arrives.
  // Nominal C remains able to solve/export, and only its successful newer
  // activation collects the obsolete heading owner. No timeout/retry gate.
  ros_time_s = 20.2;
  const auto request_c = plannerRequestForRoute(
      world, progress.snapshot(mission.id, mission.frame, 1U, 5U, 1U),
      state, *semanticFixtureMissionLimits(), 5U, start.position_enu);
  const auto request_c_outcome = facade.plan(request_c);
  ASSERT_TRUE(request_c_outcome.valid())
      << static_cast<int>(request_c_outcome.failure_stage) << ":"
      << static_cast<int>(request_c_outcome.failure_reason);
  const auto nominal = request_c_outcome.candidate;
  ASSERT_TRUE(nominal);
  ASSERT_GT(nominal->bundle_generation, replacement->bundle_generation);
  EXPECT_TRUE(facade.validateStagedCommandCandidate(
      world, 20.1, replacement->bundle_generation).valid);
  facade.onExecutionTimelineActivated(nominal->bundle_generation + 1U);
  EXPECT_TRUE(facade.validateStagedCommandCandidate(
      world, 20.1, replacement->bundle_generation).valid);
  facade.onExecutionTimelineActivated(nominal->bundle_generation);
  EXPECT_EQ(facade.committedGeneration(), nominal->bundle_generation);
  EXPECT_FALSE(facade.validateStagedCommandCandidate(
      world, 20.1, replacement->bundle_generation).valid);
  EXPECT_FALSE(facade.discardRetainedPositionHeadingCandidate(
      replacement->bundle_generation));
  facade.onExecutionTimelineActivated(replacement->bundle_generation);
  EXPECT_EQ(facade.committedGeneration(), nominal->bundle_generation);
  ASSERT_GT(nominal->duration_s, 10.1);
  auto late_emergency_request = plannerRequestForRoute(
      world, request_c.route_snapshot, state, *semanticFixtureMissionLimits(), 5U,
      start.position_enu);
  late_emergency_request.key.start_mode =
      navigation_planning::PlanningStartMode::kMeasuredEmergencyBrake;
  late_emergency_request.key.committed_bundle_generation = nominal->bundle_generation;
  late_emergency_request.start_state.velocity_world = measured_brake.velocity_world;
  late_emergency_request.history.previous_bundle =
      std::make_shared<const navigation_planning::CandidateBundle>(*nominal);
  late_emergency_request.history.previous_bundle_generation =
      nominal->bundle_generation;
  late_emergency_request.history.previous_velocity_world =
      measured_brake.velocity_world;
  ros_time_s = 30.3;
  ASSERT_TRUE(late_emergency_request.valid());
  const auto late_emergency_outcome = facade.plan(late_emergency_request);
  ASSERT_TRUE(late_emergency_outcome.valid())
      << static_cast<int>(late_emergency_outcome.failure_stage) << ":"
      << static_cast<int>(late_emergency_outcome.failure_reason);
  const auto older_position = late_emergency_outcome.candidate;
  ASSERT_TRUE(older_position);
  const auto next_heading = facade.buildImmediateHeadingRebindCandidate(
      world, second_route, state.position_world, state.velocity_world,
      state.yaw_rad, start.position_enu, 30.2, 1U, 6U, 6U,
      30200000000LL, 80000000000LL);
  ASSERT_TRUE(next_heading);
  ASSERT_GT(next_heading->bundle_generation, older_position->bundle_generation);
  facade.onExecutionTimelineActivated(next_heading->bundle_generation);
  EXPECT_EQ(facade.committedGeneration(), next_heading->bundle_generation);
  // Reservation order need not match worker completion order. Do not destroy
  // an older position proposal before its job reaches export/admission gates.
  EXPECT_TRUE(facade.hasStagedCommandCandidate());
  const auto older_position_validation = facade.validateStagedCommandCandidate(
      world, 30.3, older_position->bundle_generation);
  EXPECT_TRUE(older_position_validation.valid)
      << "failure=" << older_position_validation.failure_code
      << " first_blocked=" << older_position_validation.first_blocked_time_s
      << " samples=" << older_position_validation.sample_count
      << " blocked_role=" << older_position_validation.blocked_role;
  EXPECT_EQ(older_position->bundle_generation,
            late_emergency_outcome.candidate->bundle_generation);
}

void probeProductPassRenewal(const bool backup_allow_unknown,
                             const bool require_measured_handoff_window = false) {
  // Preserve the product's bounded map geometry and the first 9WP junction,
  // without pretending this obstacle-free fixture replays the recorded map.
  class BoundedFreeWorld final : public IdentityOnlyWorld {
   public:
    navigation_world_model::WorldGeometry geometry() const noexcept override {
      auto result = IdentityOnlyWorld::geometry();
      result.evidence_bounds.global_min_index = Eigen::Vector3i{-50, -125, -5};
      result.inflated_bounds.global_min_index = result.evidence_bounds.global_min_index;
      return result;
    }
    navigation_world_model::WorldSnapshotIdentity identity() const noexcept override {
      return {1U, 1U, 1U, 10'000'000'000LL};
    }
    bool contains(const navigation_world_model::Point3& p) const noexcept override {
      return p.allFinite() && p.x() >= -10.0 && p.x() < 40.0 &&
          p.y() >= -25.0 && p.y() < 25.0 && p.z() >= -1.0 && p.z() < 7.0;
    }
    navigation_world_model::CellState classify(
        const navigation_world_model::Point3& p,
        navigation_world_model::GridLayer) const noexcept override {
      return contains(p) ? navigation_world_model::CellState::kKnownFree
                         : navigation_world_model::CellState::kOutOfMap;
    }
    bool isSegmentTraversable(
        const navigation_world_model::Point3& begin,
        const navigation_world_model::Point3& end,
        navigation_world_model::GridLayer,
        navigation_world_model::UnknownPolicy) const noexcept override {
      return contains(begin) && contains(end);
    }
    navigation_world_model::AxisAlignedBox clampToLocalBounds(
        const navigation_world_model::AxisAlignedBox& box) const noexcept override {
      return {box.minimum.cwiseMax(Eigen::Vector3d{-10.0, -25.0, -1.0}),
              box.maximum.cwiseMin(Eigen::Vector3d{40.0, 25.0, 7.0})};
    }
  };
  auto world = std::make_shared<const BoundedFreeWorld>();
  TestCommitAuthorizer authorizer(world);
  double ros_time_s = 10.0;
  navigation_planning::DynamicLimits limits;
  limits.intent.requested_cruise_speed_mps = 5.0;
  limits.unknown_space_policy = navigation_world_model::UnknownPolicy::kAllowUnknown;
  navigation_planning_backend::PlannerFacade facade(
      backup_allow_unknown ? PLANNER_FACADE_FAST_CONFIG_PATH : PLANNER_FACADE_CONFIG_PATH,
      world, limits, authorizer, [&ros_time_s] { return ros_time_s; });

  navigation_mission::Mission mission;
  mission.id = "product-pass-reserve";
  mission.frame = "lio_odom";
  mission.planning.requested_cruise_speed_mps = 5.0;
  mission.waypoints = {
      {"previous", Eigen::Vector3d{0.0, 0.0, 3.0}, 0.9, 0.0,
       navigation_mission::MissionWaypoint::Behavior::PassThrough},
      {"current", Eigen::Vector3d{20.0, 5.0, 3.0}, 0.9, 0.0,
       navigation_mission::MissionWaypoint::Behavior::PassThrough},
      {"next", Eigen::Vector3d{50.0, 5.0, 3.0}, 0.9, 0.0,
       navigation_mission::MissionWaypoint::Behavior::Stop}};
  navigation_mission::RouteProgress progress(mission);
  ASSERT_TRUE(progress.update(mission.waypoints.front().position_enu).valid);
  auto initial_request = plannerBodySupportRequest(world, nullptr);
  initial_request.route_snapshot = progress.snapshot(mission.id, mission.frame, 1U, 31U, 1U);
  initial_request.key.route_revision = initial_request.route_snapshot.route_revision;
  initial_request.goal.mission_id = mission.id;
  initial_request.start_state.position_world = mission.waypoints.front().position_enu;
  initial_request.dynamics = limits;
  initial_request.budget.deadline = navigation_planning::PlanningBudget::Clock::now() +
      std::chrono::milliseconds(80);
  initial_request.budget.steady_deadline_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
      initial_request.budget.deadline.time_since_epoch()).count();
  ASSERT_TRUE(initial_request.valid());
  const auto initial = facade.plan(initial_request);
  ASSERT_TRUE(initial.candidate.has_value())
      << static_cast<int>(initial.failure_stage) << ":"
      << static_cast<int>(initial.failure_reason);
  auto predecessor = *initial.candidate;
  ASSERT_TRUE(predecessor.valid());
  facade.onExecutionTimelineActivated(predecessor.bundle_generation);
  int crossing_proposals = 0;
  int measured_window_proposals = 0;
  std::int64_t minimum_observed_reserve_ns = std::numeric_limits<std::int64_t>::max();
  const auto check_boundary = [&](const navigation_planning::CandidateBundle& candidate) {
    if (!candidate.route_boundary_event || candidate.route_boundary_event->kind !=
        navigation_planning::RouteBoundaryEventKind::kPassThrough) return true;
    ++crossing_proposals;
    const auto offset_ns = candidate.route_boundary_event->boundary_stamp_ns -
        candidate.declared_start_ns;
    bool main_interval_found = false;
    bool reserve_valid = false;
    std::int64_t main_end_stamp_ns = 0;
    for (const auto& interval : candidate.role_schedule) {
      if (interval.role != navigation_planning::CandidateRole::kMain) continue;
      const auto begin_ns = static_cast<std::int64_t>(std::llround(interval.begin_time_s * 1.0e9));
      const auto end_ns = static_cast<std::int64_t>(std::llround(interval.end_time_s * 1.0e9));
      if (offset_ns < begin_ns || offset_ns >= end_ns) continue;
      main_interval_found = true;
      main_end_stamp_ns = candidate.declared_start_ns + end_ns;
      const auto reserve_ns = end_ns - offset_ns;
      minimum_observed_reserve_ns = std::min(minimum_observed_reserve_ns, reserve_ns);
      // Independent arithmetic on the exported canonical schedule, not an
      // invocation of the producer's eligibility predicate under test.
      reserve_valid = reserve_ns >= static_cast<std::int64_t>(std::llround(
          navigation_planning::PlanningTimingContract::kMinimumMainReserveS * 1.0e9));
    }
    EXPECT_TRUE(main_interval_found);
    // Exercise the shared measured/ordered sphere predicate independently
    // of the producer's AABB event. Ideal measurements are candidate samples,
    // not flight evidence; no callback/DDS delay or tracking error is assumed.
    navigation_mission::RouteProgress measured_progress(mission);
    std::optional<Eigen::Vector3d> previous_position;
    int measured_handoff_samples = 0;
    constexpr std::int64_t sample_period_ns = 20'000'000LL;
    const auto reserve_ns = static_cast<std::int64_t>(std::llround(
        navigation_planning::PlanningTimingContract::kMinimumMainReserveS * 1.0e9));
    for (auto stamp_ns = candidate.declared_start_ns;
         main_interval_found && stamp_ns < main_end_stamp_ns;
         stamp_ns += sample_period_ns) {
      const auto point = candidate.sampleAtDeclaredStamp(stamp_ns);
      EXPECT_TRUE(point.has_value());
      if (!point) break;
      (void)measured_progress.update(point->position_world);
      const auto crossing = measured_progress.measuredWaypointCrossingError(
          1U, point->position_world, previous_position, 0.02, 0.25);
      if (reserve_valid && point->role == navigation_planning::CandidateRole::kMain &&
          main_end_stamp_ns - stamp_ns >= reserve_ns && crossing.has_value()) {
        ++measured_handoff_samples;
      }
      previous_position = point->position_world;
    }
    ::testing::Test::RecordProperty("measured_handoff_samples_at_last_crossing",
                                    measured_handoff_samples);
    if (require_measured_handoff_window) {
      EXPECT_TRUE(reserve_valid);
    }
    if (measured_handoff_samples > 0) ++measured_window_proposals;
    return main_interval_found && reserve_valid;
  };
  ASSERT_TRUE(check_boundary(predecessor));
  int failed_successor_count = 0;
  int successful_successor_count = 0;
  int reserve_rejected_proposals = 0;
  for (int step = 0; step < 16; ++step) {
    SCOPED_TRACE(step);
    const auto main_end_ns = predecessor.declared_start_ns +
        static_cast<std::int64_t>(std::llround(predecessor.backup_start_time_s * 1.0e9));
    const auto activation_offset_ns = std::min<std::int64_t>(
        1'000'000'000LL, (main_end_ns - predecessor.declared_start_ns) / 2);
    const auto activation_ns = predecessor.declared_start_ns +
        activation_offset_ns;
    const auto measured_ns = activation_ns - 400'000'000LL;
    ASSERT_GE(measured_ns, predecessor.declared_start_ns);
    const auto measured = predecessor.sampleAtDeclaredStamp(measured_ns);
    const auto future = predecessor.sampleAtDeclaredStamp(activation_ns);
    ASSERT_TRUE(measured.has_value());
    ASSERT_TRUE(future.has_value());
    ASSERT_EQ(future->role, navigation_planning::CandidateRole::kMain);
    // Stop this same-identity construction probe once ideal measured motion
    // reaches the ball. Only MissionController may advance the real route.
    if ((measured->position_world - mission.waypoints[1].position_enu).norm() <= 0.9) break;
    auto request = initial_request;
    request.key.start_mode = navigation_planning::PlanningStartMode::kCommittedFutureState;
    request.key.committed_bundle_generation = predecessor.bundle_generation;
    request.key.anchor_stamp_ns = measured_ns;
    request.start_state.source_stamp_ns = measured_ns;
    request.start_state.receive_stamp_ns = measured_ns;
    request.start_state.position_world = measured->position_world;
    request.start_state.velocity_world = measured->velocity_world;
    request.start_state.acceleration_world = measured->acceleration_world;
    request.start_state.jerk_world = measured->jerk_world;
    request.activation_stamp_ns = activation_ns;
    request.history.previous_bundle_generation = predecessor.bundle_generation;
    request.history.previous_velocity_world = future->velocity_world;
    request.anchor = navigation_planning::ExecutionAnchor{
        predecessor.bundle_generation, 1U, 1U, 1U, 31U, measured_ns, activation_ns,
        *future, future->role, main_end_ns, predecessor.declared_end_ns,
        predecessor.world_identity};
    request.budget.deadline = navigation_planning::PlanningBudget::Clock::now() +
        std::chrono::milliseconds(80);
    request.budget.steady_deadline_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
        request.budget.deadline.time_since_epoch()).count();
    ASSERT_TRUE(request.valid());
    ros_time_s = static_cast<double>(measured_ns) * 1.0e-9;
    const auto successor = facade.plan(request);
    EXPECT_EQ(facade.committedGeneration(), predecessor.bundle_generation);
    if (!successor.candidate) {
      const auto diagnostics = facade.diagnostics();
      ::testing::Test::RecordProperty(
          "failed_successor_step_" + std::to_string(step),
          "stage=" + std::to_string(static_cast<int>(successor.failure_stage)) +
              " reason=" + std::to_string(static_cast<int>(successor.failure_reason)) +
              " remaining_budget_us=" +
              std::to_string(diagnostics.timeline.remaining_hard_budget_us_at_finish) +
              " outgoing_geometry=" +
              std::to_string(navigation_planning_backend::outgoingGoalAtUnacceptedMainBoundary(
                                 request).has_value()) +
              " head_distance_m=" +
              std::to_string((future->position_world - mission.waypoints[1].position_enu).norm()));
      ++failed_successor_count;
      if (failed_successor_count >= 3) break;
      continue;
    }
    ::testing::Test::RecordProperty("last_anchor_position_x_m", std::to_string(future->position_world.x()));
    ::testing::Test::RecordProperty("last_anchor_speed_mps", std::to_string(future->velocity_world.norm()));
    if (!check_boundary(*successor.candidate)) {
      // Diagnostic reference-admission arithmetic only. The real runtime
      // owns authority; do not activate a staged crossing that it rejects.
      ++reserve_rejected_proposals;
      EXPECT_EQ(facade.committedGeneration(), predecessor.bundle_generation);
      break;
    }
    ++successful_successor_count;
    ros_time_s = static_cast<double>(activation_ns) * 1.0e-9;
    facade.onExecutionTimelineActivated(successor.candidate->bundle_generation);
    predecessor = *successor.candidate;
  }
  ::testing::Test::RecordProperty("failed_successor_count", failed_successor_count);
  ::testing::Test::RecordProperty("successful_successor_count", successful_successor_count);
  ::testing::Test::RecordProperty("reserve_rejected_proposal_count", reserve_rejected_proposals);
  ::testing::Test::RecordProperty("minimum_post_pass_main_reserve_ns", std::to_string(minimum_observed_reserve_ns));
  ::testing::Test::RecordProperty("observed_pass_crossing_proposal_count", crossing_proposals);
  EXPECT_GT(crossing_proposals, 0);
  ::testing::Test::RecordProperty("observed_measured_window_proposal_count", measured_window_proposals);
  // Shorter admission-valid crossing fallbacks remain allowed. This bounded
  // construction sequence must eventually expose an ideal measured window;
  // it does not require every earlier receding-horizon bundle to have one.
  if (require_measured_handoff_window) EXPECT_GT(measured_window_proposals, 0);
  EXPECT_EQ(facade.committedGeneration(), predecessor.bundle_generation);
}

TEST(PlannerFacade, SafeProductPassRenewalProbePreservesActivatedPredecessor) {
  probeProductPassRenewal(false);
}

TEST(PlannerFacade, FastProductPassRenewalProbePreservesActivatedPredecessor) {
  probeProductPassRenewal(true);
}

TEST(PlannerFacade, SafeProductCrossingHasIdealMeasuredHandoffWindow) {
  probeProductPassRenewal(false, true);
}

TEST(PlannerFacade, FastProductCrossingHasIdealMeasuredHandoffWindow) {
  probeProductPassRenewal(true, true);
}

TEST(PlannerFacade, CoincidentPassToStopPreservesCertifiedTerminalBackup) {
  auto world = std::make_shared<IdentityOnlyWorld>();
  TestCommitAuthorizer authorizer(world);
  navigation_planning::DynamicLimits limits;
  limits.intent.requested_cruise_speed_mps = 5.0;
  limits.unknown_space_policy = navigation_world_model::UnknownPolicy::kAllowUnknown;
  navigation_planning_backend::PlannerFacade facade(
      PLANNER_FACADE_CONFIG_PATH, world, limits, authorizer, [] { return 10.0; });
  navigation_mission::Mission mission;
  mission.id = "coincident-pass-stop-backup";
  mission.frame = "lio_odom";
  mission.planning.requested_cruise_speed_mps = 5.0;
  mission.waypoints = {
      {"previous", Eigen::Vector3d{0.0, 0.0, 2.0}, 0.8, 0.0,
       navigation_mission::MissionWaypoint::Behavior::PassThrough},
      {"current", Eigen::Vector3d{5.0, 0.0, 2.0}, 0.8, 0.0,
       navigation_mission::MissionWaypoint::Behavior::PassThrough},
      {"terminal", Eigen::Vector3d{5.0, 0.0, 2.0}, 0.8, 0.0,
       navigation_mission::MissionWaypoint::Behavior::Stop}};
  navigation_mission::RouteProgress progress(mission);
  ASSERT_TRUE(progress.update(mission.waypoints.front().position_enu).valid);
  auto request = plannerBodySupportRequest(world, nullptr);
  request.route_snapshot = progress.snapshot(mission.id, mission.frame, 1U, 31U, 1U);
  request.key.route_revision = request.route_snapshot.route_revision;
  request.goal.mission_id = mission.id;
  request.dynamics = limits;
  ASSERT_TRUE(navigation_mission::passThroughNextWaypointIsCoincidentStop(request.route_snapshot));
  ASSERT_TRUE(request.valid());
  const auto result = facade.plan(request);
  ASSERT_TRUE(result.candidate) << static_cast<int>(result.failure_stage) << ":"
                                << static_cast<int>(result.failure_reason);
  const auto& candidate = *result.candidate;
  ASSERT_TRUE(candidate.valid());
  EXPECT_TRUE(candidate.terminal_stop);
  // Start is 5m from the goal, beyond the main-only/rest radius shortcut.
  EXPECT_TRUE(candidate.backup_available);
  const auto endpoint = candidate.sampleAtDeclaredEnd();
  ASSERT_TRUE(endpoint);
  EXPECT_LE((endpoint->position_world - mission.waypoints[1].position_enu).norm(), 0.8);
  EXPECT_NEAR(endpoint->velocity_world.norm(), 0.0, 1.0e-6);
  EXPECT_EQ(facade.committedGeneration(), 0U);
}

TEST(PlannerFacade, PassThroughLookaheadExportsRouteBoundaryEvent) {
  auto world = std::make_shared<IdentityOnlyWorld>();
  TestCommitAuthorizer authorizer(world);
  navigation_planning_backend::PlannerFacade facade(
      PLANNER_FACADE_CONFIG_PATH, world, semanticFixtureMissionLimits(), authorizer,
      [] { return 10.0; });

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

  navigation_planning::KinematicState state;
  state.position_world = Eigen::Vector3d{0.0, 0.0, 3.0};
  state.source_stamp_ns = 10000000000LL;
  state.receive_stamp_ns = state.source_stamp_ns;
  state.localization_epoch = 1U;
  state.world_frame_id = "lio_odom";
  state.body_frame_id = "base_link";
  const auto outcome = facade.plan(plannerRequestForRoute(
      world, route, state, *semanticFixtureMissionLimits(), 1U));
  ASSERT_TRUE(outcome.valid()) << static_cast<int>(outcome.failure_stage) << ":"
                               << static_cast<int>(outcome.failure_reason);
  const auto candidate = outcome.candidate;
  ASSERT_TRUE(candidate);
  EXPECT_EQ(candidate->activation_stamp_ns, 10000000000LL);
  EXPECT_EQ(candidate->valid_from_ns, 10000000000LL);
  ASSERT_TRUE(candidate->route_boundary_constraint.has_value());
  ASSERT_TRUE(candidate->route_boundary_event.has_value());
  EXPECT_EQ(candidate->route_boundary_event->kind,
            navigation_planning::RouteBoundaryEventKind::kPassThrough);
  EXPECT_EQ(candidate->route_boundary_event->junction_index, 0U);
  EXPECT_TRUE(candidate->route_boundary_constraint->contains(
      candidate->route_boundary_event->position_world));
}

TEST(PlannerFacade, GenuineNinetyDegreePassEventUsesActualMissionSphere) {
  // This is a semantic geometry regression, not requested-5-m/s qualification
  // or a proof of a measured handoff window. Rotating a genuine 90-degree
  // corner makes the circumscribed AABB distinct from the mission sphere.
  // Keep the real facade, product budget, and complete MAIN+BACKUP gates.
  auto world = std::make_shared<IdentityOnlyWorld>();
  TestCommitAuthorizer authorizer(world);
  const auto limits = semanticFixtureMissionLimits();
  ASSERT_TRUE(limits.has_value());
  navigation_planning_backend::PlannerFacade facade(
      PLANNER_FACADE_CONFIG_PATH, world, limits, authorizer, [] { return 10.0; });

  navigation_mission::Mission mission;
  mission.id = "genuine-ninety-degree-sphere-event";
  mission.frame = "lio_odom";
  mission.planning.requested_cruise_speed_mps = limits->intent.requested_cruise_speed_mps;
  mission.waypoints = {
      {"previous", Eigen::Vector3d{0.0, 0.0, 3.0}, 0.9, 0.0,
       navigation_mission::MissionWaypoint::Behavior::PassThrough},
      {"current", Eigen::Vector3d{5.0, 5.0, 3.0}, 0.9, 0.0,
       navigation_mission::MissionWaypoint::Behavior::PassThrough},
      {"next", Eigen::Vector3d{0.0, 10.0, 3.0}, 0.9, 0.0,
       navigation_mission::MissionWaypoint::Behavior::Stop}};
  const auto& active = mission.waypoints[1U];
  const Eigen::Vector3d measured_position{2.0, 2.0, 3.0};
  const Eigen::Vector3d incoming =
      (active.position_enu - mission.waypoints[0U].position_enu).normalized();
  const Eigen::Vector3d outgoing =
      (mission.waypoints[2U].position_enu - active.position_enu).normalized();
  ASSERT_DOUBLE_EQ(incoming.dot(outgoing), 0.0);
  ASSERT_GT((measured_position - active.position_enu).norm(), active.acceptance_radius_m);

  navigation_mission::RouteProgress progress(mission);
  ASSERT_TRUE(progress.update(mission.waypoints[0U].position_enu).valid);
  ASSERT_TRUE(progress.update(measured_position).valid);
  auto request = plannerBodySupportRequest(world, nullptr);
  request.route_snapshot = progress.snapshot(mission.id, mission.frame, 1U, 31U, 1U);
  request.key.route_revision = request.route_snapshot.route_revision;
  request.goal.mission_id = mission.id;
  request.start_state.position_world = measured_position;
  request.dynamics = *limits;
  request.budget.deadline = navigation_planning::PlanningBudget::Clock::now() +
      std::chrono::duration_cast<navigation_planning::PlanningBudget::Clock::duration>(
          std::chrono::duration<double>(
              navigation_planning::PlanningTimingContract::kSolveDeadlineS));
  request.budget.steady_deadline_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
      request.budget.deadline.time_since_epoch()).count();
  ASSERT_TRUE(request.valid());
  ASSERT_EQ(request.route_snapshot.active_waypoint_index, 1U);

  const auto result = facade.plan(request);
  ASSERT_TRUE(result.candidate.has_value()) << static_cast<int>(result.failure_stage) << ":"
                                          << static_cast<int>(result.failure_reason);
  const auto& candidate = *result.candidate;
  ASSERT_TRUE(candidate.valid());
  ASSERT_EQ(candidate.kind, navigation_planning::CandidateBundleKind::kMainWithBackup);
  ASSERT_TRUE(candidate.certificates.completeFor(candidate.kind));
  ASSERT_TRUE(candidate.backup_available);
  EXPECT_FALSE(candidate.terminal_stop);
  EXPECT_EQ(candidate.request_id, request.key.request_id);
  EXPECT_EQ(candidate.goal_epoch, request.key.goal_epoch);
  EXPECT_EQ(candidate.localization_epoch, request.key.localization_epoch);
  ASSERT_TRUE(candidate.route_boundary_constraint.has_value());
  ASSERT_TRUE(candidate.route_boundary_event.has_value());
  const auto& event = *candidate.route_boundary_event;
  ASSERT_EQ(event.kind, navigation_planning::RouteBoundaryEventKind::kPassThrough);
  EXPECT_EQ(event.junction_index, 1U);
  EXPECT_EQ(candidate.route_boundary_constraint->junction_index, 1U);
  EXPECT_LE(event.incoming_tangent.dot(event.outgoing_tangent), 0.7);
  EXPECT_TRUE(candidate.route_boundary_constraint->contains(event.position_world));

  // Independent Euclidean oracle: do not call the producer's visit helper or
  // mistake its outer box for the configured mission acceptance sphere.
  const double event_error_m = (event.position_world - active.position_enu).norm();
  ::testing::Test::RecordProperty("corner_event_sphere_error_m", std::to_string(event_error_m));
  EXPECT_LE(event_error_m, active.acceptance_radius_m);
  const auto event_sample = candidate.sampleAtDeclaredStamp(event.boundary_stamp_ns);
  ASSERT_TRUE(event_sample.has_value());
  EXPECT_EQ(event_sample->role, navigation_planning::CandidateRole::kMain);
  const double sampled_event_error_m =
      (event_sample->position_world - active.position_enu).norm();
  ::testing::Test::RecordProperty("corner_stamped_event_sphere_error_m",
                                  std::to_string(sampled_event_error_m));
  EXPECT_LE(sampled_event_error_m, active.acceptance_radius_m);

  const auto main_end_ns = candidate.declared_start_ns +
      static_cast<std::int64_t>(std::llround(candidate.backup_start_time_s * 1.0e9));
  const auto reserve_ns = static_cast<std::int64_t>(std::llround(
      navigation_planning::PlanningTimingContract::kMinimumMainReserveS * 1.0e9));
  EXPECT_GE(main_end_ns - event.boundary_stamp_ns, reserve_ns);
  // No mission acceptance is granted by this planned event. Requiring a
  // measured callback to overlap the ready window is a separate liveness test.
  EXPECT_EQ(request.route_snapshot.active_waypoint_index, 1U);
  EXPECT_EQ(facade.committedGeneration(), 0U);
}

void expectFutureAnchorInsideUnacceptedPassBoundaryCanRenew(const double speed_mps) {
  const ScopedSnapshotDirectory directory;
  const ScopedSnapshotEnvironment capture("UAV_NAVIGATION_NOMINAL_SNAPSHOT_DIR",
                                          directory.path().string());
  const ScopedSnapshotEnvironment failure_only("UAV_NAVIGATION_NOMINAL_SNAPSHOT_FAILURE_ONLY", "0");
  const ScopedSnapshotEnvironment include_world("UAV_NAVIGATION_NOMINAL_SNAPSHOT_INCLUDE_WORLD",
                                                "0");
  std::int64_t activation_ns = 0;
  double grid_resolution_m = 0.0;
  {
    auto world = std::make_shared<IdentityOnlyWorld>();
    grid_resolution_m = world->geometry().evidence_resolution_m;
    TestCommitAuthorizer authorizer(world);
    double ros_time_s = 10.0;
    auto limits = semanticFixtureMissionLimits();
    limits->intent.requested_cruise_speed_mps = speed_mps;
    navigation_planning_backend::PlannerFacade facade(PLANNER_FACADE_CONFIG_PATH, world, limits,
                                                      authorizer,
                                                      [&ros_time_s] { return ros_time_s; });

    // Keep the measured mission boundary active while a bounded future sample
    // has already entered its ball. This is a same-identity renewal, not a
    // measured mission handoff or permission to relabel the predecessor.
    navigation_mission::Mission mission;
    mission.id = "future-anchor-unaccepted-boundary";
    mission.frame = "lio_odom";
    mission.planning.requested_cruise_speed_mps = speed_mps;
    mission.waypoints = {
        navigation_mission::MissionWaypoint{
            "previous", Eigen::Vector3d{-10.0, 0.0, 3.0}, 0.8, 0.0,
            navigation_mission::MissionWaypoint::Behavior::PassThrough},
        navigation_mission::MissionWaypoint{
            "current", Eigen::Vector3d{0.0, 0.0, 3.0}, 0.8, 0.0,
            navigation_mission::MissionWaypoint::Behavior::PassThrough},
        navigation_mission::MissionWaypoint{"next", Eigen::Vector3d{10.0, 0.0, 3.0}, 0.8, 0.0,
                                            navigation_mission::MissionWaypoint::Behavior::Stop}};
    navigation_mission::RouteProgress progress(mission);
    ASSERT_TRUE(progress.update(mission.waypoints.front().position_enu).valid);
    auto request = plannerBodySupportRequest(world, nullptr);
    request.route_snapshot = progress.snapshot(mission.id, mission.frame, 1U, 31U, 1U);
    request.key.route_revision = request.route_snapshot.route_revision;
    request.goal.mission_id = mission.id;
    request.start_state.position_world = mission.waypoints.front().position_enu;
    request.dynamics.intent.requested_cruise_speed_mps = speed_mps;
    request.goal_acceptance_radius_m = mission.waypoints[1].acceptance_radius_m;
    ASSERT_TRUE(request.valid());
    const auto initial = facade.plan(request);
    ASSERT_TRUE(initial.candidate.has_value()) << static_cast<int>(initial.failure_stage) << ":"
                                               << static_cast<int>(initial.failure_reason);
    const auto& committed = *initial.candidate;
    ASSERT_TRUE(committed.valid());
    ASSERT_TRUE(committed.route_boundary_event.has_value());
    facade.onExecutionTimelineActivated(committed.bundle_generation);

    const auto main_end_ns =
        committed.declared_start_ns +
        static_cast<std::int64_t>(std::llround(
            (committed.backup_available ? committed.backup_start_time_s : committed.duration_s) *
            1.0e9));
    activation_ns = committed.route_boundary_event->boundary_stamp_ns + 50'000'000LL;
    ASSERT_LT(activation_ns + 600'000'000LL, main_end_ns);
    const auto measured_ns = activation_ns - 400'000'000LL;
    const auto measured = committed.sampleAtDeclaredStamp(measured_ns);
    const auto future = committed.sampleAtDeclaredStamp(activation_ns);
    ASSERT_TRUE(measured.has_value());
    ASSERT_TRUE(future.has_value());
    ASSERT_GT((measured->position_world - mission.waypoints[1].position_enu).norm(), 0.8);
    ASSERT_LT((future->position_world - mission.waypoints[1].position_enu).norm(), 0.8);
    ASSERT_GT(future->velocity_world.x(), 0.0);
    if (speed_mps == 5.0) {
      // Keep the cruise regression moving fast enough to exercise the return
      // fold, rather than silently retaining only the slow-fixture coverage.
      ASSERT_GT(future->velocity_world.x(), 4.0);
    }

    request.key.start_mode = navigation_planning::PlanningStartMode::kCommittedFutureState;
    request.key.committed_bundle_generation = committed.bundle_generation;
    request.key.anchor_stamp_ns = measured_ns;
    request.start_state.source_stamp_ns = measured_ns;
    request.start_state.receive_stamp_ns = measured_ns;
    request.start_state.position_world = measured->position_world;
    request.start_state.velocity_world = measured->velocity_world;
    request.start_state.acceleration_world = measured->acceleration_world;
    request.start_state.jerk_world = measured->jerk_world;
    request.activation_stamp_ns = activation_ns;
    request.history.previous_bundle_generation = committed.bundle_generation;
    request.history.previous_velocity_world = future->velocity_world;
    request.anchor = navigation_planning::ExecutionAnchor{committed.bundle_generation,
                                                          1U,
                                                          1U,
                                                          1U,
                                                          31U,
                                                          measured_ns,
                                                          activation_ns,
                                                          *future,
                                                          future->role,
                                                          main_end_ns,
                                                          committed.declared_end_ns,
                                                          committed.world_identity};
    request.budget.deadline =
        navigation_planning::PlanningBudget::Clock::now() + std::chrono::seconds(10);
    ASSERT_TRUE(request.valid());
    ros_time_s = static_cast<double>(measured_ns) * 1.0e-9;

    const auto successor = facade.plan(request);
    EXPECT_TRUE(successor.candidate.has_value()) << static_cast<int>(successor.failure_stage) << ":"
                                                 << static_cast<int>(successor.failure_reason);
    if (successor.candidate) {
      const auto& candidate = *successor.candidate;
      ASSERT_TRUE(candidate.valid());
      EXPECT_EQ(candidate.request_id, committed.request_id);
      EXPECT_EQ(candidate.goal_epoch, committed.goal_epoch);
      EXPECT_EQ(candidate.localization_epoch, committed.localization_epoch);
      EXPECT_EQ(candidate.activation_stamp_ns, activation_ns);
      EXPECT_EQ(request.route_snapshot.active_waypoint_index, 1U);
      ASSERT_TRUE(candidate.backup_available);
      EXPECT_GE(candidate.backup_start_time_s,
                navigation_planning::PlanningTimingContract::kMinimumMainReserveS);
      const auto head = candidate.sampleAtDeclaredStamp(activation_ns);
      ASSERT_TRUE(head);
      EXPECT_LE((head->position_world - future->position_world).norm(), 1.0e-8);
      EXPECT_LE((head->velocity_world - future->velocity_world).norm(), 1.0e-8);
      EXPECT_LE((head->acceleration_world - future->acceleration_world).norm(), 1.0e-8);
      EXPECT_LE((head->jerk_world - future->jerk_world).norm(), 1.0e-8);
      ASSERT_TRUE(candidate.route_boundary_event.has_value());
      ASSERT_TRUE(candidate.route_boundary_constraint.has_value());
      EXPECT_EQ(candidate.route_boundary_constraint->junction_index, 1U);
      EXPECT_EQ(candidate.route_boundary_event->junction_index, 1U);
      EXPECT_EQ(candidate.route_boundary_event->boundary_stamp_ns, activation_ns);
      const auto next_sample = candidate.sampleAtDeclaredStamp(activation_ns + 200'000'000LL);
      ASSERT_TRUE(next_sample.has_value());
      EXPECT_GT(next_sample->position_world.x(), future->position_world.x());
    }
  }  // Drain the existing diagnostic writer before checking the producer guide.
  const auto accounting =
      YAML::LoadFile((directory.path() / "nominal_problem_snapshot_capture.json").string());
  EXPECT_EQ(accounting["dropped_records"].as<std::uint64_t>(), 0U);
  EXPECT_EQ(accounting["write_error_count"].as<std::uint64_t>(), 0U);
  EXPECT_EQ(accounting["pending_records"].as<std::uint64_t>(), 0U);
  std::size_t matching_snapshots = 0U;
  for (const auto& entry : std::filesystem::directory_iterator(directory.path())) {
    if (entry.path().filename() == "nominal_problem_snapshot_capture.json" ||
        entry.path().extension() != ".json")
      continue;
    const auto snapshot = YAML::LoadFile(entry.path().string());
    if (snapshot["provenance"]["activation_stamp_ns"].as<std::int64_t>() != activation_ns) continue;
    ++matching_snapshots;
    const auto guide = snapshot["problem"]["guide_path"];
    ASSERT_TRUE(guide.IsSequence());
    ASSERT_GT(guide.size(), 1U);
    double maximum_x = guide[0U][0U][0U].as<double>();
    for (std::size_t index = 1U; index < guide.size(); ++index) {
      const double x = guide[index][0U][0U].as<double>();
      // A* starts at its voxel centre, which may lie slightly behind the
      // retained join. Reject a route-scale return, not that one-cell
      // representation offset in this straight all-free fixture.
      EXPECT_GE(x, maximum_x - grid_resolution_m) << "guide index=" << index;
      maximum_x = std::max(maximum_x, x);
    }
  }
  EXPECT_EQ(matching_snapshots, 1U);
}

TEST(PlannerFacade, FutureAnchorInsideUnacceptedPassBoundaryCanRenew) {
  expectFutureAnchorInsideUnacceptedPassBoundaryCanRenew(0.5624988750005627);
}

TEST(PlannerFacade, CruiseFutureAnchorDoesNotReturnToUnacceptedPassBoundary) {
  expectFutureAnchorInsideUnacceptedPassBoundaryCanRenew(5.0);
}

navigation_planning::PlanningRequest unacceptedBoundaryGeometryRequest(
    const Eigen::Vector3d& outgoing = Eigen::Vector3d{10.0, 0.0, 2.0}) {
  auto world = std::make_shared<IdentityOnlyWorld>();
  auto request = plannerBodySupportRequest(world, nullptr);
  navigation_mission::Mission mission;
  mission.id = request.goal.mission_id;
  mission.frame = "lio_odom";
  mission.planning.requested_cruise_speed_mps = 5.0;
  mission.waypoints = {
      {"previous", Eigen::Vector3d{-10.0, 0.0, 2.0}, 0.8, 0.0,
       navigation_mission::MissionWaypoint::Behavior::PassThrough},
      {"active", Eigen::Vector3d{0.0, 0.0, 2.0}, 0.8, 0.0,
       navigation_mission::MissionWaypoint::Behavior::PassThrough},
      {"outgoing", outgoing, 0.8, 0.0, navigation_mission::MissionWaypoint::Behavior::Stop}};
  navigation_mission::RouteProgress progress(mission);
  EXPECT_TRUE(progress.update(mission.waypoints.front().position_enu).valid);
  request.route_snapshot = progress.snapshot(
      mission.id, mission.frame, request.key.localization_epoch, request.key.request_id, 1U);
  request.key.route_revision = request.route_snapshot.route_revision;
  request.key.start_mode = navigation_planning::PlanningStartMode::kCommittedFutureState;
  request.key.committed_bundle_generation = 1U;
  request.key.anchor_stamp_ns = 10'000'000'000LL;
  request.start_state.source_stamp_ns = request.key.anchor_stamp_ns;
  request.start_state.receive_stamp_ns = request.key.anchor_stamp_ns;
  request.start_state.position_world = Eigen::Vector3d{-2.0, 0.0, 2.0};
  request.activation_stamp_ns = request.key.anchor_stamp_ns + 400'000'000LL;
  navigation_planning::TrajectoryPoint head;
  head.position_world = Eigen::Vector3d{-0.5, 0.0, 2.0};
  head.velocity_world = Eigen::Vector3d{5.0, 0.0, 0.0};
  head.role = navigation_planning::CandidateRole::kMain;
  request.anchor = navigation_planning::ExecutionAnchor{1U,
                                                        1U,
                                                        request.key.localization_epoch,
                                                        request.key.goal_epoch,
                                                        request.key.request_id,
                                                        request.key.anchor_stamp_ns,
                                                        request.activation_stamp_ns,
                                                        head,
                                                        head.role,
                                                        20'000'000'000LL,
                                                        22'000'000'000LL,
                                                        world->identity()};
  request.dynamics.intent.requested_cruise_speed_mps = 5.0;
  EXPECT_TRUE(request.valid());
  return request;
}

TEST(PlannerBoundaryGeometry, OutgoingTargetIsNotMissionAcceptance) {
  const auto request = unacceptedBoundaryGeometryRequest();
  const auto target = navigation_planning_backend::outgoingGoalAtUnacceptedMainBoundary(request);
  ASSERT_TRUE(target);
  EXPECT_EQ(*target, request.route_snapshot.waypoints[2U].position_enu);
  EXPECT_EQ(request.goal.waypoint_index, 1U);
  EXPECT_EQ(request.route_snapshot.active_waypoint_index, 1U);
  EXPECT_EQ(request.key.request_id, 31U);
  EXPECT_GT((request.start_state.position_world - request.route_snapshot.waypoints[1U].position_enu)
                .norm(),
            0.8);
}

TEST(PlannerBoundaryGeometry, UsesActualSphereNotExportAabbOrLaterRouteProximity) {
  auto request = unacceptedBoundaryGeometryRequest();
  request.anchor->state.position_world = Eigen::Vector3d{0.7, 0.7, 2.0};
  ASSERT_TRUE(request.valid());  // Inside the export AABB, outside the sphere.
  EXPECT_FALSE(navigation_planning_backend::outgoingGoalAtUnacceptedMainBoundary(request));
  request.anchor->state.position_world = request.route_snapshot.waypoints[2U].position_enu;
  ASSERT_TRUE(request.valid());
  EXPECT_FALSE(navigation_planning_backend::outgoingGoalAtUnacceptedMainBoundary(request));
}

TEST(PlannerBoundaryGeometry, DoesNotApplyToHandoffRecoveryOrSafetyRoles) {
  const auto base = unacceptedBoundaryGeometryRequest();
  for (const auto role : {navigation_planning::CandidateRole::kBackup,
                          navigation_planning::CandidateRole::kEmergency}) {
    auto request = base;
    request.anchor->active_role = role;
    request.anchor->state.role = role;
    ASSERT_TRUE(request.valid());
    EXPECT_FALSE(navigation_planning_backend::outgoingGoalAtUnacceptedMainBoundary(request));
  }
  for (const bool change_goal : {false, true}) {
    auto request = base;
    if (change_goal)
      ++request.anchor->goal_epoch;
    else
      ++request.anchor->request_id;
    ASSERT_TRUE(request.valid());  // Authorized handoff is a distinct protocol.
    EXPECT_FALSE(navigation_planning_backend::outgoingGoalAtUnacceptedMainBoundary(request));
  }
  auto stopped = base;
  stopped.key.start_mode = navigation_planning::PlanningStartMode::kStoppedMeasuredState;
  stopped.key.committed_bundle_generation = 0U;
  stopped.anchor.reset();
  stopped.activation_stamp_ns = 0;
  ASSERT_TRUE(stopped.valid());
  EXPECT_FALSE(navigation_planning_backend::outgoingGoalAtUnacceptedMainBoundary(stopped));
}

TEST(PlannerBoundaryGeometry, PreservesCornerCoincidentStopAndWorldContracts) {
  const auto corner = unacceptedBoundaryGeometryRequest(Eigen::Vector3d{0.0, 10.0, 2.0});
  EXPECT_FALSE(navigation_planning_backend::outgoingGoalAtUnacceptedMainBoundary(corner));
  const auto coincident = unacceptedBoundaryGeometryRequest(Eigen::Vector3d{0.0, 0.0, 2.0});
  EXPECT_FALSE(navigation_planning_backend::outgoingGoalAtUnacceptedMainBoundary(coincident));
  auto stopped_goal = unacceptedBoundaryGeometryRequest();
  stopped_goal.route_snapshot.waypoints[1U].behavior =
      navigation_mission::MissionWaypoint::Behavior::Stop;
  EXPECT_FALSE(navigation_planning_backend::outgoingGoalAtUnacceptedMainBoundary(stopped_goal));
  auto wrong_world = unacceptedBoundaryGeometryRequest();
  ++wrong_world.key.pinned_world_revision;
  EXPECT_FALSE(wrong_world.valid());
  EXPECT_FALSE(navigation_planning_backend::outgoingGoalAtUnacceptedMainBoundary(wrong_world));
}

TEST(PlannerFacade, PassThroughLookaheadPrefixWithoutBoundaryEntryStaysValid) {
  auto world = std::make_shared<IdentityOnlyWorld>();
  TestCommitAuthorizer authorizer(world);
  navigation_planning_backend::PlannerFacade facade(PLANNER_FACADE_CONFIG_PATH, world,
                                                    semanticFixtureMissionLimits(), authorizer,
                                                    [] { return 10.0; });

  navigation_mission::Mission mission;
  mission.id = "lookahead-prefix-contract";
  mission.frame = "lio_odom";
  navigation_mission::MissionWaypoint active;
  active.id = "active";
  active.position_enu = Eigen::Vector3d{40.0, 0.0, 3.0};
  active.behavior = navigation_mission::MissionWaypoint::Behavior::PassThrough;
  active.acceptance_radius_m = 0.5;
  navigation_mission::MissionWaypoint next;
  next.id = "next";
  next.position_enu = Eigen::Vector3d{45.0, 0.0, 3.0};
  next.behavior = navigation_mission::MissionWaypoint::Behavior::Stop;
  next.acceptance_radius_m = 0.5;
  mission.waypoints = {active, next};
  navigation_mission::RouteProgress progress(mission);
  ASSERT_TRUE(progress.update(Eigen::Vector3d{0.0, 0.0, 3.0}).valid);
  navigation_planning::KinematicState state;
  state.position_world = Eigen::Vector3d{0.0, 0.0, 3.0};
  state.source_stamp_ns = 1;
  state.receive_stamp_ns = 1;
  state.localization_epoch = 1U;
  state.world_frame_id = "lio_odom";
  state.body_frame_id = "base_link";
  const auto route = progress.snapshot(mission.id, mission.frame, 1U, 1U, 0U);
  ASSERT_TRUE(route.valid());
  const auto outcome = facade.plan(plannerRequestForRoute(
      world, route, state, *semanticFixtureMissionLimits(), 1U));
  ASSERT_TRUE(outcome.valid()) << static_cast<int>(outcome.failure_stage) << ":"
                               << static_cast<int>(outcome.failure_reason);
  const auto candidate = outcome.candidate;
  ASSERT_TRUE(candidate);
  ASSERT_TRUE(candidate->valid());
  EXPECT_FALSE(candidate->route_boundary_event.has_value());
}

TEST(PlannerFacade, PassThroughEntryAfterBackupDoesNotAdvertiseBoundaryEvent) {
  auto world = std::make_shared<IdentityOnlyWorld>();
  TestCommitAuthorizer authorizer(world);
  navigation_planning_backend::PlannerFacade facade(
      PLANNER_FACADE_CONFIG_PATH, world, semanticFixtureMissionLimits(), authorizer,
      [] { return 10.0; });

  navigation_mission::Mission mission;
  mission.id = "backup-boundary-contract";
  mission.frame = "lio_odom";
  navigation_mission::MissionWaypoint active;
  active.id = "active";
  // Place this semantic witness on the BACKUP suffix in the coherent 0.2 m
  // voxel fixture. The former witness at z=1.925 lay on the artificial descent
  // manufactured by mapping every grid index to zero. Keep the same tight
  // event volume and prove the first entry is after MAIN, not just near an
  // arbitrary sampled point.
  active.position_enu = Eigen::Vector3d{14.05, 0.06, 3.0};
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
  navigation_planning::KinematicState state;
  state.position_world = Eigen::Vector3d{0.0, 0.0, 3.0};
  state.source_stamp_ns = 1;
  state.receive_stamp_ns = 1;
  state.localization_epoch = 1U;
  state.world_frame_id = "lio_odom";
  state.body_frame_id = "base_link";
  const auto route = progress.snapshot(mission.id, mission.frame, 1U, 1U, 0U);
  ASSERT_TRUE(route.valid());
  auto request = plannerRequestForRoute(
      world, route, state, *semanticFixtureMissionLimits(), 1U);
  request.planning_goal_world = Eigen::Vector3d{10.0, 0.0, 3.0};
  // Keep the corridor goal radius independent from the deliberately tighter
  // route-event sphere used to place this witness in BACKUP.
  request.goal_acceptance_radius_m = 0.5;
  const auto outcome = facade.plan(request);
  ASSERT_TRUE(outcome.valid()) << static_cast<int>(outcome.failure_stage) << ":"
                               << static_cast<int>(outcome.failure_reason);
  const auto candidate = outcome.candidate;
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

TEST(PlannerFacade, EmergencyBrakeDoesNotAdvertiseNominalPassThroughBoundary) {
  auto world = std::make_shared<IdentityOnlyWorld>();
  TestCommitAuthorizer authorizer(world);
  navigation_planning_backend::PlannerFacade facade(
      PLANNER_FACADE_CONFIG_PATH, world, std::nullopt, authorizer, [] { return 10.0; });

  navigation_mission::Mission mission;
  mission.id = "emergency-pass-through-boundary-contract";
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
  ASSERT_TRUE(progress.update(Eigen::Vector3d{9.8, 0.0, 3.0}).valid);
  const auto route = progress.snapshot(mission.id, mission.frame, 1U, 1U, 0U);

  navigation_planning::KinematicState state;
  state.position_world = Eigen::Vector3d{9.8, 0.0, 3.0};
  state.source_stamp_ns = 1;
  state.receive_stamp_ns = 1;
  state.localization_epoch = 1U;
  state.world_frame_id = "lio_odom";
  state.body_frame_id = "base_link";

  navigation_planning::TrajectoryPoint measured_command;
  measured_command.position_world = state.position_world;
  measured_command.velocity_world = Eigen::Vector3d{0.5, 0.0, 0.0};
  measured_command.acceleration_world = Eigen::Vector3d::Zero();
  measured_command.jerk_world = Eigen::Vector3d::Zero();
  measured_command.yaw = 0.0;
  measured_command.yaw_rate = 0.0;
  ASSERT_TRUE(measured_command.finite());
  navigation_planning::DynamicLimits dynamics;
  dynamics.intent.requested_cruise_speed_mps = 1.0;
  auto request = plannerRequestForRoute(
      world, route, state, dynamics, 1U);
  request.key.start_mode =
      navigation_planning::PlanningStartMode::kMeasuredEmergencyBrake;
  request.start_state.velocity_world = measured_command.velocity_world;
  ASSERT_TRUE(request.valid());
  const auto outcome = facade.plan(request);
  ASSERT_TRUE(outcome.valid()) << static_cast<int>(outcome.failure_stage) << ":"
                               << static_cast<int>(outcome.failure_reason);
  const auto candidate = outcome.candidate;
  ASSERT_TRUE(candidate);
  ASSERT_TRUE(candidate->valid());
  EXPECT_EQ(candidate->kind,
            navigation_planning::CandidateBundleKind::kEmergencyBrake);
  EXPECT_EQ(candidate->role, navigation_planning::CandidateRole::kEmergency);
  EXPECT_FALSE(candidate->route_boundary_event.has_value());
  EXPECT_FALSE(candidate->route_boundary_constraint.has_value());
}

}  // namespace
