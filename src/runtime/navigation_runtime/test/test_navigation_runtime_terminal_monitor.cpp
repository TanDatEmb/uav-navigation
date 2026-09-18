#include <gtest/gtest.h>

#include <algorithm>
#include <cstdint>
#include <memory>
#include <mutex>
#include <optional>

#include <rcl/time.h>
#include <navigation_common/time.hpp>

#include "navigation_runtime/navigation_runtime_node.hpp"

namespace navigation_runtime {

// Access-only peer: no executor is spun and no command is published. The
// controlled immutable command models scheduler inputs, not a planner or
// world certificate proof. The actual moving-capture production witness is
// FAST5-r3 / generation 16 in the diagnostic matrix report.
class NavigationRuntimeTerminalMonitorTestPeer {
 public:
  static bool install(
      NavigationRuntimeNode& node,
      const navigation_contracts::msg::NavigationGoal& goal,
      navigation_world_model::WorldModelViewPtr world,
      navigation_planning::CandidateBundle candidate) {
    node.world_snapshot_store_.publish(std::move(world));
    const auto identity = node.world_snapshot_store_.load().identity;
    const auto before = node.command_bundle_store_.snapshot();
    if (node.command_bundle_store_.publishWorldIdentityIfCurrent(
            identity, before.version, {}, false) !=
        navigation_world_model::WorldCommitDecision::kCommitted) return false;
    if (!node.command_bundle_store_.setActiveGoalEpoch(candidate.goal_epoch)) return false;
    const auto command = std::make_shared<const navigation_planning::CandidateBundle>(
        std::move(candidate));
    if (node.command_bundle_store_.tryCommit(
            {identity, command->goal_epoch, 1U}, command) !=
        navigation_execution::CommitDecision::kCommitted) return false;
    std::lock_guard localization_lock(node.localization_transition_mutex_);
    std::lock_guard input_lock(node.input_mutex_);
    std::lock_guard command_lock(
        node.command_execution_lease_failure_latch_.transitionMutex());
    node.active_goal_ = goal;
    node.executing_goal_ = goal;
    node.active_localization_epoch_.store(command->localization_epoch);
    node.localization_epoch_ready_.store(true);
    node.active_goal_epoch_.store(command->goal_epoch);
    node.command_goal_epoch_.store(command->goal_epoch);
    node.new_goal_ = false;
    node.hot_goal_transition_ = false;
    node.execution_episode_.beginGoal(command->localization_epoch, command->goal_epoch,
                                      command->request_id, false);
    node.execution_episode_.commandCommitted(*command);
    node.trajectory_completion_witness_.reset();
    node.trajectory_reaches_goal_.store(false);
    node.terminal_bundle_generation_.store(0U);
    return true;
  }

  static bool publishState(NavigationRuntimeNode& node, std::int64_t stamp_ns) {
    const auto bundle = node.command_bundle_store_.load();
    const auto sample = bundle ? bundle->sampleAtDeclaredStamp(stamp_ns) : std::nullopt;
    if (!sample) return false;
    navigation_planning::KinematicState state;
    state.position_world = sample->position_world;
    state.velocity_world = sample->velocity_world;
    state.source_stamp_ns = stamp_ns;
    state.receive_stamp_ns = navigation_common::steadyClockNowNanoseconds();
    state.localization_epoch = bundle->localization_epoch;
    state.world_frame_id = "lio_odom";
    state.body_frame_id = "base_link";
    return node.execution_state_store_.publish(std::move(state));
  }

  static auto key(NavigationRuntimeNode& node) { return node.currentPlanningKey(); }
  static auto timeline(const NavigationRuntimeNode& node) {
    return node.command_bundle_store_.snapshot();
  }
  static auto reserveFutureAnchor(NavigationRuntimeNode& node, std::int64_t stamp_ns) {
    return node.command_bundle_store_.reserveAnchor(stamp_ns, stamp_ns + 400'000'000LL);
  }
};

namespace {

constexpr std::int64_t kStartNs = 10'000'000'000LL;
constexpr std::int64_t kEndNs = 11'000'000'000LL;
constexpr std::uint64_t kGeneration = 16U;

class SchedulerIdentityWorld final : public navigation_world_model::WorldModelView {
 public:
  explicit SchedulerIdentityWorld(std::int64_t stamp_ns = kStartNs) : stamp_ns_(stamp_ns) {}
  navigation_world_model::WorldSnapshotIdentity identity() const noexcept override {
    return {1U, 2U, 1U, stamp_ns_};
  }
  navigation_world_model::WorldGeometry geometry() const noexcept override { return {}; }
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
      navigation_world_model::GridLayer, double) const override { return point; }
  bool isSegmentTraversable(
      const navigation_world_model::Point3&, const navigation_world_model::Point3&,
      navigation_world_model::GridLayer,
      navigation_world_model::UnknownPolicy) const noexcept override { return true; }
  navigation_world_model::AxisAlignedBox clampToLocalBounds(
      const navigation_world_model::AxisAlignedBox& box) const noexcept override { return box; }
  navigation_world_model::PointVector observedOccupiedPoints(
      const navigation_world_model::AxisAlignedBox&) const override { return {}; }
 private:
  const std::int64_t stamp_ns_;
};

navigation_contracts::msg::NavigationGoal schedulerGoal() {
  navigation_contracts::msg::NavigationGoal goal;
  goal.header.stamp = rclcpp::Time(kStartNs, RCL_ROS_TIME);
  goal.header.frame_id = "lio_odom";
  goal.mission_id = "terminal-monitor-phase";
  goal.waypoint_index = 1U;
  goal.request_id = 5U;
  goal.target.x = 3.5;
  goal.target.z = 3.0;
  goal.acceptance_radius_m = 0.8;
  goal.behavior = navigation_contracts::msg::NavigationGoal::BEHAVIOR_STOP;
  auto& route = goal.route;
  route.mission_id = goal.mission_id;
  route.frame_id = goal.header.frame_id;
  route.route_revision = 1U;
  route.request_id = goal.request_id;
  route.active_waypoint_index = goal.waypoint_index;
  geometry_msgs::msg::Point origin;
  origin.z = 3.0;
  route.waypoint_positions = {origin, goal.target};
  route.waypoint_ids = {"origin", "terminal"};
  route.waypoint_acceptance_radii_m = {0.8, 0.8};
  route.waypoint_behaviors = {
      navigation_contracts::msg::RouteSnapshot::BEHAVIOR_PASS_THROUGH,
      navigation_contracts::msg::RouteSnapshot::BEHAVIOR_STOP};
  route.measured_progress_valid = true;
  return goal;
}

navigation_planning::CandidateBundle schedulerMain(const bool semantic_terminal_stop) {
  navigation_planning::CandidateBundle candidate;
  candidate.world_identity = SchedulerIdentityWorld{}.identity();
  candidate.pinned_world_identity = candidate.world_identity;
  candidate.localization_epoch = 1U;
  candidate.goal_epoch = 1U;
  candidate.request_id = 5U;
  candidate.bundle_generation = kGeneration;
  candidate.start_wall_time_s = 10.0;
  candidate.duration_s = 1.0;
  candidate.valid_from_ns = kStartNs;
  candidate.valid_until_ns = kEndNs;
  candidate.activation_stamp_ns = kStartNs;
  candidate.declared_start_ns = kStartNs;
  candidate.declared_end_ns = kEndNs;
  candidate.terminal_stop = semantic_terminal_stop;
  candidate.kind = navigation_planning::CandidateBundleKind::kTerminalStop;
  candidate.certificates = {true, true, true, true};
  candidate.protected_region.minimum = Eigen::Vector3d{0.0, -1.0, 2.0};
  candidate.protected_region.maximum = Eigen::Vector3d{4.0, 1.0, 4.0};
  candidate.role_schedule = {{0.0, 1.0, navigation_planning::CandidateRole::kMain}};
  candidate.evaluator = [](std::int64_t stamp_ns, navigation_planning::TrajectoryPoint& point) {
    const double t = std::clamp(static_cast<double>(stamp_ns - kStartNs) * 1.0e-9, 0.0, 1.0);
    const double t2 = t * t;
    const double t3 = t2 * t;
    const double t4 = t3 * t;
    const double t5 = t4 * t;
    const double t6 = t5 * t;
    point.position_world = Eigen::Vector3d{2.9 + 1.2 * (t - 2.5 * t4 + 3.0 * t5 - t6), 0.0, 3.0};
    point.velocity_world.x() = 1.2 * (1.0 - 10.0 * t3 + 15.0 * t4 - 6.0 * t5);
    point.acceleration_world.x() = 1.2 * (-30.0 * t2 + 60.0 * t3 - 30.0 * t4);
    point.jerk_world.x() = 1.2 * (-60.0 * t + 180.0 * t2 - 120.0 * t3);
    point.trajectory_time_s = t;
    return true;
  };
  return candidate;
}

class NavigationRuntimeTerminalMonitor : public testing::Test {
 protected:
  void SetUp() override {
    context_ = std::make_shared<rclcpp::Context>();
    context_->init(0, nullptr);
    rclcpp::NodeOptions options;
    options.context(context_);
    options.parameter_overrides({
        rclcpp::Parameter("use_sim_time", true),
        rclcpp::Parameter("navigation_runtime.planning_frame", "lio_odom"),
        rclcpp::Parameter("navigation_runtime.body_frame_id", "base_link"),
        rclcpp::Parameter("navigation_runtime.deployment_profile", "sitl"),
        rclcpp::Parameter("navigation_runtime.config_path", NAVIGATION_PLANNER_CONFIG_PATH)});
    node_ = std::make_shared<NavigationRuntimeNode>(options);
    setTime(kStartNs);
  }
  void TearDown() override {
    node_.reset();
    context_->shutdown("terminal monitor fixture complete");
  }
  void setTime(std::int64_t stamp_ns) {
    const auto clock = node_->get_clock();
    std::lock_guard clock_lock(clock->get_clock_mutex());
    ASSERT_EQ(rcl_enable_ros_time_override(clock->get_clock_handle()), RCL_RET_OK);
    ASSERT_EQ(rcl_set_ros_time_override(clock->get_clock_handle(), stamp_ns), RCL_RET_OK);
  }
  void install(bool terminal_stop, std::int64_t stamp_ns) {
    const auto world = std::make_shared<SchedulerIdentityWorld>(stamp_ns);
    auto candidate = schedulerMain(terminal_stop);
    candidate.world_identity = world->identity();
    candidate.pinned_world_identity = candidate.world_identity;
    ASSERT_TRUE(candidate.valid());
    ASSERT_TRUE(NavigationRuntimeTerminalMonitorTestPeer::install(
        *node_, schedulerGoal(), world, candidate));
    setTime(stamp_ns);
    ASSERT_EQ(node_->now().nanoseconds(), stamp_ns);
    ASSERT_TRUE(NavigationRuntimeTerminalMonitorTestPeer::publishState(*node_, stamp_ns));
  }
  std::shared_ptr<rclcpp::Context> context_;
  std::shared_ptr<NavigationRuntimeNode> node_;
};

TEST_F(NavigationRuntimeTerminalMonitor, NonTerminalMainHasKeyAtMovingActivation) {
  install(false, kStartNs);
  const auto key = NavigationRuntimeTerminalMonitorTestPeer::key(*node_);
  ASSERT_TRUE(key);
  EXPECT_EQ(key->committed_bundle_generation, kGeneration);
  EXPECT_EQ(key->start_mode, navigation_planning::PlanningStartMode::kCommittedFutureState);
  EXPECT_TRUE(NavigationRuntimeTerminalMonitorTestPeer::reserveFutureAnchor(*node_, kStartNs));
}

TEST_F(NavigationRuntimeTerminalMonitor, NonTerminalKeyDoesNotRequireReservableFutureAnchor) {
  const auto stamp_ns = kEndNs - 200'000'000LL;
  install(false, stamp_ns);
  EXPECT_TRUE(NavigationRuntimeTerminalMonitorTestPeer::key(*node_));
  EXPECT_FALSE(NavigationRuntimeTerminalMonitorTestPeer::reserveFutureAnchor(*node_, stamp_ns));
}

TEST_F(NavigationRuntimeTerminalMonitor, TerminalEndpointSuppressesKeyWithoutDerivedAtomics) {
  install(true, kEndNs);
  EXPECT_FALSE(NavigationRuntimeTerminalMonitorTestPeer::key(*node_));
  const auto timeline = NavigationRuntimeTerminalMonitorTestPeer::timeline(*node_);
  ASSERT_TRUE(timeline.active);
  EXPECT_EQ(timeline.active->bundle_generation, kGeneration);
  EXPECT_EQ(timeline.active->declared_end_ns, kEndNs);
}

// Known RED reproducer, opt-in until the monitor/recovery seam and cross-END
// admission fence are implemented together. Do not enable a phase-only fix
// just to make these assertions pass: key availability is not recovery or
// completion evidence. Run explicitly with --gtest_also_run_disabled_tests.
TEST_F(NavigationRuntimeTerminalMonitor,
       DISABLED_MovingTerminalMainMustRemainMonitorableBeforeDeclaredEnd) {
  install(true, kStartNs);
  const auto command = NavigationRuntimeTerminalMonitorTestPeer::timeline(*node_).active;
  ASSERT_TRUE(command);
  const auto sample = command->sample(kStartNs);
  ASSERT_TRUE(sample);
  ASSERT_GT(sample->velocity_world.norm(),
            navigation_planning::PlanningTimingContract::kStationarySpeedMps);
  EXPECT_TRUE(NavigationRuntimeTerminalMonitorTestPeer::key(*node_));
}

TEST_F(NavigationRuntimeTerminalMonitor,
       DISABLED_TerminalMonitorMustRemainReachableInsideFutureAnchorLead) {
  const auto stamp_ns = kEndNs - 200'000'000LL;
  install(true, stamp_ns);
  ASSERT_FALSE(NavigationRuntimeTerminalMonitorTestPeer::reserveFutureAnchor(*node_, stamp_ns));
  EXPECT_TRUE(NavigationRuntimeTerminalMonitorTestPeer::key(*node_));
}

}  // namespace
}  // namespace navigation_runtime
