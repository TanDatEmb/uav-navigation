#include <gtest/gtest.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cstdint>
#include <future>
#include <memory>
#include <mutex>
#include <optional>

#include <rcl/time.h>
#include <navigation_common/time.hpp>
#include <navigation_contracts/command_safety_contract.hpp>
#include <navigation_planning_backend/planner_facade.hpp>

#include "navigation_runtime/navigation_runtime_node.hpp"

namespace navigation_runtime {

// Access-only peer: no executor is spun. The scheduler-only fixtures model
// inputs, not a certificate proof. The real-facade handoff fixtures also invoke
// the production command callback and queued backend ACK, without DDS dispatch
// or a flight controller. The actual moving-capture production witness is
// FAST5-r3 / generation 16 in the diagnostic matrix report.
class NavigationRuntimeTerminalMonitorTestPeer {
 public:
  static bool install(
      NavigationRuntimeNode& node,
      const navigation_contracts::msg::NavigationGoal& goal,
      navigation_world_model::WorldModelViewPtr world,
      navigation_planning::CandidateBundle candidate) {
    // The real facade fixture already pinned this exact immutable world to
    // authorize its candidate. Do not republish the same identity as new data.
    if (node.world_snapshot_store_.load().view.get() != world.get()) {
      node.world_snapshot_store_.publish(std::move(world));
    }
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
    node.execution_transaction_id_.store(1U);
    node.execution_episode_.beginGoal(command->localization_epoch, command->goal_epoch,
                                      command->request_id, false);
    node.execution_episode_.commandCommitted(*command);
    node.trajectory_completion_witness_.reset();
    node.trajectory_reaches_goal_.store(false);
    node.terminal_bundle_generation_.store(0U);
    return true;
  }

  static bool publishState(
      NavigationRuntimeNode& node, std::int64_t stamp_ns,
      const Eigen::Vector3d& offset = Eigen::Vector3d::Zero(),
      const Eigen::Vector3d& velocity_residual = Eigen::Vector3d::Zero(),
      const std::optional<Eigen::Vector3d>& measured_acceleration = std::nullopt) {
    const auto bundle = node.command_bundle_store_.load();
    const auto sample = bundle ? bundle->sampleAtDeclaredStamp(stamp_ns) : std::nullopt;
    if (!sample) return false;
    navigation_planning::KinematicState state;
    state.position_world = sample->position_world + offset;
    state.velocity_world = sample->velocity_world + velocity_residual;
    state.acceleration_world = measured_acceleration.value_or(sample->acceleration_world);
    state.jerk_world = sample->jerk_world;
    state.acceleration_estimated = !measured_acceleration.has_value();
    state.jerk_estimated = true;
    state.source_stamp_ns = stamp_ns;
    state.receive_stamp_ns = navigation_common::steadyClockNowNanoseconds();
    state.localization_epoch = bundle->localization_epoch;
    state.world_frame_id = "lio_odom";
    state.body_frame_id = "base_link";
    return node.execution_state_store_.publish(std::move(state));
  }

  static bool publishPredecessorState(
      NavigationRuntimeNode& node,
      const navigation_planning::CandidateBundle& predecessor,
      std::int64_t source_stamp_ns) {
    // A delayed measured observation belongs to A at its original source
    // timestamp, even after G becomes the current command. Never clamp or
    // extrapolate G before its declared start to manufacture temporal support.
    const auto sample = predecessor.sampleAtDeclaredStamp(source_stamp_ns);
    if (!sample) return false;
    navigation_planning::KinematicState state;
    state.position_world = sample->position_world;
    state.velocity_world = sample->velocity_world;
    state.acceleration_world = sample->acceleration_world;
    state.jerk_world = sample->jerk_world;
    state.acceleration_estimated = true;
    state.jerk_estimated = true;
    state.source_stamp_ns = source_stamp_ns;
    state.receive_stamp_ns = navigation_common::steadyClockNowNanoseconds();
    state.localization_epoch = predecessor.localization_epoch;
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
  static auto episode(NavigationRuntimeNode& node) { return node.execution_episode_.snapshot(); }
  static auto trace(NavigationRuntimeNode& node) { return node.execution_trace_store_.load(); }
  static auto observationAccounting(NavigationRuntimeNode& node) {
    return node.retained_decision_accounting_;
  }
  static auto solveGeneration(NavigationRuntimeNode& node) {
    return node.planner_solve_generation_.load();
  }
  static auto backendGeneration(NavigationRuntimeNode& node) {
    return node.planner_->committedGeneration();
  }
  static auto optimization(NavigationRuntimeNode& node) {
    return node.planner_->diagnostics().optimization;
  }
  static double trackingBudget(NavigationRuntimeNode& node) {
    return node.planner_->trackingErrorBudgetMeters();
  }
  static bool seedPriorNominalTrace(NavigationRuntimeNode& node) {
    const auto active = node.command_bundle_store_.load();
    if (!active) return false;
    ExecutionTraceSnapshot trace;
    trace.planning_cycle_id = node.cycle_count_;
    trace.solve_generation = 1U;
    trace.timestamp_ns = node.now().nanoseconds();
    trace.execution_localization_epoch = active->localization_epoch;
    trace.execution_goal_epoch = active->goal_epoch;
    trace.execution_request_id = active->request_id;
    trace.execution_bundle_generation = active->bundle_generation;
    return node.execution_trace_store_.publish(std::move(trace));
  }
  static auto boundaryRejection(NavigationRuntimeNode& node) {
    return node.last_execution_boundary_rejection_.load();
  }
  static void cycle(NavigationRuntimeNode& node, const PlanningKey& key) { node.runCycle(key); }
  static void transitionFlags(NavigationRuntimeNode& node, bool new_goal, bool hot_goal) {
    std::lock_guard localization_lock(node.localization_transition_mutex_);
    std::lock_guard input_lock(node.input_mutex_);
    node.new_goal_ = new_goal;
    node.hot_goal_transition_ = hot_goal;
  }
  static bool stagePending(NavigationRuntimeNode& node) {
    const auto active = node.command_bundle_store_.load();
    const auto anchor = node.command_bundle_store_.reserveAnchor(
        active->declared_start_ns, active->declared_start_ns + 400'000'000LL);
    if (!anchor) return false;
    auto successor = *active;
    ++successor.bundle_generation;
    successor.valid_from_ns = anchor->activation_stamp_ns;
    successor.activation_stamp_ns = anchor->activation_stamp_ns;
    return node.command_bundle_store_.stagePending(
               {active->world_identity, active->goal_epoch, 2U}, *anchor,
               std::make_shared<const navigation_planning::CandidateBundle>(successor)) ==
        navigation_execution::StageDecision::kStaged;
  }
  static void monitor(NavigationRuntimeNode& node, const PlanningKey& key) {
    const auto timeline = node.command_bundle_store_.snapshot();
    const auto episode = node.execution_episode_.snapshot();
    const NavigationRuntimeNode::RetainedValidationContext context{
        NavigationRuntimeNode::RetainedValidationPurpose::kTerminalMainMonitor,
        false, true, 0U, std::nullopt,
        retainedCommandTrackingLimit(node.planner_->trackingErrorBudgetMeters(),
                                    navigation_contracts::kCommandAnchorErrorLimitM),
        NavigationRuntimeNode::TerminalMonitorBoundary{timeline, episode}};
    node.validateRetainedCommand(node.active_goal_, key.goal_epoch,
                                 key.localization_epoch, key, context);
  }
  static navigation_planning::PlanningRequest realTerminalRequest(
      NavigationRuntimeNode& node, const navigation_contracts::msg::NavigationGoal& goal,
      const navigation_world_model::WorldModelViewPtr& world, std::int64_t stamp_ns) {
    navigation_mission::Mission mission;
    mission.id = goal.mission_id;
    mission.frame = goal.header.frame_id;
    mission.waypoints = {
        {"origin", Eigen::Vector3d{0.0, 0.0, 3.0}, 0.8, 0.0,
         navigation_mission::MissionWaypoint::Behavior::PassThrough},
        {"terminal", Eigen::Vector3d{3.5, 0.0, 3.0}, 0.8, 0.0,
         navigation_mission::MissionWaypoint::Behavior::Stop}};
    navigation_mission::RouteProgress progress(mission);
    const Eigen::Vector3d measured_start{2.9, 0.0, 3.0};
    (void)progress.update(measured_start);
    navigation_planning::PlanningRequest request;
    request.key = {1U, 1U, goal.request_id, goal.route.route_revision, 0U,
                   world->identity().generation, world->identity().revision,
                   navigation_planning::PlanningStartMode::kStoppedMeasuredState,
                   stamp_ns, node.dynamics_hash_};
    request.goal = {1U, 1U, goal.mission_id, goal.waypoint_index, goal.request_id};
    request.start_state.position_world = measured_start;
    request.start_state.source_stamp_ns = stamp_ns;
    request.start_state.receive_stamp_ns = navigation_common::steadyClockNowNanoseconds();
    request.start_state.localization_epoch = 1U;
    request.start_state.world_frame_id = "lio_odom";
    request.start_state.body_frame_id = "base_link";
    request.route_snapshot = progress.snapshot(mission.id, mission.frame,
                                              goal.route.route_revision, goal.request_id, 1U);
    request.world = world;
    request.dynamics.intent.requested_cruise_speed_mps = 5.0;
    request.dynamics.unknown_space_policy = navigation_world_model::UnknownPolicy::kRequireKnownFree;
    request.budget.deadline = navigation_planning::PlanningBudget::Clock::now() +
        std::chrono::duration_cast<navigation_planning::PlanningBudget::Clock::duration>(
            std::chrono::duration<double>(navigation_planning::PlanningTimingContract::kSolveDeadlineS));
    return request;
  }
  static auto planRealTerminal(
      NavigationRuntimeNode& node, const navigation_contracts::msg::NavigationGoal& goal,
      const navigation_world_model::WorldModelViewPtr& world, std::int64_t stamp_ns) {
    node.world_snapshot_store_.publish(world);
    return node.planner_->plan(realTerminalRequest(node, goal, world, stamp_ns));
  }
  static std::optional<navigation_planning::PlanningRequest> futureTerminalRequest(
      NavigationRuntimeNode& node, const navigation_contracts::msg::NavigationGoal& goal,
      const navigation_world_model::WorldModelViewPtr& world, const PlanningKey& key,
      const navigation_planning::ExecutionAnchor& anchor) {
    const auto measured = node.execution_state_store_.load();
    if (!measured) return std::nullopt;
    auto request = realTerminalRequest(node, goal, world, key.anchor_stamp_ns);
    request.key = key;
    request.start_state = measured->state;
    request.anchor = anchor;
    request.activation_stamp_ns = anchor.activation_stamp_ns;
    request.history.previous_bundle_generation = anchor.active_bundle_generation;
    request.history.previous_velocity_world = anchor.state.velocity_world;
    return request;
  }
  static auto planRequest(NavigationRuntimeNode& node,
                          const navigation_planning::PlanningRequest& request) {
    return node.planner_->plan(request);
  }
  static bool commitRealSuccessor(
      NavigationRuntimeNode& node, const navigation_contracts::msg::NavigationGoal& goal,
      const PlanningKey& key, const navigation_planning::CandidateBundle& candidate,
      bool& admitted) {
    return node.commitPlannerCandidate(goal, key.goal_epoch, key.localization_epoch,
                                       node.now().nanoseconds(), key, candidate, &admitted);
  }
  static void publishCommandAndApplyQueuedActivations(NavigationRuntimeNode& node) {
    node.publishCommand();
    // This is the ordering used by the serial planning worker before runCycle.
    node.applyQueuedExecutionTimelineActivations(*node.planner_);
  }
  static void acknowledge(NavigationRuntimeNode& node) {
    const auto active = node.command_bundle_store_.load();
    node.planner_->onExecutionTimelineActivated(active->bundle_generation);
  }
  static bool refreshWorld(NavigationRuntimeNode& node,
                          const navigation_world_model::WorldModelViewPtr& world,
                          std::int64_t stamp_ns) {
    const auto timeline = node.command_bundle_store_.snapshot();
    if (!timeline.active ||
        !node.planner_->validateCommittedTrajectory(
            world, static_cast<double>(stamp_ns) * 1.0e-9,
            timeline.active->bundle_generation).valid) return false;
    if (node.command_bundle_store_.publishWorldIdentityIfCurrent(
            world->identity(), timeline.version, timeline.active, true,
            stamp_ns + node.data_freshness_window_ns_) !=
        navigation_world_model::WorldCommitDecision::kCommitted) return false;
    node.world_snapshot_store_.publish(world);
    node.planner_->setWorldModelView(world);
    return true;
  }
};

namespace {

constexpr std::int64_t kStartNs = 10'000'000'000LL;
constexpr std::int64_t kEndNs = 11'000'000'000LL;
constexpr std::uint64_t kGeneration = 16U;

// Test observation only: it never changes classification/geometry/identity.
// The test thread advances the external ROS clock while validation is paused.
class ClassificationBarrier {
 public:
  void arm() { armed_.store(true); }
  void visit() noexcept {
    if (!armed_.exchange(false)) return;
    std::unique_lock lock(mutex_);
    entered_ = true;
    condition_.notify_all();
    condition_.wait(lock, [this] { return released_; });
  }
  bool waitUntilEntered() {
    std::unique_lock lock(mutex_);
    return condition_.wait_for(lock, std::chrono::seconds(5), [this] { return entered_; });
  }
  void release() {
    std::lock_guard lock(mutex_);
    released_ = true;
    condition_.notify_all();
  }
 private:
  std::atomic_bool armed_{false};
  std::mutex mutex_;
  std::condition_variable condition_;
  bool entered_{false};
  bool released_{false};
};

class SchedulerIdentityWorld final : public navigation_world_model::WorldModelView {
 public:
  explicit SchedulerIdentityWorld(
      std::int64_t stamp_ns = kStartNs, std::uint64_t revision = 1U,
      std::shared_ptr<ClassificationBarrier> barrier = {})
      : stamp_ns_(stamp_ns), revision_(revision), barrier_(std::move(barrier)) {}
  navigation_world_model::WorldSnapshotIdentity identity() const noexcept override {
    return {1U, 2U, revision_, stamp_ns_};
  }
  navigation_world_model::WorldGeometry geometry() const noexcept override {
    navigation_world_model::WorldGeometry result;
    result.evidence_resolution_m = 0.2;
    result.inflated_resolution_m = 0.2;
    result.occupied_inflation_radius_m = 1.0;
    result.effective_virtual_ground_m = -1.0;
    result.effective_virtual_ceiling_m = 5.0;
    result.local_size_m = Eigen::Vector3d{50.0, 50.0, 8.0};
    result.evidence_bounds = {Eigen::Vector3i{-125, -125, -20}, Eigen::Vector3i{250, 250, 40}};
    result.inflated_bounds = result.evidence_bounds;
    return result;
  }
  navigation_world_model::CellState classify(
      const navigation_world_model::Point3&, navigation_world_model::GridLayer) const noexcept override {
    if (barrier_) barrier_->visit();
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
  const std::uint64_t revision_;
  const std::shared_ptr<ClassificationBarrier> barrier_;
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
  virtual double trackingBaseMeters() const { return 0.0; }
  virtual bool retainedDiagnosticsEnabled() const { return true; }
  void SetUp() override {
    context_ = std::make_shared<rclcpp::Context>();
    context_->init(0, nullptr);
    rclcpp::NodeOptions options;
    options.context(context_);
    options.parameter_overrides({
        rclcpp::Parameter("use_sim_time", true),
        rclcpp::Parameter("navigation_runtime.retained_decision_diagnostics_enabled",
                          retainedDiagnosticsEnabled()),
        rclcpp::Parameter("tracking_experiment.base_m", trackingBaseMeters()),
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
  void installReal(const std::shared_ptr<ClassificationBarrier>& barrier = {},
                   const Eigen::Vector3d& velocity_residual = Eigen::Vector3d::Zero(),
                   const std::optional<Eigen::Vector3d>& measured_acceleration = {}) {
    const auto world = std::make_shared<SchedulerIdentityWorld>(kStartNs);
    const auto outcome = NavigationRuntimeTerminalMonitorTestPeer::planRealTerminal(
        *node_, schedulerGoal(), world, kStartNs);
    ASSERT_TRUE(outcome.candidate) << static_cast<int>(outcome.failure_stage) << ":"
                                   << static_cast<int>(outcome.failure_reason);
    ASSERT_EQ(outcome.candidate->kind, navigation_planning::CandidateBundleKind::kTerminalStop);
    ASSERT_TRUE(outcome.candidate->terminal_stop);
    ASSERT_FALSE(outcome.candidate->backup_available);
    ASSERT_TRUE(NavigationRuntimeTerminalMonitorTestPeer::install(
        *node_, schedulerGoal(), world, *outcome.candidate));
    NavigationRuntimeTerminalMonitorTestPeer::acknowledge(*node_);
    const auto end_ns = outcome.candidate->declared_end_ns;
    ASSERT_GT(end_ns, kStartNs + 200'000'000LL);
    real_stamp_ns_ = end_ns - 200'000'000LL;
    setTime(real_stamp_ns_);
    ASSERT_TRUE(NavigationRuntimeTerminalMonitorTestPeer::refreshWorld(
        *node_, std::make_shared<SchedulerIdentityWorld>(real_stamp_ns_, 2U, barrier),
        real_stamp_ns_));
    ASSERT_TRUE(NavigationRuntimeTerminalMonitorTestPeer::publishState(
        *node_, real_stamp_ns_, Eigen::Vector3d::Zero(), velocity_residual,
        measured_acceleration));
  }
  void checkRealFutureHandoff(const bool source_before_start) {
    // Synthetic controlled schedule, not a measured flight-delay bound:
    // actual certified A -> reserved A(now + 400 ms) -> factory-certified G.
    // The +20 ms monitor observation discriminates the source-time seam from
    // failure to create/admit/activate a complete candidate in the first place.
    const auto goal = schedulerGoal();
    const auto world = std::make_shared<SchedulerIdentityWorld>(kStartNs);
    const auto initial = NavigationRuntimeTerminalMonitorTestPeer::planRealTerminal(
        *node_, goal, world, kStartNs);
    ASSERT_TRUE(initial.candidate)
        << "FIXTURE_BLOCKED: factory did not create predecessor A: "
        << static_cast<int>(initial.failure_stage) << ":"
        << static_cast<int>(initial.failure_reason);
    ASSERT_TRUE(initial.candidate->valid());
    ASSERT_TRUE(NavigationRuntimeTerminalMonitorTestPeer::install(
        *node_, goal, world, *initial.candidate));
    NavigationRuntimeTerminalMonitorTestPeer::acknowledge(*node_);
    const auto predecessor = NavigationRuntimeTerminalMonitorTestPeer::timeline(*node_).active;
    ASSERT_TRUE(predecessor);
    ASSERT_TRUE(NavigationRuntimeTerminalMonitorTestPeer::publishPredecessorState(
        *node_, *predecessor, kStartNs));
    const auto predecessor_key = NavigationRuntimeTerminalMonitorTestPeer::key(*node_);
    ASSERT_TRUE(predecessor_key);
    ASSERT_EQ(predecessor_key->committed_bundle_generation, predecessor->bundle_generation);
    ASSERT_EQ(predecessor_key->start_mode,
              navigation_planning::PlanningStartMode::kCommittedFutureState);
    const auto anchor = NavigationRuntimeTerminalMonitorTestPeer::reserveFutureAnchor(
        *node_, kStartNs);
    ASSERT_TRUE(anchor) << "FIXTURE_BLOCKED: A cannot reserve the existing future head";
    ASSERT_TRUE(anchor->valid());
    ASSERT_EQ(anchor->activation_stamp_ns, kStartNs + 400'000'000LL);
    ASSERT_EQ(anchor->active_bundle_generation, predecessor->bundle_generation);
    ASSERT_GT(anchor->state.velocity_world.norm(),
              navigation_planning::PlanningTimingContract::kStationarySpeedMps);
    const auto request = NavigationRuntimeTerminalMonitorTestPeer::futureTerminalRequest(
        *node_, goal, world, *predecessor_key, *anchor);
    ASSERT_TRUE(request);
    ASSERT_TRUE(request->valid());
    ASSERT_EQ(request->start_state.source_stamp_ns, kStartNs);
    ASSERT_EQ(request->activation_stamp_ns, anchor->activation_stamp_ns);
    ASSERT_EQ(request->history.previous_bundle_generation, predecessor->bundle_generation);
    const auto successor = NavigationRuntimeTerminalMonitorTestPeer::planRequest(*node_, *request);
    ASSERT_TRUE(successor.candidate)
        << "FIXTURE_BLOCKED: factory did not certify future terminal G: "
        << static_cast<int>(successor.failure_stage) << ":"
        << static_cast<int>(successor.failure_reason);
    ASSERT_TRUE(successor.candidate->valid());
    ASSERT_EQ(successor.candidate->kind, navigation_planning::CandidateBundleKind::kTerminalStop)
        << "FIXTURE_BLOCKED: factory produced a different bundle kind";
    ASSERT_TRUE(successor.candidate->terminal_stop);
    ASSERT_FALSE(successor.candidate->backup_available)
        << "FIXTURE_BLOCKED: this control requires a factory-certified MAIN-only capture";
    ASSERT_EQ(successor.candidate->role, navigation_planning::CandidateRole::kMain);
    ASSERT_EQ(successor.candidate->declared_start_ns, anchor->activation_stamp_ns);
    ASSERT_EQ(successor.candidate->activation_stamp_ns, anchor->activation_stamp_ns);
    ASSERT_GT(successor.candidate->bundle_generation, predecessor->bundle_generation);
    const auto head = successor.candidate->sampleAtDeclaredStamp(anchor->activation_stamp_ns);
    ASSERT_TRUE(head);
    ASSERT_GT(head->velocity_world.norm(),
              navigation_planning::PlanningTimingContract::kStationarySpeedMps);
    bool admitted = false;
    ASSERT_TRUE(NavigationRuntimeTerminalMonitorTestPeer::commitRealSuccessor(
        *node_, goal, *predecessor_key, *successor.candidate, admitted))
        << "FIXTURE_BLOCKED: runtime rejected G: "
        << NavigationRuntimeTerminalMonitorTestPeer::boundaryRejection(*node_);
    ASSERT_TRUE(admitted);
    const auto pending = NavigationRuntimeTerminalMonitorTestPeer::timeline(*node_);
    ASSERT_EQ(pending.active, predecessor);
    ASSERT_TRUE(pending.pending);
    ASSERT_EQ(pending.pending->bundle_generation, successor.candidate->bundle_generation);
    ASSERT_EQ(pending.pending_activation_ns, anchor->activation_stamp_ns);

    const auto pre_start_source_ns = anchor->activation_stamp_ns - 8'000'000LL;
    setTime(anchor->activation_stamp_ns);
    ASSERT_TRUE(NavigationRuntimeTerminalMonitorTestPeer::publishPredecessorState(
        *node_, *predecessor, pre_start_source_ns));
    ASSERT_FALSE(pending.pending->sampleAtDeclaredStamp(pre_start_source_ns));
    NavigationRuntimeTerminalMonitorTestPeer::publishCommandAndApplyQueuedActivations(*node_);
    const auto activated = NavigationRuntimeTerminalMonitorTestPeer::timeline(*node_);
    ASSERT_TRUE(activated.active);
    ASSERT_FALSE(activated.pending);
    ASSERT_EQ(activated.active->bundle_generation, successor.candidate->bundle_generation);
    ASSERT_EQ(NavigationRuntimeTerminalMonitorTestPeer::backendGeneration(*node_),
              activated.active->bundle_generation);
    ASSERT_EQ(NavigationRuntimeTerminalMonitorTestPeer::episode(*node_).active_generation,
              activated.active->bundle_generation);

    const auto monitor_stamp_ns = anchor->activation_stamp_ns + 20'000'000LL;
    const auto measured_source_ns = source_before_start
        ? pre_start_source_ns : anchor->activation_stamp_ns + 12'000'000LL;
    setTime(monitor_stamp_ns);
    if (!source_before_start) {
      ASSERT_TRUE(NavigationRuntimeTerminalMonitorTestPeer::publishState(
          *node_, measured_source_ns));
    }
    const auto measured_sample = source_before_start
        ? predecessor->sampleAtDeclaredStamp(measured_source_ns)
        : activated.active->sampleAtDeclaredStamp(measured_source_ns);
    const auto command_now = activated.active->sampleAtDeclaredStamp(monitor_stamp_ns);
    ASSERT_TRUE(measured_sample);
    ASSERT_TRUE(command_now);
    const double independently_sampled_raw_error_m =
        (command_now->position_world - measured_sample->position_world).norm();
    ASSERT_DOUBLE_EQ(NavigationRuntimeTerminalMonitorTestPeer::trackingBudget(*node_), 0.25);
    ASSERT_LE(independently_sampled_raw_error_m,
              NavigationRuntimeTerminalMonitorTestPeer::trackingBudget(*node_));
    const auto key = NavigationRuntimeTerminalMonitorTestPeer::key(*node_);
    ASSERT_TRUE(key);
    ASSERT_EQ(key->committed_bundle_generation, activated.active->bundle_generation);
    ASSERT_EQ(key->anchor_stamp_ns, measured_source_ns);
    const auto solve_generation = NavigationRuntimeTerminalMonitorTestPeer::solveGeneration(*node_);
    NavigationRuntimeTerminalMonitorTestPeer::cycle(*node_, *key);
    const auto after = NavigationRuntimeTerminalMonitorTestPeer::timeline(*node_);
    ASSERT_TRUE(after.active);
    EXPECT_EQ(after.version, activated.version);
    EXPECT_EQ(after.active, activated.active);
    EXPECT_FALSE(after.pending);
    EXPECT_EQ(after.active->declared_end_ns, activated.active->declared_end_ns);
    EXPECT_EQ(NavigationRuntimeTerminalMonitorTestPeer::backendGeneration(*node_),
              activated.active->bundle_generation);
    const auto episode = NavigationRuntimeTerminalMonitorTestPeer::episode(*node_);
    EXPECT_EQ(episode.active_generation, activated.active->bundle_generation);
    EXPECT_EQ(episode.recovery_state, ExecutionRecoveryState::kTrackMain);
    EXPECT_FALSE(episode.failure_latched);
    EXPECT_FALSE(episode.safety_suffix_active);
    EXPECT_EQ(NavigationRuntimeTerminalMonitorTestPeer::solveGeneration(*node_), solve_generation);
    EXPECT_EQ(NavigationRuntimeTerminalMonitorTestPeer::optimization(*node_).lbfgs_attempt_count, 0);
    const auto trace = NavigationRuntimeTerminalMonitorTestPeer::trace(*node_);
    ASSERT_TRUE(trace) << "the monitor must execute, not silently skip this control";
    EXPECT_EQ(trace->execution_bundle_generation, activated.active->bundle_generation);
    EXPECT_EQ(trace->solve_generation, 0U);
    EXPECT_EQ(trace->execution_state_source_stamp_ns, measured_source_ns);
    EXPECT_EQ(trace->committed_bundle_start_stamp_ns, anchor->activation_stamp_ns);
    EXPECT_TRUE(trace->retained_fresh_vehicle_state);
    EXPECT_TRUE(trace->current_vehicle_state_known_free);
    EXPECT_TRUE(trace->sampled_path_clear);
    EXPECT_DOUBLE_EQ(trace->anchor_error_raw_m, independently_sampled_raw_error_m);
    EXPECT_TRUE(trace->committed_suffix_usable);
    EXPECT_EQ(trace->emergency_candidate_commit_result, 0);
    if (source_before_start) {
      EXPECT_TRUE(std::isnan(trace->anchor_error_time_aligned_m));
    } else {
      EXPECT_DOUBLE_EQ(trace->anchor_error_time_aligned_m, 0.0);
    }
  }
  std::shared_ptr<rclcpp::Context> context_;
  std::shared_ptr<NavigationRuntimeNode> node_;
  std::int64_t real_stamp_ns_{0};
};

// Separate strict test profile. Its explicit radius is the current planner
// YAML's clearance-reserved tracking budget, not a production gate change.
// Keep the default fixture at the actual diagnostic relaxed 0/0/0 profile.
class NavigationRuntimeTerminalMonitorStrict : public NavigationRuntimeTerminalMonitor {
 protected:
  double trackingBaseMeters() const override { return 0.25; }
  void SetUp() override {
    NavigationRuntimeTerminalMonitor::SetUp();
    ASSERT_DOUBLE_EQ(trackingBaseMeters(),
                     NavigationRuntimeTerminalMonitorTestPeer::trackingBudget(*node_));
  }
};

class NavigationRuntimeTerminalMonitorObserverOff : public NavigationRuntimeTerminalMonitor {
 protected:
  bool retainedDiagnosticsEnabled() const override { return false; }
};

TEST_F(NavigationRuntimeTerminalMonitorObserverOff,
       RealFutureMainOnlyHandoffWithFreshPreStartStatePreservesOwner) {
  ASSERT_NO_FATAL_FAILURE(checkRealFutureHandoff(true));
  const auto accounting = NavigationRuntimeTerminalMonitorTestPeer::observationAccounting(*node_);
  EXPECT_EQ(accounting.attempted, 1U);
  EXPECT_EQ(accounting.suppressed, 1U);
  EXPECT_EQ(accounting.published, 0U);
}

TEST_F(NavigationRuntimeTerminalMonitorObserverOff,
       RealFutureMainOnlyHandoffWithFreshPostStartStatePreservesOwner) {
  ASSERT_NO_FATAL_FAILURE(checkRealFutureHandoff(false));
  const auto accounting = NavigationRuntimeTerminalMonitorTestPeer::observationAccounting(*node_);
  EXPECT_EQ(accounting.attempted, 1U);
  EXPECT_EQ(accounting.suppressed, 1U);
  EXPECT_EQ(accounting.published, 0U);
}

TEST_F(NavigationRuntimeTerminalMonitor,
       RealFutureMainOnlyHandoffWithFreshPreStartStatePreservesOwner) {
  ASSERT_NO_FATAL_FAILURE(checkRealFutureHandoff(true));
}

TEST_F(NavigationRuntimeTerminalMonitor,
       RealFutureMainOnlyHandoffWithFreshPostStartStatePreservesOwner) {
  ASSERT_NO_FATAL_FAILURE(checkRealFutureHandoff(false));
}

TEST_F(NavigationRuntimeTerminalMonitorStrict,
       RealFutureMainOnlyHandoffWithFreshPreStartStatePreservesOwner) {
  ASSERT_NO_FATAL_FAILURE(checkRealFutureHandoff(true));
}

TEST_F(NavigationRuntimeTerminalMonitorStrict,
       RealFutureMainOnlyHandoffWithFreshPostStartStatePreservesOwner) {
  ASSERT_NO_FATAL_FAILURE(checkRealFutureHandoff(false));
}

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

TEST_F(NavigationRuntimeTerminalMonitor, PendingAndGoalTransitionDoNotOpenTerminalMonitor) {
  install(true, kStartNs);
  NavigationRuntimeTerminalMonitorTestPeer::transitionFlags(*node_, true, false);
  EXPECT_FALSE(NavigationRuntimeTerminalMonitorTestPeer::key(*node_));
  NavigationRuntimeTerminalMonitorTestPeer::transitionFlags(*node_, false, true);
  EXPECT_FALSE(NavigationRuntimeTerminalMonitorTestPeer::key(*node_));
  NavigationRuntimeTerminalMonitorTestPeer::transitionFlags(*node_, false, false);
  ASSERT_TRUE(NavigationRuntimeTerminalMonitorTestPeer::stagePending(*node_));
  EXPECT_FALSE(NavigationRuntimeTerminalMonitorTestPeer::key(*node_));
}

TEST_F(NavigationRuntimeTerminalMonitor, MainWithBackupKeepsOrdinaryRenewalKey) {
  auto candidate = schedulerMain(true);
  candidate.kind = navigation_planning::CandidateBundleKind::kMainWithBackup;
  candidate.backup_available = true;
  candidate.backup_start_time_s = 0.6;
  candidate.role_schedule = {
      {0.0, 0.6, navigation_planning::CandidateRole::kMain},
      {0.6, 1.0, navigation_planning::CandidateRole::kBackup}};
  ASSERT_TRUE(candidate.valid());
  ASSERT_TRUE(NavigationRuntimeTerminalMonitorTestPeer::install(
      *node_, schedulerGoal(), std::make_shared<SchedulerIdentityWorld>(), candidate));
  ASSERT_TRUE(NavigationRuntimeTerminalMonitorTestPeer::publishState(*node_, kStartNs));
  const auto key = NavigationRuntimeTerminalMonitorTestPeer::key(*node_);
  ASSERT_TRUE(key);
  EXPECT_EQ(key->start_mode, navigation_planning::PlanningStartMode::kCommittedFutureState);
}

TEST_F(NavigationRuntimeTerminalMonitor,
       MovingTerminalMainMustRemainMonitorableBeforeDeclaredEnd) {
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
       TerminalMonitorMustRemainReachableInsideFutureAnchorLead) {
  const auto stamp_ns = kEndNs - 200'000'000LL;
  install(true, stamp_ns);
  ASSERT_FALSE(NavigationRuntimeTerminalMonitorTestPeer::reserveFutureAnchor(*node_, stamp_ns));
  EXPECT_TRUE(NavigationRuntimeTerminalMonitorTestPeer::key(*node_));
}

TEST_F(NavigationRuntimeTerminalMonitor, RealBackendHealthyTailMonitorsWithoutNominalSolve) {
  ASSERT_NO_FATAL_FAILURE(installReal());
  const auto before = NavigationRuntimeTerminalMonitorTestPeer::timeline(*node_);
  const auto key = NavigationRuntimeTerminalMonitorTestPeer::key(*node_);
  ASSERT_TRUE(key);
  ASSERT_FALSE(NavigationRuntimeTerminalMonitorTestPeer::reserveFutureAnchor(*node_, real_stamp_ns_));
  const auto solve_generation = NavigationRuntimeTerminalMonitorTestPeer::solveGeneration(*node_);
  NavigationRuntimeTerminalMonitorTestPeer::cycle(*node_, *key);
  const auto after = NavigationRuntimeTerminalMonitorTestPeer::timeline(*node_);
  EXPECT_EQ(after.version, before.version);
  EXPECT_EQ(after.active, before.active);
  EXPECT_EQ(after.active->declared_end_ns, before.active->declared_end_ns);
  const auto episode = NavigationRuntimeTerminalMonitorTestPeer::episode(*node_);
  EXPECT_EQ(episode.recovery_state, ExecutionRecoveryState::kTrackMain);
  EXPECT_FALSE(episode.safety_suffix_active);
  EXPECT_FALSE(episode.failure_latched);
  EXPECT_EQ(NavigationRuntimeTerminalMonitorTestPeer::solveGeneration(*node_), solve_generation);
  EXPECT_EQ(NavigationRuntimeTerminalMonitorTestPeer::optimization(*node_).lbfgs_attempt_count, 0);
  const auto trace = NavigationRuntimeTerminalMonitorTestPeer::trace(*node_);
  ASSERT_TRUE(trace);
  EXPECT_EQ(trace->solve_generation, 0U);
  EXPECT_TRUE(trace->sampled_path_clear);
  EXPECT_EQ(trace->execution_bundle_generation, before.active->bundle_generation);
}

TEST_F(NavigationRuntimeTerminalMonitor, MonitorTraceMustReplacePriorNominalTrace) {
  ASSERT_NO_FATAL_FAILURE(installReal());
  ASSERT_TRUE(NavigationRuntimeTerminalMonitorTestPeer::seedPriorNominalTrace(*node_));
  const auto key = NavigationRuntimeTerminalMonitorTestPeer::key(*node_);
  ASSERT_TRUE(key);
  NavigationRuntimeTerminalMonitorTestPeer::cycle(*node_, *key);
  const auto trace = NavigationRuntimeTerminalMonitorTestPeer::trace(*node_);
  ASSERT_TRUE(trace);
  EXPECT_EQ(trace->solve_generation, 0U);  // monitor is not a nominal solve
  EXPECT_GT(trace->planning_cycle_id, 0U);
  EXPECT_TRUE(trace->sampled_path_clear);
}

TEST_F(NavigationRuntimeTerminalMonitor, QueuedMonitorAtEndCannotFallThroughToNominalSolve) {
  ASSERT_NO_FATAL_FAILURE(installReal());
  const auto before = NavigationRuntimeTerminalMonitorTestPeer::timeline(*node_);
  const auto key = NavigationRuntimeTerminalMonitorTestPeer::key(*node_);
  ASSERT_TRUE(key);
  const auto solve_generation = NavigationRuntimeTerminalMonitorTestPeer::solveGeneration(*node_);
  setTime(before.active->declared_end_ns);
  ASSERT_TRUE(NavigationRuntimeTerminalMonitorTestPeer::publishState(
      *node_, before.active->declared_end_ns));
  NavigationRuntimeTerminalMonitorTestPeer::cycle(*node_, *key);
  EXPECT_EQ(NavigationRuntimeTerminalMonitorTestPeer::timeline(*node_).version, before.version);
  EXPECT_EQ(NavigationRuntimeTerminalMonitorTestPeer::solveGeneration(*node_), solve_generation);
  EXPECT_FALSE(NavigationRuntimeTerminalMonitorTestPeer::episode(*node_).failure_latched);
}

TEST_F(NavigationRuntimeTerminalMonitor, RelaxedProfileSuppressesPressureBeforeFinalLeaseWindow) {
  ASSERT_NO_FATAL_FAILURE(installReal({}, Eigen::Vector3d{3.0, 0.0, 0.0}));
  const auto before = NavigationRuntimeTerminalMonitorTestPeer::timeline(*node_);
  const auto key = NavigationRuntimeTerminalMonitorTestPeer::key(*node_);
  ASSERT_TRUE(key);
  NavigationRuntimeTerminalMonitorTestPeer::cycle(*node_, *key);
  const auto trace = NavigationRuntimeTerminalMonitorTestPeer::trace(*node_);
  ASSERT_TRUE(trace);
  EXPECT_GT(trace->projected_anchor_error_m, trace->retained_tracking_limit_m);
  EXPECT_TRUE(trace->phase_execution_bridge_usable);
  EXPECT_FALSE(trace->tracking_certificate_exceeded);
  EXPECT_FALSE(trace->projected_tracking_certificate_exceeded);
  EXPECT_EQ(trace->emergency_candidate_commit_result, 0);
  EXPECT_EQ(NavigationRuntimeTerminalMonitorTestPeer::timeline(*node_).version, before.version);
  EXPECT_EQ(NavigationRuntimeTerminalMonitorTestPeer::timeline(*node_).active, before.active);
  EXPECT_FALSE(NavigationRuntimeTerminalMonitorTestPeer::episode(*node_).failure_latched);
}

TEST_F(NavigationRuntimeTerminalMonitor, RelaxedProfileStillAllowsPressureInsideFinalLeaseWindow) {
  ASSERT_NO_FATAL_FAILURE(installReal());
  const auto before = NavigationRuntimeTerminalMonitorTestPeer::timeline(*node_);
  // Cross the existing 100 ms bridge lease condition, without changing it.
  const auto stamp_ns = before.active->declared_end_ns - 50'000'000LL;
  setTime(stamp_ns);
  ASSERT_TRUE(NavigationRuntimeTerminalMonitorTestPeer::refreshWorld(
      *node_, std::make_shared<SchedulerIdentityWorld>(stamp_ns, 3U), stamp_ns));
  ASSERT_TRUE(NavigationRuntimeTerminalMonitorTestPeer::publishState(
      *node_, stamp_ns, Eigen::Vector3d::Zero(), Eigen::Vector3d{3.0, 0.0, 0.0}));
  const auto key = NavigationRuntimeTerminalMonitorTestPeer::key(*node_);
  ASSERT_TRUE(key);
  NavigationRuntimeTerminalMonitorTestPeer::cycle(*node_, *key);
  const auto trace = NavigationRuntimeTerminalMonitorTestPeer::trace(*node_);
  ASSERT_TRUE(trace);
  EXPECT_TRUE(trace->projected_tracking_certificate_exceeded);
  EXPECT_EQ(trace->emergency_candidate_commit_result, 1);  // preparation only
  const auto after = NavigationRuntimeTerminalMonitorTestPeer::timeline(*node_);
  ASSERT_TRUE(after.active);
  EXPECT_EQ(after.active->kind, navigation_planning::CandidateBundleKind::kEmergencyBrake);
  EXPECT_NE(after.active->bundle_generation, before.active->bundle_generation);
  EXPECT_EQ(NavigationRuntimeTerminalMonitorTestPeer::backendGeneration(*node_),
            after.active->bundle_generation);  // actual activated store receipt
  EXPECT_FALSE(NavigationRuntimeTerminalMonitorTestPeer::episode(*node_).failure_latched);
}

TEST_F(NavigationRuntimeTerminalMonitorStrict, ProjectedPressureCanAdmitRealBrakeWithoutFutureAnchor) {
  ASSERT_NO_FATAL_FAILURE(installReal({}, Eigen::Vector3d{3.0, 0.0, 0.0}));
  const auto key = NavigationRuntimeTerminalMonitorTestPeer::key(*node_);
  ASSERT_TRUE(key);
  ASSERT_FALSE(NavigationRuntimeTerminalMonitorTestPeer::reserveFutureAnchor(*node_, real_stamp_ns_));
  NavigationRuntimeTerminalMonitorTestPeer::cycle(*node_, *key);
  const auto after = NavigationRuntimeTerminalMonitorTestPeer::timeline(*node_);
  ASSERT_TRUE(after.active);
  EXPECT_EQ(after.active->kind, navigation_planning::CandidateBundleKind::kEmergencyBrake);
  EXPECT_EQ(NavigationRuntimeTerminalMonitorTestPeer::backendGeneration(*node_),
            after.active->bundle_generation);
  EXPECT_EQ(NavigationRuntimeTerminalMonitorTestPeer::episode(*node_).recovery_state,
            ExecutionRecoveryState::kEmergencyBrake);
  EXPECT_FALSE(NavigationRuntimeTerminalMonitorTestPeer::episode(*node_).failure_latched);
}

TEST_F(NavigationRuntimeTerminalMonitorStrict, StartedBeforeEndBrakePreparedLateIsDiscardOnlyWithoutPublisher) {
  const auto barrier = std::make_shared<ClassificationBarrier>();
  ASSERT_NO_FATAL_FAILURE(installReal(barrier, Eigen::Vector3d{3.0, 0.0, 0.0}));
  const auto before = NavigationRuntimeTerminalMonitorTestPeer::timeline(*node_);
  const auto key = NavigationRuntimeTerminalMonitorTestPeer::key(*node_);
  ASSERT_TRUE(key);
  // Pause current-body classification BEFORE brake construction. This proves
  // late preparation after a pre-END entry, not a post-certificate lock race.
  barrier->arm();
  auto monitor = std::async(std::launch::async, [&] {
    NavigationRuntimeTerminalMonitorTestPeer::monitor(*node_, *key);
  });
  const bool entered = barrier->waitUntilEntered();
  if (entered) setTime(before.active->declared_end_ns);
  barrier->release();
  monitor.get();
  ASSERT_TRUE(entered);
  const auto after = NavigationRuntimeTerminalMonitorTestPeer::timeline(*node_);
  EXPECT_EQ(after.version, before.version);
  EXPECT_EQ(after.active, before.active);
  EXPECT_EQ(NavigationRuntimeTerminalMonitorTestPeer::backendGeneration(*node_),
            before.active->bundle_generation);
  EXPECT_FALSE(NavigationRuntimeTerminalMonitorTestPeer::episode(*node_).failure_latched);
  const auto trace = NavigationRuntimeTerminalMonitorTestPeer::trace(*node_);
  ASSERT_TRUE(trace);
  EXPECT_TRUE(trace->projected_tracking_certificate_exceeded);
  EXPECT_EQ(trace->emergency_candidate_commit_result, 1);  // backend certificate, NOT store receipt
  EXPECT_EQ(NavigationRuntimeTerminalMonitorTestPeer::boundaryRejection(*node_),
            1600 + static_cast<int>(navigation_execution::CommitDecision::kAdmissionRejected));
}

TEST_F(NavigationRuntimeTerminalMonitorStrict, FailedBrakePreparationAcrossEndCannotFailCloseOwner) {
  const auto barrier = std::make_shared<ClassificationBarrier>();
  ASSERT_NO_FATAL_FAILURE(installReal(barrier, Eigen::Vector3d{3.0, 0.0, 0.0},
                                    Eigen::Vector3d{1000.0, 0.0, 0.0}));
  const auto before = NavigationRuntimeTerminalMonitorTestPeer::timeline(*node_);
  const auto key = NavigationRuntimeTerminalMonitorTestPeer::key(*node_);
  ASSERT_TRUE(key);
  barrier->arm();
  auto monitor = std::async(std::launch::async, [&] {
    NavigationRuntimeTerminalMonitorTestPeer::monitor(*node_, *key);
  });
  const bool entered = barrier->waitUntilEntered();
  if (entered) setTime(before.active->declared_end_ns);
  barrier->release();
  monitor.get();
  ASSERT_TRUE(entered);
  EXPECT_EQ(NavigationRuntimeTerminalMonitorTestPeer::timeline(*node_).version, before.version);
  EXPECT_FALSE(NavigationRuntimeTerminalMonitorTestPeer::episode(*node_).failure_latched);
  const auto trace = NavigationRuntimeTerminalMonitorTestPeer::trace(*node_);
  ASSERT_TRUE(trace);
  EXPECT_EQ(trace->emergency_candidate_commit_result, 2);
}

TEST_F(NavigationRuntimeTerminalMonitorStrict, FailedBrakePreparationBeforeEndStillFailsClosed) {
  ASSERT_NO_FATAL_FAILURE(installReal({}, Eigen::Vector3d{3.0, 0.0, 0.0},
                                    Eigen::Vector3d{1000.0, 0.0, 0.0}));
  const auto key = NavigationRuntimeTerminalMonitorTestPeer::key(*node_);
  ASSERT_TRUE(key);
  NavigationRuntimeTerminalMonitorTestPeer::monitor(*node_, *key);
  const auto trace = NavigationRuntimeTerminalMonitorTestPeer::trace(*node_);
  ASSERT_TRUE(trace);
  EXPECT_EQ(trace->emergency_candidate_commit_result, 2);
  EXPECT_TRUE(NavigationRuntimeTerminalMonitorTestPeer::episode(*node_).failure_latched);
  EXPECT_FALSE(NavigationRuntimeTerminalMonitorTestPeer::episode(*node_).command_available);
  const auto accounting = NavigationRuntimeTerminalMonitorTestPeer::observationAccounting(*node_);
  EXPECT_EQ(accounting.attempted, 1U);  // record survives actual fail-closed delivery
  EXPECT_EQ(accounting.published, 1U);  // ROS call receipt, not evidence capture
  EXPECT_EQ(accounting.failed, 0U);
}

}  // namespace
}  // namespace navigation_runtime
