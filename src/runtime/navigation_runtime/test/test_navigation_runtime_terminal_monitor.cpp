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
#include <thread>
#include <utility>

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
  static bool prepareInitialBaseline(
      NavigationRuntimeNode& node,
      const navigation_contracts::msg::NavigationGoal& goal,
      const navigation_world_model::WorldModelViewPtr& world, std::int64_t stamp_ns,
      const Eigen::Vector3d& initial_position = Eigen::Vector3d{0.0, 0.0, 3.0}) {
    node.world_snapshot_store_.publish(world);
    const auto before = node.command_bundle_store_.snapshot();
    if (node.command_bundle_store_.publishWorldIdentityIfCurrent(
            world->identity(), before.version, {}, false) !=
        navigation_world_model::WorldCommitDecision::kCommitted ||
        !node.command_bundle_store_.beginGoal(1U, 1U, false)) return false;
    {
      std::lock_guard localization_lock(node.localization_transition_mutex_);
      std::lock_guard input_lock(node.input_mutex_);
      std::lock_guard command_lock(
          node.command_execution_lease_failure_latch_.transitionMutex());
      node.active_goal_ = goal;
      node.executing_goal_.reset();
      node.active_localization_epoch_.store(1U);
      node.localization_epoch_ready_.store(true);
      node.active_goal_epoch_.store(1U);
      node.command_goal_epoch_.store(0U);
      node.new_goal_ = true;
      node.hot_goal_transition_ = false;
      node.mission_start_position_world_ = initial_position;
      node.mission_start_mission_id_ = goal.mission_id;
      node.mission_start_localization_epoch_ = 1U;
      node.mission_start_route_revision_ = goal.route.route_revision;
    }
    navigation_planning::KinematicState state;
    state.position_world = initial_position;
    state.source_stamp_ns = stamp_ns;
    state.receive_stamp_ns = navigation_common::steadyClockNowNanoseconds();
    state.localization_epoch = 1U;
    state.world_frame_id = "lio_odom";
    state.body_frame_id = "base_link";
    return node.execution_state_store_.publish(std::move(state));
  }
  static void applyQueuedActivations(NavigationRuntimeNode& node) {
    node.applyQueuedExecutionTimelineActivations(*node.planner_);
  }
  static bool advanceStoppedState(NavigationRuntimeNode& node, std::int64_t stamp_ns) {
    const auto prior = node.execution_state_store_.load();
    if (!prior || stamp_ns <= prior->state.source_stamp_ns) return false;
    auto state = prior->state;
    state.source_stamp_ns = stamp_ns;
    state.receive_stamp_ns = navigation_common::steadyClockNowNanoseconds();
    return node.execution_state_store_.publish(std::move(state));
  }
  static bool advanceInitialWorld(
      NavigationRuntimeNode& node,
      const navigation_world_model::WorldModelViewPtr& world) {
    const auto before = node.command_bundle_store_.snapshot();
    if (before.active || before.pending) return false;
    if (node.command_bundle_store_.publishWorldIdentityIfCurrent(
            world->identity(), before.version, {}, false) !=
        navigation_world_model::WorldCommitDecision::kCommitted) return false;
    node.world_snapshot_store_.publish(world);
    return true;
  }
  static auto failureReason(NavigationRuntimeNode& node) {
    return static_cast<navigation_planning::PlanningFailureReason>(
        node.last_planning_failure_reason_.load());
  }
  static auto completeOutcome(NavigationRuntimeNode& node) {
    return static_cast<navigation_planning::CompletePlanningOutcome>(
        node.last_planning_outcome_.load());
  }
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
    if (!node.command_bundle_store_.beginGoal(
            candidate.localization_epoch, candidate.goal_epoch, false)) return false;
    const auto command = std::make_shared<const navigation_planning::CandidateBundle>(
        std::move(candidate));
    if (node.command_bundle_store_.tryCommit(
            {identity, command->goal_epoch, 1U},
            std::make_shared<const navigation_contracts::msg::NavigationGoal>(goal),
            command) !=
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
      std::int64_t source_stamp_ns,
      const Eigen::Vector3d& measured_offset = Eigen::Vector3d::Zero()) {
    // A delayed measured observation belongs to A at its original source
    // timestamp, even after G becomes the current command. Never clamp or
    // extrapolate G before its declared start to manufacture temporal support.
    const auto sample = predecessor.sampleAtDeclaredStamp(source_stamp_ns);
    if (!sample) return false;
    navigation_planning::KinematicState state;
    state.position_world = sample->position_world + measured_offset;
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
  static auto episode(NavigationRuntimeNode& node) { return node.command_bundle_store_.episodeSnapshot(); }
  static auto ownerTimelineAndEpisode(NavigationRuntimeNode& node) {
    std::lock_guard localization_lock(node.localization_transition_mutex_);
    std::lock_guard input_lock(node.input_mutex_);
    std::lock_guard command_lock(node.command_execution_lease_failure_latch_.transitionMutex());
    return std::pair(node.command_bundle_store_.snapshot(), node.command_bundle_store_.episodeSnapshot());
  }
  static auto holdActivationQueue(NavigationRuntimeNode& node) {
    return std::unique_lock(node.planner_timeline_activation_mutex_);
  }
  static auto commandGoalEpoch(NavigationRuntimeNode& node) {
    return node.command_goal_epoch_.load();
  }
  static auto executingGoal(NavigationRuntimeNode& node) {
    std::lock_guard lock(node.input_mutex_);
    return node.executing_goal_;
  }
  static void changeGoal(NavigationRuntimeNode& node,
                         const navigation_contracts::msg::NavigationGoal& goal) {
    node.onGoal(std::make_shared<const navigation_contracts::msg::NavigationGoal>(goal));
  }
  static void resetEpoch(NavigationRuntimeNode& node, std::uint64_t epoch) {
    std::unique_lock lock(node.localization_transition_mutex_);
    node.resetForLocalizationEpochLocked(epoch, lock);
  }
  static void failExecution(NavigationRuntimeNode& node) {
    std::lock_guard localization_lock(node.localization_transition_mutex_);
    std::lock_guard input_lock(node.input_mutex_);
    std::lock_guard command_lock(node.command_execution_lease_failure_latch_.transitionMutex());
    node.failClosedLocked();
  }
  static auto admitPreparedImmediate(
      NavigationRuntimeNode& node, const navigation_contracts::msg::NavigationGoal& goal,
      const PlanningKey& key, const navigation_planning::CandidateBundle& candidate,
      const navigation_execution::ExecutionTimelineSnapshot& predecessor,
      const std::shared_ptr<const navigation_execution::ExecutionStateLease>& measured) {
    const auto transaction = node.execution_transaction_id_.fetch_add(1U) + 1U;
    return node.admitImmediateCandidate(
        goal, {candidate.world_identity, key.goal_epoch, transaction},
        std::make_shared<const navigation_planning::CandidateBundle>(candidate),
        predecessor, key, measured, node.data_freshness_window_ns_);
  }
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
  static void cycle(NavigationRuntimeNode& node, const PlanningKey& key) {
    node.runCycle(key, {});
  }
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
               node.command_bundle_store_.executingGoal(),
               std::make_shared<const navigation_planning::CandidateBundle>(successor)) ==
        navigation_execution::StageDecision::kStaged;
  }
  static void monitor(NavigationRuntimeNode& node, const PlanningKey& key) {
    const auto timeline = node.command_bundle_store_.snapshot();
    const auto episode = node.command_bundle_store_.episodeSnapshot();
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
    {
      std::lock_guard input_lock(node.input_mutex_);
      request.mission_start_position_world = node.mission_start_position_world_;
    }
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
  static auto prepareRealEmergency(NavigationRuntimeNode& node) {
    const auto lease = node.execution_state_store_.load();
    const auto key = node.currentPlanningKey();
    if (!lease || !key) {
      return std::optional<navigation_planning::CandidateBundle>{};
    }
    navigation_planning::TrajectoryPoint measured;
    measured.position_world = lease->state.position_world;
    measured.velocity_world = lease->state.velocity_world;
    measured.acceleration_world = lease->state.acceleration_estimated
        ? Eigen::Vector3d::Zero().eval() : lease->state.acceleration_world;
    measured.jerk_world = lease->state.jerk_estimated
        ? Eigen::Vector3d::Zero().eval() : lease->state.jerk_world;
    measured.yaw = lease->state.yaw_rad;
    if (!node.planner_->commitEmergencyBrake(
            measured, node.now().seconds(),
            key->localization_epoch, key->goal_epoch, key->request_id,
            measured.position_world.z())) {
      return std::optional<navigation_planning::CandidateBundle>{};
    }
    return node.planner_->exportCommandCandidate(
        lease->state.localization_epoch, node.active_goal_epoch_.load(),
        node.active_goal_->request_id, node.now().nanoseconds(),
        node.now().nanoseconds() + node.data_freshness_window_ns_);
  }
  static auto admitPreparedEmergency(NavigationRuntimeNode& node,
                                     const navigation_planning::CandidateBundle& candidate,
                                     std::shared_ptr<const navigation_execution::ExecutionStateLease> measured = {}) {
    const NavigationRuntimeNode::TerminalMonitorBoundary boundary{
        node.command_bundle_store_.snapshot(), node.command_bundle_store_.episodeSnapshot()};
    const auto transaction = node.execution_transaction_id_.fetch_add(1U) + 1U;
    const auto key = node.currentPlanningKey();
    if (!key) return navigation_execution::CommitDecision::kAdmissionRejected;
    return node.admitImmediateCandidate(
        *node.active_goal_, {candidate.world_identity, candidate.goal_epoch, transaction},
        std::make_shared<const navigation_planning::CandidateBundle>(candidate),
        boundary.timeline, *key, measured ? measured : node.execution_state_store_.load(),
        node.data_freshness_window_ns_, boundary);
  }
  static auto stateLease(NavigationRuntimeNode& node) {
    return node.execution_state_store_.load();
  }
  static bool refreshStagedWorldBeforeAck(
      NavigationRuntimeNode& node, const navigation_world_model::WorldModelViewPtr& world) {
    const auto timeline = node.command_bundle_store_.snapshot();
    if (!timeline.active ||
        !timeline.active->validateWorld(world, node.now().seconds()).valid) return false;
    if (node.command_bundle_store_.publishWorldIdentityIfCurrent(
            world->identity(), timeline.version, timeline.active, true,
            node.now().nanoseconds() + node.data_freshness_window_ns_) !=
        navigation_world_model::WorldCommitDecision::kCommitted) return false;
    node.world_snapshot_store_.publish(world);
    return true;
  }
  static bool refreshWorld(NavigationRuntimeNode& node,
                          const navigation_world_model::WorldModelViewPtr& world,
                          std::int64_t stamp_ns) {
    const auto timeline = node.command_bundle_store_.snapshot();
    if (!timeline.active ||
        !timeline.active->validateWorld(
            world, static_cast<double>(stamp_ns) * 1.0e-9).valid) return false;
    if (node.command_bundle_store_.publishWorldIdentityIfCurrent(
            world->identity(), timeline.version, timeline.active, true,
            stamp_ns + node.data_freshness_window_ns_) !=
        navigation_world_model::WorldCommitDecision::kCommitted) return false;
    node.world_snapshot_store_.publish(world);
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

class SchedulerIdentityWorld : public navigation_world_model::WorldModelView {
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

// Finite sensor-known-free convex box for the initial-planning control.
// A provided body witness confers no extra UNKNOWN permission: traversal is
// still sensor-only. The base World's default body-prefix API fails closed,
// so it cannot drive the runtime's actual stopped-state request with support.
class BaselineRefinementWorld final : public SchedulerIdentityWorld {
 public:
  using SchedulerIdentityWorld::SchedulerIdentityWorld;
  navigation_world_model::WorldGeometry geometry() const noexcept override {
    auto result = SchedulerIdentityWorld::geometry();
    result.evidence_bounds.global_min_index.z() = -5;
    result.inflated_bounds = result.evidence_bounds;
    return result;
  }
  bool contains(const navigation_world_model::Point3& p) const noexcept override {
    return p.allFinite() && p.x() >= -25.0 && p.x() < 25.0 &&
        p.y() >= -25.0 && p.y() < 25.0 && p.z() >= -1.0 && p.z() < 7.0;
  }
  navigation_world_model::CellState classify(const navigation_world_model::Point3& p,
      navigation_world_model::GridLayer) const noexcept override {
    return contains(p) ? navigation_world_model::CellState::kKnownFree
                       : navigation_world_model::CellState::kOutOfMap;
  }
  bool isSegmentTraversable(const navigation_world_model::Point3& a,
      const navigation_world_model::Point3& b, navigation_world_model::GridLayer,
      navigation_world_model::UnknownPolicy) const noexcept override {
    return contains(a) && contains(b);
  }
  bool isSegmentTraversableWithCurrentBodySupport(const navigation_world_model::Point3& a,
      const navigation_world_model::Point3& b, navigation_world_model::GridLayer layer,
      navigation_world_model::UnknownPolicy policy,
      const navigation_world_model::CurrentBodySupportPtr&) const noexcept override {
    return isSegmentTraversable(a, b, layer, policy);
  }
  navigation_world_model::AxisAlignedBox clampToLocalBounds(
      const navigation_world_model::AxisAlignedBox& b) const noexcept override {
    return {b.minimum.cwiseMax(Eigen::Vector3d{-25.0, -25.0, -1.0}),
            b.maximum.cwiseMin(Eigen::Vector3d{25.0, 25.0, 7.0})};
  }
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
  virtual std::string missionFile() const { return {}; }
  virtual std::int64_t failedReplanCycle() const { return 0; }
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
        rclcpp::Parameter("navigation_runtime.mission_file", missionFile()),
        rclcpp::Parameter("navigation_runtime.inject_failed_replan_cycle_id", failedReplanCycle()),
        rclcpp::Parameter("navigation_runtime.inject_failed_replan_once", failedReplanCycle() != 0),
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
  void checkRealFutureHandoff(
      const bool source_before_start, const bool tracking_pressure = false,
      const std::int64_t predecessor_start_ns = kStartNs,
      const std::int64_t monitor_offset_ns = 20'000'000LL,
      const std::int64_t predecessor_source_delay_ns = 8'000'000LL) {
    // Synthetic controlled schedule, not a measured flight-delay bound:
    // actual certified A -> reserved A(now + 400 ms) -> factory-certified G.
    // Default +20 ms controls discriminate temporal support from failure to
    // create/admit/activate G. The native 56.092 s exact-START controls below
    // additionally exercise independent absolute-seconds representations.
    setTime(predecessor_start_ns);
    auto goal = schedulerGoal();
    goal.header.stamp = rclcpp::Time(predecessor_start_ns, RCL_ROS_TIME);
    const auto world = std::make_shared<SchedulerIdentityWorld>(predecessor_start_ns);
    const auto initial = NavigationRuntimeTerminalMonitorTestPeer::planRealTerminal(
        *node_, goal, world, predecessor_start_ns);
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
        *node_, *predecessor, predecessor_start_ns));
    const auto predecessor_key = NavigationRuntimeTerminalMonitorTestPeer::key(*node_);
    ASSERT_TRUE(predecessor_key);
    ASSERT_EQ(predecessor_key->committed_bundle_generation, predecessor->bundle_generation);
    ASSERT_EQ(predecessor_key->start_mode,
              navigation_planning::PlanningStartMode::kCommittedFutureState);
    const auto anchor = NavigationRuntimeTerminalMonitorTestPeer::reserveFutureAnchor(
        *node_, predecessor_start_ns);
    ASSERT_TRUE(anchor) << "FIXTURE_BLOCKED: A cannot reserve the existing future head";
    ASSERT_TRUE(anchor->valid());
    ASSERT_EQ(anchor->activation_stamp_ns, predecessor_start_ns + 400'000'000LL);
    ASSERT_EQ(anchor->active_bundle_generation, predecessor->bundle_generation);
    ASSERT_GT(anchor->state.velocity_world.norm(),
              navigation_planning::PlanningTimingContract::kStationarySpeedMps);
    const auto request = NavigationRuntimeTerminalMonitorTestPeer::futureTerminalRequest(
        *node_, goal, world, *predecessor_key, *anchor);
    ASSERT_TRUE(request);
    ASSERT_TRUE(request->valid());
    ASSERT_EQ(request->start_state.source_stamp_ns, predecessor_start_ns);
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

    const auto pre_start_source_ns =
        anchor->activation_stamp_ns - predecessor_source_delay_ns;
    // Controlled synthetic plant residual, not replay of the native worker
    // pin. Its size comes from the independently paired SAFE2 G21 published
    // command/source positions. Never alter SOURCE, command head or certificate.
    const Eigen::Vector3d measured_offset = tracking_pressure
        ? Eigen::Vector3d{0.5020302723007433, 0.1395032219362763,
                          0.012844457407660936}
        : Eigen::Vector3d::Zero();
    setTime(anchor->activation_stamp_ns);
    ASSERT_TRUE(NavigationRuntimeTerminalMonitorTestPeer::publishPredecessorState(
        *node_, *predecessor, pre_start_source_ns, measured_offset));
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

    const auto monitor_stamp_ns = anchor->activation_stamp_ns + monitor_offset_ns;
    const auto measured_source_ns = source_before_start
        ? pre_start_source_ns : anchor->activation_stamp_ns + 12'000'000LL;
    setTime(monitor_stamp_ns);
    if (!source_before_start) {
      ASSERT_TRUE(NavigationRuntimeTerminalMonitorTestPeer::publishState(
          *node_, measured_source_ns, measured_offset));
    }
    const auto measured_sample = source_before_start
        ? predecessor->sampleAtDeclaredStamp(measured_source_ns)
        : activated.active->sampleAtDeclaredStamp(measured_source_ns);
    const auto command_now = activated.active->sampleAtDeclaredStamp(monitor_stamp_ns);
    ASSERT_TRUE(measured_sample);
    ASSERT_TRUE(command_now);
    const double independently_sampled_raw_error_m =
        (command_now->position_world -
         (measured_sample->position_world + measured_offset)).norm();
    ASSERT_DOUBLE_EQ(NavigationRuntimeTerminalMonitorTestPeer::trackingBudget(*node_), 0.25);
    if (tracking_pressure) {
      ASSERT_GT(independently_sampled_raw_error_m,
                NavigationRuntimeTerminalMonitorTestPeer::trackingBudget(*node_));
      ASSERT_LT(independently_sampled_raw_error_m,
                navigation_contracts::kCommandAnchorErrorLimitM);
    } else {
      ASSERT_LE(independently_sampled_raw_error_m,
                NavigationRuntimeTerminalMonitorTestPeer::trackingBudget(*node_));
    }
    const auto key = NavigationRuntimeTerminalMonitorTestPeer::key(*node_);
    ASSERT_TRUE(key);
    ASSERT_EQ(key->committed_bundle_generation, activated.active->bundle_generation);
    ASSERT_EQ(key->anchor_stamp_ns, measured_source_ns);
    const auto solve_generation = NavigationRuntimeTerminalMonitorTestPeer::solveGeneration(*node_);
    const auto lbfgs_attempts_before_monitor =
        NavigationRuntimeTerminalMonitorTestPeer::optimization(*node_).lbfgs_attempt_count;
    NavigationRuntimeTerminalMonitorTestPeer::cycle(*node_, *key);
    const auto after = NavigationRuntimeTerminalMonitorTestPeer::timeline(*node_);
    const auto trace = NavigationRuntimeTerminalMonitorTestPeer::trace(*node_);
    ASSERT_TRUE(trace) << "the monitor must execute, not silently skip this control";
    EXPECT_EQ(trace->execution_bundle_generation, activated.active->bundle_generation);
    EXPECT_EQ(trace->solve_generation, 0U);
    EXPECT_EQ(trace->execution_state_source_stamp_ns, measured_source_ns);
    EXPECT_EQ(trace->committed_bundle_start_stamp_ns, anchor->activation_stamp_ns);
    EXPECT_DOUBLE_EQ(trace->retained_elapsed_s,
                     static_cast<double>(monitor_offset_ns) * 1.0e-9);
    EXPECT_TRUE(trace->retained_fresh_vehicle_state);
    EXPECT_TRUE(trace->current_vehicle_state_known_free);
    EXPECT_TRUE(trace->sampled_path_clear);
    EXPECT_DOUBLE_EQ(trace->anchor_error_raw_m, independently_sampled_raw_error_m);
    EXPECT_EQ(NavigationRuntimeTerminalMonitorTestPeer::solveGeneration(*node_), solve_generation);
    EXPECT_EQ(NavigationRuntimeTerminalMonitorTestPeer::optimization(*node_).lbfgs_attempt_count,
              lbfgs_attempts_before_monitor);
    if (tracking_pressure) {
      // Missing source-time support is not a finite supported tracking error
      // and is not covered by the relaxed tracking-response bypass. Recovery
      // must authorize H independently, never manufacture a witness for G.
      if (source_before_start) {
        EXPECT_TRUE(std::isnan(trace->anchor_error_time_aligned_m));
        EXPECT_FALSE(trace->tracking_certificate_exceeded);
        EXPECT_FALSE(trace->projected_tracking_certificate_exceeded);
        EXPECT_EQ(trace->emergency_authorization_reason,
                  navigation_contracts::msg::NavigationCommand::
                      EMERGENCY_AUTHORIZATION_INDETERMINATE_PRE_START_TRACKING);
        ASSERT_EQ(trace->emergency_candidate_commit_result, 1)
            << "indeterminate support must attempt independently certified H";
        ASSERT_TRUE(after.active);
        EXPECT_EQ(after.active->kind, navigation_planning::CandidateBundleKind::kEmergencyBrake);
        EXPECT_NE(after.active->bundle_generation, activated.active->bundle_generation);
        EXPECT_EQ(NavigationRuntimeTerminalMonitorTestPeer::backendGeneration(*node_),
                  after.active->bundle_generation);
        const auto episode = NavigationRuntimeTerminalMonitorTestPeer::episode(*node_);
        EXPECT_EQ(episode.active_generation, after.active->bundle_generation);
        EXPECT_EQ(episode.recovery_state, ExecutionRecoveryState::kEmergencyBrake);
        EXPECT_FALSE(episode.failure_latched);
        EXPECT_TRUE(episode.command_available);
      } else if (trackingBaseMeters() > 0.0) {
        EXPECT_NEAR(trace->anchor_error_time_aligned_m, measured_offset.norm(), 1.0e-12);
        EXPECT_TRUE(trace->tracking_certificate_exceeded);
        ASSERT_EQ(trace->emergency_candidate_commit_result, 1)
            << "FIXTURE_BLOCKED: matched post-START brake did not certify";
        ASSERT_TRUE(after.active)
            << "FIXTURE_BLOCKED: matched brake had no canonical admission/delivery";
        EXPECT_EQ(after.active->kind, navigation_planning::CandidateBundleKind::kEmergencyBrake);
        EXPECT_NE(after.active->bundle_generation, activated.active->bundle_generation);
        EXPECT_EQ(NavigationRuntimeTerminalMonitorTestPeer::backendGeneration(*node_),
                  after.active->bundle_generation);
        EXPECT_EQ(NavigationRuntimeTerminalMonitorTestPeer::episode(*node_).recovery_state,
                  ExecutionRecoveryState::kEmergencyBrake);
        EXPECT_FALSE(NavigationRuntimeTerminalMonitorTestPeer::episode(*node_).failure_latched);
      } else {
        EXPECT_NEAR(trace->anchor_error_time_aligned_m, measured_offset.norm(), 1.0e-12);
        EXPECT_TRUE(trace->experimental_tracking_bridge_usable);
        EXPECT_FALSE(trace->tracking_certificate_exceeded);
        EXPECT_FALSE(trace->projected_tracking_certificate_exceeded);
        EXPECT_EQ(trace->emergency_candidate_commit_result, 0);
        EXPECT_EQ(after.active, activated.active);
        const auto episode = NavigationRuntimeTerminalMonitorTestPeer::episode(*node_);
        EXPECT_EQ(episode.recovery_state, ExecutionRecoveryState::kTrackMain);
        EXPECT_TRUE(episode.command_available);
        EXPECT_FALSE(episode.failure_latched);
      }
      return;
    }
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
  void checkImmediateCutoverBeforeAck(bool recertify_world) {
    const auto goal = schedulerGoal();
    const auto world = std::make_shared<SchedulerIdentityWorld>(kStartNs);
    const auto request = NavigationRuntimeTerminalMonitorTestPeer::realTerminalRequest(
        *node_, goal, world, kStartNs);
    ASSERT_TRUE(NavigationRuntimeTerminalMonitorTestPeer::prepareInitialBaseline(
        *node_, goal, world, kStartNs, request.start_state.position_world));
    const auto key = NavigationRuntimeTerminalMonitorTestPeer::key(*node_);
    ASSERT_TRUE(key);
    ASSERT_EQ(key->committed_bundle_generation, 0U);
    const auto planned = NavigationRuntimeTerminalMonitorTestPeer::planRequest(*node_, request);
    ASSERT_TRUE(planned.candidate) << "FIXTURE_BLOCKED: actual factory did not certify H";
    ASSERT_TRUE(planned.candidate->valid());
    ASSERT_EQ(planned.candidate->kind, navigation_planning::CandidateBundleKind::kTerminalStop);
    // Block only backend ACK enqueue. The real store/owner transaction must
    // already have completed before the worker can wait on this mutex.
    // Declare future first: on a fatal assertion the queue guard releases
    // before future destruction joins the worker, avoiding a test deadlock.
    bool admitted = false;
    std::future<bool> result;
    auto queue_guard = NavigationRuntimeTerminalMonitorTestPeer::holdActivationQueue(*node_);
    result = std::async(std::launch::async, [&] {
      return NavigationRuntimeTerminalMonitorTestPeer::commitRealSuccessor(
          *node_, goal, *key, *planned.candidate, admitted);
    });
    std::shared_ptr<const navigation_planning::CandidateBundle> cutover;
    const auto watchdog = std::chrono::steady_clock::now() + std::chrono::seconds(5);
    do {
      cutover = NavigationRuntimeTerminalMonitorTestPeer::timeline(*node_).active;
      if (cutover) break;
      std::this_thread::yield();
    } while (std::chrono::steady_clock::now() < watchdog);
    ASSERT_TRUE(cutover) << "FIXTURE_BLOCKED: worker did not reach actual store cutover";
    ASSERT_EQ(cutover->bundle_generation, planned.candidate->bundle_generation);
    const auto coherent = NavigationRuntimeTerminalMonitorTestPeer::ownerTimelineAndEpisode(*node_);
    ASSERT_EQ(coherent.first.active, cutover);
    EXPECT_EQ(coherent.second.active_generation,
              cutover->bundle_generation) << "owner reader must not observe store H / Episode G";
    EXPECT_EQ(NavigationRuntimeTerminalMonitorTestPeer::commandGoalEpoch(*node_), key->goal_epoch);
    if (recertify_world) {
      // Production staged validator, followed by the actual store's immutable
      // same-generation world copy. No manufactured candidate/certificate.
      ASSERT_TRUE(NavigationRuntimeTerminalMonitorTestPeer::refreshStagedWorldBeforeAck(
          *node_, std::make_shared<SchedulerIdentityWorld>(kStartNs, 2U)))
          << "FIXTURE_BLOCKED: actual H full-world validation failed";
      const auto copy = NavigationRuntimeTerminalMonitorTestPeer::timeline(*node_).active;
      ASSERT_TRUE(copy);
      EXPECT_NE(copy.get(), cutover.get());
      EXPECT_EQ(copy->bundle_generation, cutover->bundle_generation);
      EXPECT_EQ(copy->world_identity.revision, 2U);
    }
    queue_guard.unlock();
    EXPECT_TRUE(result.get());
    EXPECT_TRUE(admitted);
    const auto active = NavigationRuntimeTerminalMonitorTestPeer::timeline(*node_).active;
    ASSERT_TRUE(active);
    const auto episode = NavigationRuntimeTerminalMonitorTestPeer::episode(*node_);
    EXPECT_EQ(episode.active_generation, active->bundle_generation);
    EXPECT_TRUE(episode.command_available);
    EXPECT_FALSE(episode.failure_latched);
    EXPECT_EQ(NavigationRuntimeTerminalMonitorTestPeer::commandGoalEpoch(*node_), key->goal_epoch);
    const auto executing = NavigationRuntimeTerminalMonitorTestPeer::executingGoal(*node_);
    ASSERT_TRUE(executing);
    EXPECT_EQ(executing->request_id, goal.request_id);
    NavigationRuntimeTerminalMonitorTestPeer::publishCommandAndApplyQueuedActivations(*node_);
    EXPECT_EQ(NavigationRuntimeTerminalMonitorTestPeer::backendGeneration(*node_),
              active->bundle_generation);
    EXPECT_EQ(NavigationRuntimeTerminalMonitorTestPeer::timeline(*node_).active, active);
    EXPECT_TRUE(NavigationRuntimeTerminalMonitorTestPeer::episode(*node_).command_available);
  }
  enum class ImmediateInvalidation { kClockExpiry, kGoalChange, kEpochReset, kFailure };
  void checkPreparedImmediateInvalidation(ImmediateInvalidation invalidation) {
    const auto goal = schedulerGoal();
    const auto world = std::make_shared<SchedulerIdentityWorld>(kStartNs);
    const auto request = NavigationRuntimeTerminalMonitorTestPeer::realTerminalRequest(
        *node_, goal, world, kStartNs);
    ASSERT_TRUE(NavigationRuntimeTerminalMonitorTestPeer::prepareInitialBaseline(
        *node_, goal, world, kStartNs, request.start_state.position_world));
    const auto key = NavigationRuntimeTerminalMonitorTestPeer::key(*node_);
    ASSERT_TRUE(key);
    const auto predecessor = NavigationRuntimeTerminalMonitorTestPeer::timeline(*node_);
    const auto measured = NavigationRuntimeTerminalMonitorTestPeer::stateLease(*node_);
    const auto planned = NavigationRuntimeTerminalMonitorTestPeer::planRequest(*node_, request);
    ASSERT_TRUE(planned.candidate) << "FIXTURE_BLOCKED: actual factory did not certify H";
    ASSERT_TRUE(planned.candidate->valid());
    switch (invalidation) {
      case ImmediateInvalidation::kClockExpiry:
        setTime(kStartNs + 600'000'000LL);
        break;
      case ImmediateInvalidation::kGoalChange: {
        auto next_goal = goal;
        ++next_goal.request_id;
        next_goal.route.request_id = next_goal.request_id;
        NavigationRuntimeTerminalMonitorTestPeer::changeGoal(*node_, next_goal);
        break;
      }
      case ImmediateInvalidation::kEpochReset:
        NavigationRuntimeTerminalMonitorTestPeer::resetEpoch(*node_, 2U);
        break;
      case ImmediateInvalidation::kFailure:
        NavigationRuntimeTerminalMonitorTestPeer::failExecution(*node_);
        break;
    }
    const auto before = NavigationRuntimeTerminalMonitorTestPeer::ownerTimelineAndEpisode(*node_);
    const auto before_executing = NavigationRuntimeTerminalMonitorTestPeer::executingGoal(*node_);
    const auto before_command_epoch = NavigationRuntimeTerminalMonitorTestPeer::commandGoalEpoch(*node_);
    EXPECT_NE(NavigationRuntimeTerminalMonitorTestPeer::admitPreparedImmediate(
                  *node_, goal, *key, *planned.candidate, predecessor, measured),
              navigation_execution::CommitDecision::kCommitted);
    const auto after = NavigationRuntimeTerminalMonitorTestPeer::ownerTimelineAndEpisode(*node_);
    EXPECT_EQ(after.first.version, before.first.version);
    EXPECT_EQ(after.first.active, before.first.active);
    EXPECT_EQ(after.first.pending, before.first.pending);
    EXPECT_EQ(after.second.active_generation, before.second.active_generation);
    EXPECT_EQ(after.second.command_available, before.second.command_available);
    EXPECT_EQ(after.second.failure_latched, before.second.failure_latched);
    EXPECT_EQ(NavigationRuntimeTerminalMonitorTestPeer::executingGoal(*node_), before_executing);
    EXPECT_EQ(NavigationRuntimeTerminalMonitorTestPeer::commandGoalEpoch(*node_), before_command_epoch);
  }
};

class NavigationRuntimeTerminalMonitorObserverOff : public NavigationRuntimeTerminalMonitor {
 protected:
  bool retainedDiagnosticsEnabled() const override { return false; }
};

class NavigationRuntimeBaselineRefinement : public NavigationRuntimeTerminalMonitor {
 protected:
  double trackingBaseMeters() const override { return 0.25; }
  bool retainedDiagnosticsEnabled() const override { return false; }
  std::string missionFile() const override { return NAVIGATION_REFINEMENT_MISSION_PATH; }
  void checkInitialCapturedInputs(bool advance_state, bool advance_world) {
    // Controlled timer-key -> new producer snapshots -> worker-entry ordering.
    // No sleep, changed ownership, anchor retiming, budget or fake success.
    auto goal = schedulerGoal();
    goal.target.x = 13.1;
    goal.target.y = -2.1;
    goal.route.waypoint_positions.back() = goal.target;
    ASSERT_TRUE(NavigationRuntimeTerminalMonitorTestPeer::prepareInitialBaseline(
        *node_, goal, std::make_shared<BaselineRefinementWorld>(kStartNs), kStartNs));
    const auto scheduled = NavigationRuntimeTerminalMonitorTestPeer::key(*node_);
    ASSERT_TRUE(scheduled);
    ASSERT_EQ(scheduled->start_mode, PlanningStartMode::kStoppedMeasuredState);
    ASSERT_EQ(scheduled->anchor_stamp_ns, kStartNs);
    ASSERT_EQ(scheduled->pinned_world_revision, 1U);
    setTime(kStartNs + 20'000'000LL);
    if (advance_state) {
      ASSERT_TRUE(NavigationRuntimeTerminalMonitorTestPeer::advanceStoppedState(
          *node_, kStartNs + 20'000'000LL));
    }
    if (advance_world) {
      ASSERT_TRUE(NavigationRuntimeTerminalMonitorTestPeer::advanceInitialWorld(
          *node_, std::make_shared<BaselineRefinementWorld>(
              kStartNs + 20'000'000LL, 2U)));
    }
    const auto current = NavigationRuntimeTerminalMonitorTestPeer::key(*node_);
    ASSERT_TRUE(current);
    ASSERT_TRUE(PlanningSupervisor::resultStillCurrent(*scheduled, *current));
    ASSERT_EQ(current->anchor_stamp_ns,
              advance_state ? kStartNs + 20'000'000LL : kStartNs);
    ASSERT_EQ(current->pinned_world_revision, advance_world ? 2U : 1U);
    NavigationRuntimeTerminalMonitorTestPeer::cycle(*node_, *scheduled);
    EXPECT_NE(NavigationRuntimeTerminalMonitorTestPeer::failureReason(*node_),
              navigation_planning::PlanningFailureReason::kInvalidInput)
        << "same-owner source/revision advance must not mix key and owned inputs";
    EXPECT_EQ(NavigationRuntimeTerminalMonitorTestPeer::completeOutcome(*node_),
              navigation_planning::CompletePlanningOutcome::kBaselineCompleteBundle);
    const auto after = NavigationRuntimeTerminalMonitorTestPeer::timeline(*node_);
    ASSERT_TRUE(after.active) << "actual stopped-request factory admission, not fake success";
    EXPECT_TRUE(after.active->valid());
    EXPECT_EQ(after.active->kind, navigation_planning::CandidateBundleKind::kMainWithBackup);
    EXPECT_TRUE(after.active->backup_available);
    EXPECT_EQ(after.active->pinned_world_identity.revision, advance_world ? 2U : 1U);
    EXPECT_FALSE(after.pending);
  }
  void checkMovingDueCapturedInputs(bool advance_state, bool advance_world) {
    auto goal = schedulerGoal();
    goal.target.x = 13.1;
    goal.target.y = -2.1;
    goal.route.waypoint_positions.back() = goal.target;
    ASSERT_TRUE(NavigationRuntimeTerminalMonitorTestPeer::prepareInitialBaseline(
        *node_, goal, std::make_shared<BaselineRefinementWorld>(kStartNs), kStartNs));
    const auto initial_key = NavigationRuntimeTerminalMonitorTestPeer::key(*node_);
    ASSERT_TRUE(initial_key);
    NavigationRuntimeTerminalMonitorTestPeer::cycle(*node_, *initial_key);
    const auto initial = NavigationRuntimeTerminalMonitorTestPeer::timeline(*node_);
    ASSERT_TRUE(initial.active);
    ASSERT_TRUE(initial.active->valid());
    ASSERT_TRUE(initial.active->backup_available);
    // Drain the actual post-admission quiet tick, not a forced scheduler flag.
    setTime(kStartNs + 100'000'000LL);
    ASSERT_TRUE(NavigationRuntimeTerminalMonitorTestPeer::publishState(
        *node_, kStartNs + 100'000'000LL));
    const auto quiet_key = NavigationRuntimeTerminalMonitorTestPeer::key(*node_);
    ASSERT_TRUE(quiet_key);
    NavigationRuntimeTerminalMonitorTestPeer::cycle(*node_, *quiet_key);
    NavigationRuntimeTerminalMonitorTestPeer::applyQueuedActivations(*node_);
    ASSERT_EQ(NavigationRuntimeTerminalMonitorTestPeer::backendGeneration(*node_),
              initial.active->bundle_generation);
    const auto main_duration_ns = navigation_common::secondsToNanoseconds(
        initial.active->backup_start_time_s);
    ASSERT_TRUE(main_duration_ns);
    ASSERT_GT(*main_duration_ns, 1'000'000'000LL);
    // Ordinary DUE, still before the existing future splice; no quality hook.
    const auto due_ns = initial.active->declared_start_ns +
        *main_duration_ns - 900'000'000LL;
    setTime(due_ns);
    ASSERT_TRUE(NavigationRuntimeTerminalMonitorTestPeer::refreshWorld(
        *node_, std::make_shared<BaselineRefinementWorld>(due_ns, 2U), due_ns));
    ASSERT_TRUE(NavigationRuntimeTerminalMonitorTestPeer::publishState(*node_, due_ns));
    const auto scheduled = NavigationRuntimeTerminalMonitorTestPeer::key(*node_);
    ASSERT_TRUE(scheduled);
    ASSERT_EQ(scheduled->start_mode, PlanningStartMode::kCommittedFutureState);
    ASSERT_TRUE(NavigationRuntimeTerminalMonitorTestPeer::reserveFutureAnchor(*node_, due_ns));
    const auto captured_ns = due_ns + 20'000'000LL;
    setTime(captured_ns);
    if (advance_state) {
      ASSERT_TRUE(NavigationRuntimeTerminalMonitorTestPeer::publishState(*node_, captured_ns));
    }
    if (advance_world) {
      ASSERT_TRUE(NavigationRuntimeTerminalMonitorTestPeer::refreshWorld(
          *node_, std::make_shared<BaselineRefinementWorld>(captured_ns, 3U), captured_ns));
    }
    const auto current = NavigationRuntimeTerminalMonitorTestPeer::key(*node_);
    ASSERT_TRUE(current);
    ASSERT_TRUE(PlanningSupervisor::resultStillCurrent(*scheduled, *current));
    const auto retained = NavigationRuntimeTerminalMonitorTestPeer::timeline(*node_).active;
    ASSERT_TRUE(retained);
    const auto due = classifyPlannerRenewal(
        false, true, false, navigation_planning::CandidateRole::kMain, true,
        static_cast<double>(captured_ns - retained->declared_start_ns) * 1.0e-9,
        retained->backup_start_time_s, 0.08, 0.4, 0.1);
    ASSERT_EQ(due.reason, PlannerRenewalReason::kRenewalDue);
    const auto before_solve = NavigationRuntimeTerminalMonitorTestPeer::solveGeneration(*node_);
    NavigationRuntimeTerminalMonitorTestPeer::cycle(*node_, *scheduled);
    EXPECT_EQ(NavigationRuntimeTerminalMonitorTestPeer::solveGeneration(*node_), before_solve + 1U);
    EXPECT_NE(NavigationRuntimeTerminalMonitorTestPeer::failureReason(*node_),
              navigation_planning::PlanningFailureReason::kInvalidInput);
    // Input coherence does not promise optimizer success. If this actual
    // factory admits a successor, it must remain pending with exact PVAJ/yaw.
    const auto after = NavigationRuntimeTerminalMonitorTestPeer::timeline(*node_);
    ASSERT_EQ(after.active, retained);
    if (after.pending) {
      const auto predecessor = retained->sampleAtDeclaredStamp(after.pending->activation_stamp_ns);
      const auto successor = after.pending->sampleAtDeclaredStamp(after.pending->activation_stamp_ns);
      ASSERT_TRUE(predecessor);
      ASSERT_TRUE(successor);
      EXPECT_NEAR((predecessor->position_world - successor->position_world).norm(), 0.0, 1.0e-7);
      EXPECT_NEAR((predecessor->velocity_world - successor->velocity_world).norm(), 0.0, 1.0e-7);
      EXPECT_NEAR((predecessor->acceleration_world - successor->acceleration_world).norm(), 0.0, 1.0e-7);
      EXPECT_NEAR((predecessor->jerk_world - successor->jerk_world).norm(), 0.0, 1.0e-7);
      EXPECT_NEAR(predecessor->yaw - successor->yaw, 0.0, 1.0e-7);
      EXPECT_NEAR(predecessor->yaw_rate - successor->yaw_rate, 0.0, 1.0e-7);
      EXPECT_EQ(after.pending->pinned_world_identity.revision, advance_world ? 3U : 2U);
    }
  }
};

TEST_F(NavigationRuntimeBaselineRefinement, InitialRequestUsesCapturedStateAfterTimerKey) {
  checkInitialCapturedInputs(true, false);
}

TEST_F(NavigationRuntimeBaselineRefinement, InitialRequestUsesCapturedWorldAfterTimerKey) {
  checkInitialCapturedInputs(false, true);
}

TEST_F(NavigationRuntimeBaselineRefinement, InitialRequestUsesCapturedStateAndWorldAfterTimerKey) {
  checkInitialCapturedInputs(true, true);
}

TEST_F(NavigationRuntimeBaselineRefinement, InitialRequestUsesUnchangedCapturedInputs) {
  checkInitialCapturedInputs(false, false);
}

TEST_F(NavigationRuntimeBaselineRefinement, MovingDueUsesCapturedStateAfterTimerKey) {
  checkMovingDueCapturedInputs(true, false);
}

TEST_F(NavigationRuntimeBaselineRefinement, MovingDueUsesCapturedWorldAfterTimerKey) {
  checkMovingDueCapturedInputs(false, true);
}

TEST_F(NavigationRuntimeBaselineRefinement, MovingDueUsesCapturedStateAndWorldAfterTimerKey) {
  checkMovingDueCapturedInputs(true, true);
}

TEST_F(NavigationRuntimeBaselineRefinement, MovingDueUsesUnchangedCapturedInputs) {
  checkMovingDueCapturedInputs(false, false);
}

TEST_F(NavigationRuntimeBaselineRefinement, InputRefreshCannotRebaseOwnership) {
  auto goal = schedulerGoal();
  goal.target.x = 13.1;
  goal.target.y = -2.1;
  goal.route.waypoint_positions.back() = goal.target;
  ASSERT_TRUE(NavigationRuntimeTerminalMonitorTestPeer::prepareInitialBaseline(
      *node_, goal, std::make_shared<BaselineRefinementWorld>(kStartNs), kStartNs));
  const auto current = NavigationRuntimeTerminalMonitorTestPeer::key(*node_);
  ASSERT_TRUE(current);
  const auto before = NavigationRuntimeTerminalMonitorTestPeer::timeline(*node_);
  const auto before_solve = NavigationRuntimeTerminalMonitorTestPeer::solveGeneration(*node_);
  for (int field = 0; field < 7; ++field) {
    SCOPED_TRACE(field);
    auto superseded = *current;
    switch (field) {
      case 0: ++superseded.localization_epoch; break;
      case 1: ++superseded.goal_epoch; break;
      case 2: ++superseded.request_id; break;
      case 3: ++superseded.route_revision; break;
      case 4: ++superseded.committed_bundle_generation; break;
      case 5: ++superseded.pinned_world_generation; break;
      case 6: ++superseded.dynamics_hash; break;
    }
    ASSERT_TRUE(superseded.valid());
    ASSERT_FALSE(PlanningSupervisor::resultStillCurrent(superseded, *current));
    NavigationRuntimeTerminalMonitorTestPeer::cycle(*node_, superseded);
    EXPECT_EQ(NavigationRuntimeTerminalMonitorTestPeer::solveGeneration(*node_), before_solve);
    const auto after = NavigationRuntimeTerminalMonitorTestPeer::timeline(*node_);
    EXPECT_EQ(after.active, before.active);
    EXPECT_EQ(after.pending, before.pending);
    EXPECT_FALSE(NavigationRuntimeTerminalMonitorTestPeer::episode(*node_).failure_latched);
  }
}

TEST_F(NavigationRuntimeBaselineRefinement, ActualInitialAdmissionAckAndDebounceAllowOneEarlyAttempt) {
  // Actual runtime stopped request -> facade validation -> canonical admission
  // -> queued ACK -> future-anchor successor. Synthetic states/clocks, no DDS,
  // sensing/flight or completion/performance qualification claim.
  auto goal = schedulerGoal();
  goal.target.x = 13.1;
  goal.target.y = -2.1;
  goal.route.waypoint_positions.back() = goal.target;
  const auto world = std::make_shared<BaselineRefinementWorld>(kStartNs);
  ASSERT_TRUE(NavigationRuntimeTerminalMonitorTestPeer::prepareInitialBaseline(
      *node_, goal, world, kStartNs));
  const auto initial_key = NavigationRuntimeTerminalMonitorTestPeer::key(*node_);
  ASSERT_TRUE(initial_key);
  ASSERT_EQ(initial_key->start_mode, PlanningStartMode::kStoppedMeasuredState);
  NavigationRuntimeTerminalMonitorTestPeer::cycle(*node_, *initial_key);
  const auto initial = NavigationRuntimeTerminalMonitorTestPeer::timeline(*node_);
  ASSERT_TRUE(initial.active) << "FIXTURE_BLOCKED: no actual initial runtime admission";
  ASSERT_TRUE(initial.active->valid());
  ASSERT_EQ(initial.active->kind, navigation_planning::CandidateBundleKind::kMainWithBackup);
  ASSERT_EQ(NavigationRuntimeTerminalMonitorTestPeer::completeOutcome(*node_),
            navigation_planning::CompletePlanningOutcome::kBaselineCompleteBundle);
  ASSERT_GT(initial.active->backup_start_time_s, 1.5);
  ASSERT_EQ(NavigationRuntimeTerminalMonitorTestPeer::backendGeneration(*node_), 0U);
  const auto initial_solve = NavigationRuntimeTerminalMonitorTestPeer::solveGeneration(*node_);
  for (const auto offset_ns : {100'000'000LL, 200'000'000LL}) {
    setTime(kStartNs + offset_ns);
    ASSERT_TRUE(NavigationRuntimeTerminalMonitorTestPeer::publishState(
        *node_, kStartNs + offset_ns));
    const auto key = NavigationRuntimeTerminalMonitorTestPeer::key(*node_);
    ASSERT_TRUE(key);
    NavigationRuntimeTerminalMonitorTestPeer::cycle(*node_, *key);
    EXPECT_EQ(NavigationRuntimeTerminalMonitorTestPeer::solveGeneration(*node_), initial_solve);
    EXPECT_EQ(NavigationRuntimeTerminalMonitorTestPeer::timeline(*node_).active, initial.active);
  }
  NavigationRuntimeTerminalMonitorTestPeer::applyQueuedActivations(*node_);
  ASSERT_EQ(NavigationRuntimeTerminalMonitorTestPeer::backendGeneration(*node_),
            initial.active->bundle_generation);
  setTime(kStartNs + 300'000'000LL);
  // Validate against a fresh same-G world before extending the lease. The
  // initial lease alone cannot cover now+400ms; never fabricate validity.
  ASSERT_TRUE(NavigationRuntimeTerminalMonitorTestPeer::refreshWorld(
      *node_, std::make_shared<BaselineRefinementWorld>(kStartNs + 300'000'000LL, 2U),
      kStartNs + 300'000'000LL));
  const auto retained = NavigationRuntimeTerminalMonitorTestPeer::timeline(*node_).active;
  ASSERT_TRUE(retained);
  ASSERT_EQ(retained->bundle_generation, initial.active->bundle_generation);
  ASSERT_TRUE(NavigationRuntimeTerminalMonitorTestPeer::reserveFutureAnchor(
      *node_, kStartNs + 300'000'000LL)) << "FIXTURE_BLOCKED: future anchor lease";
  ASSERT_TRUE(NavigationRuntimeTerminalMonitorTestPeer::publishState(
      *node_, kStartNs + 300'000'000LL));
  const auto key = NavigationRuntimeTerminalMonitorTestPeer::key(*node_);
  ASSERT_TRUE(key);
  NavigationRuntimeTerminalMonitorTestPeer::cycle(*node_, *key);
  EXPECT_EQ(NavigationRuntimeTerminalMonitorTestPeer::solveGeneration(*node_), initial_solve + 1U);
  EXPECT_GT(NavigationRuntimeTerminalMonitorTestPeer::optimization(*node_).lbfgs_attempt_count, 0);
  const auto after = NavigationRuntimeTerminalMonitorTestPeer::timeline(*node_);
  ASSERT_EQ(after.active, retained) << "early replacement must stage, not steal active ownership";
  ASSERT_TRUE(after.pending) << "FIXTURE_BLOCKED: actual successor did not fully validate/admit";
  const auto old_head = retained->sampleAtDeclaredStamp(after.pending->activation_stamp_ns);
  const auto new_head = after.pending->sampleAtDeclaredStamp(after.pending->activation_stamp_ns);
  ASSERT_TRUE(old_head);
  ASSERT_TRUE(new_head);
  EXPECT_NEAR((old_head->position_world - new_head->position_world).norm(), 0.0, 1.0e-7);
  EXPECT_NEAR((old_head->velocity_world - new_head->velocity_world).norm(), 0.0, 1.0e-7);
  EXPECT_NEAR((old_head->acceleration_world - new_head->acceleration_world).norm(), 0.0, 1.0e-7);
  EXPECT_NEAR((old_head->jerk_world - new_head->jerk_world).norm(), 0.0, 1.0e-7);
  EXPECT_NEAR(old_head->yaw - new_head->yaw, 0.0, 1.0e-7);
  EXPECT_NEAR(old_head->yaw_rate - new_head->yaw_rate, 0.0, 1.0e-7);
  const auto refinement_solve = NavigationRuntimeTerminalMonitorTestPeer::solveGeneration(*node_);
  setTime(kStartNs + 400'000'000LL);
  ASSERT_TRUE(NavigationRuntimeTerminalMonitorTestPeer::publishState(
      *node_, kStartNs + 400'000'000LL));
  const auto next_key = NavigationRuntimeTerminalMonitorTestPeer::key(*node_);
  EXPECT_FALSE(next_key) << "pending successor owns the activation slot; no second request";
  EXPECT_EQ(NavigationRuntimeTerminalMonitorTestPeer::solveGeneration(*node_), refinement_solve);
  EXPECT_EQ(NavigationRuntimeTerminalMonitorTestPeer::timeline(*node_).active, retained);
}

class NavigationRuntimeBaselineRefinementFailed : public NavigationRuntimeBaselineRefinement {
 protected:
  // Existing post-solve diagnostic injection only, default OFF in product.
  std::int64_t failedReplanCycle() const override { return 3; }
};

TEST_F(NavigationRuntimeBaselineRefinementFailed, FailedEarlyReplacementRetainsValidMainWithoutQualityRetry) {
  auto goal = schedulerGoal();
  goal.target.x = 13.1;
  goal.target.y = -2.1;
  goal.route.waypoint_positions.back() = goal.target;
  const auto world = std::make_shared<BaselineRefinementWorld>(kStartNs);
  ASSERT_TRUE(NavigationRuntimeTerminalMonitorTestPeer::prepareInitialBaseline(
      *node_, goal, world, kStartNs));
  const auto initial_key = NavigationRuntimeTerminalMonitorTestPeer::key(*node_);
  ASSERT_TRUE(initial_key);
  NavigationRuntimeTerminalMonitorTestPeer::cycle(*node_, *initial_key);
  const auto initial = NavigationRuntimeTerminalMonitorTestPeer::timeline(*node_);
  ASSERT_TRUE(initial.active);
  ASSERT_EQ(NavigationRuntimeTerminalMonitorTestPeer::completeOutcome(*node_),
            navigation_planning::CompletePlanningOutcome::kBaselineCompleteBundle);
  const auto initial_solve = NavigationRuntimeTerminalMonitorTestPeer::solveGeneration(*node_);
  NavigationRuntimeTerminalMonitorTestPeer::applyQueuedActivations(*node_);
  auto retained = initial.active;
  std::uint64_t revision = 1U;
  for (const auto offset_ns : {100'000'000LL, 200'000'000LL}) {
    setTime(kStartNs + offset_ns);
    ASSERT_TRUE(NavigationRuntimeTerminalMonitorTestPeer::refreshWorld(
        *node_, std::make_shared<BaselineRefinementWorld>(kStartNs + offset_ns, ++revision),
        kStartNs + offset_ns));
    retained = NavigationRuntimeTerminalMonitorTestPeer::timeline(*node_).active;
    ASSERT_TRUE(retained);
    ASSERT_EQ(retained->bundle_generation, initial.active->bundle_generation);
    ASSERT_TRUE(NavigationRuntimeTerminalMonitorTestPeer::reserveFutureAnchor(
        *node_, kStartNs + offset_ns)) << "FIXTURE_BLOCKED: future anchor lease";
    ASSERT_TRUE(NavigationRuntimeTerminalMonitorTestPeer::publishState(
        *node_, kStartNs + offset_ns));
    const auto key = NavigationRuntimeTerminalMonitorTestPeer::key(*node_);
    ASSERT_TRUE(key);
    NavigationRuntimeTerminalMonitorTestPeer::cycle(*node_, *key);
    if (offset_ns == 100'000'000LL) {
      EXPECT_EQ(NavigationRuntimeTerminalMonitorTestPeer::solveGeneration(*node_), initial_solve);
    }
  }
  ASSERT_EQ(NavigationRuntimeTerminalMonitorTestPeer::solveGeneration(*node_), initial_solve + 1U);
  EXPECT_GT(NavigationRuntimeTerminalMonitorTestPeer::optimization(*node_).lbfgs_attempt_count, 0);
  EXPECT_EQ(NavigationRuntimeTerminalMonitorTestPeer::completeOutcome(*node_),
            navigation_planning::CompletePlanningOutcome::kNoCompleteBundle);
  EXPECT_EQ(NavigationRuntimeTerminalMonitorTestPeer::timeline(*node_).active, retained);
  EXPECT_FALSE(NavigationRuntimeTerminalMonitorTestPeer::timeline(*node_).pending);
  const auto after_failure = NavigationRuntimeTerminalMonitorTestPeer::episode(*node_);
  EXPECT_TRUE(after_failure.command_available);
  EXPECT_FALSE(after_failure.failure_latched);
  EXPECT_EQ(after_failure.recovery_state, ExecutionRecoveryState::kTrackMain);
  setTime(kStartNs + 300'000'000LL);
  ASSERT_TRUE(NavigationRuntimeTerminalMonitorTestPeer::publishState(
      *node_, kStartNs + 300'000'000LL));
  const auto retry_key = NavigationRuntimeTerminalMonitorTestPeer::key(*node_);
  ASSERT_TRUE(retry_key);
  NavigationRuntimeTerminalMonitorTestPeer::cycle(*node_, *retry_key);
  EXPECT_EQ(NavigationRuntimeTerminalMonitorTestPeer::solveGeneration(*node_), initial_solve + 1U);
  EXPECT_EQ(NavigationRuntimeTerminalMonitorTestPeer::timeline(*node_).active, retained);
  EXPECT_FALSE(NavigationRuntimeTerminalMonitorTestPeer::timeline(*node_).pending);
}

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

// These are actual-factory boundary schedules, not a replay of native G13 or
// a measured latency bound. A starts 400 ms before the native exact START;
// its genuine SOURCE stays 4 ms before G, and is deliberately unsupported by G.
TEST_F(NavigationRuntimeTerminalMonitor, ExactNativeStartPreservesFreshMainOnlyOwner) {
  ASSERT_NO_FATAL_FAILURE(checkRealFutureHandoff(
      true, false, 55'692'000'000LL, 0LL, 4'000'000LL));
}

TEST_F(NavigationRuntimeTerminalMonitorStrict, ExactNativeStartPreservesFreshMainOnlyOwner) {
  ASSERT_NO_FATAL_FAILURE(checkRealFutureHandoff(
      true, false, 55'692'000'000LL, 0LL, 4'000'000LL));
}

TEST_F(NavigationRuntimeTerminalMonitorObserverOff,
       ExactNativeStartPreservesFreshMainOnlyOwner) {
  ASSERT_NO_FATAL_FAILURE(checkRealFutureHandoff(
      true, false, 55'692'000'000LL, 0LL, 4'000'000LL));
  const auto accounting = NavigationRuntimeTerminalMonitorTestPeer::observationAccounting(*node_);
  EXPECT_EQ(accounting.attempted, 1U);
  EXPECT_EQ(accounting.suppressed, 1U);
  EXPECT_EQ(accounting.published, 0U);
}

TEST_F(NavigationRuntimeTerminalMonitorStrict, OneNanosecondAfterNativeStartPreservesOwner) {
  ASSERT_NO_FATAL_FAILURE(checkRealFutureHandoff(
      true, false, 55'692'000'000LL, 1LL, 4'000'000LL));
}

TEST_F(NavigationRuntimeTerminalMonitorStrict, AfterNativeStartPreservesFreshMainOnlyOwner) {
  ASSERT_NO_FATAL_FAILURE(checkRealFutureHandoff(
      true, false, 55'692'000'000LL, 20'000'000LL, 4'000'000LL));
}

// Unlike the zero-pressure START controls, these require real H preparation.
// Exact integral activation and the later control share A/G/world/SOURCE;
// neither clock conversion may turn the same START into an older H command.
TEST_F(NavigationRuntimeTerminalMonitorStrict,
       ExactFast9StartPressureCanPrepareAndAdmitIndependentBrake) {
  ASSERT_NO_FATAL_FAILURE(checkRealFutureHandoff(
      true, true, 83'316'000'000LL, 0LL, 8'000'000LL));
}

TEST_F(NavigationRuntimeTerminalMonitorStrict,
       MatchedFast9AfterStartPressureCanPrepareAndAdmitIndependentBrake) {
  ASSERT_NO_FATAL_FAILURE(checkRealFutureHandoff(
      true, true, 83'316'000'000LL, 20'000'000LL, 8'000'000LL));
}

TEST_F(NavigationRuntimeTerminalMonitor,
       ExactFast9StartPressureAdmitsBrakeInRelaxedProfile) {
  ASSERT_NO_FATAL_FAILURE(checkRealFutureHandoff(
      true, true, 83'316'000'000LL, 0LL, 8'000'000LL));
}

TEST_F(NavigationRuntimeTerminalMonitorObserverOff,
       ExactFast9StartPressureAdmitsBrakeWithoutObserver) {
  ASSERT_NO_FATAL_FAILURE(checkRealFutureHandoff(
      true, true, 83'316'000'000LL, 0LL, 8'000'000LL));
}

TEST_F(NavigationRuntimeTerminalMonitor,
       PreStartPressureAdmitsIndependentBrakeWithoutInventingSourceWitness) {
  ASSERT_NO_FATAL_FAILURE(checkRealFutureHandoff(true, true));
}

TEST_F(NavigationRuntimeTerminalMonitor,
       MatchedPostStartPressureUsesConfiguredRelaxedBridge) {
  ASSERT_NO_FATAL_FAILURE(checkRealFutureHandoff(false, true));
}

TEST_F(NavigationRuntimeTerminalMonitorStrict,
       PreStartPressureAdmitsIndependentBrakeWithoutInventingSourceWitness) {
  ASSERT_NO_FATAL_FAILURE(checkRealFutureHandoff(true, true));
}

TEST_F(NavigationRuntimeTerminalMonitorStrict,
       MatchedPostStartPressureAdmitsCertifiedBrake) {
  ASSERT_NO_FATAL_FAILURE(checkRealFutureHandoff(false, true));
}

TEST_F(NavigationRuntimeTerminalMonitorObserverOff,
       PreStartPressureAdmitsIndependentBrakeWithoutInventingSourceWitness) {
  ASSERT_NO_FATAL_FAILURE(checkRealFutureHandoff(true, true));
}

TEST_F(NavigationRuntimeTerminalMonitorObserverOff,
       MatchedPostStartPressureUsesConfiguredRelaxedBridge) {
  ASSERT_NO_FATAL_FAILURE(checkRealFutureHandoff(false, true));
}

TEST_F(NavigationRuntimeTerminalMonitorStrict, ImmediateEpisodeCutoverPrecedesAck) {
  ASSERT_NO_FATAL_FAILURE(checkImmediateCutoverBeforeAck(false));
}

TEST_F(NavigationRuntimeTerminalMonitorStrict, ImmediateWorldCopyBeforeAckPreservesDelivery) {
  ASSERT_NO_FATAL_FAILURE(checkImmediateCutoverBeforeAck(true));
}

TEST_F(NavigationRuntimeTerminalMonitorStrict, ImmediatePreparedClockExpiryCannotCommit) {
  ASSERT_NO_FATAL_FAILURE(checkPreparedImmediateInvalidation(ImmediateInvalidation::kClockExpiry));
}

TEST_F(NavigationRuntimeTerminalMonitorStrict, ImmediatePreparedGoalChangeCannotCommit) {
  ASSERT_NO_FATAL_FAILURE(checkPreparedImmediateInvalidation(ImmediateInvalidation::kGoalChange));
}

TEST_F(NavigationRuntimeTerminalMonitorStrict, ImmediatePreparedEpochResetCannotCommit) {
  ASSERT_NO_FATAL_FAILURE(checkPreparedImmediateInvalidation(ImmediateInvalidation::kEpochReset));
}

TEST_F(NavigationRuntimeTerminalMonitorStrict, ImmediatePreparedFailureCannotCommit) {
  ASSERT_NO_FATAL_FAILURE(checkPreparedImmediateInvalidation(ImmediateInvalidation::kFailure));
}

TEST_F(NavigationRuntimeTerminalMonitorStrict, EmergencyEpisodeCutoverPrecedesAckAndWorldCopy) {
  ASSERT_NO_FATAL_FAILURE(installReal({}, Eigen::Vector3d{3.0, 0.0, 0.0}));
  const auto predecessor = NavigationRuntimeTerminalMonitorTestPeer::timeline(*node_).active;
  ASSERT_TRUE(predecessor);
  ASSERT_EQ(NavigationRuntimeTerminalMonitorTestPeer::backendGeneration(*node_),
            predecessor->bundle_generation);
  const auto emergency = NavigationRuntimeTerminalMonitorTestPeer::prepareRealEmergency(*node_);
  ASSERT_TRUE(emergency) << "FIXTURE_BLOCKED: actual emergency factory did not certify H";
  ASSERT_TRUE(emergency->valid());
  ASSERT_EQ(NavigationRuntimeTerminalMonitorTestPeer::admitPreparedEmergency(*node_, *emergency),
            navigation_execution::CommitDecision::kCommitted);
  const auto admitted = NavigationRuntimeTerminalMonitorTestPeer::timeline(*node_).active;
  ASSERT_TRUE(admitted);
  ASSERT_EQ(admitted->bundle_generation, emergency->bundle_generation);
  EXPECT_EQ(NavigationRuntimeTerminalMonitorTestPeer::episode(*node_).active_generation,
            admitted->bundle_generation);
  EXPECT_EQ(NavigationRuntimeTerminalMonitorTestPeer::episode(*node_).recovery_state,
            ExecutionRecoveryState::kEmergencyBrake);
  // Deliberately leave the real backend ACK pending. Mapping's core schedule
  // can validate the staged H and publish a same-generation immutable copy.
  ASSERT_EQ(NavigationRuntimeTerminalMonitorTestPeer::backendGeneration(*node_),
            predecessor->bundle_generation);
  const auto new_world = std::make_shared<SchedulerIdentityWorld>(real_stamp_ns_, 3U);
  ASSERT_TRUE(NavigationRuntimeTerminalMonitorTestPeer::refreshStagedWorldBeforeAck(*node_, new_world));
  const auto refreshed = NavigationRuntimeTerminalMonitorTestPeer::timeline(*node_).active;
  ASSERT_TRUE(refreshed);
  EXPECT_NE(refreshed.get(), admitted.get());
  EXPECT_EQ(refreshed->bundle_generation, admitted->bundle_generation);
  EXPECT_TRUE(navigation_world_model::sameWorldSnapshotIdentity(
      refreshed->world_identity, new_world->identity()));
  EXPECT_EQ(NavigationRuntimeTerminalMonitorTestPeer::episode(*node_).active_generation,
            refreshed->bundle_generation);
  NavigationRuntimeTerminalMonitorTestPeer::acknowledge(*node_);
  EXPECT_EQ(NavigationRuntimeTerminalMonitorTestPeer::backendGeneration(*node_),
            refreshed->bundle_generation);
  NavigationRuntimeTerminalMonitorTestPeer::publishCommandAndApplyQueuedActivations(*node_);
  EXPECT_EQ(NavigationRuntimeTerminalMonitorTestPeer::timeline(*node_).active, refreshed);
  EXPECT_TRUE(NavigationRuntimeTerminalMonitorTestPeer::episode(*node_).command_available);
  EXPECT_FALSE(NavigationRuntimeTerminalMonitorTestPeer::episode(*node_).failure_latched);
}

TEST_F(NavigationRuntimeTerminalMonitorStrict, AdmissionAtOriginalMainEndPreservesItsEpisode) {
  ASSERT_NO_FATAL_FAILURE(installReal({}, Eigen::Vector3d{3.0, 0.0, 0.0}));
  const auto before = NavigationRuntimeTerminalMonitorTestPeer::timeline(*node_);
  const auto episode = NavigationRuntimeTerminalMonitorTestPeer::episode(*node_);
  const auto emergency = NavigationRuntimeTerminalMonitorTestPeer::prepareRealEmergency(*node_);
  ASSERT_TRUE(emergency);
  setTime(before.active->declared_end_ns);
  EXPECT_EQ(NavigationRuntimeTerminalMonitorTestPeer::admitPreparedEmergency(*node_, *emergency),
            navigation_execution::CommitDecision::kAdmissionRejected);
  EXPECT_EQ(NavigationRuntimeTerminalMonitorTestPeer::timeline(*node_).version, before.version);
  EXPECT_EQ(NavigationRuntimeTerminalMonitorTestPeer::episode(*node_).active_generation,
            episode.active_generation);
  EXPECT_FALSE(NavigationRuntimeTerminalMonitorTestPeer::episode(*node_).failure_latched);
}

TEST_F(NavigationRuntimeTerminalMonitorStrict, AdmissionRejectsWrongEpochFrameAndReceiveStaleness) {
  ASSERT_NO_FATAL_FAILURE(installReal({}, Eigen::Vector3d{3.0, 0.0, 0.0}));
  const auto before = NavigationRuntimeTerminalMonitorTestPeer::timeline(*node_);
  const auto emergency = NavigationRuntimeTerminalMonitorTestPeer::prepareRealEmergency(*node_);
  ASSERT_TRUE(emergency);
  const auto healthy = NavigationRuntimeTerminalMonitorTestPeer::stateLease(*node_);
  ASSERT_TRUE(healthy);
  for (const int invalid : {0, 1, 2, 3}) {
    auto lease = std::make_shared<navigation_execution::ExecutionStateLease>(*healthy);
    if (invalid == 0) ++lease->state.localization_epoch;
    if (invalid == 1) lease->state.world_frame_id = "wrong_world";
    if (invalid == 2) lease->state.body_frame_id = "wrong_body";
    if (invalid == 3) lease->state.receive_stamp_ns -= 1'000'000'000LL;
    EXPECT_EQ(NavigationRuntimeTerminalMonitorTestPeer::admitPreparedEmergency(
                  *node_, *emergency, std::move(lease)),
              navigation_execution::CommitDecision::kAdmissionRejected);
    EXPECT_EQ(NavigationRuntimeTerminalMonitorTestPeer::timeline(*node_).version, before.version);
    EXPECT_EQ(NavigationRuntimeTerminalMonitorTestPeer::episode(*node_).active_generation,
              before.active->bundle_generation);
    EXPECT_FALSE(NavigationRuntimeTerminalMonitorTestPeer::episode(*node_).failure_latched);
  }
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

TEST_F(NavigationRuntimeTerminalMonitorStrict, QueuedMonitorAfterBackwardClockCannotMutateOwner) {
  ASSERT_NO_FATAL_FAILURE(installReal());
  const auto before = NavigationRuntimeTerminalMonitorTestPeer::timeline(*node_);
  const auto episode = NavigationRuntimeTerminalMonitorTestPeer::episode(*node_);
  const auto key = NavigationRuntimeTerminalMonitorTestPeer::key(*node_);
  ASSERT_TRUE(key);
  const auto solve_generation = NavigationRuntimeTerminalMonitorTestPeer::solveGeneration(*node_);
  setTime(before.active->declared_start_ns - 1LL);
  NavigationRuntimeTerminalMonitorTestPeer::monitor(*node_, *key);
  const auto after = NavigationRuntimeTerminalMonitorTestPeer::timeline(*node_);
  const auto after_episode = NavigationRuntimeTerminalMonitorTestPeer::episode(*node_);
  EXPECT_EQ(after.version, before.version);
  EXPECT_EQ(after.active, before.active);
  EXPECT_EQ(after_episode.active_generation, episode.active_generation);
  EXPECT_EQ(after_episode.command_available, episode.command_available);
  EXPECT_EQ(after_episode.failure_latched, episode.failure_latched);
  EXPECT_EQ(after_episode.recovery_state, episode.recovery_state);
  EXPECT_EQ(NavigationRuntimeTerminalMonitorTestPeer::solveGeneration(*node_), solve_generation);
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
