#include <gtest/gtest.h>

#include <chrono>
#include <algorithm>
#include <atomic>
#include <condition_variable>
#include <cstring>
#include <cstdlib>
#include <cmath>
#include <future>
#include <filesystem>
#include <limits>
#include <memory>
#include <mutex>
#include <optional>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include <unistd.h>

#include <navigation_contracts/msg/navigation_command.hpp>
#include <navigation_contracts/msg/navigation_goal.hpp>
#include <navigation_contracts/msg/propagated_odometry.hpp>
#include <navigation_contracts/msg/registered_scan.hpp>
#include <rclcpp/executors/multi_threaded_executor.hpp>
#include <sensor_msgs/msg/point_field.hpp>

#include "navigation_runtime/navigation_runtime_node.hpp"
#include "navigation_runtime/runtime_boundaries.hpp"
#include "navigation_runtime/mapping_observation_contract.hpp"

namespace navigation_runtime {

class NavigationRuntimeEpochResetTestPeer {
 public:
  static void scan(NavigationRuntimeNode& node,
                   const navigation_contracts::msg::RegisteredScan& message) {
    node.onRegisteredScan(std::make_shared<const
        navigation_contracts::msg::RegisteredScan>(message));
  }
  static void health(NavigationRuntimeNode& node, std::uint64_t epoch) {
    auto message = std::make_shared<navigation_contracts::msg::EstimatorHealth>();
    message->localization_epoch = epoch;
    node.onEstimatorHealth(message);
  }
  static void goal(NavigationRuntimeNode& node,
                   const navigation_contracts::msg::NavigationGoal& message) {
    node.onGoal(std::make_shared<const
        navigation_contracts::msg::NavigationGoal>(message));
  }
  static void internalGoal(NavigationRuntimeNode& node,
                           const navigation_contracts::msg::NavigationGoal& message) {
    // Execution-only harness: use the same validated transition that Core
    // mission progression invokes, without publishing an external goal.
    std::lock_guard localization_lock(node.localization_transition_mutex_);
    std::lock_guard input_lock(node.input_mutex_);
    node.applyValidatedGoalLocked(std::make_shared<const
        navigation_contracts::msg::NavigationGoal>(message));
  }
  static void terminal(NavigationRuntimeNode& node,
                       const navigation_contracts::msg::NavigationGoal& goal) {
    auto message = std::make_shared<navigation_contracts::msg::NavigationModeStatus>();
    message->header.frame_id = "lio_odom";
    message->header.stamp = node.now();
    message->mission_id = goal.mission_id;
    message->waypoint_index = goal.waypoint_index;
    message->request_id = goal.request_id;
    message->state = navigation_contracts::msg::NavigationModeStatus::COMPLETE;
    node.onModeStatus(message);
  }
  static bool draining(const NavigationRuntimeNode& node) {
    return node.mapping_worker_->resetting();
  }
  static bool ready(const NavigationRuntimeNode& node) {
    return node.localization_epoch_ready_.load(std::memory_order_acquire);
  }
  static auto world(const NavigationRuntimeNode& node) {
    return node.world_snapshot_store_.load();
  }
  static bool ownerMatches(NavigationRuntimeNode& node, std::uint64_t request,
                           bool expect_goal) {
    std::lock_guard localization_lock(node.localization_transition_mutex_);
    std::lock_guard input_lock(node.input_mutex_);
    return node.active_localization_epoch_.load() == 2U &&
        (expect_goal ? node.active_goal_ && node.active_goal_->request_id == request
                     : !node.active_goal_) &&
        !node.execution_authority_.load() &&
        !node.execution_authority_.episodeSnapshot().command_available &&
        node.execution_authority_.executingGoalEpoch() == 0U;
  }
  static void missionState(NavigationRuntimeNode& node, double x,
                           std::int64_t source_stamp_ns) {
    navigation_planning::KinematicState state;
    state.position_world = Eigen::Vector3d{x, 0.0, 3.0};
    state.velocity_world.setZero();
    state.source_stamp_ns = source_stamp_ns;
    state.receive_stamp_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::steady_clock::now().time_since_epoch()).count();
    state.localization_epoch = 1U;
    state.world_frame_id = "lio_odom";
    state.body_frame_id = "base_link";
    ASSERT_TRUE(node.execution_state_store_.publish(std::move(state)));
  }
  static void missionActive(NavigationRuntimeNode& node) {
    auto status = std::make_shared<navigation_contracts::msg::NavigationModeStatus>();
    status->header.frame_id = "lio_odom";
    status->header.stamp = node.now();
    status->state = navigation_contracts::msg::NavigationModeStatus::ACTIVE;
    status->activation_id = 1U;
    status->airborne = true;
    node.onModeStatus(status);
  }
  static void missionPaused(NavigationRuntimeNode& node) {
    auto status = std::make_shared<navigation_contracts::msg::NavigationModeStatus>();
    status->header.frame_id = "lio_odom";
    status->header.stamp = node.now();
    status->state = navigation_contracts::msg::NavigationModeStatus::PAUSED;
    status->activation_id = 1U;
    status->reason = navigation_contracts::msg::NavigationModeStatus::OPERATOR_TAKEOVER;
    node.onModeStatus(status);
  }
  static void missionTick(NavigationRuntimeNode& node) { node.tickMissionProgress(); }
  static MissionGateIdentity missionGate(NavigationRuntimeNode& node) {
    std::lock_guard localization_lock(node.localization_transition_mutex_);
    std::lock_guard input_lock(node.input_mutex_);
    return node.mission_progress_->currentGate();
  }
  static bool missionGoalMatches(NavigationRuntimeNode& node,
                                 std::uint32_t waypoint, std::uint64_t request) {
    std::lock_guard localization_lock(node.localization_transition_mutex_);
    std::lock_guard input_lock(node.input_mutex_);
    return (node.active_goal_ && node.active_goal_->waypoint_index == waypoint &&
            node.active_goal_->request_id == request) ||
           node.pending_goal_owner_.goalMatchesStatus(
               node.mission_progress_->routeIdentity().mission_id, waypoint, request);
  }
  static void missionCommandIssued(NavigationRuntimeNode& node,
                                   const navigation_contracts::msg::NavigationCommand& command) {
    std::lock_guard localization_lock(node.localization_transition_mutex_);
    std::lock_guard input_lock(node.input_mutex_);
    node.rememberMissionCommandIssued(command);
  }
  static void missionCommandAdmitted(
      NavigationRuntimeNode& node,
      const navigation_contracts::msg::NavigationCommandAdmission& receipt) {
    node.onCommandAdmission(std::make_shared<const
        navigation_contracts::msg::NavigationCommandAdmission>(receipt));
  }
};

namespace {

using namespace std::chrono_literals;

class BlockingLifecycleObserver final : public MappingLifecycleObserver {
 public:
  void onMutableMapUpdated(std::int64_t) noexcept override {
    std::unique_lock lock(mutex_);
    map_updated_ = true;
    condition_.notify_all();
    condition_.wait(lock, [this] { return released_; });
  }

  void onShutdownComplete(
      navigation_mapping::ObservationAccounting::Snapshot lifecycle) noexcept override {
    std::lock_guard lock(mutex_);
    shutdown_snapshot_ = lifecycle;
    ++shutdown_count_;
    condition_.notify_all();
  }

  template <typename Rep, typename Period>
  bool waitForMapUpdate(std::chrono::duration<Rep, Period> timeout) {
    std::unique_lock lock(mutex_);
    return condition_.wait_for(lock, timeout, [this] { return map_updated_; });
  }

  void release() noexcept {
    std::lock_guard lock(mutex_);
    released_ = true;
    condition_.notify_all();
  }

  std::optional<navigation_mapping::ObservationAccounting::Snapshot> shutdownSnapshot() const {
    std::lock_guard lock(mutex_);
    return shutdown_snapshot_;
  }

  std::uint64_t shutdownCount() const {
    std::lock_guard lock(mutex_);
    return shutdown_count_;
  }

 private:
  mutable std::mutex mutex_;
  std::condition_variable condition_;
  bool map_updated_{false};
  bool released_{false};
  std::optional<navigation_mapping::ObservationAccounting::Snapshot> shutdown_snapshot_;
  std::uint64_t shutdown_count_{0};
};

struct ObserverReleaseGuard {
  std::shared_ptr<BlockingLifecycleObserver> observer;
  ~ObserverReleaseGuard() { observer->release(); }
};

TEST(NavigationRuntimeTelemetry, KeepsRevalidationCountersFromCurrentUpdate) {
  MappingTelemetry telemetry;
  telemetry.initialize(MappingTelemetrySnapshot{});
  MappingTelemetrySnapshot update;
  update.map.update_outcome = navigation_mapping::MapUpdateOutcome::kUpdated;
  update.command_revalidation_fast_path_count = 3U;
  update.command_revalidation_full_count = 2U;
  telemetry.recordUpdate(update);

  const auto snapshot = telemetry.snapshot();
  EXPECT_EQ(snapshot.command_revalidation_fast_path_count, 3U);
  EXPECT_EQ(snapshot.command_revalidation_full_count, 2U);
}

TEST(NavigationRuntimeBoundaries, MonotonicIdsNeverWrapToZero) {
  std::atomic_uint64_t value{0U};
  ASSERT_EQ(advanceMonotonicId(value), 1U);
  value.store(std::numeric_limits<std::uint64_t>::max() - 1U);
  ASSERT_EQ(advanceMonotonicId(value), std::numeric_limits<std::uint64_t>::max());
  EXPECT_FALSE(advanceMonotonicId(value).has_value());
  EXPECT_EQ(value.load(), std::numeric_limits<std::uint64_t>::max());
}

TEST(NavigationRuntimeBoundaries, RatePeriodMustBePositiveAndRepresentable) {
  EXPECT_FALSE(ratePeriodNanoseconds(0.0).has_value());
  EXPECT_FALSE(ratePeriodNanoseconds(-1.0).has_value());
  EXPECT_FALSE(ratePeriodNanoseconds(std::numeric_limits<double>::infinity()).has_value());
  EXPECT_FALSE(ratePeriodNanoseconds(std::numeric_limits<double>::denorm_min()).has_value());
  EXPECT_FALSE(ratePeriodNanoseconds(1.0e10).has_value());
  ASSERT_TRUE(ratePeriodNanoseconds(50.0).has_value());
  EXPECT_EQ(*ratePeriodNanoseconds(50.0), 20ms);
}

TEST(NavigationRuntimeBoundaries, TrajectorySamplingCapsBeforeFloatingPointCast) {
  EXPECT_EQ(boundedTrajectorySampleCount(0.16, 0.08, 64U), 3U);
  EXPECT_EQ(boundedTrajectorySampleCount(2.999, 1.0, 4U), 4U);
  EXPECT_EQ(boundedTrajectorySampleCount(std::numeric_limits<double>::max(), 0.08, 64U),
            64U);
  EXPECT_FALSE(boundedTrajectorySampleCount(1.0, 0.0, 64U).has_value());
}

TEST(NavigationRuntimeCadence, RejectsPlannerPeriodShorterThanSolveBudget) {
  // Keep this contract test at the pure boundary: constructing a Node solely
  // to provoke an exception causes rclcpp Context double-shutdown during
  // exception unwinding (verified by ASan), obscuring the product predicate.
  EXPECT_FALSE(plannerPeriodCoversSolveBudget(10.0, 0.18));
  EXPECT_TRUE(plannerPeriodCoversSolveBudget(
      navigation_planning::PlanningTimingContract::kPlannerRateHz,
      navigation_planning::PlanningTimingContract::kSolveDeadlineS));
  EXPECT_TRUE(plannerSolveDeadlineMatchesContract(
      navigation_planning::PlanningTimingContract::kSolveDeadlineS));
  EXPECT_FALSE(plannerSolveDeadlineMatchesContract(0.18));
}

TEST(NavigationRuntimeBoundaries, ClassifiesSensorOriginContract) {
  geometry_msgs::msg::Pose origin;
  origin.orientation.w = 1.0;
  EXPECT_EQ(
      classifySensorOriginContract(true, origin),
      navigation_mapping::MappingObservationRejectionReason::kNone);
  EXPECT_EQ(
      classifySensorOriginContract(false, origin),
      navigation_mapping::MappingObservationRejectionReason::kMissingSensorOrigin);
  origin.position.x = std::numeric_limits<double>::quiet_NaN();
  EXPECT_EQ(
      classifySensorOriginContract(true, origin),
      navigation_mapping::MappingObservationRejectionReason::kSensorOriginContractMismatch);
  origin.position.x = 0.0;
  origin.orientation.w = 0.0;
  EXPECT_EQ(
      classifySensorOriginContract(true, origin),
      navigation_mapping::MappingObservationRejectionReason::kSensorOriginContractMismatch);
}

class ExecutorStopGuard {
 public:
  ExecutorStopGuard(rclcpp::Executor& executor, std::thread& thread)
      : executor_(executor), thread_(thread) {}
  ~ExecutorStopGuard() { stop(); }
  void stop() {
    executor_.cancel();
    if (thread_.joinable()) thread_.join();
  }
 private:
  rclcpp::Executor& executor_;
  std::thread& thread_;
};

sensor_msgs::msg::PointCloud2 makeCloud(const builtin_interfaces::msg::Time& stamp) {
  sensor_msgs::msg::PointCloud2 cloud;
  cloud.header.stamp = stamp;
  cloud.header.frame_id = "lio_odom";
  cloud.height = 1;
  cloud.width = 1;
  cloud.is_bigendian = false;
  cloud.is_dense = true;
  cloud.point_step = 3U * sizeof(float);
  cloud.row_step = cloud.point_step;
  cloud.fields.resize(3);
  for (std::size_t axis = 0; axis < cloud.fields.size(); ++axis) {
    cloud.fields[axis].name = std::string(1, "xyz"[axis]);
    cloud.fields[axis].offset = static_cast<std::uint32_t>(axis * sizeof(float));
    cloud.fields[axis].datatype = sensor_msgs::msg::PointField::FLOAT32;
    cloud.fields[axis].count = 1;
  }
  const float point[3]{1.0F, 0.0F, 0.0F};
  cloud.data.resize(sizeof(point));
  std::memcpy(cloud.data.data(), point, sizeof(point));
  return cloud;
}

navigation_contracts::msg::RegisteredScan makeRegisteredScan(
    const builtin_interfaces::msg::Time& stamp) {
  navigation_contracts::msg::RegisteredScan observation;
  observation.header.stamp = stamp;
  observation.header.frame_id = "lio_odom";
  observation.localization_epoch = 1U;
  observation.scan_sequence = 1U;
  observation.body_frame_id = "base_link";
  observation.corrected_pose.pose.orientation.w = 1.0;
  observation.sensor_origin_pose.orientation.w = 1.0;
  observation.sensor_origin_valid = true;
  observation.points = makeCloud(stamp);
  return observation;
}

sensor_msgs::msg::PointCloud2 makeHandoverCloud(
    const builtin_interfaces::msg::Time& stamp) {
  sensor_msgs::msg::PointCloud2 cloud;
  cloud.header.stamp = stamp;
  cloud.header.frame_id = "lio_odom";
  cloud.height = 1;
  cloud.width = 0;
  cloud.is_bigendian = false;
  cloud.is_dense = true;
  cloud.point_step = 3U * sizeof(float);
  cloud.fields.resize(3);
  for (std::size_t axis = 0; axis < cloud.fields.size(); ++axis) {
    cloud.fields[axis].name = std::string(1, "xyz"[axis]);
    cloud.fields[axis].offset = static_cast<std::uint32_t>(axis * sizeof(float));
    cloud.fields[axis].datatype = sensor_msgs::msg::PointField::FLOAT32;
    cloud.fields[axis].count = 1;
  }
  for (int y_index = -10; y_index <= 10; ++y_index) {
    for (int z_index = -4; z_index <= 6; ++z_index) {
      const float point[3]{8.0F, static_cast<float>(y_index) * 0.4F,
                           0.3F + static_cast<float>(z_index) * 0.4F};
      const auto offset = cloud.data.size();
      cloud.data.resize(offset + sizeof(point));
      std::memcpy(cloud.data.data() + offset, point, sizeof(point));
      ++cloud.width;
    }
  }
  cloud.row_step = cloud.width * cloud.point_step;
  return cloud;
}

navigation_contracts::msg::RegisteredScan makeHandoverScan(
    const builtin_interfaces::msg::Time& stamp, const std::uint64_t sequence) {
  auto observation = makeRegisteredScan(stamp);
  observation.scan_sequence = sequence;
  observation.points = makeHandoverCloud(stamp);
  observation.corrected_pose.pose.position.z = 0.3;
  observation.sensor_origin_pose.position.z = 0.3;
  return observation;
}

navigation_contracts::msg::PropagatedOdometry makeHandoverOdometry(
    const builtin_interfaces::msg::Time& stamp, const std::uint64_t sequence,
    const double measured_x, const double measured_velocity) {
  navigation_contracts::msg::PropagatedOdometry message;
  message.odometry.header.stamp = stamp;
  message.odometry.header.frame_id = "lio_odom";
  message.odometry.child_frame_id = "base_link";
  message.odometry.pose.pose.orientation.w = 1.0;
  message.odometry.pose.pose.position.x = measured_x;
  message.odometry.pose.pose.position.z = 0.3;
  message.odometry.twist.twist.linear.x = measured_velocity;
  message.localization_epoch = 1U;
  message.sequence = sequence;
  return message;
}

navigation_contracts::msg::NavigationGoal makeHandoverGoal(
    const builtin_interfaces::msg::Time& stamp, const std::uint64_t request_id,
    const std::uint32_t waypoint_index, const std::uint64_t route_revision,
    const std::uint8_t behavior) {
  navigation_contracts::msg::NavigationGoal goal;
  goal.header.stamp = stamp;
  goal.header.frame_id = "lio_odom";
  goal.mission_id = "completed_handover";
  goal.waypoint_index = waypoint_index;
  goal.request_id = request_id;
  goal.target.x = behavior == navigation_contracts::msg::NavigationGoal::BEHAVIOR_STOP
      ? (request_id == 11U ? 6.0 : 3.5) : 3.0;
  goal.target.z = 0.3;
  goal.acceptance_radius_m = behavior == navigation_contracts::msg::NavigationGoal::BEHAVIOR_STOP
      ? 0.5 : 0.2;
  goal.behavior = behavior;
  goal.has_next_target = waypoint_index == 1U &&
      behavior == navigation_contracts::msg::NavigationGoal::BEHAVIOR_PASS_THROUGH;
  if (goal.has_next_target) goal.next_target = goal.target;
  auto& route = goal.route;
  route.mission_id = goal.mission_id;
  route.frame_id = goal.header.frame_id;
  route.route_revision = route_revision;
  route.request_id = request_id;
  route.active_waypoint_index = waypoint_index;
  geometry_msgs::msg::Point origin;
  origin.z = 0.3;
  if (behavior == navigation_contracts::msg::NavigationGoal::BEHAVIOR_STOP) {
    route.waypoint_positions = {origin, goal.target};
    route.waypoint_ids = {"origin", "stop"};
    route.waypoint_acceptance_radii_m = {0.2, goal.acceptance_radius_m};
    route.waypoint_behaviors = {
        navigation_contracts::msg::RouteSnapshot::BEHAVIOR_PASS_THROUGH,
        navigation_contracts::msg::RouteSnapshot::BEHAVIOR_STOP};
  } else {
    route.waypoint_positions = {origin, goal.target, goal.target};
    route.waypoint_ids = {"origin", "completed", "stop"};
    route.waypoint_acceptance_radii_m = {
        0.2, goal.acceptance_radius_m, goal.acceptance_radius_m};
    route.waypoint_behaviors = {
        navigation_contracts::msg::RouteSnapshot::BEHAVIOR_PASS_THROUGH,
        navigation_contracts::msg::RouteSnapshot::BEHAVIOR_PASS_THROUGH,
        navigation_contracts::msg::RouteSnapshot::BEHAVIOR_STOP};
  }
  route.measured_progress_valid = true;
  route.measured_segment_index = 0U;
  route.measured_progress_arc_m = 0.0;
  route.measured_projection_arc_m = 0.0;
  route.measured_lateral_error_m = 0.0;
  return goal;
}

// No ROS transport timing assumptions: invoke the actual ingress callbacks,
// pause the actual map owner, and observe its reset barrier before releasing it.
// The separate transport/executor tests below retain their existing scope.
void exerciseEpochDrainWithConcurrentGoal(const bool send_terminal) {
  alarm(15);
  auto context = std::make_shared<rclcpp::Context>();
  context->init(0, nullptr);
  rclcpp::NodeOptions options;
  options.context(context);
  options.parameter_overrides({
      rclcpp::Parameter("navigation_runtime.planning_frame", "lio_odom"),
      rclcpp::Parameter("navigation_runtime.body_frame_id", "base_link"),
      rclcpp::Parameter("navigation_runtime.deployment_profile", "sitl"),
      rclcpp::Parameter("navigation_runtime.config_path", NAVIGATION_PLANNER_CONFIG_PATH),
  });
  auto observer = std::make_shared<BlockingLifecycleObserver>();
  auto node = std::make_shared<NavigationRuntimeNode>(
      options, NavigationRuntimeDependencies{observer});
  const auto old_world = NavigationRuntimeEpochResetTestPeer::world(*node);
  NavigationRuntimeEpochResetTestPeer::scan(*node, makeRegisteredScan(node->now()));
  if (!observer->waitForMapUpdate(5s)) _exit(20);
  auto reset = std::async(std::launch::async, [&] {
    NavigationRuntimeEpochResetTestPeer::health(*node, 2U);
  });
  const auto deadline = std::chrono::steady_clock::now() + 5s;
  while (!NavigationRuntimeEpochResetTestPeer::draining(*node)) {
    if (std::chrono::steady_clock::now() >= deadline) _exit(21);
    std::this_thread::yield();
  }
  if (NavigationRuntimeEpochResetTestPeer::ready(*node)) _exit(22);
  const auto goal = makeHandoverGoal(node->now(), 10U, 1U, 1U,
      navigation_contracts::msg::NavigationGoal::BEHAVIOR_STOP);
  NavigationRuntimeEpochResetTestPeer::goal(*node, goal);
  if (!NavigationRuntimeEpochResetTestPeer::ownerMatches(*node, 10U, true)) _exit(23);
  if (send_terminal) NavigationRuntimeEpochResetTestPeer::terminal(*node, goal);
  observer->release();
  if (reset.wait_for(5s) != std::future_status::ready) _exit(24);
  reset.get();
  if (NavigationRuntimeEpochResetTestPeer::ready(*node) ||
      NavigationRuntimeEpochResetTestPeer::world(*node).view != old_world.view ||
      !NavigationRuntimeEpochResetTestPeer::ownerMatches(*node, 10U, !send_terminal)) {
    _exit(25);
  }
  auto scan = makeRegisteredScan(node->now());
  scan.localization_epoch = 2U;
  NavigationRuntimeEpochResetTestPeer::scan(*node, scan);
  const auto new_world_deadline = std::chrono::steady_clock::now() + 5s;
  while (NavigationRuntimeEpochResetTestPeer::world(*node).identity.localization_epoch != 2U) {
    if (std::chrono::steady_clock::now() >= new_world_deadline) _exit(26);
    std::this_thread::yield();
  }
  // Acquiring the same owner lock orders this observation after publication's
  // readiness write, even if the immutable world pointer was observed first.
  if (!NavigationRuntimeEpochResetTestPeer::ownerMatches(*node, 10U, !send_terminal) ||
      !NavigationRuntimeEpochResetTestPeer::ready(*node)) _exit(27);
  node.reset();
  const auto accounting = observer->shutdownSnapshot();
  if (!accounting || !accounting->allInvariantsHold() ||
      accounting->mapping_failed != 0U || accounting->pending != 0U) _exit(28);
  alarm(0);
  _exit(0);
}

TEST(NavigationRuntimeEpochReset, GoalAcceptedDuringDrainSurvivesOldMappingCallback) {
  ASSERT_EXIT(exerciseEpochDrainWithConcurrentGoal(false), testing::ExitedWithCode(0), "");
}

TEST(NavigationRuntimeMissionCut, InitialMeasuredPassCreatesSuccessorInsideCore) {
  auto context = std::make_shared<rclcpp::Context>();
  context->init(0, nullptr);
  rclcpp::NodeOptions options;
  options.context(context);
  options.parameter_overrides({
      rclcpp::Parameter("navigation_runtime.planning_frame", "lio_odom"),
      rclcpp::Parameter("navigation_runtime.body_frame_id", "base_link"),
      rclcpp::Parameter("navigation_runtime.deployment_profile", "sitl"),
      rclcpp::Parameter("navigation_runtime.config_path", NAVIGATION_PLANNER_CONFIG_PATH),
      rclcpp::Parameter("navigation_runtime.mission_file",
                        NAVIGATION_HANDOVER_MISSION_FILE_PATH),
  });
  auto node = std::make_shared<NavigationRuntimeNode>(options);
  NavigationRuntimeEpochResetTestPeer::missionTick(*node);  // Bind localization epoch.
  NavigationRuntimeEpochResetTestPeer::missionState(*node, 0.0,
                                                    node->now().nanoseconds());
  NavigationRuntimeEpochResetTestPeer::missionActive(*node);
  NavigationRuntimeEpochResetTestPeer::missionTick(*node);
  EXPECT_TRUE(NavigationRuntimeEpochResetTestPeer::missionGoalMatches(*node, 0U, 1U));
  std::this_thread::sleep_for(2ms);
  NavigationRuntimeEpochResetTestPeer::missionState(*node, 0.0,
                                                    node->now().nanoseconds());
  NavigationRuntimeEpochResetTestPeer::missionTick(*node);
  EXPECT_EQ(NavigationRuntimeEpochResetTestPeer::missionGate(*node).waypoint_index, 1U);
  EXPECT_TRUE(NavigationRuntimeEpochResetTestPeer::missionGoalMatches(*node, 1U, 2U));
  navigation_contracts::msg::NavigationCommand issued;
  issued.header.frame_id = "lio_odom";
  issued.header.stamp = node->now();
  issued.valid_until = rclcpp::Time(node->now().nanoseconds() + 100'000'000, RCL_SYSTEM_TIME);
  issued.execution_authorization = navigation_contracts::msg::NavigationCommand::
      EXECUTION_AUTHORIZATION_GRANTED;
  issued.mission_id = "external_mode_open_route";
  issued.localization_epoch = 1U;
  issued.goal_epoch = 2U;
  issued.mode_activation_id = 1U;
  issued.waypoint_index = 1U;
  issued.request_id = 2U;
  issued.bundle_generation = 4U;
  issued.sample_id = 11U;
  issued.role = navigation_contracts::msg::NavigationCommand::ROLE_MAIN;
  issued.status = navigation_contracts::msg::NavigationCommand::STATUS_READY;
  issued.certified_main_continuation = true;
  issued.continuation_boundary_stamp_ns =
      static_cast<std::uint64_t>(node->now().nanoseconds());
  NavigationRuntimeEpochResetTestPeer::missionCommandIssued(*node, issued);
  std::this_thread::sleep_for(2ms);
  NavigationRuntimeEpochResetTestPeer::missionState(*node, 3.0,
                                                    node->now().nanoseconds());
  NavigationRuntimeEpochResetTestPeer::missionTick(*node);
  EXPECT_EQ(NavigationRuntimeEpochResetTestPeer::missionGate(*node).waypoint_index, 1U);
  navigation_contracts::msg::NavigationCommandAdmission admitted;
  admitted.header.frame_id = "lio_odom";
  admitted.header.stamp = node->now();
  admitted.mode_activation_id = 1U;
  admitted.mission_id = issued.mission_id;
  admitted.localization_epoch = issued.localization_epoch;
  admitted.goal_epoch = issued.goal_epoch;
  admitted.waypoint_index = issued.waypoint_index;
  admitted.request_id = issued.request_id;
  admitted.bundle_generation = issued.bundle_generation;
  admitted.sample_id = issued.sample_id;
  auto foreign_activation = admitted;
  foreign_activation.mode_activation_id = 2U;
  NavigationRuntimeEpochResetTestPeer::missionCommandAdmitted(*node, foreign_activation);
  EXPECT_EQ(NavigationRuntimeEpochResetTestPeer::missionGate(*node).waypoint_index, 1U);
  NavigationRuntimeEpochResetTestPeer::missionCommandAdmitted(*node, admitted);
  EXPECT_EQ(NavigationRuntimeEpochResetTestPeer::missionGate(*node).waypoint_index, 2U);
  NavigationRuntimeEpochResetTestPeer::missionCommandAdmitted(*node, admitted);
  EXPECT_EQ(NavigationRuntimeEpochResetTestPeer::missionGate(*node).waypoint_index, 2U);
  const auto request_before_takeover =
      NavigationRuntimeEpochResetTestPeer::missionGate(*node).request_id;
  NavigationRuntimeEpochResetTestPeer::missionPaused(*node);
  NavigationRuntimeEpochResetTestPeer::missionActive(*node);  // Delayed old heartbeat.
  NavigationRuntimeEpochResetTestPeer::missionTick(*node);
  EXPECT_EQ(NavigationRuntimeEpochResetTestPeer::missionGate(*node).request_id,
            request_before_takeover);
  node.reset();
  context->shutdown("mission cut component test complete");
}

TEST(NavigationRuntimeMissionCut,
     EstablishedActivationKeepsMeasuredCrossingWhenModeHeartbeatIsLate) {
  auto context = std::make_shared<rclcpp::Context>();
  context->init(0, nullptr);
  rclcpp::NodeOptions options;
  options.context(context);
  options.parameter_overrides({
      rclcpp::Parameter("navigation_runtime.planning_frame", "lio_odom"),
      rclcpp::Parameter("navigation_runtime.body_frame_id", "base_link"),
      rclcpp::Parameter("navigation_runtime.deployment_profile", "sitl"),
      rclcpp::Parameter("navigation_runtime.config_path", NAVIGATION_PLANNER_CONFIG_PATH),
      rclcpp::Parameter("navigation_runtime.mission_file",
                        NAVIGATION_HANDOVER_MISSION_FILE_PATH),
  });
  auto node = std::make_shared<NavigationRuntimeNode>(options);
  NavigationRuntimeEpochResetTestPeer::missionTick(*node);
  NavigationRuntimeEpochResetTestPeer::missionState(*node, 0.0,
                                                    node->now().nanoseconds());
  NavigationRuntimeEpochResetTestPeer::missionActive(*node);
  NavigationRuntimeEpochResetTestPeer::missionTick(*node);
  std::this_thread::sleep_for(2ms);
  NavigationRuntimeEpochResetTestPeer::missionState(*node, 0.0,
                                                    node->now().nanoseconds());
  NavigationRuntimeEpochResetTestPeer::missionTick(*node);
  ASSERT_EQ(NavigationRuntimeEpochResetTestPeer::missionGate(*node).waypoint_index, 1U);

  // The adapter may continue accepting 100 ms leased commands while one
  // ModeStatus heartbeat is delayed. Keep the source-time crossing pair
  // observed by Core; the fresh exact admission receipt still gates acceptance.
  std::this_thread::sleep_for(100ms);
  NavigationRuntimeEpochResetTestPeer::missionState(*node, 0.0,
                                                    node->now().nanoseconds());
  NavigationRuntimeEpochResetTestPeer::missionTick(*node);
  std::this_thread::sleep_for(125ms);
  NavigationRuntimeEpochResetTestPeer::missionState(*node, 2.8,
                                                    node->now().nanoseconds());
  NavigationRuntimeEpochResetTestPeer::missionTick(*node);
  ASSERT_EQ(NavigationRuntimeEpochResetTestPeer::missionGate(*node).waypoint_index, 1U);
  navigation_contracts::msg::NavigationCommand issued;
  issued.header.frame_id = "lio_odom";
  issued.header.stamp = node->now();
  issued.valid_until = rclcpp::Time(node->now().nanoseconds() + 100'000'000,
                                    RCL_SYSTEM_TIME);
  issued.execution_authorization = navigation_contracts::msg::NavigationCommand::
      EXECUTION_AUTHORIZATION_GRANTED;
  issued.mission_id = "external_mode_open_route";
  issued.localization_epoch = 1U;
  issued.goal_epoch = 2U;
  issued.mode_activation_id = 1U;
  issued.waypoint_index = 1U;
  issued.request_id = 2U;
  issued.bundle_generation = 4U;
  issued.sample_id = 11U;
  issued.role = navigation_contracts::msg::NavigationCommand::ROLE_MAIN;
  issued.status = navigation_contracts::msg::NavigationCommand::STATUS_READY;
  issued.certified_main_continuation = true;
  issued.continuation_boundary_stamp_ns =
      static_cast<std::uint64_t>(node->now().nanoseconds());
  NavigationRuntimeEpochResetTestPeer::missionCommandIssued(*node, issued);
  navigation_contracts::msg::NavigationCommandAdmission receipt;
  receipt.header.frame_id = "lio_odom";
  receipt.header.stamp = node->now();
  receipt.mode_activation_id = 1U;
  receipt.mission_id = issued.mission_id;
  receipt.localization_epoch = issued.localization_epoch;
  receipt.goal_epoch = issued.goal_epoch;
  receipt.waypoint_index = issued.waypoint_index;
  receipt.request_id = issued.request_id;
  receipt.bundle_generation = issued.bundle_generation;
  receipt.sample_id = issued.sample_id;
  NavigationRuntimeEpochResetTestPeer::missionCommandAdmitted(*node, receipt);
  EXPECT_EQ(NavigationRuntimeEpochResetTestPeer::missionGate(*node).waypoint_index, 2U);
  node.reset();
  context->shutdown("late mode heartbeat mission test complete");
}

TEST(NavigationRuntimeEpochReset, TerminalDuringDrainCannotBeResurrectedByNewWorld) {
  ASSERT_EXIT(exerciseEpochDrainWithConcurrentGoal(true), testing::ExitedWithCode(0), "");
}

TEST(NavigationRuntimeShutdown, JoinsAnInflightRealMapUpdateBeforeDestruction) {
  const auto process_suffix = std::to_string(static_cast<long long>(::getpid()));
  const std::filesystem::path ros_log_directory =
      std::filesystem::path("/tmp") /
      ("uav-navigation-test-ros-logs_" + process_suffix);
  std::filesystem::create_directories(ros_log_directory);
  ASSERT_EQ(setenv("ROS_LOG_DIR", ros_log_directory.c_str(), 1), 0);
  auto context = std::make_shared<rclcpp::Context>();
  context->init(0, nullptr);
  auto observer = std::make_shared<BlockingLifecycleObserver>();

  const std::string prefix = "/shutdown_contract_" + process_suffix;
  rclcpp::NodeOptions options;
  options.context(context);
  options.parameter_overrides({
      rclcpp::Parameter("navigation_runtime.registered_scan_topic", prefix + "/observation"),
      rclcpp::Parameter("navigation_runtime.propagated_odometry_topic", prefix + "/propagated"),
      rclcpp::Parameter("navigation_runtime.goal_topic", prefix + "/goal"),
      rclcpp::Parameter("navigation_runtime.status_topic", prefix + "/status"),
      rclcpp::Parameter("navigation_runtime.command_topic", prefix + "/command"),
      rclcpp::Parameter("navigation_runtime.planning_frame", "lio_odom"),
      rclcpp::Parameter("navigation_runtime.deployment_profile", "sitl"),
      rclcpp::Parameter("navigation_runtime.config_path", NAVIGATION_PLANNER_CONFIG_PATH),
  });

  auto navigation = std::make_shared<NavigationRuntimeNode>(
      options, NavigationRuntimeDependencies{observer});
  ObserverReleaseGuard release_guard{observer};
  auto driver = std::make_shared<rclcpp::Node>(
      "shutdown_contract_driver_" + process_suffix, options);
  const auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).best_effort().durability_volatile();
  auto observation_publisher =
      driver->create_publisher<navigation_contracts::msg::RegisteredScan>(
          prefix + "/observation", qos);

  rclcpp::ExecutorOptions executor_options;
  executor_options.context = context;
  rclcpp::executors::MultiThreadedExecutor executor(executor_options, 2);
  executor.add_node(navigation);
  executor.add_node(driver);
  std::thread spin_thread([&executor] { executor.spin(); });
  ExecutorStopGuard executor_guard(executor, spin_thread);

  const auto test_deadline = std::chrono::steady_clock::now() + 5s;
  while (observation_publisher->get_subscription_count() == 0U &&
         std::chrono::steady_clock::now() < test_deadline) {
    std::this_thread::sleep_for(10ms);
  }
  ASSERT_GT(observation_publisher->get_subscription_count(), 0U);

  bool map_updated = false;
  while (!map_updated && std::chrono::steady_clock::now() < test_deadline) {
    // The production topic is best-effort and volatile.  Keep sending a
    // fresh timestamp until the observer sees the one accepted update.  The
    // fixed sequence makes retries idempotent after the first delivery.
    const auto stamp = driver->now();
    ASSERT_GT(rclcpp::Time(stamp).nanoseconds(), 0);
    observation_publisher->publish(makeRegisteredScan(stamp));
    const auto remaining = test_deadline - std::chrono::steady_clock::now();
    map_updated = remaining > 10ms
        ? observer->waitForMapUpdate(10ms)
        : observer->waitForMapUpdate(remaining);
  }
  ASSERT_TRUE(map_updated);

  executor_guard.stop();
  executor.remove_node(navigation);
  executor.remove_node(driver);

  std::weak_ptr<NavigationRuntimeNode> weak_navigation = navigation;
  auto destroy = std::async(std::launch::async,
      [owned = std::move(navigation)]() mutable { owned.reset(); });
  EXPECT_EQ(destroy.wait_for(100ms), std::future_status::timeout);

  observer->release();
  ASSERT_EQ(destroy.wait_for(5s), std::future_status::ready);
  destroy.get();
  EXPECT_TRUE(weak_navigation.expired());

  const auto lifecycle = observer->shutdownSnapshot();
  ASSERT_TRUE(lifecycle.has_value());
  EXPECT_EQ(observer->shutdownCount(), 1U);
  EXPECT_GE(lifecycle->received, 1U);
  EXPECT_EQ(lifecycle->rejected_before_inbox, lifecycle->received - 1U);
  EXPECT_EQ(lifecycle->accepted_to_inbox, 1U);
  EXPECT_EQ(lifecycle->mapping_started, 1U);
  EXPECT_EQ(lifecycle->mapping_published, 1U);
  EXPECT_EQ(lifecycle->mapping_failed, 0U);
  EXPECT_EQ(lifecycle->waiting, 0U);
  EXPECT_EQ(lifecycle->ready, 0U);
  EXPECT_EQ(lifecycle->in_flight, 0U);
  EXPECT_EQ(lifecycle->pending, 0U);
  EXPECT_EQ(lifecycle->violation_count, 0U);
  EXPECT_TRUE(lifecycle->allInvariantsHold());

  driver.reset();
  // The Context destructor owns the single shutdown after node/worker teardown.
}

TEST(NavigationRuntimeHandover, DispatchesNewStopAfterCompletedTerminalCommand) {
  const auto process_suffix = std::to_string(static_cast<long long>(::getpid()));
  auto context = std::make_shared<rclcpp::Context>();
  context->init(0, nullptr);
  const std::string prefix = "/completed_handover_" + process_suffix;
  rclcpp::NodeOptions options;
  options.context(context);
  options.parameter_overrides({
      rclcpp::Parameter("navigation_runtime.registered_scan_topic", prefix + "/observation"),
      rclcpp::Parameter("navigation_runtime.propagated_odometry_topic", prefix + "/propagated"),
      rclcpp::Parameter("navigation_runtime.goal_topic", prefix + "/goal"),
      rclcpp::Parameter("navigation_runtime.status_topic", prefix + "/status"),
      rclcpp::Parameter("navigation_runtime.command_topic", prefix + "/command"),
      rclcpp::Parameter("navigation_runtime.planning_frame", "lio_odom"),
      rclcpp::Parameter("navigation_runtime.body_frame_id", "base_link"),
      rclcpp::Parameter("navigation_runtime.deployment_profile", "sitl"),
      rclcpp::Parameter("navigation_runtime.config_path", NAVIGATION_PLANNER_CONFIG_PATH),
      rclcpp::Parameter("navigation_runtime.mission_file",
                        NAVIGATION_HANDOVER_MISSION_FILE_PATH),
  });
  auto navigation = std::make_shared<NavigationRuntimeNode>(options);
  auto driver = std::make_shared<rclcpp::Node>(
      "completed_handover_driver_" + process_suffix, options);
  const auto sensor_qos = rclcpp::QoS(rclcpp::KeepLast(1)).best_effort().durability_volatile();
  const auto goal_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable();
  auto observation_publisher = driver->create_publisher<
      navigation_contracts::msg::RegisteredScan>(prefix + "/observation", sensor_qos);
  auto odometry_publisher = driver->create_publisher<
      navigation_contracts::msg::PropagatedOdometry>(prefix + "/propagated", sensor_qos);
  std::mutex samples_mutex;
  std::condition_variable samples_condition;
  std::vector<navigation_contracts::msg::NavigationCommand> commands;
  std::atomic<double> measured_x{0.0};
  std::atomic<double> measured_velocity{0.0};
  std::atomic_bool terminal_adjacent{false};
  std::atomic_bool completed_witness{false};
  std::atomic_bool successor_published{false};
  std::atomic<double> completed_position_x{std::numeric_limits<double>::quiet_NaN()};
  std::atomic_bool successor_odom_recorded{false};
  std::atomic<double> first_successor_odom_x{std::numeric_limits<double>::quiet_NaN()};
  std::atomic_bool successor_command_recorded{false};
  std::mutex successor_command_mutex;
  std::optional<navigation_contracts::msg::NavigationCommand> first_successor_command;
  auto command_subscription = driver->create_subscription<
      navigation_contracts::msg::NavigationCommand>(
      prefix + "/command", goal_qos,
      [&](navigation_contracts::msg::NavigationCommand::ConstSharedPtr message) {
        if (!message) return;
        if (message->request_id == 10U) {
          const double speed = std::hypot(
              message->velocity.x, std::hypot(message->velocity.y, message->velocity.z));
          const bool completed_terminal =
              message->status == navigation_contracts::msg::NavigationCommand::STATUS_COMPLETED &&
              message->position.x > 2.5 && speed <= 0.15;
          const bool completed_terminal_role =
              message->role == navigation_contracts::msg::NavigationCommand::ROLE_MAIN ||
              (message->role == navigation_contracts::msg::NavigationCommand::ROLE_BACKUP &&
               message->backup_available);
          if (!completed_witness.load(std::memory_order_acquire)) {
            measured_x.store(message->position.x, std::memory_order_release);
            measured_velocity.store(message->velocity.x, std::memory_order_release);
          }
          if (completed_terminal_role && completed_terminal) {
            terminal_adjacent.store(true, std::memory_order_release);
            completed_position_x.store(message->position.x, std::memory_order_release);
            completed_witness.store(true, std::memory_order_release);
          }
        } else if (message->request_id == 11U &&
                   message->status == navigation_contracts::msg::NavigationCommand::STATUS_READY &&
                   message->role == navigation_contracts::msg::NavigationCommand::ROLE_MAIN &&
                   !successor_command_recorded.exchange(true, std::memory_order_acq_rel)) {
          std::lock_guard lock(successor_command_mutex);
          first_successor_command = *message;
        }
        std::lock_guard lock(samples_mutex);
        commands.push_back(*message);
        samples_condition.notify_all();
      });
  (void)command_subscription;
  rclcpp::ExecutorOptions executor_options;
  executor_options.context = context;
  rclcpp::executors::MultiThreadedExecutor executor(executor_options, 4);
  executor.add_node(navigation);
  executor.add_node(driver);
  std::thread spin_thread([&executor] { executor.spin(); });
  ExecutorStopGuard executor_guard(executor, spin_thread);

  const auto subscriptions_deadline = std::chrono::steady_clock::now() + 5s;
  while ((observation_publisher->get_subscription_count() == 0U ||
          odometry_publisher->get_subscription_count() == 0U ||
          driver->count_publishers(prefix + "/command") == 0U) &&
         std::chrono::steady_clock::now() < subscriptions_deadline) {
    std::this_thread::sleep_for(10ms);
  }
  ASSERT_GT(observation_publisher->get_subscription_count(), 0U);
  ASSERT_GT(odometry_publisher->get_subscription_count(), 0U);
  ASSERT_GT(driver->count_publishers(prefix + "/command"), 0U);

  const auto initial_stamp = driver->now();
  NavigationRuntimeEpochResetTestPeer::internalGoal(*navigation, makeHandoverGoal(
      initial_stamp, 10U, 1U, 1U,
      navigation_contracts::msg::NavigationGoal::BEHAVIOR_STOP));
  std::atomic_bool stop_publishing{false};
  std::thread sensor_thread([&] {
    std::uint64_t sequence = 1U;
    while (!stop_publishing.load(std::memory_order_acquire)) {
      const auto stamp = driver->now();
      const auto x = measured_x.load(std::memory_order_acquire);
      if (successor_published.load(std::memory_order_acquire) &&
          !successor_odom_recorded.exchange(true, std::memory_order_acq_rel)) {
        first_successor_odom_x.store(x, std::memory_order_release);
      }
      odometry_publisher->publish(makeHandoverOdometry(
          stamp, sequence, x,
          measured_velocity.load(std::memory_order_acquire)));
      observation_publisher->publish(makeHandoverScan(stamp, sequence));
      ++sequence;
      std::this_thread::sleep_for(20ms);
    }
  });
  auto cleanup = [&] {
    stop_publishing.store(true, std::memory_order_release);
    if (sensor_thread.joinable()) sensor_thread.join();
    executor_guard.stop();
    executor.remove_node(navigation);
    executor.remove_node(driver);
  };

  bool old_command_seen = false;
  const auto old_deadline = std::chrono::steady_clock::now() + 12s;
  while (std::chrono::steady_clock::now() < old_deadline) {
    {
      std::lock_guard lock(samples_mutex);
      old_command_seen = std::any_of(commands.begin(), commands.end(), [](const auto& command) {
        return command.request_id == 10U &&
               command.status == navigation_contracts::msg::NavigationCommand::STATUS_READY;
      });
    }
    if (old_command_seen) break;
    std::this_thread::sleep_for(20ms);
  }
  if (!old_command_seen) {
    cleanup();
    FAIL() << "real NavigationRuntimeNode did not dispatch the old STOP command";
  }
  // The command is now executing. Move the public measured state to the
  // declared terminal point with a small residual speed before publishing the
  // newer coincident STOP request. This exercises the runtime's existing
  // freshness/stationary evidence rather than a private state injection.
  // Let the committed trajectory reach its declared endpoint first; changing
  // the measured pose while it is still executing would trigger the
  // emergency discontinuity guard instead of terminal completion.
  bool old_terminal_seen = false;
  const auto terminal_deadline = std::chrono::steady_clock::now() + 20s;
    while (std::chrono::steady_clock::now() < terminal_deadline) {
      {
        std::lock_guard lock(samples_mutex);
        old_terminal_seen = std::any_of(commands.begin(), commands.end(), [&](const auto& command) {
        return command.request_id == 10U && terminal_adjacent.load(std::memory_order_acquire) &&
               command.status == navigation_contracts::msg::NavigationCommand::STATUS_COMPLETED &&
               (command.role == navigation_contracts::msg::NavigationCommand::ROLE_MAIN ||
                (command.role == navigation_contracts::msg::NavigationCommand::ROLE_BACKUP &&
                 command.backup_available)) &&
               command.position.x > 2.5 &&
               std::hypot(command.velocity.x, std::hypot(command.velocity.y, command.velocity.z)) <=
                   0.15;
      });
    }
    if (old_terminal_seen) break;
    std::this_thread::sleep_for(20ms);
  }
  if (!old_terminal_seen) {
    cleanup();
    std::ostringstream recent_commands;
    {
      std::lock_guard lock(samples_mutex);
      const auto first = commands.size() > 12U ? commands.size() - 12U : 0U;
      for (std::size_t index = first; index < commands.size(); ++index) {
        const auto& command = commands[index];
        if (command.request_id != 10U) continue;
        recent_commands << " [status=" << static_cast<int>(command.status)
                        << " role=" << static_cast<int>(command.role)
                        << " x=" << command.position.x
                        << " vx=" << command.velocity.x
                        << " bundle=" << command.bundle_generation
                        << " reason=" << command.reason_code
                        << " authorization=" << static_cast<int>(
                               command.execution_authorization)
                        << ']';
      }
    }
    FAIL() << "real NavigationRuntimeNode did not publish the old terminal command;"
           << " terminal_adjacent=" << terminal_adjacent.load()
           << " recent request-10 commands=" << recent_commands.str();
  }
  // Let the runtime consume the completed predecessor witness before changing
  // desired identity. The successor still starts from the same measured state.
  std::this_thread::sleep_for(500ms);
  ASSERT_FALSE(successor_published.exchange(true, std::memory_order_acq_rel));
  NavigationRuntimeEpochResetTestPeer::internalGoal(*navigation, makeHandoverGoal(
      driver->now(), 11U, 1U, 2U,
      navigation_contracts::msg::NavigationGoal::BEHAVIOR_STOP));
  bool successor_command_seen = false;
  const auto successor_deadline = std::chrono::steady_clock::now() + 12s;
  while (std::chrono::steady_clock::now() < successor_deadline) {
    {
      std::lock_guard lock(samples_mutex);
      successor_command_seen = std::any_of(commands.begin(), commands.end(), [](const auto& command) {
        return command.request_id == 11U &&
               command.status == navigation_contracts::msg::NavigationCommand::STATUS_READY &&
               command.role == navigation_contracts::msg::NavigationCommand::ROLE_MAIN;
      });
    }
    if (successor_command_seen) break;
    std::this_thread::sleep_for(20ms);
  }
  cleanup();
  EXPECT_TRUE(successor_command_seen)
      << "new same-mission STOP was not dispatched after the completed coincident boundary";
  EXPECT_TRUE(successor_published.load(std::memory_order_acquire));
  EXPECT_TRUE(successor_odom_recorded.load(std::memory_order_acquire));
  const double old_terminal_x = completed_position_x.load(std::memory_order_acquire);
  const double first_odom_x = first_successor_odom_x.load(std::memory_order_acquire);
  ASSERT_TRUE(std::isfinite(old_terminal_x));
  ASSERT_TRUE(std::isfinite(first_odom_x));
  EXPECT_NEAR(first_odom_x, old_terminal_x, 1.0e-6)
      << "successor odometry did not continue from the measured completed endpoint";
  std::optional<navigation_contracts::msg::NavigationCommand> successor_command;
  {
    std::lock_guard lock(successor_command_mutex);
    successor_command = first_successor_command;
  }
  ASSERT_TRUE(successor_command.has_value());
  // A fresh stopped-state solve exposes a new bundle from the measured state;
  // the short fresh-bundle time and endpoint continuity catch a test-side pose
  // teleport between the completed command and solve.
  EXPECT_LT(successor_command->trajectory_time_s, 0.5);
  EXPECT_NEAR(successor_command->position.x, old_terminal_x, 0.1)
      << "successor command was not anchored to the measured state";
  EXPECT_LE(std::hypot(successor_command->velocity.x,
                       std::hypot(successor_command->velocity.y,
                                  successor_command->velocity.z)), 0.15);
  (void)context->shutdown("completed handover test cleanup");
}

}  // namespace
}  // namespace navigation_runtime
