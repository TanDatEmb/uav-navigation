#include "navigation_runtime/navigation_runtime_node.hpp"
#include "navigation_runtime/mission_dynamics.hpp"
#include "navigation_runtime/mapping_fail_stop.hpp"
#include "navigation_runtime/commit_trace.hpp"

#include <navigation_mapping/mapping_actor.hpp>

#include "navigation_runtime/planner_fsm.hpp"
#include <navigation_execution/timestamp_freshness.hpp>
#include <navigation_contracts/execution_state_freshness.hpp>
#include <navigation_common/time.hpp>
#include <navigation_world_model/goal_contract.hpp>
#include <navigation_planning_backend/planner.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstring>
#include <filesystem>
#include <iomanip>
#include <limits>
#include <sstream>
#include <stdexcept>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

namespace navigation_runtime {
namespace {

bool propagatedOdometryFinite(const nav_msgs::msg::Odometry& odometry) {
  const auto& position = odometry.pose.pose.position;
  const auto& orientation = odometry.pose.pose.orientation;
  const auto& velocity = odometry.twist.twist.linear;
  const Eigen::Quaterniond quaternion{
      orientation.w, orientation.x, orientation.y, orientation.z};
  return std::isfinite(position.x) && std::isfinite(position.y) &&
         std::isfinite(position.z) && std::isfinite(velocity.x) &&
         std::isfinite(velocity.y) && std::isfinite(velocity.z) &&
         quaternion.coeffs().allFinite() && quaternion.norm() > 1.0e-6;
}

bool hasFloatField(const sensor_msgs::msg::PointCloud2& message, const std::string& name) {
  return std::any_of(message.fields.begin(), message.fields.end(), [&](const auto& field) {
    return field.name == name && field.datatype == sensor_msgs::msg::PointField::FLOAT32 &&
           field.count >= 1 && static_cast<std::uint64_t>(field.offset) + sizeof(float) <=
                                    message.point_step;
  });
}

double pointFromMessage(const geometry_msgs::msg::Point& point, int axis) {
  if (axis == 0) return point.x;
  if (axis == 1) return point.y;
  return point.z;
}

const geometry_msgs::msg::Point& plannerTarget(
    const navigation_contracts::msg::NavigationGoal& goal) {
  // planner backend accepts one terminal target per solve.  next_target is directional
  // metadata for the mission controller; using it as the terminal target
  // would remove the current checkpoint from the geometric problem entirely.
  // PASS_THROUGH continuity is provided by accepting the current checkpoint
  // before its trajectory ends and hot-retargeting the committed PVA state.
  return goal.target;
}

void addObservationAccountingValues(
    diagnostic_msgs::msg::DiagnosticStatus& status,
    const navigation_mapping::ObservationAccounting::Snapshot& accounting) {
  const auto add_value = [&status](const std::string& key, std::uint64_t value) {
    diagnostic_msgs::msg::KeyValue item;
    item.key = key;
    item.value = std::to_string(value);
    status.values.push_back(std::move(item));
  };
  // These fields are one canonical snapshot. Report consumers must not
  // reconstruct lifecycle state by combining independent diagnostic events.
  add_value("received_observation_count", accounting.received);
  add_value("observation_rejected_before_inbox_count", accounting.rejected_before_inbox);
  add_value("accepted_observation_count", accounting.accepted_to_inbox);
  add_value("observation_accepted_to_inbox_count", accounting.accepted_to_inbox);
  add_value("observation_replaced_pending_count", accounting.replaced_pending);
  add_value("observation_replaced_waiting_count", accounting.replaced_waiting);
  add_value("observation_replaced_ready_count", accounting.replaced_ready);
  add_value("dropped_cloud_count", accounting.replaced_waiting + accounting.replaced_ready);
  add_value("observation_discarded_pending_count", accounting.discarded_pending);
  add_value("observation_discarded_waiting_count", accounting.discarded_waiting);
  add_value("observation_discarded_ready_count", accounting.discarded_ready);
  add_value("observation_discarded_shutdown_ready_count", accounting.discarded_shutdown_ready);
  add_value("observation_discarded_nonmonotonic_count", accounting.discarded_nonmonotonic);
  add_value("observation_ready_submitted_count", accounting.ready_submitted);
  add_value("observation_waiting_count", accounting.waiting);
  add_value("observation_ready_count", accounting.ready);
  add_value("mapping_started_count", accounting.mapping_started);
  add_value("mapping_published_count", accounting.mapping_published);
  add_value("mapping_failed_count", accounting.mapping_failed);
  add_value("mapping_pending_count", accounting.pending);
  add_value("mapping_in_flight_count", accounting.in_flight);
  add_value("observation_accounting_valid", accounting.allInvariantsHold() ? 1U : 0U);
  add_value("observation_accounting_violation_count", accounting.violation_count);
}

}  // namespace

NavigationRuntimeNode::NavigationRuntimeNode(const rclcpp::NodeOptions& options)
    : NavigationRuntimeNode(options, NavigationRuntimeDependencies{}) {}

NavigationRuntimeNode::NavigationRuntimeNode(
    const rclcpp::NodeOptions& options, NavigationRuntimeDependencies dependencies)
    : rclcpp::Node("navigation_runtime_node", options),
      command_sampler_(command_bundle_store_),
      mapping_lifecycle_observer_(std::move(dependencies.lifecycle_observer)) {
  cloud_topic_ = declare_parameter("navigation_runtime.cloud_topic", std::string("/lio/registered_points"));
  registered_scan_topic_ = declare_parameter(
      "navigation_runtime.registered_scan_topic", std::string("/lio/mapping_observation"));
  const auto legacy_odometry_topic = declare_parameter(
      "navigation_runtime.odometry_topic", std::string("/lio/odometry_propagated"));
  RCLCPP_WARN_ONCE(get_logger(),
                   "Parameter navigation_runtime.odometry_topic is deprecated; use "
                   "navigation_runtime.propagated_odometry_topic");
  propagated_odometry_topic_ = declare_parameter(
      "navigation_runtime.propagated_odometry_topic", legacy_odometry_topic);
  corrected_odometry_topic_ = declare_parameter(
      "navigation_runtime.corrected_odometry_topic", std::string("/lio/odometry_corrected"));
  goal_topic_ = declare_parameter("navigation_runtime.goal_topic", std::string("/navigation/goal"));
  status_topic_ = declare_parameter(
      "navigation_runtime.status_topic", std::string("/navigation/mode_status"));
  command_topic_ = declare_parameter(
      "navigation_runtime.command_topic", std::string("/navigation/navigation_command"));
  planning_frame_ = declare_parameter("navigation_runtime.planning_frame", std::string("lio_odom"));
  body_frame_id_ = declare_parameter("navigation_runtime.body_frame_id", std::string("base_link"));
  deployment_profile_ = declare_parameter(
      "navigation_runtime.deployment_profile", std::string("sitl"));
  hardware_visibility_certified_ = declare_parameter(
      "navigation_runtime.hardware_visibility_certified", false);
  planner_rate_hz_ = declare_parameter("navigation_runtime.planner_rate_hz", 10.0);
  command_rate_hz_ = declare_parameter("navigation_runtime.command_rate_hz", 50.0);
  input_pair_max_skew_s_ = declare_parameter("navigation_runtime.input_pair_max_skew_s", 0.1);
  input_max_age_s_ = declare_parameter("navigation_runtime.input_max_age_s", 0.5);
  max_safety_suffix_anchor_error_m_ = declare_parameter(
      "navigation_runtime.max_safety_suffix_anchor_error_m", 0.75);
  planner_solve_timeout_s_ = declare_parameter(
      "navigation_runtime.planner_solve_timeout_s", 1.0);
  plan_from_rest_failure_confirmation_s_ = declare_parameter(
      "navigation_runtime.plan_from_rest_failure_confirmation_s", 0.5);
  const auto max_plan_from_rest_failures =
      declare_parameter("navigation_runtime.max_plan_from_rest_failures", 3);
  planner_config_path_ = declare_parameter("navigation_runtime.config_path", std::string{});
  const auto mission_file =
      declare_parameter("navigation_runtime.mission_file", std::string{});
  if (planner_config_path_.empty()) {
    planner_config_path_ = ament_index_cpp::get_package_share_directory("navigation_runtime") +
                         "/config/planner.yaml";
  }

  if (!std::filesystem::exists(planner_config_path_)) {
    throw std::runtime_error("planner backend config does not exist: " + planner_config_path_);
  }
  if (deployment_profile_ != "sitl" && deployment_profile_ != "hardware") {
    throw std::invalid_argument(
        "navigation_runtime.deployment_profile must be 'sitl' or 'hardware'");
  }
  if (deployment_profile_ == "hardware" && !hardware_visibility_certified_) {
    throw std::invalid_argument(
        "hardware planner backend runtime is blocked until the Mid-360 visibility/FOV "
        "certificate is explicitly enabled");
  }
  if (!std::isfinite(planner_rate_hz_) || planner_rate_hz_ <= 0.0) {
    throw std::invalid_argument("navigation_runtime.planner_rate_hz must be positive");
  }
  if (!std::isfinite(command_rate_hz_) || command_rate_hz_ <= 0.0 ||
      !std::isfinite(input_pair_max_skew_s_) || input_pair_max_skew_s_ <= 0.0 ||
      !std::isfinite(input_max_age_s_) || input_max_age_s_ <= 0.0 ||
      !std::isfinite(max_safety_suffix_anchor_error_m_) ||
      max_safety_suffix_anchor_error_m_ <= 0.0 ||
      !std::isfinite(planner_solve_timeout_s_) || planner_solve_timeout_s_ <= 0.0 ||
      !std::isfinite(plan_from_rest_failure_confirmation_s_) ||
      plan_from_rest_failure_confirmation_s_ <= 0.0) {
    throw std::invalid_argument(
        "planner backend command/input pairing/safety anchor parameters must be positive");
  }
  if (body_frame_id_.empty()) {
    throw std::invalid_argument("navigation_runtime.body_frame_id must not be empty");
  }
  if (max_plan_from_rest_failures <= 0) {
    throw std::invalid_argument(
        "navigation_runtime.max_plan_from_rest_failures must be positive");
  }
  max_plan_from_rest_failures_ =
      static_cast<std::uint32_t>(max_plan_from_rest_failures);
  plan_from_rest_failure_budget_ =
      ConsecutiveFailureBudget(max_plan_from_rest_failures_);

  planner_context_ = std::make_shared<navigation_planner_context::PlannerRuntimeContext>(
      [this]() { return now().seconds(); });
  diagnostics_publisher_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
      "/navigation/diagnostics", rclcpp::QoS(rclcpp::KeepLast(5)).reliable());
  std::optional<navigation_planning_backend::DynamicLimits> mission_limits;
  if (!mission_file.empty()) {
    mission_limits = loadMissionDynamicLimits(mission_file);
    RCLCPP_INFO(
        get_logger(),
        "Applying mission dynamics to planner backend before optimizer construction: "
        "velocity=%.3f acceleration=%.3f jerk=%.3f",
        mission_limits->max_velocity_mps,
        mission_limits->max_acceleration_mps2,
        mission_limits->max_jerk_mps3);
  }
  const auto ros_clock = get_clock();
  auto mapping_actor = std::make_shared<navigation_mapping::MappingActor>(
      planner_config_path_, [ros_clock] { return ros_clock->now().seconds(); });
  const auto& mapping_config = mapping_actor->config();
  if (mapping_config.ros_callback_en || mapping_config.batch_update_size != 1) {
    throw std::invalid_argument(
        "navigation_runtime owns mapping callbacks and requires rog_map/"
        "raycasting/batch_update_size=1");
  }
  if (planning_frame_ == "lio_odom" && mapping_config.virtual_ground_ceiling_en) {
    throw std::invalid_argument(
        "absolute ROG virtual ground/ceiling planes are invalid in lio_odom; "
        "set rog_map/virtual_ground_ceiling_en=false");
  }
  auto initial_world_view = mapping_actor->initialSnapshot();
  world_snapshot_store_.publish(initial_world_view);
  mapping_telemetry_ = std::make_shared<MappingTelemetry>();
  MappingTelemetrySnapshot initial_telemetry;
  initial_telemetry.snapshot_bytes = initial_world_view->byteSize();
  initial_telemetry.snapshot_owned_bytes = initial_world_view->ownedByteSize();
  initial_telemetry.snapshot_shared_metadata_bytes =
      initial_world_view->sharedMetadataByteSize();
  mapping_telemetry_->initialize(initial_telemetry);
  auto process_mapping = [mapping_actor, telemetry = mapping_telemetry_,
                          lifecycle_observer = mapping_lifecycle_observer_,
                          store = &world_snapshot_store_,
                          command_store = &command_bundle_store_,
                          epoch_ready = &localization_epoch_ready_]
      (navigation_mapping::MappingObservation&& observation) mutable {
    const auto result = mapping_actor->process(observation);
    if (result.reset_snapshot) store->publish(result.reset_snapshot);
    if (lifecycle_observer) {
      lifecycle_observer->onMutableMapUpdated(observation.stamp_ns);
    }
    MappingTelemetrySnapshot next = telemetry->snapshot();
    next.map = result.diagnostics;
    next.last_update_attempt_stamp_ns = observation.stamp_ns;
    next.snapshot_export_us = result.snapshot_export_us;
    next.pointcloud_decode_us = observation.pointcloud_decode_us;
    next.pair_wait_us = observation.pair_wait_us;
    if (result.snapshot) {
      store->publish(result.snapshot);
      (void)command_store->publishWorldIdentity(result.snapshot->identity());
      epoch_ready->store(true, std::memory_order_release);
      next.world_generation = result.world_generation;
      next.world_revision = result.world_revision;
      next.observation_stamp_ns = result.observation_stamp_ns;
      next.snapshot_bytes = result.snapshot->byteSize();
      next.snapshot_owned_bytes = result.snapshot->ownedByteSize();
      next.snapshot_shared_metadata_bytes = result.snapshot->sharedMetadataByteSize();
    }
    next.map_update_us = result.map_update_us;
    telemetry->recordUpdate(std::move(next));
  };
  auto validate_mapping = [ros_clock, telemetry = mapping_telemetry_,
                           active_epoch = &active_localization_epoch_,
                           maximum_age_ns = static_cast<std::int64_t>(
                                      input_max_age_s_ * 1e9)](
      const navigation_mapping::MappingObservation& observation) {
    const auto& pose = observation.corrected_odometry.pose.pose;
    const Eigen::Quaterniond q{
        pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z};
    const auto freshness = navigation_execution::classifyTimestampFreshness(
        ros_clock->now().nanoseconds(), observation.stamp_ns, maximum_age_ns);
    const bool invalid = !observation.cloud || observation.cloud->empty() ||
                         observation.localization_epoch !=
                             active_epoch->load(std::memory_order_acquire) ||
                         observation.localization_epoch == 0U ||
                         observation.stamp_ns <= 0 ||
                         navigation_common::rosTimeToNanoseconds(
                             observation.corrected_odometry.header.stamp).value_or(0) !=
                             observation.stamp_ns ||
                         !std::isfinite(pose.position.x) ||
                         !std::isfinite(pose.position.y) ||
                         !std::isfinite(pose.position.z) || !q.coeffs().allFinite() ||
                         q.norm() <= 1.0e-6 || freshness == navigation_execution::TimestampFreshness::INVALID;
    const bool stale = freshness == navigation_execution::TimestampFreshness::STALE;
    const bool future = freshness == navigation_execution::TimestampFreshness::FUTURE;
    if (invalid || stale || future) telemetry->recordDiscard(stale, future, invalid);
    return !invalid && !stale && !future;
  };
  mapping_worker_ = std::make_unique<navigation_mapping::MappingWorker<navigation_mapping::MappingObservation>>(
      observation_accounting_, std::move(process_mapping),
      mappingFailStop, std::move(validate_mapping),
      [publisher = diagnostics_publisher_, ros_clock,
       telemetry = mapping_telemetry_, accounting = &observation_accounting_]() {
        const auto mapping = telemetry->snapshot();
        const auto lifecycle = accounting->snapshot();
        diagnostic_msgs::msg::DiagnosticArray diagnostics;
        diagnostics.header.stamp = ros_clock->now();
        diagnostic_msgs::msg::DiagnosticStatus status;
        status.name = "navigation_mapping/world_model";
        status.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
        status.message = std::string("PUBLISHED_") +
            std::string(navigation_mapping::worldUpdateOutcomeName(mapping.map.update_outcome));
        const auto add_value = [&status](const std::string& key, std::uint64_t value) {
          diagnostic_msgs::msg::KeyValue item;
          item.key = key;
          item.value = std::to_string(value);
          status.values.push_back(std::move(item));
        };
        const auto add_duration = [&status](const std::string& key, std::int64_t value) {
          diagnostic_msgs::msg::KeyValue item;
          item.key = key;
          item.value = std::to_string(std::max<std::int64_t>(0, value));
          status.values.push_back(std::move(item));
        };
        const auto add_text = [&status](const std::string& key, const std::string& value) {
          diagnostic_msgs::msg::KeyValue item;
          item.key = key;
          item.value = value;
          status.values.push_back(std::move(item));
        };
        const auto& map = mapping.map;
        add_value("world_generation", mapping.world_generation);
        add_value("world_revision", mapping.world_revision);
        add_value("observation_stamp_ns", mapping.observation_stamp_ns);
        add_value("last_update_attempt_stamp_ns", mapping.last_update_attempt_stamp_ns);
        addObservationAccountingValues(status, lifecycle);
        add_text("mapping_update_outcome",
                 std::string(navigation_mapping::worldUpdateOutcomeName(map.update_outcome)));
        add_value("mapping_outcome_updated_count", mapping.outcome_updated);
        add_value("mapping_outcome_accumulated_count", mapping.outcome_accumulated);
        add_value("mapping_outcome_slide_only_count", mapping.outcome_slide_only);
        add_value("mapping_outcome_empty_cloud_count", mapping.outcome_empty_cloud);
        add_value("mapping_outcome_callback_owned_count", mapping.outcome_callback_owned);
        add_value("mapping_outcome_below_ground_count", mapping.outcome_below_ground);
        add_value("mapping_outcome_above_ceiling_count", mapping.outcome_above_ceiling);
        add_value("observation_accounting_valid",
                  lifecycle.allInvariantsHold() ? 1U : 0U);
        add_value("observation_accounting_violation_count", lifecycle.violation_count);
        add_value("mapping_input_point_count", map.endpoint_count);
        add_value("mapping_allocated_voxel_count", map.allocated_voxel_count);
        add_value("world_snapshot_bytes", mapping.snapshot_bytes);
        add_value("world_snapshot_owned_bytes", mapping.snapshot_owned_bytes);
        add_value("world_snapshot_shared_metadata_bytes",
                  mapping.snapshot_shared_metadata_bytes);
        add_value("world_snapshot_live_count", navigation_mapping::MappingWorldSnapshot::liveCount());
        add_value("world_snapshot_peak_live_count", navigation_mapping::MappingWorldSnapshot::peakLiveCount());
        add_value("world_snapshot_live_owned_bytes", navigation_mapping::MappingWorldSnapshot::liveOwnedBytes());
        add_value("world_snapshot_peak_live_owned_bytes",
                  navigation_mapping::MappingWorldSnapshot::peakLiveOwnedBytes());
        add_duration("ros_pointcloud_decode_us", mapping.pointcloud_decode_us);
        add_duration("observation_pair_wait_us", mapping.pair_wait_us);
        add_duration("rog_raycast_us", map.rog_raycast_us);
        add_duration("rog_probability_update_us", map.rog_probability_update_us);
        add_duration("rog_inflation_us", map.rog_inflation_us);
        add_duration("rog_slide_us", map.rog_slide_us);
        add_duration("rog_total_update_us", map.rog_total_update_us);
        add_duration("world_snapshot_export_us", mapping.snapshot_export_us);
        add_duration("mapping_callback_total_us", mapping.map_update_us);
        diagnostics.status.push_back(std::move(status));
        publisher->publish(diagnostics);
      });
  planner_ = std::make_shared<navigation_planning_backend::Planner>(
      planner_config_path_, planner_context_, world_snapshot_store_.load().view, mission_limits,
      world_snapshot_store_);
  mapping_worker_->setStrictlyIncreasingOrderKey(
      [](const navigation_mapping::MappingObservation& observation) { return observation.stamp_ns; });
  mapping_worker_->start();

  const auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).best_effort().durability_volatile();
  propagated_state_callback_group_ = create_callback_group(rclcpp::CallbackGroupType::Reentrant);
  rclcpp::SubscriptionOptions propagated_state_options;
  propagated_state_options.callback_group = propagated_state_callback_group_;
  cloud_subscription_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      cloud_topic_, qos, std::bind(&NavigationRuntimeNode::onCloud, this, std::placeholders::_1));
  registered_scan_subscription_ = create_subscription<
      navigation_contracts::msg::RegisteredScan>(
      registered_scan_topic_, qos,
      std::bind(&NavigationRuntimeNode::onRegisteredScan, this, std::placeholders::_1));
  estimator_health_subscription_ = create_subscription<
      navigation_contracts::msg::EstimatorHealth>(
      "/lio/health", rclcpp::QoS(rclcpp::KeepLast(10)).best_effort(),
      std::bind(&NavigationRuntimeNode::onEstimatorHealth, this, std::placeholders::_1));
  corrected_odometry_subscription_ = create_subscription<nav_msgs::msg::Odometry>(
      corrected_odometry_topic_, qos,
      std::bind(&NavigationRuntimeNode::onCorrectedOdometry, this, std::placeholders::_1));
  propagated_odometry_subscription_ = create_subscription<nav_msgs::msg::Odometry>(
      propagated_odometry_topic_, qos,
      std::bind(&NavigationRuntimeNode::onPropagatedOdometry, this, std::placeholders::_1),
      propagated_state_options);
  goal_subscription_ = create_subscription<navigation_contracts::msg::NavigationGoal>(
      goal_topic_, rclcpp::QoS(rclcpp::KeepLast(1)).reliable(),
      std::bind(&NavigationRuntimeNode::onGoal, this, std::placeholders::_1));
  status_subscription_ =
      create_subscription<navigation_contracts::msg::NavigationModeStatus>(
          status_topic_,
          rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local(),
          std::bind(&NavigationRuntimeNode::onModeStatus, this, std::placeholders::_1));
  command_publisher_ = create_publisher<navigation_contracts::msg::NavigationCommand>(
      command_topic_, rclcpp::QoS(rclcpp::KeepLast(1)).reliable());
  end_to_end_samples_ms_.reserve(256);

  // planner backend's planner state (last_exp_traj_, robot_state_ and CmdTraj) is not
  // re-entrant.  A 300 ms optimization must never overlap the next timer
  // tick; only the read-only command sampler is allowed to run concurrently.
  planning_callback_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  command_callback_group_ = create_callback_group(rclcpp::CallbackGroupType::Reentrant);
  const auto planning_period = std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(1.0 / planner_rate_hz_));
  planning_period_us_ =
      std::chrono::duration_cast<std::chrono::microseconds>(planning_period).count();
  const auto command_period = std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(1.0 / command_rate_hz_));
  planning_timer_ = create_wall_timer(
      planning_period, std::bind(&NavigationRuntimeNode::runCycle, this), planning_callback_group_);
  command_timer_ = create_wall_timer(
      command_period, std::bind(&NavigationRuntimeNode::publishCommand, this),
      command_callback_group_);
  RCLCPP_INFO(get_logger(),
              "planner backend runtime ready: cloud=%s corrected_odom=%s propagated_odom=%s goal=%s "
              "output=%s planner=%.1fHz command=%.1fHz",
              cloud_topic_.c_str(), corrected_odometry_topic_.c_str(),
              propagated_odometry_topic_.c_str(), goal_topic_.c_str(),
              command_topic_.c_str(), planner_rate_hz_, command_rate_hz_);
}

NavigationRuntimeNode::~NavigationRuntimeNode() {
  accepting_observations_.store(false);
  if (planning_timer_) planning_timer_->cancel();
  if (command_timer_) command_timer_->cancel();
  if (planner_) planner_->cancelActiveSolve();
  cloud_subscription_.reset();
  registered_scan_subscription_.reset();
  estimator_health_subscription_.reset();
  corrected_odometry_subscription_.reset();
  propagated_odometry_subscription_.reset();
  goal_subscription_.reset();
  status_subscription_.reset();
  {
    std::lock_guard lock(input_mutex_);
    if (latest_cloud_) {
      latest_cloud_.reset();
      pending_cloud_received_steady_ns_ = 0;
      observation_accounting_.discardedPending();
    }
  }
  if (mapping_worker_) mapping_worker_->shutdown();
  if (mapping_lifecycle_observer_) {
    mapping_lifecycle_observer_->onShutdownComplete(observation_accounting_.snapshot());
  }
}

bool NavigationRuntimeNode::decodeCloud(const sensor_msgs::msg::PointCloud2& message,
                                      navigation_mapping::PointCloud& output) {
  if (!hasFloatField(message, "x") || !hasFloatField(message, "y") ||
      !hasFloatField(message, "z") || message.point_step == 0U ||
      message.row_step < message.point_step * message.width ||
      static_cast<std::uint64_t>(message.row_step) * message.height > message.data.size()) {
    return false;
  }
  output.clear();
  output.reserve(static_cast<std::size_t>(message.width) * message.height);
  try {
    sensor_msgs::PointCloud2ConstIterator<float> x(message, "x");
    sensor_msgs::PointCloud2ConstIterator<float> y(message, "y");
    sensor_msgs::PointCloud2ConstIterator<float> z(message, "z");
    for (; x != x.end(); ++x, ++y, ++z) {
      if (std::isfinite(*x) && std::isfinite(*y) && std::isfinite(*z)) {
        output.emplace_back(*x, *y, *z, 0.0F);
      }
    }
  } catch (const std::exception&) {
    output.clear();
    return false;
  }
  return !output.empty();
}

builtin_interfaces::msg::Time NavigationRuntimeNode::rosTimeFromSeconds(double seconds) {
  return navigation_common::secondsToRosTime(seconds).value_or(
      builtin_interfaces::msg::Time{});
}

std::optional<navigation_mapping::MappingObservation> NavigationRuntimeNode::tryPromotePairLocked() {
  if (!latest_cloud_) return std::nullopt;
  auto pair = input_pairing::tryTakeExactPair(
      latest_cloud_, corrected_odometry_history_);
  if (pair) {
    last_pair_wait_us_ = pending_cloud_received_steady_ns_ > 0
        ? std::max<std::int64_t>(
              0, (navigation_common::steadyClockNowNanoseconds() -
                  pending_cloud_received_steady_ns_) /
                  navigation_common::kNanosecondsPerMicrosecond)
        : 0;
    pending_cloud_received_steady_ns_ = 0;
    return navigation_mapping::MappingObservation{
        std::move(pair->payload), std::move(pair->corrected_odometry),
        active_localization_epoch_.load(std::memory_order_acquire), 0U, pair->stamp_ns,
        last_input_conversion_us_.load(), last_pair_wait_us_.load()};
  }
  const auto pending_stamp_ns = latest_cloud_->stamp_ns;
  const auto newest_corrected_stamp_ns = corrected_odometry_history_.empty()
      ? 0 : navigation_common::rosTimeToNanoseconds(
                    corrected_odometry_history_.back().header.stamp).value_or(0);
  const auto freshness = navigation_execution::classifyTimestampFreshness(
      now().nanoseconds(), pending_stamp_ns,
      static_cast<std::int64_t>(input_max_age_s_ * 1e9));
  if (newest_corrected_stamp_ns > pending_stamp_ns) {
    ++corrected_pair_mismatch_count_;
  } else if (freshness == navigation_execution::TimestampFreshness::STALE) {
    ++stale_input_count_;
    ++stale_mapping_input_count_;
  } else if (freshness == navigation_execution::TimestampFreshness::FUTURE) {
    ++stale_input_count_;
    ++future_mapping_input_count_;
  } else {
    return std::nullopt;
  }
  latest_cloud_.reset();
  pending_cloud_received_steady_ns_ = 0;
  observation_accounting_.discardedPending();
  return std::nullopt;
}

void NavigationRuntimeNode::resetForLocalizationEpochLocked(
    const std::uint64_t localization_epoch) {
  if (localization_epoch == 0U) return;
  const auto current = active_localization_epoch_.load(std::memory_order_acquire);
  if (localization_epoch <= current) return;

  localization_epoch_ready_.store(false, std::memory_order_release);
  // Match the goal transition ordering: cancel the solve before changing the
  // epoch exposed to the planning callback, drain the mapping barrier, then
  // clear command exposure under the execution transition lock.
  planner_->cancelActiveSolve();
  if (mapping_worker_) mapping_worker_->reset();
  {
    std::lock_guard<std::mutex> input_lock(input_mutex_);
    if (latest_cloud_) {
      latest_cloud_.reset();
      pending_cloud_received_steady_ns_ = 0;
      observation_accounting_.discardedPending();
    }
    corrected_odometry_history_.clear();
  }
  active_localization_epoch_.store(localization_epoch, std::memory_order_release);
  execution_state_store_.resetForLocalizationEpoch(localization_epoch);
  command_bundle_store_.invalidate();
  last_registered_scan_epoch_.store(localization_epoch, std::memory_order_release);
  last_registered_scan_sequence_.store(0U, std::memory_order_release);
  {
    std::lock_guard<std::mutex> lock(input_mutex_);
    ++active_goal_epoch_;
    (void)command_bundle_store_.setActiveGoalEpoch(
        active_goal_epoch_.load(std::memory_order_acquire), false);
    new_goal_ = active_goal_.has_value();
    hot_goal_transition_ = false;
    restart_from_rest_ = false;
    skip_replan_once_ = false;
    trajectory_finished_.store(false);
    trajectory_reaches_goal_.store(false);
  }
  {
    std::lock_guard<std::mutex> command_lock(
        command_execution_lease_failure_latch_.transitionMutex());
    planner_command_available_.store(false);
    // Keep the legacy command path fail-closed until the new epoch publishes
    // its first valid world snapshot and the planner replans.
    planner_failure_latched_.store(true);
    safety_suffix_active_.store(false);
    command_goal_epoch_.store(0U);
  }
  RCLCPP_WARN(get_logger(),
              "Localization epoch changed to %lu; old mapping and command state invalidated",
              static_cast<unsigned long>(localization_epoch));
}

void NavigationRuntimeNode::onRegisteredScan(
    const navigation_contracts::msg::RegisteredScan::ConstSharedPtr& message) {
  if (!accepting_observations_.load(std::memory_order_acquire) ||
      message->localization_epoch == 0U ||
      message->scan_sequence == 0U ||
      message->header.frame_id != planning_frame_ ||
      message->points.header.frame_id != message->header.frame_id ||
      navigation_common::rosTimeToNanoseconds(message->points.header.stamp).value_or(0) !=
          navigation_common::rosTimeToNanoseconds(message->header.stamp).value_or(0) ||
      message->body_frame_id != body_frame_id_) {
    observation_accounting_.recordRejectedBeforeInbox();
    return;
  }
  const auto& pose = message->corrected_pose.pose;
  const Eigen::Quaterniond quaternion{
      pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z};
  if (navigation_common::rosTimeToNanoseconds(message->header.stamp).value_or(0) <= 0 ||
      !std::isfinite(pose.position.x) || !std::isfinite(pose.position.y) ||
      !std::isfinite(pose.position.z) || !quaternion.coeffs().allFinite() ||
      quaternion.norm() <= 1.0e-6) {
    observation_accounting_.recordRejectedBeforeInbox();
    return;
  }
  const auto decode_started = std::chrono::steady_clock::now();
  auto decoded = std::make_shared<navigation_mapping::PointCloud>();
  if (!decodeCloud(message->points, *decoded)) {
    observation_accounting_.recordRejectedBeforeInbox();
    return;
  }

  std::lock_guard<std::mutex> transition_lock(localization_transition_mutex_);
  const auto active_epoch = active_localization_epoch_.load(std::memory_order_acquire);
  if (message->localization_epoch < active_epoch) {
    observation_accounting_.recordRejectedBeforeInbox();
    return;
  }
  if (message->localization_epoch > active_epoch) {
    resetForLocalizationEpochLocked(message->localization_epoch);
  }
  const auto sequence_epoch = last_registered_scan_epoch_.load(std::memory_order_acquire);
  const auto previous_scan_sequence =
      last_registered_scan_sequence_.load(std::memory_order_acquire);
  if (sequence_epoch == message->localization_epoch &&
      message->scan_sequence <= previous_scan_sequence) {
    observation_accounting_.recordRejectedBeforeInbox();
    return;
  }
  last_registered_scan_epoch_.store(message->localization_epoch, std::memory_order_release);
  last_registered_scan_sequence_.store(message->scan_sequence, std::memory_order_release);
  nav_msgs::msg::Odometry corrected;
  corrected.header = message->header;
  corrected.child_frame_id = message->body_frame_id;
  corrected.pose = message->corrected_pose;
  navigation_mapping::MappingObservation observation{
      std::move(decoded), std::move(corrected), message->localization_epoch,
      message->scan_sequence,
      navigation_common::rosTimeToNanoseconds(message->header.stamp).value_or(0),
      std::chrono::duration_cast<std::chrono::microseconds>(
          std::chrono::steady_clock::now() - decode_started)
          .count(),
      0};
  observation_accounting_.recordAcceptedToInbox();
  if (mapping_worker_->submitFromWaiting(std::move(observation))) {
    typed_observation_seen_.store(true, std::memory_order_release);
  }
}

void NavigationRuntimeNode::onEstimatorHealth(
    const navigation_contracts::msg::EstimatorHealth::ConstSharedPtr& message) {
  if (!message || message->localization_epoch == 0U) return;
  std::lock_guard<std::mutex> transition_lock(localization_transition_mutex_);
  const auto active_epoch = active_localization_epoch_.load(std::memory_order_acquire);
  if (message->localization_epoch <= active_epoch) return;
  // Health announces the public-frame transition early; command exposure stays
  // disabled until a RegisteredScan of this epoch is accepted and mapped.
  resetForLocalizationEpochLocked(message->localization_epoch);
}

void NavigationRuntimeNode::onCloud(const sensor_msgs::msg::PointCloud2::ConstSharedPtr& message) {
  if (typed_observation_seen_.load(std::memory_order_acquire)) return;
  if (!accepting_observations_.load()) {
    observation_accounting_.recordRejectedBeforeInbox();
    return;
  }
  if (message->header.frame_id != planning_frame_) {
    observation_accounting_.recordRejectedBeforeInbox();
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                         "Dropping cloud in frame '%s'; planner backend input must be in '%s'",
                         message->header.frame_id.c_str(), planning_frame_.c_str());
    return;
  }
  const auto decode_started = std::chrono::steady_clock::now();
  auto decoded = std::make_shared<navigation_mapping::PointCloud>();
  if (!decodeCloud(*message, *decoded)) {
    observation_accounting_.recordRejectedBeforeInbox();
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "Dropping malformed/empty PointCloud2");
    return;
  }
  const auto stamp_ns =
      navigation_common::rosTimeToNanoseconds(message->header.stamp).value_or(0);
  if (stamp_ns <= 0) {
    observation_accounting_.recordRejectedBeforeInbox();
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                         "Dropping cloud without a valid timestamp");
    return;
  }
  std::optional<navigation_mapping::MappingObservation> ready;
  {
    std::lock_guard<std::mutex> lock(input_mutex_);
    last_input_conversion_us_ = std::chrono::duration_cast<std::chrono::microseconds>(
        std::chrono::steady_clock::now() - decode_started).count();
    observation_accounting_.recordAcceptedToInbox();
    latest_cloud_ = input_pairing::StampedObservation<
        std::shared_ptr<navigation_mapping::PointCloud>>{std::move(decoded), stamp_ns};
    pending_cloud_received_steady_ns_ = navigation_common::steadyClockNowNanoseconds();
    ready = tryPromotePairLocked();
    if (ready) (void)mapping_worker_->submitFromWaiting(std::move(*ready));
  }
}

void NavigationRuntimeNode::onCorrectedOdometry(
    const nav_msgs::msg::Odometry::ConstSharedPtr& message) {
  if (typed_observation_seen_.load(std::memory_order_acquire)) return;
  if (message->header.frame_id != planning_frame_) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                         "Dropping odometry frame '%s'; planner backend input must be in '%s'",
                         message->header.frame_id.c_str(), planning_frame_.c_str());
    return;
  }
  const auto stamp_ns =
      navigation_common::rosTimeToNanoseconds(message->header.stamp).value_or(0);
  if (stamp_ns <= 0) return;
  if (!accepting_observations_.load()) return;
  std::optional<navigation_mapping::MappingObservation> ready;
  {
    std::lock_guard<std::mutex> lock(input_mutex_);
    if (!corrected_odometry_history_.empty() &&
        stamp_ns < navigation_common::rosTimeToNanoseconds(
                          corrected_odometry_history_.back().header.stamp).value_or(0)) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                           "Dropping out-of-order odometry timestamp");
      return;
    }
    corrected_odometry_history_.push_back(*message);
    while (corrected_odometry_history_.size() > 64U) corrected_odometry_history_.pop_front();
    const auto horizon_ns = static_cast<std::int64_t>(2.0 * input_max_age_s_ * 1e9);
    while (!corrected_odometry_history_.empty() &&
           stamp_ns - navigation_common::rosTimeToNanoseconds(
                          corrected_odometry_history_.front().header.stamp).value_or(0) >
               horizon_ns) {
      corrected_odometry_history_.pop_front();
    }
    ready = tryPromotePairLocked();
    if (ready) (void)mapping_worker_->submitFromWaiting(std::move(*ready));
  }
}

void NavigationRuntimeNode::onPropagatedOdometry(
    const nav_msgs::msg::Odometry::ConstSharedPtr& message) {
  if (message->header.frame_id != planning_frame_) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                         "Dropping propagated odometry frame '%s'; expected '%s'",
                         message->header.frame_id.c_str(), planning_frame_.c_str());
    return;
  }
  const auto stamp_ns =
      navigation_common::rosTimeToNanoseconds(message->header.stamp).value_or(0);
  if (stamp_ns <= 0 || !propagatedOdometryFinite(*message)) {
    ++invalid_execution_state_count_;
    return;
  }
  const Eigen::Quaterniond orientation{
      message->pose.pose.orientation.w, message->pose.pose.orientation.x,
      message->pose.pose.orientation.y, message->pose.pose.orientation.z};
  if (!orientation.coeffs().allFinite() || orientation.norm() <= 1.0e-9 ||
      message->child_frame_id != body_frame_id_) {
    ++invalid_execution_state_count_;
    return;
  }
  navigation_planning::KinematicState typed_state;
  typed_state.position_world = Eigen::Vector3d{
      message->pose.pose.position.x, message->pose.pose.position.y,
      message->pose.pose.position.z};
  typed_state.orientation_world_body = orientation.normalized();
  const auto rotation = typed_state.orientation_world_body.toRotationMatrix();
  typed_state.yaw_rad = std::atan2(rotation(1, 0), rotation(0, 0));
  typed_state.velocity_world = typed_state.orientation_world_body * Eigen::Vector3d{
      message->twist.twist.linear.x, message->twist.twist.linear.y,
      message->twist.twist.linear.z};
  typed_state.source_stamp_ns = stamp_ns;
  typed_state.receive_stamp_ns = navigation_common::steadyClockNowNanoseconds();
  typed_state.localization_epoch = active_localization_epoch_.load(std::memory_order_acquire);
  typed_state.world_frame_id = message->header.frame_id;
  typed_state.body_frame_id = message->child_frame_id;
  (void)execution_state_store_.publish(std::move(typed_state));
}

void NavigationRuntimeNode::onGoal(const navigation_contracts::msg::NavigationGoal::ConstSharedPtr& message) {
  std::lock_guard<std::mutex> lock(input_mutex_);
  // A planner backend plan is owned by the mission waypoint identity.  The
  // continuation point is only look-ahead metadata; treating it as the
  // current goal makes a repeated waypoint publication look like a new
  // request (or, conversely, hides the actual waypoint transition).
  const bool same_checkpoint = active_goal_.has_value() &&
      active_goal_->mission_id == message->mission_id &&
      active_goal_->waypoint_index == message->waypoint_index;
  const bool same_logical_goal = same_checkpoint &&
      active_goal_->request_id == message->request_id;
  const bool reuse_completed_stop = same_checkpoint && !same_logical_goal &&
      message->behavior == navigation_contracts::msg::NavigationGoal::BEHAVIOR_STOP &&
      planner_command_available_.load() && trajectory_reaches_goal_.load() &&
      !planner_failure_latched_.load() && !safety_suffix_active_.load();
  const bool can_hot_retarget = canHotRetargetAtWaypointTransition(
      same_logical_goal,
      active_goal_.has_value() &&
          active_goal_->behavior ==
              navigation_contracts::msg::NavigationGoal::BEHAVIOR_PASS_THROUGH,
      planner_command_available_.load(), planner_failure_latched_.load(),
      safety_suffix_active_.load());
  if (!same_logical_goal) {
    // Cancel before exposing the new waypoint identity. planner backend's commit gate
    // guarantees that a solve for the previous waypoint cannot publish a new
    // CmdTraj after this callback has invalidated it. The already committed
    // atomic bundle remains available for smooth hot-retarget continuity.
    planner_->cancelActiveSolve();
    ++active_goal_epoch_;
    (void)command_bundle_store_.setActiveGoalEpoch(
        active_goal_epoch_.load(std::memory_order_acquire),
        reuse_completed_stop || can_hot_retarget);
  }
  active_goal_ = *message;
  if (reuse_completed_stop) {
    // The mission controller may republish a STOP checkpoint when one noisy
    // velocity sample temporarily breaks its hold confirmation. The completed
    // planner backend command already terminates at this same checkpoint and PX4 is
    // actively holding it; replacing it with a zero-distance PlanFromRest
    // creates a singular yaw problem and unnecessarily drops position hold.
    new_goal_ = false;
    hot_goal_transition_ = false;
    restart_from_rest_ = false;
    skip_replan_once_ = false;
    trajectory_finished_.store(true);
    {
      std::lock_guard<std::mutex> command_lock(
          command_execution_lease_failure_latch_.transitionMutex());
      command_goal_epoch_.store(active_goal_epoch_.load());
    }
    return;
  }
  if (!same_logical_goal) {
    {
      // Global order is input_mutex_ -> execution transition. Command sampling
      // snapshots input and releases it before taking the transition lock.
      std::lock_guard<std::mutex> command_lock(
          command_execution_lease_failure_latch_.transitionMutex());
      command_execution_lease_failure_latch_.resetForNewGoalWithinTransition();
      planner_failure_latched_.store(false);
      safety_suffix_active_.store(false);
      if (can_hot_retarget) {
        command_goal_epoch_.store(active_goal_epoch_.load());
      } else {
        planner_command_available_.store(false);
        command_goal_epoch_.store(0U);
      }
    }
    plan_from_rest_failure_budget_.reset();
    plan_from_rest_first_failure_steady_ns_ = 0;
    hot_goal_transition_ = can_hot_retarget;
    new_goal_ = !can_hot_retarget;
    restart_from_rest_ = false;
    skip_replan_once_ = false;
    trajectory_finished_.store(false);
    trajectory_reaches_goal_.store(false);
  }
}

void NavigationRuntimeNode::onModeStatus(
    const navigation_contracts::msg::NavigationModeStatus::ConstSharedPtr& message) {
  if (message->state == navigation_contracts::msg::NavigationModeStatus::ACTIVE ||
      message->state == navigation_contracts::msg::NavigationModeStatus::BRAKING) {
    return;
  }
  std::lock_guard<std::mutex> lock(input_mutex_);
  if (!active_goal_ || active_goal_->mission_id != message->mission_id ||
      active_goal_->waypoint_index != message->waypoint_index ||
      active_goal_->request_id != message->request_id) {
    return;
  }
  RCLCPP_INFO(get_logger(),
              "Cancelling planner backend goal after terminal mission status state=%u reason=%u "
              "mission=%s waypoint=%u request=%lu",
              message->state, message->reason, message->mission_id.c_str(),
              message->waypoint_index, static_cast<unsigned long>(message->request_id));
  planner_->cancelActiveSolve();
  ++active_goal_epoch_;
  (void)command_bundle_store_.setActiveGoalEpoch(
      active_goal_epoch_.load(std::memory_order_acquire), false);
  active_goal_.reset();
  new_goal_ = false;
  hot_goal_transition_ = false;
  restart_from_rest_ = false;
  skip_replan_once_ = false;
  {
    std::lock_guard<std::mutex> command_lock(
        command_execution_lease_failure_latch_.transitionMutex());
    planner_command_available_.store(false);
    planner_failure_latched_.store(false);
    safety_suffix_active_.store(false);
    command_goal_epoch_.store(0U);
  }
  plan_from_rest_failure_budget_.reset();
  plan_from_rest_first_failure_steady_ns_ = 0;
  trajectory_finished_.store(false);
  trajectory_reaches_goal_.store(false);
}

bool NavigationRuntimeNode::commitPlannerCandidate(
    const navigation_contracts::msg::NavigationGoal& goal,
    const std::uint64_t goal_epoch,
    const std::uint64_t localization_epoch,
    const std::int64_t now_ns) {
  if (now_ns <= 0 || goal_epoch == 0U || localization_epoch == 0U ||
      goal.request_id == 0U) {
    return false;
  }
  const auto maximum_age_ns = static_cast<std::int64_t>(input_max_age_s_ * 1.0e9);
  if (maximum_age_ns <= 0 || now_ns > std::numeric_limits<std::int64_t>::max() - maximum_age_ns) {
    return false;
  }
  const auto candidate = planner_->exportCommandCandidate(
      localization_epoch, goal_epoch, goal.request_id, now_ns,
      now_ns + maximum_age_ns);
  if (!candidate) return false;
  const auto candidate_ptr = std::make_shared<const navigation_planning::CandidateBundle>(
      *candidate);
  const navigation_execution::CommitToken token{
      candidate_ptr->world_identity, goal_epoch, ++execution_transaction_id_};
  return command_bundle_store_.tryCommit(token, candidate_ptr) ==
      navigation_execution::CommitDecision::kCommitted;
}

void NavigationRuntimeNode::runCycle() {
  const auto cycle_started = std::chrono::steady_clock::now();
  const auto cycle_started_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
      cycle_started.time_since_epoch()).count();
  if (last_cycle_started_steady_ns_ > 0) {
    const auto interval_us = (cycle_started_ns - last_cycle_started_steady_ns_) / 1000;
    last_planning_scheduling_gap_us_ =
        std::max<std::int64_t>(0, interval_us - planning_period_us_);
  }
  last_cycle_started_steady_ns_ = cycle_started_ns;
  ++cycle_count_;
  std::shared_ptr<const navigation_execution::ExecutionStateLease> propagated_state;
  std::optional<navigation_contracts::msg::NavigationGoal> goal;
  bool new_goal = false;
  bool hot_goal_transition = false;
  bool restart_from_rest = false;
  std::uint64_t goal_epoch = 0;
  std::int64_t input_conversion_us = 0;
  const auto input_lock_started = std::chrono::steady_clock::now();
  {
    std::lock_guard<std::mutex> lock(input_mutex_);
    last_input_lock_wait_us_ = std::chrono::duration_cast<std::chrono::microseconds>(
        std::chrono::steady_clock::now() - input_lock_started).count();
    // Housekeeping only: valid pairs are promoted immediately by either input
    // callback. Expire an unmatched WAITING_PAIR if no later callback arrives.
    if (latest_cloud_) {
      const auto freshness = navigation_execution::classifyTimestampFreshness(
          now().nanoseconds(), latest_cloud_->stamp_ns,
          static_cast<std::int64_t>(input_max_age_s_ * 1e9));
      if (freshness == navigation_execution::TimestampFreshness::STALE ||
          freshness == navigation_execution::TimestampFreshness::FUTURE) {
        if (freshness == navigation_execution::TimestampFreshness::STALE) ++stale_mapping_input_count_;
        if (freshness == navigation_execution::TimestampFreshness::FUTURE) ++future_mapping_input_count_;
        ++stale_input_count_;
        latest_cloud_.reset();
        pending_cloud_received_steady_ns_ = 0;
        observation_accounting_.discardedPending();
      }
    }
    goal = active_goal_;
    new_goal = new_goal_;
    hot_goal_transition = hot_goal_transition_;
    restart_from_rest = restart_from_rest_;
    goal_epoch = active_goal_epoch_.load();
    input_conversion_us = last_input_conversion_us_;
  }
  // Do not acquire the state-store mutex while holding input_mutex_: the
  // odometry callback publishes to the store before taking input_mutex_.
  propagated_state = execution_state_store_.load();
  const auto now_ns = now().nanoseconds();
  const auto maximum_age_ns = static_cast<std::int64_t>(input_max_age_s_ * 1e9);
  const auto mapping = mapping_telemetry_->snapshot();
  const auto cloud_stamp_ns = mapping.observation_stamp_ns;
  const auto corrected_stamp_ns = mapping.observation_stamp_ns;

  diagnostic_msgs::msg::DiagnosticArray diagnostics;
  diagnostics.header.stamp = now();
  diagnostic_msgs::msg::DiagnosticStatus status;
  status.name = "navigation_runtime/planner";
  status.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
  status.message = goal ? "TRACKING" : "MAP_READY";
  const auto add_value = [&status](const std::string& key, std::uint64_t value) {
    diagnostic_msgs::msg::KeyValue item;
    item.key = key;
    item.value = std::to_string(value);
    status.values.push_back(std::move(item));
  };
  const auto add_duration = [&status](const std::string& key, std::int64_t value) {
    diagnostic_msgs::msg::KeyValue item;
    item.key = key;
    item.value = std::to_string(std::max<std::int64_t>(0, value));
    status.values.push_back(std::move(item));
  };
  const auto& map_diagnostics = mapping.map;
  const auto accounting = observation_accounting_.snapshot();
  addObservationAccountingValues(status, accounting);
  add_value("cycle_count", cycle_count_);
  add_value("trajectory_publish_count", cycle_success_count_);
  add_value("stale_input_count", stale_input_count_);
  add_value("stale_mapping_input_count",
            stale_mapping_input_count_.load() + mapping.discarded_stale);
  add_value("future_mapping_input_count",
            future_mapping_input_count_.load() + mapping.discarded_future);
  add_value("worker_discarded_stale_count", mapping.discarded_stale);
  add_value("worker_discarded_future_count", mapping.discarded_future);
  add_value("worker_discarded_invalid_count", mapping.discarded_invalid);
  add_value("stale_execution_state_count", stale_execution_state_count_);
  add_value("future_execution_state_count", future_execution_state_count_);
  add_value("invalid_corrected_pose_count",
            invalid_corrected_pose_count_.load() + mapping.discarded_invalid);
  add_value("corrected_pair_mismatch_count", corrected_pair_mismatch_count_);
  add_value("invalid_execution_state_count", invalid_execution_state_count_);
  add_value("command_execution_lease_rejection_count",
            command_execution_lease_rejection_count_);
  add_value("command_execution_lease_terminal_latch_count",
            command_execution_lease_terminal_latch_count_);
  add_value("command_execution_lease_failed",
            command_execution_lease_failure_latch_.latched() ? 1U : 0U);
  add_value("command_execution_lease_reason", command_execution_lease_reason_.load());
  add_value("command_execution_source_age_us",
            command_execution_source_age_us_.load());
  add_value("command_execution_receive_age_us",
            command_execution_receive_age_us_.load());
  add_value("processing_exception_count", map_update_exception_count_);
  add_value("mapping_input_point_count", map_diagnostics.endpoint_count);
  add_value("mapping_allocated_voxel_count", map_diagnostics.allocated_voxel_count);
  add_value("localization_epoch",
            active_localization_epoch_.load(std::memory_order_acquire));
  add_value("localization_epoch_ready",
            localization_epoch_ready_.load(std::memory_order_acquire) ? 1U : 0U);
  add_value("registered_scan_epoch",
            last_registered_scan_epoch_.load(std::memory_order_acquire));
  add_value("registered_scan_sequence",
            last_registered_scan_sequence_.load(std::memory_order_acquire));
  add_value("world_generation", mapping.world_generation);
  add_value("world_revision", mapping.world_revision);
  add_value("world_snapshot_bytes", mapping.snapshot_bytes);
  add_value("world_snapshot_owned_bytes", mapping.snapshot_owned_bytes);
  add_value("world_snapshot_shared_metadata_bytes", mapping.snapshot_shared_metadata_bytes);
  add_value("world_snapshot_live_count", navigation_mapping::MappingWorldSnapshot::liveCount());
  add_value("world_snapshot_peak_live_count", navigation_mapping::MappingWorldSnapshot::peakLiveCount());
  add_value("world_snapshot_live_owned_bytes", navigation_mapping::MappingWorldSnapshot::liveOwnedBytes());
  add_value("world_snapshot_peak_live_owned_bytes", navigation_mapping::MappingWorldSnapshot::peakLiveOwnedBytes());
  add_duration("mapping_input_lock_wait_us", last_input_lock_wait_us_);
  add_duration("planning_scheduling_gap_us", last_planning_scheduling_gap_us_);
  diagnostics.status.push_back(std::move(status));
  diagnostics_publisher_->publish(diagnostics);

  // A localization reset has invalidated the previous world and command
  // epoch. Do not feed stale propagated state into planner backend until the mapping
  // worker has published the first snapshot of the new epoch.
  if (!localization_epoch_ready_.load(std::memory_order_acquire)) return;

  // Mapping runs independently in its sole-owner worker. Planner execution state is independently
  // sourced from the latest propagated odometry and may be unavailable or
  // invalid without suppressing a corrected observation from ROG-Map.
  if (!propagated_state) return;
  const auto execution_stamp_ns = propagated_state->state.source_stamp_ns;
  const auto execution_age_ns = now_ns - execution_stamp_ns;
  const auto execution_freshness =
      navigation_execution::classifyTimestampFreshness(now_ns, execution_stamp_ns, maximum_age_ns);
  if (execution_freshness != navigation_execution::TimestampFreshness::VALID) {
    ++stale_input_count_;
    if (execution_freshness == navigation_execution::TimestampFreshness::STALE) ++stale_execution_state_count_;
    if (execution_freshness == navigation_execution::TimestampFreshness::FUTURE) ++future_execution_state_count_;
    if (execution_freshness == navigation_execution::TimestampFreshness::INVALID) ++invalid_execution_state_count_;
    return;
  }
  const auto& execution_state = propagated_state->state;
  if (!execution_state.finite() || !planner_->setState(execution_state)) {
    ++invalid_execution_state_count_;
    return;
  }

  // planner backend may produce a successful local trajectory ending at the current
  // sensing frontier rather than at the mission goal. Once it finishes,
  // restart PlanFromRest for the same logical goal so newly observed map
  // cells can extend the route. Mapping has already published this cycle.
  const bool completed_trajectory = trajectory_finished_.exchange(false);
  if (completed_trajectory && goal && trajectory_reaches_goal_.load()) return;
  if (completed_trajectory && goal && !trajectory_reaches_goal_.load()) {
    const auto& target_message = plannerTarget(*goal);
    const double dx = pointFromMessage(target_message, 0) - execution_state.position_world.x();
    const double dy = pointFromMessage(target_message, 1) - execution_state.position_world.y();
    const double dz = pointFromMessage(target_message, 2) - execution_state.position_world.z();
    if (std::sqrt(dx * dx + dy * dy + dz * dz) >
        navigation_world_model::kGoalCompletionToleranceM) {
      std::lock_guard<std::mutex> lock(input_mutex_);
      if (active_goal_ && active_goal_->mission_id == goal->mission_id &&
          active_goal_->waypoint_index == goal->waypoint_index &&
          active_goal_->request_id == goal->request_id) {
        restart_from_rest_ = true;
        hot_goal_transition_ = false;
        restart_from_rest = true;
        skip_replan_once_ = false;
      }
      RCLCPP_INFO(get_logger(),
                  "planner backend local trajectory finished before goal; restarting PlanFromRest "
                  "goal=(%.2f,%.2f,%.2f) vehicle=(%.2f,%.2f,%.2f)",
                  pointFromMessage(target_message, 0), pointFromMessage(target_message, 1),
                  pointFromMessage(target_message, 2), execution_state.position_world.x(),
                  execution_state.position_world.y(), execution_state.position_world.z());
    }
  }

  if (!goal) return;
  // A terminal planner failure remains terminal for this request until the
  // mission controller acknowledges it and cancels the goal. Without this
  // gate the next planning timer could perform a fourth solve and overwrite
  // the fail-closed state with a newly discovered frontier trajectory.
  if (planner_failure_latched_.load()) return;
  // While the command publisher drains a committed safety suffix, keep
  // replanning from the current vehicle state.  A validated main trajectory
  // may replace the suffix and recover the mission; failed solves leave the
  // frozen suffix untouched.  Stopping planner callbacks here would make
  // recovery impossible by construction and force every transient hot-replan
  // miss to end in hold/handover.

  // Planner execution state is independently owned by propagated odometry;
  // ROG-Map's corrected scan-epoch pose is mapping-only.
  // Always retain the current mission checkpoint as planner backend's geometric target.
  // PASS_THROUGH is implemented by hot-retargeting while the committed
  // trajectory still carries non-zero PVA, not by skipping to next_target.
  const auto& planner_target = plannerTarget(*goal);
  const navigation_planning_backend::math::Vec3f target{
      static_cast<float>(pointFromMessage(planner_target, 0)),
      static_cast<float>(pointFromMessage(planner_target, 1)),
      static_cast<float>(pointFromMessage(planner_target, 2))};
  const auto planner_started = std::chrono::steady_clock::now();
  // This is the internal planner backend FSM boundary: each mission waypoint enters
  // PlanFromRest once, then every subsequent planning tick is ReplanOnce.
  // planner backend itself owns the committed main/backup timing; the ROS adapter must
  // not add a second horizon-expiry or trajectory-slicing policy.
  const bool plan_from_rest = new_goal || restart_from_rest;
  const bool replan_for_new_goal = hot_goal_transition && !plan_from_rest;
  if (new_goal) {
    // MissionController has invalidated the previous waypoint already. Do
    // not publish that waypoint while PlanFromRest runs.
    std::lock_guard<std::mutex> command_lock(
        command_execution_lease_failure_latch_.transitionMutex());
    planner_command_available_.store(false);
    planner_failure_latched_.store(false);
    trajectory_reaches_goal_.store(false);
  }
  if (!plan_from_rest && !replan_for_new_goal && skip_replan_once_) {
    skip_replan_once_ = false;
    return;
  }
  navigation_planning_backend::RET_CODE result = navigation_planning_backend::FAILED;
  const std::uint64_t solve_generation = ++planner_solve_generation_;
  const std::uint64_t localization_epoch_at_solve =
      active_localization_epoch_.load(std::memory_order_acquire);
  const std::uint64_t committed_generation_before_solve =
      planner_->committedGenerationSnapshot();
  const auto pinned_world = world_snapshot_store_.load();
  if (!pinned_world) {
    RCLCPP_ERROR(get_logger(), "planner backend cannot solve without a published WorldModel snapshot");
    return;
  }
  planner_->setWorldModelView(pinned_world.view);
  // Reset diagnostic-only optimizer evidence so a solve that bypasses EXP
  // cannot inherit retry metrics from the previous planning generation.
  planner_->resetExpOptimizationDiagnostics();
  planner_solve_started_steady_ns_.store(navigation_common::steadyClockNowNanoseconds());
  active_planner_solve_generation_.store(solve_generation);
  planner_->resetSolveCancellation();
  const auto solve_started_ros_ns = now().nanoseconds();
  const double execution_age_at_solve_ms =
      executionStateAgeMs(solve_started_ros_ns, execution_stamp_ns);
  try {
    result = plan_from_rest
                 ? planner_->PlanFromRest(target, 0.0, true)
                 : planner_->ReplanOnce(target, 0.0, replan_for_new_goal);
  } catch (const std::exception& error) {
    RCLCPP_ERROR(get_logger(), "planner backend planner exception: %s", error.what());
    result = navigation_planning_backend::EMER;
  }
  std::uint64_t expected_active_generation = solve_generation;
  active_planner_solve_generation_.compare_exchange_strong(
      expected_active_generation, 0U);
  last_planner_us_ = std::chrono::duration_cast<std::chrono::microseconds>(
      std::chrono::steady_clock::now() - planner_started).count();
  if (timed_out_planner_solve_generation_.load() == solve_generation) {
    RCLCPP_ERROR(get_logger(),
                 "Discarding planner backend solve generation=%lu after planner watchdog timeout",
                 static_cast<unsigned long>(solve_generation));
    return;
  }
  if (!localization_epoch_ready_.load(std::memory_order_acquire) ||
      active_localization_epoch_.load(std::memory_order_acquire) !=
          localization_epoch_at_solve) {
    RCLCPP_WARN(get_logger(),
                "Discarding planner backend solve generation=%lu after localization epoch transition",
                static_cast<unsigned long>(solve_generation));
    return;
  }
  std::vector<double> module_times;
  planner_->getModuleTimeConsuming(module_times);
  const auto exp_diagnostics = planner_->latestExpOptimizationDiagnostics();
  {
    std::lock_guard<std::mutex> lock(input_mutex_);
    // A terminal mission status or a newer waypoint may arrive while the
    // optimizer is running. The completed solve is then stale: never expose
    // its internally committed trajectory to the command publisher.
    if (!active_goal_ || active_goal_->mission_id != goal->mission_id ||
        active_goal_->waypoint_index != goal->waypoint_index ||
        active_goal_->request_id != goal->request_id) {
      return;
    }
  }
  const auto committed_metadata_after_solve =
      planner_->committedMetadataSnapshot();
  const bool solve_committed_new_generation = commitObservedThisCycle(
      committed_generation_before_solve,
      committed_metadata_after_solve.generation,
      committed_metadata_after_solve.diagnostics.generation);
  const auto disposition = classifyPlannerResult(
      result, plan_from_rest, planner_command_available_.load(),
      solve_committed_new_generation);
  if (disposition == PlannerResultDisposition::FailClosed) {
    planner_failure_latched_.store(true);
    trajectory_reaches_goal_.store(false);
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "planner backend planning failed (%d)", result);
  }
  if (disposition == PlannerResultDisposition::RestartFromRest) {
    planner_failure_latched_.store(false);
    trajectory_reaches_goal_.store(false);
    trajectory_finished_.store(true);
    std::lock_guard<std::mutex> lock(input_mutex_);
    if (active_goal_ && active_goal_->mission_id == goal->mission_id &&
        active_goal_->waypoint_index == goal->waypoint_index &&
        active_goal_->request_id == goal->request_id) {
      restart_from_rest_ = true;
      hot_goal_transition_ = false;
    }
    RCLCPP_INFO(get_logger(),
                "planner backend local trajectory boundary reached; scheduling PlanFromRest");
  }
  if (disposition == PlannerResultDisposition::RetryFromRest) {
    trajectory_reaches_goal_.store(false);
    bool failure_budget_exhausted = false;
    std::uint32_t failure_count = 0U;
    {
      std::lock_guard<std::mutex> lock(input_mutex_);
      if (active_goal_ && active_goal_->mission_id == goal->mission_id &&
          active_goal_->waypoint_index == goal->waypoint_index &&
          active_goal_->request_id == goal->request_id) {
        const auto failure_now_ns = navigation_common::steadyClockNowNanoseconds();
        if (plan_from_rest_first_failure_steady_ns_ == 0) {
          plan_from_rest_first_failure_steady_ns_ = failure_now_ns;
        }
        const bool count_exhausted = plan_from_rest_failure_budget_.recordFailure();
        const double failure_window_s = static_cast<double>(
            failure_now_ns - plan_from_rest_first_failure_steady_ns_) * 1.0e-9;
        failure_budget_exhausted = count_exhausted &&
            failure_window_s >= plan_from_rest_failure_confirmation_s_;
        failure_count = plan_from_rest_failure_budget_.failureCount();
      }
    }
    planner_failure_latched_.store(failure_budget_exhausted);
    if (failure_budget_exhausted) {
      RCLCPP_ERROR(get_logger(),
                   "planner backend PlanFromRest failed %u consecutive times; fail-closed for "
                   "mission=%s waypoint=%u",
                   failure_count, goal->mission_id.c_str(), goal->waypoint_index);
    } else {
      RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "planner backend PlanFromRest transient failure (%d); retry %u/%u", result,
          failure_count, max_plan_from_rest_failures_);
    }
  }
  if (disposition == PlannerResultDisposition::RetainCommittedCommand ||
      disposition == PlannerResultDisposition::ValidateRetainedCommand) {
    const bool validate_without_new_commit =
        disposition == PlannerResultDisposition::ValidateRetainedCommand;
    planner_->lockCommittedTraj();
    const auto committed = planner_->getCommittedPositionTrajectory();
    const auto committed_yaw = planner_->getCommittedYawTrajectory();
    const bool backup_available = planner_->committedBackupTrajectoryAvailable();
    const double backup_start_s = planner_->getCommittedBackupStartTrajectoryTime();
    planner_->unlockCommittedTraj();

    const double elapsed_s = committed.empty()
                                 ? std::numeric_limits<double>::infinity()
                                 : now().seconds() - committed.start_WT;
    const double total_duration_s = committed.getTotalDuration();
    const double clamped_elapsed_s =
        std::clamp(elapsed_s, 0.0, std::max(0.0, total_duration_s));
    const auto command_anchor = committed.empty()
                                    ? navigation_planning_backend::math::Vec3f{}
                                    : committed.getPos(clamped_elapsed_s);
    // ReplanOnce may run for more than a second while command publication and
    // vehicle motion continue concurrently. The planner state captured before
    // that solve is therefore stale by construction. Re-read the immutable
    // execution lease after the solve and apply the same dual-clock contract
    // used by command publication; a stale receive must not rescue a retained
    // or emergency command.
    const auto retained_execution_state = execution_state_store_.load();
    const auto retained_state_freshness = retained_execution_state
        ? navigation_contracts::evaluateExecutionStateFreshness(
              now().nanoseconds(), retained_execution_state->state.source_stamp_ns,
              navigation_common::steadyClockNowNanoseconds(),
              retained_execution_state->state.receive_stamp_ns,
              input_max_age_s_)
        : navigation_contracts::ExecutionStateFreshness{};
    navigation_planning_backend::math::Vec3f current_vehicle_position =
        navigation_planning_backend::math::Vec3f::Zero();
    navigation_planning_backend::math::Vec3f current_vehicle_velocity =
        navigation_planning_backend::math::Vec3f::Zero();
    double current_vehicle_yaw = 0.0;
    const bool fresh_vehicle_state = retained_execution_state &&
                                     retained_execution_state->state.finite() &&
                                     retained_state_freshness.valid();
    const double latest_vehicle_state_age_s = retained_execution_state
        ? retained_state_freshness.source_age_ms * 1.0e-3
        : std::numeric_limits<double>::infinity();
    if (fresh_vehicle_state) {
      current_vehicle_position = retained_execution_state->state.position_world;
      current_vehicle_velocity = retained_execution_state->state.velocity_world;
      current_vehicle_yaw = retained_execution_state->state.yaw_rad;
    }
    const double anchor_error_m = committed.empty() || !fresh_vehicle_state
                                      ? std::numeric_limits<double>::infinity()
                                      : (command_anchor - current_vehicle_position).norm();
    bool sampled_path_clear = !committed.empty();
    double first_blocked_sample_s = std::numeric_limits<double>::quiet_NaN();
    navigation_planning_backend::math::Vec3f first_blocked_sample = navigation_planning_backend::math::Vec3f::Constant(
        std::numeric_limits<double>::quiet_NaN());
    navigation_world_model::CellState first_blocked_grid =
        navigation_world_model::CellState::kUnknown;
    if (sampled_path_clear) {
      navigation_planning_backend::CandidateCommandBundle retained;
      retained.position = committed;
      retained.yaw = committed_yaw;
      retained.start_wall_time = committed.start_WT;
      const auto latest_world = world_snapshot_store_.latest();
      const auto validation = latest_world
          ? navigation_planning_backend::validateExecutableCandidate(
                *latest_world.view, retained, now().seconds())
          : navigation_planning_backend::SweptValidationResult{};
      sampled_path_clear = validation.valid;
      if (!sampled_path_clear) {
        first_blocked_sample_s = std::isfinite(validation.first_blocked_tt)
            ? validation.first_blocked_tt : clamped_elapsed_s;
        first_blocked_sample = committed.getPos(std::clamp(
            first_blocked_sample_s, 0.0, total_duration_s));
        const auto state = latest_world
            ? latest_world.view->classify(
                  first_blocked_sample,
                  navigation_world_model::GridLayer::kInflated)
            : navigation_world_model::CellState::kOutOfMap;
        first_blocked_grid = state == navigation_world_model::CellState::kOccupied
            ? navigation_world_model::CellState::kOccupied
            : (state == navigation_world_model::CellState::kOutOfMap
                   ? navigation_world_model::CellState::kOutOfMap
                   : navigation_world_model::CellState::kUnknown);
      }
    }
    // If replanning fails after the main-to-backup switch, the usable safety
    // suffix starts at the current command anchor, not in the past.
    const double safety_transition_s = backup_available
                                           ? std::max(backup_start_s, clamped_elapsed_s)
                                           : clamped_elapsed_s;
    bool use_safety_suffix = committedSafetySuffixIsUsable(
        backup_available, elapsed_s, total_duration_s,
        safety_transition_s,
        anchor_error_m, max_safety_suffix_anchor_error_m_, sampled_path_clear);
    bool emergency_brake_committed = false;
    if (!validate_without_new_commit && !use_safety_suffix && fresh_vehicle_state &&
        !committed.empty() &&
        !committed_yaw.empty()) {
      // Position and velocity are measured at the newest propagated odometry
      // sample. Acceleration/jerk and yaw-rate are not measured by the LIO
      // interface, so retain their continuous values from the command that
      // PX4 was tracking at the same command-clock instant.
      navigation_planning_backend::math::StatePVAJ emergency_state = committed.getState(clamped_elapsed_s);
      emergency_state.col(0) = current_vehicle_position;
      emergency_state.col(1) = current_vehicle_velocity;
      const double emergency_yaw_dot = committed_yaw.getVel(clamped_elapsed_s).x();
      emergency_brake_committed = planner_->commitEmergencyBrake(
          emergency_state, current_vehicle_yaw, emergency_yaw_dot, now().seconds());
      use_safety_suffix = emergency_brake_committed;
    }
    if (emergency_brake_committed &&
        !commitPlannerCandidate(*goal, goal_epoch, localization_epoch_at_solve,
                                now().nanoseconds())) {
      emergency_brake_committed = false;
      use_safety_suffix = false;
      RCLCPP_ERROR(get_logger(),
                   "execution boundary rejected the measured-state emergency candidate; "
                   "clearing command exposure");
    }
    const auto retained_transition = retainedValidationTransition(use_safety_suffix);
    // A visible main-only trajectory remains a MAIN command. Only an actual
    // atomic main-to-backup bundle is marked safety-owned at the PX4 boundary.
    {
      std::lock_guard<std::mutex> command_lock(
          command_execution_lease_failure_latch_.transitionMutex());
      if (!localization_epoch_ready_.load(std::memory_order_acquire) ||
          active_localization_epoch_.load(std::memory_order_acquire) !=
              localization_epoch_at_solve ||
          active_goal_epoch_.load() != goal_epoch) {
        // A newer goal owns command state now. This old solve is discard-only:
        // never invalidate a deliberately transferred hot-retarget command.
      } else if (!command_execution_lease_failure_latch_.allowsCommandExposure()) {
        planner_command_available_.store(false);
        planner_failure_latched_.store(true);
        safety_suffix_active_.store(false);
      } else if (!validate_without_new_commit ||
                 retained_transition == RetainedValidationTransition::FailClosed) {
        safety_suffix_active_.store(
            use_safety_suffix && (backup_available || emergency_brake_committed));
        planner_failure_latched_.store(!use_safety_suffix);
        if (!use_safety_suffix) {
          planner_command_available_.store(false);
          command_goal_epoch_.store(0U);
        } else if (emergency_brake_committed) {
          planner_command_available_.store(true);
          command_goal_epoch_.store(goal_epoch);
          trajectory_finished_.store(false);
        }
      }
    }
    if (emergency_brake_committed &&
        command_execution_lease_failure_latch_.allowsCommandExposure()) {
      RCLCPP_WARN(get_logger(),
                  "planner backend replaced unusable committed suffix with a measured-state "
                  "emergency backup trajectory");
    }
    if (use_safety_suffix && validate_without_new_commit) {
      RCLCPP_DEBUG(get_logger(),
                   "planner backend reported NO_NEED; retained committed command remains "
                   "latest-world valid without a new commit");
    } else if (use_safety_suffix) {
      RCLCPP_WARN(get_logger(),
                  "planner backend hot replan failed (%d); retaining visible committed trajectory "
                  "backup=%d elapsed=%.3f backup_start=%.3f end=%.3f anchor_error=%.3f",
                  result, backup_available, elapsed_s, safety_transition_s,
                  total_duration_s, anchor_error_m);
    } else {
      RCLCPP_ERROR(get_logger(),
                   "planner backend hot replan failed without a valid safety suffix: backup=%d "
                   "elapsed=%.3f backup_start=%.3f end=%.3f anchor_error=%.3f "
                   "state_age=%.3f clear=%d blocked_t=%.3f blocked_grid=%d "
                   "blocked=(%.2f,%.2f,%.2f)",
                   backup_available, elapsed_s,
                   safety_transition_s, total_duration_s,
                   anchor_error_m, latest_vehicle_state_age_s, sampled_path_clear,
                   first_blocked_sample_s, static_cast<int>(first_blocked_grid),
                   first_blocked_sample.x(), first_blocked_sample.y(),
                   first_blocked_sample.z());
    }
  }
  if (disposition == PlannerResultDisposition::CommandReady) {
    {
      std::lock_guard<std::mutex> command_lock(
          command_execution_lease_failure_latch_.transitionMutex());
      if (!localization_epoch_ready_.load(std::memory_order_acquire) ||
          active_localization_epoch_.load(std::memory_order_acquire) !=
              localization_epoch_at_solve ||
          active_goal_epoch_.load() != goal_epoch) {
        return;
      }
      if (!command_execution_lease_failure_latch_.allowsCommandExposure()) {
        planner_command_available_.store(false);
        planner_failure_latched_.store(true);
        safety_suffix_active_.store(false);
        return;
      }
      if (timed_out_planner_solve_generation_.load() == solve_generation) {
        planner_command_available_.store(false);
        planner_failure_latched_.store(true);
        safety_suffix_active_.store(false);
        return;
      }
      planner_command_available_.store(false);
      command_goal_epoch_.store(0U);
      planner_failure_latched_.store(false);
      safety_suffix_active_.store(false);
      trajectory_finished_.store(false);
    }
    planner_->lockCommittedTraj();
    const auto committed = planner_->getCommittedPositionTrajectory();
    planner_->unlockCommittedTraj();
    const auto committed_end = committed.empty()
                                   ? navigation_planning_backend::math::Vec3f{}
                                   : committed.getPos(committed.getTotalDuration());
    trajectory_reaches_goal_.store(
        !committed.empty() &&
        (committed_end - target).norm() <=
            navigation_world_model::kGoalCompletionToleranceM);
    if (!commitPlannerCandidate(*goal, goal_epoch, localization_epoch_at_solve,
                                now().nanoseconds())) {
      std::lock_guard<std::mutex> command_lock(
          command_execution_lease_failure_latch_.transitionMutex());
      planner_command_available_.store(false);
      command_goal_epoch_.store(0U);
      safety_suffix_active_.store(false);
      RCLCPP_WARN(get_logger(),
                  "execution boundary rejected planner candidate after solve; "
                  "command remains fail-closed");
      return;
    }
    {
      std::lock_guard<std::mutex> command_lock(
          command_execution_lease_failure_latch_.transitionMutex());
      if (!localization_epoch_ready_.load(std::memory_order_acquire) ||
          active_localization_epoch_.load(std::memory_order_acquire) !=
              localization_epoch_at_solve ||
          active_goal_epoch_.load() != goal_epoch ||
          !command_execution_lease_failure_latch_.allowsCommandExposure()) {
        planner_command_available_.store(false);
        command_goal_epoch_.store(0U);
        safety_suffix_active_.store(false);
        return;
      }
      planner_command_available_.store(true);
      command_goal_epoch_.store(goal_epoch);
    }
    if (plan_from_rest) skip_replan_once_ = true;
    std::lock_guard<std::mutex> lock(input_mutex_);
    plan_from_rest_failure_budget_.reset();
    plan_from_rest_first_failure_steady_ns_ = 0;
    // Do not fall back to hot replan after a failed new-goal attempt.  That
    // would keep publishing the previous waypoint while the mission has
    // already advanced.
    if (active_goal_ && active_goal_->mission_id == goal->mission_id &&
        active_goal_->waypoint_index == goal->waypoint_index &&
        active_goal_->request_id == goal->request_id) {
      if (new_goal) new_goal_ = false;
      if (plan_from_rest) hot_goal_transition_ = false;
      if (replan_for_new_goal) hot_goal_transition_ = false;
      if (restart_from_rest) restart_from_rest_ = false;
    }
  }

  // Emit one decision record for every solve generation.  Successful hot
  // replans are as important as failures for latency distributions and for
  // proving that sampled commands came from one committed generation.
  {
    const auto committed_snapshot = planner_->committedTrajectorySnapshot();
    const auto& committed = committed_snapshot.position;
    const auto committed_generation = committed_snapshot.generation;
    const auto& committed_certificate = committed_snapshot.certificate;
    const auto& commit_diagnostics = committed_snapshot.diagnostics;
    const bool commit_observed_this_cycle = commitObservedThisCycle(
        committed_generation_before_solve, committed_generation,
        commit_diagnostics.generation);
    const bool has_committed_bundle = committed_generation > 0U && !committed.empty();
    const auto committed_end = !has_committed_bundle
                                   ? navigation_planning_backend::math::Vec3f{}
                                   : committed.getPos(committed.getTotalDuration());
    const double endpoint_error = !has_committed_bundle
                                      ? std::numeric_limits<double>::infinity()
                                      : (committed_end - target).norm();
    const auto robot_grid_type = pinned_world.view->classify(
        execution_state.position_world, navigation_world_model::GridLayer::kEvidence);
    const auto robot_inflated_grid_type = pinned_world.view->classify(
        execution_state.position_world, navigation_world_model::GridLayer::kInflated);
    // WorldModel deliberately has no vendor-specific nearest-known-free or
    // nearest-occupied query. Keep these legacy diagnostics unavailable rather
    // than reaching back into worker-owned mutable ROG state.
    const double nearest_known_free_distance =
        std::numeric_limits<double>::quiet_NaN();
    navigation_planning_backend::math::Vec3f nearest_occupied = navigation_planning_backend::math::Vec3f::Constant(
        std::numeric_limits<double>::quiet_NaN());
    const double nearest_occupied_distance =
        std::numeric_limits<double>::quiet_NaN();
    const int solve_stage = planner_->solveStage();
    const int replan_return_code = planner_->latestReplanReturnCode();
    const int commit_decision = planner_->latestCommitDecision();
    const double planner_elapsed_ms = static_cast<double>(last_planner_us_) * 1.0e-3;
    const bool solve_deadline_exceeded =
        planner_elapsed_ms > planner_->solveDeadlineSeconds() * 1000.0;
    const auto trace_now_ns = now().nanoseconds();
    const double execution_age_at_trace_ms =
        static_cast<double>(trace_now_ns - execution_stamp_ns) * 1.0e-6;
    RCLCPP_INFO(get_logger(),
                "planner backend decision_trace cycle=%lu solve_generation=%lu committed_generation=%lu "
                "pinned_world_generation=%lu pinned_world_revision=%lu "
                "pinned_world_stamp_ns=%ld "
                "certificate_world_generation=%lu certificate_world_revision=%lu "
                "certificate_world_stamp_ns=%ld "
                "cloud_stamp_ns=%ld corrected_stamp_ns=%ld propagated_stamp_ns=%ld "
                "state_age_ms=%.3f mode=%s result=%d replan_code=%d commit_decision=%d "
                "solve_stage=%d solve_stage_name=%s solve_elapsed_ms=%.3f "
                "solve_deadline_exceeded=%d target=(%.2f,%.2f,%.2f) "
                "committed_end=(%.2f,%.2f,%.2f) endpoint_error=%.3f command=%d failure=%d "
                "exp_frontend_ms=%.3f exp_opt_ms=%.3f backup_frontend_ms=%.3f "
                "backup_opt_ms=%.3f robot_grid=%d robot_inf_grid=%d nearest_free_m=%.3f "
                "nearest_occ_m=%.3f nearest_occ=(%.2f,%.2f,%.2f)",
                static_cast<unsigned long>(cycle_count_),
                static_cast<unsigned long>(solve_generation),
                static_cast<unsigned long>(committed_generation),
                static_cast<unsigned long>(pinned_world.identity.generation),
                static_cast<unsigned long>(pinned_world.identity.revision),
                static_cast<long>(pinned_world.identity.observation_stamp_ns),
                static_cast<unsigned long>(committed_certificate.validated_world.generation),
                static_cast<unsigned long>(committed_certificate.validated_world.revision),
                static_cast<long>(committed_certificate.validated_world.observation_stamp_ns),
                static_cast<long>(cloud_stamp_ns), static_cast<long>(corrected_stamp_ns),
                static_cast<long>(execution_stamp_ns),
                static_cast<double>(execution_age_ns) * 1e-6,
                plan_from_rest ? "PlanFromRest" : "ReplanOnce", result,
                replan_return_code, commit_decision, solve_stage,
                navigation_planning_backend::solveStageName(solve_stage).data(), planner_elapsed_ms,
                solve_deadline_exceeded, target.x(), target.y(),
                target.z(), committed_end.x(), committed_end.y(), committed_end.z(),
                endpoint_error, planner_command_available_.load(), planner_failure_latched_.load(),
                module_times.size() > navigation_planning_backend::EPX_TRAJ_FRONTEND
                    ? module_times[navigation_planning_backend::EPX_TRAJ_FRONTEND] * 1000.0 : 0.0,
                module_times.size() > navigation_planning_backend::EXP_TRAJ_OPT
                    ? module_times[navigation_planning_backend::EXP_TRAJ_OPT] * 1000.0 : 0.0,
                module_times.size() > navigation_planning_backend::BACK_TRAJ_FRONTEND
                    ? module_times[navigation_planning_backend::BACK_TRAJ_FRONTEND] * 1000.0 : 0.0,
                module_times.size() > navigation_planning_backend::BACK_TRAJ_OPT
                    ? module_times[navigation_planning_backend::BACK_TRAJ_OPT] * 1000.0 : 0.0,
                static_cast<int>(robot_grid_type), static_cast<int>(robot_inflated_grid_type),
                nearest_known_free_distance, nearest_occupied_distance,
                nearest_occupied.x(), nearest_occupied.y(), nearest_occupied.z());

    if (commit_observed_this_cycle) {
      RCLCPP_INFO(get_logger(),
                  "planner backend commit_trace generation=%lu previous_generation=%lu "
                  "start_wall_time=%.9f execution_stamp_ns=%ld "
                  "execution_age_at_solve_ms=%.3f execution_age_at_trace_ms=%.3f "
                  "execution_p=[%.6f,%.6f,%.6f] execution_v=[%.6f,%.6f,%.6f] "
                  "candidate_start_p=[%.6f,%.6f,%.6f] "
                  "candidate_start_v=[%.6f,%.6f,%.6f] previous_valid=%d "
                  "candidate_start_a=[%.6f,%.6f,%.6f] "
                  "candidate_start_j=[%.6f,%.6f,%.6f] "
                  "previous_sample_tt=%.6f splice_p=%.6f splice_v=%.6f "
                  "splice_a=%.6f splice_j=%.6f splice_yaw=%.6f "
                  "splice_yaw_rate=%.6f",
                  static_cast<unsigned long>(commit_diagnostics.generation),
                  static_cast<unsigned long>(commit_diagnostics.previous_generation),
                  commit_diagnostics.candidate_start_wall_time,
                  static_cast<long>(execution_stamp_ns),
                  execution_age_at_solve_ms,
                  execution_age_at_trace_ms,
                  execution_state.position_world.x(), execution_state.position_world.y(),
                  execution_state.position_world.z(), execution_state.velocity_world.x(),
                  execution_state.velocity_world.y(), execution_state.velocity_world.z(),
                  commit_diagnostics.candidate_start_pvaj(0, 0),
                  commit_diagnostics.candidate_start_pvaj(1, 0),
                  commit_diagnostics.candidate_start_pvaj(2, 0),
                  commit_diagnostics.candidate_start_pvaj(0, 1),
                  commit_diagnostics.candidate_start_pvaj(1, 1),
                  commit_diagnostics.candidate_start_pvaj(2, 1),
                  commit_diagnostics.previous_valid ? 1 : 0,
                  commit_diagnostics.candidate_start_pvaj(0, 2),
                  commit_diagnostics.candidate_start_pvaj(1, 2),
                  commit_diagnostics.candidate_start_pvaj(2, 2),
                  commit_diagnostics.candidate_start_pvaj(0, 3),
                  commit_diagnostics.candidate_start_pvaj(1, 3),
                  commit_diagnostics.candidate_start_pvaj(2, 3),
                  commit_diagnostics.previous_sample_tt,
                  commit_diagnostics.position_residual.norm(),
                  commit_diagnostics.velocity_residual.norm(),
                  commit_diagnostics.acceleration_residual.norm(),
                  commit_diagnostics.jerk_residual.norm(),
                  commit_diagnostics.yaw_residual,
                  commit_diagnostics.yaw_rate_residual);
    }

    diagnostic_msgs::msg::DiagnosticArray trace_diagnostics;
    trace_diagnostics.header.stamp = now();
    diagnostic_msgs::msg::DiagnosticStatus trace_status;
    trace_status.name = "navigation_runtime/planner";
    trace_status.level = result == navigation_planning_backend::SUCCESS
                             ? diagnostic_msgs::msg::DiagnosticStatus::OK
                             : diagnostic_msgs::msg::DiagnosticStatus::WARN;
    trace_status.message = "DECISION_TRACE";
    const auto add_trace_value = [&trace_status](const std::string& key,
                                                  const auto& value) {
      diagnostic_msgs::msg::KeyValue item;
      item.key = key;
      item.value = std::to_string(value);
      trace_status.values.push_back(std::move(item));
    };
    const auto add_trace_vector = [&trace_status](const std::string& key,
                                                   const auto& value) {
      diagnostic_msgs::msg::KeyValue item;
      item.key = key;
      std::ostringstream stream;
      stream << std::setprecision(17) << '[' << value.x() << ',' << value.y()
             << ',' << value.z() << ']';
      item.value = stream.str();
      trace_status.values.push_back(std::move(item));
    };
    add_trace_value("planning_cycle_id", cycle_count_);
    add_trace_value("bundle_id", committed_generation);
    add_trace_value("solve_generation", solve_generation);
    add_trace_value("commit_observed_this_cycle", commit_observed_this_cycle ? 1 : 0);
    add_trace_value("execution_stamp_ns", execution_stamp_ns);
    add_trace_value("state_age_at_solve_ms",
                    execution_age_at_solve_ms);
    add_trace_value("state_age_at_trace_ms", execution_age_at_trace_ms);
    add_trace_vector("planning_state_position", execution_state.position_world);
    add_trace_vector("planning_state_velocity", execution_state.velocity_world);
    if (commit_observed_this_cycle) {
      add_trace_value("commit_previous_generation",
                      commit_diagnostics.previous_generation);
      add_trace_value("candidate_start_wall_time_s",
                      commit_diagnostics.candidate_start_wall_time);
      add_trace_vector("candidate_start_position",
                       commit_diagnostics.candidate_start_pvaj.col(0));
      add_trace_vector("candidate_start_velocity",
                       commit_diagnostics.candidate_start_pvaj.col(1));
      add_trace_vector("candidate_start_acceleration",
                       commit_diagnostics.candidate_start_pvaj.col(2));
      add_trace_vector("candidate_start_jerk",
                       commit_diagnostics.candidate_start_pvaj.col(3));
      add_trace_value("splice_previous_valid",
                      commit_diagnostics.previous_valid ? 1 : 0);
      add_trace_value("splice_previous_sample_tt_s",
                      commit_diagnostics.previous_sample_tt);
      add_trace_value("splice_position_residual_m",
                      commit_diagnostics.position_residual.norm());
      add_trace_value("splice_velocity_residual_mps",
                      commit_diagnostics.velocity_residual.norm());
      add_trace_value("splice_acceleration_residual_mps2",
                      commit_diagnostics.acceleration_residual.norm());
      add_trace_value("splice_jerk_residual_mps3",
                      commit_diagnostics.jerk_residual.norm());
      add_trace_value("splice_yaw_residual_rad", commit_diagnostics.yaw_residual);
      add_trace_value("splice_yaw_rate_residual_radps",
                      commit_diagnostics.yaw_rate_residual);
    }
    add_trace_value("pinned_world_generation", pinned_world.identity.generation);
    add_trace_value("pinned_world_revision", pinned_world.identity.revision);
    add_trace_value("pinned_world_stamp_ns", pinned_world.identity.observation_stamp_ns);
    add_trace_value("certificate_world_generation",
                    committed_certificate.validated_world.generation);
    add_trace_value("certificate_world_revision",
                    committed_certificate.validated_world.revision);
    add_trace_value("certificate_world_stamp_ns",
                    committed_certificate.validated_world.observation_stamp_ns);
    add_trace_value("candidate_result", static_cast<int>(result));
    add_trace_value("replan_code", replan_return_code);
    add_trace_value("commit_decision", commit_decision);
    add_trace_value("solve_stage", solve_stage);
    {
      diagnostic_msgs::msg::KeyValue item;
      item.key = "solve_stage_name";
      item.value = std::string(navigation_planning_backend::solveStageName(solve_stage));
      trace_status.values.push_back(std::move(item));
    }
    add_trace_value("planning_latency_ms", planner_elapsed_ms);
    // Keep planner stage timings explicit and unit-labelled in the structured
    // trace.  The raw planner backend log already exposes these values, but without
    // publishing them here the report cannot identify which stage causes a
    // solve deadline overrun.
    add_trace_value("exp_frontend_us",
                    module_times.size() > navigation_planning_backend::EPX_TRAJ_FRONTEND
                        ? module_times[navigation_planning_backend::EPX_TRAJ_FRONTEND] * 1.0e6
                        : 0.0);
    add_trace_value("exp_opt_us",
                    module_times.size() > navigation_planning_backend::EXP_TRAJ_OPT
                        ? module_times[navigation_planning_backend::EXP_TRAJ_OPT] * 1.0e6
                        : 0.0);
    add_trace_value("backup_frontend_us",
                    module_times.size() > navigation_planning_backend::BACK_TRAJ_FRONTEND
                        ? module_times[navigation_planning_backend::BACK_TRAJ_FRONTEND] * 1.0e6
                        : 0.0);
    add_trace_value("backup_opt_us",
                    module_times.size() > navigation_planning_backend::BACK_TRAJ_OPT
                        ? module_times[navigation_planning_backend::BACK_TRAJ_OPT] * 1.0e6
                        : 0.0);
    add_trace_value("exp_diagnostics_valid", exp_diagnostics.valid ? 1 : 0);
    add_trace_value("exp_lbfgs_attempt_count", exp_diagnostics.lbfgs_attempt_count);
    add_trace_value("exp_retry_count", exp_diagnostics.retry_count);
    add_trace_value("exp_retry_violation_mask", exp_diagnostics.retry_violation_mask);
    add_trace_value("exp_retry_stop_reason", exp_diagnostics.retry_stop_reason);
    add_trace_value("exp_lbfgs_first_return_code",
                    exp_diagnostics.first_lbfgs_return_code);
    add_trace_value("exp_lbfgs_last_return_code",
                    exp_diagnostics.last_lbfgs_return_code);
    add_trace_value("exp_lbfgs_cancelled", exp_diagnostics.cancelled ? 1 : 0);
    add_trace_value("exp_initial_normalized_dynamic_violation",
                    exp_diagnostics.initial_normalized_dynamic_violation);
    add_trace_value("exp_best_normalized_dynamic_violation",
                    exp_diagnostics.best_normalized_dynamic_violation);
    add_trace_value("exp_final_normalized_dynamic_violation",
                    exp_diagnostics.final_normalized_dynamic_violation);
    add_trace_value("exp_initial_duration_s", exp_diagnostics.initial_duration_s);
    add_trace_value("exp_final_duration_s", exp_diagnostics.final_duration_s);
    add_trace_value("exp_retry_duration_lower_bound_min_s",
                    exp_diagnostics.retry_duration_lower_bound_min_s);
    add_trace_value("exp_retry_duration_lower_bound_max_s",
                    exp_diagnostics.retry_duration_lower_bound_max_s);
    add_trace_value("exp_retry_free_duration_seed_min_s",
                    exp_diagnostics.retry_free_duration_seed_min_s);
    add_trace_value("exp_retry_free_duration_seed_max_s",
                    exp_diagnostics.retry_free_duration_seed_max_s);
    add_trace_value("guide_path_length_m", planner_->latestGuidePathLengthMeters());
    add_trace_value("guide_duration_s", planner_->latestGuideDurationSeconds());
    add_trace_value("exp_retry_budget_remaining_us",
                    exp_diagnostics.retry_budget_remaining_us);
    add_trace_value("exp_nonfinite_evaluation_count",
                    exp_diagnostics.nonfinite_evaluation_count);
    add_trace_value("exp_first_nonfinite_stage",
                    exp_diagnostics.first_nonfinite_stage);
    add_trace_value("exp_first_nonfinite_value_mask",
                    exp_diagnostics.first_nonfinite_value_mask);
    add_trace_value("exp_first_nonfinite_attempt",
                    exp_diagnostics.first_nonfinite_attempt);
    add_trace_value("exp_first_nonfinite_iteration",
                    exp_diagnostics.first_nonfinite_iteration);
    add_trace_value("exp_first_nonfinite_min_duration_s",
                    exp_diagnostics.first_nonfinite_min_duration_s);
    add_trace_value("exp_first_nonfinite_max_duration_s",
                    exp_diagnostics.first_nonfinite_max_duration_s);
    add_trace_value("exp_first_nonfinite_cost",
                    exp_diagnostics.first_nonfinite_cost);
    add_trace_value("exp_first_nonfinite_gradient_norm",
                    exp_diagnostics.first_nonfinite_gradient_norm);
    add_trace_value("solve_deadline_exceeded", solve_deadline_exceeded ? 1 : 0);
    add_trace_value("command_available", planner_command_available_.load() ? 1 : 0);
    add_trace_value("planner_failure_latched", planner_failure_latched_.load() ? 1 : 0);
    trace_diagnostics.status.push_back(std::move(trace_status));
    diagnostics_publisher_->publish(trace_diagnostics);
  }

  const auto metrics_now = std::chrono::steady_clock::now();
  const double planner_cycle_ms = static_cast<double>(
      std::chrono::duration_cast<std::chrono::microseconds>(metrics_now - cycle_started).count()) /
                                 1000.0;
  if (end_to_end_samples_ms_.size() == 256U) end_to_end_samples_ms_.erase(end_to_end_samples_ms_.begin());
  end_to_end_samples_ms_.push_back(planner_cycle_ms);
  if (metrics_now - metrics_log_time_ >= std::chrono::seconds(1)) {
    auto sorted = end_to_end_samples_ms_;
    std::sort(sorted.begin(), sorted.end());
    const auto percentile = [&](double fraction) {
      if (sorted.empty()) return 0.0;
      const auto index = std::min(sorted.size() - 1U,
                                  static_cast<std::size_t>(fraction * sorted.size()));
      return sorted[index];
    };
    planner_->lockCommittedTraj();
    const auto committed = planner_->getCommittedPositionTrajectory();
    const auto committed_generation = planner_->committedGenerationSnapshot();
    const bool backup_available = planner_->committedBackupTrajectoryAvailable();
    const double backup_start_s = planner_->getCommittedBackupStartTrajectoryTime();
    planner_->unlockCommittedTraj();
    const auto guide_end = planner_->latestGuideEnd();
    const auto guide_min = planner_->latestGuideMin();
    const auto guide_max = planner_->latestGuideMax();
    const bool committed_valid = committed_generation > 0U && !committed.empty();
    const auto committed_start = !committed_valid ? navigation_planning_backend::math::Vec3f{} : committed.getPos(0.0);
    const auto committed_end = !committed_valid
                                   ? navigation_planning_backend::math::Vec3f{}
                                   : committed.getPos(committed.getTotalDuration());
    const double committed_duration = committed.getTotalDuration();
    const auto committed_quarter = !committed_valid
                                       ? navigation_planning_backend::math::Vec3f{}
                                       : committed.getPos(committed_duration * 0.25);
    const auto committed_half = !committed_valid
                                    ? navigation_planning_backend::math::Vec3f{}
                                    : committed.getPos(committed_duration * 0.50);
    const auto committed_three_quarter = !committed_valid
                                             ? navigation_planning_backend::math::Vec3f{}
                                             : committed.getPos(committed_duration * 0.75);
    const auto main_end = !committed_valid || !backup_available
                              ? committed_end
                              : committed.getPos(std::clamp(
                                    backup_start_s, 0.0, committed_duration));
    RCLCPP_INFO(get_logger(),
                "planner_cycle_metrics cycles=%lu commands=%lu dropped_cloud=%lu "
                "planner_cycle_ms=%.3f p50_ms=%.3f p95_ms=%.3f p99_ms=%.3f "
                "input_us=%ld map_us=%ld "
                "planner_us=%ld publish_us=%ld input_lock_us=%ld target=(%.2f,%.2f,%.2f) "
                "robot=(%.2f,%.2f,%.2f) committed_start=(%.2f,%.2f,%.2f) "
                "committed_q1=(%.2f,%.2f,%.2f) committed_mid=(%.2f,%.2f,%.2f) "
                "committed_q3=(%.2f,%.2f,%.2f) committed_end=(%.2f,%.2f,%.2f) "
                "main_end=(%.2f,%.2f,%.2f) backup_start=%.3f "
                "guide_end=(%.2f,%.2f,%.2f) guide_z=[%.2f,%.2f] "
                "committed_duration=%.3f",
                static_cast<unsigned long>(cycle_count_),
                static_cast<unsigned long>(command_publish_count_.load()),
                static_cast<unsigned long>(accounting.replaced_waiting +
                                           accounting.replaced_ready), planner_cycle_ms,
                percentile(0.50), percentile(0.95), percentile(0.99),
                static_cast<long>(input_conversion_us),
                static_cast<long>(mapping.map_update_us), static_cast<long>(last_planner_us_),
                static_cast<long>(last_publish_us_.load()),
                static_cast<long>(last_input_lock_wait_us_), target.x(), target.y(), target.z(),
                execution_state.position_world.x(), execution_state.position_world.y(),
                execution_state.position_world.z(),
                committed_start.x(), committed_start.y(), committed_start.z(),
                committed_quarter.x(), committed_quarter.y(), committed_quarter.z(),
                committed_half.x(), committed_half.y(), committed_half.z(),
                committed_three_quarter.x(), committed_three_quarter.y(), committed_three_quarter.z(),
                committed_end.x(), committed_end.y(), committed_end.z(),
                main_end.x(), main_end.y(), main_end.z(), backup_start_s,
                guide_end.x(), guide_end.y(), guide_end.z(), guide_min.z(), guide_max.z(),
                committed_duration);
    metrics_log_time_ = metrics_now;
  }
}

void NavigationRuntimeNode::publishCommand() {
  const auto command_ros_time = now();
  const double now_seconds = command_ros_time.seconds();
  if (!std::isfinite(now_seconds)) return;
  const auto localization_epoch_at_command =
      active_localization_epoch_.load(std::memory_order_acquire);
  if (!localization_epoch_ready_.load(std::memory_order_acquire)) return;

  const std::uint64_t active_solve = active_planner_solve_generation_.load();
  if (active_solve != 0U) {
    const std::int64_t solve_age_ns =
        navigation_common::steadyClockNowNanoseconds() -
        planner_solve_started_steady_ns_.load();
    if (solve_age_ns > static_cast<std::int64_t>(planner_solve_timeout_s_ * 1e9)) {
      const std::uint64_t previous_timeout =
          timed_out_planner_solve_generation_.exchange(active_solve);
      if (previous_timeout != active_solve) {
        planner_->cancelActiveSolve();
        {
          std::lock_guard<std::mutex> command_lock(
              command_execution_lease_failure_latch_.transitionMutex());
          planner_command_available_.store(false);
          planner_failure_latched_.store(true);
          safety_suffix_active_.store(false);
        }
        RCLCPP_ERROR(get_logger(),
                     "planner backend planner watchdog timed out generation=%lu age=%.3f s stage=%d points=%zu; "
                     "invalidating committed main trajectory",
                     static_cast<unsigned long>(active_solve),
                     static_cast<double>(solve_age_ns) / 1e9,
                     planner_->solveStage(),
                     planner_->solvePointCount());
      }
    }
  }

  Eigen::Matrix<double, 3, 4> pvaj = Eigen::Matrix<double, 3, 4>::Zero();
  double yaw = 0.0;
  double yaw_dot = 0.0;
  bool on_backup_traj = false;
  bool traj_finish = false;
  std::uint64_t trajectory_generation = 0;
  double trajectory_time_s = 0.0;
  navigation_world_model::WorldSnapshotIdentity command_world_identity{};
  bool safety_suffix_active = safety_suffix_active_.load();
  bool planner_failed = planner_failure_latched_.load();
  bool sampled_command_valid = false;
  if (!planner_command_available_.load() && !planner_failed &&
      command_execution_lease_failure_latch_.allowsCommandExposure()) return;
  std::optional<navigation_contracts::msg::NavigationGoal> command_goal;
  std::uint64_t goal_epoch_at_command = 0U;
  std::shared_ptr<const navigation_execution::ExecutionStateLease> execution_state;
  std::int64_t execution_receive_steady_ns = 0;
  std::uint64_t execution_sequence = 0;
  execution_state = execution_state_store_.load();
  {
    std::lock_guard<std::mutex> lock(input_mutex_);
    command_goal = active_goal_;
    goal_epoch_at_command = active_goal_epoch_.load(std::memory_order_acquire);
  }
  execution_receive_steady_ns = execution_state ? execution_state->state.receive_stamp_ns : 0;
  execution_sequence = execution_state ? execution_state->ingress_sequence : 0;
  const auto execution_source_ns = execution_state ? execution_state->state.source_stamp_ns : 0;
  const auto execution_freshness = navigation_contracts::evaluateExecutionStateFreshness(
      command_ros_time.nanoseconds(), execution_source_ns,
      navigation_common::steadyClockNowNanoseconds(), execution_receive_steady_ns,
      input_max_age_s_);
  command_execution_lease_reason_.store(static_cast<int>(execution_freshness.reason));
  command_execution_source_age_us_.store(static_cast<std::int64_t>(
      execution_freshness.source_age_ms * 1000.0));
  command_execution_receive_age_us_.store(static_cast<std::int64_t>(
      execution_freshness.receive_age_ms * 1000.0));
  std::unique_lock<std::mutex> command_lock(
      command_execution_lease_failure_latch_.transitionMutex());
  if (!localization_epoch_ready_.load(std::memory_order_acquire) ||
      active_localization_epoch_.load(std::memory_order_acquire) !=
          localization_epoch_at_command) {
    planner_command_available_.store(false);
    planner_failure_latched_.store(true);
    safety_suffix_active_.store(false);
    command_goal_epoch_.store(0U);
    return;
  }
  const bool lease_already_failed =
      !command_execution_lease_failure_latch_.allowsCommandExposure();
  if (!execution_freshness.valid() || lease_already_failed) {
    ++command_execution_lease_rejection_count_;
    const bool first_failure = !execution_freshness.valid() &&
        command_execution_lease_failure_latch_.tryLatch();
    if (first_failure) {
      ++command_execution_lease_terminal_latch_count_;
    }
    // Clearing executable command state is unconditional. The latch owns
    // one-time cancellation/logging only; a solve that races past cancellation
    // must never resurrect a nominal command for this failed goal.
    planner_command_available_.store(false);
    planner_failure_latched_.store(true);
    safety_suffix_active_.store(false);
    planner_failed = true;
    safety_suffix_active = false;
    if (first_failure) {
      command_lock.unlock();
      planner_->cancelActiveSolve();
      RCLCPP_ERROR(
          get_logger(),
          "planner backend execution-state lease failed reason=%s source_age_ms=%.3f "
          "receive_age_ms=%.3f sequence=%lu active_solve=%lu; publishing terminal EMER",
          navigation_contracts::executionStateFreshnessReasonName(execution_freshness.reason),
          execution_freshness.source_age_ms, execution_freshness.receive_age_ms,
          static_cast<unsigned long>(execution_sequence),
          static_cast<unsigned long>(active_solve));
      command_lock.lock();
      planner_command_available_.store(false);
      planner_failure_latched_.store(true);
      safety_suffix_active_.store(false);
    }
  } else {
    // These three flags form one executable command decision. Reload them
    // only after acquiring the same serialization lock used by solve exposure.
    planner_failed = planner_failure_latched_.load();
    safety_suffix_active = safety_suffix_active_.load();
  }
  if (planner_command_available_.load() &&
      command_goal_epoch_.load() != active_goal_epoch_.load()) {
    return;
  }
  if (planner_command_available_.load()) {
    const auto sample = command_sampler_.sample(command_ros_time.nanoseconds());
    if (!sample) {
      planner_command_available_.store(false);
      planner_failure_latched_.store(true);
      safety_suffix_active_.store(false);
      planner_failed = true;
      safety_suffix_active = false;
      RCLCPP_ERROR_THROTTLE(
          get_logger(), *get_clock(), 1000,
          "execution boundary invalidated the committed command sample");
    } else {
      const auto& point = *sample.point;
      pvaj.col(0) = point.position_world;
      pvaj.col(1) = point.velocity_world;
      pvaj.col(2) = point.acceleration_world;
      pvaj.col(3) = point.jerk_world;
      yaw = point.yaw;
      yaw_dot = point.yaw_rate;
      trajectory_generation = sample.bundle->bundle_generation;
      trajectory_time_s = point.trajectory_time_s;
      command_world_identity = sample.bundle->world_identity;
      on_backup_traj = point.role != navigation_planning::CandidateRole::kMain;
      traj_finish = point.finished;
      sampled_command_valid = true;
    }
    // The safety suffix contains the dynamically continuous main prefix up to
    // planner backend's backup switch plus the braking polynomial. Once frozen by a
    // failed hot replan, the whole suffix is safety-owned.
    if (sampled_command_valid && safety_suffix_active) on_backup_traj = true;
    if (traj_finish) trajectory_finished_.store(true);
  } else {
    // PlanFromRest can fail before CmdTraj has ever been committed. Emit an
    // explicit terminal status so External Mode enters its PX4 Hold path;
    // never synthesize a zero-velocity nominal command from this state.
    pvaj.setZero();
  }

  if (command_world_identity.generation == 0U) {
    command_world_identity = world_snapshot_store_.load().identity;
  }
  navigation_contracts::msg::NavigationCommand command;
  command.header.frame_id = planning_frame_;
  command.header.stamp = rosTimeFromSeconds(now_seconds);
  command.localization_epoch = localization_epoch_at_command;
  command.goal_epoch = goal_epoch_at_command;
  if (command_goal) {
    command.mission_id = command_goal->mission_id;
    command.waypoint_index = command_goal->waypoint_index;
    command.request_id = command_goal->request_id;
  }
  command.world_generation = command_world_identity.generation;
  command.world_revision = command_world_identity.revision;
  command.world_observation_stamp = rosTimeFromSeconds(
      static_cast<double>(command_world_identity.observation_stamp_ns) * 1.0e-9);
  command.bundle_generation = trajectory_generation;
  command.sample_id = ++command_id_;
  command.trajectory_time_s = trajectory_time_s;
  command.state_source_stamp = execution_state
      ? navigation_common::nanosecondsToRosTime(execution_state->state.source_stamp_ns).value_or(
          builtin_interfaces::msg::Time{})
      : builtin_interfaces::msg::Time{};
  command.valid_until = rosTimeFromSeconds(now_seconds + input_max_age_s_);
  const bool main_trajectory_rejected = planner_failed && !on_backup_traj;
  command.status = main_trajectory_rejected
                       ? navigation_contracts::msg::NavigationCommand::STATUS_REJECTED
                       : traj_finish
                       ? navigation_contracts::msg::NavigationCommand::STATUS_COMPLETED
                       : navigation_contracts::msg::NavigationCommand::STATUS_READY;
  command.role = main_trajectory_rejected
                     ? navigation_contracts::msg::NavigationCommand::ROLE_EMERGENCY
                     : on_backup_traj
                     ? navigation_contracts::msg::NavigationCommand::ROLE_BACKUP
                     : navigation_contracts::msg::NavigationCommand::ROLE_MAIN;
  command.reason_code = main_trajectory_rejected ? 1U : 0U;
  command.position.x = pvaj(0, 0);
  command.position.y = pvaj(1, 0);
  command.position.z = pvaj(2, 0);
  command.velocity.x = pvaj(0, 1);
  command.velocity.y = pvaj(1, 1);
  command.velocity.z = pvaj(2, 1);
  command.acceleration.x = pvaj(0, 2);
  command.acceleration.y = pvaj(1, 2);
  command.acceleration.z = pvaj(2, 2);
  command.jerk.x = pvaj(0, 3);
  command.jerk.y = pvaj(1, 3);
  command.jerk.z = pvaj(2, 3);
  command.yaw = yaw;
  command.yaw_rate = yaw_dot;
  command_publisher_->publish(command);
  ++command_publish_count_;
  ++cycle_success_count_;
}

}  // namespace navigation_runtime
