#include "navigation_runtime/super_navigation_node.hpp"
#include "navigation_runtime/mission_dynamics.hpp"

#include "navigation_runtime/planner_fsm.hpp"
#include "navigation_runtime/timestamp_freshness.hpp"
#include "super_core/solve_stage.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstring>
#include <cstdlib>
#include <filesystem>
#include <limits>
#include <stdexcept>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

namespace navigation_runtime {
namespace {

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
    const navigation_interfaces::msg::NavigationGoal& goal) {
  // SUPER accepts one terminal target per solve.  next_target is directional
  // metadata for the mission controller; using it as the terminal target
  // would remove the current checkpoint from the geometric problem entirely.
  // PASS_THROUGH continuity is provided by accepting the current checkpoint
  // before its trajectory ends and hot-retargeting the committed PVA state.
  return goal.target;
}

std::int64_t steadyNowNanoseconds() {
  return std::chrono::duration_cast<std::chrono::nanoseconds>(
             std::chrono::steady_clock::now().time_since_epoch())
      .count();
}

class MappingDispositionGuard {
 public:
  explicit MappingDispositionGuard(ObservationAccounting* accounting)
      : accounting_(accounting) {}
  MappingDispositionGuard(const MappingDispositionGuard&) = delete;
  MappingDispositionGuard& operator=(const MappingDispositionGuard&) = delete;
  ~MappingDispositionGuard() {
    if (accounting_ != nullptr) accounting_->mappingFailed();
  }

  void publish() {
    if (accounting_ == nullptr) return;
    accounting_->mappingPublished();
    accounting_ = nullptr;
  }

 private:
  ObservationAccounting* accounting_;
};

}  // namespace

SuperNavigationNode::SuperNavigationNode(const rclcpp::NodeOptions& options)
    : rclcpp::Node("super_navigation_node", options) {
  cloud_topic_ = declare_parameter("super_navigation.cloud_topic", std::string("/lio/registered_points"));
  const auto legacy_odometry_topic = declare_parameter(
      "super_navigation.odometry_topic", std::string("/lio/odometry_propagated"));
  RCLCPP_WARN_ONCE(get_logger(),
                   "Parameter super_navigation.odometry_topic is deprecated; use "
                   "super_navigation.propagated_odometry_topic");
  propagated_odometry_topic_ = declare_parameter(
      "super_navigation.propagated_odometry_topic", legacy_odometry_topic);
  corrected_odometry_topic_ = declare_parameter(
      "super_navigation.corrected_odometry_topic", std::string("/lio/odometry_corrected"));
  goal_topic_ = declare_parameter("super_navigation.goal_topic", std::string("/navigation/goal"));
  status_topic_ = declare_parameter(
      "super_navigation.status_topic", std::string("/navigation/mode_status"));
  command_topic_ = declare_parameter(
      "super_navigation.command_topic", std::string("/navigation/super_command"));
  planning_frame_ = declare_parameter("super_navigation.planning_frame", std::string("lio_odom"));
  deployment_profile_ = declare_parameter(
      "super_navigation.deployment_profile", std::string("sitl"));
  hardware_visibility_certified_ = declare_parameter(
      "super_navigation.hardware_visibility_certified", false);
  planner_rate_hz_ = declare_parameter("super_navigation.planner_rate_hz", 10.0);
  command_rate_hz_ = declare_parameter("super_navigation.command_rate_hz", 50.0);
  input_pair_max_skew_s_ = declare_parameter("super_navigation.input_pair_max_skew_s", 0.1);
  input_max_age_s_ = declare_parameter("super_navigation.input_max_age_s", 0.5);
  max_safety_suffix_anchor_error_m_ = declare_parameter(
      "super_navigation.max_safety_suffix_anchor_error_m", 0.75);
  planner_solve_timeout_s_ = declare_parameter(
      "super_navigation.planner_solve_timeout_s", 1.0);
  plan_from_rest_failure_confirmation_s_ = declare_parameter(
      "super_navigation.plan_from_rest_failure_confirmation_s", 0.5);
  const auto max_plan_from_rest_failures =
      declare_parameter("super_navigation.max_plan_from_rest_failures", 3);
  super_config_path_ = declare_parameter("super_navigation.config_path", std::string{});
  const auto mission_file =
      declare_parameter("super_navigation.mission_file", std::string{});
  if (super_config_path_.empty()) {
    super_config_path_ = ament_index_cpp::get_package_share_directory("navigation_runtime") +
                         "/config/super_planner.yaml";
  }

  if (!std::filesystem::exists(super_config_path_)) {
    throw std::runtime_error("SUPER config does not exist: " + super_config_path_);
  }
  if (deployment_profile_ != "sitl" && deployment_profile_ != "hardware") {
    throw std::invalid_argument(
        "super_navigation.deployment_profile must be 'sitl' or 'hardware'");
  }
  if (deployment_profile_ == "hardware" && !hardware_visibility_certified_) {
    throw std::invalid_argument(
        "hardware SUPER runtime is blocked until the Mid-360 visibility/FOV "
        "certificate is explicitly enabled");
  }
  if (!std::isfinite(planner_rate_hz_) || planner_rate_hz_ <= 0.0) {
    throw std::invalid_argument("super_navigation.planner_rate_hz must be positive");
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
        "SUPER command/input pairing/safety anchor parameters must be positive");
  }
  if (max_plan_from_rest_failures <= 0) {
    throw std::invalid_argument(
        "super_navigation.max_plan_from_rest_failures must be positive");
  }
  max_plan_from_rest_failures_ =
      static_cast<std::uint32_t>(max_plan_from_rest_failures);
  plan_from_rest_failure_budget_ =
      ConsecutiveFailureBudget(max_plan_from_rest_failures_);

  ros_interface_ = std::make_shared<ros_interface::RosInterface>(
      [this]() { return now().seconds(); });
  std::optional<super_planner::DynamicLimits> mission_limits;
  if (!mission_file.empty()) {
    mission_limits = loadMissionDynamicLimits(mission_file);
    RCLCPP_INFO(
        get_logger(),
        "Applying mission dynamics to SUPER before optimizer construction: "
        "velocity=%.3f acceleration=%.3f jerk=%.3f",
        mission_limits->max_velocity_mps,
        mission_limits->max_acceleration_mps2,
        mission_limits->max_jerk_mps3);
  }
  map_ = std::make_shared<RuntimeRogMap>([this] { return now().seconds(); });
  map_->loadConfigAndInit(super_config_path_);
  auto initial_world_view = std::make_shared<RogWorldSnapshot>(
      map_->exportPlanningGrid(), navigation_world_model::WorldSnapshotIdentity{
                                      world_generation_, world_revision_, 0});
  world_snapshot_store_.publish(initial_world_view);
  world_model_view_ = world_snapshot_store_.load().view;
  planner_ = std::make_shared<super_planner::SuperPlanner>(
      super_config_path_, ros_interface_, world_model_view_, mission_limits);

  const auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).best_effort().durability_volatile();
  cloud_subscription_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      cloud_topic_, qos, std::bind(&SuperNavigationNode::onCloud, this, std::placeholders::_1));
  corrected_odometry_subscription_ = create_subscription<nav_msgs::msg::Odometry>(
      corrected_odometry_topic_, qos,
      std::bind(&SuperNavigationNode::onCorrectedOdometry, this, std::placeholders::_1));
  propagated_odometry_subscription_ = create_subscription<nav_msgs::msg::Odometry>(
      propagated_odometry_topic_, qos,
      std::bind(&SuperNavigationNode::onPropagatedOdometry, this, std::placeholders::_1));
  goal_subscription_ = create_subscription<navigation_interfaces::msg::NavigationGoal>(
      goal_topic_, rclcpp::QoS(rclcpp::KeepLast(1)).reliable(),
      std::bind(&SuperNavigationNode::onGoal, this, std::placeholders::_1));
  status_subscription_ =
      create_subscription<navigation_interfaces::msg::NavigationModeStatus>(
          status_topic_,
          rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local(),
          std::bind(&SuperNavigationNode::onModeStatus, this, std::placeholders::_1));
  command_publisher_ = create_publisher<mars_quadrotor_msgs::msg::PositionCommand>(
      command_topic_, rclcpp::QoS(rclcpp::KeepLast(1)).reliable());
  diagnostics_publisher_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
      "/navigation/diagnostics", rclcpp::QoS(rclcpp::KeepLast(5)).reliable());
  end_to_end_samples_ms_.reserve(256);

  // SUPER's planner state (last_exp_traj_, robot_state_ and CmdTraj) is not
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
      planning_period, std::bind(&SuperNavigationNode::runCycle, this), planning_callback_group_);
  command_timer_ = create_wall_timer(
      command_period, std::bind(&SuperNavigationNode::publishCommand, this),
      command_callback_group_);
  RCLCPP_INFO(get_logger(),
              "SUPER runtime ready: cloud=%s corrected_odom=%s propagated_odom=%s goal=%s "
              "output=%s planner=%.1fHz command=%.1fHz",
              cloud_topic_.c_str(), corrected_odometry_topic_.c_str(),
              propagated_odometry_topic_.c_str(), goal_topic_.c_str(),
              command_topic_.c_str(), planner_rate_hz_, command_rate_hz_);
}

bool SuperNavigationNode::decodeCloud(const sensor_msgs::msg::PointCloud2& message,
                                      rog_map::PointCloud& output) {
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

builtin_interfaces::msg::Time SuperNavigationNode::rosTimeFromSeconds(double seconds) {
  builtin_interfaces::msg::Time stamp;
  if (!std::isfinite(seconds) || seconds < 0.0) return stamp;
  const auto whole = static_cast<std::int64_t>(seconds);
  stamp.sec = static_cast<std::int32_t>(whole);
  stamp.nanosec = static_cast<std::uint32_t>((seconds - static_cast<double>(whole)) * 1e9);
  return stamp;
}

std::int64_t SuperNavigationNode::stampNanoseconds(
    const builtin_interfaces::msg::Time& stamp) {
  return input_pairing::stampNanoseconds(stamp);
}

void SuperNavigationNode::onCloud(const sensor_msgs::msg::PointCloud2::ConstSharedPtr& message) {
  ++received_cloud_count_;
  if (message->header.frame_id != planning_frame_) {
    observation_accounting_.recordRejectedBeforeInbox();
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                         "Dropping cloud in frame '%s'; SUPER input must be in '%s'",
                         message->header.frame_id.c_str(), planning_frame_.c_str());
    return;
  }
  const auto decode_started = std::chrono::steady_clock::now();
  auto decoded = std::make_shared<rog_map::PointCloud>();
  if (!decodeCloud(*message, *decoded)) {
    observation_accounting_.recordRejectedBeforeInbox();
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "Dropping malformed/empty PointCloud2");
    return;
  }
  const auto stamp_ns = stampNanoseconds(message->header.stamp);
  if (stamp_ns <= 0) {
    observation_accounting_.recordRejectedBeforeInbox();
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                         "Dropping cloud without a valid timestamp");
    return;
  }
  std::lock_guard<std::mutex> lock(input_mutex_);
  last_input_conversion_us_ = std::chrono::duration_cast<std::chrono::microseconds>(
      std::chrono::steady_clock::now() - decode_started).count();
  if (latest_cloud_) ++dropped_cloud_count_;
  observation_accounting_.recordAcceptedToInbox();
  latest_cloud_ = input_pairing::StampedObservation<
      std::shared_ptr<rog_map::PointCloud>>{std::move(decoded), stamp_ns};
  pending_cloud_received_steady_ns_ = steadyNowNanoseconds();
  ++accepted_cloud_count_;
}

void SuperNavigationNode::onCorrectedOdometry(
    const nav_msgs::msg::Odometry::ConstSharedPtr& message) {
  if (message->header.frame_id != planning_frame_) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                         "Dropping odometry frame '%s'; SUPER input must be in '%s'",
                         message->header.frame_id.c_str(), planning_frame_.c_str());
    return;
  }
  const auto stamp_ns = stampNanoseconds(message->header.stamp);
  if (stamp_ns <= 0) return;
  std::lock_guard<std::mutex> lock(input_mutex_);
  if (!corrected_odometry_history_.empty() &&
      stamp_ns < stampNanoseconds(corrected_odometry_history_.back().header.stamp)) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                         "Dropping out-of-order odometry timestamp");
    return;
  }
  corrected_odometry_history_.push_back(*message);
  while (corrected_odometry_history_.size() > 64U) corrected_odometry_history_.pop_front();
}

void SuperNavigationNode::onPropagatedOdometry(
    const nav_msgs::msg::Odometry::ConstSharedPtr& message) {
  if (message->header.frame_id != planning_frame_) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                         "Dropping propagated odometry frame '%s'; expected '%s'",
                         message->header.frame_id.c_str(), planning_frame_.c_str());
    return;
  }
  const auto stamp_ns = stampNanoseconds(message->header.stamp);
  if (stamp_ns <= 0) return;
  std::lock_guard<std::mutex> lock(input_mutex_);
  if (latest_propagated_odometry_ &&
      stamp_ns < stampNanoseconds(latest_propagated_odometry_->header.stamp)) {
    return;
  }
  latest_propagated_odometry_ = *message;
}

void SuperNavigationNode::onGoal(const navigation_interfaces::msg::NavigationGoal::ConstSharedPtr& message) {
  std::lock_guard<std::mutex> lock(input_mutex_);
  // A SUPER plan is owned by the mission waypoint identity.  The
  // continuation point is only look-ahead metadata; treating it as the
  // current goal makes a repeated waypoint publication look like a new
  // request (or, conversely, hides the actual waypoint transition).
  const bool same_checkpoint = active_goal_.has_value() &&
      active_goal_->mission_id == message->mission_id &&
      active_goal_->waypoint_index == message->waypoint_index;
  const bool same_logical_goal = same_checkpoint &&
      active_goal_->request_id == message->request_id;
  const bool reuse_completed_stop = same_checkpoint && !same_logical_goal &&
      message->behavior == navigation_interfaces::msg::NavigationGoal::BEHAVIOR_STOP &&
      planner_command_available_.load() && trajectory_reaches_goal_.load() &&
      !planner_failure_latched_.load() && !safety_suffix_active_.load();
  const bool can_hot_retarget = canHotRetargetAtWaypointTransition(
      same_logical_goal,
      active_goal_.has_value() &&
          active_goal_->behavior ==
              navigation_interfaces::msg::NavigationGoal::BEHAVIOR_PASS_THROUGH,
      planner_command_available_.load(), planner_failure_latched_.load(),
      safety_suffix_active_.load());
  if (!same_logical_goal) {
    // Cancel before exposing the new waypoint identity. SUPER's commit gate
    // guarantees that a solve for the previous waypoint cannot publish a new
    // CmdTraj after this callback has invalidated it. The already committed
    // atomic bundle remains available for smooth hot-retarget continuity.
    planner_->cancelActiveSolve();
  }
  active_goal_ = *message;
  if (reuse_completed_stop) {
    // The mission controller may republish a STOP checkpoint when one noisy
    // velocity sample temporarily breaks its hold confirmation. The completed
    // SUPER command already terminates at this same checkpoint and PX4 is
    // actively holding it; replacing it with a zero-distance PlanFromRest
    // creates a singular yaw problem and unnecessarily drops position hold.
    new_goal_ = false;
    hot_goal_transition_ = false;
    restart_from_rest_ = false;
    skip_replan_once_ = false;
    trajectory_finished_.store(true);
    return;
  }
  if (!same_logical_goal) {
    planner_failure_latched_.store(false);
    safety_suffix_active_.store(false);
    plan_from_rest_failure_budget_.reset();
    plan_from_rest_first_failure_steady_ns_ = 0;
    hot_goal_transition_ = can_hot_retarget;
    new_goal_ = !can_hot_retarget;
    if (!can_hot_retarget) {
      planner_command_available_.store(false);
    }
    restart_from_rest_ = false;
    skip_replan_once_ = false;
    trajectory_finished_.store(false);
    trajectory_reaches_goal_.store(false);
  }
}

void SuperNavigationNode::onModeStatus(
    const navigation_interfaces::msg::NavigationModeStatus::ConstSharedPtr& message) {
  if (message->state == navigation_interfaces::msg::NavigationModeStatus::ACTIVE ||
      message->state == navigation_interfaces::msg::NavigationModeStatus::BRAKING) {
    return;
  }
  std::lock_guard<std::mutex> lock(input_mutex_);
  if (!active_goal_ || active_goal_->mission_id != message->mission_id ||
      active_goal_->waypoint_index != message->waypoint_index ||
      active_goal_->request_id != message->request_id) {
    return;
  }
  RCLCPP_INFO(get_logger(),
              "Cancelling SUPER goal after terminal mission status state=%u reason=%u "
              "mission=%s waypoint=%u request=%lu",
              message->state, message->reason, message->mission_id.c_str(),
              message->waypoint_index, static_cast<unsigned long>(message->request_id));
  planner_->cancelActiveSolve();
  active_goal_.reset();
  new_goal_ = false;
  hot_goal_transition_ = false;
  restart_from_rest_ = false;
  skip_replan_once_ = false;
  planner_command_available_.store(false);
  planner_failure_latched_.store(false);
  safety_suffix_active_.store(false);
  plan_from_rest_failure_budget_.reset();
  plan_from_rest_first_failure_steady_ns_ = 0;
  trajectory_finished_.store(false);
  trajectory_reaches_goal_.store(false);
}

void SuperNavigationNode::runCycle() {
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
  std::optional<MappingObservation> mapping_observation;
  std::optional<nav_msgs::msg::Odometry> propagated_odometry;
  std::optional<navigation_interfaces::msg::NavigationGoal> goal;
  bool new_goal = false;
  bool hot_goal_transition = false;
  bool restart_from_rest = false;
  std::int64_t input_conversion_us = 0;
  std::uint64_t dropped_cloud_count = 0;
  const auto input_lock_started = std::chrono::steady_clock::now();
  {
    std::lock_guard<std::mutex> lock(input_mutex_);
    last_input_lock_wait_us_ = std::chrono::duration_cast<std::chrono::microseconds>(
        std::chrono::steady_clock::now() - input_lock_started).count();
    if (latest_cloud_) {
      auto pair = input_pairing::tryTakeExactPair(
          latest_cloud_, corrected_odometry_history_);
      if (pair) {
        mapping_observation = MappingObservation{
            std::move(pair->payload), std::move(pair->corrected_odometry), pair->stamp_ns};
        last_pair_wait_us_ = pending_cloud_received_steady_ns_ > 0
            ? std::max<std::int64_t>(
                  0, (steadyNowNanoseconds() - pending_cloud_received_steady_ns_) / 1000)
            : 0;
        pending_cloud_received_steady_ns_ = 0;
        observation_accounting_.mappingStarted();
      } else {
        const auto pending_stamp_ns = latest_cloud_->stamp_ns;
        const auto newest_corrected_stamp_ns = corrected_odometry_history_.empty()
            ? 0 : stampNanoseconds(corrected_odometry_history_.back().header.stamp);
        if (newest_corrected_stamp_ns > pending_stamp_ns) {
          ++corrected_pair_mismatch_count_;
          latest_cloud_.reset();
          pending_cloud_received_steady_ns_ = 0;
          observation_accounting_.discardedPending();
        } else if (classifyTimestampFreshness(
                       now().nanoseconds(), pending_stamp_ns,
                       static_cast<std::int64_t>(input_max_age_s_ * 1e9)) ==
                   TimestampFreshness::STALE) {
          ++stale_input_count_;
          ++stale_mapping_input_count_;
          latest_cloud_.reset();
          pending_cloud_received_steady_ns_ = 0;
          observation_accounting_.discardedPending();
        } else if (classifyTimestampFreshness(
                       now().nanoseconds(), pending_stamp_ns,
                       static_cast<std::int64_t>(input_max_age_s_ * 1e9)) ==
                   TimestampFreshness::FUTURE) {
          ++stale_input_count_;
          ++future_mapping_input_count_;
          latest_cloud_.reset();
          pending_cloud_received_steady_ns_ = 0;
          observation_accounting_.discardedPending();
        }
      }
    }
    propagated_odometry = latest_propagated_odometry_;
    goal = active_goal_;
    new_goal = new_goal_;
    hot_goal_transition = hot_goal_transition_;
    restart_from_rest = restart_from_rest_;
    input_conversion_us = last_input_conversion_us_;
    dropped_cloud_count = dropped_cloud_count_;
  }
  MappingDispositionGuard mapping_disposition{
      mapping_observation ? &observation_accounting_ : nullptr};
  if (!mapping_observation) return;
  const auto& cloud = mapping_observation->cloud;
  const auto cloud_stamp_ns = mapping_observation->stamp_ns;
  const auto& corrected_odometry = mapping_observation->corrected_odometry;
  const auto now_ns = now().nanoseconds();
  const auto corrected_stamp_ns = stampNanoseconds(corrected_odometry.header.stamp);
  const auto cloud_age_ns = now_ns - cloud_stamp_ns;
  const auto corrected_age_ns = now_ns - corrected_stamp_ns;
  const auto maximum_age_ns = static_cast<std::int64_t>(input_max_age_s_ * 1e9);
  const auto cloud_freshness =
      classifyTimestampFreshness(now_ns, cloud_stamp_ns, maximum_age_ns);
  const auto corrected_freshness =
      classifyTimestampFreshness(now_ns, corrected_stamp_ns, maximum_age_ns);
  const bool mapping_stale = cloud_freshness == TimestampFreshness::STALE ||
                             corrected_freshness == TimestampFreshness::STALE;
  const bool mapping_future = cloud_freshness == TimestampFreshness::FUTURE ||
                              corrected_freshness == TimestampFreshness::FUTURE;
  if (cloud_freshness == TimestampFreshness::INVALID ||
      corrected_freshness == TimestampFreshness::INVALID ||
      corrected_stamp_ns != cloud_stamp_ns ||
      mapping_stale || mapping_future) {
    ++stale_input_count_;
    if (mapping_stale) ++stale_mapping_input_count_;
    if (mapping_future) ++future_mapping_input_count_;
    RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "SUPER rejecting mapping observation cloud_age=%.3f corrected_age=%.3f "
        "limit=%.3f reason=%s",
        static_cast<double>(cloud_age_ns) * 1e-9,
        static_cast<double>(corrected_age_ns) * 1e-9,
        input_max_age_s_, mapping_future ? "future" : "stale_or_mismatch");
    return;
  }
  const auto& pose = corrected_odometry.pose.pose;
  const Eigen::Quaterniond corrected_q{
      pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z};
  if (!std::isfinite(pose.position.x) || !std::isfinite(pose.position.y) ||
      !std::isfinite(pose.position.z) || !corrected_q.coeffs().allFinite() ||
      corrected_q.norm() <= 1.0e-6) {
    ++invalid_corrected_pose_count_;
    return;
  }
  const auto normalized_corrected_q = corrected_q.normalized();
  const super_utils::Pose super_pose{
      super_utils::Vec3f{pose.position.x, pose.position.y, pose.position.z},
      super_utils::Quatf{normalized_corrected_q.w(), normalized_corrected_q.x(),
                         normalized_corrected_q.y(), normalized_corrected_q.z()}};
  const auto map_started = std::chrono::steady_clock::now();
  try {
    map_->updateMap(*cloud, super_pose);
  } catch (const std::exception& error) {
    ++map_update_exception_count_;
    RCLCPP_FATAL(get_logger(),
                 "ROG-Map update threw after mutation may have begun; terminating to prevent "
                 "publication of a partial world generation: %s",
                 error.what());
    throw;
  }
  const auto snapshot_started = std::chrono::steady_clock::now();
  try {
    auto next_world_view = std::make_shared<RogWorldSnapshot>(
        map_->exportPlanningGrid(), navigation_world_model::WorldSnapshotIdentity{
                                        world_generation_, world_revision_ + 1, cloud_stamp_ns});
    last_snapshot_export_us_ = std::chrono::duration_cast<std::chrono::microseconds>(
        std::chrono::steady_clock::now() - snapshot_started).count();
    last_snapshot_bytes_ = next_world_view->byteSize();
    last_snapshot_owned_bytes_ = next_world_view->ownedByteSize();
    last_snapshot_shared_metadata_bytes_ = next_world_view->sharedMetadataByteSize();
    world_snapshot_store_.publish(next_world_view);
    world_model_view_ = world_snapshot_store_.load().view;
    ++world_revision_;
  } catch (const std::exception& error) {
    ++map_update_exception_count_;
    RCLCPP_FATAL(get_logger(),
                 "World snapshot publication failed after mutable map update; terminating "
                 "instead of planning against an unpublishable world revision: %s",
                 error.what());
    throw;
  }
  last_map_update_us_ = std::chrono::duration_cast<std::chrono::microseconds>(
      std::chrono::steady_clock::now() - map_started).count();
  mapping_disposition.publish();

  diagnostic_msgs::msg::DiagnosticArray diagnostics;
  diagnostics.header.stamp = now();
  diagnostic_msgs::msg::DiagnosticStatus status;
  status.name = "super_navigation/super_planner";
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
  const auto& map_diagnostics = map_->lastDiagnostics();
  const auto accounting = observation_accounting_.snapshot();
  add_value("received_observation_count", received_cloud_count_);
  add_value("accepted_observation_count", accepted_cloud_count_);
  add_value("cycle_count", cycle_count_);
  add_value("trajectory_publish_count", cycle_success_count_);
  add_value("dropped_cloud_count", dropped_cloud_count);
  add_value("stale_input_count", stale_input_count_);
  add_value("stale_mapping_input_count", stale_mapping_input_count_);
  add_value("future_mapping_input_count", future_mapping_input_count_);
  add_value("stale_execution_state_count", stale_execution_state_count_);
  add_value("future_execution_state_count", future_execution_state_count_);
  add_value("invalid_corrected_pose_count", invalid_corrected_pose_count_);
  add_value("corrected_pair_mismatch_count", corrected_pair_mismatch_count_);
  add_value("invalid_execution_state_count", invalid_execution_state_count_);
  add_value("processing_exception_count", map_update_exception_count_);
  add_value("observation_rejected_before_inbox_count", accounting.rejected_before_inbox);
  add_value("observation_replaced_pending_count", accounting.replaced_pending);
  add_value("observation_discarded_pending_count", accounting.discarded_pending);
  add_value("mapping_started_count", accounting.mapping_started);
  add_value("mapping_published_count", accounting.mapping_published);
  add_value("mapping_failed_count", accounting.mapping_failed);
  add_value("mapping_pending_count", accounting.pending);
  add_value("mapping_in_flight_count", accounting.in_flight);
  add_value("observation_accounting_valid", accounting.allInvariantsHold() ? 1U : 0U);
  add_value("observation_accounting_violation_count", accounting.violation_count);
  add_value("mapping_input_point_count", map_diagnostics.endpoint_count);
  add_value("mapping_allocated_voxel_count", map_diagnostics.allocated_voxel_count);
  add_value("world_generation", world_generation_);
  add_value("world_revision", world_revision_);
  add_value("world_snapshot_bytes", last_snapshot_bytes_);
  add_value("world_snapshot_owned_bytes", last_snapshot_owned_bytes_);
  add_value("world_snapshot_shared_metadata_bytes", last_snapshot_shared_metadata_bytes_);
  add_value("world_snapshot_live_count", RogWorldSnapshot::liveCount());
  add_value("world_snapshot_peak_live_count", RogWorldSnapshot::peakLiveCount());
  add_value("world_snapshot_live_owned_bytes", RogWorldSnapshot::liveOwnedBytes());
  add_value("world_snapshot_peak_live_owned_bytes", RogWorldSnapshot::peakLiveOwnedBytes());
  add_duration("ros_pointcloud_decode_us", input_conversion_us);
  add_duration("observation_pair_wait_us", last_pair_wait_us_);
  add_duration("mapping_input_lock_wait_us", last_input_lock_wait_us_);
  add_duration("planning_scheduling_gap_us", last_planning_scheduling_gap_us_);
  add_duration("rog_raycast_us", map_diagnostics.rog_raycast_us);
  add_duration("rog_probability_update_us", map_diagnostics.rog_probability_update_us);
  add_duration("rog_inflation_us", map_diagnostics.rog_inflation_us);
  add_duration("rog_slide_us", map_diagnostics.rog_slide_us);
  add_duration("rog_total_update_us", map_diagnostics.rog_total_update_us);
  add_duration("world_snapshot_export_us", last_snapshot_export_us_);
  add_duration(
      "mapping_callback_total_us",
      std::chrono::duration_cast<std::chrono::microseconds>(
          std::chrono::steady_clock::now() - cycle_started).count());
  diagnostics.status.push_back(std::move(status));
  diagnostics_publisher_->publish(diagnostics);

  // Mapping ownership ends above. Planner execution state is independently
  // sourced from the latest propagated odometry and may be unavailable or
  // invalid without suppressing a corrected observation from ROG-Map.
  if (!propagated_odometry) return;
  const auto& execution_odometry = *propagated_odometry;
  const auto execution_stamp_ns = stampNanoseconds(execution_odometry.header.stamp);
  const auto execution_age_ns = now_ns - execution_stamp_ns;
  const auto execution_freshness =
      classifyTimestampFreshness(now_ns, execution_stamp_ns, maximum_age_ns);
  if (execution_freshness != TimestampFreshness::VALID) {
    ++stale_input_count_;
    if (execution_freshness == TimestampFreshness::STALE) ++stale_execution_state_count_;
    if (execution_freshness == TimestampFreshness::FUTURE) ++future_execution_state_count_;
    if (execution_freshness == TimestampFreshness::INVALID) ++invalid_execution_state_count_;
    return;
  }
  const Eigen::Quaterniond execution_q{
      execution_odometry.pose.pose.orientation.w,
      execution_odometry.pose.pose.orientation.x,
      execution_odometry.pose.pose.orientation.y,
      execution_odometry.pose.pose.orientation.z};
  const Eigen::Vector3d velocity_body{
      execution_odometry.twist.twist.linear.x,
      execution_odometry.twist.twist.linear.y,
      execution_odometry.twist.twist.linear.z};
  PlannerExecutionState execution_state;
  if (execution_q.coeffs().allFinite() && execution_q.norm() > 1.0e-6 &&
      velocity_body.allFinite()) {
    const Eigen::Quaterniond normalized = execution_q.normalized();
    const auto velocity_world = normalized * velocity_body;
    execution_state.position = super_utils::Vec3f{
        execution_odometry.pose.pose.position.x, execution_odometry.pose.pose.position.y,
        execution_odometry.pose.pose.position.z};
    execution_state.velocity = velocity_world;
    execution_state.orientation = super_utils::Quatf{
        normalized.w(), normalized.x(), normalized.y(), normalized.z()};
    const auto rotation = normalized.toRotationMatrix();
    execution_state.yaw = std::atan2(rotation(1, 0), rotation(0, 0));
    execution_state.stamp_s = static_cast<double>(execution_stamp_ns) * 1e-9;
    execution_state.valid = true;
  }
  if (!execution_state.finite() ||
      !planner_->setPlannerExecutionState(execution_state.toRobotState())) {
    ++invalid_execution_state_count_;
    return;
  }

  // SUPER may produce a successful local trajectory ending at the current
  // sensing frontier rather than at the mission goal. Once it finishes,
  // restart PlanFromRest for the same logical goal so newly observed map
  // cells can extend the route. Mapping has already published this cycle.
  const bool completed_trajectory = trajectory_finished_.exchange(false);
  if (completed_trajectory && goal && trajectory_reaches_goal_.load()) return;
  if (completed_trajectory && goal && !trajectory_reaches_goal_.load()) {
    const auto& target_message = plannerTarget(*goal);
    const double dx = pointFromMessage(target_message, 0) - execution_state.position.x();
    const double dy = pointFromMessage(target_message, 1) - execution_state.position.y();
    const double dz = pointFromMessage(target_message, 2) - execution_state.position.z();
    constexpr double kGoalReachedDistanceM = 0.20;
    if (std::sqrt(dx * dx + dy * dy + dz * dz) > kGoalReachedDistanceM) {
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
                  "SUPER local trajectory finished before goal; restarting PlanFromRest "
                  "goal=(%.2f,%.2f,%.2f) vehicle=(%.2f,%.2f,%.2f)",
                  pointFromMessage(target_message, 0), pointFromMessage(target_message, 1),
                  pointFromMessage(target_message, 2), execution_state.position.x(),
                  execution_state.position.y(), execution_state.position.z());
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
  rog_map::RobotState planner_state;
  planner_->getRobotState(planner_state);

  // Always retain the current mission checkpoint as SUPER's geometric target.
  // PASS_THROUGH is implemented by hot-retargeting while the committed
  // trajectory still carries non-zero PVA, not by skipping to next_target.
  const auto& planner_target = plannerTarget(*goal);
  const super_utils::Vec3f target{
      static_cast<float>(pointFromMessage(planner_target, 0)),
      static_cast<float>(pointFromMessage(planner_target, 1)),
      static_cast<float>(pointFromMessage(planner_target, 2))};
  const auto planner_started = std::chrono::steady_clock::now();
  // This is the internal SUPER FSM boundary: each mission waypoint enters
  // PlanFromRest once, then every subsequent planning tick is ReplanOnce.
  // SUPER itself owns the committed main/backup timing; the ROS adapter must
  // not add a second horizon-expiry or trajectory-slicing policy.
  const bool plan_from_rest = new_goal || restart_from_rest;
  const bool replan_for_new_goal = hot_goal_transition && !plan_from_rest;
  if (new_goal) {
    // MissionController has invalidated the previous waypoint already. Do
    // not publish that waypoint while PlanFromRest runs.
    planner_command_available_.store(false);
    planner_failure_latched_.store(false);
    trajectory_reaches_goal_.store(false);
  }
  if (!plan_from_rest && !replan_for_new_goal && skip_replan_once_) {
    skip_replan_once_ = false;
    return;
  }
  super_utils::RET_CODE result = super_utils::FAILED;
  const std::uint64_t solve_generation = ++planner_solve_generation_;
  const auto pinned_world = world_snapshot_store_.load();
  if (!pinned_world) {
    RCLCPP_ERROR(get_logger(), "SUPER cannot solve without a published WorldModel snapshot");
    return;
  }
  planner_->setWorldModelView(pinned_world.view);
  planner_solve_started_steady_ns_.store(steadyNowNanoseconds());
  active_planner_solve_generation_.store(solve_generation);
  planner_->resetSolveCancellation();
  try {
    result = plan_from_rest
                 ? planner_->PlanFromRest(target, 0.0, true)
                 : planner_->ReplanOnce(target, 0.0, replan_for_new_goal);
  } catch (const std::exception& error) {
    RCLCPP_ERROR(get_logger(), "SUPER planner exception: %s", error.what());
    result = super_utils::EMER;
  }
  std::uint64_t expected_active_generation = solve_generation;
  active_planner_solve_generation_.compare_exchange_strong(
      expected_active_generation, 0U);
  last_planner_us_ = std::chrono::duration_cast<std::chrono::microseconds>(
      std::chrono::steady_clock::now() - planner_started).count();
  if (timed_out_planner_solve_generation_.load() == solve_generation) {
    RCLCPP_ERROR(get_logger(),
                 "Discarding SUPER solve generation=%lu after planner watchdog timeout",
                 static_cast<unsigned long>(solve_generation));
    return;
  }
  std::vector<double> module_times;
  planner_->getModuleTimeConsuming(module_times);
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
  const auto disposition = classifyPlannerResult(
      result, plan_from_rest, planner_command_available_.load());
  if (disposition == PlannerResultDisposition::FailClosed) {
    planner_failure_latched_.store(true);
    trajectory_reaches_goal_.store(false);
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "SUPER planning failed (%d)", result);
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
                "SUPER local trajectory boundary reached; scheduling PlanFromRest");
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
        const auto failure_now_ns = steadyNowNanoseconds();
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
                   "SUPER PlanFromRest failed %u consecutive times; fail-closed for "
                   "mission=%s waypoint=%u",
                   failure_count, goal->mission_id.c_str(), goal->waypoint_index);
    } else {
      RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "SUPER PlanFromRest transient failure (%d); retry %u/%u", result,
          failure_count, max_plan_from_rest_failures_);
    }
  }
  if (disposition == PlannerResultDisposition::RetainCommittedCommand) {
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
                                    ? super_utils::Vec3f{}
                                    : committed.getPos(clamped_elapsed_s);
    // ReplanOnce may run for more than a second while command publication and
    // vehicle motion continue concurrently. The planner_state captured before
    // that solve is therefore stale by construction. Validate against the
    // freshest propagated odometry available after the solve.
    super_utils::Vec3f current_vehicle_position = planner_state.p;
    super_utils::Vec3f current_vehicle_velocity = planner_state.v;
    double current_vehicle_yaw = planner_state.yaw;
    bool fresh_vehicle_state = false;
    double latest_vehicle_state_age_s = std::numeric_limits<double>::infinity();
    latest_vehicle_state_age_s = now().seconds() - execution_state.stamp_s;
    fresh_vehicle_state = execution_state.valid && execution_state.finite() &&
                          std::abs(latest_vehicle_state_age_s) <= input_max_age_s_;
    current_vehicle_position = execution_state.position;
    current_vehicle_velocity = execution_state.velocity;
    current_vehicle_yaw = execution_state.yaw;
    const double anchor_error_m = committed.empty() || !fresh_vehicle_state
                                      ? std::numeric_limits<double>::infinity()
                                      : (command_anchor - current_vehicle_position).norm();
    bool sampled_path_clear = !committed.empty();
    double first_blocked_sample_s = std::numeric_limits<double>::quiet_NaN();
    super_utils::Vec3f first_blocked_sample = super_utils::Vec3f::Constant(
        std::numeric_limits<double>::quiet_NaN());
    rog_map::GridType first_blocked_grid = rog_map::GridType::UNKNOWN;
    if (sampled_path_clear) {
      const double spatial_step_m = std::max(0.02, 0.5 * map_->getInfResolution());
      super_utils::Vec3f previous_sample = committed.getPos(clamped_elapsed_s);
      for (double sample_t = clamped_elapsed_s; sample_t <= total_duration_s;) {
        const auto sample = committed.getPos(sample_t);
        const auto grid = sample.allFinite()
                              ? map_->getInfGridType(sample)
                              : rog_map::GridType::OUT_OF_MAP;
        if (grid == rog_map::GridType::OCCUPIED ||
            grid == rog_map::GridType::OUT_OF_MAP ||
            !map_->isLineFree(previous_sample, sample, true, false)) {
          sampled_path_clear = false;
          first_blocked_sample_s = sample_t;
          first_blocked_sample = sample;
          first_blocked_grid = grid;
          break;
        }
        previous_sample = sample;
        const double speed_mps = std::max(0.1, committed.getVel(sample_t).norm());
        const double adaptive_dt = std::clamp(spatial_step_m / speed_mps, 0.002, 0.05);
        if (sample_t >= total_duration_s) break;
        sample_t = std::min(total_duration_s, sample_t + adaptive_dt);
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
    if (!use_safety_suffix && fresh_vehicle_state && !committed.empty() &&
        !committed_yaw.empty()) {
      // Position and velocity are measured at the newest propagated odometry
      // sample. Acceleration/jerk and yaw-rate are not measured by the LIO
      // interface, so retain their continuous values from the command that
      // PX4 was tracking at the same command-clock instant.
      super_utils::StatePVAJ emergency_state = committed.getState(clamped_elapsed_s);
      emergency_state.col(0) = current_vehicle_position;
      emergency_state.col(1) = current_vehicle_velocity;
      const double emergency_yaw_dot = committed_yaw.getVel(clamped_elapsed_s).x();
      emergency_brake_committed = planner_->commitEmergencyBrake(
          emergency_state, current_vehicle_yaw, emergency_yaw_dot, now().seconds());
      use_safety_suffix = emergency_brake_committed;
    }
    // A visible main-only trajectory remains a MAIN command. Only an actual
    // atomic main-to-backup bundle is marked safety-owned at the PX4 boundary.
    safety_suffix_active_.store(
        use_safety_suffix && (backup_available || emergency_brake_committed));
    planner_failure_latched_.store(!use_safety_suffix);
    if (emergency_brake_committed) {
      planner_command_available_.store(true);
      trajectory_finished_.store(false);
      RCLCPP_WARN(get_logger(),
                  "SUPER replaced unusable committed suffix with a measured-state "
                  "emergency backup trajectory");
    }
    if (use_safety_suffix) {
      RCLCPP_WARN(get_logger(),
                  "SUPER hot replan failed (%d); retaining visible committed trajectory "
                  "backup=%d elapsed=%.3f backup_start=%.3f end=%.3f anchor_error=%.3f",
                  result, backup_available, elapsed_s, safety_transition_s,
                  total_duration_s, anchor_error_m);
    } else {
      planner_command_available_.store(false);
      RCLCPP_ERROR(get_logger(),
                   "SUPER hot replan failed without a valid safety suffix: backup=%d "
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
    planner_command_available_.store(true);
    planner_failure_latched_.store(false);
    safety_suffix_active_.store(false);
    trajectory_finished_.store(false);
    planner_->lockCommittedTraj();
    const auto committed = planner_->getCommittedPositionTrajectory();
    planner_->unlockCommittedTraj();
    const auto committed_end = committed.empty()
                                   ? super_utils::Vec3f{}
                                   : committed.getPos(committed.getTotalDuration());
    trajectory_reaches_goal_.store(
        !committed.empty() && (committed_end - target).norm() <= 0.20);
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
    planner_->lockCommittedTraj();
    const auto committed = planner_->getCommittedPositionTrajectory();
    planner_->unlockCommittedTraj();
    const auto committed_end = committed.empty()
                                   ? super_utils::Vec3f{}
                                   : committed.getPos(committed.getTotalDuration());
    const double endpoint_error = committed.empty()
                                      ? std::numeric_limits<double>::infinity()
                                      : (committed_end - target).norm();
    const auto robot_grid_type = map_->getGridType(planner_state.p);
    const auto robot_inflated_grid_type = map_->getInfGridType(planner_state.p);
    super_utils::Vec3f nearest_known_free;
    const bool nearest_known_free_found = map_->getNearestCellIs(
        rog_map::GridType::KNOWN_FREE, planner_state.p, nearest_known_free, 1.0);
    const double nearest_known_free_distance = nearest_known_free_found
                                                   ? (nearest_known_free - planner_state.p).norm()
                                                   : std::numeric_limits<double>::infinity();
    super_utils::Vec3f nearest_occupied = super_utils::Vec3f::Constant(
        std::numeric_limits<double>::quiet_NaN());
    const bool nearest_occupied_found = map_->getNearestCellIs(
        rog_map::GridType::OCCUPIED, planner_state.p, nearest_occupied, 1.5);
    const double nearest_occupied_distance = nearest_occupied_found
                                                 ? (nearest_occupied - planner_state.p).norm()
                                                 : std::numeric_limits<double>::infinity();
    const auto committed_generation = planner_->getCommittedGeneration();
    const int solve_stage = planner_->solveStage();
    const int replan_return_code = planner_->latestReplanReturnCode();
    const double planner_elapsed_ms = static_cast<double>(last_planner_us_) * 1.0e-3;
    const bool solve_deadline_exceeded =
        planner_elapsed_ms > planner_->solveDeadlineSeconds() * 1000.0;
    RCLCPP_INFO(get_logger(),
                "SUPER decision_trace cycle=%lu solve_generation=%lu committed_generation=%lu "
                "pinned_world_generation=%lu pinned_world_revision=%lu "
                "pinned_world_stamp_ns=%ld "
                "cloud_stamp_ns=%ld corrected_stamp_ns=%ld propagated_stamp_ns=%ld "
                "state_age_ms=%.3f mode=%s result=%d replan_code=%d "
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
                static_cast<long>(cloud_stamp_ns), static_cast<long>(corrected_stamp_ns),
                static_cast<long>(execution_stamp_ns),
                static_cast<double>(execution_age_ns) * 1e-6,
                plan_from_rest ? "PlanFromRest" : "ReplanOnce", result,
                replan_return_code, solve_stage,
                super_planner::solveStageName(solve_stage).data(), planner_elapsed_ms,
                solve_deadline_exceeded, target.x(), target.y(),
                target.z(), committed_end.x(), committed_end.y(), committed_end.z(),
                endpoint_error, planner_command_available_.load(), planner_failure_latched_.load(),
                module_times.size() > super_planner::EPX_TRAJ_FRONTEND
                    ? module_times[super_planner::EPX_TRAJ_FRONTEND] * 1000.0 : 0.0,
                module_times.size() > super_planner::EXP_TRAJ_OPT
                    ? module_times[super_planner::EXP_TRAJ_OPT] * 1000.0 : 0.0,
                module_times.size() > super_planner::BACK_TRAJ_FRONTEND
                    ? module_times[super_planner::BACK_TRAJ_FRONTEND] * 1000.0 : 0.0,
                module_times.size() > super_planner::BACK_TRAJ_OPT
                    ? module_times[super_planner::BACK_TRAJ_OPT] * 1000.0 : 0.0,
                static_cast<int>(robot_grid_type), static_cast<int>(robot_inflated_grid_type),
                nearest_known_free_distance, nearest_occupied_distance,
                nearest_occupied.x(), nearest_occupied.y(), nearest_occupied.z());

    diagnostic_msgs::msg::DiagnosticArray trace_diagnostics;
    trace_diagnostics.header.stamp = now();
    diagnostic_msgs::msg::DiagnosticStatus trace_status;
    trace_status.name = "super_navigation/super_planner";
    trace_status.level = result == super_utils::SUCCESS
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
    add_trace_value("planning_cycle_id", cycle_count_);
    add_trace_value("bundle_id", committed_generation);
    add_trace_value("solve_generation", solve_generation);
    add_trace_value("pinned_world_generation", pinned_world.identity.generation);
    add_trace_value("pinned_world_revision", pinned_world.identity.revision);
    add_trace_value("pinned_world_stamp_ns", pinned_world.identity.observation_stamp_ns);
    add_trace_value("candidate_result", static_cast<int>(result));
    add_trace_value("replan_code", replan_return_code);
    add_trace_value("solve_stage", solve_stage);
    {
      diagnostic_msgs::msg::KeyValue item;
      item.key = "solve_stage_name";
      item.value = std::string(super_planner::solveStageName(solve_stage));
      trace_status.values.push_back(std::move(item));
    }
    add_trace_value("planning_latency_ms", planner_elapsed_ms);
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
    const bool backup_available = planner_->committedBackupTrajectoryAvailable();
    const double backup_start_s = planner_->getCommittedBackupStartTrajectoryTime();
    planner_->unlockCommittedTraj();
    const auto guide_end = planner_->latestGuideEnd();
    const auto guide_min = planner_->latestGuideMin();
    const auto guide_max = planner_->latestGuideMax();
    const auto committed_start = committed.empty() ? super_utils::Vec3f{} : committed.getPos(0.0);
    const auto committed_end = committed.empty()
                                   ? super_utils::Vec3f{}
                                   : committed.getPos(committed.getTotalDuration());
    const double committed_duration = committed.getTotalDuration();
    const auto committed_quarter = committed.empty()
                                       ? super_utils::Vec3f{}
                                       : committed.getPos(committed_duration * 0.25);
    const auto committed_half = committed.empty()
                                    ? super_utils::Vec3f{}
                                    : committed.getPos(committed_duration * 0.50);
    const auto committed_three_quarter = committed.empty()
                                             ? super_utils::Vec3f{}
                                             : committed.getPos(committed_duration * 0.75);
    const auto main_end = committed.empty() || !backup_available
                              ? committed_end
                              : committed.getPos(std::clamp(
                                    backup_start_s, 0.0, committed_duration));
    RCLCPP_INFO(get_logger(),
                "super_cycle_metrics cycles=%lu commands=%lu dropped_cloud=%lu "
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
                static_cast<unsigned long>(dropped_cloud_count), planner_cycle_ms,
                percentile(0.50), percentile(0.95), percentile(0.99),
                static_cast<long>(input_conversion_us),
                static_cast<long>(last_map_update_us_), static_cast<long>(last_planner_us_),
                static_cast<long>(last_publish_us_.load()),
                static_cast<long>(last_input_lock_wait_us_), target.x(), target.y(), target.z(),
                planner_state.p.x(), planner_state.p.y(), planner_state.p.z(),
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

void SuperNavigationNode::publishCommand() {
  const double now_seconds = now().seconds();
  if (!std::isfinite(now_seconds)) return;

  const std::uint64_t active_solve = active_planner_solve_generation_.load();
  if (active_solve != 0U) {
    const std::int64_t solve_age_ns =
        steadyNowNanoseconds() - planner_solve_started_steady_ns_.load();
    if (solve_age_ns > static_cast<std::int64_t>(planner_solve_timeout_s_ * 1e9)) {
      const std::uint64_t previous_timeout =
          timed_out_planner_solve_generation_.exchange(active_solve);
      if (previous_timeout != active_solve) {
        planner_->cancelActiveSolve();
        planner_command_available_.store(false);
        planner_failure_latched_.store(true);
        safety_suffix_active_.store(false);
        RCLCPP_ERROR(get_logger(),
                     "SUPER planner watchdog timed out generation=%lu age=%.3f s stage=%d points=%zu; "
                     "invalidating committed main trajectory",
                     static_cast<unsigned long>(active_solve),
                     static_cast<double>(solve_age_ns) / 1e9,
                     planner_->solveStage(),
                     planner_->solvePointCount());
      }
    }
  }

  super_utils::StatePVAJ pvaj;
  double yaw = 0.0;
  double yaw_dot = 0.0;
  bool on_backup_traj = false;
  bool traj_finish = false;
  const bool safety_suffix_active = safety_suffix_active_.load();
  const bool planner_failed = planner_failure_latched_.load();
  if (!planner_command_available_.load() && !planner_failed) return;
  if (planner_command_available_.load()) {
    const auto sample = planner_->sampleCommand();
    if (!sample.valid) {
      planner_command_available_.store(false);
      planner_failure_latched_.store(true);
      safety_suffix_active_.store(false);
      RCLCPP_ERROR_THROTTLE(
          get_logger(), *get_clock(), 1000,
          "SUPER invalidated non-finite committed sample generation=%lu",
          static_cast<unsigned long>(sample.generation));
      pvaj.setZero();
    } else {
      pvaj = sample.pvaj;
      yaw = sample.yaw;
      yaw_dot = sample.yaw_rate;
      on_backup_traj = sample.role == super_planner::SuperPlanner::TrajectoryRole::BACKUP;
      traj_finish = sample.finished;
    }
    // The safety suffix contains the dynamically continuous main prefix up to
    // SUPER's backup switch plus the braking polynomial. Once frozen by a
    // failed hot replan, the whole suffix is safety-owned.
    if (sample.valid && safety_suffix_active) on_backup_traj = true;
    if (traj_finish) trajectory_finished_.store(true);
  } else {
    // PlanFromRest can fail before CmdTraj has ever been committed. Emit an
    // explicit terminal status so External Mode enters its PX4 Hold path;
    // never synthesize a zero-velocity nominal command from this state.
    pvaj.setZero();
  }

  mars_quadrotor_msgs::msg::PositionCommand command;
  command.header.frame_id = planning_frame_;
  command.header.stamp = rosTimeFromSeconds(now_seconds);
  command.trajectory_id = ++command_id_;
  const bool main_trajectory_rejected = planner_failed && !on_backup_traj;
  command.trajectory_status = main_trajectory_rejected
                                  ? mars_quadrotor_msgs::msg::PositionCommand::TRAJECTORY_STATUS_EMER
                                  : traj_finish
                                  ? mars_quadrotor_msgs::msg::PositionCommand::TRAJECTORY_STATUS_COMPLETED
                                  : mars_quadrotor_msgs::msg::PositionCommand::TRAJECTORY_STATUS_READY;
  command.trajectory_flag = on_backup_traj
                                ? mars_quadrotor_msgs::msg::PositionCommand::TRAJECTORY_FLAG_BACKUP
                                : mars_quadrotor_msgs::msg::PositionCommand::TRAJECTORY_FLAG_MAIN;
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
  command.yaw_dot = yaw_dot;
  command.vel_norm = std::sqrt(
      command.velocity.x * command.velocity.x + command.velocity.y * command.velocity.y +
      command.velocity.z * command.velocity.z);
  command.acc_norm = std::sqrt(
      command.acceleration.x * command.acceleration.x +
      command.acceleration.y * command.acceleration.y +
      command.acceleration.z * command.acceleration.z);
  command_publisher_->publish(command);
  ++command_publish_count_;
  ++cycle_success_count_;
}

}  // namespace navigation_runtime
