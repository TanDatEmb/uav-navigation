#include "navigation_runtime/super_navigation_node.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstring>
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

}  // namespace

SuperNavigationNode::SuperNavigationNode(const rclcpp::NodeOptions& options)
    : rclcpp::Node("super_navigation_node", options) {
  cloud_topic_ = declare_parameter("super_navigation.cloud_topic", std::string("/lio/registered_points"));
  odometry_topic_ = declare_parameter(
      "super_navigation.odometry_topic", std::string("/lio/odometry_propagated"));
  goal_topic_ = declare_parameter("super_navigation.goal_topic", std::string("/navigation/goal"));
  trajectory_topic_ = declare_parameter(
      "super_navigation.trajectory_topic", std::string("/navigation/super_trajectory"));
  planning_frame_ = declare_parameter("super_navigation.planning_frame", std::string("lio_odom"));
  planner_rate_hz_ = declare_parameter("super_navigation.planner_rate_hz", 10.0);
  heartbeat_rate_hz_ = declare_parameter("super_navigation.heartbeat_rate_hz", 50.0);
  heartbeat_horizon_s_ = declare_parameter("super_navigation.heartbeat_horizon_s", 3.0);
  super_config_path_ = declare_parameter("super_navigation.config_path", std::string{});
  if (super_config_path_.empty()) {
    super_config_path_ = ament_index_cpp::get_package_share_directory("navigation_runtime") +
                         "/config/super_planner.yaml";
  }

  if (!std::filesystem::exists(super_config_path_)) {
    throw std::runtime_error("SUPER config does not exist: " + super_config_path_);
  }
  if (!std::isfinite(planner_rate_hz_) || planner_rate_hz_ <= 0.0) {
    throw std::invalid_argument("super_navigation.planner_rate_hz must be positive");
  }
  if (!std::isfinite(heartbeat_rate_hz_) || heartbeat_rate_hz_ <= 0.0 ||
      !std::isfinite(heartbeat_horizon_s_) || heartbeat_horizon_s_ <= 0.0) {
    throw std::invalid_argument(
        "super_navigation heartbeat_rate_hz and heartbeat_horizon_s must be positive");
  }

  ros_interface_ = std::make_shared<ros_interface::RosInterface>();
  map_ = std::make_shared<rog_map::ROGMapROS>([this] { return now().seconds(); });
  map_->loadConfigAndInit(super_config_path_);
  planner_ = std::make_shared<super_planner::SuperPlanner>(super_config_path_, ros_interface_, map_);

  const auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).best_effort().durability_volatile();
  cloud_subscription_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      cloud_topic_, qos, std::bind(&SuperNavigationNode::onCloud, this, std::placeholders::_1));
  odometry_subscription_ = create_subscription<nav_msgs::msg::Odometry>(
      odometry_topic_, qos, std::bind(&SuperNavigationNode::onOdometry, this, std::placeholders::_1));
  goal_subscription_ = create_subscription<navigation_interfaces::msg::NavigationGoal>(
      goal_topic_, rclcpp::QoS(rclcpp::KeepLast(1)).reliable(),
      std::bind(&SuperNavigationNode::onGoal, this, std::placeholders::_1));
  trajectory_publisher_ = create_publisher<mars_quadrotor_msgs::msg::PolynomialTrajectory>(
      trajectory_topic_, rclcpp::QoS(rclcpp::KeepLast(1)).reliable());
  diagnostics_publisher_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
      "/navigation/diagnostics", rclcpp::QoS(rclcpp::KeepLast(5)).reliable());
  end_to_end_samples_ms_.reserve(256);

  // SUPER's planner state (last_exp_traj_, robot_state_ and CmdTraj) is not
  // re-entrant.  A 300 ms optimization must never overlap the next timer
  // tick; only the read-only heartbeat is allowed to run concurrently.
  planning_callback_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  heartbeat_callback_group_ = create_callback_group(rclcpp::CallbackGroupType::Reentrant);
  const auto planning_period = std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(1.0 / planner_rate_hz_));
  const auto heartbeat_period = std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(1.0 / heartbeat_rate_hz_));
  planning_timer_ = create_wall_timer(
      planning_period, std::bind(&SuperNavigationNode::runCycle, this), planning_callback_group_);
  heartbeat_timer_ = create_wall_timer(
      heartbeat_period, std::bind(&SuperNavigationNode::publishHeartbeat, this),
      heartbeat_callback_group_);
  RCLCPP_INFO(get_logger(),
              "SUPER runtime ready: cloud=%s odometry=%s goal=%s output=%s planner=%.1fHz heartbeat=%.1fHz",
              cloud_topic_.c_str(), odometry_topic_.c_str(), goal_topic_.c_str(),
              trajectory_topic_.c_str(), planner_rate_hz_, heartbeat_rate_hz_);
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

void SuperNavigationNode::onCloud(const sensor_msgs::msg::PointCloud2::ConstSharedPtr& message) {
  ++received_cloud_count_;
  if (message->header.frame_id != planning_frame_) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                         "Dropping cloud in frame '%s'; SUPER input must be in '%s'",
                         message->header.frame_id.c_str(), planning_frame_.c_str());
    return;
  }
  const auto decode_started = std::chrono::steady_clock::now();
  auto decoded = std::make_shared<rog_map::PointCloud>();
  if (!decodeCloud(*message, *decoded)) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "Dropping malformed/empty PointCloud2");
    return;
  }
  std::lock_guard<std::mutex> lock(input_mutex_);
  last_input_conversion_us_ = std::chrono::duration_cast<std::chrono::microseconds>(
      std::chrono::steady_clock::now() - decode_started).count();
  if (latest_cloud_) ++dropped_cloud_count_;
  latest_cloud_ = std::move(decoded);
  ++accepted_cloud_count_;
}

void SuperNavigationNode::onOdometry(const nav_msgs::msg::Odometry::ConstSharedPtr& message) {
  if (message->header.frame_id != planning_frame_) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                         "Dropping odometry frame '%s'; SUPER input must be in '%s'",
                         message->header.frame_id.c_str(), planning_frame_.c_str());
    return;
  }
  std::lock_guard<std::mutex> lock(input_mutex_);
  latest_odometry_ = *message;
}

void SuperNavigationNode::onGoal(const navigation_interfaces::msg::NavigationGoal::ConstSharedPtr& message) {
  std::lock_guard<std::mutex> lock(input_mutex_);
  // A SUPER plan is owned by the mission waypoint identity.  The
  // continuation point is only look-ahead metadata; treating it as the
  // current goal makes a repeated waypoint publication look like a new
  // request (or, conversely, hides the actual waypoint transition).
  const bool same_logical_goal = active_goal_.has_value() &&
      active_goal_->mission_id == message->mission_id &&
      active_goal_->waypoint_index == message->waypoint_index;
  active_goal_ = *message;
  if (!same_logical_goal) new_goal_ = true;
}

mars_quadrotor_msgs::msg::PolynomialTrajectory SuperNavigationNode::serializeTrajectory(
    const geometry_utils::Trajectory& position,
    const geometry_utils::Trajectory& yaw,
    double now_seconds,
    bool append_terminal_hold) {
  geometry_utils::Trajectory position_to_publish = position;
  geometry_utils::Trajectory yaw_to_publish = yaw;
  if (append_terminal_hold && !position_to_publish.empty()) {
    constexpr double kTerminalHoldDurationS = 2.0;
    const auto& last_position_piece =
        position_to_publish[position_to_publish.getPieceNum() - 1];
    const int position_degree = last_position_piece.getDegree();
    Eigen::MatrixXd stationary_position =
        Eigen::MatrixXd::Zero(3, static_cast<Eigen::Index>(position_degree) + 1);
    stationary_position.col(position_degree) =
        last_position_piece.getPos(last_position_piece.getDuration());
    position_to_publish.emplace_back(kTerminalHoldDurationS, stationary_position);

    if (!yaw_to_publish.empty()) {
      const auto& last_yaw_piece = yaw_to_publish[yaw_to_publish.getPieceNum() - 1];
      const int yaw_degree = last_yaw_piece.getDegree();
      Eigen::MatrixXd stationary_yaw =
          Eigen::MatrixXd::Zero(1, static_cast<Eigen::Index>(yaw_degree) + 1);
      stationary_yaw(0, yaw_degree) =
          last_yaw_piece.getCoeffMat()(0, yaw_degree);
      yaw_to_publish.emplace_back(kTerminalHoldDurationS, stationary_yaw);
    }
  }
  mars_quadrotor_msgs::msg::PolynomialTrajectory message;
  message.header.frame_id = planning_frame_;
  message.header.stamp = rosTimeFromSeconds(now_seconds);
  message.trajectory_id = ++trajectory_id_;
  message.type = mars_quadrotor_msgs::msg::PolynomialTrajectory::POSITION_TRAJ |
                 mars_quadrotor_msgs::msg::PolynomialTrajectory::HEART_BEAT;
  message.start_wt_pos = rosTimeFromSeconds(position_to_publish.start_WT);
  message.piece_num_pos = static_cast<std::uint32_t>(position_to_publish.getPieceNum());
  message.order_pos = position_to_publish.empty()
                          ? 0U
                          : static_cast<std::uint32_t>(position_to_publish[0].getDegree());
  const std::size_t coefficient_count = static_cast<std::size_t>(message.order_pos) + 1U;
  message.time_pos.reserve(message.piece_num_pos);
  message.coef_pos_x.reserve(static_cast<std::size_t>(message.piece_num_pos) * coefficient_count);
  message.coef_pos_y.reserve(static_cast<std::size_t>(message.piece_num_pos) * coefficient_count);
  message.coef_pos_z.reserve(static_cast<std::size_t>(message.piece_num_pos) * coefficient_count);
  for (std::uint32_t piece_index = 0; piece_index < message.piece_num_pos; ++piece_index) {
    const auto& piece = position_to_publish[static_cast<int>(piece_index)];
    if (piece.getDegree() != static_cast<int>(message.order_pos) || piece.getDim() != 3) {
      throw std::runtime_error("SUPER position trajectory has inconsistent piece dimensions");
    }
    const auto coefficients = piece.getCoeffMat();
    // SUPER Piece stores [t^n ... t 1], while PolynomialTrajectory and the
    // PX4 evaluator use ascending powers [1 t ... t^n].
    for (std::size_t coefficient = coefficient_count; coefficient-- > 0;) {
      message.coef_pos_x.push_back(coefficients(0, static_cast<Eigen::Index>(coefficient)));
      message.coef_pos_y.push_back(coefficients(1, static_cast<Eigen::Index>(coefficient)));
      message.coef_pos_z.push_back(coefficients(2, static_cast<Eigen::Index>(coefficient)));
    }
    message.time_pos.push_back(piece.getDuration());
  }
  if (!yaw_to_publish.empty()) {
    message.type |= mars_quadrotor_msgs::msg::PolynomialTrajectory::YAW_TRAJ;
    message.start_wt_yaw = rosTimeFromSeconds(yaw_to_publish.start_WT);
    message.piece_num_yaw = static_cast<std::uint32_t>(yaw_to_publish.getPieceNum());
    message.order_yaw = static_cast<std::uint32_t>(yaw_to_publish[0].getDegree());
    const std::size_t yaw_coefficients = static_cast<std::size_t>(message.order_yaw) + 1U;
    message.time_yaw.reserve(message.piece_num_yaw);
    message.coef_yaw.reserve(static_cast<std::size_t>(message.piece_num_yaw) * yaw_coefficients);
    for (std::uint32_t piece_index = 0; piece_index < message.piece_num_yaw; ++piece_index) {
      const auto& piece = yaw_to_publish[static_cast<int>(piece_index)];
      const auto coefficients = piece.getCoeffMat();
      for (std::size_t coefficient = yaw_coefficients; coefficient-- > 0;) {
        message.coef_yaw.push_back(coefficients(0, static_cast<Eigen::Index>(coefficient)));
      }
      message.time_yaw.push_back(piece.getDuration());
    }
  }
  message.debug_info = append_terminal_hold ? "terminal_hold=2.0" : "";
  return message;
}

void SuperNavigationNode::runCycle() {
  const auto cycle_started = std::chrono::steady_clock::now();
  ++cycle_count_;
  std::shared_ptr<rog_map::PointCloud> cloud;
  std::optional<nav_msgs::msg::Odometry> odometry;
  std::optional<navigation_interfaces::msg::NavigationGoal> goal;
  bool new_goal = false;
  std::int64_t input_conversion_us = 0;
  std::uint64_t dropped_cloud_count = 0;
  const auto input_lock_started = std::chrono::steady_clock::now();
  {
    std::lock_guard<std::mutex> lock(input_mutex_);
    last_input_lock_wait_us_ = std::chrono::duration_cast<std::chrono::microseconds>(
        std::chrono::steady_clock::now() - input_lock_started).count();
    cloud = std::move(latest_cloud_);
    odometry = latest_odometry_;
    goal = active_goal_;
    new_goal = new_goal_;
    input_conversion_us = last_input_conversion_us_;
    dropped_cloud_count = dropped_cloud_count_;
  }
  if (!cloud || !odometry) return;
  if (!std::isfinite(odometry->pose.pose.position.x) ||
      !std::isfinite(odometry->pose.pose.position.y) ||
      !std::isfinite(odometry->pose.pose.position.z)) {
    return;
  }

  const double now_seconds = now().seconds();
  ros_interface_->setSimTime(now_seconds);
  const auto& pose = odometry->pose.pose;
  const super_utils::Pose super_pose{
      super_utils::Vec3f{pose.position.x, pose.position.y, pose.position.z},
      super_utils::Quatf{pose.orientation.w, pose.orientation.x, pose.orientation.y,
                         pose.orientation.z}};
  const auto map_started = std::chrono::steady_clock::now();
  try {
    map_->updateMap(*cloud, super_pose);
  } catch (const std::exception& error) {
    ++map_update_exception_count_;
    RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 2000,
                          "SUPER ROG-Map update failed: %s", error.what());
    return;
  }
  last_map_update_us_ = std::chrono::duration_cast<std::chrono::microseconds>(
      std::chrono::steady_clock::now() - map_started).count();

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
  add_value("received_observation_count", received_cloud_count_);
  add_value("accepted_observation_count", accepted_cloud_count_);
  add_value("cycle_count", cycle_count_);
  add_value("trajectory_publish_count", cycle_success_count_);
  add_value("dropped_cloud_count", dropped_cloud_count);
  add_value("processing_exception_count", map_update_exception_count_);
  diagnostics.status.push_back(std::move(status));
  diagnostics_publisher_->publish(diagnostics);

  if (!goal) return;

  // SUPER keeps a planner-local RobotState copy while ROG-Map owns the
  // authoritative state. Refresh it after every coherent map update before
  // invoking either PlanFromRest or ReplanOnce; otherwise the imported core
  // legitimately reports "No odom" even though the ROS odometry is healthy.
  rog_map::RobotState planner_state;
  planner_->getRobotState(planner_state);

  // SUPER owns one local polynomial for the currently requested goal. The
  // mission controller may expose a lookahead waypoint for bookkeeping, but
  // sending that lookahead into the optimizer makes the planner solve beyond
  // the observed local corridor and is not part of the SUPER contract.
  const auto& planner_target = goal->target;
  const super_utils::Vec3f target{
      static_cast<float>(pointFromMessage(planner_target, 0)),
      static_cast<float>(pointFromMessage(planner_target, 1)),
      static_cast<float>(pointFromMessage(planner_target, 2))};
  const auto planner_started = std::chrono::steady_clock::now();
  bool committed_expired = false;
  planner_->lockCommittedTraj();
  const auto committed_before_plan = planner_->getCommittedPositionTrajectory();
  if (!committed_before_plan.empty()) {
    const double elapsed = now_seconds - committed_before_plan.start_WT;
    committed_expired = std::isfinite(elapsed) &&
                        elapsed >= committed_before_plan.getTotalDuration() - 0.05;
  }
  planner_->unlockCommittedTraj();
  // SUPER's rolling replanner can legitimately return NO_NEED after its
  // replan-forward window reaches the end of a short local trajectory. Start
  // a fresh horizon from the current odometry at that boundary; otherwise
  // the PX4 side would keep receiving the expired local polynomial and stop
  // at the first horizon instead of advancing toward the mission goal.
  const auto result = (new_goal || committed_expired)
                          ? planner_->PlanFromRest(target, 0.0, new_goal)
                          : planner_->ReplanOnce(target, 0.0, false);
  last_planner_us_ = std::chrono::duration_cast<std::chrono::microseconds>(
      std::chrono::steady_clock::now() - planner_started).count();
  if (result == super_utils::FAILED || result == super_utils::EMER) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "SUPER planning failed (%d)", result);
  }
  if (result == super_utils::EMER) return;
  if (result == super_utils::SUCCESS) {
    std::lock_guard<std::mutex> lock(input_mutex_);
    // Do not fall back to hot replan after a failed new-goal attempt.  That
    // would keep publishing the previous waypoint while the mission has
    // already advanced.
    if (new_goal_) new_goal_ = false;
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
    planner_->unlockCommittedTraj();
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
    RCLCPP_INFO(get_logger(),
                "super_cycle_metrics cycles=%lu heartbeats=%lu dropped_cloud=%lu "
                "planner_cycle_ms=%.3f p50_ms=%.3f p95_ms=%.3f input_us=%ld map_us=%ld "
                "planner_us=%ld publish_us=%ld input_lock_us=%ld target=(%.2f,%.2f,%.2f) "
                "robot=(%.2f,%.2f,%.2f) committed_start=(%.2f,%.2f,%.2f) "
                "committed_q1=(%.2f,%.2f,%.2f) committed_mid=(%.2f,%.2f,%.2f) "
                "committed_q3=(%.2f,%.2f,%.2f) committed_end=(%.2f,%.2f,%.2f) "
                "committed_duration=%.3f",
                static_cast<unsigned long>(cycle_count_),
                static_cast<unsigned long>(heartbeat_publish_count_.load()),
                static_cast<unsigned long>(dropped_cloud_count), planner_cycle_ms,
                percentile(0.50), percentile(0.95), static_cast<long>(input_conversion_us),
                static_cast<long>(last_map_update_us_), static_cast<long>(last_planner_us_),
                static_cast<long>(last_publish_us_.load()),
                static_cast<long>(last_input_lock_wait_us_), target.x(), target.y(), target.z(),
                planner_state.p.x(), planner_state.p.y(), planner_state.p.z(),
                committed_start.x(), committed_start.y(), committed_start.z(),
                committed_quarter.x(), committed_quarter.y(), committed_quarter.z(),
                committed_half.x(), committed_half.y(), committed_half.z(),
                committed_three_quarter.x(), committed_three_quarter.y(), committed_three_quarter.z(),
                committed_end.x(), committed_end.y(), committed_end.z(), committed_duration);
    metrics_log_time_ = metrics_now;
  }
}

void SuperNavigationNode::publishHeartbeat() {
  const double now_seconds = now().seconds();
  if (!std::isfinite(now_seconds)) return;

  bool append_terminal_hold = false;
  {
    std::lock_guard<std::mutex> lock(input_mutex_);
    append_terminal_hold = active_goal_.has_value() &&
                           active_goal_->behavior ==
                               navigation_interfaces::msg::NavigationGoal::BEHAVIOR_STOP;
  }

  geometry_utils::Trajectory position;
  geometry_utils::Trajectory yaw;
  bool terminal_hold_candidate = false;
  bool terminal_hold_required = false;
  Eigen::Vector3d terminal_endpoint = Eigen::Vector3d::Constant(
      std::numeric_limits<double>::quiet_NaN());
  planner_->lockCommittedTraj();
  const auto committed = planner_->getCommittedPositionTrajectory();
  if (!committed.empty()) {
    const double elapsed = now_seconds - committed.start_WT;
    const double total = committed.getTotalDuration();
    if (append_terminal_hold && std::isfinite(elapsed) && std::isfinite(total) &&
        elapsed >= total - 0.5) {
      terminal_hold_required = true;
      terminal_endpoint = committed.getPos(total);
    }
    if (elapsed < total) {
      const double start = std::max(0.0, elapsed);
      // SUPER's partial-trajectory helper treats an exact piece boundary as
      // belonging to the preceding piece.  That creates a zero-duration
      // first piece when the PX4 heartbeat lands on a boundary.  Probe a few
      // milliseconds ahead and reject every partial containing a non-positive
      // piece instead of publishing a message PX4 must discard.
      constexpr double kMinimumPieceDurationS = 1.0e-4;
      constexpr double kMinimumPublishTailS = 0.05;
      constexpr double kBoundaryProbeS = 0.002;
      const double safe_total = total - kMinimumPublishTailS;
      for (int attempt = 0; attempt < 4 && position.empty(); ++attempt) {
        const double candidate_start = start + static_cast<double>(attempt) * kBoundaryProbeS;
        const double candidate_end = std::min(
            safe_total, candidate_start + heartbeat_horizon_s_);
        if (candidate_end <= candidate_start + kMinimumPieceDurationS ||
            !planner_->getCommittedPartialTrajectory(
                candidate_start, candidate_end, position, yaw)) {
          position.clear();
          yaw.clear();
          continue;
        }
        bool valid = true;
        for (int i = 0; i < position.getPieceNum(); ++i) {
          valid = valid && std::isfinite(position[i].getDuration()) &&
                  position[i].getDuration() > kMinimumPieceDurationS;
        }
        for (int i = 0; i < yaw.getPieceNum(); ++i) {
          valid = valid && std::isfinite(yaw[i].getDuration()) &&
                  yaw[i].getDuration() > kMinimumPieceDurationS;
        }
        if (!valid) {
          position.clear();
          yaw.clear();
          continue;
        }
        terminal_hold_candidate = append_terminal_hold &&
                                  (safe_total - candidate_end <= 0.10 ||
                                   terminal_hold_required);
      }
    }
  }
  planner_->unlockCommittedTraj();
  if (position.empty() && terminal_hold_required && terminal_endpoint.allFinite()) {
    // Once the nominal SUPER trajectory has reached its endpoint, the planner
    // is allowed to stop producing rolling slices.  Keep the PX4 contract
    // alive with a fresh stationary polynomial so a terminal stop can settle
    // and be confirmed by MissionController instead of failing on a heartbeat
    // gap at the exact endpoint.
    constexpr int kTerminalHoldDegree = 5;
    constexpr double kTerminalHoldDurationS = 2.0;
    Eigen::MatrixXd stationary =
        Eigen::MatrixXd::Zero(3, kTerminalHoldDegree + 1);
    stationary.col(kTerminalHoldDegree) = terminal_endpoint;
    position.start_WT = now_seconds;
    position.emplace_back(kTerminalHoldDurationS, stationary);
    yaw.clear();
    terminal_hold_candidate = false;
  }
  if (position.empty()) return;

  try {
    const auto publish_started = std::chrono::steady_clock::now();
    trajectory_publisher_->publish(
        serializeTrajectory(position, yaw, now_seconds,
                            append_terminal_hold && terminal_hold_candidate));
    last_publish_us_ = std::chrono::duration_cast<std::chrono::microseconds>(
        std::chrono::steady_clock::now() - publish_started).count();
    ++heartbeat_publish_count_;
    ++cycle_success_count_;
  } catch (const std::exception& error) {
    RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 2000,
                          "SUPER trajectory heartbeat serialization failed: %s", error.what());
  }
}

}  // namespace navigation_runtime
