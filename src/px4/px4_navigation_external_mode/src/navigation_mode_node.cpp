#include "px4_navigation_external_mode/navigation_mode.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

#include <px4_ros2/components/node_with_mode.hpp>

namespace px4_navigation_external_mode {
namespace {

constexpr char kModeName[] = "UAV Navigation";
constexpr char kTrajectoryFailureReason[] = "navigation trajectory unavailable or stale";

}  // namespace

NavigationMode::NavigationMode(rclcpp::Node& node)
    : ModeBase(node, Settings{kModeName}),
      trajectory_setpoint_(std::make_shared<px4_ros2::TrajectorySetpointType>(*this)),
      trajectory_topic_(node.declare_parameter<std::string>(
          "navigation.trajectory_topic", "/navigation/trajectory")),
      planning_frame_(node.declare_parameter<std::string>(
          "navigation.planning_frame", "lio_odom")),
      stale_after_s_(node.declare_parameter<double>(
          "navigation.trajectory_stale_after_s", 0.75)) {
  if (trajectory_topic_.empty() || planning_frame_.empty() || !std::isfinite(stale_after_s_) ||
      stale_after_s_ <= 0.0) {
    throw std::invalid_argument("invalid PX4 navigation external mode parameters");
  }
  trajectory_subscription_ = node.create_subscription<
      navigation_interfaces::msg::PlannedTrajectory>(
      trajectory_topic_, rclcpp::QoS{rclcpp::KeepLast{1}}.reliable(),
      [this](const navigation_interfaces::msg::PlannedTrajectory::ConstSharedPtr& message) {
        onTrajectory(message);
      });
  setSetpointUpdateRate(50.0F);
}

void NavigationMode::onActivate() {
  std::lock_guard<std::mutex> lock(trajectory_mutex_);
  activation_time_ = node().get_clock()->now();
  if (trajectory_.has_value()) {
    trajectory_start_time_ = activation_time_;
  }
  failure_reported_ = false;
}

void NavigationMode::onDeactivate() {
  std::lock_guard<std::mutex> lock(trajectory_mutex_);
  failure_reported_ = false;
}

void NavigationMode::checkArmingAndRunConditions(
    px4_ros2::HealthAndArmingCheckReporter& reporter) {
  std::lock_guard<std::mutex> lock(trajectory_mutex_);
  if (!trajectory_.has_value()) {
    reporter.armingCheckFailureExt(
        px4_ros2::events::ID("uav_navigation_trajectory_unavailable"),
        px4_ros2::events::Log::Error, "Navigation trajectory unavailable");
  }
}

void NavigationMode::onTrajectory(
    const navigation_interfaces::msg::PlannedTrajectory::ConstSharedPtr& message) {
  const auto validation = validateTrajectory(*message, planning_frame_);
  if (!validation.valid()) {
    RCLCPP_WARN(node().get_logger(), "Rejecting navigation trajectory: %s",
                validation.message.c_str());
    std::lock_guard<std::mutex> lock(trajectory_mutex_);
    trajectory_.reset();
    return;
  }

  std::lock_guard<std::mutex> lock(trajectory_mutex_);
  trajectory_ = *message;
  trajectory_start_time_ = node().get_clock()->now();
  failure_reported_ = false;
  RCLCPP_DEBUG(node().get_logger(), "Accepted trajectory generation=%lu revision=%lu duration=%.3f s",
               static_cast<unsigned long>(message->world_generation),
               static_cast<unsigned long>(message->world_revision), message->duration_s);
}

void NavigationMode::failNavigation(const char* reason) {
  {
    std::lock_guard<std::mutex> lock(trajectory_mutex_);
    if (failure_reported_) return;
    failure_reported_ = true;
  }
  RCLCPP_ERROR(node().get_logger(), "%s; completing External Mode with failure", reason);
  completed(px4_ros2::Result::ModeFailureOther);
}

void NavigationMode::updateSetpoint(float /*dt_s*/) {
  std::optional<navigation_interfaces::msg::PlannedTrajectory> trajectory;
  rclcpp::Time start_time;
  {
    std::lock_guard<std::mutex> lock(trajectory_mutex_);
    trajectory = trajectory_;
    start_time = trajectory_start_time_;
  }

  if (!trajectory.has_value()) {
    failNavigation(kTrajectoryFailureReason);
    return;
  }

  const double elapsed_s = std::max(0.0, (node().get_clock()->now() - start_time).seconds());
  const double freshness_limit_s = trajectory->duration_s + stale_after_s_;
  if (!std::isfinite(elapsed_s) || elapsed_s > freshness_limit_s) {
    failNavigation(kTrajectoryFailureReason);
    return;
  }

  const auto sample = sampleTrajectory(*trajectory, elapsed_s);
  px4_ros2::TrajectorySetpoint setpoint;
  const Eigen::Vector3f position_ned = enuToNed(sample.position_enu);
  const Eigen::Vector3f velocity_ned = enuToNed(sample.velocity_enu);
  const Eigen::Vector3f acceleration_ned = enuToNed(sample.acceleration_enu);
  setpoint.withPosition(position_ned)
      .withVelocity(velocity_ned)
      .withAcceleration(acceleration_ned);
  trajectory_setpoint_->update(setpoint);
}

NavigationModeExecutor::NavigationModeExecutor(px4_ros2::ModeBase& owned_mode)
    : ModeExecutorBase(px4_ros2::ModeExecutorBase::Settings{}, owned_mode),
      node_(owned_mode.node()) {}

void NavigationModeExecutor::onActivate() {
  RCLCPP_INFO(node_.get_logger(), "UAV Navigation External Mode activated");
  scheduleOwnedMode(px4_ros2::Result::Success);
}

void NavigationModeExecutor::scheduleOwnedMode(px4_ros2::Result previous_result) {
  if (previous_result == px4_ros2::Result::Deactivated) {
    RCLCPP_DEBUG(node_.get_logger(), "Owned navigation mode was deactivated by mode handover");
    return;
  }
  if (previous_result != px4_ros2::Result::Success) {
    RCLCPP_ERROR(node_.get_logger(), "Navigation mode completed with result=%s; requesting RTL",
                 px4_ros2::resultToString(previous_result));
    if (isArmed()) {
      rtl([](px4_ros2::Result result) {
        (void)result;
      });
    }
    return;
  }
  scheduleMode(ownedMode().id(), [this](px4_ros2::Result result) {
    scheduleOwnedMode(result);
  });
}

void NavigationModeExecutor::onDeactivate(DeactivateReason reason) {
  RCLCPP_INFO(node_.get_logger(), "UAV Navigation External Mode deactivated (%s)",
              reason == DeactivateReason::FailsafeActivated ? "failsafe" : "mode_exit");
}

void NavigationModeExecutor::onFailsafeDeferred() {
  RCLCPP_WARN(node_.get_logger(), "PX4 requested a deferred failsafe; no failsafe is deferred");
}

}  // namespace px4_navigation_external_mode

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  using Node = px4_ros2::NodeWithModeExecutor<
      px4_navigation_external_mode::NavigationModeExecutor,
      px4_navigation_external_mode::NavigationMode>;
  rclcpp::spin(std::make_shared<Node>("px4_navigation_external_mode", true));
  rclcpp::shutdown();
  return 0;
}
