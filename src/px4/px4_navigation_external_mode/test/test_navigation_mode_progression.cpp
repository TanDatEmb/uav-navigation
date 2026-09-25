#include <cstdint>
#include <memory>

#include <gtest/gtest.h>
#include <rcl/time.h>

#include "px4_navigation_external_mode/navigation_mode.hpp"

namespace px4_navigation_external_mode {

// Exercises the real adapter boundary. Mission geometry and request creation
// are tested in Core; this fixture checks only local command admission and
// immutable Core completion receipt fencing.
class NavigationModeProgressionTest : public ::testing::Test {
 protected:
  using Command = navigation_contracts::msg::NavigationCommand;
  using Health = navigation_contracts::msg::EstimatorHealth;

  static void SetUpTestSuite() { rclcpp::init(0, nullptr); }
  static void TearDownTestSuite() { rclcpp::shutdown(); }

  void SetUp() override {
    node_ = std::make_shared<rclcpp::Node>("navigation_boundary_test");
    ASSERT_EQ(rcl_enable_ros_time_override(node_->get_clock()->get_clock_handle()), RCL_RET_OK);
    setNow(38'000'000'000);
    mode_ = std::make_unique<NavigationMode>(*node_);
    mode_->boundary_timer_->cancel();
    mode_->mode_active_ = true;
    mode_->mode_activation_id_ = 1U;
    mode_->activation_time_ = node_->get_clock()->now();
    health();
    odometry();
  }

  void TearDown() override {
    mode_.reset();
    node_.reset();
  }

  void setNow(std::int64_t ns) {
    ASSERT_EQ(rcl_set_ros_time_override(node_->get_clock()->get_clock_handle(), ns), RCL_RET_OK);
    now_ns_ = ns;
  }
  builtin_interfaces::msg::Time stamp(std::int64_t ns) const {
    return rclcpp::Time(ns, RCL_ROS_TIME);
  }
  void health() {
    auto message = std::make_shared<Health>();
    message->header.frame_id = "lio_odom";
    message->header.stamp = stamp(now_ns_);
    message->localization_epoch = 7U;
    message->state = Health::TRACKING;
    message->navigation_valid = true;
    message->covariance_valid = true;
    message->observability_valid = true;
    message->correction_fresh = true;
    message->propagation_valid = true;
    message->last_correction_stamp = message->header.stamp;
    message->last_propagated_state_stamp = message->header.stamp;
    mode_->onEstimatorHealth(message);
  }
  void odometry() {
    auto message = std::make_shared<navigation_contracts::msg::PropagatedOdometry>();
    message->localization_epoch = 7U;
    message->sequence = ++sequence_;
    auto& odom = message->odometry;
    odom.header.frame_id = "lio_odom";
    odom.header.stamp = stamp(now_ns_);
    odom.child_frame_id = "base_link";
    odom.pose.pose.position.z = 3.0;
    odom.pose.pose.orientation.w = 1.0;
    mode_->onOdometry(message);
  }
  std::int64_t lastAcceptedOdometryReceive() const {
    return mode_->last_odometry_receive_ns_;
  }
  std::uint64_t lastAcceptedOdometrySequence() const {
    return mode_->last_propagated_state_sequence_;
  }
  std::uint64_t acceptedOdometryCount() const {
    return mode_->odometry_callback_count_;
  }
  std::shared_ptr<Command> command(std::uint64_t goal_epoch,
                                   std::uint32_t waypoint,
                                   std::uint64_t request,
                                   std::string mission_id = "core_mission") {
    auto result = std::make_shared<Command>();
    result->header.frame_id = "lio_odom";
    result->header.stamp = stamp(now_ns_);
    result->localization_epoch = 7U;
    result->goal_epoch = goal_epoch;
    result->mode_activation_id = mode_->mode_activation_id_;
    result->mission_id = std::move(mission_id);
    result->waypoint_index = waypoint;
    result->request_id = request;
    result->world_generation = 2U;
    result->world_revision = 1U;
    result->world_observation_stamp = stamp(now_ns_);
    result->bundle_generation = goal_epoch + 1U;
    result->sample_id = ++sample_;
    result->state_source_stamp = stamp(now_ns_);
    result->valid_until = stamp(now_ns_ + 100'000'000);
    result->role = Command::ROLE_MAIN;
    result->status = Command::STATUS_READY;
    result->trajectory_time_s = 0.0;
    result->position.z = 3.0;
    return result;
  }
  std::uint64_t acceptedSample() const {
    return mode_->navigation_command_ ? mode_->navigation_command_->sample_id : 0U;
  }
  void admit(const std::shared_ptr<Command>& value) {
    mode_->onNavigationCommand(value);
  }
  void observeReceipt(
      const std::shared_ptr<navigation_contracts::msg::NavigationMissionProgress>& value) {
    mode_->onMissionProgress(value);
  }
  bool completed() const { return mode_->mission_completion_receipt_.has_value(); }
  bool handingOver() const { return mode_->handover_requested_; }
  void restartBoundaryForTest() {
    std::lock_guard<std::mutex> lock(mode_->trajectory_mutex_);
    mode_->navigation_command_.reset();
    mode_->mode_activation_id_ = 2U;
  }

  std::shared_ptr<rclcpp::Node> node_;
  std::unique_ptr<NavigationMode> mode_;
  std::int64_t now_ns_{0};
  std::uint64_t sequence_{0U};
  std::uint64_t sample_{0U};
};

TEST_F(NavigationModeProgressionTest, CoreSuccessorCommandAdvancesWithoutAdapterGoalWriter) {
  const auto first = command(2U, 0U, 1U);
  admit(first);
  ASSERT_EQ(acceptedSample(), first->sample_id);
  setNow(now_ns_ + 20'000'000);
  health();
  odometry();
  const auto successor = command(3U, 1U, 2U);
  admit(successor);
  EXPECT_EQ(acceptedSample(), successor->sample_id);
  EXPECT_FALSE(completed());
}

TEST_F(NavigationModeProgressionTest, RejectedSourceStampDoesNotRefreshAcceptedReceiveLease) {
  const auto accepted_receive = lastAcceptedOdometryReceive();
  const auto accepted_sequence = lastAcceptedOdometrySequence();
  const auto accepted_count = acceptedOdometryCount();
  odometry();  // Higher sequence, same source stamp: rejected.
  EXPECT_EQ(lastAcceptedOdometryReceive(), accepted_receive);
  EXPECT_EQ(lastAcceptedOdometrySequence(), accepted_sequence);
  EXPECT_EQ(acceptedOdometryCount(), accepted_count);

  setNow(now_ns_ + 20'000'000);
  odometry();
  EXPECT_EQ(lastAcceptedOdometryReceive(), now_ns_);
  EXPECT_EQ(lastAcceptedOdometrySequence(), sequence_);
  EXPECT_EQ(acceptedOdometryCount(), accepted_count + 1U);
}

TEST_F(NavigationModeProgressionTest, LatePredecessorCannotRegressAcceptedExecutionIdentity) {
  const auto first = command(2U, 0U, 1U);
  admit(first);
  setNow(now_ns_ + 20'000'000);
  health();
  odometry();
  const auto successor = command(3U, 1U, 2U);
  admit(successor);
  ASSERT_EQ(acceptedSample(), successor->sample_id);
  setNow(now_ns_ + 20'000'000);
  health();
  odometry();
  const auto late = command(2U, 0U, 1U);
  admit(late);
  EXPECT_EQ(acceptedSample(), successor->sample_id);
}

TEST_F(NavigationModeProgressionTest, ForeignMissionCannotReplaceSession) {
  const auto first = command(2U, 0U, 1U);
  admit(first);
  ASSERT_EQ(acceptedSample(), first->sample_id);
  setNow(now_ns_ + 20'000'000);
  health();
  odometry();
  admit(command(3U, 1U, 2U, "foreign"));
  EXPECT_EQ(acceptedSample(), first->sample_id);
}

TEST_F(NavigationModeProgressionTest, OldActivationCommandCannotReplayAfterModeReentry) {
  const auto first = command(2U, 0U, 1U);
  admit(first);
  ASSERT_EQ(acceptedSample(), first->sample_id);
  restartBoundaryForTest();
  setNow(now_ns_ + 20'000'000);
  health();
  odometry();
  const auto old_activation = std::make_shared<Command>(*first);
  old_activation->sample_id = ++sample_;
  old_activation->header.stamp = stamp(now_ns_);
  old_activation->valid_until = stamp(now_ns_ + 100'000'000);
  admit(old_activation);
  EXPECT_EQ(acceptedSample(), 0U);
  const auto current = command(3U, 0U, 2U);
  admit(current);
  EXPECT_EQ(acceptedSample(), current->sample_id);
}

TEST_F(NavigationModeProgressionTest, StaleCompletionReceiptCannotCompleteNewActivation) {
  auto receipt = std::make_shared<navigation_contracts::msg::NavigationMissionProgress>();
  receipt->header.frame_id = "lio_odom";
  receipt->header.stamp = stamp(now_ns_);
  receipt->mission_id = "core_mission";
  receipt->route_revision = 1U;
  receipt->localization_epoch = 7U;
  receipt->mode_activation_id = 2U;
  receipt->waypoint_index = 0U;
  receipt->request_id = 1U;
  receipt->event = navigation_contracts::msg::NavigationMissionProgress::COMPLETE;
  receipt->waypoint_accepted = true;
  receipt->accepted_waypoint_index = 0U;
  observeReceipt(receipt);
  EXPECT_FALSE(completed());
  EXPECT_FALSE(handingOver());
}

}  // namespace px4_navigation_external_mode
