#include <chrono>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <memory>

#include <gtest/gtest.h>
#include <rcl/time.h>

#include "px4_navigation_external_mode/navigation_mode.hpp"

namespace px4_navigation_external_mode {

// Runs the product adapter callbacks and mission update with a controlled ROS
// clock. FMU registration/arming and trajectory certification are not mocked:
// they are outside this unit fixture and remain integration-test obligations.
// Initial progression is seeded through MissionController's ordinary API.
class NavigationModeProgressionTest : public ::testing::Test {
 protected:
  using Command = navigation_contracts::msg::NavigationCommand;
  using Health = navigation_contracts::msg::EstimatorHealth;
  static constexpr std::int64_t kBefore = 38'140'000'000;
  static constexpr std::int64_t kCrossing = 38'156'000'000;
  static constexpr std::int64_t kReplacement = 38'172'000'000;
  static constexpr std::int64_t kNextTick = 38'190'000'000;
  static constexpr std::uint64_t kBoundary = 38'139'283'003;

  static void SetUpTestSuite() {
    rclcpp::init(0, nullptr);
  }
  static void TearDownTestSuite() {
    rclcpp::shutdown();
  }

  void SetUp() override {
    mission_path_ = std::filesystem::temp_directory_path() /
        ("uav_navigation_progression_" + std::to_string(
            std::chrono::steady_clock::now().time_since_epoch().count()) + ".yaml");
    {
      std::ofstream mission(mission_path_);
      ASSERT_TRUE(mission.good());
      mission << R"yaml(
mission:
  version: 1
  id: continuation_window
  frame: lio_odom
  waypoints:
    - {id: start, position: [0, 0, 3], acceptance_radius_m: 0.9, behavior: pass_through}
    - {id: crossing, position: [20, 5, 3], acceptance_radius_m: 0.9, behavior: pass_through}
    - {id: finish, position: [50, 5, 3], acceptance_radius_m: 0.9, behavior: stop}
  planning:
    requested_cruise_speed_mps: 5.0
    unknown_policy: blocked
)yaml";
    }
    rclcpp::NodeOptions options;
    options.parameter_overrides({
        rclcpp::Parameter("use_sim_time", false),
        rclcpp::Parameter("navigation.mission_file", mission_path_.string())});
    node_ = std::make_shared<rclcpp::Node>("navigation_mode_progression_test", options);
    ASSERT_EQ(rcl_enable_ros_time_override(node_->get_clock()->get_clock_handle()), RCL_RET_OK);
    setNow(37'000'000'000);
    mode_ = std::make_unique<NavigationMode>(*node_);
    mode_->mission_timer_->cancel();
    ASSERT_FALSE(mode_->tracking_experiment_.enabled);
    ASSERT_FALSE(mode_->tracking_experiment_.suppress_estimator_health_response);
    mode_->setPx4HoldHandover([this]() { ++hold_count_; });
    auto& controller = *mode_->mission_controller_;
    controller.activate(37.0);
    auto event = controller.update(37.0, std::nullopt, true, Eigen::Vector3d::Zero());
    ASSERT_EQ(event.type, MissionControllerEvent::Type::PublishGoal);
    // Do not emit a goal on a missing measured route snapshot. This fixture
    // starts after the ordinary airborne/measured initial checkpoint handoff.
    setNow(37'020'000'000);
    event = controller.update(37.02, Eigen::Vector3d{0, 0, 3}, true,
                              Eigen::Vector3d::Zero());
    ASSERT_TRUE(event.waypoint_accepted);
    mode_->handleMissionEvent(event, 37.02);
    ASSERT_EQ(waypoint(), 1U);
    ASSERT_EQ(request(), 2U);
  }

  void TearDown() override {
    mode_.reset();
    node_.reset();
    if (!mission_path_.empty()) std::filesystem::remove(mission_path_);
  }

  void setNow(std::int64_t now_ns) {
    ASSERT_EQ(rcl_set_ros_time_override(node_->get_clock()->get_clock_handle(), now_ns), RCL_RET_OK);
    ASSERT_EQ(node_->get_clock()->now().nanoseconds(), now_ns);
    now_ns_ = now_ns;
  }
  builtin_interfaces::msg::Time stamp(std::int64_t ns) const {
    return rclcpp::Time(ns, RCL_ROS_TIME);
  }
  void health(bool valid = true, std::uint64_t epoch = 7U,
              std::int64_t source_ns = 0) {
    auto message = std::make_shared<Health>();
    message->header.frame_id = "lio_odom";
    message->header.stamp = stamp(source_ns > 0 ? source_ns : now_ns_);
    message->localization_epoch = epoch;
    message->state = valid ? Health::TRACKING : Health::DEGRADED;
    message->navigation_valid = valid;
    message->covariance_valid = valid;
    message->observability_valid = valid;
    message->correction_fresh = valid;
    message->propagation_valid = valid;
    message->last_correction_stamp = message->header.stamp;
    message->last_propagated_state_stamp = message->header.stamp;
    mode_->onEstimatorHealth(message);
  }
  void odometry(const Eigen::Vector3d& position, std::int64_t source_ns = 0) {
    auto message = std::make_shared<navigation_contracts::msg::PropagatedOdometry>();
    message->localization_epoch = 7U;
    message->sequence = ++sequence_;
    auto& odom = message->odometry;
    odom.header.frame_id = "lio_odom";
    odom.header.stamp = stamp(source_ns > 0 ? source_ns : now_ns_);
    odom.child_frame_id = "base_link";
    odom.pose.pose.position.x = position.x();
    odom.pose.pose.position.y = position.y();
    odom.pose.pose.position.z = position.z();
    odom.pose.pose.orientation.w = 1.0;
    odom.twist.twist.linear.x = 3.49;
    mode_->onOdometry(message);
  }
  static Eigen::Vector3d inBall() {
    return {19.149405286591282, 4.816194674794652, 2.961046013093615};
  }
  std::shared_ptr<Command> command(bool continuation = true) {
    auto message = std::make_shared<Command>();
    message->header.frame_id = "lio_odom";
    message->header.stamp = stamp(now_ns_);
    message->localization_epoch = 7U;
    message->goal_epoch = 2U;
    message->mission_id = "continuation_window";
    message->waypoint_index = 1U;
    message->request_id = 2U;
    message->world_generation = 2U;
    message->world_revision = 1U;
    message->world_observation_stamp = stamp(now_ns_);
    message->bundle_generation = 8U;
    message->sample_id = ++sample_;
    message->state_source_stamp = stamp(now_ns_);
    message->valid_until = stamp(now_ns_ + 100'000'000);
    message->certified_main_continuation = continuation;
    message->continuation_boundary_stamp_ns = continuation ? kBoundary : 0U;
    message->role = Command::ROLE_MAIN;
    message->status = Command::STATUS_READY;
    message->trajectory_time_s = static_cast<double>(now_ns_ - 37'612'000'000) / 1e9;
    if (mode_->odometry_) message->position = mode_->odometry_->pose.pose.position;
    message->velocity.x = 3.49;
    return message;
  }
  void admit(const std::shared_ptr<Command>& message) {
    mode_->onNavigationCommand(message);
  }
  void timerTick() { mode_->updateMission(); }
  std::size_t waypoint() const { return mode_->mission_controller_->activeWaypointIndex(); }
  std::uint64_t request() const { return mode_->mission_controller_->activeRequestId(); }
  std::uint64_t acceptedSample() const {
    return mode_->navigation_command_ ? mode_->navigation_command_->sample_id : 0U;
  }
  bool failed() const { return mode_->failure_reported_; }
  void requestHold() { mode_->safetyStopNavigation("progression fixture handover"); }
  void expectSerializedCallbacks() const {
    const auto group = node_->get_node_base_interface()->get_default_callback_group();
    EXPECT_EQ(group->type(), rclcpp::CallbackGroupType::MutuallyExclusive);
    EXPECT_EQ(group->find_subscription_ptrs_if([this](const auto& subscription) {
      return subscription == mode_->navigation_command_subscription_;
    }), mode_->navigation_command_subscription_);
    EXPECT_EQ(group->find_timer_ptrs_if([this](const auto& timer) {
      return timer == mode_->mission_timer_;
    }), mode_->mission_timer_);
  }
  void startOutside() {
    setNow(kBefore);
    health();
    odometry({19.0, 4.8, 3.0});
    admit(command());
    timerTick();
    ASSERT_EQ(waypoint(), 1U);
  }

  std::filesystem::path mission_path_;
  std::shared_ptr<rclcpp::Node> node_;
  std::unique_ptr<NavigationMode> mode_;
  std::int64_t now_ns_{0};
  std::uint64_t sample_{0U};
  std::uint64_t sequence_{0U};
  std::size_t hold_count_{0U};
};

TEST_F(NavigationModeProgressionTest, AdmittedContinuationAdvancesBeforeNextTimerTick) {
  startOutside();
  setNow(kCrossing);
  health();
  odometry(inBall());
  const auto crossing = command();
  admit(crossing);
  ASSERT_EQ(acceptedSample(), crossing->sample_id);
  EXPECT_EQ(waypoint(), 2U);
  EXPECT_EQ(request(), 3U);
  setNow(kReplacement);
  health();
  odometry(inBall());
  admit(command(false));  // Retained predecessor must not advance the new WP.
  setNow(kNextTick);
  timerTick();
  EXPECT_EQ(waypoint(), 2U);
  EXPECT_EQ(request(), 3U);
  EXPECT_EQ(hold_count_, 0U);
}

TEST_F(NavigationModeProgressionTest, FalseReplacementDoesNotLatchEarlierPermission) {
  startOutside();
  setNow(kCrossing);
  health();
  admit(command());  // Last true command arrives while measured state is outside.
  EXPECT_EQ(waypoint(), 1U);
  setNow(kReplacement);
  health();
  odometry(inBall());
  admit(command(false));
  setNow(kNextTick);
  timerTick();
  EXPECT_EQ(waypoint(), 1U);
}

TEST_F(NavigationModeProgressionTest, FreshTimerWitnessStillAdvancesNormally) {
  startOutside();
  setNow(kCrossing);
  health();
  admit(command());
  setNow(kCrossing + 1'000'000);
  health();
  odometry(inBall());
  timerTick();
  EXPECT_EQ(waypoint(), 2U);
}

TEST_F(NavigationModeProgressionTest, DuplicateCannotReusePermissionWithNewMeasuredState) {
  startOutside();
  setNow(kCrossing);
  health();
  const auto last = command();
  admit(last);
  setNow(kReplacement);
  health();
  odometry(inBall());
  admit(last);
  EXPECT_EQ(waypoint(), 1U);
}

TEST_F(NavigationModeProgressionTest, WrongEpochCannotAuthorizeProgression) {
  startOutside();
  setNow(kCrossing);
  health();
  odometry(inBall());
  auto wrong = command();
  wrong->localization_epoch = 8U;
  admit(wrong);
  EXPECT_EQ(waypoint(), 1U);
  EXPECT_NE(acceptedSample(), wrong->sample_id);
}

TEST_F(NavigationModeProgressionTest, WrongMissionCannotAuthorizeProgression) {
  startOutside();
  setNow(kCrossing);
  health();
  odometry(inBall());
  auto wrong = command();
  wrong->mission_id = "different_mission";
  admit(wrong);
  EXPECT_EQ(waypoint(), 1U);
  EXPECT_NE(acceptedSample(), wrong->sample_id);
}

TEST_F(NavigationModeProgressionTest, ExpiredCommandCannotAuthorizeProgression) {
  startOutside();
  setNow(kCrossing);
  health();
  odometry(inBall());
  auto expired = command();
  expired->header.stamp = stamp(now_ns_ - 200'000'000);
  expired->valid_until = stamp(now_ns_ - 100'000'000);
  admit(expired);
  EXPECT_EQ(waypoint(), 1U);
  EXPECT_NE(acceptedSample(), expired->sample_id);
}

TEST_F(NavigationModeProgressionTest, StaleHealthAtUseCannotAuthorizeProgression) {
  setNow(kCrossing);
  health(true, 7U, now_ns_ - 201'000'000);
  odometry(inBall());
  auto fresh_command = command();
  admit(fresh_command);
  EXPECT_EQ(acceptedSample(), fresh_command->sample_id);
  EXPECT_EQ(waypoint(), 1U);
}

TEST_F(NavigationModeProgressionTest, StaleOdometryFailsClosedBeforeProgression) {
  setNow(kCrossing);
  health();
  odometry(inBall(), now_ns_ - 201'000'000);
  admit(command());
  EXPECT_EQ(waypoint(), 1U);
  EXPECT_TRUE(failed());
}

TEST_F(NavigationModeProgressionTest, InvalidHealthCannotAuthorizeProgression) {
  startOutside();
  setNow(kCrossing);
  odometry(inBall());
  health(false);
  auto fresh_command = command();
  admit(fresh_command);
  EXPECT_EQ(waypoint(), 1U);
  EXPECT_NE(acceptedSample(), fresh_command->sample_id);
}

TEST_F(NavigationModeProgressionTest, FreshBackupIsNotMainContinuation) {
  setNow(kCrossing);
  health();
  odometry(inBall());
  auto backup = command(false);
  backup->role = Command::ROLE_BACKUP;
  admit(backup);
  EXPECT_EQ(acceptedSample(), backup->sample_id);
  EXPECT_EQ(waypoint(), 1U);
}

TEST_F(NavigationModeProgressionTest, EpochResetRevokesPreviouslyAdmittedWitness) {
  startOutside();
  setNow(kCrossing);
  health(true, 8U);
  odometry(inBall());  // Old epoch is rejected by the actual state callback.
  admit(command());
  timerTick();
  EXPECT_EQ(waypoint(), 1U);
  EXPECT_EQ(acceptedSample(), 0U);
}

TEST_F(NavigationModeProgressionTest, TimerAndCommandUseSameSerializedCallbackGroup) {
  expectSerializedCallbacks();
}

TEST_F(NavigationModeProgressionTest, AcceptedPredecessorCannotAuthorizeCurrentWaypoint) {
  setNow(kCrossing);
  health();
  odometry(inBall());
  auto predecessor = command();
  predecessor->waypoint_index = 0U;
  predecessor->request_id = 1U;
  admit(predecessor);
  EXPECT_EQ(acceptedSample(), predecessor->sample_id);  // Legitimate continuity bridge.
  EXPECT_EQ(waypoint(), 1U);
  timerTick();
  EXPECT_EQ(waypoint(), 1U);
}

TEST_F(NavigationModeProgressionTest, WrongCurrentRequestCannotAuthorizeProgression) {
  startOutside();
  setNow(kCrossing);
  health();
  odometry(inBall());
  auto wrong = command();
  wrong->request_id = 3U;
  admit(wrong);
  EXPECT_NE(acceptedSample(), wrong->sample_id);
  EXPECT_EQ(waypoint(), 1U);
}

TEST_F(NavigationModeProgressionTest, HandoverCannotBeReversedByContinuationArrival) {
  startOutside();
  requestHold();
  ASSERT_EQ(hold_count_, 1U);
  setNow(kCrossing);
  health();
  odometry(inBall());
  admit(command());
  timerTick();
  EXPECT_EQ(waypoint(), 1U);
  EXPECT_EQ(acceptedSample(), 0U);
  EXPECT_EQ(hold_count_, 1U);
}

}  // namespace px4_navigation_external_mode
