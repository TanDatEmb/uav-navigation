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
  static std::int64_t fixtureStampNanoseconds(
      const builtin_interfaces::msg::Time& value) {
    // Fixture clocks are small, positive and representable. Use an independent
    // integral conversion rather than the producer/receiver contract helper.
    return static_cast<std::int64_t>(value.sec) * 1'000'000'000 + value.nanosec;
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
  void odometry(const Eigen::Vector3d& position, std::int64_t source_ns = 0,
                const Eigen::Vector3d& velocity = Eigen::Vector3d{3.49, 0.0, 0.0}) {
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
    odom.twist.twist.linear.x = velocity.x();
    odom.twist.twist.linear.y = velocity.y();
    odom.twist.twist.linear.z = velocity.z();
    mode_->onOdometry(message);
  }
  std::shared_ptr<Command> completedStopCommand() {
    auto message = command(false);
    message->goal_epoch = 3U;
    message->waypoint_index = static_cast<std::uint32_t>(waypoint());
    message->request_id = request();
    message->bundle_generation = 9U;
    message->status = Command::STATUS_COMPLETED;
    message->certified_main_continuation = false;
    message->velocity = geometry_msgs::msg::Vector3{};
    message->valid_until = stamp(now_ns_ + 100'000'000);
    return message;
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
  std::shared_ptr<Command> commandFromCanonicalMainReserve(
      const std::int64_t main_end_ns, const std::int64_t sphere_entry_ns) {
    // Synthetic arithmetic for the current 600 ms publisher contract. This
    // package has no navigation_planning dependency; keep the test independent
    // instead of introducing a runtime/backend dependency or a production gate.
    constexpr std::int64_t kSyntheticMainReserveNs = 600'000'000;
    auto message = command(false);
    const auto command_stamp_ns = fixtureStampNanoseconds(message->header.stamp);
    message->certified_main_continuation =
        main_end_ns - command_stamp_ns >= kSyntheticMainReserveNs;
    message->continuation_boundary_stamp_ns = message->certified_main_continuation
        ? static_cast<std::uint64_t>(sphere_entry_ns) : 0U;
    const Eigen::Vector3d velocity =
        3.49 * Eigen::Vector3d{20.0, 5.0, 0.0}.normalized();
    message->velocity.x = velocity.x();
    message->velocity.y = velocity.y();
    message->velocity.z = velocity.z();
    return message;
  }
  static Eigen::Vector3d syntheticSphereEntryPosition(
      const std::int64_t sample_ns, const std::int64_t sphere_entry_ns) {
    const Eigen::Vector3d incoming = Eigen::Vector3d{20.0, 5.0, 0.0}.normalized();
    const double elapsed_s = static_cast<double>(sample_ns - sphere_entry_ns) / 1.0e9;
    return Eigen::Vector3d{20.0, 5.0, 3.0} +
        (3.49 * elapsed_s - 0.9) * incoming;
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
  bool missionCompleted() const { return mode_->mission_complete_published_; }
  bool holding() const { return mode_->mission_controller_->holding(); }
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

TEST_F(NavigationModeProgressionTest,
       ExactSphereEntryReserveFalseReplacementBeforeTimerBlocksProgression) {
  // Map conceptual entry=10.000 s and MAIN end=10.600 s onto fixture clocks.
  // These callback phases are synthetic schedules, not measured flight bounds.
  constexpr auto entry_ns = kCrossing;
  constexpr auto main_end_ns = entry_ns + 600'000'000;
  const Eigen::Vector3d velocity =
      3.49 * Eigen::Vector3d{20.0, 5.0, 0.0}.normalized();
  setNow(entry_ns - 10'000'000);
  health();
  odometry(syntheticSphereEntryPosition(now_ns_, entry_ns), 0, velocity);
  const auto before_entry = commandFromCanonicalMainReserve(main_end_ns, entry_ns);
  ASSERT_TRUE(before_entry->certified_main_continuation);
  ASSERT_EQ(before_entry->continuation_boundary_stamp_ns,
            static_cast<std::uint64_t>(entry_ns));
  admit(before_entry);
  ASSERT_EQ(acceptedSample(), before_entry->sample_id);
  ASSERT_EQ(waypoint(), 1U);

  setNow(entry_ns + 10'000'000);
  health();
  odometry(syntheticSphereEntryPosition(now_ns_, entry_ns), 0, velocity);
  const auto after_entry = commandFromCanonicalMainReserve(main_end_ns, entry_ns);
  ASSERT_FALSE(after_entry->certified_main_continuation);
  ASSERT_EQ(after_entry->continuation_boundary_stamp_ns, 0U);
  admit(after_entry);
  ASSERT_EQ(acceptedSample(), after_entry->sample_id);

  setNow(entry_ns + 25'000'000);
  health();
  odometry(syntheticSphereEntryPosition(now_ns_, entry_ns), 0, velocity);
  timerTick();
  EXPECT_EQ(waypoint(), 1U);
  EXPECT_EQ(request(), 2U);
  EXPECT_FALSE(failed());
  EXPECT_EQ(hold_count_, 0U);
}

TEST_F(NavigationModeProgressionTest,
       ExactSphereEntryReserveTimerBeforeFalseReplacementConsumesLease) {
  // Same canonical geometry/reserve as the paired test; only consumption order
  // changes. The receiver must not invent a 600 ms-at-use gate: its accepted
  // true witness remains consumable during the existing 100 ms command lease.
  constexpr auto entry_ns = kCrossing;
  constexpr auto main_end_ns = entry_ns + 600'000'000;
  const Eigen::Vector3d velocity =
      3.49 * Eigen::Vector3d{20.0, 5.0, 0.0}.normalized();
  setNow(entry_ns - 10'000'000);
  health();
  odometry(syntheticSphereEntryPosition(now_ns_, entry_ns), 0, velocity);
  const auto before_entry = commandFromCanonicalMainReserve(main_end_ns, entry_ns);
  ASSERT_TRUE(before_entry->certified_main_continuation);
  admit(before_entry);
  ASSERT_EQ(acceptedSample(), before_entry->sample_id);
  ASSERT_EQ(waypoint(), 1U);

  setNow(entry_ns + 10'000'000);
  health();
  odometry(syntheticSphereEntryPosition(now_ns_, entry_ns), 0, velocity);
  const auto command_stamp_ns = fixtureStampNanoseconds(before_entry->header.stamp);
  const auto valid_until_ns = fixtureStampNanoseconds(before_entry->valid_until);
  ASSERT_GT(command_stamp_ns, 0);
  ASSERT_LE(command_stamp_ns, now_ns_);
  ASSERT_GT(valid_until_ns, command_stamp_ns);
  ASSERT_LE(now_ns_, valid_until_ns);
  ASSERT_LT(main_end_ns - now_ns_, 600'000'000);
  timerTick();
  ASSERT_EQ(waypoint(), 2U);
  ASSERT_EQ(request(), 3U);

  setNow(entry_ns + 25'000'000);
  health();
  odometry(syntheticSphereEntryPosition(now_ns_, entry_ns), 0, velocity);
  const auto after_entry = commandFromCanonicalMainReserve(main_end_ns, entry_ns);
  ASSERT_FALSE(after_entry->certified_main_continuation);
  admit(after_entry);  // A retained predecessor cannot advance the next goal.
  timerTick();
  EXPECT_EQ(waypoint(), 2U);
  EXPECT_EQ(request(), 3U);
  EXPECT_FALSE(failed());
  EXPECT_EQ(hold_count_, 0U);
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

TEST_F(NavigationModeProgressionTest, CharacterizeTerminalStopSpeedAcrossCallbacks) {
  startOutside();
  setNow(kCrossing);
  health();
  odometry(inBall());
  admit(command());
  ASSERT_EQ(waypoint(), 2U);
  ASSERT_EQ(request(), 3U);
  const auto start = kCrossing + 20'000'000;
  // Characterization, not approval of this timer policy: alternating speed
  // samples clear/re-arm recovery but cannot satisfy continuous STOP arrival.
  for (int step = 0; step <= 1200; ++step) {
    setNow(start + static_cast<std::int64_t>(step) * 10'000'000);
    if (step % 2 == 0) {
      health();
      const double speed = (step / 20) % 2 == 0 ? 0.16 : 0.14;
      odometry({50.0, 5.0, 3.0}, 0, {speed, 0.0, 0.0});
      admit(completedStopCommand());
    }
    if (step % 5 == 0) timerTick();
    EXPECT_FALSE(failed());
    EXPECT_EQ(hold_count_, 0U);
    EXPECT_FALSE(missionCompleted());
    EXPECT_EQ(waypoint(), 2U);
  }
}

TEST_F(NavigationModeProgressionTest, CompletedBackupHoldAcceptsLateMeasuredSuffixStop) {
  startOutside();
  setNow(kCrossing);
  health();
  odometry({20.99, 5.24, 3.0}, 0, {0.8, 0.0, 0.0});
  auto endpoint = command(false);
  endpoint->position.x = 20.863;
  endpoint->position.y = 5.096;
  endpoint->position.z = 3.0;
  endpoint->velocity = geometry_msgs::msg::Vector3{};
  endpoint->role = Command::ROLE_BACKUP;
  endpoint->status = Command::STATUS_COMPLETED;
  admit(endpoint);
  timerTick();
  ASSERT_EQ(waypoint(), 1U);

  setNow(kCrossing + 1'000'000'000);
  health();
  odometry({20.80, 5.08, 3.0}, 0, {0.1, 0.0, 0.0});
  auto hold = command(false);
  hold->position = endpoint->position;
  hold->velocity = geometry_msgs::msg::Vector3{};
  hold->role = Command::ROLE_BACKUP;
  hold->status = Command::STATUS_COMPLETED;
  admit(hold);
  EXPECT_EQ(acceptedSample(), hold->sample_id);
  timerTick();
  EXPECT_EQ(acceptedSample(), 0U);
  EXPECT_EQ(waypoint(), 2U);
  EXPECT_EQ(request(), 3U);
  EXPECT_FALSE(failed());
  EXPECT_EQ(hold_count_, 0U);
  // A fresh sample ID does not renew permission for the next waypoint.
  setNow(kCrossing + 1'020'000'000);
  health();
  odometry({20.80, 5.08, 3.0}, 0, {0.1, 0.0, 0.0});
  auto predecessor = command(false);
  predecessor->position = endpoint->position;
  predecessor->velocity = geometry_msgs::msg::Vector3{};
  predecessor->role = Command::ROLE_BACKUP;
  predecessor->status = Command::STATUS_COMPLETED;
  admit(predecessor);
  timerTick();
  EXPECT_NE(acceptedSample(), predecessor->sample_id);
  EXPECT_EQ(waypoint(), 2U);
  EXPECT_EQ(request(), 3U);
}

TEST_F(NavigationModeProgressionTest, CompletedMainHoldDoesNotInventSuffixPermission) {
  startOutside();
  setNow(kCrossing);
  health();
  odometry({20.80, 5.08, 3.0}, 0, {0.1, 0.0, 0.0});
  auto hold = command(false);
  hold->velocity = geometry_msgs::msg::Vector3{};
  hold->status = Command::STATUS_COMPLETED;
  admit(hold);
  timerTick();
  EXPECT_EQ(acceptedSample(), hold->sample_id);
  EXPECT_EQ(waypoint(), 1U);
  EXPECT_EQ(request(), 2U);
  EXPECT_FALSE(failed());
  EXPECT_EQ(hold_count_, 0U);
}

TEST_F(NavigationModeProgressionTest, CompletedBackupHoldDoesNotAcceptMovingMeasuredState) {
  startOutside();
  setNow(kCrossing);
  health();
  odometry({20.80, 5.08, 3.0}, 0, {0.16, 0.0, 0.0});
  auto hold = command(false);
  hold->velocity = geometry_msgs::msg::Vector3{};
  hold->role = Command::ROLE_BACKUP;
  hold->status = Command::STATUS_COMPLETED;
  admit(hold);
  timerTick();
  EXPECT_EQ(acceptedSample(), hold->sample_id);
  EXPECT_EQ(waypoint(), 1U);
  EXPECT_EQ(request(), 2U);
  EXPECT_FALSE(failed());
}

TEST_F(NavigationModeProgressionTest, TerminalStopStillRequiresContinuousConfirmation) {
  startOutside();
  setNow(kCrossing);
  health();
  odometry(inBall());
  admit(command());
  ASSERT_EQ(waypoint(), 2U);
  const auto start = kCrossing + 20'000'000;
  for (int step = 0; step <= 60; ++step) {
    setNow(start + static_cast<std::int64_t>(step) * 10'000'000);
    if (step % 2 == 0) {
      health();
      odometry({50.0, 5.0, 3.0}, 0, {0.14, 0.0, 0.0});
      admit(completedStopCommand());
    }
    if (step % 5 == 0) timerTick();
    if (step < 50) {
      EXPECT_FALSE(holding());
      EXPECT_FALSE(missionCompleted());
    }
  }
  EXPECT_TRUE(holding() || missionCompleted());
  EXPECT_FALSE(failed());
  EXPECT_EQ(hold_count_, 0U);
}

}  // namespace px4_navigation_external_mode
