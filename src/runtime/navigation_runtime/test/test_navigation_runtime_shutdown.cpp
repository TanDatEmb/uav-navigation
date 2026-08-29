#include <gtest/gtest.h>

#include <chrono>
#include <condition_variable>
#include <cstring>
#include <cstdlib>
#include <future>
#include <filesystem>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

#include <unistd.h>

#include <navigation_contracts/msg/registered_scan.hpp>
#include <rclcpp/executors/multi_threaded_executor.hpp>
#include <sensor_msgs/msg/point_field.hpp>

#include "navigation_runtime/navigation_runtime_node.hpp"
#include "navigation_runtime/runtime_boundaries.hpp"

namespace navigation_runtime {
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

  bool waitForMapUpdate(std::chrono::seconds timeout) {
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

TEST(NavigationRuntimeCadence, RejectsPlannerPeriodShorterThanSolveBudget) {
  auto context = std::make_shared<rclcpp::Context>();
  context->init(0, nullptr);
  rclcpp::NodeOptions options;
  options.context(context);
  options.parameter_overrides({
      rclcpp::Parameter("navigation_runtime.planning_frame", "lio_odom"),
      rclcpp::Parameter("navigation_runtime.deployment_profile", "sitl"),
      rclcpp::Parameter("navigation_runtime.planner_rate_hz", 10.0),
      rclcpp::Parameter("navigation_runtime.config_path", NAVIGATION_PLANNER_CONFIG_PATH),
  });

  EXPECT_THROW(
      {
        auto navigation = std::make_shared<NavigationRuntimeNode>(options);
      },
      std::invalid_argument);
  context->shutdown("cadence contract test complete");
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
  observation.points = makeCloud(stamp);
  return observation;
}

TEST(NavigationRuntimeShutdown, JoinsAnInflightRealMapUpdateBeforeDestruction) {
  const std::filesystem::path ros_log_directory =
      "/tmp/uav-navigation-test-ros-logs";
  std::filesystem::create_directories(ros_log_directory);
  ASSERT_EQ(setenv("ROS_LOG_DIR", ros_log_directory.c_str(), 1), 0);
  auto context = std::make_shared<rclcpp::Context>();
  context->init(0, nullptr);
  auto observer = std::make_shared<BlockingLifecycleObserver>();

  const auto process_suffix = std::to_string(static_cast<long long>(::getpid()));
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
      rclcpp::Parameter("navigation_runtime.planner_rate_hz", 0.2),
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

  const auto discovery_deadline = std::chrono::steady_clock::now() + 5s;
  while (observation_publisher->get_subscription_count() == 0U &&
         std::chrono::steady_clock::now() < discovery_deadline) {
    std::this_thread::sleep_for(10ms);
  }
  ASSERT_GT(observation_publisher->get_subscription_count(), 0U);

  const builtin_interfaces::msg::Time stamp = driver->now();
  ASSERT_GT(rclcpp::Time(stamp).nanoseconds(), 0);
  observation_publisher->publish(makeRegisteredScan(stamp));
  ASSERT_TRUE(observer->waitForMapUpdate(5s));

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
  EXPECT_EQ(lifecycle->received, 1U);
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
  context->shutdown("test complete");
}

}  // namespace
}  // namespace navigation_runtime
