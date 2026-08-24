#include <gtest/gtest.h>

#include <chrono>
#include <condition_variable>
#include <cstring>
#include <future>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

#include <unistd.h>

#include <rclcpp/executors/multi_threaded_executor.hpp>
#include <sensor_msgs/msg/point_field.hpp>

#include "navigation_runtime/super_navigation_node.hpp"

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

  void onShutdownComplete(ObservationAccounting::Snapshot lifecycle) noexcept override {
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

  std::optional<ObservationAccounting::Snapshot> shutdownSnapshot() const {
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
  std::optional<ObservationAccounting::Snapshot> shutdown_snapshot_;
  std::uint64_t shutdown_count_{0};
};

struct ObserverReleaseGuard {
  std::shared_ptr<BlockingLifecycleObserver> observer;
  ~ObserverReleaseGuard() { observer->release(); }
};

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

TEST(SuperNavigationShutdown, JoinsAnInflightRealMapUpdateBeforeDestruction) {
  auto context = std::make_shared<rclcpp::Context>();
  context->init(0, nullptr);
  auto observer = std::make_shared<BlockingLifecycleObserver>();

  const auto process_suffix = std::to_string(static_cast<long long>(::getpid()));
  const std::string prefix = "/shutdown_contract_" + process_suffix;
  rclcpp::NodeOptions options;
  options.context(context);
  options.parameter_overrides({
      rclcpp::Parameter("super_navigation.cloud_topic", prefix + "/cloud"),
      rclcpp::Parameter("super_navigation.corrected_odometry_topic", prefix + "/corrected"),
      rclcpp::Parameter("super_navigation.propagated_odometry_topic", prefix + "/propagated"),
      rclcpp::Parameter("super_navigation.goal_topic", prefix + "/goal"),
      rclcpp::Parameter("super_navigation.status_topic", prefix + "/status"),
      rclcpp::Parameter("super_navigation.command_topic", prefix + "/command"),
      rclcpp::Parameter("super_navigation.planning_frame", "lio_odom"),
      rclcpp::Parameter("super_navigation.deployment_profile", "sitl"),
      rclcpp::Parameter("super_navigation.planner_rate_hz", 0.2),
      rclcpp::Parameter("super_navigation.config_path", SUPER_PRODUCT_CONFIG_PATH),
  });

  auto navigation = std::make_shared<SuperNavigationNode>(
      options, SuperNavigationDependencies{observer});
  ObserverReleaseGuard release_guard{observer};
  auto driver = std::make_shared<rclcpp::Node>(
      "shutdown_contract_driver_" + process_suffix, options);
  const auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).best_effort().durability_volatile();
  auto corrected_publisher =
      driver->create_publisher<nav_msgs::msg::Odometry>(prefix + "/corrected", qos);
  auto cloud_publisher =
      driver->create_publisher<sensor_msgs::msg::PointCloud2>(prefix + "/cloud", qos);

  rclcpp::ExecutorOptions executor_options;
  executor_options.context = context;
  rclcpp::executors::MultiThreadedExecutor executor(executor_options, 2);
  executor.add_node(navigation);
  executor.add_node(driver);
  std::thread spin_thread([&executor] { executor.spin(); });
  ExecutorStopGuard executor_guard(executor, spin_thread);

  const auto discovery_deadline = std::chrono::steady_clock::now() + 5s;
  while ((corrected_publisher->get_subscription_count() == 0U ||
          cloud_publisher->get_subscription_count() == 0U) &&
         std::chrono::steady_clock::now() < discovery_deadline) {
    std::this_thread::sleep_for(10ms);
  }
  ASSERT_GT(corrected_publisher->get_subscription_count(), 0U);
  ASSERT_GT(cloud_publisher->get_subscription_count(), 0U);

  const builtin_interfaces::msg::Time stamp = driver->now();
  ASSERT_GT(rclcpp::Time(stamp).nanoseconds(), 0);
  nav_msgs::msg::Odometry corrected;
  corrected.header.stamp = stamp;
  corrected.header.frame_id = "lio_odom";
  corrected.child_frame_id = "base_link";
  corrected.pose.pose.orientation.w = 1.0;
  corrected_publisher->publish(corrected);
  cloud_publisher->publish(makeCloud(stamp));
  ASSERT_TRUE(observer->waitForMapUpdate(5s));

  executor_guard.stop();
  executor.remove_node(navigation);
  executor.remove_node(driver);

  std::weak_ptr<SuperNavigationNode> weak_navigation = navigation;
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
