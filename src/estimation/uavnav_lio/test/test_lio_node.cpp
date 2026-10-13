// The lio node in process (SYSTEM_DESIGN §3.5, §6.1, §6.4, §6.5; D33/O20, O21; Review Focus 4).
//
// Conventions:
// - Synthetic data only (room_scene.hpp: a stationary IMU in a 10 m box room), published by a peer node on
//   /lidar/imu (200 Hz) and /lidar/points (10 Hz, every 20th IMU stamp, after that IMU sample, as in SITL).
// - Time: use_sim_time is false, so the node's ROS time is the system time. The stamps follow a simulated 5 ms
//   schedule that starts 1 s before the ROS time at the start of the stream, and the samples are published at
//   real-time pace (sleep_until), so a stamp stays about 1 s behind ROS now and never comes near the
//   kMaxStampAhead (0.5 s) refusal. Real-time pace also keeps the backend's ICP and the depth-5 sensor QoS in
//   their SITL regime (no burst larger than the queue).
// - Every wait is bounded at 60 s (Eventually, Spinner::Stop); shutdown() runs under std::async with a 10 s
//   bound. No assertion on wall-clock durations below 1 s.
// - The rclcpp context comes from ros_test_main.cpp (per-process DDS domain, localhost); CMake serialises the
//   in-process ROS tests with RESOURCE_LOCK ros_dds.
#include <gtest/gtest.h>

#include <unistd.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <future>
#include <limits>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <string_view>
#include <system_error>
#include <thread>
#include <vector>

#include "cloud_msg.hpp"
#include "rclcpp/rclcpp.hpp"
#include "room_scene.hpp"
#include "uavnav/core/config.hpp"
#include "uavnav/lio/config.hpp"
#include "uavnav/lio_node/lio_node.hpp"
#include "uavnav/lio_node/node_config.hpp"
#include "uavnav/ros/qos.hpp"
#include "uavnav_interfaces/msg/event_record.hpp"
#include "uavnav_interfaces/msg/lio_health.hpp"
#include "uavnav_interfaces/msg/lio_odometry.hpp"
#include "uavnav_interfaces/msg/lio_state.hpp"

using namespace uavnav;
using namespace uavnav::lio_node;
using namespace std::chrono_literals;

namespace {

using LioStateMsg = uavnav_interfaces::msg::LioState;
using LioHealthMsg = uavnav_interfaces::msg::LioHealth;
using LioOdometryMsg = uavnav_interfaces::msg::LioOdometry;
using EventMsg = uavnav_interfaces::msg::EventRecord;

constexpr auto kRosWait = 60s;
constexpr std::int64_t kImuPeriodNs = room::kImuPeriod.ns;
constexpr int kImuPerScan = 20;
constexpr int kInfEvery = 10;  // every 10th cloud point is a Gazebo no-return (inf)

template <class Pred>
bool Eventually(Pred pred, std::chrono::steady_clock::time_point deadline) {
  while (!pred()) {
    if (std::chrono::steady_clock::now() > deadline) return pred();
    std::this_thread::sleep_for(10ms);
  }
  return true;
}

std::filesystem::path TempJsonl(const std::string& name) {
  const auto dir = std::filesystem::temp_directory_path() / ("uavnav_lio_node_test_" + std::to_string(::getpid()));
  std::filesystem::create_directories(dir);
  const auto file = dir / (name + ".jsonl");
  std::filesystem::remove(file);
  return file;
}

rclcpp::NodeOptions Options(const std::string& config, const std::string& node_config, const std::string& events) {
  rclcpp::NodeOptions o;
  o.parameter_overrides({rclcpp::Parameter(std::string(kConfigFileParam), config),
                         rclcpp::Parameter(std::string(kNodeConfigFileParam), node_config),
                         rclcpp::Parameter(std::string(kEventsJsonlParam), events)});
  return o;
}

rclcpp::NodeOptions SimOptions(const std::filesystem::path& events) {
  return Options(UAVNAV_LIO_DEFAULT_YAML, UAVNAV_LIO_NODE_YAML, events.string());
}

std::vector<std::string> ReadLines(const std::filesystem::path& file) {
  std::ifstream in(file);
  std::vector<std::string> lines;
  for (std::string line; std::getline(in, line);) lines.push_back(line);
  return lines;
}

std::string EventKey(std::string_view name) { return "\"event\":\"" + std::string(name) + "\""; }

std::size_t CountEvent(const std::vector<std::string>& lines, std::string_view name) {
  const std::string key = EventKey(name);
  return static_cast<std::size_t>(
      std::count_if(lines.begin(), lines.end(), [&](const std::string& l) { return l.find(key) != std::string::npos; }));
}

/// The room scan once (the scene is static), with every kInfEvery-th point replaced by an inf no-return.
sensor_msgs::msg::PointCloud2 RoomCloud(int* inf_points) {
  const auto values = config::load_params_file(UAVNAV_LIO_DEFAULT_YAML, lio::kLioSpecs);
  EXPECT_TRUE(values.has_value());
  const auto cfg = lio::load_lio_config(*values);
  EXPECT_TRUE(cfg.has_value());
  const lio::ScanInput scan = room::scan(time::SensorTime{1}, imu_T_lidar(*cfg));
  std::vector<Eigen::Vector3f> pts;
  *inf_points = 0;
  for (std::size_t i = 0; i < scan.points.size(); ++i) {
    if (i % kInfEvery == 0) {
      pts.emplace_back(std::numeric_limits<float>::infinity(), 0.0F, 0.0F);
      ++*inf_points;
    }
    pts.push_back(scan.points[i].position_lidar_m);
  }
  return testmsg::Cloud(pts, 0);
}

/// Spins one executor on its own thread; Stop() cancels until spin() has returned (a cancel() that comes before
/// spin() starts would be lost, so it is repeated), bounded at 60 s.
class Spinner {
 public:
  Spinner(std::shared_ptr<rclcpp::Executor> exec, const rclcpp::Node::SharedPtr& node) : exec_(std::move(exec)) {
    exec_->add_node(node);
    done_ = std::async(std::launch::async, [e = exec_] { e->spin(); });
  }
  ~Spinner() { Stop(); }
  Spinner(const Spinner&) = delete;
  Spinner& operator=(const Spinner&) = delete;

  void Stop() {
    if (!done_.valid()) return;
    const auto deadline = std::chrono::steady_clock::now() + kRosWait;
    while (done_.wait_for(10ms) != std::future_status::ready) {
      exec_->cancel();
      if (std::chrono::steady_clock::now() > deadline) {
        ADD_FAILURE() << "an executor did not stop within 60 s: aborting instead of hanging";
        std::abort();
      }
    }
    done_.get();
  }

 private:
  std::shared_ptr<rclcpp::Executor> exec_;
  std::future<void> done_;
};

/// The other side of the node: publishes the sensor streams, records every output topic.
class Peer {
 public:
  Peer() : node_(std::make_shared<rclcpp::Node>("lio_node_test_peer")) {
    imu_pub_ = node_->create_publisher<sensor_msgs::msg::Imu>("/lidar/imu", rosutil::qos::sensor_input());
    points_pub_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>("/lidar/points", rosutil::qos::sensor_input());
    const auto best_effort = rclcpp::QoS(rclcpp::KeepLast(5000)).best_effort();
    const auto reliable = rclcpp::QoS(rclcpp::KeepLast(5000)).reliable();
    state_sub_ = node_->create_subscription<LioStateMsg>("/lio/state", best_effort, [this](const LioStateMsg& m) {
      const std::scoped_lock lock(mutex_);
      states_.push_back(m);
    });
    health_sub_ = node_->create_subscription<LioHealthMsg>("/lio/health", reliable, [this](const LioHealthMsg& m) {
      const std::scoped_lock lock(mutex_);
      health_.push_back(m);
    });
    odom_sub_ = node_->create_subscription<LioOdometryMsg>("/lio/odometry", reliable, [this](const LioOdometryMsg& m) {
      const std::scoped_lock lock(mutex_);
      odometry_.push_back(m);
    });
    events_sub_ = node_->create_subscription<EventMsg>("/events", reliable, [this](const EventMsg& m) {
      const std::scoped_lock lock(mutex_);
      events_.push_back(m.event);
    });
    spinner_ = std::make_unique<Spinner>(std::make_shared<rclcpp::executors::SingleThreadedExecutor>(), node_);
  }
  ~Peer() { spinner_.reset(); }

  /// Every topic matched with the node under test (bounded).
  bool WaitMatched() {
    return Eventually(
        [&] {
          return imu_pub_->get_subscription_count() > 0 && points_pub_->get_subscription_count() > 0 &&
                 state_sub_->get_publisher_count() > 0 && health_sub_->get_publisher_count() > 0 &&
                 odom_sub_->get_publisher_count() > 0 && events_sub_->get_publisher_count() > 0;
        },
        std::chrono::steady_clock::now() + kRosWait);
  }

  std::int64_t RosNowNs() const { return node_->now().nanoseconds(); }

  /// IMU samples k in [from, to) of the schedule base_ns + k * 5 ms, and a cloud after every 20th sample, stamped
  /// like it. `burst` samples are published back to back every burst * 5 ms (1: one every 5 ms); the average pace
  /// is real time either way.
  void Stream(std::int64_t base_ns, int from, int to, const sensor_msgs::msg::PointCloud2& cloud, int burst = 1) {
    const auto start = std::chrono::steady_clock::now();
    for (int k = from; k < to; ++k) {
      if ((k - from) % burst == 0) std::this_thread::sleep_until(start + (k - from) * std::chrono::nanoseconds(kImuPeriodNs));
      const std::int64_t stamp = base_ns + k * kImuPeriodNs;
      imu_pub_->publish(testmsg::Imu(stamp, Eigen::Vector3d::Zero(), Eigen::Vector3d(0.0, 0.0, room::kGravity)));
      if (k % kImuPerScan == kImuPerScan - 1) {
        sensor_msgs::msg::PointCloud2 c = cloud;
        c.header.stamp = testmsg::Stamp(stamp);
        points_pub_->publish(c);
      }
    }
  }

  std::vector<LioStateMsg> States() {
    const std::scoped_lock lock(mutex_);
    return states_;
  }
  bool HealthReached(std::uint8_t state) {
    const std::scoped_lock lock(mutex_);
    return std::any_of(health_.begin(), health_.end(), [&](const LioHealthMsg& h) { return h.state == state; });
  }
  std::size_t OdometryCount() {
    const std::scoped_lock lock(mutex_);
    return odometry_.size();
  }
  std::vector<LioOdometryMsg> Odometry() {
    const std::scoped_lock lock(mutex_);
    return odometry_;
  }
  bool EventSeen(std::string_view name) {
    const std::scoped_lock lock(mutex_);
    return std::find(events_.begin(), events_.end(), name) != events_.end();
  }

 private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr points_pub_;
  rclcpp::Subscription<LioStateMsg>::SharedPtr state_sub_;
  rclcpp::Subscription<LioHealthMsg>::SharedPtr health_sub_;
  rclcpp::Subscription<LioOdometryMsg>::SharedPtr odom_sub_;
  rclcpp::Subscription<EventMsg>::SharedPtr events_sub_;
  std::mutex mutex_;
  std::vector<LioStateMsg> states_;
  std::vector<LioHealthMsg> health_;
  std::vector<LioOdometryMsg> odometry_;
  std::vector<std::string> events_;
  std::unique_ptr<Spinner> spinner_;  // last: stopped first, before the subscriptions die
};

double StampSeconds(const builtin_interfaces::msg::Time& t) { return t.sec + 1e-9 * t.nanosec; }

/// Shuts the node down under a 10 s bound (std::async); aborts instead of hanging.
void BoundedShutdown(LioNode& node) {
  auto done = std::async(std::launch::async, [&node] { node.shutdown(); });
  if (done.wait_for(10s) != std::future_status::ready) {
    ADD_FAILURE() << "LioNode::shutdown() did not return within 10 s: aborting instead of hanging";
    std::abort();
  }
}

}  // namespace

TEST(LioNode, InProcessSyntheticRunPublishesAllTopics) {
  const auto deadline = std::chrono::steady_clock::now() + kRosWait;
  const auto jsonl = TempJsonl("synthetic");
  int inf_points = 0;
  const auto cloud = RoomCloud(&inf_points);
  Peer peer;
  auto node = std::make_shared<LioNode>(SimOptions(jsonl));
  Spinner spinner(std::make_shared<rclcpp::executors::SingleThreadedExecutor>(), node);
  ASSERT_TRUE(peer.WaitMatched()) << "discovery did not match the topics within 60 s";

  peer.Stream(peer.RosNowNs() - 1'000'000'000, 0, 2000, cloud);  // 10 s of sensor time
  EXPECT_TRUE(Eventually(
      [&] {
        return peer.HealthReached(LioHealthMsg::TRACKING) && peer.OdometryCount() >= 1 &&
               peer.EventSeen("StateTransition");
      },
      deadline))
      << "TRACKING on /lio/health, one /lio/odometry and a StateTransition on /events";
  spinner.Stop();
  BoundedShutdown(*node);

  // /lio/state: one sample per 10 ms of sensor time, in order, over the span from the first TRACKING sample.
  const auto states = peer.States();
  const auto first = std::find_if(states.begin(), states.end(),
                                  [](const LioStateMsg& s) { return s.lio_state == LioHealthMsg::TRACKING; });
  ASSERT_NE(first, states.end()) << "no /lio/state sample in TRACKING";
  const auto n = static_cast<double>(std::distance(first, states.end()));
  const double span_s = StampSeconds(states.back().stamp) - StampSeconds(first->stamp);
  ASSERT_GT(span_s, 3.0) << "TRACKING was reached late: " << span_s << " s of tracked /lio/state";
  EXPECT_NEAR((n - 1.0) / span_s, 100.0, 5.0) << n << " samples over " << span_s << " s";
  for (std::size_t i = 1; i < states.size(); ++i) {
    ASSERT_GT(StampSeconds(states[i].stamp), StampSeconds(states[i - 1].stamp)) << "state " << i;
    EXPECT_TRUE(std::isnan(states[i].pose_covariance[0]) && std::isnan(states[i].velocity_covariance[8]));
  }
  for (const auto& o : peer.Odometry()) {
    EXPECT_TRUE(std::isfinite(o.pose.position.x) && std::isfinite(o.pose_covariance[0]));
    EXPECT_LE(o.quality, 100U);
  }

  // The JSONL file carries the same events, and ends with the ingest's summary (stop() after the node's own).
  const auto lines = ReadLines(jsonl);
  ASSERT_FALSE(lines.empty());
  EXPECT_GE(CountEvent(lines, "StateTransition"), 1U);
  EXPECT_EQ(CountEvent(lines, "LioInputRejected"), 0U) << "no garbage, no future stamp in this stream";
  EXPECT_EQ(CountEvent(lines, "LioIngestReentered"), 0U);
  EXPECT_NE(lines.back().find(EventKey("IngestSummary")), std::string::npos) << lines.back();
  ASSERT_EQ(CountEvent(lines, "LioNodeSummary"), 1U);
  const auto summary = std::find_if(lines.begin(), lines.end(), [](const std::string& l) {
    return l.find(EventKey("LioNodeSummary")) != std::string::npos;
  });
  // The node counts the dropped no-returns: inf_points per converted scan (some scans may be lost on the wire,
  // so the expected total is derived from the node's own scan count).
  const std::string& s = *summary;
  const auto value = [&s](const std::string& key) {
    const auto at = s.find("\"" + key + "\":");
    EXPECT_NE(at, std::string::npos) << key;
    return at == std::string::npos ? -1.0 : std::stod(s.substr(at + key.size() + 3));
  };
  const double scans = value("scans_converted");
  EXPECT_GE(scans, 80.0) << "most of the 100 clouds arrive";
  EXPECT_EQ(value("points_non_finite"), scans * inf_points);
  EXPECT_EQ(value("rejected_no_ros_time"), 0.0);
  if (!HasFailure()) std::filesystem::remove(jsonl);
}

// Review Focus 4.
TEST(LioNode, MultiThreadedExecutorKeepsCallbacksSerialised) {
  const auto jsonl = TempJsonl("multithreaded");
  int inf_points = 0;
  const auto cloud = RoomCloud(&inf_points);
  Peer peer;
  auto node = std::make_shared<LioNode>(SimOptions(jsonl));
  Spinner spinner(std::make_shared<rclcpp::executors::MultiThreadedExecutor>(rclcpp::ExecutorOptions(), 3), node);
  ASSERT_TRUE(peer.WaitMatched());

  // 5 s, in bursts of 4 IMU samples (and the scan of every 5th burst) every 20 ms: several messages wait at once,
  // so three executor threads would take them concurrently if the callback group allowed it.
  peer.Stream(peer.RosNowNs() - 1'000'000'000, 0, 1000, cloud, 4);
  EXPECT_TRUE(Eventually([&] { return peer.States().size() >= 100; }, std::chrono::steady_clock::now() + kRosWait));
  spinner.Stop();
  BoundedShutdown(*node);

  const auto lines = ReadLines(jsonl);
  ASSERT_FALSE(lines.empty());
  EXPECT_EQ(CountEvent(lines, "LioIngestReentered"), 0U) << "one MutuallyExclusive group serialises the callbacks";
  EXPECT_NE(lines.back().find(EventKey("IngestSummary")), std::string::npos);
  EXPECT_NE(lines.back().find("\"reentered\":0"), std::string::npos) << lines.back();
  const auto states = peer.States();
  for (std::size_t i = 1; i < states.size(); ++i) {
    ASSERT_GT(StampSeconds(states[i].stamp), StampSeconds(states[i - 1].stamp)) << "IMU states in order, " << i;
  }
  if (!HasFailure()) std::filesystem::remove(jsonl);
}

TEST(LioNode, ShutdownJoinsTheBackendThenFlushesEvents) {
  const auto jsonl = TempJsonl("shutdown");
  int inf_points = 0;
  const auto cloud = RoomCloud(&inf_points);
  Peer peer;
  auto node = std::make_shared<LioNode>(SimOptions(jsonl));
  Spinner spinner(std::make_shared<rclcpp::executors::SingleThreadedExecutor>(), node);
  ASSERT_TRUE(peer.WaitMatched());

  // The signal path: shutdown() while the executor still spins and messages still arrive. The node stops taking
  // callbacks first, then LioIngest::stop() joins the backend, then the recorder drains and is destroyed.
  const std::int64_t base = peer.RosNowNs() - 1'000'000'000;
  auto streaming = std::async(std::launch::async, [&] { peer.Stream(base, 0, 800, cloud); });  // 4 s
  std::this_thread::sleep_for(2500ms);  // past the IMU initialisation: the backend has scans in flight
  BoundedShutdown(*node);
  const auto after_shutdown = ReadLines(jsonl);
  ASSERT_EQ(streaming.wait_for(kRosWait), std::future_status::ready);
  std::this_thread::sleep_for(200ms);  // callbacks keep coming after shutdown: refused, nothing is written
  spinner.Stop();

  ASSERT_FALSE(after_shutdown.empty());
  EXPECT_EQ(CountEvent(after_shutdown, "IngestSummary"), 1U);
  EXPECT_NE(after_shutdown.back().find(EventKey("IngestSummary")), std::string::npos)
      << "the JSONL ends with IngestSummary: " << after_shutdown.back();
  ASSERT_GE(after_shutdown.size(), 2U);
  EXPECT_NE(after_shutdown[after_shutdown.size() - 2].find(EventKey("LioNodeSummary")), std::string::npos);
  EXPECT_GE(CountEvent(after_shutdown, "ImuInitialized"), 1U) << "the backend ran before the shutdown";

  BoundedShutdown(*node);  // idempotent
  node.reset();            // the destructor after shutdown() writes nothing more
  EXPECT_EQ(ReadLines(jsonl), after_shutdown);
  if (!HasFailure()) std::filesystem::remove(jsonl);
}

TEST(LioNode, MissingOrInvalidConfigRefusesToStart) {
  const auto jsonl = TempJsonl("refused");
  const auto refused = [&](const rclcpp::NodeOptions& options) -> std::optional<StartError> {
    try {
      LioNode node(options);
    } catch (const LioNodeStartFailed& e) {
      return e.reason();
    }
    return std::nullopt;
  };
  EXPECT_EQ(refused(Options("/nonexistent/lio.yaml", UAVNAV_LIO_NODE_YAML, jsonl.string())),
            StartError::kConfigInvalid);
  // The node table in the core's place: every core key is missing.
  EXPECT_EQ(refused(Options(UAVNAV_LIO_NODE_YAML, UAVNAV_LIO_NODE_YAML, jsonl.string())), StartError::kConfigInvalid);
  EXPECT_EQ(refused(Options(UAVNAV_LIO_DEFAULT_YAML, UAVNAV_LIO_DEFAULT_YAML, jsonl.string())),
            StartError::kNodeConfigInvalid);
  EXPECT_EQ(refused(Options(UAVNAV_LIO_DEFAULT_YAML, UAVNAV_LIO_NODE_YAML, "")), StartError::kParameterInvalid);
  EXPECT_EQ(refused(Options("", UAVNAV_LIO_NODE_YAML, jsonl.string())), StartError::kParameterInvalid);
  EXPECT_FALSE(std::filesystem::exists(jsonl)) << "refused before the recorder (and any thread) starts";
  EXPECT_EQ(refused(Options(UAVNAV_LIO_DEFAULT_YAML, UAVNAV_LIO_NODE_YAML, "/proc/uavnav_no_such_dir/e.jsonl")),
            StartError::kEventsFileFailed);
  std::error_code ignored;  // removes the directory only when empty (the passing tests removed their files)
  std::filesystem::remove(jsonl.parent_path(), ignored);
}
