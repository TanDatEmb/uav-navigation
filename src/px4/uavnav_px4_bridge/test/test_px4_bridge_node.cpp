// The px4_bridge node (SYSTEM_DESIGN §4.1, §6.1, §6.4, §6.5; D33/O19-O22; Review Focus 5).
//
// The node runs as a CHILD PROCESS (the px4_bridge_node executable), and this test is its ROS peer. The node class
// lives in src/px4_bridge_node.cpp, the only place besides ros_conversion.* where tools/uavnav/arch_rules.yaml lets
// ROS in, so a test cannot include it; running the executable also tests the real main (parameters, signal
// handling, exit code).
//
// Conventions:
// - Synthetic data only: the LIO and PX4 streams of a vehicle that moves slowly, consistent with the truth
//   T = (1, 2, 0, 0.3) (the world of test_bridge.cpp). PX4 samples at 50 Hz, one odometry every 100 ms published
//   ~73 ms after its stamp (the LIO scan-to-output delay), so every scan has PX4 samples on both sides.
// - Time: use_sim_time is false, so ROS time is the system time in both processes and the stamps are ROS now.
// - Isolation: main() puts the test and its child in a DDS domain derived from the pid (215..232), localhost only;
//   CMake serialises the in-process ROS tests with RESOURCE_LOCK ros_dds.
// - Every wait is bounded (60 s; a child that does not stop is killed and the test fails).
#include <gtest/gtest.h>

#include <signal.h>
#include <spawn.h>
#include <sys/wait.h>
#include <unistd.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <future>
#include <limits>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <string_view>
#include <thread>
#include <vector>

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include "px4_msgs/msg/vehicle_local_position.hpp"
#include "px4_msgs/msg/vehicle_odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "uavnav/ros/qos.hpp"
#include "uavnav_interfaces/msg/alignment.hpp"
#include "uavnav_interfaces/msg/lio_health.hpp"
#include "uavnav_interfaces/msg/lio_odometry.hpp"

extern char** environ;

using namespace std::chrono_literals;

namespace {

using AlignmentMsg = uavnav_interfaces::msg::Alignment;
using HealthMsg = uavnav_interfaces::msg::LioHealth;
using OdometryMsg = uavnav_interfaces::msg::LioOdometry;
using LocalPositionMsg = px4_msgs::msg::VehicleLocalPosition;
using VehicleOdometryMsg = px4_msgs::msg::VehicleOdometry;

constexpr auto kWait = 60s;
constexpr std::int64_t kPx4PeriodNs = 20'000'000;
constexpr std::int64_t kOdometryPeriodNs = 100'000'000;

constexpr const char* kLocalPositionTopic = "/fmu/out/vehicle_local_position_v1";
constexpr const char* kEvTopic = "/fmu/in/vehicle_visual_odometry";

template <class Pred>
bool Eventually(Pred pred, std::chrono::steady_clock::time_point deadline) {
  while (!pred()) {
    if (std::chrono::steady_clock::now() > deadline) return pred();
    std::this_thread::sleep_for(10ms);
  }
  return true;
}

std::filesystem::path TempJsonl(const std::string& name) {
  const auto dir = std::filesystem::temp_directory_path() / ("uavnav_px4_bridge_node_test_" + std::to_string(::getpid()));
  std::filesystem::create_directories(dir);
  const auto file = dir / (name + ".jsonl");
  std::filesystem::remove(file);
  return file;
}

std::vector<std::string> ReadLines(const std::filesystem::path& file) {
  std::ifstream in(file);
  std::vector<std::string> lines;
  for (std::string line; std::getline(in, line);) lines.push_back(line);
  return lines;
}

std::string EventKey(std::string_view name) { return "\"event\":\"" + std::string(name) + "\""; }

/// The lines of event `name` (and, when given, reason `reason`), in file order.
std::vector<std::string> EventLines(const std::vector<std::string>& lines, std::string_view name,
                                    std::string_view reason = {}) {
  std::vector<std::string> out;
  for (const std::string& l : lines) {
    if (l.find(EventKey(name)) == std::string::npos) continue;
    if (!reason.empty() && l.find("\"reason\":\"" + std::string(reason) + "\"") == std::string::npos) continue;
    out.push_back(l);
  }
  return out;
}

/// The number `key` of an event's values in a JSONL line; NaN (and a failure) when it is missing.
double JsonNumber(const std::string& line, const std::string& key) {
  const auto at = line.find("\"" + key + "\":");
  EXPECT_NE(at, std::string::npos) << key << " in " << line;
  return at == std::string::npos ? std::numeric_limits<double>::quiet_NaN() : std::stod(line.substr(at + key.size() + 3));
}

// --- the node under test, as a process ---------------------------------------------------------------------------

class NodeProcess {
 public:
  explicit NodeProcess(const std::vector<std::string>& ros_args) {
    std::vector<std::string> args = {UAVNAV_PX4_BRIDGE_NODE_EXE, "--ros-args"};
    args.insert(args.end(), ros_args.begin(), ros_args.end());
    std::vector<char*> argv;
    for (std::string& a : args) argv.push_back(a.data());
    argv.push_back(nullptr);
    const int rc = ::posix_spawn(&pid_, UAVNAV_PX4_BRIDGE_NODE_EXE, nullptr, nullptr, argv.data(), environ);
    EXPECT_EQ(rc, 0) << "cannot start " << UAVNAV_PX4_BRIDGE_NODE_EXE;
    if (rc != 0) pid_ = 0;
  }
  ~NodeProcess() { Kill(); }
  NodeProcess(const NodeProcess&) = delete;
  NodeProcess& operator=(const NodeProcess&) = delete;

  /// SIGINT, as Ctrl-C and `ros2 launch` do.
  void Interrupt() const {
    if (pid_ > 0) ::kill(pid_, SIGINT);
  }

  /// The exit code (128 + signal when killed by one), or nullopt when it did not exit in `limit`.
  std::optional<int> WaitExit(std::chrono::seconds limit) {
    if (pid_ <= 0) return std::nullopt;
    const auto deadline = std::chrono::steady_clock::now() + limit;
    for (;;) {
      int status = 0;
      const pid_t r = ::waitpid(pid_, &status, WNOHANG);
      if (r == pid_) {
        pid_ = 0;
        return WIFEXITED(status) ? WEXITSTATUS(status) : 128 + WTERMSIG(status);
      }
      if (std::chrono::steady_clock::now() > deadline) return std::nullopt;
      std::this_thread::sleep_for(20ms);
    }
  }

  /// SIGINT, then the exit code within 20 s; a child that stays is killed and the test fails.
  int Stop() {
    Interrupt();
    const auto code = WaitExit(20s);
    if (!code) {
      ADD_FAILURE() << "the node did not stop within 20 s of SIGINT";
      Kill();
      return -1;
    }
    return *code;
  }

 private:
  void Kill() {
    if (pid_ <= 0) return;
    ::kill(pid_, SIGKILL);
    int status = 0;
    ::waitpid(pid_, &status, 0);
    pid_ = 0;
  }

  pid_t pid_{0};
};

std::vector<std::string> NodeArgs(const std::filesystem::path& events, bool sim_time = false) {
  std::vector<std::string> a = {"-p", std::string("config_file:=") + UAVNAV_PX4_BRIDGE_SIM_YAML, "-p",
                                "events_jsonl:=" + events.string()};
  if (sim_time) {
    a.emplace_back("-p");
    a.emplace_back("use_sim_time:=true");
  }
  return a;
}

// --- the synthetic world -----------------------------------------------------------------------------------------

double Wrap(double a) { return std::atan2(std::sin(a), std::cos(a)); }

Eigen::Vector2d Rot(double yaw, const Eigen::Vector2d& v) {
  return {std::cos(yaw) * v.x() - std::sin(yaw) * v.y(), std::sin(yaw) * v.x() + std::cos(yaw) * v.y()};
}

struct World {
  // T: lio_odom FRD -> PX4 local NED (x, y, z, yaw).
  double tx{1.0}, ty{2.0}, tz{0.0}, tyaw{0.3};
  Eigen::Vector3d p0_flu{1.0, -2.0, 3.0};
  Eigen::Vector3d v_flu{0.5, 0.2, 0.0};
  double yaw_flu{0.4};
  std::uint32_t epoch{1};
  std::int64_t t0_ns{0};  // stamp origin: the position is p0 + v * (stamp - t0)

  Eigen::Vector3d PFlu(std::int64_t stamp_ns) const {
    return p0_flu + v_flu * (static_cast<double>(stamp_ns - t0_ns) * 1e-9);
  }
  Eigen::Vector3d PNed(std::int64_t stamp_ns) const {
    const Eigen::Vector3d frd = PFlu(stamp_ns).cwiseProduct(Eigen::Vector3d(1.0, -1.0, -1.0));
    const Eigen::Vector2d xy = Rot(tyaw, frd.head<2>()) + Eigen::Vector2d(tx, ty);
    return {xy.x(), xy.y(), frd.z() + tz};
  }
  static builtin_interfaces::msg::Time Stamp(std::int64_t ns) {
    builtin_interfaces::msg::Time t;
    t.sec = static_cast<std::int32_t>(ns / 1'000'000'000);
    t.nanosec = static_cast<std::uint32_t>(ns % 1'000'000'000);
    return t;
  }

  OdometryMsg Odometry(std::int64_t stamp_ns) const {
    OdometryMsg m;
    m.stamp = Stamp(stamp_ns);
    m.epoch = epoch;
    const Eigen::Vector3d p = PFlu(stamp_ns);
    m.pose.position.x = p.x();
    m.pose.position.y = p.y();
    m.pose.position.z = p.z();
    const Eigen::Quaterniond q(Eigen::AngleAxisd(yaw_flu, Eigen::Vector3d::UnitZ()));
    m.pose.orientation.w = q.w();
    m.pose.orientation.x = q.x();
    m.pose.orientation.y = q.y();
    m.pose.orientation.z = q.z();
    m.velocity.x = v_flu.x();
    m.velocity.y = v_flu.y();
    m.velocity.z = v_flu.z();
    for (std::size_t i = 0; i < 6; ++i) m.pose_covariance[i * 6 + i] = 1e-4;
    for (std::size_t i = 0; i < 3; ++i) m.velocity_covariance[i * 3 + i] = 1e-4;
    m.quality = 80;
    return m;
  }

  HealthMsg Health(std::int64_t stamp_ns, std::uint8_t state) const {
    HealthMsg m;
    m.stamp = Stamp(stamp_ns);
    m.state = state;
    m.epoch = epoch;
    return m;
  }

  LocalPositionMsg LocalPosition(std::int64_t stamp_ns) const {
    LocalPositionMsg m;
    const Eigen::Vector3d p = PNed(stamp_ns);
    m.timestamp = static_cast<std::uint64_t>(stamp_ns / 1000);
    m.timestamp_sample = m.timestamp;
    m.xy_valid = true;
    m.z_valid = true;
    m.x = static_cast<float>(p.x());
    m.y = static_cast<float>(p.y());
    m.z = static_cast<float>(p.z());
    m.heading = static_cast<float>(Wrap(tyaw - yaw_flu));
    return m;
  }
};

// --- spinning and the peer ---------------------------------------------------------------------------------------

/// Spins one executor on its own thread; Stop() cancels until spin() has returned (a cancel() that comes before
/// spin() starts would be lost, so it is repeated), bounded at 60 s.
class Spinner {
 public:
  explicit Spinner(const rclcpp::Node::SharedPtr& node) : exec_(std::make_shared<rclcpp::executors::SingleThreadedExecutor>()) {
    exec_->add_node(node);
    done_ = std::async(std::launch::async, [e = exec_] { e->spin(); });
  }
  ~Spinner() { Stop(); }
  Spinner(const Spinner&) = delete;
  Spinner& operator=(const Spinner&) = delete;

  void Stop() {
    if (!done_.valid()) return;
    const auto deadline = std::chrono::steady_clock::now() + kWait;
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
  std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> exec_;
  std::future<void> done_;
};

/// The other side of the node: publishes the LIO and PX4 streams, records /alignment and the EV.
class Peer {
 public:
  Peer() : node_(std::make_shared<rclcpp::Node>("px4_bridge_node_test_peer")) {
    odometry_pub_ = node_->create_publisher<OdometryMsg>("/lio/odometry", uavnav::rosutil::qos::reliable_output());
    health_pub_ = node_->create_publisher<HealthMsg>("/lio/health", uavnav::rosutil::qos::reliable_output());
    local_position_pub_ = node_->create_publisher<LocalPositionMsg>(kLocalPositionTopic, uavnav::rosutil::qos::px4());
    alignment_sub_ = node_->create_subscription<AlignmentMsg>(
        "/alignment", uavnav::rosutil::qos::reliable_output(), [this](const AlignmentMsg& m) {
          const std::scoped_lock lock(mutex_);
          alignments_.push_back(m);
        });
    ev_sub_ = node_->create_subscription<VehicleOdometryMsg>(
        kEvTopic, uavnav::rosutil::qos::px4(), [this](const VehicleOdometryMsg& m) {
          const std::scoped_lock lock(mutex_);
          ev_.push_back(m);
        });
    spinner_ = std::make_unique<Spinner>(node_);
  }
  ~Peer() { spinner_.reset(); }

  /// Every topic matched with the node under test (bounded). `outputs` false: only the inputs (the node publishes
  /// nothing before its first tick, which needs ROS time).
  bool WaitMatched(bool outputs = true) {
    return Eventually(
        [&] {
          return odometry_pub_->get_subscription_count() > 0 && health_pub_->get_subscription_count() > 0 &&
                 local_position_pub_->get_subscription_count() > 0 &&
                 (!outputs || (alignment_sub_->get_publisher_count() > 0 && ev_sub_->get_publisher_count() > 0));
        },
        std::chrono::steady_clock::now() + kWait);
  }

  std::int64_t RosNowNs() const { return node_->now().nanoseconds(); }
  void Publish(const OdometryMsg& m) { odometry_pub_->publish(m); }
  void Publish(const HealthMsg& m) { health_pub_->publish(m); }
  void Publish(const LocalPositionMsg& m) { local_position_pub_->publish(m); }

  /// 20 ms steps k in [0, steps) from `world.t0_ns` at real-time pace: a PX4 sample stamped t0 + k * 20 ms every
  /// step; every 5th step (from the 4th) the odometry stamped 7 ms after a multiple of 100 ms and a TRACKING health
  /// stamped like the PX4 sample, i.e. an odometry ~73 ms after its stamp. Returns the number of odometry sent.
  int Stream(const World& world, int steps) {
    const auto start = std::chrono::steady_clock::now();
    int odometry = 0;
    for (int k = 0; k < steps; ++k) {
      std::this_thread::sleep_until(start + k * std::chrono::nanoseconds(kPx4PeriodNs));
      const std::int64_t stamp = world.t0_ns + k * kPx4PeriodNs;
      Publish(world.LocalPosition(stamp));
      if (k >= 4 && (k - 4) % 5 == 0) {
        const std::int64_t scan = world.t0_ns + ((k - 4) / 5) * kOdometryPeriodNs + 7'000'000;
        Publish(world.Health(stamp, HealthMsg::TRACKING));
        Publish(world.Odometry(scan));
        ++odometry;
      }
    }
    return odometry;
  }

  std::vector<AlignmentMsg> Alignments() {
    const std::scoped_lock lock(mutex_);
    return alignments_;
  }
  std::vector<VehicleOdometryMsg> Ev() {
    const std::scoped_lock lock(mutex_);
    return ev_;
  }

 private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<OdometryMsg>::SharedPtr odometry_pub_;
  rclcpp::Publisher<HealthMsg>::SharedPtr health_pub_;
  rclcpp::Publisher<LocalPositionMsg>::SharedPtr local_position_pub_;
  rclcpp::Subscription<AlignmentMsg>::SharedPtr alignment_sub_;
  rclcpp::Subscription<VehicleOdometryMsg>::SharedPtr ev_sub_;
  std::mutex mutex_;
  std::vector<AlignmentMsg> alignments_;
  std::vector<VehicleOdometryMsg> ev_;
  std::unique_ptr<Spinner> spinner_;  // last: stopped first, before the subscriptions die
};

}  // namespace

TEST(Px4BridgeNode, InProcessPublishesEvAndAlignment) {
  // (The plan's name; the node is a child process, see the file comment.)
  const auto deadline = std::chrono::steady_clock::now() + kWait;
  const auto jsonl = TempJsonl("synthetic");
  Peer peer;
  NodeProcess node(NodeArgs(jsonl));
  ASSERT_TRUE(peer.WaitMatched()) << "discovery did not match the topics within 60 s";

  World world;
  world.t0_ns = peer.RosNowNs();
  const int odometry_sent = peer.Stream(world, 250);  // 5 s of PX4 samples, 49 odometry
  ASSERT_GE(odometry_sent, 45);

  EXPECT_TRUE(Eventually(
      [&] {
        const auto a = peer.Alignments();
        return !a.empty() && a.back().state == AlignmentMsg::VALID;
      },
      deadline))
      << "/alignment VALID";
  const auto alignments = peer.Alignments();
  ASSERT_FALSE(alignments.empty());
  const AlignmentMsg& last = alignments.back();
  EXPECT_EQ(last.state, AlignmentMsg::VALID);
  EXPECT_NEAR(last.x_m, 1.0, 0.05);
  EXPECT_NEAR(last.y_m, 2.0, 0.05);
  EXPECT_NEAR(last.z_m, 0.0, 0.05);
  EXPECT_NEAR(last.yaw_rad, 0.3, 0.01);
  EXPECT_LT(last.residual_position_m, 0.05);

  // The EV: one per accepted odometry, FRD frames, the sample's own epoch, 10 Hz by its stamps.
  const auto ev = peer.Ev();
  ASSERT_GE(ev.size(), 40U) << "of " << odometry_sent << " odometry";
  EXPECT_LE(ev.size(), static_cast<std::size_t>(odometry_sent));
  for (std::size_t i = 1; i < ev.size(); ++i) {
    ASSERT_GT(ev[i].timestamp_sample, ev[i - 1].timestamp_sample) << "EV " << i;
  }
  const double span_s = static_cast<double>(ev.back().timestamp_sample - ev.front().timestamp_sample) * 1e-6;
  ASSERT_GT(span_s, 3.0);
  const double rate_hz = static_cast<double>(ev.size() - 1) / span_s;
  EXPECT_NEAR(rate_hz, 10.0, 1.0) << ev.size() << " EV over " << span_s << " s";
  for (const auto& m : ev) {
    EXPECT_EQ(m.pose_frame, VehicleOdometryMsg::POSE_FRAME_FRD);
    EXPECT_EQ(m.velocity_frame, VehicleOdometryMsg::VELOCITY_FRAME_FRD);
    EXPECT_EQ(m.reset_counter, 1) << "the epoch of the sample, never the output predictor's counter (F14)";
    for (const float w : m.angular_velocity) EXPECT_TRUE(std::isnan(w));
    EXPECT_GE(m.timestamp, m.timestamp_sample);
    EXPECT_GT(m.position_variance[0], 0.0F);
  }
  // The first EV: the odometry stamped t0 + 7 ms, FLU -> FRD.
  const std::int64_t first_stamp_ns = static_cast<std::int64_t>(ev.front().timestamp_sample) * 1000;
  const Eigen::Vector3d p = world.PFlu(first_stamp_ns);
  EXPECT_NEAR(ev.front().position[0], p.x(), 1e-3);
  EXPECT_NEAR(ev.front().position[1], -p.y(), 1e-3);
  EXPECT_NEAR(ev.front().position[2], -p.z(), 1e-3);

  EXPECT_EQ(node.Stop(), 0);
  const auto lines = ReadLines(jsonl);
  ASSERT_FALSE(lines.empty());
  EXPECT_FALSE(EventLines(lines, "AlignmentTransition", "PAIRS_CONSISTENT").empty());
  EXPECT_NE(lines.back().find(EventKey("BridgeNodeSummary")), std::string::npos)
      << "the JSONL file ends with the node summary: " << lines.back();
  EXPECT_GE(JsonNumber(lines.back(), "ev_published"), 40.0);
  EXPECT_GE(JsonNumber(lines.back(), "ticks"), 30.0);
  EXPECT_EQ(JsonNumber(lines.back(), "publish_failed"), 0.0);
}

TEST(Px4BridgeNode, GarbageMessagesAreRefusedWithEventsAndChangeNothing) {  // Review Focus 5
  const auto jsonl = TempJsonl("garbage");
  Peer peer;
  NodeProcess node(NodeArgs(jsonl));
  ASSERT_TRUE(peer.WaitMatched());
  ASSERT_TRUE(Eventually([&] { return !peer.Alignments().empty(); }, std::chrono::steady_clock::now() + kWait));

  World world;
  world.t0_ns = peer.RosNowNs();
  OdometryMsg bad_odometry = world.Odometry(world.t0_ns);
  bad_odometry.stamp.nanosec = 1'000'000'000;
  HealthMsg lost_looking = world.Health(world.t0_ns, 9);  // an unknown state code, not "LOST"
  LocalPositionMsg overflow = world.LocalPosition(world.t0_ns);
  overflow.timestamp_sample = std::numeric_limits<std::uint64_t>::max();
  LocalPositionMsg nan_heading = world.LocalPosition(world.t0_ns);
  nan_heading.heading = std::numeric_limits<float>::quiet_NaN();
  for (int i = 0; i < 3; ++i) {
    peer.Publish(bad_odometry);
    peer.Publish(lost_looking);
    peer.Publish(overflow);
    peer.Publish(nan_heading);
    std::this_thread::sleep_for(50ms);
  }
  std::this_thread::sleep_for(1s);  // the node takes them; nothing is observable on the topics

  EXPECT_EQ(node.Stop(), 0);
  const auto lines = ReadLines(jsonl);
  const auto odometry = EventLines(lines, "LioInputRejected", "NOT_FINITE");
  ASSERT_FALSE(odometry.empty());
  const auto stream = [](const std::string& l) { return JsonNumber(l, "stream"); };
  EXPECT_TRUE(std::any_of(odometry.begin(), odometry.end(), [&](const std::string& l) { return stream(l) == 0.0; }))
      << "odometry (stream 0)";
  EXPECT_TRUE(std::any_of(odometry.begin(), odometry.end(), [&](const std::string& l) { return stream(l) == 1.0; }))
      << "health (stream 1)";
  EXPECT_FALSE(EventLines(lines, "Px4SampleRejected", "NOT_FINITE").empty());
  // Nothing reached the bridge: no pair, no transition, no EV, T still INIT.
  EXPECT_TRUE(EventLines(lines, "AlignmentTransition").empty());
  EXPECT_TRUE(EventLines(lines, "AlignmentPairRejected").empty());
  EXPECT_TRUE(peer.Ev().empty());
  for (const auto& a : peer.Alignments()) EXPECT_EQ(a.state, AlignmentMsg::INIT);

  const std::string& summary = lines.back();
  ASSERT_NE(summary.find(EventKey("BridgeNodeSummary")), std::string::npos) << summary;
  EXPECT_EQ(JsonNumber(summary, "odometry_received"), 3.0);
  EXPECT_EQ(JsonNumber(summary, "health_received"), 3.0);
  EXPECT_EQ(JsonNumber(summary, "local_position_received"), 6.0);
  EXPECT_EQ(JsonNumber(summary, "rejected_odometry"), 3.0);
  EXPECT_EQ(JsonNumber(summary, "rejected_health"), 3.0);
  EXPECT_EQ(JsonNumber(summary, "rejected_local_position"), 6.0);
  EXPECT_EQ(JsonNumber(summary, "ev_published"), 0.0);
}

TEST(Px4BridgeNode, ShutdownFlushesOpenRejectionEpisodesBeforeTheSummary) {
  const auto jsonl = TempJsonl("flush");
  Peer peer;
  NodeProcess node(NodeArgs(jsonl));
  ASSERT_TRUE(peer.WaitMatched());
  ASSERT_TRUE(Eventually([&] { return !peer.Alignments().empty(); }, std::chrono::steady_clock::now() + kWait));

  // Odometry stamped 10 s ahead: refused kStampInFuture. The first of the episode is reported at once, the others
  // are only counted; no accepted odometry follows, so the episode is still open when the node is stopped.
  World world;
  world.t0_ns = peer.RosNowNs();
  for (int i = 0; i < 4; ++i) {
    peer.Publish(world.Odometry(peer.RosNowNs() + 10'000'000'000));
    std::this_thread::sleep_for(100ms);
  }
  std::this_thread::sleep_for(500ms);
  EXPECT_EQ(node.Stop(), 0);

  const auto lines = ReadLines(jsonl);
  const auto rejected = EventLines(lines, "LioInputRejected", "STAMP_IN_FUTURE");
  ASSERT_EQ(rejected.size(), 2U) << "the first of the episode, and the summary written by flush at shutdown";
  EXPECT_EQ(JsonNumber(rejected[0], "count"), 1.0);
  EXPECT_EQ(JsonNumber(rejected[1], "count"), 4.0);
  EXPECT_EQ(JsonNumber(rejected[1], "suppressed"), 3.0);
  EXPECT_EQ(JsonNumber(rejected[1], "summary"), 1.0);
  // Order at shutdown: the flushed summary, then the node summary, which closes the file.
  const auto at = [&](const std::string& what) {
    return std::find(lines.begin(), lines.end(), what) - lines.begin();
  };
  EXPECT_LT(at(rejected[1]), at(lines.back()));
  EXPECT_NE(lines.back().find(EventKey("BridgeNodeSummary")), std::string::npos) << lines.back();
}

TEST(Px4BridgeNode, NoRosTimeRefusesEveryMessageBeforeTheBridge) {
  // use_sim_time with no /clock: the node's ROS time stays 0, so no tick and no input may reach the bridge (a
  // stamp would be refused as "ahead" of a tick at 0, or the bridge's reference time would be 0).
  const auto jsonl = TempJsonl("no_ros_time");
  Peer peer;
  NodeProcess node(NodeArgs(jsonl, true));
  ASSERT_TRUE(peer.WaitMatched(false));

  World world;
  world.t0_ns = peer.RosNowNs();
  for (int i = 0; i < 3; ++i) {
    peer.Publish(world.Odometry(world.t0_ns));
    peer.Publish(world.Health(world.t0_ns, HealthMsg::TRACKING));
    peer.Publish(world.LocalPosition(world.t0_ns));
    std::this_thread::sleep_for(50ms);
  }
  std::this_thread::sleep_for(1s);
  EXPECT_EQ(node.Stop(), 0);

  EXPECT_TRUE(peer.Alignments().empty()) << "no tick without ROS time";
  EXPECT_TRUE(peer.Ev().empty());
  const auto lines = ReadLines(jsonl);
  const auto refused = EventLines(lines, "BridgeNoRosTime", "NO_ROS_TIME");
  ASSERT_FALSE(refused.empty());
  EXPECT_EQ(JsonNumber(refused[0], "count"), 1.0);
  const std::string& summary = lines.back();
  ASSERT_NE(summary.find(EventKey("BridgeNodeSummary")), std::string::npos) << summary;
  // Every message that arrived was refused (the best-effort PX4 stream may lose one to discovery, so count what arrived).
  const double received = JsonNumber(summary, "odometry_received") + JsonNumber(summary, "health_received") +
                          JsonNumber(summary, "local_position_received");
  EXPECT_GE(received, 6.0);
  EXPECT_EQ(JsonNumber(summary, "no_ros_time"), received);
  EXPECT_EQ(JsonNumber(summary, "odometry_received"), 3.0);
  EXPECT_EQ(JsonNumber(summary, "health_received"), 3.0);
  EXPECT_EQ(JsonNumber(summary, "ticks"), 0.0);
  EXPECT_TRUE(EventLines(lines, "AlignmentTransition").empty());
}

TEST(Px4BridgeNode, MissingOrInvalidConfigRefusesToStart) {
  const auto jsonl = TempJsonl("refuse");
  const auto expect_refusal = [&](const std::vector<std::string>& args, const char* what) {
    NodeProcess node(args);
    const auto code = node.WaitExit(30s);
    ASSERT_TRUE(code.has_value()) << what << ": the node kept running";
    EXPECT_NE(*code, 0) << what;
  };
  expect_refusal({}, "no parameters");
  expect_refusal({"-p", "events_jsonl:=" + jsonl.string()}, "no config_file");
  expect_refusal({"-p", std::string("config_file:=") + UAVNAV_PX4_BRIDGE_SIM_YAML}, "no events_jsonl");
  expect_refusal({"-p", "config_file:=/nonexistent/px4_bridge.yaml", "-p", "events_jsonl:=" + jsonl.string()},
                 "unreadable config_file");
  expect_refusal({"-p", std::string("config_file:=") + UAVNAV_PX4_BRIDGE_SIM_YAML, "-p",
                  "events_jsonl:=/nonexistent_dir/events.jsonl"},
                 "events file that cannot be opened");
  // A config that violates a bound or the cross rule (frozen_max > valid_stale).
  const auto bad = jsonl.parent_path() / "bad_config.yaml";
  {
    std::ifstream in(UAVNAV_PX4_BRIDGE_SIM_YAML);
    std::string text((std::istreambuf_iterator<char>(in)), std::istreambuf_iterator<char>());
    const auto at = text.find("alignment_frozen_max_s: 10.0");
    ASSERT_NE(at, std::string::npos);
    text.replace(at, std::string("alignment_frozen_max_s: 10.0").size(), "alignment_frozen_max_s: 0.5");
    std::ofstream(bad) << text;
  }
  expect_refusal({"-p", "config_file:=" + bad.string(), "-p", "events_jsonl:=" + jsonl.string()}, "invalid config_file");
}

int main(int argc, char** argv) {
  ::setenv("ROS_AUTOMATIC_DISCOVERY_RANGE", "LOCALHOST", 1);
  ::setenv("ROS_LOCALHOST_ONLY", "1", 1);
  // The child inherits the domain: 215..232 (lower domains map onto UDP ports inside the Linux ephemeral range).
  ::setenv("ROS_DOMAIN_ID", std::to_string(215 + (::getpid() % 18)).c_str(), 1);
  rclcpp::init(argc, argv);
  ::testing::InitGoogleTest(&argc, argv);
  const int rc = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return rc;
}
