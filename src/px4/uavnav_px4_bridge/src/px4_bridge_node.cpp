// The px4_bridge process (SYSTEM_DESIGN §4.1, §6.1, §6.4, §6.5; D33/O19-O22; S1b runtime design): one Px4BridgeNode
// on one SingleThreadedExecutor. The node converts, spins and publishes; every decision is in Px4Bridge (ROS-free).
// Everything is in this file because tools/uavnav/arch_rules.yaml lets ROS into this package only here and in
// ros_conversion.*.
//
// Start (constructor, in order): (1) the two string parameters (config_file, events_jsonl; no defaults) and the
// alignment config, validated against kAlignmentSpecs; a missing or invalid one is ONE RCLCPP_FATAL and
// Px4BridgeStartFailed, before anything else exists; (2) the process's one EventRecorder over
// TeeSink(JsonlSink, RosEventSink on /events); (3) Px4Bridge; (4) the two publishers; (5) ONE MutuallyExclusive
// callback group; (6) ONE Px4Bridge::on_tick, BEFORE any subscription exists, so a message stamped far ahead of
// ROS time that arrives before the first timer tick is refused as a future stamp instead of held in the alignment
// (limits::kMaxStampAhead has no reference before the first tick); skipped while ROS time is 0 (use_sim_time
// before the first /clock), in which case every callback runs the tick before its input if none has run yet;
// (7) the three subscriptions and the 100 ms timer, all in that group, so even a multi-threaded executor makes
// every Px4Bridge call one at a time (the core class has no overlap guard).
//
// Topics and QoS (SYSTEM_DESIGN §6.5, uavnav/ros/qos.hpp; the versioned PX4 topic of VehicleLocalPosition,
// MESSAGE_VERSION 1): in /lio/odometry, /lio/health (reliable_output), /fmu/out/vehicle_local_position_v1 (px4:
// best effort, depth 5); out /fmu/in/vehicle_visual_odometry (px4), /alignment (reliable_output).
//
// Threads. The executor thread makes every bridge call. The recorder's writer publishes /events. Every callback:
// one rosutil::snapshot (TimeSnapshot), refused while its ROS time is 0; conversion (ros_conversion.hpp; a refusal
// is a rate-limited "LioInputRejected" / "Px4SampleRejected" with BridgeReason::kNotFinite); the bridge call; then
// every output is published on the same thread, each in its own failure scope (a publish never waits for a reader).
//
// Shutdown, shutdown(): (1) the node stops taking callbacks: a callback that starts after this returns at once,
// and shutdown waits for the one in flight to return (it never blocks); (2) Px4Bridge::flush(now) ends every open
// rejection episode (their summaries are not lost); (3) "BridgeNodeSummary", the last record of the JSONL file;
// (4) the bridge is destroyed, then the recorder (its writer drains the ring into the JSONL file and /events,
// joins, closes the file). shutdown() is idempotent and callable from any thread while an executor still spins
// the node: main() runs it from the context's pre-shutdown callback that rclcpp's SIGINT/SIGTERM handling invokes,
// so the final records still reach /events. It must never be called from a callback of this node.
//
// Events of the node (component "px4_bridge"; the core's events are in bridge.hpp):
//   "LioInputRejected"   BridgeReason::kNotFinite: a LioOdometry (stream 0) or LioHealth (stream 1) that did not
//                        convert (stamp, state code); rate-limited (the 1st and every kEventSummaryEvery-th;
//                        values stream, count).
//   "Px4SampleRejected"  BridgeReason::kNotFinite: a VehicleLocalPosition that did not convert (stamp out of
//                        range, a non-finite number); same rate limit; values count.
//   "BridgeNoRosTime"    NodeReason::kNoRosTime: a message (or the timer) while ROS time is 0; values stream
//                        (0 odometry, 1 health, 2 PX4, 3 tick), count.
//   "BridgePublishFailed" NodeReason::kPublishFailed: one conversion or publish threw; values topic (0 EV,
//                        1 alignment), count. Only that message is lost.
//   "BridgeNodeFailed"   NodeReason::kCallbackException: an exception left the bridge or a conversion; emitted,
//                        then rethrown to the executor (main: RCLCPP_FATAL, shutdown(), exit 1).
//   "BridgeNodeSummary"  NodeReason::kShutdown, once, in shutdown(): the node's counters.
#include <atomic>
#include <chrono>
#include <cstdint>
#include <cstdlib>
#include <exception>
#include <memory>
#include <mutex>
#include <optional>
#include <stdexcept>
#include <string>
#include <string_view>
#include <thread>
#include <utility>

#include "rcl_interfaces/msg/parameter_descriptor.hpp"
#include "rclcpp/rclcpp.hpp"
#include "uavnav/core/config.hpp"
#include "uavnav/core/event_builder.hpp"
#include "uavnav/core/event_recorder.hpp"
#include "uavnav/core/jsonl_sink.hpp"
#include "uavnav/core/result.hpp"
#include "uavnav/core/tee_sink.hpp"
#include "uavnav/core/time.hpp"
#include "uavnav/px4bridge/bridge.hpp"
#include "uavnav/px4bridge/config.hpp"
#include "uavnav/px4bridge/limits.hpp"
#include "uavnav/px4bridge/ros_conversion.hpp"
#include "uavnav/ros/qos.hpp"
#include "uavnav/ros/ros_event_sink.hpp"
#include "uavnav/ros/time_msg.hpp"

namespace uavnav::px4bridge {

namespace {

using events::Component;
using events::EventBuilder;

inline constexpr std::string_view kNodeName = "px4_bridge";
inline constexpr std::string_view kConfigFileParam = "config_file";    ///< config/px4_bridge/sim.yaml (kAlignmentSpecs)
inline constexpr std::string_view kEventsJsonlParam = "events_jsonl";  ///< the JSONL event file to append
inline constexpr std::string_view kLioOdometryTopic = "/lio/odometry";
inline constexpr std::string_view kLioHealthTopic = "/lio/health";
inline constexpr std::string_view kLocalPositionTopic = "/fmu/out/vehicle_local_position_v1";
inline constexpr std::string_view kEvTopic = "/fmu/in/vehicle_visual_odometry";
inline constexpr std::string_view kAlignmentTopic = "/alignment";

/// "stream" value of LioInputRejected and BridgeNoRosTime: the bridge's own numbering (bridge.hpp "input").
constexpr int kStreamOdometry = 0;
constexpr int kStreamHealth = 1;
constexpr int kStreamPx4 = 2;
constexpr int kStreamTick = 3;
/// "topic" value of BridgePublishFailed.
constexpr int kTopicEv = 0;
constexpr int kTopicAlignment = 1;

enum class StartError : std::uint8_t {
  kParameterInvalid,       ///< a path parameter is unset, empty or not a string
  kConfigInvalid,          ///< config_file unreadable or invalid against kAlignmentSpecs
  kEventsFileFailed,       ///< the JSONL event file cannot be opened for appending
  kEventsPublisherFailed,  ///< RosEventSink::create failed
  kRecorderFailed          ///< TeeSink or EventRecorder refused
};

constexpr std::string_view to_string(StartError e) {
  switch (e) {
    case StartError::kParameterInvalid:
      return "PARAMETER_INVALID";
    case StartError::kConfigInvalid:
      return "CONFIG_INVALID";
    case StartError::kEventsFileFailed:
      return "EVENTS_FILE_FAILED";
    case StartError::kEventsPublisherFailed:
      return "EVENTS_PUBLISHER_FAILED";
    case StartError::kRecorderFailed:
      return "RECORDER_FAILED";
  }
  return "";
}

static_assert(ReasonEnum<StartError>);

/// Thrown by the constructor (a constructor has no Result) after its one RCLCPP_FATAL; main exits non-zero.
class Px4BridgeStartFailed : public std::runtime_error {
 public:
  Px4BridgeStartFailed(StartError reason, const std::string& detail)
      : std::runtime_error(std::string(to_string(reason)) + ": " + detail) {}
};

enum class NodeReason : std::uint8_t { kNoRosTime, kPublishFailed, kCallbackException, kShutdown };

constexpr std::string_view to_string(NodeReason r) {
  switch (r) {
    case NodeReason::kNoRosTime:
      return "NO_ROS_TIME";
    case NodeReason::kPublishFailed:
      return "PUBLISH_FAILED";
    case NodeReason::kCallbackException:
      return "CALLBACK_EXCEPTION";
    case NodeReason::kShutdown:
      return "SHUTDOWN";
  }
  return "";
}

static_assert(ReasonEnum<NodeReason>);

/// Rate limit of the node's per-message events: the 1st and every kEventSummaryEvery-th occurrence.
constexpr bool spam_due(std::uint64_t count) noexcept {
  return count == 1 || count % limits::kEventSummaryEvery == 0;
}

using Counter = std::atomic<std::uint64_t>;

/// Increment of a counter with one writer at a time (the callbacks; see the class): returns the new value.
std::uint64_t bump(Counter& c) noexcept {
  const std::uint64_t v = c.load(std::memory_order_relaxed) + 1;
  c.store(v, std::memory_order_relaxed);
  return v;
}

double as_double(const Counter& c) noexcept { return static_cast<double>(c.load(std::memory_order_relaxed)); }

std::string describe(const std::string& path, const config::ConfigError& e) {
  return path + ": " + std::string(config::to_string(e.kind)) + (e.key.empty() ? "" : " '" + e.key + "'") +
         (e.detail.empty() ? "" : " (" + e.detail + ")");
}

/// The tick's ROS time as SensorTime: in SITL every stamp is the simulation /clock (D29), so the identity.
time::SensorTime sensor_time_of(const time::TimeSnapshot& now) noexcept { return time::SensorTime{now.ros.ns}; }

class Px4BridgeNode : public rclcpp::Node {
 public:
  /// Throws Px4BridgeStartFailed (after one RCLCPP_FATAL) when the node cannot start; see the file comment.
  explicit Px4BridgeNode(const rclcpp::NodeOptions& options) : rclcpp::Node(std::string(kNodeName), options) {
    const std::string config = path_parameter(kConfigFileParam);
    const std::string events = path_parameter(kEventsJsonlParam);
    start(config, events);
    RCLCPP_INFO(get_logger(), "px4_bridge started: %s=%s %s=%s", kConfigFileParam.data(), config.c_str(),
                kEventsJsonlParam.data(), events.c_str());
  }
  ~Px4BridgeNode() override { shutdown(); }
  Px4BridgeNode(const Px4BridgeNode&) = delete;
  Px4BridgeNode& operator=(const Px4BridgeNode&) = delete;

  /// See the file comment "Shutdown". Never throws.
  void shutdown() noexcept {
    try {
      std::call_once(shutdown_once_, [this] { finish(); });
    } catch (...) {
      // std::call_once reports a failed once-flag (std::system_error); finish() itself never throws.
    }
  }

 private:
  /// Entered by every callback before it touches the bridge, the recorder or a publisher. Dekker-style on two
  /// seq_cst atomics: the callback increments in_flight_ THEN reads accepting_; finish() clears accepting_ THEN
  /// waits for in_flight_ == 0. In the single total order of these operations, either the callback reads false
  /// (and touches nothing), or finish() sees it in flight and waits for its decrement, which happens-before
  /// everything finish() does next. The callback never waits; RAII keeps the count right when an exception
  /// leaves the callback.
  class CallbackScope {
   public:
    explicit CallbackScope(Px4BridgeNode& node) noexcept : node_(node) {
      node_.in_flight_.fetch_add(1, std::memory_order_seq_cst);
      entered_ = node_.accepting_.load(std::memory_order_seq_cst);
    }
    ~CallbackScope() { node_.in_flight_.fetch_sub(1, std::memory_order_seq_cst); }
    CallbackScope(const CallbackScope&) = delete;
    CallbackScope& operator=(const CallbackScope&) = delete;
    explicit operator bool() const noexcept { return entered_; }

   private:
    Px4BridgeNode& node_;
    bool entered_;
  };

  // --- start -----------------------------------------------------------------------------------------------

  [[noreturn]] void fail(StartError e, const std::string& detail) {
    RCLCPP_FATAL(get_logger(), "px4_bridge refuses to start: %s: %s", std::string(to_string(e)).c_str(),
                 detail.c_str());
    throw Px4BridgeStartFailed(e, detail);
  }

  std::string path_parameter(std::string_view name) {
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.description = "path of a file; required, no default (SYSTEM_DESIGN §6.2, D22)";
    descriptor.read_only = true;
    std::string value;
    try {
      value = declare_parameter<std::string>(std::string(name), std::string(), descriptor);
    } catch (const std::exception& e) {  // e.g. InvalidParameterTypeException: an override that is not a string
      fail(StartError::kParameterInvalid, "parameter '" + std::string(name) + "': " + e.what());
    }
    if (value.empty()) fail(StartError::kParameterInvalid, "parameter '" + std::string(name) + "' is not set");
    return value;
  }

  void start(const std::string& config, const std::string& events) {
    // (1) The alignment config, before anything starts a thread.
    const auto values = config::load_params_file(config, kAlignmentSpecs);
    if (!values) fail(StartError::kConfigInvalid, describe(config, values.error()));
    const auto alignment_config = load_alignment_config(*values);
    if (!alignment_config) fail(StartError::kConfigInvalid, describe(config, alignment_config.error()));

    // (2) The process's one recorder (§6.1): TeeSink(JsonlSink, RosEventSink). Its writer is the first thread.
    auto jsonl = events::JsonlSink::open(events);
    if (!jsonl) fail(StartError::kEventsFileFailed, events + ": cannot open for appending");
    auto ros_sink = rosutil::RosEventSink::create(*this);
    if (!ros_sink) fail(StartError::kEventsPublisherFailed, std::string(rosutil::to_string(ros_sink.error())));
    auto tee = events::TeeSink::create(std::move(*jsonl), std::move(*ros_sink));
    if (!tee) fail(StartError::kRecorderFailed, std::string(events::to_string(tee.error())));
    auto recorder = events::EventRecorder::create(std::move(*tee));
    if (!recorder) fail(StartError::kRecorderFailed, std::string(events::to_string(recorder.error())));
    recorder_ = std::move(*recorder);

    // (3) The bridge, (4) the publishers (QoS O21), (5) the one callback group.
    bridge_ = std::make_unique<Px4Bridge>(*alignment_config, *recorder_);
    ev_pub_ = create_publisher<px4_msgs::msg::VehicleOdometry>(std::string(kEvTopic), rosutil::qos::px4());
    alignment_pub_ = create_publisher<uavnav_interfaces::msg::Alignment>(std::string(kAlignmentTopic),
                                                                         rosutil::qos::reliable_output());
    group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    accepting_.store(true, std::memory_order_seq_cst);  // before the first callback can be taken

    // (6) One tick before any subscription exists (the file comment). No callback can run yet, so no gate.
    const time::TimeSnapshot now = rosutil::snapshot(*get_clock());
    if (now.ros.ns != 0) tick(now);

    // (7) The subscriptions and the timer, all in the group.
    rclcpp::SubscriptionOptions options;
    options.callback_group = group_;
    odometry_sub_ = create_subscription<uavnav_interfaces::msg::LioOdometry>(
        std::string(kLioOdometryTopic), rosutil::qos::reliable_output(),
        [this](const uavnav_interfaces::msg::LioOdometry& m) { on_odometry(m); }, options);
    health_sub_ = create_subscription<uavnav_interfaces::msg::LioHealth>(
        std::string(kLioHealthTopic), rosutil::qos::reliable_output(),
        [this](const uavnav_interfaces::msg::LioHealth& m) { on_health(m); }, options);
    local_position_sub_ = create_subscription<px4_msgs::msg::VehicleLocalPosition>(
        std::string(kLocalPositionTopic), rosutil::qos::px4(),
        [this](const px4_msgs::msg::VehicleLocalPosition& m) { on_local_position(m); }, options);
    timer_ = create_timer(std::chrono::nanoseconds(limits::kTickPeriod.ns), [this] { on_timer(); }, group_);
  }

  // --- callbacks (the executor thread) ---------------------------------------------------------------------

  void on_odometry(const uavnav_interfaces::msg::LioOdometry& msg) {
    const CallbackScope scope(*this);
    if (!scope) return;
    bump(odometry_received_);
    const auto now = take_snapshot(kStreamOdometry);
    if (!now) return;
    try {
      ensure_ticked(*now);
      const auto in = to_lio_odometry_in(msg);
      if (!in) return reject(kStreamOdometry, rejected_odometry_, "LioInputRejected", true, *now);
      publish(bridge_->on_lio_odometry(*in, *now), *now);
    } catch (...) {
      report_exception(kStreamOdometry, *now);
      throw;
    }
  }

  void on_health(const uavnav_interfaces::msg::LioHealth& msg) {
    const CallbackScope scope(*this);
    if (!scope) return;
    bump(health_received_);
    const auto now = take_snapshot(kStreamHealth);
    if (!now) return;
    try {
      ensure_ticked(*now);
      const auto in = to_lio_health_in(msg);
      if (!in) return reject(kStreamHealth, rejected_health_, "LioInputRejected", true, *now);
      publish(bridge_->on_lio_health(*in, *now), *now);
    } catch (...) {
      report_exception(kStreamHealth, *now);
      throw;
    }
  }

  void on_local_position(const px4_msgs::msg::VehicleLocalPosition& msg) {
    const CallbackScope scope(*this);
    if (!scope) return;
    bump(local_position_received_);
    const auto now = take_snapshot(kStreamPx4);
    if (!now) return;
    try {
      ensure_ticked(*now);
      const auto in = to_px4_local_position(msg);
      if (!in) return reject(kStreamPx4, rejected_local_position_, "Px4SampleRejected", false, *now);
      publish(bridge_->on_px4_local_position(*in, *now), *now);
    } catch (...) {
      report_exception(kStreamPx4, *now);
      throw;
    }
  }

  void on_timer() {
    const CallbackScope scope(*this);
    if (!scope) return;
    const auto now = take_snapshot(kStreamTick);
    if (!now) return;
    try {
      tick(*now);
    } catch (...) {
      report_exception(kStreamTick, *now);
      throw;
    }
  }

  /// The callback's snapshot; nullopt (and a rate-limited NO_ROS_TIME) while the ROS time is 0 (use_sim_time before
  /// the first /clock, or an unconvertible clock): a stamp would be refused as ahead of a tick at 0.
  std::optional<time::TimeSnapshot> take_snapshot(int stream) {
    const time::TimeSnapshot now = rosutil::snapshot(*get_clock());
    if (now.ros.ns == 0) {
      const std::uint64_t n = bump(no_ros_time_);
      if (spam_due(n)) {
        emit(EventBuilder(Component::kPx4Bridge, "BridgeNoRosTime", now)
                 .reason(NodeReason::kNoRosTime)
                 .value("stream", stream)
                 .value("count", static_cast<double>(n)));
      }
      return std::nullopt;
    }
    return now;
  }

  /// One Px4Bridge::on_tick at `now`, its alignment published.
  void tick(const time::TimeSnapshot& now) {
    ticked_ = true;
    bump(ticks_);
    publish(bridge_->on_tick(sensor_time_of(now), now), now);
  }

  /// Before the first input of a node that started with ROS time 0 (the constructor could not tick): the tick
  /// that gives the future-stamp check its reference (file comment, start (6)).
  void ensure_ticked(const time::TimeSnapshot& now) {
    if (!ticked_) tick(now);
  }

  void reject(int stream, Counter& counter, std::string_view event, bool with_stream,
              const time::TimeSnapshot& now) noexcept {
    const std::uint64_t n = bump(counter);
    if (!spam_due(n)) return;
    EventBuilder b(Component::kPx4Bridge, event, now);
    b.reason(BridgeReason::kNotFinite);
    if (with_stream) b.value("stream", stream);
    emit(b.value("count", static_cast<double>(n)));
  }

  /// Converts and publishes one call's outputs, each message in its own failure scope: the EV first (the stream
  /// PX4 is waiting for), then the alignment. An exception from one conversion or publish is counted and reported
  /// and never skips the other message.
  void publish(const BridgeOutputs& out, const time::TimeSnapshot& now) noexcept {
    if (out.ev) {
      try {
        const std::int64_t us = now.ros.ns / 1000;  // PX4 time = ROS time in SITL (D29); ros.ns != 0 here
        ev_pub_->publish(to_vehicle_odometry(*out.ev, us > 0 ? static_cast<std::uint64_t>(us) : 0));
        bump(ev_published_);
      } catch (...) {
        publish_failed(kTopicEv, now);
      }
    }
    if (out.alignment) {
      try {
        alignment_pub_->publish(to_msg(*out.alignment));
        bump(alignment_published_);
      } catch (...) {
        publish_failed(kTopicAlignment, now);
      }
    }
  }

  void publish_failed(int topic, const time::TimeSnapshot& now) noexcept {
    const std::uint64_t n = bump(publish_failed_);
    if (spam_due(n)) {
      emit(EventBuilder(Component::kPx4Bridge, "BridgePublishFailed", now)
               .reason(NodeReason::kPublishFailed)
               .value("topic", topic)
               .value("count", static_cast<double>(n)));
    }
  }

  void report_exception(int stream, const time::TimeSnapshot& now) noexcept {
    emit(EventBuilder(Component::kPx4Bridge, "BridgeNodeFailed", now)
             .reason(NodeReason::kCallbackException)
             .value("stream", stream));
  }

  void emit(const EventBuilder& b) noexcept {
    if (recorder_) (void)recorder_->emit(b.build());
  }

  // --- shutdown --------------------------------------------------------------------------------------------

  void finish() noexcept {
    // (1) No callback enters any more; wait for the one in flight (it never blocks: the bridge and publish do not wait).
    accepting_.store(false, std::memory_order_seq_cst);
    while (in_flight_.load(std::memory_order_seq_cst) != 0) std::this_thread::sleep_for(std::chrono::milliseconds(1));

    const time::TimeSnapshot now = rosutil::snapshot(*get_clock());
    // (2) The open rejection episodes end here: one summary each, before the node's own.
    if (bridge_) bridge_->flush(now);
    // (3) The node's counters: the last record of the JSONL file.
    emit(EventBuilder(Component::kPx4Bridge, "BridgeNodeSummary", now)
             .reason(NodeReason::kShutdown)
             .value("odometry_received", as_double(odometry_received_))
             .value("health_received", as_double(health_received_))
             .value("local_position_received", as_double(local_position_received_))
             .value("rejected_odometry", as_double(rejected_odometry_))
             .value("rejected_health", as_double(rejected_health_))
             .value("rejected_local_position", as_double(rejected_local_position_))
             .value("no_ros_time", as_double(no_ros_time_))
             .value("ticks", as_double(ticks_))
             .value("ev_published", as_double(ev_published_))
             .value("alignment_published", as_double(alignment_published_))
             .value("publish_failed", as_double(publish_failed_)));
    // (4) The bridge (nothing runs in it any more), then the recorder: its destructor drains the ring into the
    // JSONL file and /events and joins the writer. The publishers and the node go with the object.
    bridge_.reset();
    recorder_.reset();
  }

  // Destroyed in reverse order: the bridge before the recorder it emits into.
  std::unique_ptr<events::EventRecorder> recorder_;
  std::unique_ptr<Px4Bridge> bridge_;
  rclcpp::Publisher<px4_msgs::msg::VehicleOdometry>::SharedPtr ev_pub_;
  rclcpp::Publisher<uavnav_interfaces::msg::Alignment>::SharedPtr alignment_pub_;
  rclcpp::CallbackGroup::SharedPtr group_;
  rclcpp::Subscription<uavnav_interfaces::msg::LioOdometry>::SharedPtr odometry_sub_;
  rclcpp::Subscription<uavnav_interfaces::msg::LioHealth>::SharedPtr health_sub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr local_position_sub_;
  rclcpp::TimerBase::SharedPtr timer_;

  // The callback gate (see CallbackScope).
  std::atomic<bool> accepting_{false};
  std::atomic<std::uint32_t> in_flight_{0};
  std::once_flag shutdown_once_;

  // Touched by the group's callbacks (and the constructor before they exist): one at a time.
  bool ticked_{false};

  // Counters: written by the callbacks (one at a time), read by finish() after the gate closed.
  Counter odometry_received_{0}, health_received_{0}, local_position_received_{0}, rejected_odometry_{0},
      rejected_health_{0}, rejected_local_position_{0}, no_ros_time_{0}, ticks_{0}, ev_published_{0},
      alignment_published_{0}, publish_failed_{0};
};

}  // namespace

}  // namespace uavnav::px4bridge

/// The process: one Px4BridgeNode on one SingleThreadedExecutor, whose thread makes every bridge call.
int main(int argc, char** argv) {
  using uavnav::px4bridge::Px4BridgeNode;
  using uavnav::px4bridge::Px4BridgeStartFailed;
  rclcpp::init(argc, argv);
  std::shared_ptr<Px4BridgeNode> node;
  try {
    node = std::make_shared<Px4BridgeNode>(rclcpp::NodeOptions());
  } catch (const Px4BridgeStartFailed&) {
    rclcpp::shutdown();  // the node logged its one RCLCPP_FATAL
    return EXIT_FAILURE;
  } catch (const std::exception& e) {
    RCLCPP_FATAL(rclcpp::get_logger(std::string(uavnav::px4bridge::kNodeName)), "px4_bridge refuses to start: %s", e.what());
    rclcpp::shutdown();
    return EXIT_FAILURE;
  }

  // rclcpp's SIGINT/SIGTERM handling shuts the context down; it first runs the pre-shutdown callbacks, so
  // shutdown() runs while the context is still valid and the final records still reach /events.
  const rclcpp::Context::SharedPtr context = node->get_node_base_interface()->get_context();
  Px4BridgeNode* const raw = node.get();
  const auto hook = context->add_pre_shutdown_callback([raw] { raw->shutdown(); });

  int rc = EXIT_SUCCESS;
  {
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    try {
      executor.spin();
    } catch (const std::exception& e) {  // a callback threw: the node emitted BridgeNodeFailed
      RCLCPP_FATAL(node->get_logger(), "px4_bridge executor failed: %s", e.what());
      rc = EXIT_FAILURE;
    } catch (...) {
      RCLCPP_FATAL(node->get_logger(), "px4_bridge executor failed: unknown exception");
      rc = EXIT_FAILURE;
    }
  }
  node->shutdown();  // the executor has stopped
  context->remove_pre_shutdown_callback(hook);
  node.reset();
  rclcpp::shutdown();
  return rc;
}
