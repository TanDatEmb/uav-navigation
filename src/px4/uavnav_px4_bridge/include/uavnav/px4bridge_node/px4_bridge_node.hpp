#pragma once

#include <atomic>
#include <cstdint>
#include <memory>
#include <mutex>
#include <optional>
#include <stdexcept>
#include <string>
#include <string_view>

#include "px4_msgs/msg/vehicle_local_position.hpp"
#include "px4_msgs/msg/vehicle_odometry.hpp"
#include "rclcpp/callback_group.hpp"
#include "rclcpp/node.hpp"
#include "rclcpp/publisher.hpp"
#include "rclcpp/subscription.hpp"
#include "rclcpp/timer.hpp"
#include "uavnav/core/event_builder.hpp"
#include "uavnav/core/event_recorder.hpp"
#include "uavnav/core/result.hpp"
#include "uavnav/core/time.hpp"
#include "uavnav/px4bridge/bridge.hpp"
#include "uavnav_interfaces/msg/alignment.hpp"
#include "uavnav_interfaces/msg/lio_health.hpp"
#include "uavnav_interfaces/msg/lio_odometry.hpp"

// The px4_bridge ROS node (SYSTEM_DESIGN §4.1, §6.1, §6.4, §6.5; D33/O19-O22; S1b runtime design). It converts,
// spins and publishes; every decision is in Px4Bridge (ROS-free, px4bridge/bridge.hpp). The core target
// (uavnav_px4_bridge_core) never includes this header.
//
// Start (constructor, in order): (1) the two string parameters (kConfigFileParam, kEventsJsonlParam; no defaults)
// and the alignment config, validated against kAlignmentSpecs; a missing or invalid one is ONE RCLCPP_FATAL and
// Px4BridgeNodeStartFailed, before anything else exists; (2) the process's one EventRecorder over
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
// Threads. The executor thread(s) make every bridge call, one at a time. The recorder's writer publishes /events.
// Every callback: one rosutil::snapshot (TimeSnapshot), refused while its ROS time is 0; conversion
// (ros_conversion.hpp; a refusal is a rate-limited "LioInputRejected" / "Px4SampleRejected" with
// BridgeReason::kNotFinite); the bridge call; then every output is published on the same thread, each in its own
// failure scope. The EV topic is best-effort, so its publish does not wait for a reader; /alignment is reliable
// (keep-last, depth 10) and may wait behind a slow reader (not measured).
//
// Shutdown, shutdown(): (1) the node stops taking callbacks: a callback that starts after this returns at once,
// and shutdown waits for the one in flight to return (it never blocks); (2) Px4Bridge::flush(now) ends every open
// rejection episode (their summaries are not lost); (3) "BridgeNodeSummary", the last record of the JSONL file;
// (4) the bridge is destroyed, then the recorder (its writer drains the ring into the JSONL file and /events,
// joins, closes the file). shutdown() is idempotent and callable from any thread while an executor still spins
// the node: run() calls it from the context's pre-shutdown callback that rclcpp's SIGINT/SIGTERM handling invokes,
// so the final records still reach /events. It must never be called from a callback of this node. The destructor
// calls it.
//
// Events of the node (component "px4_bridge"; the core's events are in bridge.hpp). The node-level `count_total`
// is the running total since start (the 1st and every limits::kEventSummaryEvery-th is emitted); it is NOT the
// core's episode `count` (with `suppressed` and `summary`) of events that share a name:
//   "LioInputRejected"   BridgeReason::kNotFinite: a LioOdometry (stream 0) or LioHealth (stream 1) that did not
//                        convert (stamp, state code); values stream, count_total.
//   "Px4SampleRejected"  BridgeReason::kNotFinite: a VehicleLocalPosition that did not convert (stamp out of
//                        range, a non-finite number); values count_total.
//   "BridgeNoRosTime"    NodeReason::kNoRosTime: a message (or the timer) while ROS time is 0; values stream
//                        (0 odometry, 1 health, 2 PX4, 3 tick), count_total.
//   "BridgePublishFailed" NodeReason::kPublishFailed: one conversion or publish threw; values topic (0 EV,
//                        1 alignment), count_total. Only that message is lost.
//   "BridgeNodeFailed"   NodeReason::kCallbackException: an exception left the bridge or a conversion; emitted,
//                        then rethrown to the executor (run: RCLCPP_FATAL, shutdown(), exit 1).
//   "BridgeNodeSummary"  NodeReason::kShutdown, once, in shutdown(): the node's counters and `first_tick` (which
//                        path ran the first Px4Bridge::on_tick: 0 constructor, 1 timer, 2 an input callback; -1 none).
namespace uavnav::px4bridge_node {

inline constexpr std::string_view kNodeName = "px4_bridge";
inline constexpr std::string_view kConfigFileParam = "config_file";    ///< config/px4_bridge/sim.yaml (kAlignmentSpecs)
inline constexpr std::string_view kEventsJsonlParam = "events_jsonl";  ///< the JSONL event file to append
inline constexpr std::string_view kLioOdometryTopic = "/lio/odometry";
inline constexpr std::string_view kLioHealthTopic = "/lio/health";
inline constexpr std::string_view kLocalPositionTopic = "/fmu/out/vehicle_local_position_v1";
inline constexpr std::string_view kEvTopic = "/fmu/in/vehicle_visual_odometry";
inline constexpr std::string_view kAlignmentTopic = "/alignment";

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

/// Thrown by the constructor (a constructor has no Result) after its one RCLCPP_FATAL; run() exits non-zero.
class Px4BridgeNodeStartFailed : public std::runtime_error {
 public:
  Px4BridgeNodeStartFailed(StartError reason, const std::string& detail)
      : std::runtime_error(std::string(to_string(reason)) + ": " + detail), reason_(reason) {}
  StartError reason() const noexcept { return reason_; }

 private:
  StartError reason_;
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

class Px4BridgeNode : public rclcpp::Node {
 public:
  /// Throws Px4BridgeNodeStartFailed (after one RCLCPP_FATAL) when the node cannot start; see "Start" above.
  explicit Px4BridgeNode(const rclcpp::NodeOptions& options);
  ~Px4BridgeNode() override;  ///< shutdown()
  Px4BridgeNode(const Px4BridgeNode&) = delete;
  Px4BridgeNode& operator=(const Px4BridgeNode&) = delete;

  /// See "Shutdown" above. Never throws.
  void shutdown() noexcept;

 private:
  using Counter = std::atomic<std::uint64_t>;
  class CallbackScope;
  /// Who ran the first Px4Bridge::on_tick (BridgeNodeSummary `first_tick`).
  enum class FirstTick : int { kNone = -1, kConstructor = 0, kTimer = 1, kInput = 2 };

  [[noreturn]] void fail(StartError e, const std::string& detail);
  std::string path_parameter(std::string_view name);
  void start(const std::string& config, const std::string& events);

  void on_odometry(const uavnav_interfaces::msg::LioOdometry& msg);
  void on_health(const uavnav_interfaces::msg::LioHealth& msg);
  void on_local_position(const px4_msgs::msg::VehicleLocalPosition& msg);
  void on_timer();
  /// The callback's snapshot; nullopt (and a rate-limited NO_ROS_TIME) while the ROS time is 0.
  std::optional<time::TimeSnapshot> take_snapshot(int stream);
  void tick(const time::TimeSnapshot& now, FirstTick by);
  void ensure_ticked(const time::TimeSnapshot& now);
  void reject(int stream, Counter& counter, std::string_view event, bool with_stream,
              const time::TimeSnapshot& now) noexcept;
  void publish(const px4bridge::BridgeOutputs& out, const time::TimeSnapshot& now) noexcept;
  void publish_failed(int topic, const time::TimeSnapshot& now) noexcept;
  void report_exception(int stream, const time::TimeSnapshot& now) noexcept;
  void emit(const events::EventBuilder& b) noexcept;
  void finish() noexcept;  // the body of shutdown(), run once

  // Destroyed in reverse order: the bridge before the recorder it emits into.
  std::unique_ptr<events::EventRecorder> recorder_;
  std::unique_ptr<px4bridge::Px4Bridge> bridge_;
  rclcpp::Publisher<px4_msgs::msg::VehicleOdometry>::SharedPtr ev_pub_;
  rclcpp::Publisher<uavnav_interfaces::msg::Alignment>::SharedPtr alignment_pub_;
  rclcpp::CallbackGroup::SharedPtr group_;
  rclcpp::Subscription<uavnav_interfaces::msg::LioOdometry>::SharedPtr odometry_sub_;
  rclcpp::Subscription<uavnav_interfaces::msg::LioHealth>::SharedPtr health_sub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr local_position_sub_;
  rclcpp::TimerBase::SharedPtr timer_;

  // The callback gate (see CallbackScope in px4_bridge_node.cpp).
  std::atomic<bool> accepting_{false};
  std::atomic<std::uint32_t> in_flight_{0};
  std::once_flag shutdown_once_;

  // Touched by the group's callbacks (and the constructor before they exist): one at a time.
  FirstTick first_tick_{FirstTick::kNone};

  // Counters: written by the callbacks (one at a time), read by finish() after the gate closed.
  Counter odometry_received_{0}, health_received_{0}, local_position_received_{0}, rejected_odometry_{0},
      rejected_health_{0}, rejected_local_position_{0}, no_ros_time_{0}, ticks_{0}, ev_published_{0},
      alignment_published_{0}, publish_failed_{0};
};

/// The process: one Px4BridgeNode on one SingleThreadedExecutor, whose thread makes every bridge call. Exit code 0
/// after SIGINT/SIGTERM, 1 when the node cannot start or a callback threw. Called by main (px4_bridge_main.cpp).
int run(int argc, char** argv);

}  // namespace uavnav::px4bridge_node
