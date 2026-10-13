#pragma once

#include <array>
#include <atomic>
#include <cstdint>
#include <memory>
#include <mutex>
#include <optional>
#include <stdexcept>
#include <string>
#include <string_view>

#include "rclcpp/callback_group.hpp"
#include "rclcpp/node.hpp"
#include "rclcpp/publisher.hpp"
#include "rclcpp/subscription.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "uavnav/core/event_builder.hpp"
#include "uavnav/core/event_recorder.hpp"
#include "uavnav/core/result.hpp"
#include "uavnav/core/time.hpp"
#include "uavnav/lio_node/ingest.hpp"
#include "uavnav/lio_node/ros_conversion.hpp"
#include "uavnav_interfaces/msg/lio_health.hpp"
#include "uavnav_interfaces/msg/lio_odometry.hpp"
#include "uavnav_interfaces/msg/lio_state.hpp"

// The lio ROS node (SYSTEM_DESIGN §3.5 "Node lio", §6.1, §6.4, §6.5; D33/O19-O21; S1b runtime design). It
// converts, spins and publishes; every decision is in LioIngest / LioRunner (ROS-free).
//
// Start (constructor, in order): (1) the three string parameters (kConfigFileParam, kNodeConfigFileParam,
// kEventsJsonlParam; no defaults) and the two parameter files, validated against kLioSpecs / kLioNodeSpecs;
// a missing or invalid one is ONE RCLCPP_FATAL and LioNodeStartFailed, before any thread of this node starts;
// (2) the process's one EventRecorder over TeeSink(JsonlSink, RosEventSink on /events); (3) LioIngest (which
// starts LioRunner's estimator thread); (4) the publishers and both subscriptions in ONE MutuallyExclusive
// callback group, so even a multi-threaded executor runs them one at a time (LioIngest refuses an overlapping
// call anyway: "LioIngestReentered").
//
// Threads. The executor thread is the ingest thread. The recorder's writer publishes /events. Every callback:
// one rosutil::snapshot (TimeSnapshot), refused while its ROS time is 0 (use_sim_time before the first /clock:
// "no ROS time yet", ingest.hpp "ROS time zero"); SharedRosTime::store of that ROS time; conversion
// (ros_conversion.hpp; a refusal is a rate-limited "LioInputRejected" with the ConversionError); LioIngest; then
// every output is published on the same thread. /lio/state is best-effort, so its publish does not wait for a
// reader; the reliable topics (/lio/odometry, /lio/health; keep-last, depth 10) may wait in publish() behind a slow
// reader. That wait was not measured.
//
// Shutdown (runtime design "Shutdown" row), shutdown(): (1) the node stops taking callbacks: a callback that
// starts after this returns at once, and shutdown waits for the one in flight, if any, to return (ingest does not
// wait; a reliable publish may, see Threads); (2) "LioNodeSummary"; (3) LioIngest::stop(): LioRunner::stop joins the
// estimator thread before its channels go, then "IngestSummary"; (4) the ingest is destroyed, then the recorder
// (its writer drains the ring into the JSONL file and /events, joins, closes the file), so the JSONL file ends
// with IngestSummary; (5) the publishers, subscriptions and the node itself go with the LioNode object.
// shutdown() is idempotent (a second caller waits for the first to finish) and callable from any thread while an
// executor still spins the node, e.g. from the context's pre-shutdown callback that rclcpp's SIGINT/SIGTERM
// handling runs (lio_node_main.cpp). It must never be called from a callback of this node (it would wait for
// itself). The destructor calls it.
//
// Events of the node (component "lio"; listed in the uavnav_lio catalogue, ingest.hpp):
//   "LioInputRejected"  ConversionError (BAD_STAMP, NOT_FINITE, WRONG_FRAME, MALFORMED_CLOUD) or
//                       NodeReason::kNoRosTime; rate-limited per reason (the 1st and every
//                       lio::limits::kEventSummaryEvery-th; values stream 0 IMU / 1 scan, count).
//   "LioPublishFailed"  NodeReason::kPublishFailed: one conversion or publish threw (rate-limited; values topic
//                       0 state / 1 health / 2 odometry, count). Only that message is lost: each message has its
//                       own failure scope, health first (detail/publish_outputs.hpp); the node goes on.
//   "LioNodeFailed"     NodeReason::kCallbackException: an exception left the ingest or a conversion; emitted,
//                       then rethrown to the executor (lio_node_main: RCLCPP_FATAL, shutdown(), exit 1).
//   "LioNodeSummary"    NodeReason::kShutdown, once, in shutdown() before IngestSummary: the node's counters.
namespace uavnav::lio_node {

inline constexpr std::string_view kNodeName = "lio";
inline constexpr std::string_view kConfigFileParam = "config_file";            ///< config/lio/sim.yaml (kLioSpecs)
inline constexpr std::string_view kNodeConfigFileParam = "node_config_file";   ///< config/lio/node_sim.yaml
inline constexpr std::string_view kEventsJsonlParam = "events_jsonl";          ///< the JSONL event file to append
inline constexpr std::string_view kImuTopic = "/lidar/imu";
inline constexpr std::string_view kPointsTopic = "/lidar/points";
inline constexpr std::string_view kStateTopic = "/lio/state";
inline constexpr std::string_view kHealthTopic = "/lio/health";
inline constexpr std::string_view kOdometryTopic = "/lio/odometry";

enum class StartError : std::uint8_t {
  kParameterInvalid,       ///< a path parameter is unset, empty or not a string
  kConfigInvalid,          ///< config_file unreadable or invalid against kLioSpecs (load_lio_config)
  kNodeConfigInvalid,      ///< node_config_file unreadable or invalid against kLioNodeSpecs
  kEventsFileFailed,       ///< the JSONL event file cannot be opened for appending
  kEventsPublisherFailed,  ///< RosEventSink::create failed
  kRecorderFailed,         ///< TeeSink or EventRecorder refused
  kIngestFailed            ///< LioIngest::create (LioRunner::create) refused the configuration
};

constexpr std::string_view to_string(StartError e) {
  switch (e) {
    case StartError::kParameterInvalid:
      return "PARAMETER_INVALID";
    case StartError::kConfigInvalid:
      return "CONFIG_INVALID";
    case StartError::kNodeConfigInvalid:
      return "NODE_CONFIG_INVALID";
    case StartError::kEventsFileFailed:
      return "EVENTS_FILE_FAILED";
    case StartError::kEventsPublisherFailed:
      return "EVENTS_PUBLISHER_FAILED";
    case StartError::kRecorderFailed:
      return "RECORDER_FAILED";
    case StartError::kIngestFailed:
      return "INGEST_FAILED";
  }
  return "";
}

static_assert(ReasonEnum<StartError>);

/// Thrown by the constructor (a constructor has no Result) after its one RCLCPP_FATAL; main exits non-zero.
class LioNodeStartFailed : public std::runtime_error {
 public:
  LioNodeStartFailed(StartError reason, const std::string& detail)
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

class LioNode : public rclcpp::Node {
 public:
  /// Throws LioNodeStartFailed (after one RCLCPP_FATAL) when the node cannot start; see "Start" above.
  explicit LioNode(const rclcpp::NodeOptions& options);
  ~LioNode() override;  ///< shutdown()
  LioNode(const LioNode&) = delete;
  LioNode& operator=(const LioNode&) = delete;

  /// See "Shutdown" above. Never throws.
  void shutdown() noexcept;

 private:
  using Counter = std::atomic<std::uint64_t>;
  class CallbackScope;

  [[noreturn]] void fail(StartError e, const std::string& detail);
  std::string path_parameter(std::string_view name);
  void start(const std::string& config, const std::string& node_config, const std::string& events);

  void on_imu(const sensor_msgs::msg::Imu& msg);
  void on_points(const sensor_msgs::msg::PointCloud2& msg);
  /// The callback's snapshot; nullopt (and a rate-limited NO_ROS_TIME) while the ROS time is 0.
  std::optional<time::TimeSnapshot> take_snapshot(int stream);
  void reject(ConversionError e, int stream, const time::TimeSnapshot& now) noexcept;
  void publish(const IngestOutputs& out, const time::TimeSnapshot& now) noexcept;
  void report_exception(int stream, const time::TimeSnapshot& now) noexcept;
  void emit(const events::EventBuilder& b) noexcept;
  void finish() noexcept;  // the body of shutdown(), run once

  // Destroyed in reverse order: the ingest before the recorder it emits into and the ROS time its backend reads.
  SharedRosTime ros_time_;
  std::unique_ptr<events::EventRecorder> recorder_;
  std::unique_ptr<LioIngest> ingest_;
  rclcpp::Publisher<uavnav_interfaces::msg::LioState>::SharedPtr state_pub_;
  rclcpp::Publisher<uavnav_interfaces::msg::LioHealth>::SharedPtr health_pub_;
  rclcpp::Publisher<uavnav_interfaces::msg::LioOdometry>::SharedPtr odometry_pub_;
  rclcpp::CallbackGroup::SharedPtr group_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr points_sub_;

  // The callback gate (see "Shutdown" and CallbackScope in lio_node.cpp).
  std::atomic<bool> accepting_{false};
  std::atomic<std::uint32_t> in_flight_{0};
  std::once_flag shutdown_once_;

  // Counters: written by the callbacks (one at a time), read by shutdown() after the gate closed.
  Counter imu_received_{0}, scans_received_{0}, scans_converted_{0}, points_in_{0}, points_non_finite_{0};
  std::array<Counter, kConversionErrorCount> rejected_{};  // by ConversionError
  Counter no_ros_time_{0}, ingest_refused_{0}, states_published_{0}, health_published_{0}, odometry_published_{0},
      publish_failed_{0};
};

}  // namespace uavnav::lio_node
