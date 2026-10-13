#include "uavnav/px4bridge_node/px4_bridge_node.hpp"

#include <chrono>
#include <cstdlib>
#include <exception>
#include <thread>
#include <utility>

#include "rcl_interfaces/msg/parameter_descriptor.hpp"
#include "rclcpp/rclcpp.hpp"
#include "uavnav/core/config.hpp"
#include "uavnav/core/jsonl_sink.hpp"
#include "uavnav/core/tee_sink.hpp"
#include "uavnav/px4bridge/config.hpp"
#include "uavnav/px4bridge/limits.hpp"
#include "uavnav/px4bridge/ros_conversion.hpp"
#include "uavnav/ros/qos.hpp"
#include "uavnav/ros/ros_event_sink.hpp"
#include "uavnav/ros/time_msg.hpp"

namespace uavnav::px4bridge_node {

namespace {

using events::Component;
using events::EventBuilder;
using namespace uavnav::px4bridge;

/// "stream" value of LioInputRejected and BridgeNoRosTime: the bridge's own numbering (bridge.hpp "input").
constexpr int kStreamOdometry = 0;
constexpr int kStreamHealth = 1;
constexpr int kStreamPx4 = 2;
constexpr int kStreamTick = 3;
/// "topic" value of BridgePublishFailed.
constexpr int kTopicEv = 0;
constexpr int kTopicAlignment = 1;

/// Rate limit of the node's per-message events: the 1st and every kEventSummaryEvery-th occurrence.
constexpr bool spam_due(std::uint64_t count) noexcept {
  return count == 1 || count % px4bridge::limits::kEventSummaryEvery == 0;
}

/// Increment of a counter with one writer at a time (the callbacks; see the header): returns the new value.
std::uint64_t bump(std::atomic<std::uint64_t>& c) noexcept {
  const std::uint64_t v = c.load(std::memory_order_relaxed) + 1;
  c.store(v, std::memory_order_relaxed);
  return v;
}

double as_double(const std::atomic<std::uint64_t>& c) noexcept { return static_cast<double>(c.load(std::memory_order_relaxed)); }

std::string describe(const std::string& path, const config::ConfigError& e) {
  return path + ": " + std::string(config::to_string(e.kind)) + (e.key.empty() ? "" : " '" + e.key + "'") +
         (e.detail.empty() ? "" : " (" + e.detail + ")");
}

/// The tick's ROS time as SensorTime: in SITL every stamp is the simulation /clock (D29), so the identity.
time::SensorTime sensor_time_of(const time::TimeSnapshot& now) noexcept { return time::SensorTime{now.ros.ns}; }

}  // namespace

// --- the callback gate -------------------------------------------------------------------------------------

/// Entered by every callback before it touches the bridge, the recorder or a publisher. Dekker-style on two
/// seq_cst atomics: the callback increments in_flight_ THEN reads accepting_; finish() clears accepting_ THEN
/// waits for in_flight_ == 0. In the single total order of these operations, either the callback reads false
/// (and touches nothing), or finish() sees it in flight and waits for its decrement, which happens-before
/// everything finish() does next. The callback never waits; RAII keeps the count right when an exception
/// leaves the callback.
class Px4BridgeNode::CallbackScope {
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

// --- start -------------------------------------------------------------------------------------------------

Px4BridgeNode::Px4BridgeNode(const rclcpp::NodeOptions& options) : rclcpp::Node(std::string(kNodeName), options) {
  const std::string config = path_parameter(kConfigFileParam);
  const std::string events = path_parameter(kEventsJsonlParam);
  start(config, events);
  RCLCPP_INFO(get_logger(), "px4_bridge started: %s=%s %s=%s", kConfigFileParam.data(), config.c_str(),
              kEventsJsonlParam.data(), events.c_str());
}

Px4BridgeNode::~Px4BridgeNode() { shutdown(); }

void Px4BridgeNode::fail(StartError e, const std::string& detail) {
  RCLCPP_FATAL(get_logger(), "px4_bridge refuses to start: %s: %s", std::string(to_string(e)).c_str(), detail.c_str());
  throw Px4BridgeNodeStartFailed(e, detail);
}

std::string Px4BridgeNode::path_parameter(std::string_view name) {
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

void Px4BridgeNode::start(const std::string& config, const std::string& events) {
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

  // (6) One tick before any subscription exists (the header). No callback can run yet, so no gate.
  const time::TimeSnapshot now = rosutil::snapshot(*get_clock());
  if (now.ros.ns != 0) tick(now, FirstTick::kConstructor);

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
  timer_ = create_timer(std::chrono::nanoseconds(px4bridge::limits::kTickPeriod.ns), [this] { on_timer(); }, group_);
}

// --- callbacks (the executor thread) -----------------------------------------------------------------------

void Px4BridgeNode::on_odometry(const uavnav_interfaces::msg::LioOdometry& msg) {
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

void Px4BridgeNode::on_health(const uavnav_interfaces::msg::LioHealth& msg) {
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

void Px4BridgeNode::on_local_position(const px4_msgs::msg::VehicleLocalPosition& msg) {
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

void Px4BridgeNode::on_timer() {
  const CallbackScope scope(*this);
  if (!scope) return;
  const auto now = take_snapshot(kStreamTick);
  if (!now) return;
  try {
    tick(*now, FirstTick::kTimer);
  } catch (...) {
    report_exception(kStreamTick, *now);
    throw;
  }
}

std::optional<time::TimeSnapshot> Px4BridgeNode::take_snapshot(int stream) {
  const time::TimeSnapshot now = rosutil::snapshot(*get_clock());
  if (now.ros.ns == 0) {
    // No ROS time yet (use_sim_time before the first /clock) or an unconvertible clock: a stamp would be refused
    // as ahead of a tick at 0.
    const std::uint64_t n = bump(no_ros_time_);
    if (spam_due(n)) {
      emit(EventBuilder(Component::kPx4Bridge, "BridgeNoRosTime", now)
               .reason(NodeReason::kNoRosTime)
               .value("stream", stream)
               .value("count_total", static_cast<double>(n)));
    }
    return std::nullopt;
  }
  return now;
}

void Px4BridgeNode::tick(const time::TimeSnapshot& now, FirstTick by) {
  if (first_tick_ == FirstTick::kNone) first_tick_ = by;
  bump(ticks_);
  publish(bridge_->on_tick(sensor_time_of(now), now), now);
}

void Px4BridgeNode::ensure_ticked(const time::TimeSnapshot& now) {
  if (first_tick_ == FirstTick::kNone) tick(now, FirstTick::kInput);
}

void Px4BridgeNode::reject(int stream, Counter& counter, std::string_view event, bool with_stream,
                           const time::TimeSnapshot& now) noexcept {
  const std::uint64_t n = bump(counter);
  if (!spam_due(n)) return;
  EventBuilder b(Component::kPx4Bridge, event, now);
  b.reason(BridgeReason::kNotFinite);
  if (with_stream) b.value("stream", stream);
  emit(b.value("count_total", static_cast<double>(n)));
}

// Converts and publishes one call's outputs, each message in its own failure scope: the EV first (the stream PX4
// is waiting for), then the alignment. An exception from one conversion or publish is counted and reported and
// never skips the other message.
void Px4BridgeNode::publish(const BridgeOutputs& out, const time::TimeSnapshot& now) noexcept {
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

void Px4BridgeNode::publish_failed(int topic, const time::TimeSnapshot& now) noexcept {
  const std::uint64_t n = bump(publish_failed_);
  if (spam_due(n)) {
    emit(EventBuilder(Component::kPx4Bridge, "BridgePublishFailed", now)
             .reason(NodeReason::kPublishFailed)
             .value("topic", topic)
             .value("count_total", static_cast<double>(n)));
  }
}

void Px4BridgeNode::report_exception(int stream, const time::TimeSnapshot& now) noexcept {
  emit(EventBuilder(Component::kPx4Bridge, "BridgeNodeFailed", now)
           .reason(NodeReason::kCallbackException)
           .value("stream", stream));
}

void Px4BridgeNode::emit(const EventBuilder& b) noexcept {
  if (recorder_) (void)recorder_->emit(b.build());
}

// --- shutdown ----------------------------------------------------------------------------------------------

void Px4BridgeNode::shutdown() noexcept {
  try {
    std::call_once(shutdown_once_, [this] { finish(); });
  } catch (...) {
    // std::call_once reports a failed once-flag (std::system_error); finish() itself never throws.
  }
}

void Px4BridgeNode::finish() noexcept {
  // (1) No callback enters any more; wait for the one in flight (the bridge does not wait; a reliable publish may,
  // see px4_bridge_node.hpp "Threads").
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
           .value("first_tick", static_cast<double>(static_cast<int>(first_tick_)))
           .value("ev_published", as_double(ev_published_))
           .value("alignment_published", as_double(alignment_published_))
           .value("publish_failed", as_double(publish_failed_)));
  // (4) The bridge (nothing runs in it any more), then the recorder: its destructor drains the ring into the
  // JSONL file and /events and joins the writer. The publishers and the node go with the object.
  bridge_.reset();
  recorder_.reset();
}

// --- the process -------------------------------------------------------------------------------------------

int run(int argc, char** argv) {
  rclcpp::init(argc, argv);
  std::shared_ptr<Px4BridgeNode> node;
  try {
    node = std::make_shared<Px4BridgeNode>(rclcpp::NodeOptions());
  } catch (const Px4BridgeNodeStartFailed&) {
    rclcpp::shutdown();  // the node logged its one RCLCPP_FATAL
    return EXIT_FAILURE;
  } catch (const std::exception& e) {
    RCLCPP_FATAL(rclcpp::get_logger(std::string(kNodeName)), "px4_bridge refuses to start: %s", e.what());
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

}  // namespace uavnav::px4bridge_node
