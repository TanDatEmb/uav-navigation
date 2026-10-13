#include "uavnav/lio_node/lio_node.hpp"

#include <chrono>
#include <exception>
#include <thread>
#include <utility>

#include "rcl_interfaces/msg/parameter_descriptor.hpp"
#include "rclcpp/logging.hpp"
#include "uavnav/core/config.hpp"
#include "uavnav/core/jsonl_sink.hpp"
#include "uavnav/core/tee_sink.hpp"
#include "uavnav/lio/config.hpp"
#include "uavnav/lio/limits.hpp"
#include "uavnav/lio_node/detail/publish_outputs.hpp"
#include "uavnav/lio_node/node_config.hpp"
#include "uavnav/ros/qos.hpp"
#include "uavnav/ros/ros_event_sink.hpp"
#include "uavnav/ros/time_msg.hpp"

namespace uavnav::lio_node {

namespace {

using events::Component;
using events::EventBuilder;

/// "stream" value of LioInputRejected (the frontend's numbering: 0 IMU, 1 scan).
constexpr int kStreamImu = 0;
constexpr int kStreamScan = 1;

/// Rate limit of the per-sample events: the 1st and every kEventSummaryEvery-th occurrence (the frontend's).
constexpr bool spam_due(std::uint64_t count) noexcept {
  return count == 1 || count % lio::limits::kEventSummaryEvery == 0;
}

/// Increment of a counter with one writer at a time (the callbacks; see the header): returns the new value.
std::uint64_t bump(std::atomic<std::uint64_t>& c, std::uint64_t by = 1) noexcept {
  const std::uint64_t v = c.load(std::memory_order_relaxed) + by;
  c.store(v, std::memory_order_relaxed);
  return v;
}

double as_double(const std::atomic<std::uint64_t>& c) noexcept {
  return static_cast<double>(c.load(std::memory_order_relaxed));
}

std::string describe(const std::string& path, const config::ConfigError& e) {
  return path + ": " + std::string(config::to_string(e.kind)) + (e.key.empty() ? "" : " '" + e.key + "'") +
         (e.detail.empty() ? "" : " (" + e.detail + ")");
}

}  // namespace

// --- the callback gate -------------------------------------------------------------------------------------

/// Entered by every callback before it touches the ingest, the recorder or a publisher. Dekker-style on two
/// seq_cst atomics: the callback increments in_flight_ THEN reads accepting_; finish() clears accepting_ THEN
/// waits for in_flight_ == 0. In the single total order of these operations, either the callback reads false
/// (and touches nothing), or finish() sees it in flight and waits for its decrement, which happens-before
/// everything finish() does next. The callback never waits; RAII keeps the count right when an exception
/// leaves the callback.
class LioNode::CallbackScope {
 public:
  explicit CallbackScope(LioNode& node) noexcept : node_(node) {
    node_.in_flight_.fetch_add(1, std::memory_order_seq_cst);
    entered_ = node_.accepting_.load(std::memory_order_seq_cst);
  }
  ~CallbackScope() { node_.in_flight_.fetch_sub(1, std::memory_order_seq_cst); }
  CallbackScope(const CallbackScope&) = delete;
  CallbackScope& operator=(const CallbackScope&) = delete;
  explicit operator bool() const noexcept { return entered_; }

 private:
  LioNode& node_;
  bool entered_;
};

// --- start -------------------------------------------------------------------------------------------------

LioNode::LioNode(const rclcpp::NodeOptions& options) : rclcpp::Node(std::string(kNodeName), options) {
  const std::string config = path_parameter(kConfigFileParam);
  const std::string node_config = path_parameter(kNodeConfigFileParam);
  const std::string events = path_parameter(kEventsJsonlParam);
  start(config, node_config, events);
  RCLCPP_INFO(get_logger(), "lio started: %s=%s %s=%s %s=%s", kConfigFileParam.data(), config.c_str(),
              kNodeConfigFileParam.data(), node_config.c_str(), kEventsJsonlParam.data(), events.c_str());
}

LioNode::~LioNode() { shutdown(); }

void LioNode::fail(StartError e, const std::string& detail) {
  RCLCPP_FATAL(get_logger(), "lio refuses to start: %s: %s", std::string(to_string(e)).c_str(), detail.c_str());
  throw LioNodeStartFailed(e, detail);
}

std::string LioNode::path_parameter(std::string_view name) {
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

void LioNode::start(const std::string& config, const std::string& node_config, const std::string& events) {
  // (1) Both parameter files, before anything starts a thread.
  const auto lio_values = config::load_params_file(config, lio::kLioSpecs);
  if (!lio_values) fail(StartError::kConfigInvalid, describe(config, lio_values.error()));
  const auto lio_config = lio::load_lio_config(*lio_values);
  if (!lio_config) fail(StartError::kConfigInvalid, describe(config, lio_config.error()));
  const auto node_values = config::load_params_file(node_config, kLioNodeSpecs);
  if (!node_values) fail(StartError::kNodeConfigInvalid, describe(node_config, node_values.error()));
  const auto mount = load_lio_node_config(*node_values);
  if (!mount) fail(StartError::kNodeConfigInvalid, describe(node_config, mount.error()));

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

  // (3) LioIngest, which starts LioRunner's estimator thread.
  auto ingest = LioIngest::create(*lio_config, *mount, *recorder_, ros_time_);
  if (!ingest) fail(StartError::kIngestFailed, describe(config, ingest.error()));
  ingest_ = std::move(*ingest);

  // (4) Publishers (QoS O21), then both subscriptions in ONE MutuallyExclusive group.
  state_pub_ =
      create_publisher<uavnav_interfaces::msg::LioState>(std::string(kStateTopic), rosutil::qos::state_output());
  health_pub_ =
      create_publisher<uavnav_interfaces::msg::LioHealth>(std::string(kHealthTopic), rosutil::qos::reliable_output());
  odometry_pub_ = create_publisher<uavnav_interfaces::msg::LioOdometry>(std::string(kOdometryTopic),
                                                                        rosutil::qos::reliable_output());
  group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  rclcpp::SubscriptionOptions options;
  options.callback_group = group_;
  accepting_.store(true, std::memory_order_seq_cst);  // before the first callback can be taken
  imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
      std::string(kImuTopic), rosutil::qos::sensor_input(), [this](const sensor_msgs::msg::Imu& m) { on_imu(m); },
      options);
  points_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      std::string(kPointsTopic), rosutil::qos::sensor_input(),
      [this](const sensor_msgs::msg::PointCloud2& m) { on_points(m); }, options);
}

// --- callbacks (the ingest thread) -------------------------------------------------------------------------

void LioNode::on_imu(const sensor_msgs::msg::Imu& msg) {
  const CallbackScope scope(*this);
  if (!scope) return;
  bump(imu_received_);
  const auto now = take_snapshot(kStreamImu);
  if (!now) return;
  try {
    const auto imu = to_imu_input(msg);
    if (!imu) return reject(imu.error(), kStreamImu, *now);
    const auto out = ingest_->on_imu(*imu, *now);
    if (!out) {
      bump(ingest_refused_);  // the ingest (or the core) emitted the event of its refusal
      return;
    }
    publish(*out, *now);
  } catch (...) {
    report_exception(kStreamImu, *now);
    throw;
  }
}

void LioNode::on_points(const sensor_msgs::msg::PointCloud2& msg) {
  const CallbackScope scope(*this);
  if (!scope) return;
  bump(scans_received_);
  const auto now = take_snapshot(kStreamScan);
  if (!now) return;
  try {
    CloudStats stats{0, 0};
    auto scan = to_scan_input(msg, stats);
    if (!scan) return reject(scan.error(), kStreamScan, *now);
    bump(scans_converted_);
    bump(points_in_, stats.points_in);
    bump(points_non_finite_, stats.points_non_finite);
    const auto out = ingest_->on_scan(std::move(*scan), *now);
    if (!out) {
      bump(ingest_refused_);
      return;
    }
    publish(*out, *now);
  } catch (...) {
    report_exception(kStreamScan, *now);
    throw;
  }
}

std::optional<time::TimeSnapshot> LioNode::take_snapshot(int stream) {
  const time::TimeSnapshot now = rosutil::snapshot(*get_clock());
  if (now.ros.ns == 0) {
    // No ROS time yet (use_sim_time before the first /clock) or an unconvertible clock: the ingest would refuse
    // every stamp as in the future and rewind the backend clock to 0 (ingest.hpp "ROS time zero").
    const std::uint64_t n = bump(no_ros_time_);
    if (spam_due(n)) {
      emit(EventBuilder(Component::kLio, "LioInputRejected", now)
               .reason(NodeReason::kNoRosTime)
               .value("stream", stream)
               .value("count", static_cast<double>(n)));
    }
    return std::nullopt;
  }
  ros_time_.store(now.ros);  // the backend clock follows every callback, also one refused below
  return now;
}

void LioNode::reject(ConversionError e, int stream, const time::TimeSnapshot& now) noexcept {
  const std::uint64_t n = bump(rejected_[static_cast<std::size_t>(e)]);
  if (spam_due(n)) {
    emit(EventBuilder(Component::kLio, "LioInputRejected", now)
             .reason(e)
             .value("stream", stream)
             .value("count", static_cast<double>(n)));
  }
}

void LioNode::publish(const IngestOutputs& out, const time::TimeSnapshot& now) noexcept {
  detail::publish_outputs(
      out,
      [this](const uavnav_interfaces::msg::LioState& m) {
        state_pub_->publish(m);
        bump(states_published_);
      },
      [this](const uavnav_interfaces::msg::LioHealth& m) {
        health_pub_->publish(m);
        bump(health_published_);
      },
      [this](const uavnav_interfaces::msg::LioOdometry& m) {
        odometry_pub_->publish(m);
        bump(odometry_published_);
      },
      [this, &now](int topic) noexcept {
        const std::uint64_t n = bump(publish_failed_);
        if (spam_due(n)) {
          emit(EventBuilder(Component::kLio, "LioPublishFailed", now)
                   .reason(NodeReason::kPublishFailed)
                   .value("topic", topic)
                   .value("count", static_cast<double>(n)));
        }
      });
}

void LioNode::report_exception(int stream, const time::TimeSnapshot& now) noexcept {
  emit(EventBuilder(Component::kLio, "LioNodeFailed", now)
           .reason(NodeReason::kCallbackException)
           .value("stream", stream));
}

void LioNode::emit(const events::EventBuilder& b) noexcept {
  if (recorder_) (void)recorder_->emit(b.build());
}

// --- shutdown ----------------------------------------------------------------------------------------------

void LioNode::shutdown() noexcept {
  try {
    std::call_once(shutdown_once_, [this] { finish(); });
  } catch (...) {
    // std::call_once reports a failed once-flag (std::system_error); finish() itself never throws.
  }
}

void LioNode::finish() noexcept {
  // (1) No callback enters any more; wait for the one in flight (it never blocks: ingest and publish do not wait).
  accepting_.store(false, std::memory_order_seq_cst);
  while (in_flight_.load(std::memory_order_seq_cst) != 0) std::this_thread::sleep_for(std::chrono::milliseconds(1));

  // (2) The node's counters, before the ingest's summary (the JSONL file ends with IngestSummary).
  const time::TimeSnapshot now = rosutil::snapshot(*get_clock());
  const auto rejected = [this](ConversionError e) { return as_double(rejected_[static_cast<std::size_t>(e)]); };
  emit(EventBuilder(Component::kLio, "LioNodeSummary", now)
           .reason(NodeReason::kShutdown)
           .value("imu_received", as_double(imu_received_))
           .value("scans_received", as_double(scans_received_))
           .value("scans_converted", as_double(scans_converted_))
           .value("points_in", as_double(points_in_))
           .value("points_non_finite", as_double(points_non_finite_))
           .value("rejected_bad_stamp", rejected(ConversionError::kBadStamp))
           .value("rejected_not_finite", rejected(ConversionError::kNotFinite))
           .value("rejected_wrong_frame", rejected(ConversionError::kWrongFrame))
           .value("rejected_malformed_cloud", rejected(ConversionError::kMalformedCloud))
           .value("rejected_no_ros_time", as_double(no_ros_time_))
           .value("ingest_refused", as_double(ingest_refused_))
           .value("states_published", as_double(states_published_))
           .value("health_published", as_double(health_published_))
           .value("odometry_published", as_double(odometry_published_))
           .value("publish_failed", as_double(publish_failed_)));

  // (3) LioRunner::stop joins the estimator thread before its channels go; then "IngestSummary".
  if (ingest_) ingest_->stop();
  // (4) The ingest (nothing runs in it any more), then the recorder: its destructor drains the ring into the
  // JSONL file and /events and joins the writer. The publishers and the node go with the LioNode object.
  ingest_.reset();
  recorder_.reset();
}

}  // namespace uavnav::lio_node
