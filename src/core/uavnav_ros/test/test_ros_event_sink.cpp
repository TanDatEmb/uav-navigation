#include <gtest/gtest.h>

#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <memory>
#include <stdexcept>
#include <string>
#include <string_view>
#include <thread>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "uavnav/core/event.hpp"
#include "uavnav/core/event_recorder.hpp"
#include "uavnav/ros/detail/publish_batch.hpp"
#include "uavnav/ros/qos.hpp"
#include "uavnav/ros/ros_event_sink.hpp"
#include "uavnav_interfaces/msg/event_record.hpp"

using namespace uavnav;
using events::Component;

namespace {
using Msg = uavnav_interfaces::msg::EventRecord;
using Rec = events::EventRecord;

constexpr std::array<std::string_view, 16> kKeys = {"k0", "k1", "k2",  "k3",  "k4",  "k5",  "k6",  "k7",
                                                    "k8", "k9", "k10", "k11", "k12", "k13", "k14", "k15"};

std::atomic<int> g_node_counter{0};

// A subscriber on its own node, spun in the test thread. The wall clock is only a timeout (60 s): the
// tests wait for a condition, never for a duration.
class EventsFixture : public ::testing::Test {
 protected:
  void SetUp() override {
    const std::string suffix = std::to_string(g_node_counter.fetch_add(1));
    pub_node_ = std::make_shared<rclcpp::Node>("event_sink_pub_" + suffix);
    sub_node_ = std::make_shared<rclcpp::Node>("event_sink_sub_" + suffix);
    sub_ = sub_node_->create_subscription<Msg>("/events", rosutil::qos::events(),
                                               [this](Msg::UniquePtr m) { received_.push_back(*m); });
    executor_.add_node(sub_node_);
    auto sink = rosutil::RosEventSink::create(*pub_node_);
    ASSERT_TRUE(sink.has_value());
    sink_ = std::move(*sink);
    // Reliable + volatile: a message written before the two ends match is lost, so wait for both ends.
    ASSERT_TRUE(wait_until([this] { return sink_->subscription_count() > 0 && sub_->get_publisher_count() > 0; }));
  }

  template <class Pred>
  bool wait_until(Pred pred) {
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(60);
    while (!pred()) {
      if (std::chrono::steady_clock::now() > deadline) return false;
      executor_.spin_some(std::chrono::milliseconds(10));
    }
    return true;
  }

  std::shared_ptr<rclcpp::Node> pub_node_, sub_node_;
  rclcpp::Subscription<Msg>::SharedPtr sub_;
  rclcpp::executors::SingleThreadedExecutor executor_;
  std::unique_ptr<rosutil::RosEventSink> sink_;
  std::vector<Msg> received_;
};

std::vector<Rec> three_records() {
  std::vector<Rec> batch(3);

  Rec& a = batch[0];
  a.t_steady_ns = 1'000;
  a.t_ros_ns = 2'000;
  a.component = Component::kLio;
  a.event = "LioStarted";
  a.state_before = "INIT";
  a.state_after = "TRACKING";
  a.reason = "FIRST_SCAN";
  a.identity = {11, 7, 13, 17, 19};
  a.add_value("alpha", 1.5);

  Rec& b = batch[1];  // no values, no identity, empty strings
  b.t_steady_ns = 3'000;
  b.t_ros_ns = 0;
  b.component = Component::kSupervisor;
  b.event = "Tick";

  Rec& c = batch[2];  // the full 16 values, in a known order, with values that are easy to mix up
  c.t_steady_ns = std::int64_t{1} << 40;
  c.t_ros_ns = 1'700'000'000'123'456'789;
  c.component = Component::kPx4Bridge;
  c.event = "EvSent";
  c.state_before = "A";
  c.state_after = "B";
  c.reason = "OK";
  c.identity = {~std::uint64_t{0}, ~std::uint32_t{0}, 5, 6, 7};
  for (std::size_t i = 0; i < kKeys.size(); ++i) EXPECT_TRUE(c.add_value(kKeys[i], 100.0 + static_cast<double>(i) * 0.25));
  return batch;
}

void expect_matches(const Rec& r, const Msg& m) {
  EXPECT_EQ(m.t_steady_ns, r.t_steady_ns);
  EXPECT_EQ(m.t_ros_ns, r.t_ros_ns);
  EXPECT_EQ(m.component, std::string(events::to_string(r.component)));
  EXPECT_EQ(m.event, std::string(r.event));
  EXPECT_EQ(m.state_before, std::string(r.state_before));
  EXPECT_EQ(m.state_after, std::string(r.state_after));
  EXPECT_EQ(m.reason, std::string(r.reason));
  EXPECT_EQ(m.mission_id, r.identity.mission_id);
  EXPECT_EQ(m.lio_epoch, r.identity.lio_epoch);
  EXPECT_EQ(m.request_id, r.identity.request_id);
  EXPECT_EQ(m.bundle_id, r.identity.bundle_id);
  EXPECT_EQ(m.world_revision, r.identity.world_revision);
  ASSERT_EQ(m.value_keys.size(), r.value_count);
  ASSERT_EQ(m.values.size(), r.value_count);
  for (std::size_t i = 0; i < r.value_count; ++i) {
    EXPECT_EQ(m.value_keys[i], std::string(r.values[i].key)) << "value " << i;
    EXPECT_EQ(m.values[i], r.values[i].value) << "value " << i;
  }
}

}  // namespace

TEST_F(EventsFixture, PublishesEveryRecordFieldByField) {
  const std::vector<Rec> batch = three_records();
  ASSERT_EQ(batch[2].value_count, 16);

  const auto result = sink_->write(batch);
  ASSERT_TRUE(result.has_value());
  ASSERT_TRUE(wait_until([this] { return received_.size() >= 3; }));
  ASSERT_EQ(received_.size(), 3U);  // exactly one message per record, in order
  for (std::size_t i = 0; i < batch.size(); ++i) {
    SCOPED_TRACE("record " + std::to_string(i));
    expect_matches(batch[i], received_[i]);
  }
  EXPECT_EQ(received_[2].value_keys.size(), 16U);
  EXPECT_EQ(received_[2].values.back(), 100.0 + 15 * 0.25);
}

TEST_F(EventsFixture, NonFiniteValuesAreNotDropped) {
  Rec r;
  r.event = "NanValue";
  r.add_value("nan", std::nan(""));
  r.add_value("inf", HUGE_VAL);
  r.add_value("neg", -0.0);
  ASSERT_TRUE(sink_->write(std::span<const Rec>(&r, 1)).has_value());
  ASSERT_TRUE(wait_until([this] { return !received_.empty(); }));
  ASSERT_EQ(received_[0].values.size(), 3U);
  EXPECT_TRUE(std::isnan(received_[0].values[0]));
  EXPECT_TRUE(std::isinf(received_[0].values[1]));
  EXPECT_EQ(received_[0].value_keys[2], "neg");
}

TEST_F(EventsFixture, EmptyBatchPublishesNothing) {
  EXPECT_TRUE(sink_->write(std::span<const Rec>{}).has_value());
  // Flush the topic with a marker record: it is the only message that may arrive.
  Rec marker;
  marker.event = "Marker";
  ASSERT_TRUE(sink_->write(std::span<const Rec>(&marker, 1)).has_value());
  ASSERT_TRUE(wait_until([this] { return !received_.empty(); }));
  ASSERT_EQ(received_.size(), 1U);
  EXPECT_EQ(received_[0].event, "Marker");
}

// The production wiring: the recorder's writer thread, not the test thread, calls write() and publishes.
TEST_F(EventsFixture, PublishesFromTheRecordersWriterThread) {
  auto recorder = events::EventRecorder::create(std::move(sink_));
  ASSERT_TRUE(recorder.has_value());
  const std::vector<Rec> batch = three_records();
  for (const Rec& r : batch) ASSERT_TRUE((*recorder)->emit(r));
  (*recorder)->flush();
  ASSERT_TRUE(wait_until([this] { return received_.size() >= 3; }));
  ASSERT_EQ(received_.size(), 3U);
  for (std::size_t i = 0; i < batch.size(); ++i) expect_matches(batch[i], received_[i]);
  EXPECT_EQ((*recorder)->stats().sink_failures, 0U);
}

// A publish that throws comes back as kIo; nothing escapes, and the records before it were published.
// The batch loop is driven directly with a throwing publish function (no test hook in the product sink).
TEST(RosEventSink, PublishBatchReturnsIoWhenPublishThrows) {
  std::vector<std::string> published;
  int calls = 0;
  const auto publish = [&](const Msg& m) {
    if (++calls == 2) throw std::runtime_error("publish failed");
    published.push_back(m.event);
  };

  std::array<Rec, 3> batch{};
  batch[0].event = "first";
  batch[1].event = "second";
  batch[2].event = "third";
  Result<void, events::SinkError> result;
  EXPECT_NO_THROW(result = rosutil::detail::publish_batch(batch, publish));
  ASSERT_FALSE(result.has_value());
  EXPECT_EQ(result.error(), events::SinkError::kIo);
  EXPECT_EQ(published, std::vector<std::string>{"first"});

  // The next batch is attempted again (not latched into failure): the 3rd call does not throw.
  std::array<Rec, 1> next{};
  next[0].event = "again";
  EXPECT_TRUE(rosutil::detail::publish_batch(next, publish).has_value());
  EXPECT_EQ(published, (std::vector<std::string>{"first", "again"}));
}

namespace {

// A context of its own, so shutting it down does not touch the process-wide one.
std::shared_ptr<rclcpp::Context> private_context() {
  auto context = std::make_shared<rclcpp::Context>();
  rclcpp::InitOptions init_options;
  init_options.set_domain_id(215);
  context->init(0, nullptr, init_options);
  return context;
}

std::shared_ptr<rclcpp::Node> node_on(const std::shared_ptr<rclcpp::Context>& context, const std::string& base) {
  rclcpp::NodeOptions options;
  options.context(context);
  return std::make_shared<rclcpp::Node>(base + std::to_string(g_node_counter.fetch_add(1)), options);
}

}  // namespace

// rclcpp ignores a publish on a shut-down context without any error, so write() checks the context itself;
// through the recorder this shows up as one counted sink failure instead of a silent loss.
TEST(RosEventSink, WriteAfterContextShutdownReturnsIo) {
  const auto context = private_context();
  const auto node = node_on(context, "event_sink_dead_");
  auto sink = rosutil::RosEventSink::create(*node);
  ASSERT_TRUE(sink.has_value());
  Rec r;
  r.event = "BeforeShutdown";
  EXPECT_TRUE((*sink)->write(std::span<const Rec>(&r, 1)).has_value());

  auto recorder = events::EventRecorder::create(std::move(*sink));
  ASSERT_TRUE(recorder.has_value());
  context->shutdown("test");

  Rec after;
  after.event = "AfterShutdown";
  ASSERT_TRUE((*recorder)->emit(after));
  (*recorder)->flush();
  EXPECT_EQ((*recorder)->stats().sink_failures, 1U);
  EXPECT_EQ((*recorder)->stats().written, 0U);
}

TEST(RosEventSink, WriteReturnsIoOnADirectCallAfterShutdown) {
  const auto context = private_context();
  const auto node = node_on(context, "event_sink_dead_direct_");
  auto sink = rosutil::RosEventSink::create(*node);
  ASSERT_TRUE(sink.has_value());
  context->shutdown("test");
  Rec r;
  r.event = "AfterShutdown";
  Result<void, events::SinkError> result;
  EXPECT_NO_THROW(result = (*sink)->write(std::span<const Rec>(&r, 1)));
  ASSERT_FALSE(result.has_value());
  EXPECT_EQ(result.error(), events::SinkError::kIo);
}

TEST(RosEventSink, CreateFailsWhenTheNodesContextIsShutDown) {
  const auto context = private_context();
  const auto node = node_on(context, "event_sink_create_");
  context->shutdown("test");
  const auto sink = rosutil::RosEventSink::create(*node);
  ASSERT_FALSE(sink.has_value());
  EXPECT_EQ(sink.error(), rosutil::RosSinkError::kPublisherCreateFailed);
  EXPECT_EQ(rosutil::to_string(sink.error()), "PUBLISHER_CREATE_FAILED");
}
