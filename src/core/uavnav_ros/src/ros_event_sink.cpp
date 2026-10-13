#include "uavnav/ros/ros_event_sink.hpp"

#include <algorithm>
#include <string>
#include <utility>

#include "uavnav/ros/qos.hpp"

namespace uavnav::rosutil {

namespace {

uavnav_interfaces::msg::EventRecord to_message(const events::EventRecord& r) {
  uavnav_interfaces::msg::EventRecord m;
  m.t_steady_ns = r.t_steady_ns;
  m.t_ros_ns = r.t_ros_ns;
  m.component = std::string(events::to_string(r.component));
  m.event = std::string(r.event);
  m.state_before = std::string(r.state_before);
  m.state_after = std::string(r.state_after);
  m.reason = std::string(r.reason);
  m.mission_id = r.identity.mission_id;
  m.lio_epoch = r.identity.lio_epoch;
  m.request_id = r.identity.request_id;
  m.bundle_id = r.identity.bundle_id;
  m.world_revision = r.identity.world_revision;
  // value_count never exceeds the array (add_value checks it); min() keeps a corrupt count from reading past it.
  const std::size_t n = std::min<std::size_t>(r.value_count, r.values.size());
  m.value_keys.reserve(n);
  m.values.reserve(n);
  for (std::size_t i = 0; i < n; ++i) {
    m.value_keys.emplace_back(r.values[i].key);
    m.values.push_back(r.values[i].value);
  }
  return m;
}

}  // namespace

RosEventSink::RosEventSink(rclcpp::Publisher<Msg>::SharedPtr publisher,
                           std::function<void(const Msg&)> publish_override) noexcept
    : publisher_(std::move(publisher)), publish_override_(std::move(publish_override)) {}

std::unique_ptr<RosEventSink> RosEventSink::create(rclcpp::Node& node) {
  try {
    return std::unique_ptr<RosEventSink>(new RosEventSink(node.create_publisher<Msg>("/events", qos::events()), nullptr));
  } catch (...) {
    return nullptr;
  }
}

std::unique_ptr<RosEventSink> RosEventSink::create_with_publish_function(std::function<void(const Msg&)> publish) {
  return std::unique_ptr<RosEventSink>(new RosEventSink(nullptr, std::move(publish)));
}

Result<void, events::SinkError> RosEventSink::write(std::span<const events::EventRecord> batch) {
  try {
    for (const events::EventRecord& record : batch) {
      Msg message = to_message(record);
      if (publish_override_) {
        publish_override_(message);
      } else {
        publisher_->publish(std::move(message));
      }
    }
  } catch (...) {
    return std::unexpected(events::SinkError::kIo);
  }
  return {};
}

std::size_t RosEventSink::subscription_count() const noexcept {
  if (!publisher_) return 0;
  try {
    return publisher_->get_subscription_count();
  } catch (...) {
    return 0;
  }
}

}  // namespace uavnav::rosutil
