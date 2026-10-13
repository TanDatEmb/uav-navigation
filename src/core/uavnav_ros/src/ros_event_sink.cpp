#include "uavnav/ros/ros_event_sink.hpp"

#include <utility>

#include "uavnav/ros/detail/publish_batch.hpp"
#include "uavnav/ros/qos.hpp"

namespace uavnav::rosutil {

RosEventSink::RosEventSink(rclcpp::Publisher<Msg>::SharedPtr publisher, rclcpp::Context::SharedPtr context) noexcept
    : publisher_(std::move(publisher)), context_(std::move(context)) {}

Result<std::unique_ptr<RosEventSink>, RosSinkError> RosEventSink::create(rclcpp::Node& node) {
  try {
    rclcpp::Context::SharedPtr context = node.get_node_base_interface()->get_context();
    if (!context || !context->is_valid()) return std::unexpected(RosSinkError::kPublisherCreateFailed);
    auto publisher = node.create_publisher<Msg>("/events", qos::events());
    return std::unique_ptr<RosEventSink>(new RosEventSink(std::move(publisher), std::move(context)));
  } catch (...) {
    return std::unexpected(RosSinkError::kPublisherCreateFailed);
  }
}

Result<void, events::SinkError> RosEventSink::write(std::span<const events::EventRecord> batch) {
  if (!context_->is_valid()) return std::unexpected(events::SinkError::kIo);
  return detail::publish_batch(batch, [this](const Msg& message) { publisher_->publish(message); });
}

std::size_t RosEventSink::subscription_count() const noexcept {
  try {
    return publisher_->get_subscription_count();
  } catch (...) {
    return 0;
  }
}

}  // namespace uavnav::rosutil
