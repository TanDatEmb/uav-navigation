#pragma once

#include <cstddef>
#include <functional>
#include <memory>
#include <span>

#include "rclcpp/rclcpp.hpp"
#include "uavnav/core/event.hpp"
#include "uavnav/core/event_recorder.hpp"
#include "uavnav_interfaces/msg/event_record.hpp"

namespace uavnav::rosutil {

/// Publishes event batches as uavnav_interfaces/msg/EventRecord on `/events` (SYSTEM_DESIGN §6.1, §6.5)
/// with qos::events(). Combine it with the JSONL sink in a TeeSink so one EventRecorder per process both
/// writes the file and publishes.
///
/// Contract:
/// - One message per record, in batch order. Strings are copied (the record's views are static, the message
///   owns std::string); `component` is to_string(Component); value_keys[i] / values[i] are the record's
///   values in order, all value_count of them, NaN and inf included. Nothing is dropped silently.
/// - Threading: write() runs on the recorder's single writer thread, not the node's thread. rclcpp
///   publishers are safe to publish from any thread, and this sink holds no lock of its own, so a publish
///   never blocks other event emitters. The publisher is created in create() on the caller's (node's) thread.
/// - Failure: an exception from conversion or publish (allocation failure, a shut-down context) is caught
///   and returned as SinkError::kIo; nothing escapes (AGENTS §2.6). The recorder counts it and goes on.
class RosEventSink final : public events::EventSink {
 public:
  /// Creates the publisher on `node`. Returns nullptr when the publisher cannot be created (the exception
  /// is swallowed here); EventRecorder/TeeSink::create reject a null sink, so a caller cannot miss it.
  static std::unique_ptr<RosEventSink> create(rclcpp::Node& node);

  /// Test seam: a sink whose "publish" is `publish` instead of a publisher (no node, no `/events`), so a
  /// test can make the publish step throw. Everything else (conversion, kIo mapping) is the real code.
  static std::unique_ptr<RosEventSink> create_with_publish_function(
      std::function<void(const uavnav_interfaces::msg::EventRecord&)> publish);

  /// kIo if any record could not be converted or published. The records before the failing one were
  /// published; the rest of the batch is not attempted.
  Result<void, events::SinkError> write(std::span<const events::EventRecord> batch) override;

  /// Number of subscriptions matched to the publisher (rmw graph view). For tests and diagnostics only.
  std::size_t subscription_count() const noexcept;

 private:
  using Msg = uavnav_interfaces::msg::EventRecord;
  RosEventSink(rclcpp::Publisher<Msg>::SharedPtr publisher, std::function<void(const Msg&)> publish_override) noexcept;

  rclcpp::Publisher<Msg>::SharedPtr publisher_;                // null for the test seam
  std::function<void(const Msg&)> publish_override_;           // empty in production
};

}  // namespace uavnav::rosutil
