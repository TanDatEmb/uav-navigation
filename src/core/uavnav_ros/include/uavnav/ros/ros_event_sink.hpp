#pragma once

#include <cstddef>
#include <cstdint>
#include <memory>
#include <span>
#include <string_view>

#include "rclcpp/context.hpp"
#include "rclcpp/node.hpp"
#include "rclcpp/publisher.hpp"
#include "uavnav/core/event.hpp"
#include "uavnav/core/event_recorder.hpp"
#include "uavnav/core/result.hpp"
#include "uavnav_interfaces/msg/event_record.hpp"

namespace uavnav::rosutil {

enum class RosSinkError : std::uint8_t { kPublisherCreateFailed };

constexpr std::string_view to_string(RosSinkError e) {
  switch (e) {
    case RosSinkError::kPublisherCreateFailed: return "PUBLISHER_CREATE_FAILED";
  }
  return "unknown";  // unreachable for valid enumerators
}
static_assert(ReasonEnum<RosSinkError>);

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
/// - Failure: an exception from conversion or publish (allocation failure, ...) is caught and returned as
///   SinkError::kIo; nothing escapes (AGENTS §2.6). So is a write after the node's rclcpp context was shut
///   down: rclcpp ignores such a publish silently, which would lose the batch without a trace, so write()
///   checks the context first. The recorder counts either as one sink failure. The check is not atomic with
///   the publish: a context shut down between is_valid() and publish() still loses that batch silently (TOCTOU).
/// - Lifetime: the sink keeps the publisher and the node's context alive, not the node. Destroy the
///   EventRecorder (and with it the sink; its destructor drains the ring into write()) BEFORE the node and
///   before rclcpp::shutdown(), or the last batch is reported as failed.
class RosEventSink final : public events::EventSink {
 public:
  /// Creates the publisher on `node`. Fails with kPublisherCreateFailed when the node's context is no
  /// longer valid or rclcpp throws while creating the publisher.
  static Result<std::unique_ptr<RosEventSink>, RosSinkError> create(rclcpp::Node& node);

  /// kIo if the context is shut down or any record could not be converted or published. The records before
  /// the failing one were published; the rest of the batch is not attempted.
  Result<void, events::SinkError> write(std::span<const events::EventRecord> batch) override;

  /// Number of subscriptions matched to the publisher (rmw graph view). Read-only, for diagnostics and tests.
  std::size_t subscription_count() const noexcept;

 private:
  using Msg = uavnav_interfaces::msg::EventRecord;
  RosEventSink(rclcpp::Publisher<Msg>::SharedPtr publisher, rclcpp::Context::SharedPtr context) noexcept;

  rclcpp::Publisher<Msg>::SharedPtr publisher_;
  rclcpp::Context::SharedPtr context_;
};

}  // namespace uavnav::rosutil
