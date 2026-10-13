#pragma once

#include <algorithm>
#include <cstddef>
#include <span>
#include <string>
#include <utility>

#include "uavnav/core/event.hpp"
#include "uavnav/core/event_recorder.hpp"
#include "uavnav/core/result.hpp"
#include "uavnav_interfaces/msg/event_record.hpp"

// Internals of RosEventSink, header-only so a test can drive the batch loop with its own publish
// function (for instance one that throws) without any test hook in the product class (AGENTS §2.8).
namespace uavnav::rosutil::detail {

/// events::EventRecord -> uavnav_interfaces/msg/EventRecord. Strings are copied (the record holds views of
/// static storage, the message owns std::string); value_keys[i] / values[i] are the record's values in
/// order, all value_count of them.
inline uavnav_interfaces::msg::EventRecord to_message(const events::EventRecord& r) {
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

/// Converts each record and hands it to `publish(const Msg&)`, in order. An exception from the conversion or
/// from `publish` is caught and returned as kIo; the records before the failing one were published and the
/// rest of the batch is not attempted. Never throws.
template <class Publish>
Result<void, events::SinkError> publish_batch(std::span<const events::EventRecord> batch, Publish&& publish) noexcept {
  try {
    for (const events::EventRecord& record : batch) {
      const uavnav_interfaces::msg::EventRecord message = to_message(record);
      publish(message);
    }
  } catch (...) {
    return std::unexpected(events::SinkError::kIo);
  }
  return {};
}

}  // namespace uavnav::rosutil::detail
