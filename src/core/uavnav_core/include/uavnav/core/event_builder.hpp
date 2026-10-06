#pragma once

#include <string_view>

#include "uavnav/core/event.hpp"
#include "uavnav/core/result.hpp"
#include "uavnav/core/time.hpp"

namespace uavnav::events {

/// Fills an EventRecord with typed arguments (AGENTS.md §2.3: one structured event per
/// decision). Typed state and reason enums are stored by name, so a call site cannot
/// log a free-form or dangling string; the clocks come from one TimeSnapshot, so both
/// stamps describe the same instant.
///
/// Static storage only: `event` and every value key must be string literals (or other
/// static views). State and reason names come from `to_string(ReasonEnum)`, which is
/// static by the ReasonEnum contract. The record is copied into the recorder's ring
/// and written later on another thread.
class EventBuilder {
 public:
  EventBuilder(Component c, std::string_view event, const time::TimeSnapshot& now) noexcept {
    record_.t_steady_ns = now.steady.ns;
    record_.t_ros_ns = now.ros.ns;
    record_.component = c;
    record_.event = event;
  }

  template <ReasonEnum S>
  EventBuilder& states(S before, S after) noexcept {
    record_.state_before = to_string(before);
    record_.state_after = to_string(after);
    return *this;
  }

  template <ReasonEnum R>
  EventBuilder& reason(R r) noexcept {
    record_.reason = to_string(r);
    return *this;
  }

  EventBuilder& identity(const EventIdentity& id) noexcept {
    record_.identity = id;
    return *this;
  }

  /// Adds a named number. A duplicate key or the 17th value is ignored (the record
  /// keeps the first), because add_value() returns false and leaves the record unchanged.
  EventBuilder& value(std::string_view key, double v) noexcept {
    (void)record_.add_value(key, v);
    return *this;
  }

  EventRecord build() const noexcept { return record_; }

 private:
  EventRecord record_;
};

}  // namespace uavnav::events
