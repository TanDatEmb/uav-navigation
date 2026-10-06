#pragma once

#include <array>
#include <cstdint>
#include <string_view>
#include <type_traits>

#include "uavnav/core/limits.hpp"

// The structured event record (SYSTEM_DESIGN §6.1, AGENTS.md §2.3): one record per
// decision, carrying who decided, the state change, the reason and the values that
// drove it.
//
// Lifetime rule: every std::string_view in an EventRecord (event, state names,
// reason, value keys) must refer to STATIC storage: a string literal or a
// `to_string(enum)` result. Records are copied into the recorder's ring and written
// later on another thread, so a view of a local or heap string would dangle.
namespace uavnav::events {

enum class Component : std::uint8_t { kCore, kLio, kPx4Bridge, kMapping, kPlanner, kSupervisor, kPx4Mode };

constexpr std::string_view to_string(Component c) {
  switch (c) {
    case Component::kCore: return "core";
    case Component::kLio: return "lio";
    case Component::kPx4Bridge: return "px4_bridge";
    case Component::kMapping: return "mapping";
    case Component::kPlanner: return "planner";
    case Component::kSupervisor: return "supervisor";
    case Component::kPx4Mode: return "px4_mode";
  }
  return "unknown";  // unreachable for valid enumerators
}

/// One named number. `key` is a string literal (static storage only).
struct EventValue {
  std::string_view key;
  double value{0.0};
};

/// Correlation ids; 0 means "not applicable".
struct EventIdentity {
  std::uint64_t mission_id{0};
  std::uint32_t lio_epoch{0};
  std::uint64_t request_id{0};
  std::uint64_t bundle_id{0};
  std::uint64_t world_revision{0};
};

/// Fixed-size and trivially copyable, so emitting one never allocates.
struct EventRecord {
  std::int64_t t_steady_ns{0};
  std::int64_t t_ros_ns{0};
  Component component{Component::kCore};
  std::string_view event, state_before, state_after, reason;  // static storage only
  EventIdentity identity;
  std::array<EventValue, limits::kMaxEventValues> values{};
  std::uint8_t value_count{0};

  /// Appends a value. Returns false and leaves the record unchanged when full.
  bool add_value(std::string_view key, double v) noexcept {
    if (value_count >= values.size()) return false;
    values[value_count++] = EventValue{key, v};
    return true;
  }
};

static_assert(limits::kMaxEventValues <= UINT8_MAX, "value_count is a uint8_t");
static_assert(std::is_trivially_copyable_v<EventRecord>);

}  // namespace uavnav::events
