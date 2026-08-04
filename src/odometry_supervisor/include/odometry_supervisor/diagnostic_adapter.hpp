#pragma once

#include <cstdint>
#include <string>
#include <unordered_map>

#include <diagnostic_msgs/msg/diagnostic_array.hpp>

namespace odometry_supervisor {

struct DiagnosticSnapshot {
  bool found{false};
  std::uint8_t level{diagnostic_msgs::msg::DiagnosticStatus::STALE};
  std::string message;
  std::unordered_map<std::string, std::string> values;
  std::int64_t stamp_ns{0};

  bool hasSchemaV1() const;
  bool boolean(const std::string& key, bool fallback = false) const;
  std::int64_t integer(const std::string& key, std::int64_t fallback = 0) const;
  std::uint64_t unsignedInteger(const std::string& key, std::uint64_t fallback = 0) const;
  double number(const std::string& key, double fallback = 0.0) const;
  std::string string(const std::string& key, std::string fallback = {}) const;
};

// P0.9 external publisher readiness is limited to the bridge node and PX4
// transport. Timestamp validity and supervisor authorization are independent
// publication gates, so this helper cannot form a cycle.
[[nodiscard]] bool externalPublisherReady(const DiagnosticSnapshot& snapshot,
                                          std::int64_t now_ns,
                                          std::int64_t maximum_age_ns);

DiagnosticSnapshot selectDiagnostic(
    const diagnostic_msgs::msg::DiagnosticArray& array, const std::string& name);

}  // namespace odometry_supervisor
