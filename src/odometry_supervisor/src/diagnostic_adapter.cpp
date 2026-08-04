#include "odometry_supervisor/diagnostic_adapter.hpp"

#include <charconv>
#include <utility>

namespace odometry_supervisor {

bool DiagnosticSnapshot::hasSchemaV1() const { return string("diagnostic_schema_version") == "1"; }

bool DiagnosticSnapshot::boolean(const std::string& key, bool fallback) const {
  const auto value = string(key);
  if (value == "true") return true;
  if (value == "false") return false;
  return fallback;
}

std::int64_t DiagnosticSnapshot::integer(const std::string& key, std::int64_t fallback) const {
  const auto value = string(key);
  if (value.empty()) return fallback;
  std::int64_t result = fallback;
  const auto parsed = std::from_chars(value.data(), value.data() + value.size(), result);
  return parsed.ec == std::errc{} ? result : fallback;
}

std::uint64_t DiagnosticSnapshot::unsignedInteger(const std::string& key,
                                                  std::uint64_t fallback) const {
  const auto value = string(key);
  if (value.empty()) return fallback;
  std::uint64_t result = fallback;
  const auto parsed = std::from_chars(value.data(), value.data() + value.size(), result);
  return parsed.ec == std::errc{} ? result : fallback;
}

double DiagnosticSnapshot::number(const std::string& key, double fallback) const {
  const auto value = string(key);
  if (value.empty()) return fallback;
  try {
    return std::stod(value);
  } catch (...) {
    return fallback;
  }
}

std::string DiagnosticSnapshot::string(const std::string& key, std::string fallback) const {
  const auto iterator = values.find(key);
  return iterator == values.end() ? std::move(fallback) : iterator->second;
}

bool externalPublisherReady(const DiagnosticSnapshot& snapshot,
                            const std::int64_t now_ns,
                            const std::int64_t maximum_age_ns) {
  return snapshot.found && snapshot.string("diagnostic_schema_version") == "2" &&
         snapshot.boolean("publisher_ready") && snapshot.stamp_ns > 0 &&
         now_ns >= snapshot.stamp_ns &&
         now_ns - snapshot.stamp_ns <= maximum_age_ns;
}

DiagnosticSnapshot selectDiagnostic(
    const diagnostic_msgs::msg::DiagnosticArray& array, const std::string& name) {
  DiagnosticSnapshot result;
  result.stamp_ns = static_cast<std::int64_t>(array.header.stamp.sec) * 1'000'000'000LL +
                    static_cast<std::int64_t>(array.header.stamp.nanosec);
  for (const auto& status : array.status) {
    if (status.name != name) continue;
    result.found = true;
    result.level = status.level;
    result.message = status.message;
    for (const auto& value : status.values) result.values[value.key] = value.value;
    break;
  }
  return result;
}

}  // namespace odometry_supervisor
