#pragma once

#include <cstdint>
#include <optional>
#include <stdexcept>
#include <string>
#include <string_view>

namespace px4_odometry_bridge {

struct TimestampConversionResult {
  bool valid{false};
  bool suppressed{false};
  std::string source_domain{"UNRESOLVED"};
  std::string target_domain{"PX4_TIME_UNRESOLVED"};
  std::uint64_t measurement_time_us{0};
  std::uint64_t publication_time_us{0};
  std::uint64_t timestamp_mapping_generation{0};
  std::int64_t timestamp_age_ns{0};
  std::string reason{"TIME_DOMAIN_UNRESOLVED"};
};

struct TimestampConversionDiagnostics {
  // regression_count is retained as a compatibility aggregate. The
  // reason-specific counters below are the authoritative diagnostics.
  std::uint64_t regression_count{0};
  std::uint64_t timestamp_sample_regression_count{0};
  std::uint64_t publication_timestamp_regression_count{0};
  std::uint64_t duplicate_measurement_suppressed_count{0};
  std::uint64_t timestamp_mapping_generation{0};
  std::uint64_t timestamp_mapping_generation_change_count{0};
  std::uint64_t conversion_failure_count{0};
  std::string failure_reason{"NONE"};
};

// The PX4 uXRCE-DDS client owns the transport-side clock offset.  The
// navigation bridge must only declare which source domain it is publishing;
// it must not apply a second, locally estimated offset.
enum class TimestampMappingMode : std::uint8_t {
  kUnresolved,
  kSimulationIdentity,
  kRealtimeTransport,
};

[[nodiscard]] TimestampMappingMode timestampMappingModeFor(
    bool use_sim_time, std::string_view input_clock_domain) noexcept;

class TimestampConverter final {
 public:
  explicit TimestampConverter(std::int64_t maximum_age_ns)
      : maximum_age_ns_(maximum_age_ns) {
    if (maximum_age_ns_ <= 0) {
      throw std::invalid_argument("PX4 timestamp maximum age must be positive");
    }
  }

  [[nodiscard]] TimestampConversionResult convert(
      std::int64_t measurement_time_ns, std::int64_t publication_time_ns,
      TimestampMappingMode mapping_mode,
      std::uint64_t timestamp_mapping_generation);

  [[nodiscard]] TimestampConversionDiagnostics diagnostics() const noexcept {
    return diagnostics_;
  }

 private:
  std::int64_t maximum_age_ns_;
  std::optional<std::uint64_t> last_measurement_time_us_;
  std::optional<std::uint64_t> last_publication_time_us_;
  std::optional<std::uint64_t> last_timestamp_mapping_generation_;
  TimestampConversionDiagnostics diagnostics_;
};

}  // namespace px4_odometry_bridge
