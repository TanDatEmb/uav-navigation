#pragma once

#include <cstdint>

namespace odometry_supervisor {

enum class QueryFailureKind : std::uint8_t {
  kTransport,
  kGenerationMismatch,
  kGeometric,
  kContract,
};

// A failed request is accounted for exactly once at its terminal failure
// classification.  Transport, generation, and geometric counters are
// subsets of total failures; a plain contract rejection only increments the
// total.
struct QueryFailureCounters {
  std::uint64_t failure_count{0};
  std::uint64_t transport_failure_count{0};
  std::uint64_t generation_mismatch_count{0};
  std::uint64_t geometric_failure_count{0};

  void record(QueryFailureKind kind) noexcept {
    ++failure_count;
    switch (kind) {
      case QueryFailureKind::kTransport:
        ++transport_failure_count;
        break;
      case QueryFailureKind::kGenerationMismatch:
        ++generation_mismatch_count;
        break;
      case QueryFailureKind::kGeometric:
        ++geometric_failure_count;
        break;
      case QueryFailureKind::kContract:
        break;
    }
  }
};

}  // namespace odometry_supervisor
