#pragma once

#include <string_view>

namespace uav::nav::lio {

enum class DeskewMode {
  kSimultaneousScan,
  kPerPoint,
  kAuto,
};

enum class DeskewReference {
  kScanStart,
  kScanEnd,
};

[[nodiscard]] constexpr std::string_view toString(DeskewMode mode) noexcept {
  switch (mode) {
    case DeskewMode::kSimultaneousScan:
      return "simultaneous_scan";
    case DeskewMode::kPerPoint:
      return "per_point";
    case DeskewMode::kAuto:
      return "auto";
  }
  return "unknown";
}

}  // namespace uav::nav::lio
