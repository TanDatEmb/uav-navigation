#pragma once

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <memory>
#include <vector>

#include <navigation_world_model/world_model_view.hpp>

namespace navigation_mapping {

enum class TraversedChainResetReason : std::uint8_t {
  kNone = 0,
  kFirstPose,
  kEpochChanged,
  kTimestampRegression,
  kSourceGap,
  kImplausibleMotion,
};

struct TraversedFreeSegment {
  navigation_world_model::Point3 start{navigation_world_model::Point3::Zero()};
  navigation_world_model::Point3 end{navigation_world_model::Point3::Zero()};
  std::uint64_t localization_epoch{0};
  std::int64_t source_stamp_ns{0};
  std::int64_t last_traversed_stamp_ns{0};
  double support_radius_m{0.0};
};

// Immutable, bounded geometric provenance.  This is intentionally not an
// occupancy/probability map.  A point support (radius=0) is the conservative
// default until a separately verified vehicle hull is provided.
class TraversedFreeSpace final {
 public:
  using Segment = TraversedFreeSegment;
  using SegmentVector = std::vector<Segment>;

  TraversedFreeSpace() = default;
  TraversedFreeSpace(SegmentVector segments, const double max_age_s)
      : segments_(std::move(segments)), max_age_s_(max_age_s) {}

  [[nodiscard]] const SegmentVector& segments() const noexcept { return segments_; }
  [[nodiscard]] double maxAgeSeconds() const noexcept { return max_age_s_; }

  [[nodiscard]] bool contains(const navigation_world_model::Point3& point,
                              const std::uint64_t epoch,
                              const std::int64_t now_stamp_ns,
                              const double tolerance_m = 0.0) const noexcept {
    if (!point.allFinite() || epoch == 0U || now_stamp_ns <= 0 ||
        !std::isfinite(max_age_s_) || max_age_s_ < 0.0) {
      return false;
    }
    const auto max_age_ns = static_cast<long double>(max_age_s_) * 1.0e9L;
    for (const auto& segment : segments_) {
      if (segment.localization_epoch != epoch ||
          segment.last_traversed_stamp_ns <= 0 ||
          now_stamp_ns < segment.last_traversed_stamp_ns ||
          static_cast<long double>(now_stamp_ns - segment.last_traversed_stamp_ns) >
              max_age_ns) {
        continue;
      }
      if (!segment.start.allFinite() || !segment.end.allFinite() ||
          !std::isfinite(segment.support_radius_m) || segment.support_radius_m < 0.0) {
        continue;
      }
      const auto delta = segment.end - segment.start;
      const double length_squared = delta.squaredNorm();
      double projection = 0.0;
      if (std::isfinite(length_squared) && length_squared > 1.0e-12) {
        projection = (point - segment.start).dot(delta) / length_squared;
        projection = std::clamp(projection, 0.0, 1.0);
      }
      const auto closest = segment.start + projection * delta;
      const double radius = segment.support_radius_m + std::max(0.0, tolerance_m);
      if ((point - closest).squaredNorm() <= radius * radius + 1.0e-12) return true;
    }
    return false;
  }

  [[nodiscard]] std::size_t byteSize() const noexcept {
    return segments_.size() * sizeof(Segment);
  }

 private:
  SegmentVector segments_;
  double max_age_s_{0.0};
};

using TraversedFreeSpacePtr = std::shared_ptr<const TraversedFreeSpace>;

}  // namespace navigation_mapping
