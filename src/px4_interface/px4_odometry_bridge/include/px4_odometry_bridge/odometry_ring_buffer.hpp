#pragma once

#include <cstddef>
#include <cstdint>
#include <deque>
#include <optional>

#include "px4_odometry_bridge/frame_converter.hpp"

namespace px4_odometry_bridge {

struct RingBufferConfig {
  std::int64_t duration_ns{2'000'000'000};
  std::size_t capacity{512};
  std::int64_t max_gap_ns{50'000'000};
  std::size_t stable_samples{3};
};

struct SampledOdometry {
  ConvertedOdometry value;
  bool interpolated{false};
};

class OdometryRingBuffer {
 public:
  explicit OdometryRingBuffer(RingBufferConfig config = {});
  bool push(const ConvertedOdometry &sample);
  std::optional<SampledOdometry> sample(std::int64_t timestamp_ns) const;
  void clear();
  std::size_t size() const { return samples_.size(); }

 private:
  static ConvertedOdometry interpolate(const ConvertedOdometry &a,
                                       const ConvertedOdometry &b,
                                       std::int64_t timestamp_ns);
  RingBufferConfig config_;
  std::deque<ConvertedOdometry> samples_;
};

}  // namespace px4_odometry_bridge
