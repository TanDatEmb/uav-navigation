#pragma once

#include <cstddef>
#include <cstdint>
#include <deque>
#include <optional>
#include <stdexcept>

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
  explicit OdometryRingBuffer(RingBufferConfig config = {}) : config_(config) {
    if (config_.duration_ns <= 0 || config_.capacity == 0U ||
        config_.max_gap_ns <= 0 || config_.stable_samples == 0U) {
      throw std::invalid_argument("invalid odometry ring-buffer configuration");
    }
  }
  bool push(const ConvertedOdometry &sample);
  std::optional<SampledOdometry> sample(std::int64_t timestamp_ns) const;
  void clear();
  void setStableSamples(std::size_t stable_samples);
  std::size_t size() const { return samples_.size(); }
  std::size_t stableSampleCount() const { return stable_sample_count_; }
  std::size_t stableSamplesRequired() const { return config_.stable_samples; }
  bool postResetStable() const { return post_reset_stable_; }

 private:
  static std::uint64_t frameGeneration(const ConvertedOdometry& sample);
  static ConvertedOdometry interpolate(const ConvertedOdometry &a,
                                       const ConvertedOdometry &b,
                                       std::int64_t timestamp_ns);
  RingBufferConfig config_;
  std::deque<ConvertedOdometry> samples_;
  std::uint64_t current_generation_{0};
  bool generation_initialized_{false};
  std::size_t stable_sample_count_{0};
  bool post_reset_stable_{false};
};

}  // namespace px4_odometry_bridge
