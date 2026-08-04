#include "px4_odometry_bridge/odometry_ring_buffer.hpp"

#include <algorithm>

namespace px4_odometry_bridge {

std::uint64_t OdometryRingBuffer::frameGeneration(const ConvertedOdometry &sample) {
  return sample.frame_generation;
}

OdometryRingBuffer::OdometryRingBuffer(RingBufferConfig config) : config_(config) {}

bool OdometryRingBuffer::push(const ConvertedOdometry &sample) {
  if (sample.timestamp_ns <= 0 || sample.frame_generation == 0 ||
      (!samples_.empty() && sample.timestamp_ns <= samples_.back().timestamp_ns)) {
    samples_.clear();
    post_reset_stable_ = false;
    stable_sample_count_ = 0;
    return false;
  }

  if (!generation_initialized_ || frameGeneration(sample) != current_generation_) {
    if (generation_initialized_) samples_.clear();
    current_generation_ = frameGeneration(sample);
    generation_initialized_ = true;
    stable_sample_count_ = 0;
    post_reset_stable_ = false;
  }
  ++stable_sample_count_;
  if (stable_sample_count_ < config_.stable_samples) {
    return false;
  }
  post_reset_stable_ = true;
  samples_.push_back(sample);
  while (samples_.size() > config_.capacity ||
         (samples_.back().timestamp_ns - samples_.front().timestamp_ns) >
             config_.duration_ns) {
    samples_.pop_front();
  }
  return true;
}

std::optional<SampledOdometry> OdometryRingBuffer::sample(std::int64_t timestamp_ns) const {
  if (samples_.empty() || timestamp_ns < samples_.front().timestamp_ns ||
      timestamp_ns > samples_.back().timestamp_ns) {
    return std::nullopt;
  }
  auto upper = std::lower_bound(samples_.begin(), samples_.end(), timestamp_ns,
                                [](const auto &sample, auto time) {
                                  return sample.timestamp_ns < time;
                                });
  if (upper != samples_.end() && upper->timestamp_ns == timestamp_ns) {
    return SampledOdometry{*upper, false};
  }
  if (upper == samples_.begin() || upper == samples_.end()) return std::nullopt;
  const auto &before = *(upper - 1);
  const auto &after = *upper;
  if (frameGeneration(after) != frameGeneration(before) ||
      after.time_generation != before.time_generation ||
      after.timestamp_ns - before.timestamp_ns > config_.max_gap_ns) {
    return std::nullopt;
  }
  return SampledOdometry{interpolate(before, after, timestamp_ns), true};
}

ConvertedOdometry OdometryRingBuffer::interpolate(const ConvertedOdometry &a,
                                                  const ConvertedOdometry &b,
                                                  std::int64_t timestamp_ns) {
  const double alpha = static_cast<double>(timestamp_ns - a.timestamp_ns) /
                       static_cast<double>(b.timestamp_ns - a.timestamp_ns);
  ConvertedOdometry output = a;
  output.timestamp_ns = timestamp_ns;
  output.position = a.position + alpha * (b.position - a.position);
  output.velocity_body = a.velocity_body + alpha * (b.velocity_body - a.velocity_body);
  output.angular_velocity_body =
      a.angular_velocity_body + alpha * (b.angular_velocity_body - a.angular_velocity_body);
  output.orientation = a.orientation.slerp(alpha, b.orientation).normalized();
  output.position_variance = a.position_variance.cwiseMax(b.position_variance);
  output.velocity_variance = a.velocity_variance.cwiseMax(b.velocity_variance);
  output.orientation_variance = a.orientation_variance.cwiseMax(b.orientation_variance);
  output.angular_velocity_valid = a.angular_velocity_valid && b.angular_velocity_valid;
  return output;
}

void OdometryRingBuffer::clear() {
  samples_.clear();
  current_generation_ = 0;
  generation_initialized_ = false;
  stable_sample_count_ = 0;
  post_reset_stable_ = false;
}

void OdometryRingBuffer::setStableSamples(std::size_t stable_samples) {
  config_.stable_samples = std::max<std::size_t>(1, stable_samples);
  stable_sample_count_ = 0;
  post_reset_stable_ = false;
}

}  // namespace px4_odometry_bridge
