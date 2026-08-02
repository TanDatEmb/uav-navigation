#include "px4_odometry_bridge/reset_compensator.hpp"

namespace px4_odometry_bridge {

std::optional<ConvertedOdometry> ResetCompensator::observe(
    ConvertedOdometry sample, DetailedResetMetadata metadata) {
  if (!initialized_) {
    initialized_ = true;
    last_counter_ = sample.reset_counter;
  } else {
    const std::uint8_t delta = static_cast<std::uint8_t>(sample.reset_counter - last_counter_);
    if (delta != 0) {
      // A one-count modulo-256 increment is the only reset transition that
      // can be associated unambiguously with this sample.
      if (delta != 1 || !metadata.available || !last_output_.has_value()) return std::nullopt;
      const Eigen::Matrix3d old_rotation = last_output_->orientation.toRotationMatrix();
      const Eigen::Matrix3d new_rotation = sample.orientation.toRotationMatrix();
      const Eigen::Matrix3d old_continuous = old_rotation;
      continuity_rotation_ = old_continuous * new_rotation.transpose();
      continuity_translation_ = last_output_->position - continuity_rotation_ * sample.position;
      ++reset_generation_;
      last_counter_ = sample.reset_counter;
      // The transition sample is intentionally suppressed. The next sample
      // proves that the continuity transform remains valid.
      return std::nullopt;
    }
  }

  sample.position = continuity_translation_ + continuity_rotation_ * sample.position;
  sample.orientation = Eigen::Quaterniond(continuity_rotation_ *
                                           sample.orientation.toRotationMatrix()).normalized();
  sample.reset_generation = reset_generation_;
  last_output_ = sample;
  return sample;
}

void ResetCompensator::clear() {
  initialized_ = false;
  last_counter_ = 0;
  reset_generation_ = 0;
  continuity_rotation_.setIdentity();
  continuity_translation_.setZero();
  last_output_.reset();
}

}  // namespace px4_odometry_bridge
