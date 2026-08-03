#include "px4_odometry_bridge/reset_compensator.hpp"

#include <cmath>

#include "px4_odometry_bridge/frame_converter.hpp"

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
      if (delta != 1 || !metadata.available || !metadata.hasReset() ||
          !last_output_.has_value() || metadata.timestamp_ns <= 0 ||
          !metadata.position_delta_source.allFinite() ||
          !metadata.velocity_delta_source.allFinite() ||
          !std::isfinite(metadata.heading_delta_rad)) {
        return std::nullopt;
      }

      Eigen::Matrix3d reset_rotation_world = Eigen::Matrix3d::Identity();
      if (metadata.attitude_reset) {
        if (!metadata.attitude_delta.coeffs().allFinite() ||
            metadata.attitude_delta.norm() < 1e-9) {
          return std::nullopt;
        }
        reset_rotation_world = FrameConverter::c_enu_ned() *
                               metadata.attitude_delta.normalized().toRotationMatrix() *
                               FrameConverter::c_enu_ned().transpose();
      } else if (metadata.heading_reset) {
        reset_rotation_world =
            FrameConverter::c_enu_ned() *
            Eigen::AngleAxisd(metadata.heading_delta_rad, Eigen::Vector3d::UnitZ())
                .toRotationMatrix() *
            FrameConverter::c_enu_ned().transpose();
      }
      const Eigen::Matrix3d inverse_reset = reset_rotation_world.transpose();
      const Eigen::Vector3d position_delta =
          FrameConverter::c_enu_ned() * metadata.position_delta_source;
      const Eigen::Vector3d velocity_delta =
          FrameConverter::c_enu_ned() * metadata.velocity_delta_source;
      continuity_translation_ -= continuity_rotation_ * inverse_reset * position_delta;
      continuity_velocity_translation_ -=
          continuity_rotation_ * inverse_reset * velocity_delta;
      continuity_rotation_ *= inverse_reset;
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
  sample.velocity_world = continuity_velocity_translation_ +
                          continuity_rotation_ * sample.velocity_world;
  sample.velocity_body = sample.orientation.toRotationMatrix().transpose() *
                         sample.velocity_world;
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
  continuity_velocity_translation_.setZero();
  last_output_.reset();
}

}  // namespace px4_odometry_bridge
