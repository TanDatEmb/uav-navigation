#include "px4_odometry_bridge/reset_compensator.hpp"

#include <cmath>

#include "px4_odometry_bridge/frame_converter.hpp"

#include <limits>

namespace px4_odometry_bridge {

namespace {

bool knownPoseFrame(const PoseFrame frame) {
  return frame == PoseFrame::kNed || frame == PoseFrame::kFrd;
}

bool knownVelocityFrame(const VelocityFrame frame) {
  return frame == VelocityFrame::kNed || frame == VelocityFrame::kFrd ||
         frame == VelocityFrame::kBodyFrd;
}

bool knownWorldConvention(const WorldConvention convention) {
  return convention == WorldConvention::kRosEnu ||
         convention == WorldConvention::kPx4FrdLocal;
}

Eigen::Vector3d rotateDiagonalVariance(const Eigen::Matrix3d &rotation,
                                       const Eigen::Vector3d &variance) {
  if (!variance.allFinite()) return variance;
  Eigen::Vector3d result = Eigen::Vector3d::Constant(-1.0);
  for (int index = 0; index < 3; ++index) {
    if (variance[index] >= 0.0 && std::isfinite(variance[index])) {
      result[index] =
          (rotation.row(index).array().square() * variance.transpose().array()).sum();
    }
  }
  return result;
}

}  // namespace

const char* toString(const ResetObservationStatus status) noexcept {
  switch (status) {
    case ResetObservationStatus::kAccepted: return "accepted";
    case ResetObservationStatus::kResetTransitionSuppressed: return "reset_transition_suppressed";
    case ResetObservationStatus::kMetadataPending: return "metadata_pending";
    case ResetObservationStatus::kInvalidMetadata: return "invalid_metadata";
    case ResetObservationStatus::kCounterDiscontinuity: return "counter_discontinuity";
    case ResetObservationStatus::kInvalidResetRotation: return "invalid_reset_rotation";
    case ResetObservationStatus::kProbableSourceRestart: return "probable_source_restart";
    case ResetObservationStatus::kGenerationExhausted: return "generation_exhausted";
  }
  return "unknown";
}

ResetObservation ResetCompensator::observe(
    ConvertedOdometry sample, DetailedResetMetadata metadata) {
  ResetObservation result;
  const double orientation_norm_squared = sample.orientation.squaredNorm();
  const auto valid_variance = [](const Eigen::Vector3d& value, const bool available) {
    return value.allFinite() && (!available || (value.array() >= 0.0).all());
  };
  if (sample.timestamp_ns <= 0 || sample.frame_generation == 0U ||
      !knownPoseFrame(sample.source_pose_frame) ||
      !knownVelocityFrame(sample.source_velocity_frame) ||
      !knownWorldConvention(sample.world_convention) ||
      !sample.position.allFinite() || !sample.velocity_world.allFinite() ||
      !sample.velocity_body.allFinite() || !sample.angular_velocity_body.allFinite() ||
      !sample.orientation.coeffs().allFinite() ||
      !std::isfinite(orientation_norm_squared) ||
      std::abs(orientation_norm_squared - 1.0) > 1.0e-6 ||
      !valid_variance(sample.position_variance, sample.position_covariance_available) ||
      !valid_variance(sample.velocity_variance, sample.velocity_covariance_available) ||
      !valid_variance(sample.orientation_variance, sample.orientation_covariance_available)) {
    result.status = ResetObservationStatus::kInvalidMetadata;
    result.reset_generation = reset_generation_;
    return result;
  }
  if (metadata.matched_odometry_timestamp_ns != 0 &&
      metadata.matched_odometry_timestamp_ns != sample.timestamp_ns) {
    result.status = ResetObservationStatus::kInvalidMetadata;
    result.reset_generation = reset_generation_;
    return result;
  }
  if (!initialized_) {
    initialized_ = true;
    last_counter_ = sample.reset_counter;
  } else {
    const std::uint8_t delta = static_cast<std::uint8_t>(sample.reset_counter - last_counter_);
    if (delta != 0) {
      // A one-count modulo-256 increment is the only reset transition that
      // can be associated unambiguously with this sample.
      if (delta != 1) {
        result.status = ResetObservationStatus::kCounterDiscontinuity;
        result.reset_generation = reset_generation_;
        return result;
      }
      if (metadata.association_invalid) {
        result.status = ResetObservationStatus::kInvalidMetadata;
        result.reset_generation = reset_generation_;
        return result;
      }
      if (!metadata.available || !metadata.hasReset()) {
        result.status = ResetObservationStatus::kMetadataPending;
        result.reset_generation = reset_generation_;
        return result;
      }
      if (reset_generation_ == std::numeric_limits<std::uint64_t>::max()) {
        result.status = ResetObservationStatus::kGenerationExhausted;
        result.reset_generation = reset_generation_;
        return result;
      }
      if (!last_output_.has_value() || metadata.timestamp_ns <= 0 ||
          ((metadata.position_xy_reset || metadata.position_z_reset) &&
           !metadata.position_delta_source.allFinite()) ||
          ((metadata.velocity_xy_reset || metadata.velocity_z_reset) &&
           !metadata.velocity_delta_source.allFinite()) ||
          (metadata.heading_reset && !std::isfinite(metadata.heading_delta_rad))) {
        result.status = ResetObservationStatus::kInvalidMetadata;
        result.reset_generation = reset_generation_;
        return result;
      }

      Eigen::Matrix3d reset_rotation_world = Eigen::Matrix3d::Identity();
      if (metadata.attitude_reset) {
        const double attitude_norm_squared = metadata.attitude_delta.squaredNorm();
        if (!metadata.attitude_delta.coeffs().allFinite() ||
            !std::isfinite(attitude_norm_squared) ||
            std::abs(attitude_norm_squared - 1.0) > 1.0e-6) {
          result.status = ResetObservationStatus::kInvalidResetRotation;
          result.reset_generation = reset_generation_;
          return result;
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
      continuity_rotation_ *= inverse_reset;
      if (metadata.position_xy_reset || metadata.position_z_reset) {
        const Eigen::Vector3d position_delta =
            continuity_rotation_ *
            (FrameConverter::c_enu_ned() * metadata.position_delta_source);
        continuity_translation_ -= position_delta;
      }
      if (metadata.velocity_xy_reset || metadata.velocity_z_reset) {
        const Eigen::Vector3d velocity_delta =
            continuity_rotation_ *
            (FrameConverter::c_enu_ned() * metadata.velocity_delta_source);
        continuity_velocity_translation_ -= velocity_delta;
      }
      ++reset_generation_;
      last_counter_ = sample.reset_counter;
      // The transition sample is intentionally suppressed. The next sample
      // proves that the continuity transform remains valid.
      result.status = ResetObservationStatus::kResetTransitionSuppressed;
      result.reset_generation = reset_generation_;
      return result;
    }
  }

  // A reset defines a new sample world basis. Apply the accumulated inverse
  // reset to every world-expressed component, not just the quaternion; doing
  // otherwise publishes a non-rigid pose (continuous attitude with a position
  // left in the post-reset basis). Position/velocity deltas are then applied
  // in the same continuous world basis.
  sample.position = continuity_translation_ + continuity_rotation_ * sample.position;
  sample.orientation = Eigen::Quaterniond(continuity_rotation_ *
                                           sample.orientation.toRotationMatrix()).normalized();
  sample.position_variance =
      rotateDiagonalVariance(continuity_rotation_, sample.position_variance);
  sample.velocity_world =
      continuity_velocity_translation_ + continuity_rotation_ * sample.velocity_world;
  sample.velocity_body = sample.orientation.toRotationMatrix().transpose() *
                         sample.velocity_world;
  sample.reset_generation = reset_generation_;
  sample.reset_event_generation = reset_generation_;
  last_output_ = sample;
  result.status = ResetObservationStatus::kAccepted;
  result.sample = std::move(sample);
  result.reset_generation = reset_generation_;
  if (result.sample) {
    result.sample->reset_event_generation = reset_generation_;
  }
  return result;
}

std::optional<Eigen::Vector3d> ResetCompensator::rebasePositionAtCurrentOutput() {
  if (!last_output_.has_value() || !last_output_->position.allFinite()) {
    return std::nullopt;
  }
  const Eigen::Vector3d origin = last_output_->position;
  continuity_translation_ -= origin;
  last_output_->position -= origin;
  return origin;
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
