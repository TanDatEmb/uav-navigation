#pragma once

#include <cstdint>
#include <Eigen/Geometry>
#include <optional>

#include "px4_odometry_bridge/frame_converter.hpp"

namespace px4_odometry_bridge {

struct DetailedResetMetadata {
  // timestamp_ns is the timestamp carried by VehicleLocalPosition or
  // VehicleAttitude. It is never replaced with the VehicleOdometry timestamp.
  std::int64_t timestamp_ns{0};
  std::int64_t matched_odometry_timestamp_ns{0};
  bool available{false};
  bool association_invalid{false};
  bool position_xy_reset{false};
  bool position_z_reset{false};
  bool velocity_xy_reset{false};
  bool velocity_z_reset{false};
  bool heading_reset{false};
  bool attitude_reset{false};

  Eigen::Vector3d position_delta_source{Eigen::Vector3d::Zero()};
  Eigen::Vector3d velocity_delta_source{Eigen::Vector3d::Zero()};
  double heading_delta_rad{0.0};
  Eigen::Quaterniond attitude_delta{Eigen::Quaterniond::Identity()};

  std::uint8_t xy_reset_counter{0};
  std::uint8_t z_reset_counter{0};
  std::uint8_t vxy_reset_counter{0};
  std::uint8_t vz_reset_counter{0};
  std::uint8_t heading_reset_counter{0};
  std::uint8_t attitude_reset_counter{0};

  [[nodiscard]] bool hasReset() const noexcept {
    return position_xy_reset || position_z_reset || velocity_xy_reset ||
           velocity_z_reset || heading_reset || attitude_reset;
  }
};

enum class ResetObservationStatus : std::uint8_t {
  kAccepted = 0,
  kResetTransitionSuppressed,
  kMetadataPending,
  kInvalidMetadata,
  kCounterDiscontinuity,
  kInvalidResetRotation,
  kProbableSourceRestart,
  kGenerationExhausted,
};

[[nodiscard]] const char* toString(ResetObservationStatus status) noexcept;

struct ResetObservation {
  ResetObservationStatus status{ResetObservationStatus::kMetadataPending};
  std::optional<ConvertedOdometry> sample;
  std::uint64_t reset_generation{0};

  [[nodiscard]] bool accepted() const noexcept { return sample.has_value(); }
  [[nodiscard]] bool has_value() const noexcept { return sample.has_value(); }
  explicit operator bool() const noexcept { return sample.has_value(); }
  [[nodiscard]] const ConvertedOdometry* operator->() const noexcept { return &*sample; }
  [[nodiscard]] ConvertedOdometry* operator->() noexcept { return &*sample; }
};

class ResetCompensator {
 public:
  [[nodiscard]] ResetObservation observe(ConvertedOdometry sample,
                                         DetailedResetMetadata metadata = {});
  // Rebase the continuous output frame at the first published sample. This
  // is used once at bridge startup so an EKF bootstrap reset cannot become the
  // origin of the downstream local-odometry contract.
  std::optional<Eigen::Vector3d> rebasePositionAtCurrentOutput();
  void clear();
  std::uint64_t reset_generation() const { return reset_generation_; }

 private:
  bool initialized_{false};
  std::uint8_t last_counter_{0};
  std::uint64_t reset_generation_{0};
  Eigen::Matrix3d continuity_rotation_{Eigen::Matrix3d::Identity()};
  Eigen::Vector3d continuity_translation_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d continuity_velocity_translation_{Eigen::Vector3d::Zero()};
  std::optional<ConvertedOdometry> last_output_;
};

}  // namespace px4_odometry_bridge
