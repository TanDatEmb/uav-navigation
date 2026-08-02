#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <optional>
#include <string>

#include "fast_lio_core/geometry/frame.hpp"
#include "fast_lio_core/time/timestamp.hpp"

namespace uav::nav::lio {

enum class InitialStatePriorSource { kZero, kFixed, kTopic };
enum class InitialStatePriorContext { kGroundStartup, kInFlightReinitialization };
enum class PriorAttitudeMode { kNone, kYawOnly, kFull };
enum class InitialPriorFallback { kReject, kZero, kFixed };
enum class InitialPriorStatus {
  kNotRequired,
  kWaiting,
  kCandidateAvailable,
  kApplied,
  kFallbackApplied,
  kRejected,
  kClosed,
};

[[nodiscard]] const char* toString(InitialStatePriorSource value) noexcept;
[[nodiscard]] const char* toString(InitialStatePriorContext value) noexcept;
[[nodiscard]] const char* toString(PriorAttitudeMode value) noexcept;
[[nodiscard]] const char* toString(InitialPriorStatus value) noexcept;

struct InitialStatePriorMask {
  bool position{false};
  bool velocity{false};
  PriorAttitudeMode attitude{PriorAttitudeMode::kNone};
};

struct InitialStatePrior {
  Timestamp sample_time{};
  FrameId reference_frame{FrameId("odom")};
  FrameId body_frame{FrameId("base_link")};
  InitialStatePriorSource source{InitialStatePriorSource::kZero};
  InitialStatePriorContext context{InitialStatePriorContext::kGroundStartup};
  InitialStatePriorMask mask{};
  Eigen::Vector3d position_odom_base_m{Eigen::Vector3d::Zero()};
  Eigen::Quaterniond orientation_odom_base{Eigen::Quaterniond::Identity()};
  std::optional<Eigen::Vector3d> linear_velocity_base_m_s;
  std::optional<Eigen::Vector3d> angular_velocity_base_rad_s;
  std::string provenance;

  [[nodiscard]] bool allFinite() const noexcept;
};

}  // namespace uav::nav::lio
