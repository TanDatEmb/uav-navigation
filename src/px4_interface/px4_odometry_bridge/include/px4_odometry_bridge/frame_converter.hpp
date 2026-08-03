#pragma once

#include <array>
#include <cstdint>
#include <optional>
#include <string>

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace px4_odometry_bridge {

enum class PoseFrame : std::uint8_t { kUnknown = 0, kNed = 1, kFrd = 2 };
enum class VelocityFrame : std::uint8_t {
  kUnknown = 0,
  kNed = 1,
  kFrd = 2,
  kBodyFrd = 3
};

enum class WorldConvention : std::uint8_t {
  kUnknown = 0,
  kRosEnu = 1,
  kPx4FrdLocal = 2,
};

struct Px4OdometrySample {
  std::int64_t timestamp_ns{0};
  PoseFrame pose_frame{PoseFrame::kUnknown};
  VelocityFrame velocity_frame{VelocityFrame::kUnknown};
  Eigen::Vector3d position{Eigen::Vector3d::Zero()};
  Eigen::Quaterniond orientation{Eigen::Quaterniond::Identity()};
  Eigen::Vector3d velocity{Eigen::Vector3d::Zero()};
  Eigen::Vector3d angular_velocity{Eigen::Vector3d::Zero()};
  Eigen::Vector3d position_variance{Eigen::Vector3d::Constant(-1.0)};
  Eigen::Vector3d velocity_variance{Eigen::Vector3d::Constant(-1.0)};
  Eigen::Vector3d orientation_variance{Eigen::Vector3d::Constant(-1.0)};
  std::uint8_t reset_counter{0};
  bool angular_velocity_valid{true};
};

struct ConvertedOdometry {
  std::int64_t timestamp_ns{0};
  PoseFrame source_pose_frame{PoseFrame::kUnknown};
  VelocityFrame source_velocity_frame{VelocityFrame::kUnknown};
  WorldConvention world_convention{WorldConvention::kUnknown};
  Eigen::Vector3d position{Eigen::Vector3d::Zero()};
  Eigen::Quaterniond orientation{Eigen::Quaterniond::Identity()};
  Eigen::Vector3d velocity_world{Eigen::Vector3d::Zero()};
  Eigen::Vector3d velocity_body{Eigen::Vector3d::Zero()};
  Eigen::Vector3d angular_velocity_body{Eigen::Vector3d::Zero()};
  Eigen::Vector3d position_variance{Eigen::Vector3d::Constant(-1.0)};
  Eigen::Vector3d velocity_variance{Eigen::Vector3d::Constant(-1.0)};
  Eigen::Vector3d orientation_variance{Eigen::Vector3d::Constant(-1.0)};
  std::uint8_t reset_counter{0};
  std::uint64_t reset_generation{0};
  std::uint64_t time_generation{0};
  bool position_covariance_available{false};
  bool velocity_covariance_available{false};
  bool orientation_covariance_available{false};
  bool angular_velocity_valid{false};
};

struct ConversionResult {
  std::optional<ConvertedOdometry> value;
  std::string reason;
  explicit operator bool() const { return value.has_value(); }
};

class FrameConverter {
 public:
  ConversionResult convert(const Px4OdometrySample &sample);
  void reset_sign_continuity() { previous_orientation_.reset(); }

  static const Eigen::Matrix3d &c_enu_ned();
  static const Eigen::Matrix3d &c_flu_frd();
  static const Eigen::Matrix3d &rotation_ros_local_from_px4_frd_world();

 private:
  std::optional<Eigen::Quaterniond> previous_orientation_;
};

}  // namespace px4_odometry_bridge
