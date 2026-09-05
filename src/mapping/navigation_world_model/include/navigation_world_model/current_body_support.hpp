#pragma once

#include <array>
#include <cmath>
#include <cstdint>
#include <limits>
#include <string_view>

#include <Eigen/Geometry>

#include <navigation_world_model/world_model_view.hpp>

namespace navigation_world_model {

// Project-owned geometry contract for the bundled x500_mid360 model.  The
// propeller collision links are intentionally excluded: they are a separate
// swept-volume problem and must not be silently treated as rigid body support.
inline constexpr std::string_view kX500Mid360BodyGeometryProvenance =
    "repo:src/uav_simulation/models/x500_mid360/model.sdf@sha256="
    "7e924e71a275bfcdddd2ad7469b77f947053511a113f112efa03d876fa3d4c09;"
    "px4-gazebo:x500/base_link_collision_0@sha256="
    "e807dca3406f7cd5cb3d545898601c3ec17e0e5242e25cf467a69df1812cf436;"
    "repo:src/uav_simulation/models/lidar_mid360/model.sdf@sha256="
    "fcd1679e0cc9a652b665d220738bf6ed70c4af2e99656220aae64994de90d2dc;"
    "component=base_link_collision_0_main_obb_only;"
    "excluded_disconnected=base_link_collision_1..4,housing_collision";

[[nodiscard]] inline CurrentBodySupport
makeX500Mid360CurrentBodySupport(
    const Point3& body_position,
    const Eigen::Quaterniond& body_orientation,
    const WorldSnapshotIdentity& snapshot_identity,
    const char* world_frame_id,
    const char* body_frame_id,
    const std::uint64_t measured_localization_epoch,
    const std::int64_t measured_source_stamp_ns) noexcept {
  using Support = CurrentBodySupport;
  Support support;
  support.snapshot_identity = snapshot_identity;
  support.body_position = body_position;
  support.body_orientation = body_orientation;
  support.localization_epoch = measured_localization_epoch;
  support.source_stamp_ns = measured_source_stamp_ns;
  if (world_frame_id != nullptr) support.world_frame_id = world_frame_id;
  if (body_frame_id != nullptr) support.body_frame_id = body_frame_id;
  support.geometry_provenance = std::string(kX500Mid360BodyGeometryProvenance);

  // x500_base/model.sdf base_link_collision_0 main OBB, expressed in
  // base_link. Other collision components are physically disconnected from
  // the measured origin and cannot belong to an ordered UNKNOWN prefix.
  support.body_box = Support::Box{{0.0, 0.0, 0.007},
                   {0.17677669529663687, 0.17677669529663687, 0.025},
                   Eigen::Quaterniond::Identity()};

  const auto finite_positive = [](const auto value) {
    return std::isfinite(value) && value > 0;
  };
  support.valid = body_position.allFinite() && body_orientation.coeffs().allFinite() &&
      finite_positive(body_orientation.norm()) &&
      std::abs(body_orientation.norm() - 1.0) <= 1.0e-6 &&
      measured_localization_epoch != 0U &&
      snapshot_identity.generation != 0U &&
      snapshot_identity.revision != 0U &&
      measured_source_stamp_ns > 0 &&
      snapshot_identity.localization_epoch == measured_localization_epoch &&
      !support.world_frame_id.empty() && support.body_frame_id == "base_link";
  return support;
}

}  // namespace navigation_world_model
