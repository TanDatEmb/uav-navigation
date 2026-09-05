#pragma once

#include <array>
#include <cmath>
#include <cstdint>
#include <limits>

#include <Eigen/Geometry>

#include <navigation_world_model/world_model_view.hpp>

namespace navigation_mapping {

// Project-owned geometry contract for the bundled x500_mid360 model.  The
// propeller collision links are intentionally excluded: they are a separate
// swept-volume problem and must not be silently treated as rigid body support.
inline constexpr std::uint64_t kX500Mid360BodyGeometryProvenance =
    0x783530305f626f78ULL;  // "x500_box"

[[nodiscard]] inline navigation_world_model::CurrentBodySupport
makeX500Mid360CurrentBodySupport(
    const navigation_world_model::Point3& body_position,
    const Eigen::Quaterniond& body_orientation,
    const navigation_world_model::WorldSnapshotIdentity& snapshot_identity,
    const std::uint64_t scan_sequence) noexcept {
  using Support = navigation_world_model::CurrentBodySupport;
  Support support;
  support.snapshot_identity = snapshot_identity;
  support.body_position = body_position;
  support.body_orientation = body_orientation;
  support.localization_epoch = snapshot_identity.localization_epoch;
  support.scan_sequence = scan_sequence;
  support.source_stamp_ns = snapshot_identity.observation_stamp_ns;
  support.geometry_provenance_id = kX500Mid360BodyGeometryProvenance;

  // x500_base/model.sdf base_link_collision_0..4, expressed in base_link.
  support.body_boxes = {
      Support::Box{{0.0, 0.0, 0.007},
                   {0.17677669529663687, 0.17677669529663687, 0.025},
                   Eigen::Quaterniond::Identity()},
      Support::Box{{0.0, -0.098, -0.123},
                   {0.0075, 0.0075, 0.105},
                   Eigen::Quaterniond(Eigen::AngleAxisd(
                       -0.35, Eigen::Vector3d::UnitX()))},
      Support::Box{{0.0, 0.098, -0.123},
                   {0.0075, 0.0075, 0.105},
                   Eigen::Quaterniond(Eigen::AngleAxisd(
                       0.35, Eigen::Vector3d::UnitX()))},
      Support::Box{{0.0, -0.132, -0.2195},
                   {0.125, 0.0075, 0.0075},
                   Eigen::Quaterniond::Identity()},
      Support::Box{{0.0, 0.132, -0.2195},
                   {0.125, 0.0075, 0.0075},
                   Eigen::Quaterniond::Identity()},
  };
  // lidar_mid360/model.sdf housing_collision, mounted at z=0.28 in base_link.
  support.sensor_housing = Support::Cylinder{
      {0.0, 0.0, 0.28}, 0.055, 0.030, Eigen::Quaterniond::Identity()};

  const auto finite_positive = [](const auto value) {
    return std::isfinite(value) && value > 0;
  };
  support.valid = body_position.allFinite() && body_orientation.coeffs().allFinite() &&
      finite_positive(body_orientation.norm()) &&
      std::abs(body_orientation.norm() - 1.0) <= 1.0e-6 &&
      snapshot_identity.localization_epoch != 0U &&
      snapshot_identity.generation != 0U &&
      snapshot_identity.revision != 0U &&
      snapshot_identity.observation_stamp_ns > 0 && scan_sequence != 0U;
  return support;
}

}  // namespace navigation_mapping
