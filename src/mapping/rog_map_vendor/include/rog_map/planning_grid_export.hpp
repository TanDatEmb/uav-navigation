#pragma once

#include <cstddef>
#include <cstdint>
#include <memory>
#include <vector>

#include <navigation_math/eigen_alias.hpp>

namespace rog_map {

struct PlanningGridLayoutExport {
  double resolution_m{0.0};
  navigation_math::Vec3i global_min_index{navigation_math::Vec3i::Zero()};
  navigation_math::Vec3i dimensions{navigation_math::Vec3i::Zero()};
  navigation_math::Vec3f local_center_m{navigation_math::Vec3f::Zero()};
  navigation_math::Vec3f local_size_m{navigation_math::Vec3f::Zero()};
};

struct InflatedPlanningGridExport {
  PlanningGridLayoutExport layout;
  std::vector<std::uint8_t> occupied;
  std::vector<std::uint8_t> unknown;
};

struct PlanningGridExport {
  PlanningGridLayoutExport base_layout;
  InflatedPlanningGridExport inflated;
  std::vector<std::uint8_t> base_state;
  std::shared_ptr<const std::vector<navigation_math::Vec3i>> nearest_offsets;
  bool unknown_inflation_enabled{false};
  bool virtual_ground_ceiling_enabled{true};
  double virtual_ground_m{0.0};
  double virtual_ceiling_m{0.0};
  double inflated_virtual_ground_m{0.0};
  double inflated_virtual_ceiling_m{0.0};
  double occupied_inflation_radius_m{0.0};

  [[nodiscard]] std::size_t ownedByteSize() const noexcept {
    return base_state.size() + inflated.occupied.size() + inflated.unknown.size();
  }

  [[nodiscard]] std::size_t sharedMetadataByteSize() const noexcept {
    return nearest_offsets ? nearest_offsets->size() * sizeof(navigation_math::Vec3i) : 0U;
  }

  [[nodiscard]] std::size_t byteSize() const noexcept {
    return ownedByteSize() + sharedMetadataByteSize();
  }
};

// A bounded update window in the same logical index domain as the full
// export. Consumers may apply this to an immutable snapshot without scanning
// or allocating the unchanged map volume.
struct PlanningGridPatchExport {
  PlanningGridLayoutExport base_layout;
  InflatedPlanningGridExport inflated;
  std::vector<std::uint8_t> base_state;
};

}  // namespace rog_map
