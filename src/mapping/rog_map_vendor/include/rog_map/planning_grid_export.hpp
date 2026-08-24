#pragma once

#include <cstddef>
#include <cstdint>
#include <vector>

#include <super_utils/eigen_alias.hpp>

namespace rog_map {

struct PlanningGridLayoutExport {
  double resolution_m{0.0};
  super_utils::Vec3i global_min_index{super_utils::Vec3i::Zero()};
  super_utils::Vec3i dimensions{super_utils::Vec3i::Zero()};
  super_utils::Vec3f local_center_m{super_utils::Vec3f::Zero()};
  super_utils::Vec3f local_size_m{super_utils::Vec3f::Zero()};
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
  std::vector<super_utils::Vec3i> nearest_offsets;
  bool unknown_inflation_enabled{false};
  double virtual_ground_m{0.0};
  double virtual_ceiling_m{0.0};
  double occupied_inflation_radius_m{0.0};

  [[nodiscard]] std::size_t byteSize() const noexcept {
    return base_state.size() + inflated.occupied.size() + inflated.unknown.size() +
           nearest_offsets.size() * sizeof(super_utils::Vec3i);
  }
};

}  // namespace rog_map
