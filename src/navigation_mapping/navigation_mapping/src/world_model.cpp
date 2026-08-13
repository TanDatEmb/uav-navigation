#include "navigation_mapping/world_model.hpp"

#include "navigation_mapping/rog_map_adapter.hpp"

namespace navigation_mapping {
namespace {

rog_map::Vec3f toRogPosition(const Vec3& position) {
  return rog_map::Vec3f(static_cast<float>(position.x()), static_cast<float>(position.y()),
                        static_cast<float>(position.z()));
}

CellState fromRogGridType(super_utils::GridType type) {
  switch (type) {
    case super_utils::KNOWN_FREE:
      return CellState::KnownFree;
    case super_utils::OCCUPIED:
      return CellState::Occupied;
    case super_utils::UNKNOWN:
    case super_utils::FRONTIER:
    case super_utils::OUT_OF_MAP:
    case super_utils::UNDEFINED:
    default:
      return CellState::Unknown;
  }
}

rog_map::Vec3i layerHalfSize(const rog_map::Config& config, WorldLayer layer) {
  return layer == WorldLayer::Probability ? config.half_map_size_i : config.inf_half_map_size_i;
}

}  // namespace

WorldModel::WorldModel(const RogMapAdapter& adapter) : adapter_(adapter) {}

rog_map::Vec3i toRogIndex(const GridIndex3& index) {
  return rog_map::Vec3i(index.x, index.y, index.z);
}

GridIndex3 fromRogIndex(const rog_map::Vec3i& index) {
  return GridIndex3{index.x(), index.y(), index.z()};
}

std::uint64_t WorldModel::generation() const noexcept {
  return adapter_.generation();
}

CellState WorldModel::cellState(WorldLayer layer, const GridIndex3& index) const {
  const auto& map = adapter_.map();
  if (!bounds(layer).contains(index)) {
    return CellState::Unknown;
  }
  if (layer == WorldLayer::Probability) {
    auto rog_index = toRogIndex(index);
    return fromRogGridType(map.getGridType(rog_index));
  }
  return fromRogGridType(map.getInfGridType(toRogPosition(gridToWorld(layer, index))));
}

GridIndex3 WorldModel::worldToGrid(WorldLayer layer, const Vec3& position) const {
  const auto& map = adapter_.map();
  rog_map::Vec3i index;
  const auto position_f = toRogPosition(position);
  if (layer == WorldLayer::Probability) {
    map.probMapPosToGlobalIndex(position_f, index);
  } else {
    map.infMapPosToGlobalIndex(position_f, index);
  }
  return fromRogIndex(index);
}

Vec3 WorldModel::gridToWorld(WorldLayer layer, const GridIndex3& index) const {
  const auto& map = adapter_.map();
  rog_map::Vec3f position;
  const auto rog_index = toRogIndex(index);
  if (layer == WorldLayer::Probability) {
    map.probMapGlobalIndexToPos(rog_index, position);
  } else {
    map.infMapGlobalIndexToPos(rog_index, position);
  }
  return Vec3(position.x(), position.y(), position.z());
}

double WorldModel::resolution(WorldLayer layer) const {
  const auto& map = adapter_.map();
  return layer == WorldLayer::Probability ? map.getResolution() : map.getInfResolution();
}

GridBounds WorldModel::bounds(WorldLayer layer) const {
  const auto& map = adapter_.map();
  const auto config = map.getMapConfig();
  const auto origin = map.getLocalMapOrigin();
  const auto center = worldToGrid(layer, Vec3(origin.x(), origin.y(), origin.z()));
  const auto half_size = layerHalfSize(config, layer);
  return GridBounds{
      GridIndex3{center.x - half_size.x(), center.y - half_size.y(), center.z - half_size.z()},
      GridIndex3{center.x + half_size.x(), center.y + half_size.y(), center.z + half_size.z()}};
}

}  // namespace navigation_mapping
