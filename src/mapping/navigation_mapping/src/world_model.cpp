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

GridBounds probabilityBounds(const rog_map::ROGMap& map) {
  const auto& config = map.getMapConfig();
  rog_map::Vec3i center;
  map.probMapPosToGlobalIndex(map.getLocalMapOrigin(), center);
  const auto half_size = config.half_map_size_i;
  return GridBounds{GridIndex3{center.x() - half_size.x(), center.y() - half_size.y(),
                              center.z() - half_size.z()},
                    GridIndex3{center.x() + half_size.x(), center.y() + half_size.y(),
                               center.z() + half_size.z()}};
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

std::uint64_t WorldModel::revision() const noexcept { return adapter_.revision(); }

double WorldModel::clearanceRadius() const noexcept { return adapter_.clearanceRadius(); }

bool WorldModel::isReady() const noexcept { return adapter_.isInitialized(); }

CellState WorldModel::cellState(WorldLayer layer, const GridIndex3& index) const {
  if (!isReady()) {
    return CellState::Unknown;
  }
  const auto& map = adapter_.map();
  if (layer == WorldLayer::Probability) {
    auto rog_index = toRogIndex(index);
    return fromRogGridType(map.getGridType(rog_index));
  }
  const auto position = toRogPosition(gridToWorld(layer, index));
  const auto inflated_type = map.getInfGridType(position);
  if (inflated_type == super_utils::OCCUPIED) {
    return CellState::Occupied;
  }
  if (inflated_type == super_utils::UNKNOWN || inflated_type == super_utils::OUT_OF_MAP) {
    return CellState::Unknown;
  }

  // ROG's inflation grid reports every non-occupied cell as KNOWN_FREE when
  // unknown inflation is disabled. Query the CounterMap aggregate directly;
  // sampling the inflated cell center would misclassify coarse cells.
  return fromRogGridType(map.getInfBaseGridType(toRogIndex(index)));
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
  const auto probability_bounds = probabilityBounds(map);
  if (layer == WorldLayer::Probability) {
    return probability_bounds;
  }

  // InfMap allocates a larger maintenance halo around the probability map.
  // Project only the valid probability-cell centers into the inflated grid;
  // the halo must never become a planner search domain.
  rog_map::Vec3f min_position;
  rog_map::Vec3f max_position;
  map.probMapGlobalIndexToPos(toRogIndex(probability_bounds.min), min_position);
  map.probMapGlobalIndexToPos(toRogIndex(probability_bounds.max), max_position);
  rog_map::Vec3i min_index;
  rog_map::Vec3i max_index;
  map.infMapPosToGlobalIndex(min_position, min_index);
  map.infMapPosToGlobalIndex(max_position, max_index);
  return GridBounds{fromRogIndex(min_index), fromRogIndex(max_index)};
}

}  // namespace navigation_mapping
