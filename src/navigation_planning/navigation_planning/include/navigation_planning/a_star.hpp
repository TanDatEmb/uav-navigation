#pragma once

#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <limits>
#include <queue>
#include <unordered_map>
#include <vector>

#include "navigation_mapping/world_model.hpp"

namespace navigation_planning {

enum class UnknownPolicy {
  TreatUnknownAsBlocked,
  TreatUnknownAsTraversable,
};

enum class SearchFailureCode {
  None,
  WorldModelUnavailable,
  StartOutsideBounds,
  GoalOutsideBounds,
  StartOccupied,
  GoalOccupied,
  NoPath,
};

struct SearchRequest {
  navigation_mapping::WorldLayer layer{navigation_mapping::WorldLayer::Probability};
  UnknownPolicy unknown_policy{UnknownPolicy::TreatUnknownAsBlocked};
  navigation_mapping::Vec3 start_world{navigation_mapping::Vec3::Zero()};
  navigation_mapping::Vec3 goal_world{navigation_mapping::Vec3::Zero()};
};

struct SearchStatistics {
  std::uint64_t expanded_nodes{0};
  std::uint64_t generated_nodes{0};
  std::uint64_t cell_state_queries{0};
  std::uint64_t peak_open_set_size{0};
  std::uint64_t path_node_count{0};
  double path_length_m{0.0};
  std::int64_t search_time_us{0};
};

struct SearchResult {
  bool success{false};
  SearchFailureCode failure{SearchFailureCode::None};
  std::uint64_t world_generation{0};
  std::vector<navigation_mapping::GridIndex3> path;
  SearchStatistics statistics;
};

namespace detail {

struct GridIndexHash {
  std::size_t operator()(const navigation_mapping::GridIndex3& index) const noexcept {
    std::size_t seed = std::hash<int>{}(index.x);
    seed ^= std::hash<int>{}(index.y) + 0x9e3779b9U + (seed << 6U) + (seed >> 2U);
    seed ^= std::hash<int>{}(index.z) + 0x9e3779b9U + (seed << 6U) + (seed >> 2U);
    return seed;
  }
};

inline bool sameIndex(const navigation_mapping::GridIndex3& lhs,
                      const navigation_mapping::GridIndex3& rhs) noexcept {
  return lhs == rhs;
}

inline navigation_mapping::GridIndex3 add(
    const navigation_mapping::GridIndex3& lhs,
    const navigation_mapping::GridIndex3& rhs) noexcept {
  return navigation_mapping::GridIndex3{lhs.x + rhs.x, lhs.y + rhs.y, lhs.z + rhs.z};
}

inline const std::array<navigation_mapping::GridIndex3, 26>& neighborOffsets() {
  static const auto offsets = [] {
    std::array<navigation_mapping::GridIndex3, 26> result{};
    std::size_t cursor = 0;
    for (int x = -1; x <= 1; ++x) {
      for (int y = -1; y <= 1; ++y) {
        for (int z = -1; z <= 1; ++z) {
          if (x == 0 && y == 0 && z == 0) continue;
          result[cursor++] = navigation_mapping::GridIndex3{x, y, z};
        }
      }
    }
    return result;
  }();
  return offsets;
}

template <typename Model>
SearchResult searchModel(const Model& model, const SearchRequest& request) {
  const auto started = std::chrono::steady_clock::now();
  SearchResult result;
  const auto finish = [&]() {
    result.statistics.search_time_us = std::chrono::duration_cast<std::chrono::microseconds>(
        std::chrono::steady_clock::now() - started).count();
    return result;
  };

  if constexpr (requires { model.isReady(); }) {
    if (!model.isReady()) {
      result.failure = SearchFailureCode::WorldModelUnavailable;
      return finish();
    }
  }
  result.world_generation = model.generation();
  const auto start = model.worldToGrid(request.layer, request.start_world);
  const auto goal = model.worldToGrid(request.layer, request.goal_world);
  const auto bounds = model.bounds(request.layer);
  if (!bounds.contains(start)) {
    result.failure = SearchFailureCode::StartOutsideBounds;
    return finish();
  }
  if (!bounds.contains(goal)) {
    result.failure = SearchFailureCode::GoalOutsideBounds;
    return finish();
  }

  const auto traversable = [&](const navigation_mapping::CellState state) {
    if (state == navigation_mapping::CellState::Occupied) return false;
    return state == navigation_mapping::CellState::KnownFree ||
           request.unknown_policy == UnknownPolicy::TreatUnknownAsTraversable;
  };

  struct Record {
    double g{std::numeric_limits<double>::infinity()};
    navigation_mapping::GridIndex3 parent{};
    navigation_mapping::CellState state{navigation_mapping::CellState::Unknown};
    bool state_queried{false};
    bool has_parent{false};
    bool closed{false};
  };
  std::unordered_map<navigation_mapping::GridIndex3, Record, GridIndexHash> records;
  records.reserve(1024);
  const auto queryState = [&](const navigation_mapping::GridIndex3& index) {
    auto [record_it, inserted] = records.try_emplace(index);
    if (!inserted && record_it->second.state_queried) {
      return record_it->second.state;
    }
    ++result.statistics.cell_state_queries;
    record_it->second.state = model.cellState(request.layer, index);
    record_it->second.state_queried = true;
    return record_it->second.state;
  };
  const auto start_state = queryState(start);
  if (!traversable(start_state)) {
    result.failure = start_state == navigation_mapping::CellState::Occupied
                         ? SearchFailureCode::StartOccupied
                         : SearchFailureCode::NoPath;
    return finish();
  }
  if (sameIndex(start, goal)) {
    result.success = true;
    result.path = {start};
    result.statistics.path_node_count = 1;
    return finish();
  }
  const auto goal_state = queryState(goal);
  if (!traversable(goal_state)) {
    result.failure = goal_state == navigation_mapping::CellState::Occupied
                         ? SearchFailureCode::GoalOccupied
                         : SearchFailureCode::NoPath;
    return finish();
  }

  struct OpenEntry {
    navigation_mapping::GridIndex3 index;
    double g{0.0};
    double f{0.0};
    std::uint64_t sequence{0};
  };
  struct OpenCompare {
    bool operator()(const OpenEntry& lhs, const OpenEntry& rhs) const noexcept {
      if (lhs.f != rhs.f) return lhs.f > rhs.f;
      if (lhs.g != rhs.g) return lhs.g < rhs.g;
      return lhs.sequence > rhs.sequence;
    }
  };

  std::priority_queue<OpenEntry, std::vector<OpenEntry>, OpenCompare> open;
  const double resolution = model.resolution(request.layer);
  const auto heuristic = [&](const navigation_mapping::GridIndex3& index) {
    const int dx = index.x - goal.x;
    const int dy = index.y - goal.y;
    const int dz = index.z - goal.z;
    return resolution * std::sqrt(static_cast<double>(dx * dx + dy * dy + dz * dz));
  };
  const auto transitionCost = [&](const navigation_mapping::GridIndex3& offset) {
    return resolution * std::sqrt(static_cast<double>(offset.x * offset.x +
                                                       offset.y * offset.y +
                                                       offset.z * offset.z));
  };
  const auto transitionIsValid = [&](const navigation_mapping::GridIndex3& current,
                                     const navigation_mapping::GridIndex3& offset) {
    const int changed_axes = (offset.x != 0) + (offset.y != 0) + (offset.z != 0);
    if (changed_axes <= 1) return true;

    // A diagonal transition crosses every non-empty proper subset of its
    // changed axes. Requiring those support cells to be traversable prevents
    // 2-D corner cutting and its 3-D face/edge equivalents.
    const int subset_count = (1 << changed_axes) - 1;
    std::array<int, 3> axes{};
    int axis_count = 0;
    if (offset.x != 0) axes[axis_count++] = 0;
    if (offset.y != 0) axes[axis_count++] = 1;
    if (offset.z != 0) axes[axis_count++] = 2;
    for (int subset = 1; subset < subset_count; ++subset) {
      navigation_mapping::GridIndex3 support = current;
      for (int bit = 0; bit < changed_axes; ++bit) {
        if ((subset & (1 << bit)) == 0) continue;
        if (axes[bit] == 0) support.x += offset.x;
        if (axes[bit] == 1) support.y += offset.y;
        if (axes[bit] == 2) support.z += offset.z;
      }
      if (!bounds.contains(support) || !traversable(queryState(support))) return false;
    }
    return true;
  };
  std::uint64_t sequence = 0;
  records[start].g = 0.0;
  open.push(OpenEntry{start, 0.0, heuristic(start), sequence++});
  result.statistics.peak_open_set_size = 1;

  while (!open.empty()) {
    const OpenEntry current = open.top();
    open.pop();
    auto current_record = records.find(current.index);
    if (current_record == records.end() || current_record->second.closed ||
        current.g > current_record->second.g) {
      continue;
    }
    current_record->second.closed = true;
    ++result.statistics.expanded_nodes;
    if (sameIndex(current.index, goal)) {
      std::vector<navigation_mapping::GridIndex3> reversed;
      auto cursor = current.index;
      while (true) {
        reversed.push_back(cursor);
        if (sameIndex(cursor, start)) break;
        cursor = records.at(cursor).parent;
      }
      result.path.assign(reversed.rbegin(), reversed.rend());
      result.success = true;
      result.statistics.path_node_count = result.path.size();
      for (std::size_t i = 1; i < result.path.size(); ++i) {
        const auto& previous = result.path[i - 1];
        const auto& current_node = result.path[i];
        result.statistics.path_length_m += transitionCost(
            navigation_mapping::GridIndex3{current_node.x - previous.x,
                                           current_node.y - previous.y,
                                           current_node.z - previous.z});
      }
      return finish();
    }

    const double current_g = current_record->second.g;
    for (const auto& offset : neighborOffsets()) {
      const auto neighbor = add(current.index, offset);
      if (!bounds.contains(neighbor)) continue;
      const auto state = queryState(neighbor);
      if (!traversable(state)) continue;
      if (!transitionIsValid(current.index, offset)) continue;
      ++result.statistics.generated_nodes;
      const double candidate_g = current_g + transitionCost(offset);
      auto [record_it, inserted] = records.try_emplace(neighbor);
      if (record_it->second.closed || (!inserted && candidate_g >= record_it->second.g)) {
        continue;
      }
      record_it->second.g = candidate_g;
      record_it->second.parent = current.index;
      record_it->second.has_parent = true;
      open.push(OpenEntry{neighbor, candidate_g, candidate_g + heuristic(neighbor), sequence++});
      result.statistics.peak_open_set_size =
          std::max<std::uint64_t>(result.statistics.peak_open_set_size, open.size());
    }
  }

  result.failure = SearchFailureCode::NoPath;
  return finish();
}

}  // namespace detail

class AStar final {
 public:
  [[nodiscard]] SearchResult search(const navigation_mapping::WorldModel& model,
                                    const SearchRequest& request) const;

  // Deterministic grid-model hook used by unit tests; production consumers
  // call search(WorldModel, ...), so no runtime polymorphism is required.
  template <typename Model>
  [[nodiscard]] SearchResult searchForTest(const Model& model,
                                           const SearchRequest& request) const {
    return detail::searchModel(model, request);
  }
};

}  // namespace navigation_planning
