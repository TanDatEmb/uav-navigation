#include "fast_lio_core/mapping/ikd_tree_registration_map.hpp"

#if defined(__GNUC__)
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wpedantic"
#endif
#include <ikd_Tree.h>
#if defined(__GNUC__)
#pragma GCC diagnostic pop
#endif

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <limits>
#include <optional>
#include <stdexcept>
#include <tuple>
#include <utility>

namespace uav::nav::lio {
namespace {

using UpstreamPoint = ikdTree_PointType;
using UpstreamTree = KD_TREE<UpstreamPoint>;
using UpstreamPointVector = UpstreamTree::PointVector;

bool isRepresentableAsUpstreamPoint(const Eigen::Vector3d& point) {
  constexpr double kMaximumFloat =
      static_cast<double>(std::numeric_limits<float>::max());
  return point.allFinite() &&
         (point.array().abs() <= kMaximumFloat).all();
}

UpstreamPoint toUpstreamPoint(const Eigen::Vector3d& point) {
  return UpstreamPoint{static_cast<float>(point.x()),
                       static_cast<float>(point.y()),
                       static_cast<float>(point.z())};
}

Eigen::Vector3d toEigenPoint(const UpstreamPoint& point) {
  return Eigen::Vector3d(static_cast<double>(point.x),
                         static_cast<double>(point.y),
                         static_cast<double>(point.z));
}

bool lexicographicPointLess(const Eigen::Vector3d& left,
                            const Eigen::Vector3d& right) {
  return std::tie(left.x(), left.y(), left.z()) <
         std::tie(right.x(), right.y(), right.z());
}

bool upstreamPointLess(const UpstreamPoint& left,
                       const UpstreamPoint& right) {
  return std::tie(left.x, left.y, left.z) <
         std::tie(right.x, right.y, right.z);
}

bool validConfig(const IkdTreeRegistrationMapConfig& config) {
  return config.voxel_size_m > 0.0 &&
         std::isfinite(config.voxel_size_m) &&
         config.voxel_size_m <=
             static_cast<double>(std::numeric_limits<float>::max()) &&
         config.deletion_rebuild_ratio > 0.0 &&
         config.deletion_rebuild_ratio < 1.0 &&
         std::isfinite(config.deletion_rebuild_ratio) &&
         config.balance_rebuild_ratio > 0.5 &&
         config.balance_rebuild_ratio < 1.0 &&
         std::isfinite(config.balance_rebuild_ratio);
}

}  // namespace

class IkdTreeRegistrationMap::Impl {
 public:
  explicit Impl(const IkdTreeRegistrationMapConfig& config)
      : tree(std::make_unique<UpstreamTree>(
            static_cast<float>(config.deletion_rebuild_ratio),
            static_cast<float>(config.balance_rebuild_ratio),
            static_cast<float>(config.voxel_size_m),
            config.enable_asynchronous_rebuild)) {}

  [[nodiscard]] UpstreamPointVector snapshotUpstream() const {
    UpstreamPointVector points;
    if (!built || validPointCount() == 0U) {
      return points;
    }
    const BoxPointType range = tree->tree_range();
    BoxPointType inclusive_range = range;
    for (std::size_t axis = 0; axis < 3U; ++axis) {
      inclusive_range.vertex_min[axis] =
          std::nextafter(range.vertex_min[axis],
                         -std::numeric_limits<float>::infinity());
      inclusive_range.vertex_max[axis] =
          std::nextafter(range.vertex_max[axis],
                         std::numeric_limits<float>::infinity());
    }
    tree->Box_Search(inclusive_range, points);
    return points;
  }

  [[nodiscard]] std::size_t validPointCount() const noexcept {
    const int signed_count = tree->validnum();
    if (signed_count >= 0) {
      cached_valid_point_count = static_cast<std::size_t>(signed_count);
      return cached_valid_point_count;
    }
    // validnum() uses trylock and returns -1 while an asynchronous root
    // rebuild owns the working state. Bookkeeping must never wait for it.
    ++valid_point_count_busy_count;
    return cached_valid_point_count;
  }

  std::unique_ptr<UpstreamTree> tree;
  bool built{false};
  mutable std::size_t cached_valid_point_count{0U};
  mutable std::size_t valid_point_count_busy_count{0U};
};

IkdTreeRegistrationMap::IkdTreeRegistrationMap(
    IkdTreeRegistrationMapConfig config)
    : config_(config) {
  if (!validConfig(config_)) {
    throw std::invalid_argument(
        "invalid upstream ikd-Tree registration map configuration");
  }
  impl_ = std::make_unique<Impl>(config_);
}

IkdTreeRegistrationMap::~IkdTreeRegistrationMap() = default;

bool IkdTreeRegistrationMap::nearestSearch(
    const Eigen::Vector3d& query_odom_m,
    double maximum_distance_m,
    NeighborSet& output) const {
  output.count = 0U;
  if (!isRepresentableAsUpstreamPoint(query_odom_m) ||
      !(maximum_distance_m > 0.0) ||
      !std::isfinite(maximum_distance_m)) {
    return false;
  }

  if (!impl_->built) {
    return false;
  }

  std::array<UpstreamPoint, NeighborSet::kCapacity> upstream_neighbors{};
  std::array<float, NeighborSet::kCapacity> upstream_distances{};
  std::array<UpstreamTree::PointType_CMP, 2U * NeighborSet::kCapacity>
      heap_storage{};
  int count = 0;
  impl_->tree->Nearest_Search_Into(
      toUpstreamPoint(query_odom_m), static_cast<int>(NeighborSet::kCapacity),
      upstream_neighbors.data(), upstream_distances.data(),
      static_cast<int>(upstream_neighbors.size()), count, heap_storage.data(),
      static_cast<int>(heap_storage.size()), maximum_distance_m);
  output.count = static_cast<std::size_t>(std::max(count, 0));
  for (std::size_t index = 0U; index < output.count; ++index) {
    output.points[index] = toEigenPoint(upstream_neighbors[index]);
    output.squared_distances[index] = upstream_distances[index];
  }
  return output.count == NeighborSet::kCapacity;
}

std::size_t IkdTreeRegistrationMap::insert(
    std::span<const Eigen::Vector3d> points_odom_m) {
  UpstreamPointVector points;
  points.reserve(points_odom_m.size());
  for (const Eigen::Vector3d& point_odom_m : points_odom_m) {
    if (isRepresentableAsUpstreamPoint(point_odom_m)) {
      points.push_back(toUpstreamPoint(point_odom_m));
    }
  }
  if (points.empty()) {
    return 0U;
  }
  std::sort(points.begin(), points.end(), upstreamPointLess);

  const std::size_t size_before =
      impl_->built ? impl_->validPointCount() : 0U;
  if (!impl_->built || size_before == 0U) {
    UpstreamPointVector initial_point{points.front()};
    impl_->tree->Build(initial_point);
    impl_->built = true;
    points.erase(points.begin());
  }
  if (!points.empty()) {
    static_cast<void>(impl_->tree->Add_Points(points, true));
  }
  const std::size_t size_after = impl_->validPointCount();
  return size_after > size_before ? size_after - size_before : 0U;
}

std::size_t IkdTreeRegistrationMap::cropLocal(
    const Eigen::Vector3d& center_odom_m,
    const Eigen::Vector3d& half_extent_m) {
  if (!isRepresentableAsUpstreamPoint(center_odom_m) ||
      !half_extent_m.allFinite() ||
      (half_extent_m.array() <= 0.0).any()) {
    return 0U;
  }

  const Eigen::Vector3d minimum_odom_m =
      center_odom_m - half_extent_m;
  const Eigen::Vector3d maximum_odom_m =
      center_odom_m + half_extent_m;
  if (!isRepresentableAsUpstreamPoint(minimum_odom_m) ||
      !isRepresentableAsUpstreamPoint(maximum_odom_m)) {
    return 0U;
  }

  if (!impl_->built || impl_->validPointCount() == 0U) {
    return 0U;
  }
  const std::size_t size_before = impl_->validPointCount();
  const BoxPointType range = impl_->tree->tree_range();
  const std::array<float, 3> tree_min{
      range.vertex_min[0], range.vertex_min[1], range.vertex_min[2]};
  const std::array<float, 3> tree_max{
      std::nextafter(range.vertex_max[0],
                     std::numeric_limits<float>::infinity()),
      std::nextafter(range.vertex_max[1],
                     std::numeric_limits<float>::infinity()),
      std::nextafter(range.vertex_max[2],
                     std::numeric_limits<float>::infinity())};
  const std::array<float, 3> keep_min{
      std::max(tree_min[0], static_cast<float>(minimum_odom_m.x())),
      std::max(tree_min[1], static_cast<float>(minimum_odom_m.y())),
      std::max(tree_min[2], static_cast<float>(minimum_odom_m.z()))};
  const std::array<float, 3> keep_max{
      std::min(tree_max[0],
               std::nextafter(static_cast<float>(maximum_odom_m.x()),
                              std::numeric_limits<float>::infinity())),
      std::min(tree_max[1],
               std::nextafter(static_cast<float>(maximum_odom_m.y()),
                              std::numeric_limits<float>::infinity())),
      std::min(tree_max[2],
               std::nextafter(static_cast<float>(maximum_odom_m.z()),
                              std::numeric_limits<float>::infinity()))};

  std::vector<BoxPointType> deletion_boxes;
  deletion_boxes.reserve(6U);
  const auto append_box = [&deletion_boxes](
                              const std::array<float, 3>& box_min,
                              const std::array<float, 3>& box_max) {
    if (box_min[0] >= box_max[0] || box_min[1] >= box_max[1] ||
        box_min[2] >= box_max[2]) {
      return;
    }
    BoxPointType box{};
    for (std::size_t axis = 0; axis < 3U; ++axis) {
      box.vertex_min[axis] = box_min[axis];
      box.vertex_max[axis] = box_max[axis];
    }
    deletion_boxes.push_back(box);
  };

  // Partition the part of the old tree range outside the new local cube into
  // at most six non-overlapping slabs. This is the FAST-LIO2 moving-cube
  // deletion pattern and avoids an O(map-size) snapshot/per-point box pass.
  append_box(tree_min, {keep_min[0], tree_max[1], tree_max[2]});
  append_box({keep_max[0], tree_min[1], tree_min[2]}, tree_max);
  append_box({keep_min[0], tree_min[1], tree_min[2]},
             {keep_max[0], keep_min[1], tree_max[2]});
  append_box({keep_min[0], keep_max[1], tree_min[2]},
             {keep_max[0], tree_max[1], tree_max[2]});
  append_box({keep_min[0], keep_min[1], tree_min[2]},
             {keep_max[0], keep_max[1], keep_min[2]});
  append_box({keep_min[0], keep_min[1], keep_max[2]},
             {keep_max[0], keep_max[1], tree_max[2]});

  if (!deletion_boxes.empty()) {
    static_cast<void>(impl_->tree->Delete_Point_Boxes(deletion_boxes));
    if (impl_->validPointCount() == 0U) {
      impl_->built = false;
    }
  }
  const std::size_t size_after = impl_->validPointCount();
  return size_before > size_after ? size_before - size_after : 0U;
}

std::vector<Eigen::Vector3d>
IkdTreeRegistrationMap::snapshot() const {
  const UpstreamPointVector upstream_points =
      impl_->snapshotUpstream();
  std::vector<Eigen::Vector3d> points;
  points.reserve(upstream_points.size());
  for (const UpstreamPoint& point : upstream_points) {
    points.push_back(toEigenPoint(point));
  }
  std::sort(points.begin(), points.end(), lexicographicPointLess);
  return points;
}

std::size_t IkdTreeRegistrationMap::size() const {
  return impl_->built ? impl_->validPointCount() : 0U;
}

std::size_t IkdTreeRegistrationMap::validPointCountBusyCount() const noexcept {
  return impl_->valid_point_count_busy_count;
}

void IkdTreeRegistrationMap::clear() {
  impl_ = std::make_unique<Impl>(config_);
}

}  // namespace uav::nav::lio
