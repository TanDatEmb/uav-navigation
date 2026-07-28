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
#include <cmath>
#include <cstdint>
#include <limits>
#include <stdexcept>
#include <tuple>
#include <unordered_set>
#include <utility>

namespace uav::nav::lio {
namespace {

using UpstreamPoint = ikdTree_PointType;
using UpstreamTree = KD_TREE<UpstreamPoint>;
using UpstreamPointVector = UpstreamTree::PointVector;

struct VoxelIndex {
  std::int64_t x{0};
  std::int64_t y{0};
  std::int64_t z{0};

  bool operator==(const VoxelIndex&) const = default;
};

struct VoxelIndexHash {
  std::size_t operator()(const VoxelIndex& index) const noexcept {
    constexpr std::size_t kMagic =
        static_cast<std::size_t>(0x9e3779b97f4a7c15ULL);
    std::size_t seed = 0U;
    const auto mix = [&](std::uint64_t value) {
      seed ^= static_cast<std::size_t>(value) + kMagic +
              (seed << 6U) + (seed >> 2U);
    };
    mix(static_cast<std::uint64_t>(index.x));
    mix(static_cast<std::uint64_t>(index.y));
    mix(static_cast<std::uint64_t>(index.z));
    return seed;
  }
};

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

VoxelIndex voxelIndex(const UpstreamPoint& point,
                      double voxel_size_m) {
  return {
      static_cast<std::int64_t>(
          std::floor(static_cast<double>(point.x) / voxel_size_m)),
      static_cast<std::int64_t>(
          std::floor(static_cast<double>(point.y) / voxel_size_m)),
      static_cast<std::int64_t>(
          std::floor(static_cast<double>(point.z) / voxel_size_m)),
  };
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
            static_cast<float>(config.voxel_size_m))) {}

  [[nodiscard]] UpstreamPointVector snapshotUpstream() const {
    UpstreamPointVector points;
    if (!built) {
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

  void rebuildVoxelMetadata(double voxel_size_m) {
    represented_voxels.clear();
    for (const UpstreamPoint& point : snapshotUpstream()) {
      represented_voxels.insert(voxelIndex(point, voxel_size_m));
    }
  }

  std::unique_ptr<UpstreamTree> tree;
  std::unordered_set<VoxelIndex, VoxelIndexHash> represented_voxels;
  bool built{false};
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

IkdTreeRegistrationMap::~IkdTreeRegistrationMap() {
  std::scoped_lock lock(mutex_);
  impl_.reset();
}

NearestNeighborResult IkdTreeRegistrationMap::nearestNeighbors(
    const Eigen::Vector3d& query_odom_m,
    std::size_t neighbor_count,
    double maximum_distance_m) const {
  NearestNeighborResult result;
  if (!isRepresentableAsUpstreamPoint(query_odom_m) ||
      neighbor_count == 0U ||
      neighbor_count >
          static_cast<std::size_t>(std::numeric_limits<int>::max()) ||
      !(maximum_distance_m > 0.0) ||
      !std::isfinite(maximum_distance_m)) {
    return result;
  }

  std::scoped_lock lock(mutex_);
  if (!impl_->built || impl_->represented_voxels.empty()) {
    return result;
  }

  UpstreamPointVector upstream_neighbors;
  std::vector<float> upstream_squared_distances;
  impl_->tree->Nearest_Search(
      toUpstreamPoint(query_odom_m),
      static_cast<int>(neighbor_count), upstream_neighbors,
      upstream_squared_distances, maximum_distance_m);

  struct Candidate {
    Eigen::Vector3d point_odom_m;
    double squared_distance_m2{0.0};
  };
  std::vector<Candidate> candidates;
  candidates.reserve(upstream_neighbors.size());
  const double maximum_squared_distance_m2 =
      maximum_distance_m * maximum_distance_m;
  for (const UpstreamPoint& upstream_point : upstream_neighbors) {
    const Eigen::Vector3d point_odom_m =
        toEigenPoint(upstream_point);
    const double squared_distance_m2 =
        (point_odom_m - query_odom_m).squaredNorm();
    if (std::isfinite(squared_distance_m2) &&
        squared_distance_m2 <= maximum_squared_distance_m2) {
      candidates.push_back({point_odom_m, squared_distance_m2});
    }
  }
  std::sort(candidates.begin(), candidates.end(),
            [](const Candidate& left, const Candidate& right) {
              if (left.squared_distance_m2 !=
                  right.squared_distance_m2) {
                return left.squared_distance_m2 <
                       right.squared_distance_m2;
              }
              return lexicographicPointLess(left.point_odom_m,
                                            right.point_odom_m);
            });

  result.points_odom_m.reserve(candidates.size());
  result.squared_distances_m2.reserve(candidates.size());
  for (const Candidate& candidate : candidates) {
    result.points_odom_m.push_back(candidate.point_odom_m);
    result.squared_distances_m2.push_back(
        candidate.squared_distance_m2);
  }
  return result;
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

  std::scoped_lock lock(mutex_);
  const std::size_t size_before = impl_->represented_voxels.size();
  if (!impl_->built || impl_->represented_voxels.empty()) {
    UpstreamPointVector initial_point{points.front()};
    impl_->tree->Build(initial_point);
    impl_->built = true;
    impl_->represented_voxels.insert(
        voxelIndex(points.front(), config_.voxel_size_m));
    points.erase(points.begin());
  }
  if (!points.empty()) {
    static_cast<void>(impl_->tree->Add_Points(points, true));
    for (const UpstreamPoint& point : points) {
      impl_->represented_voxels.insert(
          voxelIndex(point, config_.voxel_size_m));
    }
  }
  return impl_->represented_voxels.size() - size_before;
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

  std::scoped_lock lock(mutex_);
  if (!impl_->built || impl_->represented_voxels.empty()) {
    return 0U;
  }
  const std::size_t size_before = impl_->represented_voxels.size();
  const UpstreamPointVector represented_points =
      impl_->snapshotUpstream();
  std::vector<BoxPointType> deletion_boxes;
  deletion_boxes.reserve(represented_points.size());
  for (const UpstreamPoint& point : represented_points) {
    const Eigen::Vector3d point_odom_m = toEigenPoint(point);
    if ((point_odom_m.array() >= minimum_odom_m.array()).all() &&
        (point_odom_m.array() <= maximum_odom_m.array()).all()) {
      continue;
    }
    BoxPointType box{};
    box.vertex_min[0] = point.x;
    box.vertex_min[1] = point.y;
    box.vertex_min[2] = point.z;
    box.vertex_max[0] =
        std::nextafter(point.x,
                       std::numeric_limits<float>::infinity());
    box.vertex_max[1] =
        std::nextafter(point.y,
                       std::numeric_limits<float>::infinity());
    box.vertex_max[2] =
        std::nextafter(point.z,
                       std::numeric_limits<float>::infinity());
    deletion_boxes.push_back(box);
  }

  if (!deletion_boxes.empty()) {
    static_cast<void>(
        impl_->tree->Delete_Point_Boxes(deletion_boxes));
    impl_->rebuildVoxelMetadata(config_.voxel_size_m);
    if (impl_->represented_voxels.empty()) {
      impl_->built = false;
    }
  }
  return size_before - impl_->represented_voxels.size();
}

std::vector<Eigen::Vector3d>
IkdTreeRegistrationMap::snapshot() const {
  std::scoped_lock lock(mutex_);
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

std::size_t IkdTreeRegistrationMap::size() const noexcept {
  std::scoped_lock lock(mutex_);
  return impl_->represented_voxels.size();
}

void IkdTreeRegistrationMap::clear() {
  std::scoped_lock lock(mutex_);
  impl_ = std::make_unique<Impl>(config_);
}

}  // namespace uav::nav::lio
