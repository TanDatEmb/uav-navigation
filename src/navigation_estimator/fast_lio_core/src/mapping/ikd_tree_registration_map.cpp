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
#include <chrono>
#include <cmath>
#include <limits>
#include <optional>
#include <stdexcept>
#include <thread>
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

std::size_t safeValidPointCount(UpstreamTree& tree) {
  constexpr auto kTimeout = std::chrono::milliseconds(10);
  const auto deadline = std::chrono::steady_clock::now() + kTimeout;
  do {
    const int signed_count = tree.validnum();
    if (signed_count >= 0) {
      return static_cast<std::size_t>(signed_count);
    }
    // Upstream returns -1 while an asynchronous root rebuild owns its working
    // state. Keep that state distinct from a genuinely empty tree.
    std::this_thread::yield();
  } while (std::chrono::steady_clock::now() < deadline);
  throw std::runtime_error(
      "ikd-Tree valid point count remained busy during rebuild");
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
    if (!built || safeValidPointCount(*tree) == 0U) {
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

  std::unique_ptr<UpstreamTree> tree;
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
  if (!impl_->built || safeValidPointCount(*impl_->tree) == 0U) {
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
  const std::size_t size_before =
      impl_->built ? safeValidPointCount(*impl_->tree) : 0U;
  if (!impl_->built || size_before == 0U) {
    UpstreamPointVector initial_point{points.front()};
    impl_->tree->Build(initial_point);
    impl_->built = true;
    points.erase(points.begin());
  }
  if (!points.empty()) {
    static_cast<void>(impl_->tree->Add_Points(points, true));
  }
  const std::size_t size_after = safeValidPointCount(*impl_->tree);
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

  std::scoped_lock lock(mutex_);
  if (!impl_->built || safeValidPointCount(*impl_->tree) == 0U) {
    return 0U;
  }
  const std::size_t size_before = safeValidPointCount(*impl_->tree);
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
    if (safeValidPointCount(*impl_->tree) == 0U) {
      impl_->built = false;
    }
  }
  const std::size_t size_after = safeValidPointCount(*impl_->tree);
  return size_before > size_after ? size_before - size_after : 0U;
}

std::size_t IkdTreeRegistrationMap::pruneFarthest(
    const Eigen::Vector3d& center_odom_m,
    std::size_t target_point_count,
    double distance_shell_size_m) {
  if (!isRepresentableAsUpstreamPoint(center_odom_m) ||
      target_point_count == 0U || !(distance_shell_size_m > 0.0) ||
      !std::isfinite(distance_shell_size_m)) {
    return 0U;
  }
  std::scoped_lock lock(mutex_);
  if (!impl_->built ||
      safeValidPointCount(*impl_->tree) <= target_point_count) {
    return 0U;
  }
  UpstreamPointVector points = impl_->snapshotUpstream();
  const auto distance_key = [&](const UpstreamPoint& point) {
    const double distance = (toEigenPoint(point) - center_odom_m).norm();
    const auto shell =
        static_cast<std::uint64_t>(std::floor(distance / distance_shell_size_m));
    return std::pair{shell, distance};
  };
  std::nth_element(
      points.begin(), points.begin() + static_cast<std::ptrdiff_t>(target_point_count),
      points.end(), [&](const UpstreamPoint& left, const UpstreamPoint& right) {
        return distance_key(left) < distance_key(right);
      });
  UpstreamPointVector points_to_delete(
      points.begin() + static_cast<std::ptrdiff_t>(target_point_count),
      points.end());
  const std::size_t size_before = safeValidPointCount(*impl_->tree);
  impl_->tree->Delete_Points(points_to_delete);
  const std::size_t size_after = safeValidPointCount(*impl_->tree);
  if (size_after == 0U) {
    impl_->built = false;
  }
  return size_before > size_after ? size_before - size_after : 0U;
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

std::size_t IkdTreeRegistrationMap::size() const {
  std::scoped_lock lock(mutex_);
  return impl_->built
             ? safeValidPointCount(*impl_->tree)
             : 0U;
}

void IkdTreeRegistrationMap::clear() {
  std::scoped_lock lock(mutex_);
  impl_ = std::make_unique<Impl>(config_);
}

}  // namespace uav::nav::lio
