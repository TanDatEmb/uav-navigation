// ikd-Tree wrapper implementation
// Wraps HKU ikd-Tree with ROS2-compatible interface

#include "fast_lio/ikd_tree_backend.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <numeric>

namespace fast_lio {

IKDTreeBackend::IKDTreeBackend(float delete_criterion, float balance_criterion,
                               float downsample_resolution, int rebuild_threshold)
    : config_{delete_criterion, balance_criterion, downsample_resolution, rebuild_threshold} {
    // Set ikd-Tree parameters
    tree_.Set_delete_criterion_param(delete_criterion);
    tree_.Set_balance_criterion_param(balance_criterion);
    tree_.set_downsample_param(downsample_resolution);
    // ikd-Tree doesn't expose set_rebuild_threshold
    (void)rebuild_threshold;

    (void)downsample_resolution;  // stored in config_ and passed to tree_
}

IKDTreeBackend::~IKDTreeBackend() {
    // ikd-Tree destructor handles pthread cleanup
}

size_t IKDTreeBackend::addPoints(const PointVec& points, bool downsample) {
    std::unique_lock<std::shared_mutex> lock(mutex_);

    // Convert PointVec to ikd-Tree's PointVector
    std::vector<PointType, Eigen::aligned_allocator<PointType>> ikd_points;
    ikd_points.reserve(points.size());
    for (const auto& pt : points) {
        ikd_points.push_back(pt);
    }

    // Add to tree (ikd-Tree handles locking internally)
    tree_.Add_Points(ikd_points, downsample);

    return points.size();
}

size_t IKDTreeBackend::deletePoints(const std::vector<BoxPointType>& boxes) {
    std::unique_lock<std::shared_mutex> lock(mutex_);

    // Convert to ikd-Tree's BoxPointType (same layout, different namespace)
    std::vector<::BoxPointType> ikd_boxes;
    ikd_boxes.reserve(boxes.size());
    for (const auto& box : boxes) {
        ::BoxPointType ikd_box;
        memcpy(ikd_box.vertex_min, box.vertex_min, sizeof(float) * 3);
        memcpy(ikd_box.vertex_max, box.vertex_max, sizeof(float) * 3);
        ikd_boxes.push_back(ikd_box);
    }

    tree_.Delete_Point_Boxes(ikd_boxes);

    return boxes.size();
}

size_t IKDTreeBackend::nearestKSearch(const PointType& query, int k, std::vector<int>& indices,
                                      std::vector<float>& distances) const {
    // Delegate to nearestKSearchPoints; synthesize sequential indices
    // for interface compatibility (callers that still use indices).
    PointVec neighbors;
    size_t found = nearestKSearchPoints(query, k, neighbors, distances);
    indices.resize(found);
    std::iota(indices.begin(), indices.end(), 0);
    return found;
}

size_t IKDTreeBackend::nearestKSearchPoints(const PointType& query, int k, PointVec& neighbors,
                                            std::vector<float>& distances) const {
    // ikd-Tree handles its own internal locking (pthread mutexes).
    // External mutex would deadlock with background rebuild thread.

    // ikd-Tree Nearest_Search returns actual neighbor points directly.
    PointVec ikd_neighbors;
    std::vector<float> ikd_distances;
    const_cast<KD_TREE<PointType>&>(tree_).Nearest_Search(query, k, ikd_neighbors, ikd_distances,
                                                          INFINITY);

    neighbors.clear();
    neighbors.reserve(ikd_neighbors.size());
    for (const auto& pt : ikd_neighbors) {
        neighbors.push_back(pt);
    }

    distances = std::move(ikd_distances);
    return neighbors.size();
}

void IKDTreeBackend::build(const CloudType::Ptr& cloud) {
    std::unique_lock<std::shared_mutex> lock(mutex_);

    std::vector<PointType, Eigen::aligned_allocator<PointType>> points;
    points.reserve(cloud->points.size());
    for (const auto& pt : cloud->points) {
        points.push_back(pt);
    }

    tree_.Build(points);
}

void IKDTreeBackend::rebuild() {
    std::unique_lock<std::shared_mutex> lock(mutex_);
    // ikd-Tree maintains balance via background thread; no explicit rebuild needed.
}

void IKDTreeBackend::clear() {
    std::unique_lock<std::shared_mutex> lock(mutex_);
    std::vector<PointType, Eigen::aligned_allocator<PointType>> empty_points;
    tree_.Build(empty_points);
}

size_t IKDTreeBackend::size() const {
    auto& tree = const_cast<KD_TREE<PointType>&>(tree_);
    const int valid_points = tree.validnum();
    if (valid_points >= 0) {
        return static_cast<size_t>(valid_points);
    }

    // validnum() can transiently return -1 while the root is being rebuilt.
    // Fall back to the total node count rather than reporting an empty map.
    return static_cast<size_t>(std::max(0, tree.size()));
}

bool IKDTreeBackend::empty() const {
    return static_cast<size_t>(const_cast<KD_TREE<PointType>&>(tree_).size()) == 0;
}

bool IKDTreeBackend::valid() const {
    return tree_.Root_Node != nullptr;
}

bool IKDTreeBackend::rebuilding() const {
    // ikd-Tree rebuilds in background thread, check status
    std::shared_lock<std::shared_mutex> lock(mutex_);
    // ikd-Tree doesn't expose this directly
    return false;
}

void IKDTreeBackend::setDownsampleParam(float resolution) {
    std::unique_lock<std::shared_mutex> lock(mutex_);
    config_.downsample_resolution = resolution;
    tree_.set_downsample_param(resolution);
}

void IKDTreeBackend::setLocalMapRange(const BoxPointType& box) {
    (void)box;  // Caller handles deletion via deletePoints()
}

void IKDTreeBackend::getAllPoints(PointVec& points) const {
    auto& tree = const_cast<KD_TREE<PointType>&>(tree_);
    points.clear();
    if (tree.Root_Node == nullptr) {
        return;
    }

    // PCL_Storage is scratch space used only during rebuilds and is not an
    // authoritative copy of incrementally added points. Query the full tree
    // range through the ikd-Tree search API so deleted points are excluded.
    ::BoxPointType range = tree.tree_range();
    for (int axis = 0; axis < 3; ++axis) {
        range.vertex_max[axis] =
            std::nextafter(range.vertex_max[axis], std::numeric_limits<float>::infinity());
    }
    tree.Box_Search(range, points);
}

void IKDTreeBackend::getRemovedPoints(PointVec& points) {
    std::unique_lock<std::shared_mutex> lock(mutex_);

    std::vector<PointType, Eigen::aligned_allocator<PointType>> removed;
    tree_.acquire_removed_points(removed);

    points.clear();
    points.reserve(removed.size());
    for (const auto& pt : removed) {
        points.push_back(pt);
    }
}

void IKDTreeBackend::acquireRemovedPoints(PointVec& points) {
    getRemovedPoints(points);
}

void IKDTreeBackend::forceRebuild() {
    std::unique_lock<std::shared_mutex> lock(mutex_);
    // ikd-Tree rebuilds automatically during Add_Points/Delete operations.
}

}  // namespace fast_lio
