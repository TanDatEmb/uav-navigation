// ikd-Tree wrapper implementation
// Wraps HKU ikd-Tree with ROS2-compatible interface

#include "fast_lio/ikd_tree_backend.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace fast_lio {

namespace {

inline float squaredDistance(const PointType& a, const PointType& b) {
    const float dx = a.x - b.x;
    const float dy = a.y - b.y;
    const float dz = a.z - b.z;
    return dx * dx + dy * dy + dz * dz;
}

inline PointType voxelCenter(float resolution, const PointType& pt) {
    PointType center;
    center.x = std::floor(pt.x / resolution) * resolution + 0.5f * resolution;
    center.y = std::floor(pt.y / resolution) * resolution + 0.5f * resolution;
    center.z = std::floor(pt.z / resolution) * resolution + 0.5f * resolution;
    center.intensity = 0.0f;
    return center;
}

}  // namespace

IKDTreeBackend::IKDTreeBackend(float delete_criterion, float balance_criterion,
                               float downsample_resolution)
    : config_{delete_criterion, balance_criterion, downsample_resolution} {
    // Set ikd-Tree parameters
    tree_.Set_delete_criterion_param(delete_criterion);
    tree_.Set_balance_criterion_param(balance_criterion);
    tree_.set_downsample_param(downsample_resolution);
}

IKDTreeBackend::~IKDTreeBackend() {
    // ikd-Tree destructor handles pthread cleanup
}

bool IKDTreeBackend::findVoxelRepresentative(const PointType& query, float resolution,
                                             PointType& representative) {
    // ikd-Tree's Nearest_Search assumes a non-empty tree.
    if (tree_.size() <= 0) {
        return false;
    }

    const PointType center = voxelCenter(resolution, query);
    // A point inside the same voxel is at most half the space diagonal away.
    const float search_radius = 0.8660254f * resolution + 1e-4f;  // sqrt(3)/2
    const float squared_radius = search_radius * search_radius;

    PointVec neighbors;
    std::vector<float> distances;
    tree_.Nearest_Search(center, 1, neighbors, distances, search_radius);

    if (neighbors.empty() || distances.front() > squared_radius) {
        return false;
    }

    representative = neighbors.front();
    return true;
}

size_t IKDTreeBackend::addPoints(const PointVec& points, bool /*downsample*/) {
    std::unique_lock<std::shared_mutex> lock(mutex_);

    const float resolution = config_.downsample_resolution;
    if (resolution <= 0.0f) {
        std::vector<PointType, Eigen::aligned_allocator<PointType>> ikd_points;
        ikd_points.reserve(points.size());
        for (const auto& pt : points) {
            ikd_points.push_back(pt);
        }
        tree_.Add_Points(ikd_points, false);
        return points.size();
    }

    // FAST-LIO2 map_incremental logic:
    //   - For each new point, determine its voxel.
    //   - If the voxel is empty, the point can be inserted without downsampling.
    //   - If the voxel already has a representative, keep only the point that is
    //     closer to the voxel center. Those points are inserted with downsampling
    //     enabled so ikd-Tree replaces the old representative.
    std::vector<PointType, Eigen::aligned_allocator<PointType>> points_to_downsample;
    std::vector<PointType, Eigen::aligned_allocator<PointType>> points_no_downsample;
    points_to_downsample.reserve(points.size());
    points_no_downsample.reserve(points.size());

    for (const auto& pt : points) {
        PointType representative;
        if (findVoxelRepresentative(pt, resolution, representative)) {
            const PointType center = voxelCenter(resolution, pt);
            const float new_dist_sq = squaredDistance(pt, center);
            const float old_dist_sq = squaredDistance(representative, center);
            if (new_dist_sq < old_dist_sq) {
                points_to_downsample.push_back(pt);
            }
            // Else: existing representative is better, drop the new point.
        } else {
            points_no_downsample.push_back(pt);
        }
    }

    if (!points_to_downsample.empty()) {
        tree_.Add_Points(points_to_downsample, true);
    }
    if (!points_no_downsample.empty()) {
        tree_.Add_Points(points_no_downsample, false);
    }

    return points_to_downsample.size() + points_no_downsample.size();
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

void IKDTreeBackend::setDownsampleParam(float resolution) {
    std::unique_lock<std::shared_mutex> lock(mutex_);
    config_.downsample_resolution = resolution;
    tree_.set_downsample_param(resolution);
}

}  // namespace fast_lio
