#ifndef FAST_LIO_PCL_TREE_IMPL_HPP_
#define FAST_LIO_PCL_TREE_IMPL_HPP_

#include "fast_lio/spatial_index.hpp"

#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/filters/voxel_grid.h>

#include <mutex>

namespace fast_lio {

/**
 * @brief PCL KD-tree implementation (non-incremental, fallback).
 *
 * Used for:
 * - Initial development and testing
 * - Platforms where ikd-Tree is not available
 * - Baseline comparison
 *
 * Limitations:
 * - Full rebuild on each update: O(N log N)
 * - Not suitable for real-time with large maps
 * - No lazy deletion
 */
class PCLTreeBackend : public MapTreeInterface {
 public:
    explicit PCLTreeBackend(float downsample_resolution = 0.2f);
    ~PCLTreeBackend() override = default;

    // Core operations
    size_t addPoints(const PointVec& points, bool downsample = true) override;
    size_t deletePoints(const std::vector<BoxPointType>& boxes) override;
    size_t nearestKSearch(const PointType& query, int k,
                         std::vector<int>& indices,
                         std::vector<float>& distances) const override;
    size_t nearestKSearchPoints(const PointType& query, int k,
                                PointVec& neighbors,
                                std::vector<float>& distances) const override;

    // Tree management
    void build(const CloudType::Ptr& cloud) override;
    void rebuild() override;
    void clear() override;

    // State queries
    size_t size() const override;
    bool empty() const override;
    bool valid() const override;
    bool rebuilding() const override { return false; }

    // Configuration
    void setDownsampleParam(float resolution) override;
    void setLocalMapRange(const BoxPointType& box) override;

    // Accessors
    void getAllPoints(PointVec& points) const override;
    void getRemovedPoints(PointVec& points) override;

 private:
    mutable std::mutex mutex_;

    // PCL KD-tree (rebuilt on each update)
    pcl::KdTreeFLANN<PointType> kdtree_;
    CloudType::Ptr cloud_;

    // Voxel filter for downsampling
    pcl::VoxelGrid<PointType> voxel_filter_;
    float downsample_res_;

    // Local map range
    BoxPointType local_range_;
    bool has_range_;

    // Deleted points tracking (for compatibility)
    PointVec removed_points_;

    // Rebuild the tree from current cloud_
    void rebuildTree();

    // Filter cloud by local range
    void applyRangeFilter();
};

}  // namespace fast_lio

#endif  // FAST_LIO_PCL_TREE_IMPL_HPP_
