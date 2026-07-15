#ifndef FAST_LIO_MAP_TREE_INTERFACE_HPP_
#define FAST_LIO_MAP_TREE_INTERFACE_HPP_

#include "fast_lio/commons.hpp"

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <vector>
#include <memory>

namespace fast_lio {

// Forward declaration for ikd-Tree implementation
namespace ikdtree {
    template<typename PointType> class KD_TREE;
}

/**
 * @brief Abstract interface for 3D spatial index used in FAST-LIO.
 *
 * This interface decouples the LIO algorithm from the specific spatial index
 * implementation. Supports both incremental (ikd-Tree) and non-incremental
 * (PCL KD-tree) backends.
 *
 * Design goals:
 * - Real-time: O(log N) add, delete, nearest search
 * - Thread-safe: Support parallel operations
 * - Memory-efficient: Minimal allocations during operation
 * - UAV-optimized: Parameters tuned for aerial navigation
 */
class MapTreeInterface {
 public:
    virtual ~MapTreeInterface() = default;

    // ============================================================
    // Core Operations (must be real-time safe)
    // ============================================================

    /**
     * @brief Add points to the tree.
     * @param points Points to add
     * @param downsample If true, apply voxel downsampling
     * @return Number of points actually added
     *
     * Time complexity: O(M log N) where M = number of points
     */
    virtual size_t addPoints(const PointVec& points, bool downsample = true) = 0;

    /**
     * @brief Delete points within box regions.
     * @param boxes Axis-aligned boxes to delete
     * @return Number of points marked for deletion
     *
     * Time complexity: O(K log N) where K = number of boxes
     * Note: May use lazy deletion; actual removal during rebuild
     */
    virtual size_t deletePoints(const std::vector<BoxPointType>& boxes) = 0;

    /**
     * @brief Find k nearest neighbors.
     * @param query Query point
     * @param k Number of neighbors
     * @param[out] indices Indices of nearest points
     * @param[out] distances Squared distances to nearest points
     * @return Number of neighbors found
     *
     * Time complexity: O(log N + k)
     */
    virtual size_t nearestKSearch(const PointType& query, int k,
                                   std::vector<int>& indices,
                                   std::vector<float>& distances) const = 0;

    /**
     * @brief Find k nearest neighbors and return their actual point coordinates.
     *
     * ikd-Tree does not expose stable point indices, so callers that need
     * the neighbor *points* (e.g. for plane fitting) should use this method
     * instead of nearestKSearch() + getAllPoints().
     *
     * @param query Query point
     * @param k Number of neighbors
     * @param[out] neighbors Actual neighbor points
     * @param[out] distances Squared distances to nearest points
     * @return Number of neighbors found
     */
    virtual size_t nearestKSearchPoints(const PointType& query, int k,
                                        PointVec& neighbors,
                                        std::vector<float>& distances) const = 0;

    // ============================================================
    // Tree Management
    // ============================================================

    /**
     * @brief Build tree from point cloud (initial construction).
     * @param cloud Input point cloud
     *
     * Time complexity: O(N log N)
     */
    virtual void build(const CloudType::Ptr& cloud) = 0;

    /**
     * @brief Rebuild the tree (force compaction).
     *
     * For incremental trees: trigger background rebuild
     * For non-incremental trees: full rebuild
     */
    virtual void rebuild() = 0;

    /**
     * @brief Clear all points.
     */
    virtual void clear() = 0;

    // ============================================================
    // State Queries
    // ============================================================

    /**
     * @brief Get number of valid points in tree.
     */
    virtual size_t size() const = 0;

    /**
     * @brief Check if tree is empty.
     */
    virtual bool empty() const = 0;

    /**
     * @brief Get tree validity status.
     * @return True if tree is valid for queries
     */
    virtual bool valid() const = 0;

    /**
     * @brief Check if background rebuild is in progress.
     * @return True if rebuild thread is active
     */
    virtual bool rebuilding() const = 0;

    // ============================================================
    // Configuration
    // ============================================================

    /**
     * @brief Set downsampling resolution.
     * @param resolution Voxel size in meters
     */
    virtual void setDownsampleParam(float resolution) = 0;

    /**
     * @brief Set local map bounding box.
     * @param box Box defining valid region
     */
    virtual void setLocalMapRange(const BoxPointType& box) = 0;

    // ============================================================
    // Accessors (for debugging/visualization)
    // ============================================================

    /**
     * @brief Get all valid points.
     * @param[out] points Output point vector
     */
    virtual void getAllPoints(PointVec& points) const = 0;

    /**
     * @brief Get removed points (for visualization).
     * @param[out] points Output point vector
     */
    virtual void getRemovedPoints(PointVec& points) = 0;

    // ============================================================
    // Factory Methods
    // ============================================================

    /**
     * @brief Create ikd-Tree implementation (real-time, incremental).
     * @return Shared pointer to ikd-Tree instance
     */
    static std::shared_ptr<MapTreeInterface> createIKDTree();

    /**
     * @brief Create PCL KD-tree implementation (fallback, non-incremental).
     * @return Shared pointer to PCL KD-tree instance
     */
    static std::shared_ptr<MapTreeInterface> createPCLTree();
};

}  // namespace fast_lio

#endif  // FAST_LIO_MAP_TREE_INTERFACE_HPP_
