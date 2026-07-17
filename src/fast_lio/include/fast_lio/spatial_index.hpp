#ifndef FAST_LIO_MAP_TREE_INTERFACE_HPP_
#define FAST_LIO_MAP_TREE_INTERFACE_HPP_

#include "fast_lio/commons.hpp"

#include <memory>
#include <vector>

namespace fast_lio {

// Forward declaration for ikd-Tree implementation
namespace ikdtree {
    template<typename PointType> class KD_TREE;
}

/**
 * @brief Minimal registration-map interface for FAST-LIO2 scan matching.
 *
 * The implementation is fixed to ikd-Tree. This interface is intentionally
 * small: it only exposes operations required by the estimator. Removed
 * capabilities (PCL fallback, stable indices, removed-point visualization)
 * are not needed in the production path.
 *
 * Reference: ikd-Tree, "An Incremental k-d Tree for Robotic Applications"
 * (https://github.com/hku-mars/ikd-Tree).
 */
class MapTreeInterface {
 public:
    virtual ~MapTreeInterface() = default;

    /// Build the tree from an initial point cloud.
    virtual void build(const CloudType::Ptr& cloud) = 0;

    /// Add points with optional incremental downsampling.
    virtual size_t addPoints(const PointVec& points, bool downsample = true) = 0;

    /// Delete points inside axis-aligned boxes.
    virtual size_t deletePoints(const std::vector<BoxPointType>& boxes) = 0;

    /// k-NN search returning neighbor points (ikd-Tree has no stable indices).
    virtual size_t nearestKSearchPoints(const PointType& query, int k,
                                        PointVec& neighbors,
                                        std::vector<float>& distances) const = 0;

    /// Number of points currently in the tree.
    virtual size_t size() const = 0;

    /// Set incremental downsampling resolution.
    virtual void setDownsampleParam(float resolution) = 0;

    /// Set sliding local-map bounding box.
    virtual void setLocalMapRange(const BoxPointType& box) = 0;

    /// Create the production ikd-Tree backend.
    static std::shared_ptr<MapTreeInterface> createIKDTree();
};

}  // namespace fast_lio

#endif  // FAST_LIO_MAP_TREE_INTERFACE_HPP_
