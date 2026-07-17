#ifndef FAST_LIO_IKDTREE_IMPL_HPP_
#define FAST_LIO_IKDTREE_IMPL_HPP_

#include "fast_lio/spatial_index.hpp"
#include "fast_lio/ikd/ikd_tree.h"

#include <mutex>
#include <shared_mutex>

namespace fast_lio {

/**
 * @brief ikd-Tree wrapper implementation (incremental, real-time).
 *
 * Wraps the HKU ikd-Tree implementation with thread-safe interface.
 * Uses pthread internally (ikd-Tree native) but provides std::shared_mutex
 * for external synchronization.
 *
 * Reference: https://github.com/hku-mars/ikd-Tree
 * Paper: "ikd-Tree: An Incremental k-d Tree for Robotic Applications"
 */
class IKDTreeBackend : public MapTreeInterface {
 public:
    /**
     * @brief Constructor with UAV-optimized defaults.
     */
    explicit IKDTreeBackend(
        float delete_criterion = 0.4f,
        float balance_criterion = 0.6f,
        float downsample_resolution = 0.15f
    );

    ~IKDTreeBackend() override;

    // Disable copy
    IKDTreeBackend(const IKDTreeBackend&) = delete;
    IKDTreeBackend& operator=(const IKDTreeBackend&) = delete;

    void build(const CloudType::Ptr& cloud) override;
    size_t addPoints(const PointVec& points, bool downsample = true) override;
    size_t deletePoints(const std::vector<BoxPointType>& boxes) override;
    size_t nearestKSearchPoints(const PointType& query, int k,
                                PointVec& neighbors,
                                std::vector<float>& distances) const override;
    size_t size() const override;
    void setDownsampleParam(float resolution) override;

 private:
    /// Find the existing point inside the voxel of @p query that is closest to
    /// the voxel center. Returns true if a representative was found.
    bool findVoxelRepresentative(const PointType& query, float resolution,
                                 PointType& representative);

    // ikd-Tree instance
    KD_TREE<PointType> tree_;

    // External mutex for thread-safe interface
    mutable std::shared_mutex mutex_;

    // Configuration
    struct Config {
        float delete_criterion;
        float balance_criterion;
        float downsample_resolution;
    } config_;
};

}  // namespace fast_lio

#endif  // FAST_LIO_IKDTREE_IMPL_HPP_
