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
        float downsample_resolution = 0.15f,
        int rebuild_threshold = 1500
    );

    ~IKDTreeBackend() override;

    // Disable copy
    IKDTreeBackend(const IKDTreeBackend&) = delete;
    IKDTreeBackend& operator=(const IKDTreeBackend&) = delete;

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
    bool rebuilding() const override;

    // Configuration
    void setDownsampleParam(float resolution) override;
    void setLocalMapRange(const BoxPointType& box) override;

    // Accessors
    void getAllPoints(PointVec& points) const override;
    void getRemovedPoints(PointVec& points) override;

    // ikd-Tree specific
    void acquireRemovedPoints(PointVec& points);
    void forceRebuild();

 private:
    // ikd-Tree instance
    KD_TREE<PointType> tree_;

    // External mutex for thread-safe interface
    mutable std::shared_mutex mutex_;

    // Configuration
    struct Config {
        float delete_criterion;
        float balance_criterion;
        float downsample_resolution;
        int rebuild_threshold;
    } config_;
};

}  // namespace fast_lio

#endif  // FAST_LIO_IKDTREE_IMPL_HPP_
