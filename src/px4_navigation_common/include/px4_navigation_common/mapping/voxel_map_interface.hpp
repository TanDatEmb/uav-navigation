#ifndef PX4_NAVIGATION_COMMON_MAPPING_VOXEL_MAP_INTERFACE_HPP_
#define PX4_NAVIGATION_COMMON_MAPPING_VOXEL_MAP_INTERFACE_HPP_

#include <Eigen/Core>
#include <cstdint>
#include <vector>

namespace px4_navigation_common::mapping {

/**
 * @brief Abstract interface between the mapping layer and the navigation layer.
 *
 * The navigation layer holds a shared pointer to an implementation of this
 * interface. Direct method calls are used instead of ROS 2 pub/sub to avoid
 * serialization latency and keep the planning thread decoupled from DDS
 * timing.
 *
 * All positions are expressed in the PX4 NED frame with the EKF2 local
 * origin as reference.
 */
class IVoxMapManager {
   public:
    virtual ~IVoxMapManager() = default;

    /**
     * @brief Return the canonical voxel resolution in metres.
     * @return Voxel edge length in metres.
     */
    virtual double GetResolution() const = 0;

    /**
     * @brief Return occupied voxel centres within a radius of a query point.
     *
     * This is the source of truth for planning grid construction. The
     * caller owns the output vector and is encouraged to retain capacity
     * across calls to avoid heap allocation on the mapping side.
     *
     * Thread safety: implementations must take any required internal mutex
     * only for the duration of the voxel walk.
     *
     * @param center Query centre in the NED frame.
     * @param radius Search radius in metres.
     * @param[out] out Vector filled with occupied voxel centres in the NED frame.
     */
    virtual void GetOccupiedPointsInRadius(const Eigen::Vector3d &center, double radius,
                                           std::vector<Eigen::Vector3d> &out) = 0;

    /**
     * @brief Return true once the map has captured initial alignment.
     *
     * The navigation layer must check this before planning to avoid issuing
     * setpoints into an empty pre-alignment map.
     * @return true if the map is ready, false otherwise.
     */
    virtual bool IsReady() const noexcept = 0;

    /**
     * @brief Monotonic counter of cloud frames dropped by the mapping layer.
     *
     * Drops may occur due to pose lookup miss, alignment guard, or
     * non-monotonic timestamp. The navigation layer can read the delta over
     * a window to detect map staleness independently of topic latency.
     * @return Number of dropped frames since startup.
     */
    virtual std::uint64_t FramesDropped() const noexcept = 0;

    /**
     * @brief Return the lidar-to-IMU extrinsic translation in metres.
     *
     * This is the single source of truth for the sensor mount offset, loaded
     * by the mapping layer from configuration at startup. Other layers must
     * use this getter rather than parsing configuration independently to keep
     * calibration consistent.
     * @return Translation vector in metres.
     */
    virtual Eigen::Vector3d GetExtrinsicTranslation() const noexcept = 0;
};

}  // namespace px4_navigation_common::mapping

#endif  // PX4_NAVIGATION_COMMON_MAPPING_VOXEL_MAP_INTERFACE_HPP_
