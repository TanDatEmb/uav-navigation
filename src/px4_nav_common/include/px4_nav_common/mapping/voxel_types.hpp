#ifndef PX4_NAV_COMMON_MAPPING_VOXEL_TYPES_HPP_
#define PX4_NAV_COMMON_MAPPING_VOXEL_TYPES_HPP_

#include <cmath>
#include <cstdint>

namespace px4_nav_common::mapping {

/**
 * @brief Default voxel resolution for the local occupancy map in metres.
 *
 * This value is the canonical resolution used by both the sparse voxel hash
 * map (Layer 2) and the dense planning grid (Layer 3). All grid index
 * conversions use this value unless overridden by parameters.
 */
inline constexpr double kDefaultVoxelResolutionM = 0.2;

/**
 * @brief Hard safety cap on the number of voxels retained by the sparse map.
 *
 * Distance-based eviction is the primary memory management mechanism and
 * normally keeps the voxel count well below this limit.
 */
inline constexpr std::size_t kMaxVoxels = 1800000U;

/**
 * @brief Log-odds update value for a hit (occupied) voxel.
 */
inline constexpr float kLogOddsHit = 0.8f;

/**
 * @brief Log-odds update value for a miss (free) voxel.
 */
inline constexpr float kLogOddsMiss = -0.3f;

/**
 * @brief Minimum allowed log-odds value.
 *
 * Voxel occupancy probabilities are clamped to this lower bound to avoid
 * overconfidence in free space.
 */
inline constexpr float kLogOddsMin = -1.2f;

/**
 * @brief Maximum allowed log-odds value.
 */
inline constexpr float kLogOddsMax = 1.6f;

/**
 * @brief Log-odds threshold above which a voxel is considered occupied.
 */
inline constexpr float kLogOddsOccupiedThreshold = 0.6f;

/**
 * @brief Maximum ray length for raycasting in metres.
 *
 * Rays longer than this value are traversed only to the limit. Their clipped
 * endpoints are treated as free space rather than occupied returns.
 */
inline constexpr double kMaxRayLengthM = 20.0;

/**
 * @brief Maximum voxel frame age before age-based eviction.
 *
 * Age eviction is secondary to distance-based eviction.
 */
inline constexpr std::uint32_t kMaxFrameAge = 18000U;

/**
 * @brief Distance-based eviction radius in metres.
 *
 * Voxels further than this radius from the drone are removed. The radius
 * is set above the planning query radius (15 m) by a safety margin so that
 * voxels at the planning boundary do not flicker as they approach the edge.
 */
inline constexpr double kEvictRadiusM = 18.0;

/**
 * @brief Period between distance-based eviction sweeps in milliseconds.
 */
inline constexpr std::uint32_t kEvictIntervalMs = 500U;

/**
 * @brief Default inflation radius in voxels for the dense planning grid.
 *
 * Each occupied voxel is inflated by this many voxels in the XY plane to
 * enforce a safety margin. The actual margin is
 * kInflationVoxels * kDefaultVoxelResolutionM.
 */
inline constexpr int kInflationVoxels = 1;

/**
 * @brief Maximum update time budget for a single point cloud frame.
 *
 * Remaining points are dropped if integration exceeds this duration.
 */
inline constexpr std::int64_t kUpdateTimeoutUs = 50000LL;

/**
 * @brief Internal pool size for voxel containers.
 *
 * Allocated slightly above kMaxVoxels to reduce reallocation during
 * operation.
 */
inline constexpr std::size_t kVoxelPoolSize = kMaxVoxels + 10000U;

}  // namespace px4_nav_common::mapping

#endif  // PX4_NAV_COMMON_MAPPING_VOXEL_TYPES_HPP_
