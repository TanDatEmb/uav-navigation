#ifndef PX4_COMMON_MATH_GRID_HPP_
#define PX4_COMMON_MATH_GRID_HPP_

#include <math.h>
#include <Eigen/Dense>

namespace px4_common::math {

/**
 * @brief Convert a world position to a discrete voxel/grid index.
 *
 * The origin is the lower-left-bottom corner of grid cell (0, 0, 0). A point
 * exactly on the origin maps to index 0 in that dimension.
 *
 * @param position World position in metres.
 * @param origin Grid origin in metres.
 * @param resolution Grid resolution in metres per cell.
 * @return Discrete grid index.
 */
inline Eigen::Vector3i WorldToIndex(const Eigen::Vector3d &position, const Eigen::Vector3d &origin,
                                    double resolution) noexcept {
    const double inv_resolution = 1.0 / resolution;
    return Eigen::Vector3i(
        static_cast<int>(std::floor((position.x() - origin.x()) * inv_resolution)),
        static_cast<int>(std::floor((position.y() - origin.y()) * inv_resolution)),
        static_cast<int>(std::floor((position.z() - origin.z()) * inv_resolution)));
}

/**
 * @brief Convert a discrete grid index to the centre of the corresponding world cell.
 *
 * @param index Discrete grid index.
 * @param origin Grid origin in metres.
 * @param resolution Grid resolution in metres per cell.
 * @return World position of the cell centre in metres.
 */
inline Eigen::Vector3d IndexToWorld(const Eigen::Vector3i &index, const Eigen::Vector3d &origin,
                                    double resolution) noexcept {
    return Eigen::Vector3d(origin.x() + (index.x() + 0.5) * resolution,
                           origin.y() + (index.y() + 0.5) * resolution,
                           origin.z() + (index.z() + 0.5) * resolution);
}

/**
 * @brief Check whether a 3D index lies inside grid bounds.
 * @param index Discrete grid index.
 * @param size Grid size as [size_x, size_y, size_z].
 * @return true if the index is inside the grid, false otherwise.
 */
inline bool IsIndexInBounds(const Eigen::Vector3i &index, const Eigen::Vector3i &size) noexcept {
    return index.x() >= 0 && index.x() < size.x() && index.y() >= 0 && index.y() < size.y() &&
           index.z() >= 0 && index.z() < size.z();
}

/**
 * @brief Convert a 3D index to a flat 1D array address using row-major order.
 *
 * Addressing follows x * (size_y * size_z) + y * size_z + z.
 *
 * @param index Discrete grid index.
 * @param size Grid size as [size_x, size_y, size_z].
 * @return Flat array address. The caller must ensure the index is in bounds.
 */
inline int IndexToAddress(const Eigen::Vector3i &index, const Eigen::Vector3i &size) noexcept {
    return index.x() * size.y() * size.z() + index.y() * size.z() + index.z();
}

/**
 * @brief Convert a flat 1D array address back to a 3D index.
 *
 * This is the inverse of IndexToAddress.
 *
 * @param address Flat array address.
 * @param size Grid size as [size_x, size_y, size_z].
 * @return Discrete grid index.
 */
inline Eigen::Vector3i AddressToIndex(int address, const Eigen::Vector3i &size) noexcept {
    const int yz_plane = size.y() * size.z();
    Eigen::Vector3i index;
    index.x() = address / yz_plane;
    const int remainder = address % yz_plane;
    index.y() = remainder / size.z();
    index.z() = remainder % size.z();
    return index;
}

/**
 * @brief Check whether a world position lies inside the grid volume.
 * @param position World position in metres.
 * @param origin Grid origin in metres.
 * @param size Grid size as [size_x, size_y, size_z].
 * @param resolution Grid resolution in metres per cell.
 * @return true if the position maps to an in-bounds index, false otherwise.
 */
inline bool IsPositionInMap(const Eigen::Vector3d &position, const Eigen::Vector3d &origin,
                            const Eigen::Vector3i &size, double resolution) noexcept {
    return IsIndexInBounds(WorldToIndex(position, origin, resolution), size);
}

}  // namespace px4_common::math

#endif  // PX4_COMMON_MATH_GRID_HPP_
