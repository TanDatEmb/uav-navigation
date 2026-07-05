#ifndef PX4_NAVIGATION_LOCAL_PLAN_GRID_HPP_
#define PX4_NAVIGATION_LOCAL_PLAN_GRID_HPP_

#include <Eigen/Dense>
#include <cstring>
#include <vector>

#include <px4_common/mapping/voxel_types.hpp>
#include <px4_common/math/grid.hpp>

namespace px4_navigation {

/**
 * @brief Dense 3D occupancy grid for local path planning.
 *
 * This class implements a dense occupancy grid used for local path planning
 * in the NED (North-East-Down) coordinate frame. It supports marking voxels
 * as occupied or free, inflating obstacles for safety margins, and querying
 * occupancy status at world coordinates.
 *
 * Coordinate Frame: NED (North-East-Down)
 * - X axis: North (positive) / South (negative)
 * - Y axis: East (positive) / West (negative)
 * - Z axis: Down (positive) / Up (negative)
 */
class LocalPlanGrid {
   public:
    /// Default XY span in metres centered on drone
    inline static constexpr double kGridXySpanM = 30.0;

    /// Default Z span in metres
    inline static constexpr double kGridZSpanM = 8.0;

    /// Default constructor
    LocalPlanGrid() = default;

    /**
     * @brief Reset the grid with new dimensions and parameters.
     *
     * @param size Size of the grid in voxels [size_x, size_y, size_z]
     * @param origin Origin of the grid in world coordinates (NED frame)
     * @param resolution Grid resolution in metres per voxel
     */
    void reset(const Eigen::Vector3i& size, const Eigen::Vector3d& origin, double resolution);

    /**
     * @brief Mark a voxel as occupied at the given world position.
     *
     * @param position World position in NED frame (metres)
     */
    void markOccupied(const Eigen::Vector3d& position);

    /**
     * @brief Mark a voxel as free at the given world position.
     *
     * @param position World position in NED frame (metres)
     */
    void markFree(const Eigen::Vector3d& position);

    /**
     * @brief Inflate obstacles in the XY plane with a configurable radius.
     *
     * @param inflation_radius_voxels Inflation radius in voxels
     */
    void inflateObstacles(int inflation_radius_voxels);

    /**
     * @brief Check if a world position is free.
     *
     * @param x X coordinate in NED frame (metres)
     * @param y Y coordinate in NED frame (metres)
     * @param z Z coordinate in NED frame (metres)
     * @return true if the position is free, false otherwise
     */
    bool isFree(double x, double y, double z) const;

    /**
     * @brief Check if a world position is occupied.
     *
     * @param x X coordinate in NED frame (metres)
     * @param y Y coordinate in NED frame (metres)
     * @param z Z coordinate in NED frame (metres)
     * @return true if the position is occupied, false otherwise
     */
    bool isOccupied(double x, double y, double z) const;

    /**
     * @brief Get the grid resolution in metres per voxel.
     * @return Grid resolution
     */
    double resolution() const {
        return resolution_;
    }

    /**
     * @brief Get the grid origin in world coordinates.
     * @return Grid origin in NED frame
     */
    const Eigen::Vector3d& origin() const {
        return origin_;
    }

    /**
     * @brief Get the grid size in voxels.
     * @return Grid size [size_x, size_y, size_z]
     */
    const Eigen::Vector3i& size() const {
        return size_;
    }

   private:
    /// Grid data as a flat array of occupancy values (0 = free, 1 = occupied)
    std::vector<uint8_t> grid_;

    /// Grid origin in world coordinates (NED frame)
    Eigen::Vector3d origin_ = Eigen::Vector3d::Zero();

    /// Grid size in voxels [size_x, size_y, size_z]
    Eigen::Vector3i size_ = Eigen::Vector3i::Zero();

    /// Grid resolution in metres per voxel
    double resolution_ = px4_common::mapping::kDefaultVoxelResolutionM;

    /// Inflation radius in voxels
    int inflation_radius_voxels_ = px4_common::mapping::kInflationVoxels;
};

}  // namespace px4_navigation

#endif  // PX4_NAVIGATION_LOCAL_PLAN_GRID_HPP_