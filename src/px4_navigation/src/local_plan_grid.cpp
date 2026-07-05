#include <px4_navigation/local_plan_grid.hpp>

#include <algorithm>
#include <cmath>

namespace px4_navigation {

void LocalPlanGrid::reset(const Eigen::Vector3i& size, const Eigen::Vector3d& origin,
                          double resolution) {
    size_ = size;
    origin_ = origin;
    resolution_ = resolution;

    // Calculate total number of cells
    const int total_cells = size.x() * size.y() * size.z();

    // Resize grid and initialize to zero (free)
    grid_.resize(total_cells, 0);
    std::fill(grid_.begin(), grid_.end(), 0);
}

void LocalPlanGrid::markOccupied(const Eigen::Vector3d& position) {
    // Convert world position to grid index
    const Eigen::Vector3i index = px4_common::math::WorldToIndex(position, origin_, resolution_);

    // Check if index is within bounds
    if (px4_common::math::IsIndexInBounds(index, size_)) {
        // Convert index to flat address and mark as occupied
        const int address = px4_common::math::IndexToAddress(index, size_);
        grid_[address] = 1;
    }
}

void LocalPlanGrid::markFree(const Eigen::Vector3d& position) {
    // Convert world position to grid index
    const Eigen::Vector3i index = px4_common::math::WorldToIndex(position, origin_, resolution_);

    // Check if index is within bounds
    if (px4_common::math::IsIndexInBounds(index, size_)) {
        // Convert index to flat address and mark as free
        const int address = px4_common::math::IndexToAddress(index, size_);
        grid_[address] = 0;
    }
}

void LocalPlanGrid::inflateObstacles(int inflation_radius_voxels) {
    // Store original grid data to avoid inflating already inflated cells
    const std::vector<uint8_t> original_grid = grid_;

    // Update inflation radius
    inflation_radius_voxels_ = inflation_radius_voxels;

    // Iterate through all cells
    for (int x = 0; x < size_.x(); ++x) {
        for (int y = 0; y < size_.y(); ++y) {
            for (int z = 0; z < size_.z(); ++z) {
                Eigen::Vector3i current_index(x, y, z);
                const int current_address = px4_common::math::IndexToAddress(current_index, size_);

                // If this cell was originally occupied, inflate around it
                if (original_grid[current_address] == 1) {
                    // Inflate in XY plane only
                    for (int dy = -inflation_radius_voxels; dy <= inflation_radius_voxels; ++dy) {
                        for (int dx = -inflation_radius_voxels; dx <= inflation_radius_voxels;
                             ++dx) {
                            // Check if within inflation bounds (square inflation)
                            if (std::abs(dx) <= inflation_radius_voxels &&
                                std::abs(dy) <= inflation_radius_voxels) {
                                // Calculate inflated index
                                Eigen::Vector3i inflated_index(x + dx, y + dy, z);

                                // Check if inflated index is within bounds
                                if (px4_common::math::IsIndexInBounds(inflated_index, size_)) {
                                    // Mark as occupied
                                    const int inflated_address =
                                        px4_common::math::IndexToAddress(inflated_index, size_);
                                    grid_[inflated_address] = 1;
                                }
                            }
                        }
                    }
                }
            }
        }
    }
}

bool LocalPlanGrid::isFree(double x, double y, double z) const {
    const Eigen::Vector3d position(x, y, z);
    const Eigen::Vector3i index = px4_common::math::WorldToIndex(position, origin_, resolution_);

    // Check if index is within bounds
    if (!px4_common::math::IsIndexInBounds(index, size_)) {
        return false;  // Out of bounds is considered occupied for safety
    }

    // Convert index to flat address and check occupancy
    const int address = px4_common::math::IndexToAddress(index, size_);
    return grid_[address] == 0;
}

bool LocalPlanGrid::isOccupied(double x, double y, double z) const {
    const Eigen::Vector3d position(x, y, z);
    const Eigen::Vector3i index = px4_common::math::WorldToIndex(position, origin_, resolution_);

    // Check if index is within bounds
    if (!px4_common::math::IsIndexInBounds(index, size_)) {
        return true;  // Out of bounds is considered occupied for safety
    }

    // Convert index to flat address and check occupancy
    const int address = px4_common::math::IndexToAddress(index, size_);
    return grid_[address] == 1;
}

}  // namespace px4_navigation