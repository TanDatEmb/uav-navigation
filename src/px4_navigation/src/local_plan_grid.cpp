#include <px4_navigation/local_plan_grid.hpp>

#include <algorithm>
#include <cmath>

namespace px4_navigation {

void LocalPlanGrid::reset(const Eigen::Vector3i &size, const Eigen::Vector3d &origin,
                          double resolution) {
    size_ = size;
    origin_ = origin;
    resolution_ = resolution;

    const int total_cells = size.x() * size.y() * size.z();
    grid_.assign(total_cells, 0);
}

void LocalPlanGrid::markOccupied(const Eigen::Vector3d &position) {
    const Eigen::Vector3i index = px4_common::math::WorldToIndex(position, origin_, resolution_);
    if (!px4_common::math::IsIndexInBounds(index, size_)) {
        return;
    }

    const int address = px4_common::math::IndexToAddress(index, size_);
    grid_[address] = 1;
}

void LocalPlanGrid::markFree(const Eigen::Vector3d &position) {
    const Eigen::Vector3i index = px4_common::math::WorldToIndex(position, origin_, resolution_);
    if (!px4_common::math::IsIndexInBounds(index, size_)) {
        return;
    }

    const int address = px4_common::math::IndexToAddress(index, size_);
    grid_[address] = 0;
}

void LocalPlanGrid::inflateObstacles(int inflation_radius_voxels) {
    if (inflation_radius_voxels <= 0) {
        return;
    }

    inflation_radius_voxels_ = inflation_radius_voxels;

    const std::vector<uint8_t> original_grid = grid_;
    const double radius_sq = static_cast<double>(inflation_radius_voxels) * inflation_radius_voxels;

    for (int x = 0; x < size_.x(); ++x) {
        for (int y = 0; y < size_.y(); ++y) {
            for (int z = 0; z < size_.z(); ++z) {
                const Eigen::Vector3i current_index(x, y, z);
                const int current_address = px4_common::math::IndexToAddress(current_index, size_);

                if (original_grid[current_address] == 0) {
                    continue;
                }

                for (int dz = -inflation_radius_voxels; dz <= inflation_radius_voxels; ++dz) {
                    for (int dy = -inflation_radius_voxels; dy <= inflation_radius_voxels; ++dy) {
                        for (int dx = -inflation_radius_voxels; dx <= inflation_radius_voxels;
                             ++dx) {
                            const double distance_sq = dx * dx + dy * dy + dz * dz;
                            if (distance_sq > radius_sq) {
                                continue;
                            }

                            const Eigen::Vector3i inflated_index(x + dx, y + dy, z + dz);
                            if (!px4_common::math::IsIndexInBounds(inflated_index, size_)) {
                                continue;
                            }

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

bool LocalPlanGrid::isFree(double x, double y, double z) const {
    const Eigen::Vector3d position(x, y, z);
    const Eigen::Vector3i index = px4_common::math::WorldToIndex(position, origin_, resolution_);

    if (!px4_common::math::IsIndexInBounds(index, size_)) {
        return false;
    }

    const int address = px4_common::math::IndexToAddress(index, size_);
    return grid_[address] == 0;
}

bool LocalPlanGrid::isOccupied(double x, double y, double z) const {
    const Eigen::Vector3d position(x, y, z);
    const Eigen::Vector3i index = px4_common::math::WorldToIndex(position, origin_, resolution_);

    if (!px4_common::math::IsIndexInBounds(index, size_)) {
        return true;
    }

    const int address = px4_common::math::IndexToAddress(index, size_);
    return grid_[address] == 1;
}

}  // namespace px4_navigation
