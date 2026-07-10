#include <px4_navigation/local_plan_grid.hpp>

#include <algorithm>
#include <cmath>

namespace px4_navigation {

void LocalPlanGrid::Reset(const Eigen::Vector3i &size, const Eigen::Vector3d &origin,
                          double resolution) {
    size_ = size;
    origin_ = origin;
    resolution_ = resolution;

    const int total_cells = size.x() * size.y() * size.z();
    grid_.assign(total_cells, 0);
}

void LocalPlanGrid::MarkOccupied(const Eigen::Vector3d &position) {
    const Eigen::Vector3i index = px4_common::math::WorldToIndex(position, origin_, resolution_);
    if (!px4_common::math::IsIndexInBounds(index, size_)) {
        return;
    }

    const int address = px4_common::math::IndexToAddress(index, size_);
    grid_[address] = 1;
}

void LocalPlanGrid::MarkFree(const Eigen::Vector3d &position) {
    const Eigen::Vector3i index = px4_common::math::WorldToIndex(position, origin_, resolution_);
    if (!px4_common::math::IsIndexInBounds(index, size_)) {
        return;
    }

    const int address = px4_common::math::IndexToAddress(index, size_);
    grid_[address] = 0;
}

void LocalPlanGrid::InflateObstacles(int inflation_radius_voxels) {
    if (inflation_radius_voxels <= 0) {
        return;
    }

    inflation_radius_voxels_ = inflation_radius_voxels;

    const std::vector<uint8_t> original_grid = grid_;
    const double radius_sq = static_cast<double>(inflation_radius_voxels) * inflation_radius_voxels;

    // Precompute the spherical stencil once. For a radius of 5 voxels this
    // replaces ~11^3 = 1331 distance checks per occupied voxel with a compact
    // list of ~515 offsets.
    std::vector<Eigen::Vector3i> stencil;
    stencil.reserve(
        static_cast<size_t>((2 * inflation_radius_voxels + 1) * (2 * inflation_radius_voxels + 1) *
                            (2 * inflation_radius_voxels + 1)));
    for (int dz = -inflation_radius_voxels; dz <= inflation_radius_voxels; ++dz) {
        for (int dy = -inflation_radius_voxels; dy <= inflation_radius_voxels; ++dy) {
            for (int dx = -inflation_radius_voxels; dx <= inflation_radius_voxels; ++dx) {
                const double distance_sq = dx * dx + dy * dy + dz * dz;
                if (distance_sq <= radius_sq + 1e-9) {
                    stencil.emplace_back(dx, dy, dz);
                }
            }
        }
    }

    const int total_voxels = size_.x() * size_.y() * size_.z();
    for (int address = 0; address < total_voxels; ++address) {
        if (original_grid[address] == 0) {
            continue;
        }

        const Eigen::Vector3i current_index = px4_common::math::AddressToIndex(address, size_);

        for (const auto &offset : stencil) {
            const Eigen::Vector3i inflated_index = current_index + offset;
            if (!px4_common::math::IsIndexInBounds(inflated_index, size_)) {
                continue;
            }

            const int inflated_address = px4_common::math::IndexToAddress(inflated_index, size_);
            grid_[inflated_address] = 1;
        }
    }
}

bool LocalPlanGrid::IsFree(double x, double y, double z) const {
    const Eigen::Vector3d position(x, y, z);
    const Eigen::Vector3i index = px4_common::math::WorldToIndex(position, origin_, resolution_);

    if (!px4_common::math::IsIndexInBounds(index, size_)) {
        return false;
    }

    const int address = px4_common::math::IndexToAddress(index, size_);
    return grid_[address] == 0;
}

bool LocalPlanGrid::IsOccupied(double x, double y, double z) const {
    const Eigen::Vector3d position(x, y, z);
    const Eigen::Vector3i index = px4_common::math::WorldToIndex(position, origin_, resolution_);

    if (!px4_common::math::IsIndexInBounds(index, size_)) {
        return true;
    }

    const int address = px4_common::math::IndexToAddress(index, size_);
    return grid_[address] == 1;
}

}  // namespace px4_navigation
