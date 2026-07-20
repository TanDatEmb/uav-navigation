#ifndef PX4_MAPPING_VOXEL_HPP_
#define PX4_MAPPING_VOXEL_HPP_

#include <cstddef>
#include <cstdint>
#include <vector>

#include <Eigen/Core>

namespace px4_mapping {

/**
 * @brief Single voxel cell state.
 *
 * Voxel stores log-odds occupancy, intensity, and LRU metadata.
 * Uses EIGEN_MAKE_ALIGNED_OPERATOR_NEW for proper Eigen alignment.
 */
struct Voxel {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    Eigen::Vector3i index{Eigen::Vector3i::Zero()};
    float log_odds{0.0f};
    float intensity{0.0f};
    bool is_occupied{false};
    std::uint64_t last_update_id{0U};

    // LRU list pointers - kept as raw pointers for compatibility with VoxelHashMap
    Voxel *prev{nullptr};
    Voxel *next{nullptr};

    /**
     * @brief Index into VoxelHashMap::occupied_list_, or -1 if not occupied.
     */
    int occupied_list_index{-1};

    void Init(const Eigen::Vector3i &idx) {
        index = idx;
        log_odds = 0.0f;
        intensity = 0.0f;
        is_occupied = false;
        last_update_id = 0U;
        prev = nullptr;
        next = nullptr;
        occupied_list_index = -1;
    }
};

/**
 * @brief Pre-allocated memory pool for voxels with proper C++ construction.
 *
 * Uses std::vector with Eigen aligned allocator instead of malloc/free.
 * Freed voxels are pushed onto a free stack for immediate reuse.
 */
class VoxelPool {
   public:
    explicit VoxelPool(std::size_t capacity) : capacity_(capacity) {
        storage_.resize(capacity);
        free_stack_.reserve(capacity);
        for (std::size_t i = 0; i < capacity; ++i) {
            free_stack_.push_back(&storage_[i]);
        }
    }

    ~VoxelPool() = default;

    VoxelPool(const VoxelPool &) = delete;
    VoxelPool &operator=(const VoxelPool &) = delete;
    VoxelPool(VoxelPool &&) = default;
    VoxelPool &operator=(VoxelPool &&) = default;

    /**
     * @brief Allocate a fresh voxel from the pool.
     * @param idx Voxel index to store in the returned voxel.
     * @return Pointer to an initialized voxel, or nullptr if the pool is empty.
     */
    Voxel *Allocate(const Eigen::Vector3i &idx) {
        if (free_stack_.empty()) {
            return nullptr;
        }

        Voxel *voxel = free_stack_.back();
        free_stack_.pop_back();
        voxel->Init(idx);
        return voxel;
    }

    /**
     * @brief Return a voxel to the pool for reuse.
     * @param voxel Pointer returned by Allocate().
     */
    void Deallocate(Voxel *voxel) {
        if (voxel != nullptr) {
            free_stack_.push_back(voxel);
        }
    }

    std::size_t Available() const {
        return free_stack_.size();
    }
    std::size_t Capacity() const {
        return capacity_;
    }

   private:
    std::size_t capacity_{0U};
    std::vector<Voxel, Eigen::aligned_allocator<Voxel>> storage_;
    std::vector<Voxel *> free_stack_;
};

}  // namespace px4_mapping

#endif  // PX4_MAPPING_VOXEL_HPP_
