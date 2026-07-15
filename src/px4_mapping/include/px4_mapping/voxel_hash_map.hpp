#ifndef PX4_MAPPING_VOXEL_HASH_MAP_HPP_
#define PX4_MAPPING_VOXEL_HASH_MAP_HPP_

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <limits>
#include <mutex>
#include <unordered_map>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <px4_nav_common/mapping/voxel_map_interface.hpp>
#include <px4_nav_common/mapping/voxel_types.hpp>
#include <px4_nav_common/math/grid.hpp>
#include <px4_nav_common/types.hpp>

#include <px4_mapping/voxel.hpp>

namespace px4_mapping {

/**
 * @brief Layer 2 sparse voxel hash map with raycasting and LRU eviction.
 *
 * The map stores log-odds occupancy in a hash table keyed by voxel index.
 * A doubly-linked list orders voxels by recency so that memory pressure and
 * age-based eviction can prune from the tail in O(1). A separate contiguous
 * occupied_list_ stores pointers to currently-occupied voxels, allowing
 * planning queries to snapshot indices quickly and release the map mutex
 * before converting indices to world positions.
 */
class VoxelHashMap : public px4_nav_common::mapping::IVoxMapManager {
   public:
    VoxelHashMap()
        : pool_(px4_nav_common::mapping::kVoxelPoolSize),
          head_(nullptr),
          tail_(nullptr),
          current_size_(0U),
          occupied_count_(0U),
          frame_count_(0U),
          points_processed_(0U),
          timed_out_(false),
          ready_(false),
          frames_dropped_(0U),
          extrinsic_translation_(Eigen::Vector3d::Zero()) {
        map_table_.max_load_factor(0.8f);
        map_table_.reserve(px4_nav_common::mapping::kMaxVoxels * 1.5);
        occupied_list_.reserve(px4_nav_common::mapping::kMaxVoxels);
    }

    ~VoxelHashMap() override {
        ClearAll();
    }

    VoxelHashMap(const VoxelHashMap &) = delete;
    VoxelHashMap &operator=(const VoxelHashMap &) = delete;
    VoxelHashMap(VoxelHashMap &&) = delete;
    VoxelHashMap &operator=(VoxelHashMap &&) = delete;

    /**
     * @brief Reset the map to an empty state.
     */
    void Clear() {
        std::lock_guard<std::mutex> lock(map_mutex_);
        ClearAll();
    }

    /**
     * @brief Integrate a point cloud frame using 3D DDA raycasting.
     *
     * Intermediate voxels along each ray receive free-space evidence, the
     * endpoint receives occupied evidence. Integration aborts after a bounded
     * time budget to protect real-time performance.
     *
     * @param points Sensor-frame points in metres.
     * @param sensor_origin Sensor origin in the NED frame in metres.
     */
    void Update(const std::vector<px4_nav_common::PointLivox> &points,
                const Eigen::Vector3d &sensor_origin) {
        std::lock_guard<std::mutex> lock(map_mutex_);

        frame_count_++;
        new_voxels_.clear();
        kept_voxels_.clear();
        points_processed_ = 0U;
        timed_out_ = false;

        const auto deadline = std::chrono::steady_clock::now() +
                              std::chrono::microseconds(px4_nav_common::mapping::kUpdateTimeoutUs);

        for (std::size_t i = 0U; i < points.size(); ++i) {
            if ((i & 0xFFU) == 0xFFU) {
                if (std::chrono::steady_clock::now() >= deadline) {
                    timed_out_ = true;
                    break;
                }
            }

            const auto &pt = points[i];
            const Eigen::Vector3d endpoint(pt.x, pt.y, pt.z);
            Raycast(sensor_origin, endpoint, pt);
            points_processed_++;
        }

        ManageMemory();
        ready_ = true;
    }

    /**
     * @brief Return occupied voxels changed in the most recent Update().
     *
     * The internal new/kept voxel buffers are converted to world points while
     * holding the map mutex. Only the resulting point snapshot is released
     * before returning, so callers never hold dangling voxel pointers.
     *
     * @param[out] out Vector filled with occupied voxel centres.
     */
    void GetChangedPoints(std::vector<px4_nav_common::PointLivox> &out) {
        std::vector<px4_nav_common::PointLivox> new_snap;
        std::vector<px4_nav_common::PointLivox> keep_snap;

        {
            std::lock_guard<std::mutex> lock(map_mutex_);
            new_snap.reserve(new_voxels_.size());
            keep_snap.reserve(kept_voxels_.size());
            for (const Voxel *voxel : new_voxels_) {
                if (voxel->is_occupied) {
                    new_snap.push_back(VoxelToPoint(const_cast<Voxel *>(voxel)));
                }
            }
            for (const Voxel *voxel : kept_voxels_) {
                if (voxel->is_occupied) {
                    keep_snap.push_back(VoxelToPoint(const_cast<Voxel *>(voxel)));
                }
            }
        }

        out.clear();
        out.reserve(new_snap.size() + keep_snap.size());
        out.insert(out.end(), new_snap.begin(), new_snap.end());
        out.insert(out.end(), keep_snap.begin(), keep_snap.end());
    }

    /**
     * @brief Return all currently occupied voxels for debug/global dump.
     * @return Vector of occupied voxel centres.
     */
    std::vector<px4_nav_common::PointLivox> GetPointCloud() {
        std::lock_guard<std::mutex> lock(map_mutex_);

        std::vector<px4_nav_common::PointLivox> out;
        out.reserve(occupied_count_);

        Voxel *current = head_;
        while (current != nullptr) {
            if (current->is_occupied) {
                out.push_back(VoxelToPoint(current));
            }
            current = current->next;
        }
        return out;
    }

    std::uint64_t GetFrameCount() const noexcept {
        return frame_count_;
    }
    std::size_t Size() const noexcept {
        return current_size_;
    }
    std::size_t OccupiedCount() const noexcept {
        return occupied_count_;
    }
    std::size_t NewCount() const noexcept {
        return new_voxels_.size();
    }
    std::size_t UpdatedCount() const noexcept {
        return kept_voxels_.size();
    }
    std::size_t PointsProcessed() const noexcept {
        return points_processed_;
    }
    bool WasTimedOut() const noexcept {
        return timed_out_;
    }

    /**
     * @brief Evict voxels further than kEvictRadiusM from the drone.
     * @param drone_pos Drone position in the NED frame.
     * @return Number of evicted voxels.
     */
    std::size_t EvictDistant(const Eigen::Vector3d &drone_pos) {
        std::lock_guard<std::mutex> lock(map_mutex_);

        const double radius_voxels =
            px4_nav_common::mapping::kEvictRadiusM / px4_nav_common::mapping::kDefaultVoxelResolutionM;
        const double radius_sq = radius_voxels * radius_voxels;
        const Eigen::Vector3i drone_idx = px4_nav_common::math::WorldToIndex(
            drone_pos, Eigen::Vector3d::Zero(), px4_nav_common::mapping::kDefaultVoxelResolutionM);

        std::size_t evicted = 0U;
        Voxel *current = tail_;

        while (current != nullptr) {
            Voxel *previous = current->prev;
            const Eigen::Vector3i diff = current->index - drone_idx;
            const double distance_sq = diff.cast<double>().squaredNorm();

            if (distance_sq > radius_sq) {
                if (current->is_occupied) {
                    occupied_count_--;
                    RemoveFromOccupiedList(current);
                }
                RemoveFromFrameBuffers(current);
                map_table_.erase(current->index);
                DetachNode(current);
                pool_.Deallocate(current);
                current_size_--;
                evicted++;
            }
            current = previous;
        }

        return evicted;
    }

    // IVoxMapManager interface implementation.
    double GetResolution() const override {
        return px4_nav_common::mapping::kDefaultVoxelResolutionM;
    }

    void GetOccupiedPointsInRadius(const Eigen::Vector3d &center, double radius,
                                   std::vector<Eigen::Vector3d> &out) override {
        thread_local std::vector<Eigen::Vector3i> index_snapshot;

        // Step 1: fast index snapshot under lock.
        {
            std::lock_guard<std::mutex> lock(map_mutex_);
            index_snapshot.clear();
            index_snapshot.reserve(occupied_list_.size());
            for (Voxel *voxel : occupied_list_) {
                index_snapshot.push_back(voxel->index);
            }
        }

        // Step 2: convert and filter without holding the lock.
        out.clear();
        const double radius_sq = radius * radius;
        for (const Eigen::Vector3i &idx : index_snapshot) {
            const Eigen::Vector3d position = px4_nav_common::math::IndexToWorld(
                idx, Eigen::Vector3d::Zero(), px4_nav_common::mapping::kDefaultVoxelResolutionM);
            if ((position - center).squaredNorm() <= radius_sq) {
                out.push_back(position);
            }
        }
    }

    bool IsReady() const noexcept override {
        return ready_;
    }

    std::uint64_t FramesDropped() const noexcept override {
        return frames_dropped_;
    }

    Eigen::Vector3d GetExtrinsicTranslation() const noexcept override {
        return extrinsic_translation_;
    }

    void SetExtrinsicTranslation(const Eigen::Vector3d &translation) noexcept {
        extrinsic_translation_ = translation;
    }

    void IncrementFramesDropped() noexcept {
        frames_dropped_++;
    }

   private:
    VoxelPool pool_;
    std::unordered_map<Eigen::Vector3i, Voxel *, px4_nav_common::VoxelHash> map_table_;
    std::vector<Voxel *> new_voxels_;
    std::vector<Voxel *> kept_voxels_;
    std::vector<Voxel *> occupied_list_;

    Voxel *head_{nullptr};
    Voxel *tail_{nullptr};
    std::size_t current_size_{0U};
    std::size_t occupied_count_{0U};
    std::uint64_t frame_count_{0U};

    std::size_t points_processed_{0U};
    bool timed_out_{false};
    bool ready_{false};
    std::uint64_t frames_dropped_{0U};
    Eigen::Vector3d extrinsic_translation_{Eigen::Vector3d::Zero()};

    mutable std::mutex map_mutex_;

    px4_nav_common::PointLivox VoxelToPoint(Voxel *voxel) {
        const Eigen::Vector3d position = px4_nav_common::math::IndexToWorld(
            voxel->index, Eigen::Vector3d::Zero(), px4_nav_common::mapping::kDefaultVoxelResolutionM);

        px4_nav_common::PointLivox point;
        point.x = position.x();
        point.y = position.y();
        point.z = position.z();
        point.intensity = voxel->intensity;
        return point;
    }

    void AddToOccupiedList(Voxel *voxel) {
        if (voxel == nullptr || voxel->occupied_list_index >= 0) {
            return;
        }
        voxel->occupied_list_index = static_cast<int>(occupied_list_.size());
        occupied_list_.push_back(voxel);
    }

    void RemoveFromFrameBuffers(Voxel *voxel) {
        if (voxel == nullptr) {
            return;
        }
        new_voxels_.erase(
            std::remove(new_voxels_.begin(), new_voxels_.end(), voxel),
            new_voxels_.end());
        kept_voxels_.erase(
            std::remove(kept_voxels_.begin(), kept_voxels_.end(), voxel),
            kept_voxels_.end());
    }

    void RemoveFromOccupiedList(Voxel *voxel) {
        if (voxel->occupied_list_index < 0) {
            return;
        }

        const int idx = voxel->occupied_list_index;
        Voxel *last = occupied_list_.back();
        occupied_list_[idx] = last;
        last->occupied_list_index = idx;
        occupied_list_.pop_back();
        voxel->occupied_list_index = -1;
    }

    void Raycast(const Eigen::Vector3d &origin, const Eigen::Vector3d &endpoint,
                 const px4_nav_common::PointLivox &point) {
        const Eigen::Vector3d origin_grid = origin / px4_nav_common::mapping::kDefaultVoxelResolutionM;
        const Eigen::Vector3d endpoint_grid =
            endpoint / px4_nav_common::mapping::kDefaultVoxelResolutionM;

        Eigen::Vector3i start_idx = origin_grid.array().floor().cast<int>();
        Eigen::Vector3i end_idx = endpoint_grid.array().floor().cast<int>();

        if (start_idx == end_idx) {
            TouchVoxelHit(end_idx, point);
            return;
        }

        Eigen::Vector3d direction = endpoint - origin;
        double ray_length = direction.norm();
        if (ray_length < 1e-6) {
            return;
        }

        if (ray_length > px4_nav_common::mapping::kMaxRayLengthM) {
            direction = direction / ray_length * px4_nav_common::mapping::kMaxRayLengthM;
            end_idx = px4_nav_common::math::WorldToIndex(origin + direction, Eigen::Vector3d::Zero(),
                                                     px4_nav_common::mapping::kDefaultVoxelResolutionM);
            ray_length = px4_nav_common::mapping::kMaxRayLengthM;
        }

        direction /= ray_length;

        Eigen::Vector3i step;
        Eigen::Vector3d t_max;
        Eigen::Vector3d t_delta;

        for (int axis = 0; axis < 3; ++axis) {
            if (direction(axis) > 1e-9) {
                step(axis) = 1;
                const double boundary =
                    (start_idx(axis) + 1) * px4_nav_common::mapping::kDefaultVoxelResolutionM;
                t_max(axis) = (boundary - origin(axis)) / direction(axis);
                t_delta(axis) = px4_nav_common::mapping::kDefaultVoxelResolutionM / direction(axis);
            } else if (direction(axis) < -1e-9) {
                step(axis) = -1;
                const double boundary =
                    start_idx(axis) * px4_nav_common::mapping::kDefaultVoxelResolutionM;
                t_max(axis) = (boundary - origin(axis)) / direction(axis);
                t_delta(axis) = px4_nav_common::mapping::kDefaultVoxelResolutionM / (-direction(axis));
            } else {
                step(axis) = 0;
                t_max(axis) = std::numeric_limits<double>::max();
                t_delta(axis) = std::numeric_limits<double>::max();
            }
        }

        Eigen::Vector3i current = start_idx;
        const int max_steps =
            static_cast<int>(ray_length / px4_nav_common::mapping::kDefaultVoxelResolutionM) + 3;

        for (int step_count = 0; step_count < max_steps; ++step_count) {
            if (t_max.x() < t_max.y()) {
                if (t_max.x() < t_max.z()) {
                    current.x() += step.x();
                    t_max.x() += t_delta.x();
                } else {
                    current.z() += step.z();
                    t_max.z() += t_delta.z();
                }
            } else {
                if (t_max.y() < t_max.z()) {
                    current.y() += step.y();
                    t_max.y() += t_delta.y();
                } else {
                    current.z() += step.z();
                    t_max.z() += t_delta.z();
                }
            }

            if (current == end_idx) {
                TouchVoxelHit(end_idx, point);
                return;
            }

            TouchVoxelMiss(current);
        }

        TouchVoxelHit(end_idx, point);
    }

    void TouchVoxelHit(const Eigen::Vector3i &idx, const px4_nav_common::PointLivox &point) {
        auto it = map_table_.find(idx);

        if (it == map_table_.end()) {
            Voxel *voxel = pool_.Allocate(idx);
            if (voxel == nullptr) {
                return;
            }

            map_table_[idx] = voxel;
            current_size_++;
            voxel->last_update_id = frame_count_;
            voxel->intensity = point.intensity;
            UpdateOccupancy(voxel, px4_nav_common::mapping::kLogOddsHit);
            AttachToHead(voxel);
            new_voxels_.push_back(voxel);
        } else {
            Voxel *voxel = it->second;
            if (voxel->last_update_id != frame_count_) {
                voxel->last_update_id = frame_count_;
                voxel->intensity = point.intensity;
                UpdateOccupancy(voxel, px4_nav_common::mapping::kLogOddsHit);
                DetachNode(voxel);
                AttachToHead(voxel);
                kept_voxels_.push_back(voxel);
            }
        }
    }

    void TouchVoxelMiss(const Eigen::Vector3i &idx) {
        auto it = map_table_.find(idx);
        if (it == map_table_.end()) {
            return;
        }

        Voxel *voxel = it->second;
        if (voxel->last_update_id == frame_count_) {
            return;
        }

        voxel->last_update_id = frame_count_;
        UpdateOccupancy(voxel, px4_nav_common::mapping::kLogOddsMiss);
        DetachNode(voxel);
        AttachToHead(voxel);
        kept_voxels_.push_back(voxel);
    }

    void ClearAll() {
        while (head_ != nullptr) {
            Voxel *temp = head_;
            head_ = head_->next;
            pool_.Deallocate(temp);
        }
        head_ = nullptr;
        tail_ = nullptr;
        map_table_.clear();
        new_voxels_.clear();
        kept_voxels_.clear();
        occupied_list_.clear();
        current_size_ = 0U;
        occupied_count_ = 0U;
        frame_count_ = 0U;
    }

    bool UpdateOccupancy(Voxel *voxel, float delta) {
        const bool was_occupied = voxel->is_occupied;

        voxel->log_odds += delta;
        voxel->log_odds = std::clamp(voxel->log_odds, px4_nav_common::mapping::kLogOddsMin,
                                     px4_nav_common::mapping::kLogOddsMax);
        voxel->is_occupied = (voxel->log_odds >= px4_nav_common::mapping::kLogOddsOccupiedThreshold);

        if (!was_occupied && voxel->is_occupied) {
            occupied_count_++;
            AddToOccupiedList(voxel);
            return true;
        }
        if (was_occupied && !voxel->is_occupied) {
            occupied_count_--;
            RemoveFromOccupiedList(voxel);
        }
        return false;
    }

    void DetachNode(Voxel *node) {
        if (node->prev != nullptr) {
            node->prev->next = node->next;
        } else {
            head_ = node->next;
        }

        if (node->next != nullptr) {
            node->next->prev = node->prev;
        } else {
            tail_ = node->prev;
        }

        node->prev = nullptr;
        node->next = nullptr;
    }

    void AttachToHead(Voxel *node) {
        node->next = head_;
        node->prev = nullptr;
        if (head_ != nullptr) {
            head_->prev = node;
        }
        head_ = node;
        if (tail_ == nullptr) {
            tail_ = head_;
        }
    }

    void RemoveTail() {
        if (tail_ == nullptr) {
            return;
        }

        Voxel *temp = tail_;

        if (temp->is_occupied) {
            occupied_count_--;
            RemoveFromOccupiedList(temp);
        }

        RemoveFromFrameBuffers(temp);
        map_table_.erase(temp->index);

        if (tail_->prev != nullptr) {
            tail_ = tail_->prev;
            tail_->next = nullptr;
        } else {
            head_ = nullptr;
            tail_ = nullptr;
        }

        pool_.Deallocate(temp);
        current_size_--;
    }

    void ManageMemory() {
        while (current_size_ > px4_nav_common::mapping::kMaxVoxels) {
            RemoveTail();
        }

        while (tail_ != nullptr &&
               (frame_count_ - tail_->last_update_id) > px4_nav_common::mapping::kMaxFrameAge) {
            RemoveTail();
        }
    }
};

}  // namespace px4_mapping

#endif  // PX4_MAPPING_VOXEL_HASH_MAP_HPP_
