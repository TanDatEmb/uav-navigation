/**
* This file is part of ROG-Map
*
* Copyright 2024 Yunfan REN, MaRS Lab, University of Hong Kong, <mars.hku.hk>
* Developed by Yunfan REN <renyf at connect dot hku dot hk>
* for more information see <https://github.com/hku-mars/ROG-Map>.
* If you use this code, please cite the respective publications as
* listed on the above website.
*
* ROG-Map is free software: you can redistribute it and/or modify
* it under the terms of the GNU Lesser General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ROG-Map is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU Lesser General Public License
* along with ROG-Map. If not, see <http://www.gnu.org/licenses/>.
*/


#pragma once

#include <queue>
#include <cstddef>
#include <cstdint>
#include <rog_map/inf_map.h>
#include <rog_map/free_cnt_map.h>
#include <rog_map/esdf_map.h>
#include <rog_map/rog_map_core/raycaster.h>


namespace rog_map {
    using super_utils::Pose;


    class ProbMap : public SlidingMap {
    public:
        // Aggregate, per-update instrumentation. This is intentionally a
        // snapshot rather than a per-ray log.
        struct RaycastDiagnostics {
            std::uint64_t endpoint_count{0};
            std::uint64_t attempt_count{0};
            std::uint64_t processed_count{0};
            std::uint64_t clipped_count{0};
            std::uint64_t skipped_count{0};
            std::uint64_t skip_nonfinite{0};
            std::uint64_t skip_intensity{0};
            std::uint64_t skip_point_filter{0};
            std::uint64_t skip_below_raycast_min_range{0};
            std::uint64_t skip_endpoint_outside_local_map{0};
            std::uint64_t clipped_virtual_ground_or_ceiling{0};
            std::uint64_t clipped_raycast_max_range{0};
            std::uint64_t clipped_local_update_box{0};
            std::uint64_t ray_outside_local_map_step{0};
            std::uint64_t voxel_traversal_count_total{0};
            std::uint64_t voxel_traversal_count_max{0};
            std::uint64_t hit_candidate_count{0};
            std::uint64_t miss_candidate_count{0};
            std::uint64_t unique_hit_voxel_count{0};
            std::uint64_t unique_miss_voxel_count{0};
            std::uint64_t update_cache_entry_count{0};
            std::uint64_t unique_update_cache_voxel_count{0};
            std::uint64_t map_slide_check_count{0};
            std::uint64_t map_slide_count{0};
            std::int64_t map_slide_voxel_shift_x{0};
            std::int64_t map_slide_voxel_shift_y{0};
            std::int64_t map_slide_voxel_shift_z{0};
            std::uint64_t map_slide_cells_cleared{0};
            std::uint64_t inflation_update_count{0};
            std::int64_t rog_total_update_us{0};
            std::int64_t rog_raycast_us{0};
            std::int64_t rog_probability_update_us{0};
            std::int64_t rog_inflation_us{0};
            std::int64_t rog_slide_us{0};
            std::uint64_t allocated_voxel_count{0};
        };

        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        typedef std::shared_ptr<ProbMap> Ptr;

        ProbMap() = default;

        ~ProbMap() override = default;

        void initProbMap();

        bool isOccupied(const Vec3f &pos) const;

        bool isUnknown(const Vec3f &pos) const;

        bool isKnownFree(const Vec3f &pos) const;

        bool isOccupiedInflate(const Vec3f &pos) const;

        bool isUnknownInflate(const Vec3f &pos) const;

        bool isKnownFreeInflate(const Vec3f & pos) const;

        bool isFrontier(const Vec3f &pos) const;

        bool isFrontier(const Vec3i &id_g) const;

        // Query result
        GridType getGridType(Vec3i &id_g) const;

        GridType getGridType(const Vec3f &pos) const;

        GridType getInfGridType(const Vec3f &pos) const;

        GridType getInfBaseGridType(const Vec3i &id_g) const;

        double getMapValue(const Vec3f &pos) const;

        void boxSearch(const Vec3f &_box_min, const Vec3f &_box_max,
                       const GridType &gt, vec_E<Vec3f> &out_points) const;

        void boxSearchInflate(const Vec3f &box_min, const Vec3f &box_max,
                              const GridType &gt, vec_E<Vec3f> &out_points) const;

        void boundBoxByLocalMap(Vec3f &box_min, Vec3f &box_max) const;

        Vec3f getLocalMapOrigin() const;

        Vec3f getLocalMapSize() const;

        double getResolution() const{
            return sc_.resolution;
        }

        double getInfResolution()const {
            return inf_map_->getResolution();
        }

        void updateOccPointCloud(const PointCloud &input_cloud);

        void writeTimeConsumingToLog(std::ofstream &log_file);

        void writeMapInfoToLog(std::ofstream &log_file);

        void updateProbMap(const PointCloud &cloud, const Pose &pose);

        [[nodiscard]] const RaycastDiagnostics& lastDiagnostics() const noexcept {
            return last_diagnostics_;
        }

        [[nodiscard]] std::uint64_t allocatedVoxelCount() const noexcept {
            return static_cast<std::uint64_t>(sc_.map_vox_num);
        }

        // Sparse-checkpoint helper. This is intentionally not called from the
        // per-scan update path; it hashes the logical occupancy buffer only
        // when an offline benchmark requests a checkpoint.
        [[nodiscard]] std::uint64_t deterministicDigest() const noexcept;

    protected:
        rog_map::Config cfg_;
        InfMap::Ptr inf_map_;
        FreeCntMap::Ptr fcnt_map_;
        ESDFMap::Ptr esdf_map_;
        /// Spherical neighborhood lookup table
        std::vector<float> occupancy_buffer_;

        // uav-navigation local patch (see rog_map_vendor/UPSTREAM.md): guards
        // double-init of *this* instance. Upstream used a function-local
        // static, which made every ProbMap instance in the process share one
        // guard and made a second instance construction always throw. That is
        // incompatible with the public-frame-generation reset contract, which
        // requires destroying and reconstructing the map in the same process.
        bool initialized_once_{false};
        bool first_frame_clear_pending_{true};

        bool map_empty_{true};
        struct RaycastData {
            raycaster::RayCaster raycaster;
            std::queue<Vec3i> update_cache_id_g;
            std::vector<uint16_t> operation_cnt;
            std::vector<uint16_t> hit_cnt;
            Vec3f cache_box_max, cache_box_min, local_update_box_max, local_update_box_min;
            int batch_update_counter{0};
            std::mutex raycast_range_mtx;
        } raycast_data_;

        vector<double> time_consuming_;
        vector<string> time_consuming_name_{"Total", "Raycast", "Update_cache", "Inflation", "PointCloudNumber",
                                            "CacheNumber", "InflationNumber"};

        RaycastDiagnostics last_diagnostics_{};
        // standardization query
        // Known free < l_free
        // occupied >= l_occ
        bool isKnownFree(const double &prob) const {
            return prob < cfg_.l_free;
        }

        bool isOccupied(const double &prob) const {
            return prob >= cfg_.l_occ;
        }

        bool isUnknown(const double &prob) const {
            return prob >= cfg_.l_free && prob < cfg_.l_occ;
        }

        void slideAllMap(const Vec3f &pos);

        // warning using this function will cause memory leak if the id_g is not in the map
        bool isOccupied(const Vec3i &id_g) const;

        bool isUnknown(const Vec3i &id_g) const;

        bool isKnownFree(const Vec3i &id_g) const;

        //====================================================================
        void resetCell(const int &hash_id) override;

        void probabilisticMapFromCache();

        void hitPointUpdate(const Vec3f &pos, const int &hash_id, const int &hit_num);

        void missPointUpdate(const Vec3f &pos, const int &hash_id, const int &hit_num);

        void raycastProcess(const PointCloud &input_cloud, const Vec3f &cur_odom);

        void insertUpdateCandidate(const Vec3i &id_g, bool is_hit);

        void updateLocalBox(const Vec3f &cur_odom);

        void resetLocalMap() override;
    };
}
