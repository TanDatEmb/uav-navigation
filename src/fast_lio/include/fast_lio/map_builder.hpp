#ifndef FAST_LIO_MAP_BUILDER_HPP_
#define FAST_LIO_MAP_BUILDER_HPP_

#include "fast_lio/commons.hpp"
#include "fast_lio/ieskf.hpp"
#include "fast_lio/imu_processor.hpp"
#include "fast_lio/lidar_processor.hpp"
#include "fast_lio/spatial_index.hpp"

#include <memory>

namespace fast_lio {

/**
 * @brief Main FAST-LIO2 map building pipeline (15-DOF).
 *
 * Integrates IMU propagation and LiDAR scan matching with IESKF.
 * Outputs odometry and registered point clouds.
 *
 * Architecture:
 *   IMU (200Hz) → IESKF predict
 *   LiDAR (10-20Hz) → LidarProcessor → IESKF update
 *   MapTree → Incremental local map
 *
 * Design references: docs/frame_contract.md and docs/ieskf_design.md
 */
class MapBuilder {
   public:
    /**
     * @brief Constructor.
     *
     * @param config Algorithm configuration
     * @param kf Shared IESKF instance (15-DOF)
     */
    MapBuilder(const Config& config, std::shared_ptr<IESKF> kf);

    /**
     * @brief Process synchronized package.
     *
     * Pipeline:
     * 1. Initialize IMU (if not done)
     * 2. IMU propagation to end of scan
     * 3. LiDAR scan matching with point-to-plane ICP
     * 4. IESKF iterated update
     * 5. Incremental map update
     *
     * @param package Synchronized LiDAR + IMU
     */
    void process(SyncPackage& package);

    /**
     * @brief Get current builder status.
     */
    BuilderStatus status() const {
        return status_;
    }

    /**
     * @brief Check if initialized and mapping.
     */
    bool isInitialized() const {
        return status_ == BuilderStatus::MAPPING;
    }

    /**
     * @brief Get IESKF instance.
     */
    std::shared_ptr<IESKF> getKF() const {
        return kf_;
    }

    /**
     * @brief Get LiDAR processor.
     */
    std::shared_ptr<LidarProcessor> getLidarProcessor() const {
        return lidar_processor_;
    }

    /**
     * @brief Get current LiDAR pose in world frame.
     *
     * T_lio_world_mid360_lidar = T_lio_world_mid360_imu * T_mid360_imu_mid360_lidar
     */
    SE3d getLiDARPose() const;

    /**
     * @brief Get current IMU pose in world frame.
     *
     * T_lio_world_mid360_imu
     */
    SE3d getIMUPose() const;

   private:
    Config config_;
    std::shared_ptr<IESKF> kf_;
    std::unique_ptr<IMUProcessor> imu_processor_;
    std::shared_ptr<LidarProcessor> lidar_processor_;
    std::shared_ptr<MapTreeInterface> map_tree_;

    BuilderStatus status_;
    int imu_init_count_;

    // Initialize IMU with static measurements
    bool initializeIMU(const std::deque<IMUData>& imu_buffer);

    // Create map tree (PCL fallback or ikd-Tree)
    void createMapTree();
};

}  // namespace fast_lio

#endif  // FAST_LIO_MAP_BUILDER_HPP_
