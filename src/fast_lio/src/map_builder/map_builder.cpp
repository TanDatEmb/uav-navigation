// Copyright (c) 2026, LeTanDat
// SPDX-License-Identifier: BSD-3-Clause

#include "fast_lio/map_builder.hpp"
#include "fast_lio/ikd_tree_backend.hpp"
#include "fast_lio/lidar_deskewer.hpp"

#include <iostream>

namespace fast_lio {

std::shared_ptr<MapTreeInterface> MapTreeInterface::createIKDTree() {
    return std::make_shared<IKDTreeBackend>();
}

MapBuilder::MapBuilder(const Config& config, std::shared_ptr<Estimator> kf)
    : config_(config), kf_(kf), status_(BuilderStatus::INITIALIZING), imu_init_count_(0) {
    kf_->configure(config_);
    imu_processor_ = std::make_unique<IMUProcessor>(config);
    createMapTree();
    lidar_processor_ = std::make_shared<LidarProcessor>(config, kf, map_tree_);
    deskewer_ = std::make_unique<LidarDeskewer>();
}

void MapBuilder::createMapTree() {
    map_tree_ = MapTreeInterface::createIKDTree();
    map_tree_->setDownsampleParam(config_.map_resolution);
    std::cout << "[MapBuilder] Using ikd-Tree (incremental)"
              << " with resolution=" << config_.map_resolution << std::endl;
}

LidarUpdateResult MapBuilder::process(SyncPackage& package) {
    LidarUpdateResult result;

    // Initialize IMU if not done
    if (status_ == BuilderStatus::INITIALIZING) {
        if (initializeIMU(package.imus)) {
            std::cout << "[MapBuilder] IMU initialized, mean_acc=["
                      << imu_processor_->getMeanAcc().transpose()
                      << "], accel_scale=" << imu_processor_->getAccelScale() << std::endl;

            if (config_.gravity_align) {
                kf_->initWithGravity(imu_processor_->getMeanAcc());
            }

            State15 init_state = kf_->getState();
            init_state.b_w = imu_processor_->getMeanGyro();
            init_state.T_I_L = SE3d(SO3d(config_.R_I_L), config_.t_I_L);
            kf_->setState(init_state);
            status_ = BuilderStatus::MAPPING;
        } else {
            return result;
        }
    }

    if (status_ != BuilderStatus::MAPPING) return result;

    // 1. Forward propagation: build trajectory + predict state to scan_end
    ImuTrajectory trajectory = imu_processor_->propagate(
        kf_, package.imus, package.cloud_start_time, package.cloud_end_time,
        package.imu_before_scan, package.imu_after_scan);

    // 2. Deskew point cloud (if needed)
    NormalizedLidarScan scan;
    scan.cloud = package.cloud;
    scan.scan_start_time_ns = static_cast<std::int64_t>(package.cloud_start_time * 1e9);
    scan.scan_end_time_ns = static_cast<std::int64_t>(package.cloud_end_time * 1e9);
    scan.has_per_point_time = package.has_per_point_time;
    scan.timing_model = package.has_per_point_time ? LidarTimingModel::kAbsolutePointTime
                                                   : LidarTimingModel::kSnapshot;

    CloudType::Ptr cloud_for_matching = package.cloud;  // Default: use raw

    if (LidarDeskewer::needsDeskew(scan) && trajectory.size() >= 2) {
        const SE3d T_I_L = kf_->getState().T_I_L;
        const SE3d T_W_I_end(kf_->getState().R_wb, kf_->getState().p_w);
        DeskewResult deskew = deskewer_->deskewToScanEnd(scan, trajectory, T_I_L, T_W_I_end);
        
        if (deskew.status == DeskewStatus::kSuccess) {
            cloud_for_matching = deskew.cloud;
        } else {
            // Deskew failed - log warning and skip this scan
            std::cerr << "[MapBuilder] Deskew failed (status=" << static_cast<int>(deskew.status)
                      << "), skipping scan" << std::endl;
            return result;
        }
    }

    // 3. LiDAR scan matching + estimator update
    // Create temporary package with deskewed cloud for matching
    SyncPackage matching_package = package;
    matching_package.cloud = cloud_for_matching;
    result = lidar_processor_->process(matching_package);

    std::cout << "[MapBuilder] LidarUpdate: status=" << static_cast<int>(result.status)
              << " input=" << result.input_points << " down=" << result.downsampled_points
              << " queried=" << result.queried_points << " planes=" << result.plane_candidates
              << " accepted=" << result.accepted_correspondences
              << " update=" << result.update_applied << " inserted=" << result.map_inserted
              << " converged=" << result.converged << std::endl;

    return result;
}

bool MapBuilder::initializeIMU(const std::deque<IMUData>& imu_buffer) {
    return imu_processor_->initialize(imu_buffer);
}

SE3d MapBuilder::getLiDARPose() const {
    return lidar_processor_->getLiDARPose();
}

SE3d MapBuilder::getIMUPose() const {
    const auto& state = kf_->getState();
    return state.getPose();
}

ImuInitializationDiagnostics MapBuilder::imuInitializationDiagnostics() const {
    if (!imu_processor_) {
        return ImuInitializationDiagnostics{};
    }
    return imu_processor_->initializationDiagnostics();
}

}  // namespace fast_lio