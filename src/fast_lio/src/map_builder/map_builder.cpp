#include "fast_lio/map_builder.hpp"
#include "fast_lio/ikd_tree_backend.hpp"
#include "fast_lio/pcl_tree_backend.hpp"

#include <iostream>

namespace fast_lio {

// Factory methods — merged from map_tree_factory.cpp
std::shared_ptr<MapTreeInterface> MapTreeInterface::createPCLTree() {
    return std::make_shared<PCLTreeBackend>();
}

std::shared_ptr<MapTreeInterface> MapTreeInterface::createIKDTree() {
    return std::make_shared<IKDTreeBackend>();
}

MapBuilder::MapBuilder(const Config& config, std::shared_ptr<IESKF> kf)
    : config_(config), kf_(kf), status_(BuilderStatus::INITIALIZING), imu_init_count_(0) {
    kf_->configure(config_);
    imu_processor_ = std::make_unique<IMUProcessor>(config);
    createMapTree();
    lidar_processor_ = std::make_shared<LidarProcessor>(config, kf, map_tree_);
}

void MapBuilder::createMapTree() {
    // Create ikd-Tree for real-time performance
    map_tree_ = MapTreeInterface::createIKDTree();
    map_tree_->setDownsampleParam(config_.map_resolution);

    std::cout << "[MapBuilder] Using ikd-Tree (incremental)"
              << " with resolution=" << config_.map_resolution << std::endl;
}

void MapBuilder::process(SyncPackage& package) {
    // Initialize IMU if not done
    if (status_ == BuilderStatus::INITIALIZING) {
        if (initializeIMU(package.imus)) {
            std::cout << "[MapBuilder] IMU initialized, " << "mean_acc=["
                      << imu_processor_->getMeanAcc().transpose()
                      << "], accel_scale=" << imu_processor_->getAccelScale() << std::endl;

            if (config_.gravity_align) {
                kf_->initWithGravity(imu_processor_->getMeanAcc());
            }

            // Preserve the gravity-aligned filter state while applying the calibrated
            // LiDAR-to-IMU transform. Constructing a default State15 here previously
            // discarded the initialized attitude.
            State15 init_state = kf_->getState();
            init_state.b_w = imu_processor_->getMeanGyro();
            init_state.T_imu_lidar = SE3d(SO3d(config_.r_il), config_.t_il);
            kf_->setState(init_state);
            status_ = BuilderStatus::MAPPING;
        } else {
            return;  // Wait for more IMU data
        }
    }

    if (status_ != BuilderStatus::MAPPING)
        return;

    // IMU propagation
    imu_processor_->processIMU(kf_, package.imus, package.cloud_end_time);

    // LiDAR update
    lidar_processor_->process(package);
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

}  // namespace fast_lio
