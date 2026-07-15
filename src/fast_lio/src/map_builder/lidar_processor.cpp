#include "fast_lio/lidar_processor.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <limits>

namespace fast_lio {

LidarProcessor::LidarProcessor(const Config& config, std::shared_ptr<IESKF> kf,
                               std::shared_ptr<MapTreeInterface> tree)
    : config_(config), kf_(kf), map_tree_(tree), local_map_{} {
    cloud_down_lidar_.reset(new CloudType);
    cloud_down_world_.reset(new CloudType);
    cloud_effect_lidar_.reset(new CloudType);
    cloud_effect_world_.reset(new CloudType);

    voxel_filter_.setLeafSize(config_.scan_resolution, config_.scan_resolution,
                              config_.scan_resolution);
}

void LidarProcessor::process(SyncPackage& package) {
    if (!package.cloud || package.cloud->empty()) {
        return;
    }

    // Downsample input cloud
    if (config_.scan_resolution > 0) {
        voxel_filter_.setInputCloud(package.cloud);
        voxel_filter_.filter(*cloud_down_lidar_);
    } else {
        pcl::copyPointCloud(*package.cloud, *cloud_down_lidar_);
    }

    // Filter by range
    CloudType::Ptr cloud_filtered(new CloudType);
    for (const auto& pt : cloud_down_lidar_->points) {
        double range = std::sqrt(pt.x * pt.x + pt.y * pt.y + pt.z * pt.z);
        if (range >= config_.lidar_min_range && range <= config_.lidar_max_range) {
            cloud_filtered->points.push_back(pt);
        }
    }
    cloud_filtered->width = cloud_filtered->points.size();
    cloud_filtered->height = 1;
    cloud_down_lidar_ = cloud_filtered;

    if (cloud_down_lidar_->empty())
        return;

    // Initialize the map in the world frame, just like every incremental scan.
    if (!local_map_.initialized) {
        initMap(cloud_down_lidar_);
        return;
    }

    // Update local map based on current position
    const auto& state = kf_->getState();
    Eigen::Vector3d pos_lidar = state.p_w + state.R_wb.matrix() * state.T_imu_lidar.translation();
    local_map_.update(pos_lidar, config_.cube_len, config_.move_thresh, config_.det_range);
    updateLocalMap();

    // IESKF update with point-to-plane constraints
    auto loss_func = [this](const State15& s, SharedState15& sh) {
        this->computePointToPlaneConstraint(s, sh);
    };

    auto stop_func = [](const V15D& delta) -> bool {
        // Convergence: rotation < 0.01 deg, translation < 0.015 m
        double rot_norm = delta.segment<3>(0).norm() * 180.0 / M_PI;
        double pos_norm = delta.segment<3>(3).norm();
        return (rot_norm < 0.01) && (pos_norm < 0.015);
    };

    kf_->setLossFunction(loss_func);
    kf_->setConvergenceCheck(stop_func);
    kf_->update(config_.ieskf_max_iter);

    // Update map
    incrementMap();
}

void LidarProcessor::initMap(const CloudType::Ptr& cloud) {
    const SE3d T_world_lidar = kf_->getState().getLiDARPose();
    cloud_down_world_ = transformCloud(cloud, T_world_lidar);
    map_tree_->build(cloud_down_world_);

    // Initialize local map bounds around the actual LiDAR origin in world coordinates.
    for (int i = 0; i < 3; ++i) {
        local_map_.corner.vertex_min[i] =
            static_cast<float>(T_world_lidar.translation()(i) - config_.cube_len / 2.0);
        local_map_.corner.vertex_max[i] =
            static_cast<float>(T_world_lidar.translation()(i) + config_.cube_len / 2.0);
    }
    local_map_.initialized = true;
}

void LidarProcessor::updateLocalMap() {
    // Delete points outside local map
    if (!local_map_.boxes_to_remove.empty()) {
        map_tree_->deletePoints(local_map_.boxes_to_remove);
        local_map_.boxes_to_remove.clear();
    }
}

void LidarProcessor::incrementMap() {
    if (cloud_down_lidar_->empty())
        return;

    // Transform points to world frame using current state
    const auto& state = kf_->getState();
    SE3d T_world_lidar = state.getLiDARPose();

    cloud_down_world_->clear();
    for (const auto& pt : cloud_down_lidar_->points) {
        Eigen::Vector3d p_lidar(pt.x, pt.y, pt.z);
        // Simplified transform - SE3d needs proper operator*
        Eigen::Vector3d p_world =
            T_world_lidar.rotation().matrix() * p_lidar + T_world_lidar.translation();

        PointType pt_world;
        pt_world.x = p_world.x();
        pt_world.y = p_world.y();
        pt_world.z = p_world.z();
        pt_world.intensity = pt.intensity;
        cloud_down_world_->points.push_back(pt_world);
    }
    cloud_down_world_->width = cloud_down_world_->points.size();
    cloud_down_world_->height = 1;

    // Add to map tree
    map_tree_->addPoints(cloud_down_world_->points, true);
}

void LidarProcessor::computePointToPlaneConstraint(const State15& state, SharedState15& shared) {
    // Point-to-plane constraint for IESKF
    // Jacobian: H = [H_θ, H_p, 0, 0, 0]
    // H_θ = -n^T * R * [p_body]×
    // H_p = n^T

    shared.reset(cloud_down_lidar_->points.size());

    SE3d T_world_lidar = state.getLiDARPose();

    for (size_t i = 0; i < cloud_down_lidar_->points.size(); ++i) {
        const auto& pt = cloud_down_lidar_->points[i];
        Eigen::Vector3d p_lidar(pt.x, pt.y, pt.z);

        // Transform to world
        Eigen::Vector3d p_world =
            T_world_lidar.rotation().matrix() * p_lidar + T_world_lidar.translation();

        // Find plane correspondence
        Eigen::Vector3d normal, plane_point;
        if (!findPlaneCorrespondence(p_lidar, p_world, normal, plane_point)) {
            continue;
        }

        // Transform point to IMU frame for Jacobian
        Eigen::Vector3d p_imu =
            state.T_imu_lidar.rotation().matrix() * p_lidar + state.T_imu_lidar.translation();

        // Point-to-plane residual: r = n^T * (p_world - q_plane)
        double residual = normal.dot(p_world - plane_point);

        // Jacobian w.r.t. rotation (right perturbation)
        // H_θ = -n^T * R * [p_imu]×
        Eigen::Matrix3d p_imu_skew = skewSymmetric(p_imu);
        Eigen::Vector3d H_theta =
            -(normal.transpose() * state.R_wb.matrix() * p_imu_skew).transpose();

        // Jacobian w.r.t. position
        Eigen::Vector3d H_p = normal;

        // Full Jacobian (1×15)
        Eigen::Matrix<double, 1, 15> H;
        H.setZero();
        H.segment<3>(0) = H_theta;  // δθ
        H.segment<3>(3) = H_p;      // δp
        // H.segment<3>(6) = 0 (velocity)
        // H.segment<3>(9) = 0 (accel bias)
        // H.segment<3>(12) = 0 (gyro bias)

        // Add to shared state
        if (shared.num_measurements < shared.H.rows()) {
            shared.H.row(shared.num_measurements) = H;
            shared.b(shared.num_measurements) = residual;
            ++shared.num_measurements;
        }
    }

    shared.valid = (shared.num_measurements > 0);
}

bool LidarProcessor::findPlaneCorrespondence(const Eigen::Vector3d& /*point_lidar*/,
                                             const Eigen::Vector3d& point_world,
                                             Eigen::Vector3d& normal,
                                             Eigen::Vector3d& plane_point) {
    // Find nearest neighbors for plane fitting.
    PointType query;
    query.x = point_world.x();
    query.y = point_world.y();
    query.z = point_world.z();

    PointVec neighbor_points;
    std::vector<float> distances;

    size_t found =
        map_tree_->nearestKSearchPoints(query, config_.near_search_num, neighbor_points, distances);

    if (found < 5)
        return false;

    // Check distance threshold
    if (distances.back() > 1.0)
        return false;

    std::vector<Eigen::Vector3d> neighbors_eigen;
    neighbors_eigen.reserve(neighbor_points.size());
    for (const auto& pt : neighbor_points) {
        neighbors_eigen.emplace_back(pt.x, pt.y, pt.z);
    }

    return estimatePlane(neighbors_eigen, normal, plane_point);
}

bool LidarProcessor::estimatePlane(const std::vector<Eigen::Vector3d>& points,
                                   Eigen::Vector3d& normal, Eigen::Vector3d& plane_point) {
    if (points.size() < 3)
        return false;

    // Compute centroid
    plane_point = Eigen::Vector3d::Zero();
    for (const auto& p : points) {
        plane_point += p;
    }
    plane_point /= points.size();

    // Compute covariance matrix
    Eigen::Matrix3d covariance = Eigen::Matrix3d::Zero();
    for (const auto& p : points) {
        Eigen::Vector3d diff = p - plane_point;
        covariance += diff * diff.transpose();
    }
    covariance /= points.size();

    // Eigen decomposition - smallest eigenvalue gives normal
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver(covariance);
    if (solver.info() != Eigen::Success)
        return false;

    normal = solver.eigenvectors().col(0);  // Smallest eigenvalue

    // Ensure consistent normal direction
    if (normal.dot(plane_point) < 0) {
        normal = -normal;
    }

    // Check planarity: ratio of smallest to largest eigenvalue
    double eigenvalues = solver.eigenvalues()(0);
    if (eigenvalues > 0.01)
        return false;  // Not planar enough

    return true;
}

SE3d LidarProcessor::getLiDARPose() const {
    const auto& state = kf_->getState();
    return state.getLiDARPose();
}

CloudType::Ptr LidarProcessor::transformCloud(const CloudType::Ptr& cloud,
                                              const SE3d& transform) const {
    CloudType::Ptr result(new CloudType);

    for (const auto& pt : cloud->points) {
        Eigen::Vector3d p(pt.x, pt.y, pt.z);
        Eigen::Vector3d p_transformed = transform.rotation().matrix() * p + transform.translation();

        PointType pt_out;
        pt_out.x = p_transformed.x();
        pt_out.y = p_transformed.y();
        pt_out.z = p_transformed.z();
        pt_out.intensity = pt.intensity;
        result->points.push_back(pt_out);
    }

    result->width = result->points.size();
    result->height = 1;
    return result;
}

void LocalMap::update(const Eigen::Vector3d& position, double cube_len, double move_thresh,
                      double det_range) {
    if (!initialized)
        return;

    const double det_thresh = move_thresh * det_range;
    bool need_move = false;
    for (int axis = 0; axis < 3; ++axis) {
        const double dist_to_min = position(axis) - corner.vertex_min[axis];
        const double dist_to_max = corner.vertex_max[axis] - position(axis);
        if (dist_to_min <= det_thresh || dist_to_max <= det_thresh) {
            need_move = true;
            break;
        }
    }
    if (!need_move)
        return;

    const BoxPointType old_corner = corner;
    BoxPointType new_corner{};
    for (int axis = 0; axis < 3; ++axis) {
        new_corner.vertex_min[axis] = static_cast<float>(position(axis) - cube_len / 2.0);
        new_corner.vertex_max[axis] = static_cast<float>(position(axis) + cube_len / 2.0);
    }

    // If the old and new cubes do not overlap, remove the old cube exactly once.
    for (int axis = 0; axis < 3; ++axis) {
        if (new_corner.vertex_min[axis] >= old_corner.vertex_max[axis] ||
            new_corner.vertex_max[axis] <= old_corner.vertex_min[axis]) {
            boxes_to_remove.push_back(old_corner);
            corner = new_corner;
            return;
        }
    }

    // Remove only the slabs that left the sliding cube. Removing the complete
    // old cube here would discard the overlap and repeatedly reset the map.
    for (int axis = 0; axis < 3; ++axis) {
        if (new_corner.vertex_min[axis] > old_corner.vertex_min[axis]) {
            BoxPointType slab = old_corner;
            slab.vertex_max[axis] = std::nextafter(new_corner.vertex_min[axis],
                                                   -std::numeric_limits<float>::infinity());
            boxes_to_remove.push_back(slab);
        } else if (new_corner.vertex_max[axis] < old_corner.vertex_max[axis]) {
            BoxPointType slab = old_corner;
            slab.vertex_min[axis] =
                std::nextafter(new_corner.vertex_max[axis], std::numeric_limits<float>::infinity());
            boxes_to_remove.push_back(slab);
        }
    }
    corner = new_corner;
}

}  // namespace fast_lio
