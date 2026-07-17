#include "fast_lio/lidar_processor.hpp"
#include "fast_lio/point_plane_measurement.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <limits>

namespace fast_lio {

LidarProcessor::LidarProcessor(const Config& config, std::shared_ptr<Estimator> kf,
                               std::shared_ptr<MapTreeInterface> tree)
    : config_(config), kf_(kf), map_tree_(tree), local_map_{} {
    cloud_down_lidar_.reset(new CloudType);
    cloud_down_world_.reset(new CloudType);
    cloud_effect_lidar_.reset(new CloudType);
    cloud_effect_world_.reset(new CloudType);

    voxel_filter_.setLeafSize(config_.scan_resolution, config_.scan_resolution,
                              config_.scan_resolution);
}

LidarUpdateResult LidarProcessor::process(SyncPackage& package) {
    LidarUpdateResult result;
    result.input_points = package.cloud ? package.cloud->size() : 0;

    // Check empty input
    if (!package.cloud || package.cloud->empty()) {
        result.status = LidarUpdateStatus::kEmptyInput;
        return result;
    }

    auto t_downsample_start = std::chrono::high_resolution_clock::now();

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

    result.downsampled_points = cloud_down_lidar_->size();

    auto t_downsample_end = std::chrono::high_resolution_clock::now();
    result.downsample_time_ms =
        std::chrono::duration<double, std::milli>(t_downsample_end - t_downsample_start).count();

    // Check insufficient downsampled points
    if (cloud_down_lidar_->empty()) {
        result.status = LidarUpdateStatus::kInsufficientDownsampledPoints;
        return result;
    }

    // Initialize the map on first scan
    if (!local_map_.initialized) {
        auto t_map_start = std::chrono::high_resolution_clock::now();
        initMap(cloud_down_lidar_);
        auto t_map_end = std::chrono::high_resolution_clock::now();
        result.map_insertion_time_ms =
            std::chrono::duration<double, std::milli>(t_map_end - t_map_start).count();

        result.status = LidarUpdateStatus::kMapInitialized;
        result.map_inserted = true;
        return result;
    }

    // Update local map based on current position
    const auto& state = kf_->getState();
    Eigen::Vector3d pos_lidar = state.p_w + state.R_wb.matrix() * state.T_I_L.translation();
    local_map_.update(pos_lidar, config_.cube_len, config_.move_thresh, config_.det_range);
    updateLocalMap();

    // Estimator update with point-to-plane constraints
    auto t_correspondence_start = std::chrono::high_resolution_clock::now();

    kf_->setMeasurementCallback(
        [this](const State15& s, SharedState15& sh) { return this->computePointToPlaneConstraint(s, sh); });

    auto t_update_start = std::chrono::high_resolution_clock::now();
    const auto update_result = kf_->update(config_.ieskf_max_iter);
    auto t_update_end = std::chrono::high_resolution_clock::now();

    result.update_time_ms =
        std::chrono::duration<double, std::milli>(t_update_end - t_update_start).count();

    result.estimator_update = update_result;
    result.ieskf_iterations = update_result.iterations;
    result.converged = update_result.converged;

    // Handle estimator status
    if (!update_result.success()) {
        if (update_result.status == EstimatorUpdateStatus::kInsufficientMeasurements) {
            result.status = LidarUpdateStatus::kInsufficientMeasurements;
        } else if (update_result.status == EstimatorUpdateStatus::kNoMeasurements) {
            result.status = LidarUpdateStatus::kNoMeasurements;
        } else {
            result.status = LidarUpdateStatus::kIeskfFailure;
        }
        result.update_applied = false;
        result.map_inserted = false;
        return result;
    }

    // Estimator succeeded - record measurement stats from last shared state
    result.update_applied = true;
    result.accepted_correspondences = update_result.measurements;
    result.queried_points = last_shared_state_.queried_points;
    result.plane_candidates = last_shared_state_.plane_candidates;
    result.residual = last_shared_state_.residual;

    // Map insertion
    auto t_map_insert_start = std::chrono::high_resolution_clock::now();
    incrementMap();
    auto t_map_insert_end = std::chrono::high_resolution_clock::now();
    result.map_insertion_time_ms =
        std::chrono::duration<double, std::milli>(t_map_insert_end - t_map_insert_start).count();

    result.map_inserted = true;
    result.status = LidarUpdateStatus::kSuccess;

    return result;
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

bool LidarProcessor::computePointToPlaneConstraint(const State15& state, SharedState15& shared) {
    // Point-to-plane constraint for IESKF using tested measurement helper
    shared.reset(cloud_down_lidar_->points.size());

    // Track statistics for diagnostics
    std::size_t queried_points = 0;
    std::size_t plane_candidates = 0;
    std::size_t valid_measurements = 0;
    std::size_t residual_rejected = 0;

    double residual_sum = 0.0;
    double residual_abs_sum = 0.0;
    double residual_squared_sum = 0.0;
    double residual_max_abs = 0.0;

    for (size_t i = 0; i < cloud_down_lidar_->points.size(); ++i) {
        const auto& pt = cloud_down_lidar_->points[i];
        Eigen::Vector3d p_lidar(pt.x, pt.y, pt.z);
        ++queried_points;

        // Transform to world for KNN query
        SE3d T_world_lidar = state.getLiDARPose();
        Eigen::Vector3d p_world = T_world_lidar * p_lidar;

        // Find plane correspondence
        Eigen::Vector3d normal, plane_point;
        if (!findPlaneCorrespondence(p_lidar, p_world, normal, plane_point)) {
            continue;
        }
        ++plane_candidates;

        // Evaluate measurement using tested helper
        const auto measurement =
            evaluatePointPlaneMeasurement(state, p_lidar, normal, plane_point);

        // Skip invalid measurements
        if (!measurement.isFinite()) {
            continue;
        }
        ++valid_measurements;

        // Residual gating: skip if residual exceeds threshold
        if (std::abs(measurement.residual) > config_.max_point_plane_residual_m) {
            ++residual_rejected;
            continue;
        }

        // Track residual statistics
        const double r = measurement.residual;
        residual_sum += r;
        residual_abs_sum += std::abs(r);
        residual_squared_sum += r * r;
        residual_max_abs = std::max(residual_max_abs, std::abs(r));

        // Add to shared state
        if (shared.num_measurements < shared.H.rows()) {
            shared.H.row(shared.num_measurements) = measurement.H;
            shared.b(shared.num_measurements) = measurement.residual;
            ++shared.num_measurements;
        }
    }

    // Store statistics
    shared.queried_points = queried_points;
    shared.plane_candidates = plane_candidates;
    shared.valid_measurements = valid_measurements;

    // Check minimum threshold
    if (shared.num_measurements == 0) {
        shared.valid = false;
        shared.validation_status = MeasurementValidationStatus::kNoMeasurements;
    } else if (static_cast<std::size_t>(shared.num_measurements) < config_.min_effective_correspondences) {
        shared.valid = false;
        shared.validation_status = MeasurementValidationStatus::kInsufficientMeasurements;
    } else {
        shared.valid = true;
        shared.validation_status = MeasurementValidationStatus::kValid;
    }

    // Save for diagnostics access after IESKF update
    // Use resize/copy instead of operator= to avoid Eigen alignment issues in tests
    last_shared_state_.reset();
    last_shared_state_.valid = shared.valid;
    last_shared_state_.validation_status = shared.validation_status;
    last_shared_state_.queried_points = shared.queried_points;
    last_shared_state_.plane_candidates = shared.plane_candidates;
    last_shared_state_.valid_measurements = shared.valid_measurements;
    last_shared_state_.num_measurements = shared.num_measurements;
    last_shared_state_.residual = shared.residual;

    // Populate residual statistics
    if (shared.num_measurements > 0) {
        const double n = static_cast<double>(shared.num_measurements);
        last_shared_state_.residual.mean_signed = residual_sum / n;
        last_shared_state_.residual.mean_absolute = residual_abs_sum / n;
        last_shared_state_.residual.rms = std::sqrt(residual_squared_sum / n);
        last_shared_state_.residual.max_absolute = residual_max_abs;
    }

    return shared.valid;
}

bool LidarProcessor::findPlaneCorrespondence(
    const Eigen::Vector3d& /*point_lidar*/,
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

    size_t found = map_tree_->nearestKSearchPoints(
        query, static_cast<int>(config_.knn_search_count), neighbor_points, distances);

    // Check min neighbors (configurable)
    if (found < config_.min_plane_neighbors) {
        return false;
    }

    // Check max neighbor distance (configurable)
    // Convert squared distances to linear for comparison
    double max_dist_sq = config_.max_neighbor_distance_m * config_.max_neighbor_distance_m;
    if (distances.back() > max_dist_sq) {
        return false;
    }

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

    // Eigen decomposition - eigenvalues sorted in ascending order
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver(covariance);
    if (solver.info() != Eigen::Success)
        return false;

    normal = solver.eigenvectors().col(0);  // Smallest eigenvalue

    // Ensure consistent normal direction
    if (normal.dot(plane_point) < 0) {
        normal = -normal;
    }

    // Check planarity using eigenvalue ratios
    // Eigenvalues: lambda_0 <= lambda_1 <= lambda_2
    const double lambda_0 = solver.eigenvalues()(0);
    const double lambda_1 = solver.eigenvalues()(1);
    const double lambda_2 = solver.eigenvalues()(2);

    // Reject if covariance is too small (degenerate)
    if (lambda_2 <= 1e-9) {
        return false;
    }

    // Check planarity: lambda_0 / lambda_2 <= threshold
    // Small ratio = flat plane, large ratio = volumetric
    const double planarity_ratio = lambda_0 / lambda_2;
    if (planarity_ratio > config_.max_plane_eigen_ratio) {
        return false;  // Not planar enough
    }

    // Check linearity: lambda_1 / lambda_2 >= threshold
    // Small ratio = line-like, large ratio = planar
    const double linearity_ratio = lambda_1 / lambda_2;
    if (linearity_ratio < config_.min_second_eigen_ratio) {
        return false;  // Too linear (line-like neighborhood)
    }

    // Check that neighbors are close to the plane
    double max_plane_dist = 0.0;
    for (const auto& p : points) {
        double dist = std::abs(normal.dot(p - plane_point));
        max_plane_dist = std::max(max_plane_dist, dist);
        if (dist > config_.max_neighbor_plane_distance_m) {
            return false;  // Neighbor too far from plane
        }
    }

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
