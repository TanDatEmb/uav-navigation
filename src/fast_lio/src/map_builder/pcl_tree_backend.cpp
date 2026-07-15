#include "fast_lio/pcl_tree_backend.hpp"

#include <pcl/filters/crop_box.h>
#include <pcl/filters/extract_indices.h>

namespace fast_lio {

PCLTreeBackend::PCLTreeBackend(float downsample_resolution)
    : downsample_res_(downsample_resolution),
      has_range_(false) {
    cloud_.reset(new CloudType);
    voxel_filter_.setLeafSize(downsample_res_, downsample_res_, downsample_res_);
}

size_t PCLTreeBackend::addPoints(const PointVec& points, bool downsample) {
    std::lock_guard<std::mutex> lock(mutex_);

    // Add to cloud
    for (const auto& pt : points) {
        cloud_->points.push_back(pt);
    }

    // Apply downsampling if requested
    if (downsample && !cloud_->points.empty()) {
        CloudType::Ptr temp(new CloudType);
        pcl::copyPointCloud(*cloud_, *temp);

        voxel_filter_.setInputCloud(temp);
        voxel_filter_.filter(*cloud_);
    }

    // Apply range filter
    if (has_range_) {
        applyRangeFilter();
    }

    // Rebuild tree
    rebuildTree();

    return points.size();
}

size_t PCLTreeBackend::deletePoints(const std::vector<BoxPointType>& boxes) {
    std::lock_guard<std::mutex> lock(mutex_);

    if (cloud_->empty()) return 0;

    // Store removed points for compatibility
    removed_points_.clear();

    // Mark points for deletion
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);

    for (size_t i = 0; i < cloud_->points.size(); ++i) {
        const auto& pt = cloud_->points[i];
        bool in_any_box = false;

        for (const auto& box : boxes) {
            if (pt.x >= box.vertex_min[0] && pt.x <= box.vertex_max[0] &&
                pt.y >= box.vertex_min[1] && pt.y <= box.vertex_max[1] &&
                pt.z >= box.vertex_min[2] && pt.z <= box.vertex_max[2]) {
                in_any_box = true;
                removed_points_.push_back(pt);
                break;
            }
        }

        if (!in_any_box) {
            inliers->indices.push_back(static_cast<int>(i));
        }
    }

    // Extract remaining points
    pcl::ExtractIndices<PointType> extract;
    extract.setInputCloud(cloud_);
    extract.setIndices(inliers);
    CloudType::Ptr filtered(new CloudType);
    extract.filter(*filtered);

    cloud_ = filtered;

    // Rebuild tree
    rebuildTree();

    return removed_points_.size();
}

size_t PCLTreeBackend::nearestKSearch(const PointType& query, int k,
                                     std::vector<int>& indices,
                                     std::vector<float>& distances) const {
    std::lock_guard<std::mutex> lock(mutex_);

    if (cloud_->empty() || !kdtree_.getInputCloud()) {
        return 0;
    }

    return kdtree_.nearestKSearch(query, k, indices, distances);
}

size_t PCLTreeBackend::nearestKSearchPoints(const PointType& query, int k,
                                          PointVec& neighbors,
                                          std::vector<float>& distances) const {
    std::lock_guard<std::mutex> lock(mutex_);

    if (cloud_->empty() || !kdtree_.getInputCloud()) {
        neighbors.clear();
        distances.clear();
        return 0;
    }

    std::vector<int> indices;
    kdtree_.nearestKSearch(query, k, indices, distances);

    neighbors.clear();
    neighbors.reserve(indices.size());
    for (int idx : indices) {
        neighbors.push_back(cloud_->points[static_cast<size_t>(idx)]);
    }
    return neighbors.size();
}

void PCLTreeBackend::build(const CloudType::Ptr& cloud) {
    std::lock_guard<std::mutex> lock(mutex_);

    pcl::copyPointCloud(*cloud, *cloud_);
    rebuildTree();
}

void PCLTreeBackend::rebuild() {
    std::lock_guard<std::mutex> lock(mutex_);
    rebuildTree();
}

void PCLTreeBackend::clear() {
    std::lock_guard<std::mutex> lock(mutex_);
    cloud_->clear();
    removed_points_.clear();
    kdtree_.setInputCloud(CloudType::Ptr());
}

size_t PCLTreeBackend::size() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return cloud_->points.size();
}

bool PCLTreeBackend::empty() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return cloud_->points.empty();
}

bool PCLTreeBackend::valid() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return !cloud_->empty() && kdtree_.getInputCloud() != nullptr;
}

void PCLTreeBackend::setDownsampleParam(float resolution) {
    std::lock_guard<std::mutex> lock(mutex_);
    downsample_res_ = resolution;
    voxel_filter_.setLeafSize(downsample_res_, downsample_res_, downsample_res_);
}

void PCLTreeBackend::setLocalMapRange(const BoxPointType& box) {
    std::lock_guard<std::mutex> lock(mutex_);
    local_range_ = box;
    has_range_ = true;
}

void PCLTreeBackend::getAllPoints(PointVec& points) const {
    std::lock_guard<std::mutex> lock(mutex_);
    points.clear();
    points.reserve(cloud_->points.size());
    for (const auto& pt : cloud_->points) {
        points.push_back(pt);
    }
}

void PCLTreeBackend::getRemovedPoints(PointVec& points) {
    std::lock_guard<std::mutex> lock(mutex_);
    points = removed_points_;
}

void PCLTreeBackend::rebuildTree() {
    if (cloud_->empty()) {
        kdtree_.setInputCloud(CloudType::Ptr());
        return;
    }

    cloud_->width = cloud_->points.size();
    cloud_->height = 1;
    cloud_->is_dense = true;

    kdtree_.setInputCloud(cloud_);
}

void PCLTreeBackend::applyRangeFilter() {
    if (!has_range_ || cloud_->empty()) return;

    pcl::CropBox<PointType> crop;
    crop.setMin(Eigen::Vector4f(local_range_.vertex_min[0],
                                   local_range_.vertex_min[1],
                                   local_range_.vertex_min[2], 1.0f));
    crop.setMax(Eigen::Vector4f(local_range_.vertex_max[0],
                                   local_range_.vertex_max[1],
                                   local_range_.vertex_max[2], 1.0f));
    crop.setInputCloud(cloud_);

    CloudType::Ptr filtered(new CloudType);
    crop.filter(*filtered);

    cloud_ = filtered;
}

}  // namespace fast_lio
