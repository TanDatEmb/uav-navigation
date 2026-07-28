#include "fast_lio_core/registration/correspondence_search.hpp"

#include <cmath>
#include <stdexcept>

namespace uav::nav::lio {

CorrespondenceSearch::CorrespondenceSearch(CorrespondenceSearchConfig config) : config_(config) {
  if (config_.neighbor_count < 3U || !(config_.maximum_neighbor_distance_m > 0.0) ||
      !std::isfinite(config_.maximum_neighbor_distance_m)) {
    throw std::invalid_argument("invalid correspondence search configuration");
  }
}

CorrespondenceSearchResult CorrespondenceSearch::search(
    std::span<const Eigen::Vector3d> points_lidar_m, std::span<const Eigen::Vector3d> points_odom_m,
    const RegistrationMap& map) const {
  CorrespondenceSearchResult result;
  if (points_lidar_m.size() != points_odom_m.size()) {
    return result;
  }
  result.query_count = points_odom_m.size();
  result.correspondences.reserve(points_odom_m.size());
  for (std::size_t index = 0; index < points_odom_m.size(); ++index) {
    if (!points_lidar_m[index].allFinite() || !points_odom_m[index].allFinite()) {
      ++result.insufficient_neighbor_count;
      continue;
    }
    NearestNeighborResult neighbors = map.nearestNeighbors(
        points_odom_m[index], config_.neighbor_count, config_.maximum_neighbor_distance_m);
    if (!neighbors.complete(config_.neighbor_count)) {
      ++result.insufficient_neighbor_count;
      continue;
    }
    Correspondence correspondence;
    correspondence.point_index = index;
    correspondence.point_lidar_m = points_lidar_m[index];
    correspondence.point_odom_m = points_odom_m[index];
    correspondence.neighbors_odom_m = std::move(neighbors.points_odom_m);
    result.correspondences.push_back(std::move(correspondence));
  }
  return result;
}

}  // namespace uav::nav::lio
