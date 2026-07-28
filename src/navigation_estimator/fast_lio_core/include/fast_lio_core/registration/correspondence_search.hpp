#pragma once

#include <Eigen/Core>
#include <cstddef>
#include <span>
#include <vector>

#include "fast_lio_core/mapping/registration_map.hpp"
#include "fast_lio_core/registration/correspondence.hpp"

namespace uav::nav::lio {

struct CorrespondenceSearchConfig {
  std::size_t neighbor_count{5};
  double maximum_neighbor_distance_m{2.0};
};

struct CorrespondenceSearchResult {
  std::vector<Correspondence> correspondences;
  std::size_t query_count{0};
  std::size_t insufficient_neighbor_count{0};
};

class CorrespondenceSearch {
 public:
  explicit CorrespondenceSearch(CorrespondenceSearchConfig config = {});

  [[nodiscard]] CorrespondenceSearchResult search(std::span<const Eigen::Vector3d> points_lidar_m,
                                                  std::span<const Eigen::Vector3d> points_odom_m,
                                                  const RegistrationMap& map) const;

 private:
  CorrespondenceSearchConfig config_;
};

}  // namespace uav::nav::lio
