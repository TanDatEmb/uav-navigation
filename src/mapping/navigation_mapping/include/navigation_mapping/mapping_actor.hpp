#pragma once

#include <chrono>
#include <cstdint>
#include <functional>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>

#include <Eigen/Geometry>

#include <navigation_mapping/mapping_observation.hpp>
#include <navigation_mapping/mapping_diagnostics.hpp>
#include <navigation_mapping/mapping_world_model_adapter.hpp>
#include <navigation_mapping/mapping_world_snapshot.hpp>

namespace navigation_mapping {

struct MappingUpdateResult {
  MapUpdateOutcome outcome{MapUpdateOutcome::EMPTY_CLOUD};
  RaycastDiagnostics diagnostics{};
  std::shared_ptr<const MappingWorldSnapshot> reset_snapshot;
  std::shared_ptr<const MappingWorldSnapshot> snapshot;
  std::uint64_t localization_epoch{0};
  std::uint64_t world_generation{0};
  std::uint64_t world_revision{0};
  std::int64_t observation_stamp_ns{0};
  std::int64_t map_update_us{0};
  std::int64_t snapshot_export_us{0};
};

// Sole mutable map owner and immutable snapshot builder. The caller may
// publish returned snapshots after process() returns; no mutable map handle
// leaves this class.
class MappingActor final {
 public:
  explicit MappingActor(const std::string& config_path,
                        std::function<double()> wall_clock_seconds = {})
      : map_(std::make_shared<RuntimeMappingMap>(std::move(wall_clock_seconds))) {
    map_->loadConfigAndInit(config_path);
  }

  [[nodiscard]] const rog_map::Config& config() const noexcept {
    return map_->getMapConfig();
  }

  [[nodiscard]] std::shared_ptr<const MappingWorldSnapshot> initialSnapshot() const {
    return std::make_shared<MappingWorldSnapshot>(
        map_->exportPlanningGrid(),
        navigation_world_model::WorldSnapshotIdentity{localization_epoch_, world_generation_,
                                                       0U, 0});
  }

  MappingUpdateResult process(const MappingObservation& observation) {
    if (!observation.cloud || observation.cloud->empty() || observation.stamp_ns <= 0 ||
        observation.localization_epoch == 0) {
      throw std::invalid_argument("mapping observation is malformed");
    }
    if (observation.localization_epoch < localization_epoch_) {
      throw std::runtime_error("mapping observation belongs to an older localization epoch");
    }

    MappingUpdateResult result;
    if (observation.localization_epoch > localization_epoch_) {
      map_->init();
      localization_epoch_ = observation.localization_epoch;
      ++world_generation_;
      world_revision_ = 0;
      result.reset_snapshot = std::make_shared<MappingWorldSnapshot>(
          map_->exportPlanningGrid(),
          navigation_world_model::WorldSnapshotIdentity{localization_epoch_, world_generation_,
                                                         0U, 0});
    }

    const auto& pose = observation.corrected_odometry.pose.pose;
    const Eigen::Quaterniond corrected_q{
        pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z};
    if (!corrected_q.coeffs().allFinite() || corrected_q.norm() <= 1.0e-6) {
      throw std::invalid_argument("mapping observation pose is invalid");
    }
    const auto normalized_q = corrected_q.normalized();
    const rog_map::Pose map_pose{
        Eigen::Vector3d{pose.position.x, pose.position.y, pose.position.z},
        Eigen::Quaterniond{normalized_q.w(), normalized_q.x(), normalized_q.y(),
                           normalized_q.z()}};

    const auto map_started = std::chrono::steady_clock::now();
    result.outcome = map_->updateMap(*observation.cloud, map_pose);
    result.diagnostics = map_->lastDiagnostics();
    if (!worldUpdateAdvanced(result.outcome)) {
      result.map_update_us = std::chrono::duration_cast<std::chrono::microseconds>(
          std::chrono::steady_clock::now() - map_started).count();
      throw std::runtime_error(
          std::string("mapping observation did not advance the immutable world: ") +
          std::string(worldUpdateOutcomeName(result.outcome)));
    }

    const auto export_started = std::chrono::steady_clock::now();
    ++world_revision_;
    result.snapshot = std::make_shared<MappingWorldSnapshot>(
        map_->exportPlanningGrid(),
        navigation_world_model::WorldSnapshotIdentity{
            localization_epoch_, world_generation_, world_revision_, observation.stamp_ns});
    result.snapshot_export_us = std::chrono::duration_cast<std::chrono::microseconds>(
        std::chrono::steady_clock::now() - export_started).count();
    result.map_update_us = std::chrono::duration_cast<std::chrono::microseconds>(
        std::chrono::steady_clock::now() - map_started).count();
    result.localization_epoch = localization_epoch_;
    result.world_generation = world_generation_;
    result.world_revision = world_revision_;
    result.observation_stamp_ns = observation.stamp_ns;
    return result;
  }

 private:
  std::shared_ptr<RuntimeMappingMap> map_;
  std::uint64_t localization_epoch_{1};
  std::uint64_t world_generation_{1};
  std::uint64_t world_revision_{0};
};

}  // namespace navigation_mapping
