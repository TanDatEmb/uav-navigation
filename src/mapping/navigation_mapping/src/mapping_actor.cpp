#include <navigation_mapping/mapping_actor.hpp>
#include <navigation_mapping/mapping_world_snapshot.hpp>
#include "mapping_world_model_adapter.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <limits>
#include <optional>
#include <stdexcept>
#include <utility>
#include <vector>

#include <Eigen/Geometry>
#include <navigation_common/time.hpp>
#include <rog_map/rog_map.h>

namespace navigation_mapping {
namespace {

[[nodiscard]] bool finiteNonzeroQuaternion(const Eigen::Quaterniond& quaternion) {
  const double scale = quaternion.coeffs().cwiseAbs().maxCoeff();
  return quaternion.coeffs().allFinite() && std::isfinite(scale) && scale > 1.0e-6;
}

[[nodiscard]] MapUpdateOutcome toProductOutcome(
    const rog_map::MapUpdateOutcome outcome) {
  switch (outcome) {
    case rog_map::MapUpdateOutcome::UPDATED: return MapUpdateOutcome::kUpdated;
    case rog_map::MapUpdateOutcome::ACCUMULATED: return MapUpdateOutcome::kAccumulated;
    case rog_map::MapUpdateOutcome::SLIDE_ONLY: return MapUpdateOutcome::kSlideOnly;
    case rog_map::MapUpdateOutcome::EMPTY_CLOUD: return MapUpdateOutcome::kEmptyCloud;
    case rog_map::MapUpdateOutcome::CALLBACK_OWNED: return MapUpdateOutcome::kCallbackOwned;
    case rog_map::MapUpdateOutcome::BELOW_GROUND: return MapUpdateOutcome::kBelowGround;
    case rog_map::MapUpdateOutcome::ABOVE_CEILING: return MapUpdateOutcome::kAboveCeiling;
  }
  throw std::logic_error("backend returned an unknown map update outcome");
}

[[nodiscard]] RaycastDiagnostics toProductDiagnostics(
  const rog_map::ProbMap::RaycastDiagnostics& source) {
  RaycastDiagnostics result;
  result.update_outcome = toProductOutcome(source.update_outcome);
  result.endpoint_count = source.endpoint_count;
  result.free_space_endpoint_count = source.free_space_endpoint_count;
  result.attempt_count = source.attempt_count;
  result.processed_count = source.processed_count;
  result.free_space_attempt_count = source.free_space_attempt_count;
  result.free_space_processed_count = source.free_space_processed_count;
  result.free_space_clipped_count = source.free_space_clipped_count;
  result.free_space_skipped_count = source.free_space_skipped_count;
  result.clipped_count = source.clipped_count;
  result.skipped_count = source.skipped_count;
  result.skip_nonfinite = source.skip_nonfinite;
  result.skip_intensity = source.skip_intensity;
  result.skip_point_filter = source.skip_point_filter;
  result.skip_below_raycast_min_range = source.skip_below_raycast_min_range;
  result.skip_endpoint_outside_local_map = source.skip_endpoint_outside_local_map;
  result.clipped_virtual_ground_or_ceiling = source.clipped_virtual_ground_or_ceiling;
  result.clipped_raycast_max_range = source.clipped_raycast_max_range;
  result.clipped_local_update_box = source.clipped_local_update_box;
  result.ray_outside_local_map_step = source.ray_outside_local_map_step;
  result.voxel_traversal_count_total = source.voxel_traversal_count_total;
  result.voxel_traversal_count_max = source.voxel_traversal_count_max;
  result.hit_candidate_count = source.hit_candidate_count;
  result.miss_candidate_count = source.miss_candidate_count;
  result.unique_hit_voxel_count = source.unique_hit_voxel_count;
  result.unique_miss_voxel_count = source.unique_miss_voxel_count;
  result.update_cache_entry_count = source.update_cache_entry_count;
  result.unique_update_cache_voxel_count = source.unique_update_cache_voxel_count;
  result.map_slide_check_count = source.map_slide_check_count;
  result.map_slide_count = source.map_slide_count;
  result.map_slide_voxel_shift_x = source.map_slide_voxel_shift_x;
  result.map_slide_voxel_shift_y = source.map_slide_voxel_shift_y;
  result.map_slide_voxel_shift_z = source.map_slide_voxel_shift_z;
  result.map_slide_cells_cleared = source.map_slide_cells_cleared;
  result.body_neighborhood_cells_cleared = source.body_neighborhood_cells_cleared;
  result.inflation_update_count = source.inflation_update_count;
  result.map_update_us = source.rog_total_update_us;
  result.raycast_us = source.rog_raycast_us;
  result.probability_update_us = source.rog_probability_update_us;
  result.inflation_us = source.rog_inflation_us;
  result.slide_us = source.rog_slide_us;
  result.allocated_voxel_count = source.allocated_voxel_count;
  result.changed_region_min = source.changed_region_min.cast<double>();
  result.changed_region_max = source.changed_region_max.cast<double>();
  result.changed_region_covers_world = source.changed_region_covers_world;
  return result;
}

[[nodiscard]] PlanningGridLayout toProductLayout(
    const rog_map::PlanningGridLayoutExport& source) {
  return {
      source.resolution_m,
      source.global_min_index,
      source.dimensions,
      source.local_center_m.cast<double>(),
      source.local_size_m.cast<double>(),
  };
}

[[nodiscard]] std::shared_ptr<const std::vector<navigation_world_model::GridIndex3>>
toProductNearestOffsets(
    const std::shared_ptr<const std::vector<navigation_math::Vec3i>>& source) {
  if (!source) return {};
  auto offsets = std::make_shared<std::vector<navigation_world_model::GridIndex3>>();
  offsets->reserve(source->size());
  for (const auto& offset : *source) {
    offsets->emplace_back(offset.x(), offset.y(), offset.z());
  }
  return offsets;
}

[[nodiscard]] PlanningGrid toProductGrid(
    rog_map::PlanningGridExport source,
    const std::shared_ptr<const std::vector<navigation_world_model::GridIndex3>>&
        nearest_offsets) {
  PlanningGrid result;
  result.base_layout = toProductLayout(source.base_layout);
  result.inflated.layout = toProductLayout(source.inflated.layout);
  result.base_state = std::move(source.base_state);
  result.inflated.occupied = std::move(source.inflated.occupied);
  result.inflated.unknown = std::move(source.inflated.unknown);
  result.nearest_offsets = nearest_offsets;
  result.unknown_inflation_enabled = source.unknown_inflation_enabled;
  result.virtual_ground_ceiling_enabled = source.virtual_ground_ceiling_enabled;
  result.virtual_ground_m = source.virtual_ground_m;
  result.virtual_ceiling_m = source.virtual_ceiling_m;
  result.inflated_virtual_ground_m = source.inflated_virtual_ground_m;
  result.inflated_virtual_ceiling_m = source.inflated_virtual_ceiling_m;
  result.occupied_inflation_radius_m = source.occupied_inflation_radius_m;
  return result;
}

[[nodiscard]] PlanningGridPatch toProductPatch(
    rog_map::PlanningGridPatchExport source) {
  PlanningGridPatch result;
  result.base_layout = toProductLayout(source.base_layout);
  result.inflated.layout = toProductLayout(source.inflated.layout);
  result.base_state = std::move(source.base_state);
  result.inflated.occupied = std::move(source.inflated.occupied);
  result.inflated.unknown = std::move(source.inflated.unknown);
  return result;
}

void fillBackendCloud(const PointCloud& source, rog_map::PointCloud& result) {
  result.clear();
  result.reserve(source.size());
  for (const auto& point : source) {
    if (!std::isfinite(point.x) || !std::isfinite(point.y) ||
        !std::isfinite(point.z) || !std::isfinite(point.intensity)) {
      throw std::invalid_argument("mapping point cloud contains a non-finite point");
    }
    rog_map::PclPoint backend_point;
    backend_point.x = point.x;
    backend_point.y = point.y;
    backend_point.z = point.z;
    backend_point.intensity = point.intensity;
    result.push_back(backend_point);
  }
}

}  // namespace

namespace {

// The change window is an optimization for stale-certificate recertification,
// not safety evidence by itself. Retain a bounded newest-first window so a
// long-running mapper cannot accumulate unbounded provenance. If a candidate
// is older than this window, MappingWorldSnapshot::changedRegionIntersectsSince()
// sees the missing revision and fails closed.
constexpr std::size_t kWorldChangeHistoryRetention = 256U;
constexpr std::size_t kMaximumSnapshotPatchDepth = 8U;
constexpr double kMaximumPatchToFullOwnedByteRatio = 0.40;

[[nodiscard]] bool includeTraversedSegmentRegion(
    navigation_world_model::AxisAlignedBox& region,
    const TraversedFreeSegment& segment,
    const double query_tolerance_m) noexcept {
  if (!segment.start.allFinite() || !segment.end.allFinite() ||
      !std::isfinite(segment.support_radius_m) || segment.support_radius_m < 0.0 ||
      !std::isfinite(query_tolerance_m) || query_tolerance_m < 0.0) {
    return false;
  }
  const double expansion = segment.support_radius_m + query_tolerance_m;
  const auto minimum = segment.start.cwiseMin(segment.end).array() - expansion;
  const auto maximum = segment.start.cwiseMax(segment.end).array() + expansion;
  if (!minimum.allFinite() || !maximum.allFinite()) return false;
  if (!region.valid()) {
    region.minimum = minimum.matrix();
    region.maximum = maximum.matrix();
  } else {
    region.minimum = region.minimum.cwiseMin(minimum.matrix());
    region.maximum = region.maximum.cwiseMax(maximum.matrix());
  }
  return region.valid();
}

[[nodiscard]] std::optional<navigation_world_model::AxisAlignedBox>
traversedAffectedRegion(const TraversedFreeSpacePtr& previous,
                        const TraversedFreeSpacePtr& next,
                        const double inflated_resolution_m) noexcept {
  if (!std::isfinite(inflated_resolution_m) || inflated_resolution_m <= 0.0) {
    return std::nullopt;
  }
  navigation_world_model::AxisAlignedBox region;
  const double query_tolerance_m = 0.5 * inflated_resolution_m;
  const auto include = [&](const TraversedFreeSpacePtr& overlay) {
    if (!overlay) return true;
    for (const auto& segment : overlay->segments()) {
      if (!includeTraversedSegmentRegion(region, segment, query_tolerance_m)) {
        return false;
      }
    }
    return true;
  };
  if (!include(previous) || !include(next)) return std::nullopt;
  return region.valid()
      ? std::optional<navigation_world_model::AxisAlignedBox>{region}
      : std::nullopt;
}

}  // namespace

class MappingActor::Impl final {
 public:
  Impl(const std::string& config_path, std::function<double()> wall_clock_seconds,
       MappingFrameContract frame_contract, const double snapshot_publication_period_s)
      : config_path_(config_path),
        wall_clock_seconds_(std::move(wall_clock_seconds)),
        map_(std::make_unique<internal::RuntimeMappingMap>(wall_clock_seconds_)),
        snapshot_publication_period_s_(snapshot_publication_period_s) {
    const auto period_ns = navigation_common::secondsToNanoseconds(
        snapshot_publication_period_s_);
    if (!period_ns || *period_ns <= 0) {
      throw std::invalid_argument(
          "mapping snapshot publication period must be positive and finite");
    }
    snapshot_publication_period_ns_ = *period_ns;
    map_->loadConfigAndInit(config_path_);
    expected_world_frame_id_ = std::move(frame_contract.world_frame_id);
    expected_body_frame_id_ = std::move(frame_contract.body_frame_id);
  }

  [[nodiscard]] MappingConfiguration configuration() const noexcept {
    const auto& config = map_->getMapConfig();
    return {config.ros_callback_en, config.batch_update_size,
            config.raycasting_en, true, config.virtual_ground_ceiling_en,
            config.traversed_free_max_age_s,
            config.traversed_free_support_radius_m};
  }

  [[nodiscard]] MappingSnapshotPublication initialSnapshot() {
    auto exported = map_->exportPlanningGrid();
    full_snapshot_owned_bytes_ = exported.ownedByteSize();
    nearest_offsets_ = toProductNearestOffsets(exported.nearest_offsets);
    auto snapshot = std::make_shared<MappingWorldSnapshot>(
        toProductGrid(std::move(exported), nearest_offsets_),
        navigation_world_model::WorldSnapshotIdentity{localization_epoch_, world_generation_,
                                                       0U, 0},
        change_history_);
    current_snapshot_ = snapshot;
    pending_changed_region_ = {};
    pending_change_covers_world_ = false;
    return {snapshot, snapshot->metrics()};
  }

  MappingUpdateResult process(const MappingObservation& observation) {
    if (poisoned_) {
      throw std::logic_error("mapping actor is poisoned after a failed map transition");
    }
    validateObservation(observation);
    if (observation.localization_epoch < localization_epoch_) {
      throw std::runtime_error("mapping observation belongs to an older localization epoch");
    }

    const auto& pose = observation.corrected_odometry.pose.pose;
    const Eigen::Quaterniond corrected_q{
        pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z};
    if (!finiteNonzeroQuaternion(corrected_q)) {
      throw std::invalid_argument("mapping observation orientation is invalid");
    }
    const double quaternion_scale = corrected_q.coeffs().cwiseAbs().maxCoeff();
    const Eigen::Quaterniond normalized_q = Eigen::Quaterniond(
        corrected_q.w() / quaternion_scale, corrected_q.x() / quaternion_scale,
        corrected_q.y() / quaternion_scale, corrected_q.z() / quaternion_scale).normalized();
    const rog_map::Pose map_pose{
        Eigen::Vector3d{pose.position.x, pose.position.y, pose.position.z},
        Eigen::Quaterniond{normalized_q.w(), normalized_q.x(), normalized_q.y(),
                           normalized_q.z()}};

    const auto previous_traversed_free_space = traversed_free_space_;
    const auto previous_localization_epoch = localization_epoch_;
    try {
      MappingUpdateResult result;
      if (observation.localization_epoch > localization_epoch_) {
        if (world_generation_ == std::numeric_limits<std::uint64_t>::max()) {
          throw std::runtime_error("mapping world generation exhausted");
        }
        // ROGMap::init() is intentionally one-shot for each instance. Build
        // and initialize a replacement before swapping it in so a public
        // localization-frame transition cannot double-init the live map. If
        // construction fails, the old map remains untouched until the actor's
        // existing fail-stop path poisons it.
        auto replacement = std::make_unique<internal::RuntimeMappingMap>(
            wall_clock_seconds_);
        replacement->loadConfigAndInit(config_path_);
        map_ = std::move(replacement);
        localization_epoch_ = observation.localization_epoch;
        ++world_generation_;
        world_revision_ = 0;
        last_observation_stamp_ns_ = 0;
        last_scan_sequence_ = 0;
        change_history_.reset();
        current_snapshot_.reset();
        pending_changed_region_ = {};
        pending_change_covers_world_ = false;
        traversed_free_space_.reset();
        last_traversed_pose_.reset();
        last_traversed_stamp_ns_ = 0;
      }

      if (world_revision_ == std::numeric_limits<std::uint64_t>::max()) {
        throw std::runtime_error("mapping world revision exhausted");
      }

      const auto map_started = std::chrono::steady_clock::now();
      result.diagnostics = {};
      fillBackendCloud(*observation.cloud, backend_cloud_);
      backend_free_space_cloud_.clear();
      if (observation.free_space_endpoints) {
        fillBackendCloud(*observation.free_space_endpoints,
                         backend_free_space_cloud_);
      }
      const auto sensor_origin = *observation.sensor_origin_world;
      const auto backend_outcome = map_->updateMap(
          backend_cloud_, backend_free_space_cloud_, map_pose,
          sensor_origin);
      result.outcome = toProductOutcome(backend_outcome);
      result.diagnostics = toProductDiagnostics(map_->lastDiagnostics());
      if (!worldUpdateAdvanced(result.outcome)) {
        result.map_update_us = std::chrono::duration_cast<std::chrono::microseconds>(
            std::chrono::steady_clock::now() - map_started).count();
        throw std::runtime_error(
            std::string("mapping observation did not advance the immutable world: ") +
            std::string(worldUpdateOutcomeName(result.outcome)));
      }

      const auto next_traversed = updateTraversedFreeSpace(
          observation, navigation_world_model::Point3{pose.position.x, pose.position.y,
                                                       pose.position.z});
      traversed_free_space_ = next_traversed;
      const auto& map_config = map_->getMapConfig();
      const bool epoch_reset = observation.localization_epoch !=
          previous_localization_epoch;
      const auto traversed_region = traversedAffectedRegion(
          previous_traversed_free_space, next_traversed,
          map_config.inflation_resolution);
      if (epoch_reset ||
          ((previous_traversed_free_space || next_traversed) &&
           !traversed_region.has_value())) {
        result.diagnostics.changed_region_covers_world = true;
      } else if (traversed_region.has_value()) {
        if (!result.diagnostics.changed_region_min.allFinite() ||
            !result.diagnostics.changed_region_max.allFinite()) {
          result.diagnostics.changed_region_covers_world = true;
        } else {
          result.diagnostics.changed_region_min =
              result.diagnostics.changed_region_min.cwiseMin(
                  traversed_region->minimum);
          result.diagnostics.changed_region_max =
              result.diagnostics.changed_region_max.cwiseMax(
                  traversed_region->maximum);
        }
      }
      result.diagnostics.base_pose_world =
          navigation_world_model::Point3{pose.position.x, pose.position.y, pose.position.z};
      result.diagnostics.sensor_origin_world = sensor_origin;
      result.diagnostics.traversed_segment_count = next_traversed
          ? next_traversed->segments().size() : 0U;
      result.diagnostics.traversed_latest_stamp_ns = observation.stamp_ns;
      result.diagnostics.traversed_latest_age_s = 0.0;
      result.diagnostics.traversed_chain_reset_reason =
          static_cast<std::uint8_t>(traversed_reset_reason_);

      const auto export_started = std::chrono::steady_clock::now();
      ++world_revision_;
      const auto identity = navigation_world_model::WorldSnapshotIdentity{
          localization_epoch_, world_generation_, world_revision_, observation.stamp_ns};
      navigation_world_model::WorldChangeRecord change;
      change.identity = identity;
      change.affects_whole_world = result.diagnostics.changed_region_covers_world;
      if (!change.affects_whole_world) {
        const double inflation_radius = map_config.inflation_resolution * static_cast<double>(
            std::max(map_config.inflation_step, map_config.unk_inflation_step));
        // The backend reports metric endpoints, while map updates are made at
        // quantized evidence and inflated voxel centers. Include both half
        // voxel shells before recording a disjointness certificate, then step
        // the bounds outward so floating-point rounding cannot shrink it.
        const double quantization_shell = 0.5 *
            (map_config.resolution + map_config.inflation_resolution);
        const double change_margin = inflation_radius + quantization_shell;
        if (!std::isfinite(change_margin) || change_margin < 0.0 ||
            !result.diagnostics.changed_region_min.allFinite() ||
            !result.diagnostics.changed_region_max.allFinite()) {
          change.affects_whole_world = true;
        } else {
          for (int axis = 0; axis < 3; ++axis) {
            const double minimum = result.diagnostics.changed_region_min[axis] - change_margin;
            const double maximum = result.diagnostics.changed_region_max[axis] + change_margin;
            if (!std::isfinite(minimum) || !std::isfinite(maximum)) {
              change.affects_whole_world = true;
              break;
            }
            change.affected_region.minimum[axis] =
                std::nextafter(minimum, -std::numeric_limits<double>::infinity());
            change.affected_region.maximum[axis] =
                std::nextafter(maximum, std::numeric_limits<double>::infinity());
          }
        }
        if (!change.affected_region.valid()) change.affects_whole_world = true;
      }
      std::vector<navigation_world_model::WorldChangeRecord> records;
      // The new record is part of the retention bound. Keep at most N-1 old
      // records before prepending it, so the immutable window never grows to
      // N+1 at rollover.
      const std::size_t previous_record_limit = kWorldChangeHistoryRetention > 0U
          ? kWorldChangeHistoryRetention - 1U : 0U;
      const std::size_t retained_count = change_history_
          ? std::min(previous_record_limit, change_history_->records.size())
          : 0U;
      records.reserve(retained_count + 1U);
      records.push_back(change);
      if (change_history_ && retained_count != 0U) {
        records.insert(records.end(), change_history_->records.begin(),
                       change_history_->records.begin() +
                           static_cast<std::ptrdiff_t>(retained_count));
      }
      change_history_ = std::make_shared<navigation_world_model::WorldChangeHistory>(
          navigation_world_model::WorldChangeHistory{std::move(records)});

      if (change.affects_whole_world) {
        pending_change_covers_world_ = true;
      } else if (change.affected_region.valid()) {
        if (!pending_changed_region_.valid()) {
          pending_changed_region_ = change.affected_region;
        } else {
          pending_changed_region_.minimum =
              pending_changed_region_.minimum.cwiseMin(change.affected_region.minimum);
          pending_changed_region_.maximum =
              pending_changed_region_.maximum.cwiseMax(change.affected_region.maximum);
        }
      } else {
        pending_change_covers_world_ = true;
      }

      const auto snapshot_age = current_snapshot_
          ? navigation_common::checkedDifference(
                observation.stamp_ns,
                current_snapshot_->identity().observation_stamp_ns)
          : std::optional<std::int64_t>{};
      if (current_snapshot_ && !snapshot_age.has_value()) {
        throw std::runtime_error("mapping snapshot timestamp arithmetic overflow");
      }
      const bool publication_due = !current_snapshot_ || pending_change_covers_world_ ||
          *snapshot_age >= snapshot_publication_period_ns_;
      if (!publication_due) {
        result.map_update_us = std::chrono::duration_cast<std::chrono::microseconds>(
            std::chrono::steady_clock::now() - map_started).count();
        result.localization_epoch = localization_epoch_;
        result.world_generation = world_generation_;
        result.world_revision = world_revision_;
        result.observation_stamp_ns = observation.stamp_ns;
        last_observation_stamp_ns_ = observation.stamp_ns;
        if (observation.scan_sequence != 0U) {
          last_scan_sequence_ = observation.scan_sequence;
        }
        return result;
      }

      std::shared_ptr<const MappingWorldSnapshot> snapshot;
      const bool can_apply_patch = current_snapshot_ && !pending_change_covers_world_ &&
          pending_changed_region_.valid() &&
          current_snapshot_->patchDepth() < kMaximumSnapshotPatchDepth;
      bool patch_is_efficient = false;
      if (can_apply_patch && full_snapshot_owned_bytes_ > 0U) {
        const auto estimate = map_->estimatePlanningGridRegionSize(
            pending_changed_region_.minimum, pending_changed_region_.maximum);
        patch_is_efficient = estimate.valid &&
            static_cast<double>(estimate.owned_byte_count) <
                kMaximumPatchToFullOwnedByteRatio *
                    static_cast<double>(full_snapshot_owned_bytes_);
        if (!patch_is_efficient) {
          result.snapshot_full_export_reason =
              SnapshotFullExportReason::kPatchTooLarge;
        }
      }
      if (can_apply_patch && patch_is_efficient) {
        const auto patch_export = map_->exportPlanningGridRegion(
            pending_changed_region_.minimum,
            pending_changed_region_.maximum);
        result.snapshot_export_base_cells = patch_export.base_state.size();
        result.snapshot_export_inflated_cells = patch_export.inflated.occupied.size();
        if (!patch_export.base_state.empty() && !patch_export.inflated.occupied.empty()) {
          snapshot = std::make_shared<MappingWorldSnapshot>(
              current_snapshot_, toProductPatch(patch_export), identity, change_history_,
              traversed_free_space_);
          result.snapshot_export_mode = SnapshotExportMode::kPatch;
        } else {
          result.snapshot_full_export_reason = SnapshotFullExportReason::kEmptyPatch;
        }
      }
      if (!snapshot) {
        if (result.snapshot_full_export_reason == SnapshotFullExportReason::kNone) {
          result.snapshot_full_export_reason = !current_snapshot_
              ? SnapshotFullExportReason::kNoCurrentSnapshot
              : pending_change_covers_world_
                  ? SnapshotFullExportReason::kWholeWorldChanged
                  : !pending_changed_region_.valid()
                      ? SnapshotFullExportReason::kInvalidChangedRegion
                      : SnapshotFullExportReason::kPatchDepthLimit;
        }
        auto exported = map_->exportPlanningGrid();
        full_snapshot_owned_bytes_ = exported.ownedByteSize();
        result.snapshot_export_base_cells = exported.base_state.size();
        result.snapshot_export_inflated_cells = exported.inflated.occupied.size();
        if (!nearest_offsets_) {
          nearest_offsets_ = toProductNearestOffsets(exported.nearest_offsets);
        }
        snapshot = std::make_shared<MappingWorldSnapshot>(
            toProductGrid(std::move(exported), nearest_offsets_), identity, change_history_,
            traversed_free_space_);
        result.snapshot_export_mode = SnapshotExportMode::kFull;
      }
      result.snapshot = snapshot;
      result.snapshot_metrics = snapshot->metrics();
      result.snapshot_patch_depth = snapshot->patchDepth();
      result.snapshot_export_us = std::chrono::duration_cast<std::chrono::microseconds>(
          std::chrono::steady_clock::now() - export_started).count();
      result.map_update_us = std::chrono::duration_cast<std::chrono::microseconds>(
          std::chrono::steady_clock::now() - map_started).count();
      result.localization_epoch = localization_epoch_;
      result.world_generation = world_generation_;
      result.world_revision = world_revision_;
      result.observation_stamp_ns = observation.stamp_ns;
      last_observation_stamp_ns_ = observation.stamp_ns;
      if (observation.scan_sequence != 0U) {
        last_scan_sequence_ = observation.scan_sequence;
      }
      current_snapshot_ = snapshot;
      pending_changed_region_ = {};
      pending_change_covers_world_ = false;
      return result;
    } catch (...) {
      // The backend is mutable and has no transaction/rollback API. Any
      // exception after admission may have changed part of its state; poison
      // this actor so no partially committed map can be used as evidence.
      poisoned_ = true;
      throw;
    }
  }

 private:
  void validateObservation(const MappingObservation& observation) const {
    if (!observation.cloud || observation.cloud->empty() || observation.stamp_ns <= 0 ||
        observation.localization_epoch == 0U) {
      throw std::invalid_argument("mapping observation is malformed");
    }
    const auto odometry_stamp_ns = navigation_common::rosTimeToNanoseconds(
        observation.corrected_odometry.header.stamp);
    if (!odometry_stamp_ns || *odometry_stamp_ns != observation.stamp_ns) {
      throw std::invalid_argument(
          "mapping observation and corrected odometry timestamps differ");
    }
    if (observation.corrected_odometry.header.frame_id.empty() ||
        observation.corrected_odometry.child_frame_id.empty() ||
        (!expected_world_frame_id_.empty() &&
         observation.corrected_odometry.header.frame_id != expected_world_frame_id_) ||
        (!expected_body_frame_id_.empty() &&
         observation.corrected_odometry.child_frame_id != expected_body_frame_id_)) {
      throw std::invalid_argument("mapping observation frame contract is invalid");
    }
    const auto& pose = observation.corrected_odometry.pose.pose;
    if (!std::isfinite(pose.position.x) || !std::isfinite(pose.position.y) ||
        !std::isfinite(pose.position.z)) {
      throw std::invalid_argument("mapping observation position is invalid");
    }
    const Eigen::Quaterniond quaternion{
        pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z};
    if (!finiteNonzeroQuaternion(quaternion)) {
      throw std::invalid_argument("mapping observation pose is invalid");
    }
    for (const auto& point : *observation.cloud) {
      if (!std::isfinite(point.x) || !std::isfinite(point.y) ||
          !std::isfinite(point.z) || !std::isfinite(point.intensity)) {
        throw std::invalid_argument("mapping point cloud contains a non-finite point");
      }
    }
    if (observation.free_space_endpoints) {
      for (const auto& point : *observation.free_space_endpoints) {
        if (!std::isfinite(point.x) || !std::isfinite(point.y) ||
            !std::isfinite(point.z) || !std::isfinite(point.intensity)) {
          throw std::invalid_argument(
              "mapping free-space endpoint cloud contains a non-finite point");
        }
      }
    }
    if (observation.sensor_origin_world &&
        !observation.sensor_origin_world->allFinite()) {
      throw std::invalid_argument("mapping sensor origin is invalid");
    }
    // Even endpoint-only mode uses the origin for the configured sensor-range
    // contract. There is no safe implicit base-link substitute in either
    // mode; a non-raycasting deployment must provide the same explicit
    // sensor-origin field.
    if (!observation.sensor_origin_world.has_value()) {
      throw MappingObservationRejected(
          MappingObservationRejectionReason::kMissingSensorOrigin);
    }
    if (observation.sensor_origin_world.has_value() &&
        (observation.sensor_origin_localization_epoch != observation.localization_epoch ||
         observation.sensor_origin_stamp_ns != observation.stamp_ns)) {
      throw MappingObservationRejected(
          MappingObservationRejectionReason::kSensorOriginContractMismatch);
    }
    if (observation.localization_epoch == localization_epoch_) {
      if (observation.stamp_ns <= last_observation_stamp_ns_) {
        throw std::runtime_error("mapping observation timestamp is not increasing");
      }
      if (observation.scan_sequence != 0U && last_scan_sequence_ != 0U &&
          observation.scan_sequence <= last_scan_sequence_) {
        throw std::runtime_error("mapping observation sequence is not increasing");
      }
    }
  }

  [[nodiscard]] TraversedFreeSpacePtr updateTraversedFreeSpace(
      const MappingObservation& observation,
      const navigation_world_model::Point3& base_pose) {
    const auto& config = map_->getMapConfig();
    std::vector<TraversedFreeSegment> segments = traversed_free_space_
        ? traversed_free_space_->segments() : std::vector<TraversedFreeSegment>{};
    // Keep bounded stale segment metadata for typed handover diagnostics.  It
    // is never returned as traversed-free evidence: TraversedFreeSpace::contains
    // applies max_age_s at query time.  The bounded ring below prevents this
    // diagnostic history from becoming an unbounded map or free-space source.
    traversed_reset_reason_ = TraversedChainResetReason::kNone;
    bool append_sweep = false;
    if (!last_traversed_pose_) {
      traversed_reset_reason_ = TraversedChainResetReason::kFirstPose;
    } else if (observation.localization_epoch != last_traversed_epoch_) {
      segments.clear();
      traversed_reset_reason_ = TraversedChainResetReason::kEpochChanged;
    } else {
      const auto delta_ns = observation.stamp_ns - last_traversed_stamp_ns_;
      const double dt_s = static_cast<double>(delta_ns) * 1.0e-9;
      const double distance_m = (base_pose - *last_traversed_pose_).norm();
      const bool monotonic = delta_ns > 0;
      const bool gap_valid = std::isfinite(dt_s) && dt_s <= config.traversed_free_max_pose_gap_s;
      const bool plausible = std::isfinite(distance_m) &&
          distance_m <= config.traversed_free_max_speed_mps * std::max(0.0, dt_s) +
              1.0e-6;
      if (monotonic && gap_valid && plausible) {
        append_sweep = true;
      } else if (!monotonic) {
        traversed_reset_reason_ = TraversedChainResetReason::kTimestampRegression;
      } else if (!gap_valid) {
        traversed_reset_reason_ = TraversedChainResetReason::kSourceGap;
      } else {
        traversed_reset_reason_ = TraversedChainResetReason::kImplausibleMotion;
      }
    }
    if (append_sweep) {
      segments.push_back(TraversedFreeSegment{
          *last_traversed_pose_, base_pose, observation.localization_epoch,
          last_traversed_stamp_ns_, observation.stamp_ns,
          config.traversed_free_support_radius_m});
    }
    segments.push_back(TraversedFreeSegment{
        base_pose, base_pose, observation.localization_epoch,
        observation.stamp_ns, observation.stamp_ns,
        config.traversed_free_support_radius_m});
    constexpr std::size_t kMaximumSegments = 512U;
    if (segments.size() > kMaximumSegments) {
      segments.erase(segments.begin(), segments.begin() +
          static_cast<std::ptrdiff_t>(segments.size() - kMaximumSegments));
    }
    last_traversed_pose_ = base_pose;
    last_traversed_epoch_ = observation.localization_epoch;
    last_traversed_stamp_ns_ = observation.stamp_ns;
    return std::make_shared<const TraversedFreeSpace>(
        std::move(segments), config.traversed_free_max_age_s);
  }

  std::string config_path_;
  std::function<double()> wall_clock_seconds_;
  std::unique_ptr<internal::RuntimeMappingMap> map_;
  double snapshot_publication_period_s_{0.05};
  std::int64_t snapshot_publication_period_ns_{50'000'000};
  std::size_t full_snapshot_owned_bytes_{0U};
  rog_map::PointCloud backend_cloud_;
  rog_map::PointCloud backend_free_space_cloud_;
  std::shared_ptr<const std::vector<navigation_world_model::GridIndex3>> nearest_offsets_;
  std::shared_ptr<const MappingWorldSnapshot> current_snapshot_;
  std::uint64_t localization_epoch_{1};
  std::uint64_t world_generation_{1};
  std::uint64_t world_revision_{0};
  std::int64_t last_observation_stamp_ns_{0};
  std::uint64_t last_scan_sequence_{0};
  bool poisoned_{false};
  TraversedFreeSpacePtr traversed_free_space_;
  std::optional<navigation_world_model::Point3> last_traversed_pose_;
  std::uint64_t last_traversed_epoch_{0};
  std::int64_t last_traversed_stamp_ns_{0};
  TraversedChainResetReason traversed_reset_reason_{TraversedChainResetReason::kNone};
  navigation_world_model::WorldChangeHistoryPtr change_history_;
  navigation_world_model::AxisAlignedBox pending_changed_region_{};
  bool pending_change_covers_world_{false};
  std::string expected_world_frame_id_;
  std::string expected_body_frame_id_;
};

MappingActor::MappingActor(const std::string& config_path,
                           std::function<double()> wall_clock_seconds,
                           MappingFrameContract frame_contract,
                           const double snapshot_publication_period_s)
    : impl_(std::make_unique<Impl>(config_path, std::move(wall_clock_seconds),
                                   std::move(frame_contract),
                                   snapshot_publication_period_s)) {}

MappingActor::~MappingActor() = default;

MappingConfiguration MappingActor::configuration() const noexcept {
  return impl_->configuration();
}

MappingSnapshotPublication MappingActor::initialSnapshot() {
  return impl_->initialSnapshot();
}

MappingUpdateResult MappingActor::process(const MappingObservation& observation) {
  return impl_->process(observation);
}

}  // namespace navigation_mapping
