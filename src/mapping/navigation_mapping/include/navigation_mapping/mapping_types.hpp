#pragma once

#include <cstddef>
#include <cstdint>
#include <memory>
#include <limits>
#include <stdexcept>
#include <string>
#include <string_view>
#include <vector>

#include <navigation_world_model/world_model_view.hpp>
#include <navigation_mapping/traversed_free_space.hpp>

namespace navigation_mapping {

// Product-owned point representation at the mapping boundary.  The intensity
// field is retained because the sensor adapter and the map filter both use it;
// no PCL type is part of the product contract.
struct PointXYZI {
  float x{0.0F};
  float y{0.0F};
  float z{0.0F};
  float intensity{0.0F};

  friend bool operator==(const PointXYZI&, const PointXYZI&) = default;
};

using PointCloud = std::vector<PointXYZI>;

enum class MapUpdateOutcome : std::uint8_t {
  kUpdated,
  kAccumulated,
  kSlideOnly,
  kEmptyCloud,
  kCallbackOwned,
  kBelowGround,
  kAboveCeiling,
};

enum class MappingObservationRejectionReason : std::uint8_t {
  kNone = 0,
  kMalformedObservation,
  kMissingSensorOrigin,
  kSensorOriginContractMismatch,
};

[[nodiscard]] constexpr std::string_view mappingObservationRejectionReasonName(
    const MappingObservationRejectionReason reason) noexcept {
  switch (reason) {
    case MappingObservationRejectionReason::kNone: return "NONE";
    case MappingObservationRejectionReason::kMalformedObservation:
      return "MALFORMED_OBSERVATION";
    case MappingObservationRejectionReason::kMissingSensorOrigin:
      return "MISSING_SENSOR_ORIGIN";
    case MappingObservationRejectionReason::kSensorOriginContractMismatch:
      return "SENSOR_ORIGIN_CONTRACT_MISMATCH";
  }
  return "UNKNOWN";
}

class MappingObservationRejected final : public std::invalid_argument {
 public:
  explicit MappingObservationRejected(
      const MappingObservationRejectionReason reason)
      : std::invalid_argument(
            std::string(mappingObservationRejectionReasonName(reason))),
        reason_(reason) {}

  [[nodiscard]] MappingObservationRejectionReason reason() const noexcept {
    return reason_;
  }

 private:
  MappingObservationRejectionReason reason_;
};

[[nodiscard]] constexpr bool worldUpdateAdvanced(MapUpdateOutcome outcome) noexcept {
  return outcome == MapUpdateOutcome::kUpdated ||
         outcome == MapUpdateOutcome::kSlideOnly;
}

[[nodiscard]] constexpr std::string_view worldUpdateOutcomeName(
    MapUpdateOutcome outcome) noexcept {
  switch (outcome) {
    case MapUpdateOutcome::kUpdated: return "UPDATED";
    case MapUpdateOutcome::kAccumulated: return "ACCUMULATED";
    case MapUpdateOutcome::kSlideOnly: return "SLIDE_ONLY";
    case MapUpdateOutcome::kEmptyCloud: return "EMPTY_CLOUD";
    case MapUpdateOutcome::kCallbackOwned: return "CALLBACK_OWNED";
    case MapUpdateOutcome::kBelowGround: return "BELOW_GROUND";
    case MapUpdateOutcome::kAboveCeiling: return "ABOVE_CEILING";
  }
  return "UNKNOWN";
}

// Aggregate map-update instrumentation.  These fields intentionally use
// product vocabulary and units; backend-specific structures do not cross the
// mapping boundary.
struct RaycastDiagnostics {
  MapUpdateOutcome update_outcome{MapUpdateOutcome::kEmptyCloud};
  std::uint64_t endpoint_count{0};
  std::uint64_t free_space_endpoint_count{0};
  std::uint64_t attempt_count{0};
  std::uint64_t processed_count{0};
  std::uint64_t free_space_attempt_count{0};
  std::uint64_t free_space_processed_count{0};
  std::uint64_t free_space_clipped_count{0};
  std::uint64_t free_space_skipped_count{0};
  std::uint64_t clipped_count{0};
  std::uint64_t skipped_count{0};
  std::uint64_t skip_nonfinite{0};
  std::uint64_t skip_intensity{0};
  std::uint64_t skip_point_filter{0};
  std::uint64_t skip_below_raycast_min_range{0};
  std::uint64_t skip_endpoint_outside_local_map{0};
  std::uint64_t clipped_virtual_ground_or_ceiling{0};
  std::uint64_t clipped_raycast_max_range{0};
  std::uint64_t clipped_local_update_box{0};
  std::uint64_t ray_outside_local_map_step{0};
  std::uint64_t voxel_traversal_count_total{0};
  std::uint64_t voxel_traversal_count_max{0};
  std::uint64_t hit_candidate_count{0};
  std::uint64_t miss_candidate_count{0};
  std::uint64_t unique_hit_voxel_count{0};
  std::uint64_t unique_miss_voxel_count{0};
  std::uint64_t update_cache_entry_count{0};
  std::uint64_t unique_update_cache_voxel_count{0};
  std::uint64_t map_slide_check_count{0};
  std::uint64_t map_slide_count{0};
  std::int64_t map_slide_voxel_shift_x{0};
  std::int64_t map_slide_voxel_shift_y{0};
  std::int64_t map_slide_voxel_shift_z{0};
  std::uint64_t map_slide_cells_cleared{0};
  std::uint64_t body_neighborhood_cells_cleared{0};
  std::uint64_t traversed_segment_count{0};
  std::int64_t traversed_latest_stamp_ns{0};
  double traversed_latest_age_s{std::numeric_limits<double>::quiet_NaN()};
  std::uint8_t traversed_chain_reset_reason{0};
  navigation_world_model::Point3 base_pose_world{
      navigation_world_model::Point3::Constant(std::numeric_limits<double>::quiet_NaN())};
  navigation_world_model::Point3 sensor_origin_world{
      navigation_world_model::Point3::Constant(std::numeric_limits<double>::quiet_NaN())};
  std::uint64_t inflation_update_count{0};
  std::int64_t map_update_us{0};
  std::int64_t raycast_us{0};
  std::int64_t probability_update_us{0};
  std::int64_t inflation_us{0};
  std::int64_t slide_us{0};
  std::uint64_t allocated_voxel_count{0};
  navigation_world_model::Point3 changed_region_min{
      navigation_world_model::Point3::Zero()};
  navigation_world_model::Point3 changed_region_max{
      navigation_world_model::Point3::Zero()};
  bool changed_region_covers_world{false};
};

// Snapshot resource accounting belongs to the mapping boundary rather than
// to the concrete storage type.  Runtime and report code consume this value
// without depending on the snapshot implementation or its backend details.
struct MappingSnapshotMetrics {
  std::uint64_t bytes{0};
  std::uint64_t owned_bytes{0};
  std::uint64_t shared_metadata_bytes{0};
  std::uint64_t live_count{0};
  std::uint64_t peak_live_count{0};
  std::uint64_t live_owned_bytes{0};
  std::uint64_t peak_live_owned_bytes{0};
  std::uint64_t traversed_free_bytes{0};
};

struct MappingSnapshotPublication {
  navigation_world_model::WorldModelViewPtr view;
  MappingSnapshotMetrics metrics{};

  [[nodiscard]] explicit operator bool() const noexcept {
    return static_cast<bool>(view);
  }
};

}  // namespace navigation_mapping
