#pragma once

#include <cstdint>
#include <functional>
#include <memory>
#include <string>

#include <navigation_mapping/mapping_observation.hpp>
#include <navigation_mapping/mapping_diagnostics.hpp>
#include <navigation_mapping/visibility_control.hpp>

namespace navigation_mapping {

enum class SnapshotExportMode : std::uint8_t {
  kDeferred = 0,
  kFull = 1,
  kPatch = 2,
};

enum class SnapshotFullExportReason : std::uint8_t {
  kNone = 0,
  kNoCurrentSnapshot = 1,
  kWholeWorldChanged = 2,
  kInvalidChangedRegion = 3,
  kPatchDepthLimit = 4,
  kEmptyPatch = 5,
};

struct MappingUpdateResult {
  MapUpdateOutcome outcome{MapUpdateOutcome::kEmptyCloud};
  RaycastDiagnostics diagnostics{};
  navigation_world_model::WorldModelViewPtr snapshot;
  MappingSnapshotMetrics snapshot_metrics{};
  std::uint64_t localization_epoch{0};
  std::uint64_t world_generation{0};
  std::uint64_t world_revision{0};
  std::int64_t observation_stamp_ns{0};
  std::int64_t map_update_us{0};
  std::int64_t snapshot_export_us{0};
  SnapshotExportMode snapshot_export_mode{SnapshotExportMode::kDeferred};
  SnapshotFullExportReason snapshot_full_export_reason{SnapshotFullExportReason::kNone};
  std::uint64_t snapshot_export_base_cells{0};
  std::uint64_t snapshot_export_inflated_cells{0};
  std::uint64_t snapshot_patch_depth{0};
};

// Product-owned configuration facts consumed by runtime composition. The
// mutable backend configuration remains private to navigation_mapping.
struct MappingConfiguration {
  bool callbacks_enabled{false};
  int raycasting_batch_update_size{1};
  bool virtual_ground_ceiling_enabled{true};
};

struct MappingFrameContract {
  std::string world_frame_id;
  std::string body_frame_id;
};

// Sole mutable map owner and immutable snapshot builder. The caller may
// publish returned snapshots after process() returns; no mutable map handle
// leaves this class.
class NAVIGATION_MAPPING_PUBLIC MappingActor final {
 public:
  explicit MappingActor(const std::string& config_path,
                        std::function<double()> wall_clock_seconds = {},
                        MappingFrameContract frame_contract = {},
                        double snapshot_publication_period_s = 0.05);
  ~MappingActor();

  MappingActor(const MappingActor&) = delete;
  MappingActor& operator=(const MappingActor&) = delete;
  MappingActor(MappingActor&&) = delete;
  MappingActor& operator=(MappingActor&&) = delete;

  [[nodiscard]] MappingConfiguration configuration() const noexcept;
  [[nodiscard]] MappingSnapshotPublication initialSnapshot();
  MappingUpdateResult process(const MappingObservation& observation);

 private:
  class Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace navigation_mapping
