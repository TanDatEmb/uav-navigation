# R6 Mapping layer (navigation_mapping, navigation_world_model, rog_map_vendor)

Static review at `main @ 7e0b850`. Findings are R6-01..R6-23 in `findings.csv`. Upstream diffs were taken against `out/R6/upstream/` (ROG-Map, HKU MaRS).

## Role
- The layer is the **sole owner of the mutable occupancy map** and the **producer of immutable world snapshots** that planners pin. Its chain is: registered LiDAR scan + corrected odometry + sensor origin → probabilistic voxel map (ROG-Map ProbMap + InfMap) → detached `MappingWorldSnapshot` (`WorldModelView`) → `WorldSnapshotStore` publication gate → planner, runtime and execution validators.
- `navigation_world_model` is the product query contract, header-only: CellState, UnknownPolicy, WorldGeometry, WorldSnapshotIdentity, change history, CurrentBodySupport witness, continuous tube clearance, WorldCommitAuthorizer.
- Layer position: it sits between estimation (FAST-LIO registered cloud) and planning. It holds no ROS node; the runtime node composes it (`navigation_runtime_node.cpp:877-895, 1510`).
- Provenance of rog_map_vendor (diff vs upstream):

| file | status |
|---|---|
| prob_map.cpp/.h | **heavily project-modified** (about 460 diff lines). Changes: per-instance init guard, the virtual planes gated by an enable flag, the MapUpdateOutcome enum, RaycastDiagnostics, a separate `ray_origin` (lever arm), a free-space endpoint branch (project copy of the upstream clip bug, R6-01), and continuing after a slide instead of returning. Hit/miss math, `resetCell=0` and midpoint reset remain upstream (R6-02). |
| rog_map.cpp/.h | **mostly project-owned**. exportPlanningGrid, exportPlanningGridRegion, estimatePlanningGridRegionSize, the hardened isLineFree/isLineKnownFree/findNearest* and updateMap overloads. |
| inf_map.cpp/.h | project-modified: exports, the planning-state counter (dead, R6-08), the triggerJumpingEdge rewrite, getBaseGridType. The counter logic is upstream. |
| sliding_map.cpp | project-hardened: checked index conversion, deterministic initial origin, slide-cleared counter. The wrap math is upstream. |
| raycaster.cpp | project: stepIndex, the finished-axis guard (no overshoot/infinite loop), checked conversions. |
| config.hpp | project: `validateConfiguration`, and virtual-plane quantization rewritten with long double. Loading and defaults are upstream (R6-20). |
| counter_map, esdf_map, free_cnt_map | upstream with a namespace rename only (esdf/frontier are disabled in the product). |
| navigation_math/* | upstream `super_utils` with the namespace/macro rename and zero-initialized RobotState (V3, R6-23). yaml_loader was hardened (type errors throw) but missing keys still default silently. |
| planning_grid_export.hpp | project-owned. |

## Components
- `MappingActor` (+`Impl`, mapping_actor.cpp): validates the observation, drives the backend update, maintains epoch/generation/revision, builds the bounded change history (256), decides full vs patch export and poisons itself on any post-admission exception.
- `MappingObservation` (mapping_observation.hpp): the immutable, unique-owned input: cloud, corrected odom, epoch, sequence, stamp, no-return endpoints, sensor origin plus origin identity.
- `mapping_types.hpp`: product PointXYZI/PointCloud, MapUpdateOutcome, rejection reasons, RaycastDiagnostics, snapshot metrics.
- `MappingWorker<Obs>` (mapping_worker.hpp): single jthread that owns the mutable map through the process callback. It has a latest-only READY slot, order-key monotonicity, validate at submit and at dequeue, a reset barrier, and fatal fail-stop.
- `ObservationAccounting`: mutex-serialized lifecycle counters with invariants (received/inbox/ready/mapping).
- `MappingWorldSnapshot` (mapping_world_snapshot.hpp): immutable root grid or patch-over-parent (depth ≤ 8). Implements classify, the DDA supercover segment test, the body-support segment oracle, nearestNotOccupied, observedOccupiedPoints, the changed-region certificate and live byte accounting.
- `WorldSnapshotStore`: `atomic<shared_ptr>` latest view plus a publication gate. Handles publish/publishAndFinalize with a monotonic identity, and commitIfCurrent(OrUnaffected).
- `current_body_support.hpp`: the X500 base_link OBB witness factory, with hard-coded geometry plus sha256 provenance.
- `internal::RuntimeMappingMap` (adapter.hpp): ROGMap subclass supplying the wall clock and loadConfigAndInit.
- `internal::MappingWorldModelView`: live-map WorldModelView used **only by tests** (R6-06).
- `ROGMap` (rog_map.cpp): public backend facade: updateMap, the exports, line-free queries, nearest search.
- `ProbMap`: log-odds occupancy buffer (float per voxel), raycast cache, hit/miss update, slide, virtual planes.
- `InfMap`/`CounterMap`: inflated occupancy counters (int16 per cell; a spherical stencil of radius `inflation_step`), optional unknown inflation.
- `SlidingMap`: circular-buffer index math (global→local→hash), slab clearing on slide.
- `RayCaster`: 3D DDA over voxel indices (start voxel .. endpoint exclusive).
- `Config`: YAML → parameters, derived sizes, log-odds, stencils, validation.
- `world_model_view.hpp`: query contract, `isCellTraversable`, `directionalSupportToLocalBoundary`, CurrentBodySupport.
- `continuous_clearance.hpp`: `observedOccupiedTubeIsClear` (segment vs occupied voxel centres, radius ± 1 cm).
- `goal_contract.hpp`: goal tolerances (0.20 m ×3) and `isGoalSegmentTraversable`.
- `world_commit_authorizer.hpp`: WorldValidationLease, the WorldCommitDecision enum, the authorizer interface.

## Processes & threads
- The layer runs inside the runtime process; there is no own executor. Threads involved:
  1. **ROS subscription callback (runtime)** pairs the cloud with the odom and calls `MappingWorker::submitFromWaiting`. It takes the worker `mutex_` and runs `validate_` (freshness/epoch/origin) **under that mutex** (mapping_worker.hpp:66-89).
  2. **Mapping worker jthread** (`MappingWorker::run`): waits on `condition_variable_any` with a stop_token, re-validates at dequeue (freshness 0.5 s = `data_freshness_window_ns_`), then calls `process_` → `MappingActor::process` → backend update → snapshot build → runtime revalidation → `publishAndFinalizeDecision` → `published_handler_`, which builds about 100 diagnostics key/values with `std::to_string` on this same thread.
  3. **Planner / execution threads**: `WorldSnapshotStore::load()` is lock-free and pins a shared_ptr. `commitIfCurrent*` takes `publication_gate_` (a mutex shared with publication).
- Backpressure: a single READY slot is replaced on every new submission (`replaced_ready`); IN_FLIGHT is never pre-empted. With a p50 of 20 ms and a max of 57 ms at 10 Hz this rarely drops, but a full export plus an epoch rebuild (R6-12) can exceed a period. No deadline or budget exists (N11).
- Locks: worker `mutex_` → `ObservationAccounting::mutex_` (one direction only). `publication_gate_` is held only for the identity check plus the light finalize callback; validation stays outside (runtime :1225-1252). `raycast_range_mtx` in ProbMap is uncontended (single thread).
- Process-global statics: `MappingWorldSnapshot` live counters (atomics), `static` locals in `updateOccPointCloud` / `updateMap` empty-cloud counter (not on the product path).

## Flows
- **Input:** `PendingRegisteredScan` (runtime) → `MappingObservation{cloud(unique_ptr<const>), corrected_odometry, epoch, scan_sequence, stamp_ns, free_space_endpoints, sensor_origin_world(+epoch, stamp)}`.
- **Admission** (`validateObservation`, mapping_actor.cpp:507-572): non-empty cloud, stamp>0, epoch≠0, odom stamp == stamp, frame ids == contract, finite pose, non-zero quaternion, finite points and endpoints, finite origin with a matching epoch/stamp, monotonic stamp/sequence within an epoch.
- **Epoch change** (:249-271): build a new RuntimeMappingMap and init it (full allocation) → swap; generation++, revision=0, history reset.
- **Backend** (`ROGMap::updateMap` → `ProbMap::updateProbMap`): outcome gates (callback-owned, empty, above/below plane) → slide if the pose is outside the window or has moved more than 1.5 m from the origin (clears slabs, `changed_region_covers_world`) → `updateLocalBox` → `raycastProcess`. raycastProcess applies the intensity/point filter, virtual plane clip, max range, min range and local box clip, inserts hit candidates, and walks the DDA miss rays from `origin + dir*range_min` to the endpoint (exclusive) → `probabilisticMapFromCache` (hit wins if any hit in the voxel; the log-odds update is clamped [l_min,l_max]) → counter/inflation updates on class transitions.
- **Dirty region** (:307-378): the backend AABB (origin ∪ endpoints) + inflation radius + half-voxel shells → a WorldChangeRecord, prepended to the history (≤256). The pending union covers deferred revisions.
- **Export** (:391-472): publish if no snapshot exists, the whole world changed, or age ≥ 50 ms. Patch if region valid ∧ depth<8 ∧ estimate < 0.4×full; otherwise a full export (`exportPlanningGrid`: 251×251×41 base uint8 + inflated). In practice nearly always full (R6-09).
- **Output:** `MappingUpdateResult{snapshot, identity, timings, export mode/reason, byte metrics}` → runtime revalidates the retained command → `WorldSnapshotStore::publishAndFinalizeDecision` → planners `load()` the pinned view.
- **Queries** (planner/execution): `classify`, `isSegmentTraversable` (DDA supercover + tube clearance on the inflated layer), `isSegmentTraversableWithCurrentBodySupport`, `nearestNotOccupied`, `observedOccupiedPoints`, `clampToLocalBounds`, `changedRegionIntersectsSince`, `directionalSupportToLocalBoundary`.

## State machines
- `MapUpdateOutcome` (mapping_types.hpp:30-38, backend prob_map.h:39-47). The product only accepts UPDATED/SLIDE_ONLY (`worldUpdateAdvanced`). Everything else throws and poisons (mapping_actor.cpp:295-301, R6-04). Transition sites: prob_map.cpp:373-384 (ABOVE_CEILING/BELOW_GROUND), 446-449 (UPDATED/SLIDE_ONLY/ACCUMULATED), rog_map.cpp:700-714 (CALLBACK_OWNED/EMPTY_CLOUD).
- MappingActor implicit states: `READY` → (`poisoned_` = true on any exception inside the try, :497-503) → `POISONED` (terminal: every call throws logic_error, :223-225). Identity: `localization_epoch_` (monotone, a new epoch triggers a rebuild at :249), `world_generation_` (++ per epoch), `world_revision_` (++ per advanced update, reset per epoch), `last_observation_stamp_ns_`.
- `SnapshotExportMode {kDeferred,kFull,kPatch}` and `SnapshotFullExportReason {kNone,kNoCurrentSnapshot,kWholeWorldChanged,kInvalidChangedRegion,kPatchDepthLimit,kEmptyPatch,kPatchTooLarge}` (mapping_actor.hpp:14-28), set at mapping_actor.cpp:428-457.
- MappingWorker lifecycle (bool flags, mapping_worker.hpp:284-290): NOT_STARTED -start()→ RUNNING. RUNNING -submit→ READY slot occupied -dequeue→ IN_FLIGHT → RUNNING. RUNNING -reset()→ RESETTING (drops READY, waits for IN_FLIGHT) → RUNNING. Any validate/order/process/published-handler exception → FATAL (terminal, thread exits, :76-83, 199-208, 224-240, 246-263). shutdown() → STOPPING → JOINED.
- ObservationAccounting per-observation: RECEIVED → {REJECTED_BEFORE_INBOX | WAITING} → {REPLACED_WAITING | DISCARDED_WAITING | NONMONOTONIC | READY} → {REPLACED_READY | DISCARDED_READY | DISCARDED_SHUTDOWN | STARTED} → {PUBLISHED | FAILED} (R6-15: PUBLISHED also covers deferred and superseded).
- `WorldCommitDecision` (world_commit_authorizer.hpp:19-30), decided in world_snapshot_store.hpp:81-160 (kCommitted, kNoPublishedWorld, kWorldAdvanced, kCancelled; kSuperseded passes through from finalize).
- Voxel class (log-odds): UNKNOWN [l_free, l_occ) ↔ KNOWN_FREE (< l_free) ↔ OCCUPIED (≥ l_occ). Transitions happen in hitPointUpdate/missPointUpdate (prob_map.cpp:708-803). Reset to 0 (resetCell :679) or to the midpoint (resetLocalMap :1110) (R6-02).

## Behavior per branch
- `MappingActor::process`: poisoned → throw logic_error. Invalid observation → throw (no poison, pre-mutation). Older epoch → throw runtime_error (no poison). New epoch → rebuild the map; a rebuild failure poisons. Revision exhausted → throw (poison). Backend non-advancing → throw (poison, even when the backend did not mutate, R6-04). Change margin non-finite → whole-world record. Not due → return without a snapshot (deferred). Patch efficient and non-empty → patch snapshot. Otherwise → full snapshot. Any exception → poison + rethrow.
- `validateObservation`: each check throws `invalid_argument` or `MappingObservationRejected(kSensorOriginContractMismatch)`. Stamp/sequence regressions throw `runtime_error`. Checks run before the try block, so they never poison.
- `ProbMap::updateProbMap`: pose above the ceiling or below the ground (only if planes are enabled) → early return with no mutation. Pose outside the window → slide, then continue (project change; upstream returned). Moved more than the threshold, or the map is empty → slide. Non-finite ray origin → throw after a possible slide. batch counter < size → ACCUMULATED (the runtime forbids a size other than 1).
- `ProbMap::raycastProcess`, per point: intensity below threshold → skip. Filter modulo → skip. Non-finite → skip. Raycasting disabled → endpoint hit only (inside the map and ≥ min range). Above/below plane → clip, no hit (**clip length wrong, R6-01**). Beyond max range → scale to max, no hit. Below min range → skip. Outside the update box → intersect with the box (−1e-3 param), no hit. Otherwise → hit candidate plus miss ray. Free-space endpoints: same clipping, never a hit.
- `probabilisticMapFromCache`: hit_cnt>0 → `ret += l_hit*hits`, clamped to l_max (misses in the same voxel are ignored). Otherwise `ret += l_miss*misses`, clamped to l_min. A class change → counter/inflation update (+ESDF/frontier if enabled). The base change counter covers hits only (R6-08).
- `SlidingMap::mapSliding`: non-representable pose → silent return (no slide; the window stays). A shift larger than the size on any axis → full reset (midpoint seed). Otherwise per axis, `shift` slabs are cleared through `resetCell` (seed 0).
- `MappingWorker::submitFromWaiting`: not accepting/fatal/resetting → discard. Validate throws → fatal. Validate false → discard. Order-key throw → fatal. Key ≤ highest → discard as non-monotonic. Otherwise → replace/fill READY.
- `MappingWorker::run`: stop → discard READY and exit. Validate throws → fatal and exit. Invalid → discard and continue. Process throws → FAILED, fatal, exit. Published handler throws → fatal, exit (stays PUBLISHED).
- `WorldSnapshotStore::publishAndFinalizeDecision`: null or invalid identity → throw. Non-monotonic → throw logic_error. Finalize returns non-committed → keep the old world and return the decision. Finalize throws → kCancelled. Otherwise → store.
- `commitIfCurrentOrUnaffected`: no world → kNoPublishedWorld. Identity differs and history is not proven disjoint → kWorldAdvanced. Otherwise → final_commit(lease).
- `MappingWorldSnapshot::changedRegionIntersectsSince`: invalid region or a different epoch/generation, or older is newer → true (fail closed). Same revision → true iff the stamps differ. Retained history shorter than the gap → true. Any record whole-world, overlapping, or non-contiguous → true. Otherwise false.
- `MappingWorldSnapshot::classify`: non-finite → OUT_OF_MAP. Evidence: planes (if enabled) → OCCUPIED; outside the base bounds → OUT_OF_MAP; otherwise the stored state. Inflated: outside base∩inflated → OUT_OF_MAP; planes → OCCUPIED; inflated occupied → OCCUPIED; unknown inflation on → UNKNOWN/FREE by the inflated unknown flag; otherwise the base state.
- `isSegmentTraversable`: any endpoint outside → false. Start cell not traversable → false. DDA over cells with the tied-axis supercover → false on the first non-traversable cell. End cell checked explicitly. Inflated layer → additionally `observedOccupiedTubeIsClear` (O(L³) box scan, R6-14; centre distance, R6-13).
- `observedOccupiedTubeIsClear`: invalid input → false. Any occupied centre within radius−1 cm of the segment → false. Exception → false.

## Information needs
| input | source | frame | clock | freshness bound | authority |
|---|---|---|---|---|---|
| registered cloud (points, intensity) | FAST-LIO via runtime `PendingRegisteredScan` | planning world frame (`planning_frame_`, e.g. lio_odom ENU) | ROS time (sim) → `stamp_ns` int64 | runtime validator: `data_freshness_window_ns_` (0.5 s) at submit and dequeue; no age check inside the actor | estimator (sole) |
| corrected odometry pose (base_link) | same message (`corrected_pose`) | header.frame_id == planning frame, child == body frame (validated) | header.stamp must equal the cloud stamp | same as the cloud | estimator |
| sensor ray origin | same message (`sensor_origin_pose`) | world | stamp and epoch must equal the cloud's | same | estimator/extrinsic |
| explicit no-return endpoints | same message (optional) | world | same stamp | same | estimator/sensor adapter |
| localization epoch | runtime `active_localization_epoch_` | n/a | monotone counter | an older epoch is rejected; a newer one rebuilds the map | runtime/localization |
| wall clock for backend `robot_state_.rcv_time` | `ros_clock->now()` | n/a | ROS time | informational only | runtime |
| mapping config | planner.yaml `rog_map:` | n/a | startup | static; missing keys default silently (R6-20) | config |
| snapshot publication period | `PlanningTimingContract::kSnapshotPeriodS` (0.05 s) | n/a | observation stamp domain (not wall) | age of the last built snapshot ≥ period | planning contract |
| consumer: pinned snapshot identity | WorldSnapshotStore | n/a | `observation_stamp_ns` (sensor stamp) | the consumer must compare against now (not enforced in this layer; `classifyFreeSpace` ignores `now_stamp_ns`) | mapping (sole writer) |
| consumer: CurrentBodySupport | runtime/planner factory | world→base_link | stamp must equal both the snapshot and the measured pose | exact equality | estimator + model SDF provenance |

## Bottlenecks
- **Full snapshot export on almost every publication** (R6-09). The 2.58M base cells plus the inflated grid are walked, copied and allocated (about 6.8 MB) per scan. The AABB dirty region of a 360° LiDAR defeats the patch path; slides (every 1.5 m) force a full export anyway. This is the main cost inside the 20 ms p50 / 57 ms max, and it runs on the single mapping thread with no budget (N11).
- **Epoch rebuild on the mapping thread** (R6-12): full re-allocation, the stencil build (about 69k nearest offsets), a debug file open and heavy stdout.
- **Per-transition overhead**: TimeConsuming string alloc and clock reads (R6-21); a 257-neighbour inflation stencil per occupied transition (upstream design).
- **Triple finiteness scan plus a PCL copy** of every cloud (R6-05).
- **Consumer side**: `observedOccupiedTubeIsClear` scans O(L³) voxels and allocates per segment on planner threads (R6-14); patch-chain lookups cost up to 8 offset computations per voxel query.
- **The diagnostics handler** builds about 100 `std::to_string` key/values on the mapping thread after each scan (runtime-owned lambda, mapping_worker.hpp:242-265).

## Notes for target architecture
Keep:
- The single-owner actor with immutable, validated snapshots, the fail-closed identity (epoch/generation/revision/stamp), the monotonic publication gate, and the change-history certificate (bounded, contiguous, fail-closed).
- The latest-only worker with exact lifecycle accounting and invariants; validation at admission and at dequeue.
- The project fixes to vendor code: per-instance init guard, checked index math, the DDA finished-axis guard, a separate sensor ray origin, no-return endpoints as miss-only evidence, and no processing loss after a slide.
- The product-owned `WorldModelView` contract and the explicit `UnknownPolicy`; the supercover DDA with tie handling; the explicit body-support witness scoped to the start prefix.

Over-engineered or unnecessary:
- The patch-snapshot machinery (depth-8 chain, size estimates, 7 full-export reasons) does not pay off with an AABB dirty region (R6-09). Replace it with chunked copy-on-write storage and a dirty-chunk set, and drop the patch chain.
- `MappingWorldModelView` (a test-only second implementation, R6-06), the dead planning-state counters (R6-08, R6-22), research timers and logs (R6-12, R6-21), and the ESDF/frontier/PCD/ROS-callback code paths that are all disabled in the product (keep them only behind the vendor boundary).
- Virtual-plane logic: it is disabled in the product frame, yet it carries four inconsistent predicates, two config copies and a wrong clip formula (R6-01, R6-03, R6-07). Either remove it from the product build or unify it into one predicate before re-enabling.
- Two unknown seeds (R6-02) and silent config defaults (R6-20) should become one explicit schema with required keys and one seed constant.
- The GridType↔CellState ordinal reinterpretation (R6-11) should become an explicit mapping table.
