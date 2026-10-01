# R4 Estimation layer (FAST-LIO: fast_lio_core, fast_lio_ros, fast_lio_tools, ikfom_vendor, ikd_tree_vendor)

Static review of `main @ 7e0b850`. All paths below are relative to `src/estimation/`.

## Role

- This is the only source of navigation state. It fuses LiDAR (Gazebo PointCloud2 in SITL, Livox CustomMsg or per-point PointCloud2 on datasets) with the IMU in an IKFoM iterated ESEKF (FAST-LIO2 23-DoF error state, gravity on S2). The registration map is an ikd-tree.
- It publishes:
  - `/lio/odometry_corrected`: nav_msgs/Odometry at scan rate, base_link in lio_odom.
  - `/lio/odometry_propagated`: PropagatedOdometry at 50 Hz, with a localization epoch and a sequence number.
  - Dynamic TF lio_odom→base_link, driven by the propagated state.
  - `/lio/mapping_observation`: RegisteredScan, plus the optional `/lio/registered_points`.
  - `/lio/health`: typed EstimatorHealth.
  - `/lio/diagnostics`: DiagnosticArray with estimator, transport and propagated-odometry entries.
- Downstream consumers:
  - The PX4 external-mode adapter (odometry, typed health, localization epoch).
  - The runtime (typed health; it re-derives acceleration and jerk itself in `navigation_runtime/kinematic_derivative_estimator`, and LIO publishes no acceleration).
  - The mapping layer (RegisteredScan).
- Layer: sensor-fusion/estimation, below world model and planning. It owns clock-domain tagging (`Timestamp` + `ClockDomain`), the frame contract (lio_odom ENU, base_link FLU, livox_imu_frame, livox_frame) and the public frame epoch.

## Components

| Class / file | Responsibility |
|---|---|
| `Timestamp`, `Duration`, `ClockDomain`, `TimestampValidator` (fast_lio_core/time) | int64 ns time with overflow-checked arithmetic, clock-domain tags and per-stream monotonicity. |
| `Status`/`Result<T>` (common) | Error propagation. `Result::value()` throws on error. |
| `ImuSample`, `LidarPoint`, `LidarScan`, `MeasurementGroup` (sensor) | Typed inputs. A scan has start/end and uint32 relative point time. |
| `MeasurementBuffer` | Bounded deques (32 scans / 8192 IMU, hard-coded defaults) with validators and a mutex. |
| `MeasurementSynchronizer` | Pops the front scan, brackets IMU from the previous synchronized end to the scan end, gates the IMU gap, emits a group or a discontinuity, trims IMU. |
| `ImuInitializer` | Stationary mean/variance gate. Gravity = −g·Z, orientation = FromTwoVectors(acc, Z), gyro bias = gyro mean. The accel-norm error goes into accel bias. |
| `InitialStatePriorApplicator` / `InitialStatePriorPolicy` | Applies zero/fixed/topic priors (yaw-only/full attitude, lever arm, velocity) to the IMU-initialized state. |
| `IkfomEstimator` (+ `ikfom_state.cpp`) | Wraps `esekfom::esekf`: transactional predict (midpoint IMU input, 20 ms step cap), iterated update through a static thread-local measurement callback, frozen extrinsic and gravity. |
| esekfom.hpp patch (`detail::solve_active_normal_equations`) | Information-form gain on the 12 active columns with diagonal R. Solve failure → rejected update. |
| `ScanDeskewer` + `ImuTrajectory` | Per-point motion compensation to scan end from predicted IMU states. It is bypassed in simultaneous_scan (SITL). |
| `PointCloudPreprocessor`/`PointFilter`/`VoxelFilter` | Range filter and first-point-per-voxel downsample in the LiDAR frame. |
| `ResidualBuilder`/`PlaneEstimator`/`ResidualGate` | OpenMP (3 threads) 5-NN search, plane fit, Huber gate, point-to-plane H rows (6 active columns) and translational observability ratio. |
| `IkdTreeRegistrationMap` | PIMPL over the patched ikd-tree: allocation-free k-NN, sorted downsampled insertion, six-slab moving-cube crop, async rebuild, trylock size. |
| `LocalMapManager` / `MapInsertionPolicy` | Crops when moved 5 m. The absolute point guard latches insertion freeze until reset. Insertion is allowed only when Tracking. |
| `FastLioPipeline` | Lifecycle state machine and orchestration (sync → predict → deskew → filter → bootstrap/correct → map). Also covers diagnostics, discontinuity recovery, the prior mailbox and the generation counter. |
| `AngularVelocityResolver` | Bias-corrected gyro at the exact state epoch (interpolated with integer ns). |
| `BaseLinkStateConverter` | ^odom T_imu → ^odom T_base, lever-arm velocity, body-frame twist. |
| `BaseLinkCovarianceProjector` | 23×23 IMU error covariance → 6×6 pose (odom axes) and twist (base axes), with PSD repair and rejection. |
| `ImuStatePropagator` | Independent IKFoM copy for high-rate propagation: IMU history, re-anchor + replay at each correction, continuity epochs on gaps. |
| `PropagatedOdometryWorker` (fast_lio_ros) | `lio_propagated` thread: IMU ingress, correction mailbox, control generation, load shedding, stale-correction stop, 50 Hz publication. |
| `FastLioNode` | Subscriptions, ingress queues, `fast_lio_main` worker loop, fan-out of IMU to the propagated worker, result publication, timers. |
| `RosLidarAdapter` / `RosLivoxCustomAdapter` / `RosImuAdapter` | ROS → core conversion, point-time normalization, overlap trim, frame/clock checks. |
| `RosOutputPublisher` | Corrected odometry, registered scan + visibility association, typed health (two producers), diagnostics. |
| `RosPropagatedOdometryPublisher`, `RosTransformPublisher`, `RosOdometrySerializer`, `RosTimeConverter` | Propagated output, TF, message construction, ns↔ROS time. |
| `LioPublicFrameGeneration` | Public localization epoch, seeded from steady_clock and bumped only on pipeline reset. |
| `ParameterLoader`/`EstimatorProfile` | Declares and validates ROS parameters and maps them to `EstimatorConfig`. It rejects unknown overrides. The canonical frame check is offline-only. |
| `RuntimeStatistics`, `CovarianceProjectionRuntime` | Full-run latency distributions (exact samples ≤65,536) and atomic covariance availability. |
| `fast_lio_tools/lio_offline` | Bag replay through the same pipeline. Bag order; no propagated worker, no wall-clock supervision. |

## Processes & threads

One process, `fast_lio`, with `rclcpp::spin` on a single-threaded executor (main.cpp:15). Its threads:

1. **Executor thread.** It runs all subscription callbacks: IMU, LiDAR/Livox (full point conversion in callback), visibility cloud and initial prior. It also runs the timers:
   - 10 ms prior-publisher match,
   - 0.5 s `publishDiagnosticsSnapshot`,
   - 1 s `publishTransportSnapshot`. This one sorts latency samples while holding `input_mutex_` (R4-19).

   The IMU callback enqueues to the main queue and then to the propagated worker (fast_lio_node.cpp:298-337).
2. **`fast_lio_main`** (`processingLoop`, fast_lio_node.cpp:596). It is the single owner of `FastLioPipeline`.
   - It pops one measurement at a time, IMU first when `imu.time <= lidar.end`, and calls `pushImu`/`pushLidar`.
   - It then runs `publishAvailableResults()`, which loops over `processNext()` → `output_publisher_.publish()` (which may wait 10 ms for the visibility cloud, R4-07) → `enqueueEstimatorState`.
   - When no input arrives for 20 ms wall, it calls `superviseLidarTimeout()`, which works on wall clock (R4-22).
   - Map insertion and crop run inline here (R4-18).
3. **`lio_propagated`** (`PropagatedOdometryWorker::run`). It drains the IMU batch, handles the correction mailbox (replay outside the mutex) and publishes the propagated odometry, TF and `/lio/health` through the node callback.
4. **OpenMP team** (3 threads) inside `ResidualBuilder::buildInto` for the k-NN search and plane fit.
5. **ikd-tree async rebuild thread** (vendor, `enable_asynchronous_rebuild=true`).

Locks:
- `input_mutex_` guards the ingress queues and runtime diagnostics.
- `MeasurementBuffer::mutex_`.
- `initial_prior_mutex_`.
- Worker `mutex_`.
- `diagnostics_mutex_`, `converter_mutex_` and `visibility_cloud_mutex_` in the output publisher.
- Vendor search/rebuild mutexes.

## Flows

1. **Ingress.**
   - `/lidar/imu` → `RosImuAdapter::convert` (frame and finiteness checks, `Timestamp(stamp, clock_domain)`) → `enqueue` (capacity 4096) and `PropagatedOdometryWorker::enqueueImu` (capacity 4096).
   - `/lidar/points` → `RosLidarAdapter::convert`. For a simultaneous scan, start = end = header. For per_point, relative times are computed with overlap trim. The result goes to `enqueue` (capacity 16).
2. **Core step** (`fast_lio_main`):
   - `pushImu` → `MeasurementBuffer` + `prior_imu_history_` (1 s) + initializer (until InitializingMap).
   - `pushLidar` → buffer.
   - `processNext`:
     - `synchronizeNext` returns one of: group / discontinuity (→ `recoverFromDiscontinuity`, a zero-order-hold rebase, R4-03) / error / wait.
     - `processInternal` then runs these steps:
       1. validate,
       2. epoch/rebase checks,
       3. `buildPredictionImuSamples`,
       4. `IkfomEstimator::predict` (state_time_ = scan end),
       5. deskew,
       6. preprocess,
       7. bootstrap capture (first scan) or `IkfomEstimator::correct`: up to 4 iterations, each a full k-NN and plane rebuild (R4-17),
       8. lifecycle transition,
       9. `AngularVelocityResolver::resolve`,
       10. registered points in odom,
       11. map insert and crop,
       12. `ProcessResult`.
3. **Output** (`publishAvailableResults`):
   - A public-epoch bump happens only when the pipeline generation increased.
   - `RosOutputPublisher::publish`:
     - converter → projector → serializer → `/lio/odometry_corrected`,
     - `makeCloud` → `/lio/registered_points` and `RegisteredScan` (with the visibility cloud converted to odom free-space endpoints) → `/lio/mapping_observation`,
     - `/lio/health` stamped with scan end, and `/lio/diagnostics` on validity edges.
   - Then `EstimatorStateUpdate{status_after, navigation_valid, corrected_estimate, correction_sequence}` goes to the worker.
4. **Propagation** (`lio_propagated`):
   - Each IMU goes to `acceptImu`. A gap > 20 ms starts a new continuity epoch.
   - Each correction goes to `reanchorAndReplay` (rebase to the corrected state and covariance, replay history to the latest IMU).
   - At each deadline (20 ms), `flushPendingPrediction` runs, followed by the correction-age check (0.5 s) and then `maybePublishOnImu`:
     - `kinematicEstimate` (exact IMU at the state epoch),
     - `RosPropagatedOdometryPublisher::publish` → `/lio/odometry_propagated` (epoch, sequence),
     - `RosTransformPublisher` → TF,
     - `publishPropagatedHealth` → `/lio/health` stamped with the propagated time.
5. **Diagnostics timers.**
   - 0.5 s: latest health snapshot → `/lio/diagnostics` (stamp = `clock_->now()`).
   - 1 s: transport and propagated-worker diagnostics. This also sets `propagation_valid_`.

## State machines

**`EstimatorStatus`** (pipeline/estimator_diagnostics.hpp:15). All lines are in fast_lio_pipeline.cpp.
- WaitingForSensors → CollectingImu: :160 (first IMU), :379 (direct process).
- CollectingImu → InitializingImu: :1233 (minimum samples), :1247 (evaluating).
- InitializingImu → InitializingMap: :1375 (prior or fallback accepted). The in-flight path is :1251-1282.
- InitializingMap → Tracking: :708-711 (first successful correction; sets tracking_ever_confirmed_).
- InitializingMap → Lost: :689 (initial map registration exhausted).
- InitializingMap → InitializingMap: :1199/:1204 (IMU discontinuity before tracking).
- Tracking → Degraded:
  - :254 (wall-clock LiDAR timeout),
  - :1661 (any uncorrected update after tracking),
  - :798 (map absolute guard).
- Degraded → Lost: :259 (wall timeout), :1659 (consecutive uncorrected ≥ lost_after_registration_failures).
- Discontinuity → Lost: :1209 (long discontinuity), :1185 (inflation failure).
- Degraded/Lost → Tracking: :724 (recovery_confirmation_updates successes; blocked forever by map_recovery_required_, :715-719).
- Lost/Degraded → Degraded: :727 (confirmation pending), :717.
- Any → Resetting → WaitingForSensors: :848/:906 (reset(); **never called by the ROS node**).

Latent dead ends:
- Pre-tracking prediction failure: permanent PROPAGATION_START mismatch (R4-12).
- IMU buffer saturation (R4-01).
- Map guard latch (R4-16).

**`PropagatedOdometryStatus`** (imu_state_propagator.hpp:19). All lines are in imu_state_propagator.cpp unless marked worker.
- WaitingForCorrection → Ready: :290 (flush), :351 (replay).
- Ready → ImuGap: :249 (continuity restart).
- Ready → TimestampRegression / ImuGap / MissingBracket / InvalidState: :463-481 (setFailure).
- Ready → QueueOverflow: worker :218.
- Ready → MainEstimatorInvalid: worker :222.
- Ready → StaleCorrection: worker :330.
- Ready → WaitingForCorrection: worker :459 (superseded).
- Every non-Ready state needs a new correction re-anchor, gated by `control_generation_` (worker :111, :211, :373) and `suspended_`.

**`InitialPriorStatus`**:
- NotRequired/Waiting: :142-145.
- Waiting: :1443, :1464.
- CandidateAvailable: :1121 (reporting only).
- Applied: :1408, :1451.
- FallbackApplied: :1500.
- Rejected: :1467, :1495, :1338.
- Closed: :542 (first prediction).
- Topic-prior gating: `initial_prior_gate_closed_`/`estimator_initialized_` atomics (:177, :541).

**Public frame generation** (lio_public_frame_generation.cpp:9-30): the generation increments only on `kPublicFrameDiscontinuity` with a new token. The node calls it only when the pipeline generation grows (fast_lio_node.cpp:756-766), which happens only via reset(). The epoch is therefore constant for the process lifetime.

**`SynchronizationFailureKind`** is {None, ImuDiscontinuity}. Only the discontinuity is ever set (measurement_synchronizer.cpp:139).

## Behavior per branch

| Function | Branch → outcome |
|---|---|
| `MeasurementSynchronizer::synchronizeNext` (measurement_synchronizer.cpp:11) | Empty queue → wait. Invalid front scan / clock mismatch / overlap / missing start bracket → pop scan + error. No end bracket → wait (forever if IMU is rejected, R4-01). Max gap > limit → pop scan, erase IMU up to gap_end, epoch = gap_end (can make the next scan an 'overlap', R4-02). OK → group; retain [end_bracket−1, …]. |
| `FastLioPipeline::processNext` (:263) | Topic prior pending → nullopt (scans accumulate). Sync error → ProcessResult with reason + recordUncorrectedUpdate. Discontinuity → recoverFromDiscontinuity. Wait → nullopt. |
| `FastLioPipeline::processInternal` (:308) | Invalid interval / bracket / scan / IMU sequence → reject (uncorrected). In init states → initializer; not ready → PRIOR_PENDING / IMU_NOT_READY. First epoch → state_time_ = prior or propagation epoch (committed before prediction, R4-12). Mismatch: rebase only if tracking was confirmed and Degraded/Lost, else reject. Prediction fail → reject, state unchanged. Deskew/preprocess fail → reject, state is predicted-only. Too few points → reject. First InitializingMap scan → bootstrap capture. Correction fail → roll back to predicted; map-init counter; Lost at limit. Success → Tracking/recovery logic, kinematic resolve (silently absent on failure), map insert if Tracking and not frozen, crop/guard. |
| `FastLioPipeline::recoverFromDiscontinuity` (:1147) | Not yet estimating → reason only. Inflation fail → Lost. Else rebase at the old state with time = resume_time (ZOH, R4-03). Before tracking: long gap → restart bootstrap, short gap → keep. After tracking: long gap → Lost, short gap → Degraded (via recordUncorrectedUpdate). |
| `FastLioPipeline::buildPredictionImuSamples` (:913) | 'Steady' only if start == scan.start (never in simultaneous_scan, R4-14); else merge with the 1 s history. Missing start/end bracket → error. Gaps compared to the group's *measured* max (R4-13). |
| `IkfomEstimator::predict` (ikfom_estimator.cpp:210) | Validation or step > 20 ms → error before mutation. Per step NaN / asymmetry > 1e-8 → restore and error. |
| `IkfomEstimator::correct` (:329) | Update with a numeric failure, too few residuals, observability invalid, or non-finite / asymmetric P → restore the predicted state. Else keep (converged or at the iteration limit); overwrite extrinsic, re-freeze gravity. |
| `InitialStatePriorApplicator::apply` | Frame mismatch / non-finite → reject. Full attitude on the ground with tilt disagreement → reject. Velocity mask without twist → reject. |
| `ImuStatePropagator::validateAndRecordImu` | Invalid / clock change / non-increasing → setFailure (invalid until next correction). Gap > 20 ms → new continuity epoch. OK → record (+ pending if anchored). |
| `PropagatedOdometryWorker::enqueueEstimatorState` (:100) | Status ≠ Tracking or !navigation_valid → generation++, drop pending, invalidate. Correction with seq 0 / non-finite / clock change → reject. Not newer → drop. Else mailbox (coalesced). |
| `PropagatedOdometryWorker::processPendingCorrection` (:391) | Generation / main-state change → drop. Replay OK → resume. Newer pending → invalidate and wait. Missing end bracket → requeue. Missing start → drop. |
| `FastLioNode::processingLoop` (:596) | Core IMU rejection → worker set Degraded/invalid (R4-05). Exception → worker failed, stopping_ (node stays up, no outputs). |
| `RosOutputPublisher::publish` (:386) | Health snapshot. Validity edge → immediate diagnostics. Uncorrected → typed health only. Converter / projector / serializer fail → health only (no odometry, silent; frame mismatch R4-11). Corrected → odometry, registered scan + visibility (10 ms wait), health. |
| `ParameterLoader::declareAndLoad` / `validate` | Unknown override → throw. Frame names not checked against the core constants (R4-11). IKFoM noise / step not configurable. |

## Information needs

| Input | Source | Frame | Clock | Freshness bound | Authority |
|---|---|---|---|---|---|
| IMU angular rate / specific force | `/lidar/imu` (Gazebo, Livox) | livox_imu_frame (FLU) | `timing.clock_domain` (sim: simulation_time) | Gap ≤ `timing.max_imu_gap_s` (0.02) in the synchronizer; ≤ 20 ms hard-coded in IKFoM and propagator (R4-13) | Sole inertial source |
| LiDAR points | `/lidar/points` PointCloud2 (sim) / Livox CustomMsg | livox_frame | Same domain as IMU (checked) | Wall-clock 0.2 s degraded / 1.0 s lost (R4-22); lidar queue 16 | Sole correction source |
| Per-point time | PointCloud2 field / Livox offset_time | — | Sensor domain, relative uint32 ns | Scan ≤ 0.2 s, header offset ≤ 0.2 s | Adapter |
| Static base→IMU | `/tf_static` (robot_state_publisher) at startup | ^base_link T_livox_imu_frame | TimePointZero | Once at construction (blocking retry) | TF tree; frozen afterwards |
| IMU→LiDAR extrinsic | YAML `extrinsic.*` | ^imu T_lidar | — | Static; frozen in the filter | YAML; also re-read in the output publisher for the sensor origin (duplicate source) |
| Initial prior | zero (sim) / fixed / `initial_prior.topic` Odometry | lio_odom←base_link, body-FLU twist | Input domain | Age ≤ 0.5 s, wait 2 s | YAML policy; topic gate closes at the first prediction |
| Visibility cloud | `input.visibility_points_topic` | livox_frame | Stamp must equal the scan stamp | ≤ 0.5 s association window, 10 ms wait | Optional mapping evidence |
| Corrected state → propagator | `fast_lio_main` (in-process) | lio_odom / IMU | Scan end | Correction age ≤ 0.5 s | Main estimator (generation / sequence guarded) |
| Outputs to consumers | `/lio/odometry_propagated`, `/lio/health` | lio_odom ENU, base_link FLU; quaternion w/xyz from Eigen | Input domain; health has two stamp bases (R4-06) | Adapter: 0.2 s staleness | LIO public epoch |

## Bottlenecks

1. All work on `fast_lio_main` is serial per scan: predict, deskew (per-point allocations on datasets, R4-15), up to 4 full k-NN re-associations (R4-17), map insert + crop inline (R4-18), output serialization, and up to a 10 ms visibility-cloud wait (R4-07). All of it runs before the propagated worker receives the correction, so correction latency directly delays re-anchoring.
2. Per IMU sample, the main thread runs `processNext` (sync lock + diagnostics reset) and `pipeline_.diagnostics()`, which includes a 23×23 eigen solve (R4-04).
3. The single-threaded executor mixes IMU fan-out with large point-cloud conversion and timers (R4-20). The 1 Hz transport snapshot sorts full-run samples under `input_mutex_` (R4-19).
4. In SITL (simultaneous scan), every scan merges and validates 1 s of IMU history (R4-14).
5. The ikd-tree Search can spin-wait (`usleep(1)` loop) on subtrees being rebuilt asynchronously while OpenMP threads query.

## Notes for target architecture

Keep:
- Typed int64-ns `Timestamp` with clock-domain tags and checked arithmetic.
- Explicit `^target T_source` `RigidTransform` naming.
- The transactional predict/correct with rollback.
- Frozen extrinsic and gravity.
- The verified point-to-plane Jacobians and base-link lever-arm and covariance Jacobians, all verified in this review.
- Generation- and sequence-guarded correction hand-off to the propagated worker.
- Bounded ingress with fail-closed load shedding.
- The allocation-free k-NN wrapper.
- Explicit discontinuity events.
- The unknown-parameter rejection.
- The strong unit tests: IKFoM compact-vs-dense equivalence, Jacobian finite differences, propagator replay equivalence.

Fix structurally:
- One owner and one clock for `/lio/health` (R4-06).
- An explicit reset/relocalize path with an epoch bump. Today several latches are terminal: R4-01, R4-12, R4-16.
- One IMU-continuity policy object (R4-13).
- Supervision in the sensor clock (R4-22).
- Frame names either injected into or removed from the parameters (R4-11).
- Move publishing, visibility association and map maintenance off the critical correction path (R4-07, R4-18).
- A separate IMU callback group (R4-20).

Over-engineered or unnecessary:
- Large diagnostic structs copied per IMU sample.
- Duplicated planarity gates (PlaneEstimator ratio vs ResidualGate planarity).
- Three separate gap thresholds.
- `EstimatorConfig normalizeConfig()`, which is a no-op.
- `DeskewMode::kAuto`, which the node rejects.
- The history-merge path for simultaneous scans.
- The dual propagated/scan health producers.
- Two copies of the extrinsic (filter state vs YAML in the output publisher).
- Acceleration is available in the propagator (bias-corrected IMU) but not published, so the runtime re-derives it by finite differences. Publishing it would remove a duplicate derivation.
