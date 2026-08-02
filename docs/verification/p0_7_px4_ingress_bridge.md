# P0.7 PX4 Ingress Bridge

## Result

BLOCKED

The ROS-independent ingress core and the repository-safe build/test path are
implemented. Full P0.7 acceptance remains blocked because the exact locked
`px4_msgs` and `px4-ros2-interface-lib` workspaces are not installed in this
environment, so the generated-message node, stable PX4 SITL, beta compatibility
build, and live transport checks have not been claimed.

Propagated odometry is now a mandatory repository feature: the loader default
is enabled, all canonical estimator profiles enable it, and disabling it is a
configuration error.

## Revision

- Parent: `b39e44dad70362b5c44772979cafa9412bd09afc`
- Branch: `feat/p0.7-px4-ingress-bridge`
- Commits:
  - `65a7abc build(px4): pin ingress compatibility matrix`
  - `c1fb916 feat(px4): convert PX4 odometry to ROS base-link contract`
  - `d210213 feat(px4): preserve reset continuity and buffered prior history`
  - `600cfd8 test(px4): validate stable and beta ingress compatibility`
  - `docs(verification): validate P0.7 PX4 ingress` (this report)
- P0.8: not started.

## Dependency lock

### PX4 v1.17

- Canonical ref: `v1.17.0`
- Canonical SHA: `d6f12ad1c4f70ad3230afd7d86e971421e02fef4`
- Audited local release regression SHA: `6249cc3d892e161f5834d8976e41b5ee27443864`

### PX4 v1.18 beta

- Compatibility release ref/SHA: `release/1.18` /
  `55a549a75abe59aaf1cf21cf32a3b892a88e346c`
- Beta tag ref/SHA: `v1.18.0-beta1` /
  `fd132028513748238be1762e57893bbca9fcf179`
- The exact compatibility checkout is not present locally.

### px4_msgs

- Canonical: `release/1.17` /
  `86d8239e962f6939e05c3737784f60c02fa884db`
- Compatibility: `release/1.18` /
  `598c7aad7b2386f9406ebd2a2f841619fddc3c78`
- Neither exact generated-message workspace is available locally.

### px4-ros2-interface-lib

- Canonical: `release/1.17` /
  `4a3370f084ac6f1ef001a4afa2b007845ffd0837`
- Compatibility: `release/1.18` /
  `c3e410f035806e8c56246708432ded09c976434b`
- Neither exact workspace is available locally. The bridge core does not link
  this library; it is reserved for the explicit PX4-enabled validation path.

### Micro-XRCE-DDS Agent

- Required ref: `v2.4.3`
- Not invoked by the standalone build.

The machine-readable lock is
`config/px4/p0_7_compatibility.lock.yaml`. The existing `/home/letandat/Dev/Autopilot`
checkout was not switched, reset, stashed, or modified.

## Message compatibility audit

The PX4 v1.17 source definitions were audited directly. The ROS-side hashes
cannot be recorded until the exact `px4_msgs` workspaces are supplied.

| Message | PX4 hash | ROS hash | Version | Topic |
|---|---|---|---:|---|
| VehicleOdometry | `cf117ff82cdbf191bf576db91db900b7ce34f6a7` | BLOCKED | 0 | `/fmu/out/vehicle_odometry` |
| VehicleLocalPosition | `3178a00bb3ce1fba273f8d4fb16db98781dc8d83` | BLOCKED | 1 | `/fmu/out/vehicle_local_position_v1` |
| VehicleAttitude | `fde3a85546c705676f1ff9be8e241f09af13ef54` | BLOCKED | 0 | `/fmu/out/vehicle_attitude` |
| TimesyncStatus | `71e84e85ab51178670339a5309da5a908518a1bf` | BLOCKED | 0 | `/fmu/out/timesync_status` |

Topic construction is generic: a compile-time `MESSAGE_VERSION` greater than
zero appends `_vN`; version zero has no suffix. VehicleOdometry is the only
state source. There is no VehicleAngularVelocity fallback.

## Package boundary

The new package is `src/px4_interface/px4_odometry_bridge`. Its core has only
Eigen and ROS-independent data types; PX4 message headers are optional and are
compiled only when an exact `px4_msgs` overlay is supplied. A separate
`src/navigation_interfaces` package owns `SampleOdometryAtTime.srv`.

No PX4 dependency was added to `fast_lio_core` or `fast_lio_ros`. No generated
PX4 message was copied into this repository. The PX4 targets are explicit:
`make px4-ingress-build`, `make px4-ingress-test`, `make px4-ingress-sitl`, and
`make px4-ingress-smoke`.

## Frame conversion

The core uses the audited matrices:

```text
C_ENU_NED = [[0, 1, 0], [1, 0, 0], [0, 0, -1]]
C_FLU_FRD = diag(1, -1, -1)
```

NED pose conversion is `R_odom_base = C_ENU_NED R_NED_FRD C_FRD_FLU` and
`p_odom = C_ENU_NED p_NED`. FRD pose and world-FRD velocity use the explicit
Z-UP/FRD reflection. NED, FRD, and BODY_FRD velocity inputs are handled
separately. Body angular velocity is converted FRD to FLU; invalid or zero
angular velocity suppresses the sample and never falls back to another topic.

The quaternion is treated as Hamilton `[w,x,y,z]`, normalized, rejected when
near zero/non-finite, and sign-continuous against the previous accepted sample.
The bridge publishes `/px4/odometry_ros` with `header.frame_id=odom`,
`child_frame_id=base_link`, and twist in `base_link`. The bridge publishes no
TF.

## Time contract

- Why no manual offset is applied: PX4 `timestamp_sample` is the authoritative
  measurement epoch; a guessed or callback-time offset would silently mix
  clock domains.
- Real mode: use PX4 timestamp samples and expose XRCE/timesync health in
  `/px4/diagnostics`; no offset is added.
- SIM mode: one `simulation_clock` authority is selected by parameter; no
  second offset path is introduced.
- Timestamp equation: `stamp_ns = checked_uint64(timestamp_sample_us * 1000)`.

Zero, overflow, duplicate, regressing, stale, and future timestamps are
rejected. No callback-time substitution is used. Exact ROS message-node time
validation and transport behavior remain BLOCKED until the generated-message
workspace is available.

## Reference-point contract

The core exposes an explicit `source_body_frame`, `output_body_frame`, and
cached `T_base_source`. The point and velocity equations are:

```text
p_OB = p_OS - R_OB r_BS
v_OB = v_OS - R_OB (omega_B x r_BS)
v_B  = R_BO v_OB
```

The canonical SIM source is `base_link`; the real path has no fabricated FCU
lever arm. Static geometry is resolved once.

## Covariance semantic

PX4 marginal position, orientation, and velocity variances are rotated using
the squared rotation coefficients. Unknown/negative entries remain unavailable.
No angular-rate covariance or cross covariance is fabricated. The query service
reports an explicit availability mask.

## Reset semantic audit

- PX4 source paths: `src/modules/ekf2/EKF2Selector.cpp` and
  `src/modules/ekf2/EKF2.cpp` in the audited PX4 v1.17 checkout.
- Delta signs: the selector accepts the published local-position deltas and
  derives a current-minus-previous delta on instance mismatch.
- Quaternion reset multiplication: the selector uses
  `Quatf(attitude.q) * Quatf(previous_attitude.q).inversed()` when deriving a
  reset from an instance mismatch; published `delta_q_reset` is consumed as
  the source-provided reset quaternion.
- Counter rules: an equal counter is continuous; a one-count uint8 modulo
  increment is a reset; any other jump fails closed.

Detailed reset metadata is associated by timestamp, not callback ordering.
The transition sample is suppressed when metadata is missing. Full generated
message audit of all detailed fields remains BLOCKED with the ROS workspace.

## Continuity implementation

`ResetCompensator` computes a world-frame continuity rotation and translation
from the last accepted continuous sample to the post-reset source sample. It
increments a reset generation, suppresses the transition sample, and applies
the transform to subsequent samples. Synthetic counter-wrap, counter-jump,
missing-metadata, and continuity tests are present in the standalone suite.

## Ring buffer and query service

- Ring-buffer duration: 2 seconds.
- Ring-buffer capacity: 512 samples.
- Interpolation: linear position/velocity/angular velocity, quaternion SLERP,
  conservative componentwise maximum covariance.
- Extrapolation: never allowed.
- Generation crossing: never interpolated.
- Query service: `/px4/sample_odometry_at_time`, returning exact/interpolated
  `nav_msgs/Odometry`, reason, and generation/mask metadata.

## P0.6 integration

The service/message boundary is ready for the P0.6 startup prior. The bridge
can be remapped to `/initial_state_prior` for startup only; it does not provide
a continuous PX4 correction stream into LIO. Late prior behavior and
bridge-disconnect isolation require the PX4-enabled node/runtime and are not
claimed here.

## Diagnostics

The node publishes `/px4/diagnostics` with source, output frames, simulation
clock mode, XRCE synchronization parameter, TimesyncStatus presence, and
buffer size. No estimator/registration diagnostic is changed by this bridge.

## Standalone validation

- `make build-safe PACKAGES='navigation_interfaces px4_odometry_bridge'`: PASS
  using `--parallel-workers 1 --executor sequential` and isolated
  `build-safe/install-safe/log-safe` directories.
- `colcon test --packages-select px4_odometry_bridge`: PASS, 5/5 tests.
- `make test`: PASS; repository test suites completed with 12 packages and all
  package/tool tests passing.
- `git diff --check`: PASS.
- `make check`: PASS; 277 tests, zero errors/failures, dataset catalog check
  passed.
- `make vendor-check`: PASS.

## Dataset

Not rerun in this P0.7 implementation pass. The prior AIST evidence remains
the P0.6 baseline evidence; because propagated odometry is now mandatory, a
fresh dataset acceptance run is required before P0.7 can be PASS.

## PX4 v1.17 SITL

BLOCKED. The exact stable generated-message workspace and isolated canonical
PX4 worktree are not available through the explicit PX4 target. No SITL result
is claimed.

## PX4 v1.18 compatibility

BLOCKED. The exact release/beta refs are recorded in the lock but their
generated-message and interface-library workspaces are not installed. No
compile/unit/smoke result is claimed.

## Synthetic reset

PASS for the ROS-independent core tests: transition suppression, continuity,
uint8 wrap arithmetic, jump rejection, and missing metadata fail-closed behavior
are covered.

## Live reset

BLOCKED / NOT DEMONSTRATED. No PX4 transport runtime was started.

## Disconnect/restart

BLOCKED / NOT DEMONSTRATED. The standalone package does not fabricate a live
transport result.

## Carried findings

### P0.0 findings

| ID | Area | Severity | Result | Scope |
|---|---|---|---|---|
| P0.0-F01 | Dataset runtime | Failure | Processing-lag predicate triggered | Pre-existing frozen-baseline finding; not addressed by P0.7 |
| P0.0-F02 | Simulation observer | Warning | Finite-point warning triggered | Pre-existing frozen-baseline finding; not addressed by P0.7 |

### P0.5R findings

| ID | Area | Severity | Result | Scope |
|---|---|---|---|---|
| P0.5R-R01 | Corrected/propagated covariance diagnostics | Observation | Aggregate fields retain last-writer semantics | Not addressed by P0.7 |
| P0.5R-R02 | Output covariance diagnostics | Observation | Output asymmetry is currently classified as non-PSD | Not addressed by P0.7 |
| P0.5R-R03 | Numerical Jacobian tests | Observation | Randomized-seed coverage can be expanded | Not addressed by P0.7 |
| P0.5R-R04 | Angular-rate covariance documentation | Observation | Wording cleanup remains deferred | Not addressed by P0.7 |

### New findings

| ID | Area | Severity | Result | Scope |
|---|---|---|---|---|
| P0.7-B01 | PX4 dependency validation | Blocked | Exact generated-message/interface workspaces unavailable | Must be resolved before P0.7 PASS |
| P0.7-B02 | Dataset acceptance | Blocked | Mandatory propagated-odometry configuration has not had a fresh replay | Must be resolved before P0.7 PASS |

## Files changed

- `config/px4/p0_7_compatibility.lock.yaml`
- `src/navigation_interfaces/`
- `src/px4_interface/px4_odometry_bridge/`
- `tools/runtime/px4_ingress.py`
- `tools/runtime/build.py`
- `Makefile`
- Mandatory propagated-odometry defaults/config/tests under
  `src/navigation_estimator/fast_lio_ros/`
- This verification report.

## Non-goals confirmed

No P0.8 supervisor, P0.9 reinitialization, P0.10 egress, `/fmu/in/*`, PX4
dependency in FAST-LIO packages, handwritten IMU integrator, continuous PX4
overwrite of LIO, TF authority, planner, safety, occupancy, loop closure, or
global localization work was started.

## Acceptance checklist

- [x] Parent exact `b39e44d`
- [x] Separate bridge package
- [x] No PX4 dependency in FAST-LIO packages
- [x] Standalone build works without PX4
- [x] Dependency lock records exact SHAs
- [x] Canonical PX4 v1.17 and compatibility v1.18 refs recorded
- [x] No use of `main`
- [x] Generic topic versioning helper
- [x] VehicleOdometry primary source in node design
- [x] No angular-velocity fallback
- [x] NED/FRD conversion core tests
- [x] Output `odom -> base_link`; bridge emits no TF
- [x] `timestamp_sample` conversion and checked arithmetic
- [x] No manual TimesyncStatus offset addition
- [x] Explicit reference-point and covariance semantics
- [x] Bounded ring buffer, no extrapolation, generation guard
- [x] Query service definition
- [x] Propagated odometry mandatory by default and config validation
- [x] Build-safe sequential workflow
- [x] Standalone focused tests pass
- [x] Repository `make test` passes
- [ ] Exact ROS message hashes match
- [ ] Generated-message node compiles against locked `px4_msgs`
- [ ] PX4 v1.17 stable SITL passes
- [ ] PX4 v1.18 beta compile/unit/smoke passes
- [ ] Fresh mandatory-odometry dataset replay passes
- [ ] Live reset and disconnect/restart demonstrated
- [x] `make check` and `make vendor-check` final gate recorded
- [ ] Working tree final clean
- [x] P0.8 not started

## Final conclusion

P0.7 status: BLOCKED

The repository-safe PX4 ingress core and mandatory propagated-odometry
configuration are implemented and standalone-tested. Full P0.7 acceptance is
blocked by missing exact PX4 generated-message/interface workspaces and the
required fresh runtime validations. P0.8 must not start until those gates are
completed; the recommended next task remains P0.7 dependency/runtime closure,
followed by P0.8 — Odometry supervisor.
