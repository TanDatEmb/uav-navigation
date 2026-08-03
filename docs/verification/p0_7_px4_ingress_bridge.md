# P0.7 PX4 Ingress Bridge — Stable v1.17 Acceptance

## Result

**P0.7 status: PASS**

The stable PX4 v1.17 ingress bridge, the Gazebo-authoritative simulation
clock contract, the P0.6 topic-prior epoch contract, the mandatory propagated
odometry contract, and the required dataset/SITL gates are reproducible. The
SIM finite-point condition remains a retained warning observation; it does not
invalidate P0.7 and no root cause is claimed from the available artifact.

P0.8 was not started.

## Revision

- Starting commit: `6fe8e6c5d354bfe2846af4d7c27e24e289796b87`
- Branch: `feat/p0.7-px4-ingress-bridge`
- Corrective changes are new commits after the starting commit; existing
  history was not amended, pushed, merged, or tagged.
- P0.8: not started.

## Architecture review remediation

The following review decisions are authoritative for P0.7. The previous
report described the dependency as a vcstool manifest and treated propagated
odometry as optional; both statements were corrected by this continuation.

| Review item | Before | After | Result |
|---|---|---|---|
| `px4_msgs` dependency | `.repos` nested checkout workflow | Git submodule | PASS |
| `.repos` workflow | Present in previous report/tooling | Removed from active repository workflow | PASS |
| External workspace vars | Legacy contract audited | Removed | PASS |
| Interface library | Absent | Deferred to P0.10 | PASS |
| PX4 v1.18 | Not in stable acceptance | Future migration branch only | PASS |
| Bridge package | Separate | Preserved | PASS |
| Extra core package | Absent | Not introduced | PASS |
| Frame/time conversion | Existing | Preserved; focused tests pass | PASS |
| Reset compensation | Existing | Preserved; synthetic reset tests pass | PASS |
| Ring buffer | Existing | Preserved and bounded | PASS |
| Query service | Existing timestamp query | Scope frozen | PASS |
| Optional CMake | Previous reduced-mode risk | `find_package(px4_msgs REQUIRED)` | PASS |
| Propagated odometry | Profile-selectable in old report | Mandatory canonical output | PASS |
| Runtime gates | P0.6 clock gate incomplete | Dataset, PX4, SITL, prior, and restart evidence captured | PASS |

### `px4_msgs` submodule

The canonical dependency is:

```text
repository: https://github.com/PX4/px4_msgs.git
path:       src/external/px4_msgs
SHA:        86d8239e962f6939e05c3737784f60c02fa884db
```

`.gitmodules` tracks the path and remote without a tracking branch. The parent
repository tracks a gitlink, the submodule worktree is clean, and
`git submodule update --init --recursive` is the supported fresh-clone path.
The locked message revision is compatible with PX4 v1.17.0
`d6f12ad1c4f70ad3230afd7d86e971421e02fef4` for `VehicleOdometry`,
`VehicleLocalPosition`, `VehicleAttitude`, and `TimesyncStatus`.

### Removed `.repos` and external-workspace workflow

The P0.7-specific `.repos` manifests, the manifest synchronization helper, and
the old dependency README were removed. No active `vcs import` workflow
remains. `PX4_MSGS_WS` and `PX4_ROS2_WS` are not used by Makefiles, runtime
scripts, tests, diagnostics, or this report. `make deps-px4-check` only
verifies the submodule gitlink, exact SHA, official remote, package files, and
clean state; it does not write Git state or manage an external overlay.

The standalone build/test workflow skips the complete PX4 bridge package when
the submodule is unavailable. The PX4-integrated workflow builds the bridge
from `src/external/px4_msgs` in the same colcon workspace.

### Stable v1.17 only and interface-library deferral

P0.7 supports only the isolated PX4 v1.17.0 worktree and the pinned
`px4_msgs` SHA. There is no active PX4 v1.18 compatibility path, release/1.18
profile, or beta matrix. Future migration requires a dedicated branch.

`px4_ros2_interface_lib` and `px4_ros2_cpp` are not imported, linked, or
required. P0.7 uses direct `px4_msgs` subscriptions; evaluation of an
interface library is deferred to P0.10.

### Package, CMake, and query scope

The separate package `src/px4_interface/px4_odometry_bridge` is retained. No
additional `px4_odometry_core`, `px4_ingress_core`, or transport package was
created. Internal conversion, reset, ring-buffer, and ROS-adapter separation
remains inside this package.

`package.xml` declares `<depend>px4_msgs</depend>`, CMake uses
`find_package(px4_msgs REQUIRED)`, and the bridge executable is always built
when the bridge package is selected. A missing submodule is a clear build
error; there is no optional half-package build. The bridge does not publish
TF.

The existing `/px4/sample_odometry_at_time` service is frozen to one timestamp
query with exact/bounded interpolation, failure reason, and generation/validity
metadata. No bulk, range, streaming, persistence, or additional interpolation
API was added.

### Mandatory propagated odometry

`propagated_odometry.enabled` defaults to `true`, all canonical profiles set it
to `true`, and an explicit `false` is rejected as a configuration error. The
corrected topic remains available, while propagated odometry is the canonical
high-rate navigation output and owns the dynamic `odom -> base_link` TF in
the enabled canonical profiles. There is no simultaneous corrected/propagated
TF authority.

## SITL clock-domain contract

### Root-cause capture

The original failing session was:

```text
.artifacts/simulation/px4-mid360-20260803-073142
```

During that session Gazebo/ROS `/clock` was approximately `19.108e9 ns`, while
PX4 `timestamp_sample` was approximately `1.785e18 ns`. The bridge had
converted the PX4 microsecond timestamp correctly, but PX4's default DDS time
sync left the sample in a host-epoch domain. P0.6 therefore timed out and used
zero fallback. No callback arrival time or manual TimesyncStatus offset was
used as a fix.

### Corrected architecture

Gazebo `/clock` is the single simulation-time authority. The corrected launch
path passes `use_sim_time=true` to `px4_odometry_bridge`, `fast_lio`,
`robot_state_publisher`, the sensor-frame path, and the observer. PX4 SITL is
started with:

```text
PX4_PARAM_UXRCE_DDS_SYNCT=0
```

The bridge converts `timestamp_sample` from microseconds to nanoseconds and
does not apply a TimesyncStatus offset or callback-time substitution.

Corrected direct bridge diagnostics from the startup validation include:

```text
px4_timestamp_sample_ns = 55956000000
px4_ros_output_stamp_ns = 55956000000
bridge_node_now_ns = 55236000000
timestamp_rejected_count = 0
conversion_rejected_count = 0
reset_suppressed_count = 0
timesync_seen = false
simulation_clock = true
bridge_use_sim_time = true
```

The `/clock` publisher count was exactly one. PX4, ROS, IMU, and LiDAR
timestamps were observed in the simulation domain for the corrected workflow.

## P0.6 topic-prior acceptance

The SIM profile retains `source: topic` and
`topic: /px4/odometry_ros`. The prior is consumed once at startup and does not
continuously overwrite the estimator.

The final corrected SITL artifact is:

```text
.artifacts/simulation/px4-mid360-20260803-075510
```

Its diagnostics show:

```text
initial_prior_status = closed
initial_prior_source = topic
initial_prior_applied = true
initial_prior_fallback_applied = false
initial_prior_candidate_timestamp_ns = 12700000000
initial_prior_application_timestamp_ns = 12712000000
initial_prior_candidate_age_ns = 12000000
initial_prior_time_delta_ns = 12000000
initial_prior_clock_domain = simulation_time
initial_prior_candidate_count = 0
initial_prior_rejected_count = 0
initial_prior_future_rejected_count = 0
initial_prior_wait_timeout_count = 0
initial_prior_zero_fallback_count = 0
initial_prior_reason = TOPIC_PRIOR_ACCEPTED
```

The focused pipeline tests also prove that a same-domain future candidate is
retained through a wait timeout and accepted when the sensor/application epoch
catches up. It is not discarded, extrapolated, replaced by callback time, or
converted into an early zero fallback.

Result: **PASS** — topic prior applied once, fallback was not applied, and the
bootstrap map remained valid.

## PX4 v1.17 runtime acceptance

The isolated PX4 worktree is:

```text
/home/letandat/Dev/Autopilot-p0.7-v1.17
HEAD d6f12ad1c4f70ad3230afd7d86e971421e02fef4
working tree clean
```

The final clean SITL report is:

```text
.artifacts/simulation/px4-mid360-20260803-075510/REPORT.md
```

Observed final-session stream counts included 12,375 `/clock` messages,
10,481 IMU messages, 524 LiDAR messages, 485 odometry messages, 485
registered-point messages, 49 local-map messages, and 2,528 TF messages. No
process crash occurred. `/clock` had one publisher and observer
`use_sim_time` was true.

The bridge received PX4 `VehicleOdometry` and published finite
`/px4/odometry_ros` with `odom -> base_link`. It published no TF. The actual
dynamic TF authority was `odom -> base_link`; the static tree was:

```text
base_link
└── livox_frame       [0, 0, 0.28] in SIM
    └── livox_imu_frame [0.011, 0.02329, -0.04412]
```

Registered points and local map both used `odom`. Propagated odometry was
present and owned the dynamic TF in the enabled SIM profile. There was no
second dynamic TF authority.

The observer retained this warning:

```text
Finding ID: P0.7-F02
Title: SIM observer finite-point warning
Severity: WARNING
```

Across 52 sampled scans, finite ratio was `0.40478515625`; NaN count was `0`,
positive infinity count `63284`, negative infinity count `104`, and process
crash count `0`. The artifact does not prove a cause. The workflow completed
and cleaned up successfully. The finite-point condition is retained as a
baseline warning for later investigation.

## Dataset gate

Fresh AIST replay artifact:

```text
.artifacts/datasets/aist-mid360-drive/6fe8e6c-replay-1.0x-20260803T005643235351Z
```

| Metric | Result |
|---|---:|
| IMU received / processed | 55,435 / 55,435 |
| LiDAR received / processed | 2,772 / 2,772 |
| Drop count | 0 |
| Overflow count | 0 |
| Invalid timestamp count | 0 |
| Final input/IMU/LiDAR queues | 0 / 0 / 0 |
| Maximum queue depth | 48 |
| Processing-lag predicate | not triggered |
| Estimator exit code | 0 |
| Replay exit code | 0 |
| Propagated odometry | present; 13,775 exact angular-velocity samples |
| Covariance projection failures | 0 |

The output bag contains `/lio/registered_points` and `/lio/local_map` with
`header.frame_id=odom`, corrected and propagated odometry in `odom`, and the
expected static sensor transform. One missing-bracket correction diagnostic
was observed in the summary, but it did not fail the acceptance summary and
the replay/estimator both exited zero.

## Reset, ring-buffer, and disconnect/restart acceptance

Frame/time conversion, reset compensation, ring-buffer ordering/generation,
and the frozen query service remain covered by the existing focused bridge
tests. `make px4-ingress-test` passed all 6/6 bridge tests, including synthetic
reset continuity, transition suppression, counter handling, interpolation,
no extrapolation, and generation-boundary rejection.

Live PX4 reset injection was not performed:

```text
Live reset: NOT DEMONSTRATED
```

This is an evidence boundary, not a failure claim; synthetic reset acceptance
passed.

The isolated disconnect/restart session
`.artifacts/simulation/px4-mid360-20260802-223406` passed: stopping only the
bridge did not stop LIO or TF, bridge output stopped after graph settling, and
restart produced fresh `/px4/odometry_ros` output without stale-buffer reuse.
The current changes preserve that path.

## Validation checklist

- [x] Exact starting HEAD captured
- [x] Working tree clean at task start
- [x] `px4_msgs` submodule exact SHA/remote/clean state verified
- [x] No active `.repos` dependency workflow
- [x] No `vcs import` workflow
- [x] No external PX4 message workspace contract
- [x] No PX4 v1.18 active support
- [x] `px4_msgs` required package/CMake dependency
- [x] Bridge package remains separate; no extra core package
- [x] Explicit CMake source inventory
- [x] Mandatory propagated odometry default/config/tests
- [x] Gazebo is the single `/clock` authority
- [x] Relevant ROS nodes use simulation time
- [x] `UXRCE_DDS_SYNCT=0`
- [x] PX4 v1.17 build passed
- [x] PX4 bridge tests passed
- [x] Full standalone build-safe workflow passed
- [x] Full `make test` passed
- [x] `make check` passed
- [x] `make vendor-check` passed
- [x] Fresh dataset replay passed
- [x] Corrected SITL completed and cleaned up
- [x] P0.6 topic prior applied without fallback
- [x] Disconnect/restart isolation passed
- [x] Synthetic reset passed
- [ ] Live reset demonstrated — not run in P0.7
- [x] P0.8 not started

## Frozen and retained findings

| ID | Area | Severity | Result | Scope |
|---|---|---|---|---|
| P0.0-F01 | Dataset runtime | Failure | Processing-lag predicate triggered at frozen baseline | Pre-existing; not caused or addressed by P0.7 |
| P0.0-F02 | Simulation observer | Warning | Finite-point warning triggered at frozen baseline | Pre-existing; retained for later investigation |
| P0.7-F02 | Simulation observer | Warning | Finite-point warning triggered during corrected SITL | No proven root cause; retained for later investigation |

No P0.7 failure finding remains. The P0.0 dataset failure and SIM warning are
not attributed to the P0.7 frame, time, or ingress changes without a measured
regression against their frozen baselines.

## Architecture remediation final fields

```text
px4_msgs dependency type: Git submodule
px4_msgs submodule path: src/external/px4_msgs
px4_msgs submodule SHA: 86d8239e962f6939e05c3737784f60c02fa884db
px4_msgs submodule clean: yes
.gitmodules: present
.repos manifests remaining: none
vcs import remaining: no
PX4_MSGS_WS remaining: no
PX4_ROS2_WS remaining: no
px4_ros2_interface_lib integrated: no; deferred to P0.10
PX4 v1.18 active support: no
Bridge remains separate package: yes
Additional core package created: no
Frame/time conversion regression: PASS
Reset compensation regression: PASS (synthetic); live reset not demonstrated
Ring-buffer regression: PASS
Query-service scope frozen: yes
px4_msgs CMake mode: REQUIRED
Bridge executable always built: yes
Propagated odometry mandatory: yes
Explicit false rejected: yes
Propagated TF ownership: propagated output owns dynamic odom -> base_link in canonical enabled profiles
Dataset gate: PASS
PX4 build gate: PASS
PX4 SITL gate: PASS
P0.6 topic-prior gate: PASS
Disconnect/restart gate: PASS
Architecture review result: PASS
```

## Conclusion

```text
P0.7 status: PASS

The stable PX4 v1.17 ingress bridge and the Gazebo-authoritative clock-domain
contract are reproducible. The P0.6 topic prior is accepted in the same
simulation epoch without fallback, the dataset gate passes, and the required
architecture remediation is complete. The SIM finite-point condition remains
a baseline warning for later investigation; it is not assigned a root cause
and does not invalidate P0.7. P0.8 was subsequently completed from this
frozen parent; its separate report records the implementation and validation.

## P0.8 addendum

P0.8 result: **PASS**. The detailed validation is in
[`p0_8_odometry_supervisor.md`](p0_8_odometry_supervisor.md). The P0.8 prerequisite hardening and odometry supervisor
were added without rewriting P0.7 history. The P0.7 PX4 v1.17 boundary, clock
authority, frame contract, and propagated-output ownership remain unchanged.
The P0.0 dataset processing-lag failure and retained SIM finite-point warning
remain carried findings and are not attributed to P0.8.
```
