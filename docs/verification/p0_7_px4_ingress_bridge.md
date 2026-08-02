# P0.7 PX4 Ingress Bridge — Stable v1.17 Acceptance

## Result

**BLOCKED**

The stable PX4 v1.17 ingress implementation, dependency lock, generated-message
build, focused tests, dataset regression, and live bridge transport are complete.
P0.7 acceptance remains **BLOCKED** by the measured live P0.6 startup-prior
clock-domain mismatch: the topic prior timed out and the configured zero
fallback was used. This report does not convert that observation into a PASS.

## Revision

- Starting commit: `b03c536d673777a1cf71bc73a5e8bece36bffd26`
- Branch: `feat/p0.7-px4-ingress-bridge`
- Required implementation commits: five, listed below.
- P0.8: not started.

## Stable-only scope decision

This task supports only the stable PX4 v1.17 baseline:

- PX4 ref `v1.17.0`, SHA
  `d6f12ad1c4f70ad3230afd7d86e971421e02fef4`.
- `px4_msgs` ref `release/1.17`, SHA
  `86d8239e962f6939e05c3737784f60c02fa884db`.
- No PX4 v1.18, PX4 `main`, beta compatibility matrix, or current support
  claim is included.
- `px4_ros2_interface_lib` is not integrated; it remains future P0.10 scope.
- No PX4 source checkout outside the isolated v1.17 SITL worktree was changed.

The machine-readable lock is
`config/px4/p0_7_compatibility.lock.yaml`.

## In-workspace px4_msgs dependency

`px4_msgs` is imported with vcstool, not copied, vendored, or represented as a
Git submodule.

| Field | Value |
|---|---|
| Dependency manifest | `dependencies/px4/px4_msgs.repos` |
| Workspace path | `src/external/px4_msgs` |
| Remote | `https://github.com/PX4/px4_msgs.git` |
| Expected ref/SHA | `release/1.17` / `86d8239e962f6939e05c3737784f60c02fa884db` |
| Actual SHA | `86d8239e962f6939e05c3737784f60c02fa884db` |
| Working tree | clean |

`make deps-px4-sync` is safe-by-default: it does not fetch when the checkout
is already valid, refuses a dirty or wrong-remote checkout, and never
overwrites local changes. `make deps-px4-verify` validates the remote, SHA,
clean state, and required message files. `src/external/*` is ignored except
for its README, so the checkout remains an explicit workspace dependency.

## Exact v1.17 message audit

The four bridge inputs were compared against the exact PX4 v1.17 definitions.
The definitions match at the locked ROS and PX4 revisions.

| Message | Definition hash | Version | Topic |
|---|---|---:|---|
| `VehicleOdometry` | `cf117ff82cdbf191bf576db91db900b7ce34f6a7` | 0 | `/fmu/out/vehicle_odometry` |
| `VehicleLocalPosition` | `3178a00bb3ce1fba273f8d4fb16db98781dc8d83` | 1 | `/fmu/out/vehicle_local_position_v1` |
| `VehicleAttitude` | `fde3a85546c705676f1ff9be8e241f09af13ef54` | 0 | `/fmu/out/vehicle_attitude` |
| `TimesyncStatus` | `71e84e85ab51178670339a5309da5a908518a1bf` | 0 | `/fmu/out/timesync_status` |

The topic helper appends `_vN` only for a positive generated
`MESSAGE_VERSION`. Unit tests instantiate it with the real generated v1.17
message types. VehicleOdometry is the only state source; no angular-velocity
fallback topic is used.

## Package dependency cleanup

The package layout is:

- `src/external/px4_msgs`
- `src/px4_interface/px4_odometry_bridge`

`px4_odometry_bridge` has a required `<depend>px4_msgs</depend>` and
`find_package(px4_msgs REQUIRED)`. The bridge node and tests therefore cannot
silently build in a reduced mode. The core source list is explicit:

```text
src/frame_converter.cpp
src/reference_point_converter.cpp
src/reset_compensator.cpp
src/odometry_ring_buffer.cpp
src/time_validator.cpp
```

There is no source glob and no optional generated-message path. The standalone
build/test helpers centrally skip only `px4_msgs` and `px4_odometry_bridge`;
the PX4-enabled targets build the same workspace through
`colcon build --packages-up-to px4_odometry_bridge`.

## Propagated-odometry scope correction

Propagated odometry is profile-selectable, not a globally invalid state:

- loader default: `false`;
- disabled profile: accepted, with corrected odometry owning dynamic TF;
- enabled profile: accepted, with propagated odometry owning dynamic TF;
- production `mid360_real` profile: enabled;
- invalid rate, capacity, or history/correction-age relationship: rejected.

Focused parameter-loader tests cover both enabled and disabled valid profiles.
The standalone validation confirms the existing production profile remains
enabled. This preserves the existing corrected/propagated ownership contract
without forcing every profile to publish propagation.

## PX4 v1.17 runtime acceptance

The isolated SITL worktree is:

```text
/home/letandat/Dev/Autopilot-p0.7-v1.17
HEAD d6f12ad1c4f70ad3230afd7d86e971421e02fef4
working tree clean
```

It was built and run with the PX4 v1.17 binary, ROS 2 Jazzy, Gazebo Harmonic,
and a separate `MicroXRCEAgent udp4 -p 8888`. The clean live session was:

```text
.artifacts/simulation/px4-mid360-20260802-223727
```

The bridge observed live PX4 topics:

```text
/fmu/out/vehicle_odometry
/fmu/out/vehicle_local_position_v1
/fmu/out/vehicle_attitude
/fmu/out/timesync_status
```

The bridge published `/px4/odometry_ros` with finite values,
`header.frame_id=odom`, and `child_frame_id=base_link`. Its diagnostics
reported `timesync_seen=true`, `simulation_clock=true`, and the expected
output frames. The bridge publishes no TF.

The live LIO observation showed:

- `/lio/odometry_corrected`: `odom -> base_link`;
- `/lio/registered_points`: frame `odom`;
- `/lio/local_map`: frame `odom`;
- `/tf`: one dynamic owner, `odom -> base_link`, owned by propagated output in
  the enabled SIM profile;
- `/tf_static`: `base_link -> livox_frame` at `[0, 0, 0.28]` and
  `livox_frame -> livox_imu_frame` at `[0.011, 0.02329, -0.04412]`.

The clean SITL observer report recorded zero process crashes and retained one
baseline observer warning:

```text
Finding ID: P0.7-F02
Title: SIM observer finite-point warning
Severity: WARNING
```

The report measured 101 sampled scans, finite ratio
`0.40478515625`, NaN `0`, positive infinity `122917`, negative infinity `202`,
and zero `is_dense` violations. The artifact does not prove a root cause.
The workflow completed and was cleaned up successfully; the condition is
retained for later investigation.

## P0.6 integration

The SIM profile uses the startup-only source:

```text
source: topic
topic: /px4/odometry_ros
```

The bridge publishes the topic and does not continuously correct the LIO
state. The prior is intended to be consumed once during startup, with no
continuous overwrite.

The synthetic P0.6 prior pipeline tests pass, including startup application,
late rejection, and one-time/closed behavior. The live SITL result is not a
PASS:

```text
initial_prior_source=topic
initial_prior_status=closed
initial_prior_applied=true
initial_prior_fallback_applied=true
initial_prior_reason=TOPIC_PRIOR_TIMEOUT_ZERO_FALLBACK
initial_prior_candidate_count=0
initial_prior_wait_timeout_count=1
```

The observed bridge `timestamp_sample` values are PX4 host-epoch samples,
while the LIO/Gazebo `/clock` domain is simulation time. The bridge correctly
converts `timestamp_sample` by multiplying microseconds by 1000 and does not
add a manual TimesyncStatus offset or substitute callback time. Therefore the
measured live P0.6 topic-prior acceptance is **BLOCKED** pending an explicit
clock-domain contract resolution.

## Reset acceptance

The focused bridge suite passes synthetic reset continuity, transition
suppression, and reset-counter handling. The reset path preserves generation
and pose continuity without extrapolation. Live PX4 reset injection was not
performed:

```text
Live reset: NOT DEMONSTRATED
```

This is an observation boundary, not a claim that live reset behavior has
passed.

## Disconnect/restart acceptance

In session
`.artifacts/simulation/px4-mid360-20260802-223406`, only the bridge process
group was stopped. LIO continued publishing corrected odometry and dynamic TF;
bridge output stopped after graph settling. The bridge was then restarted as a
new process and `/px4/odometry_ros` returned with the `odom`/`base_link`
contract. The session was cleaned up with the scoped simulation stop target.

Result: **PASS** for bridge disconnect/restart isolation. The clean session
`223727` subsequently confirmed zero process crashes for the normal workflow.

## Dataset runtime regression

The required AIST replay was run after the configuration change. Artifact:

```text
.artifacts/datasets/aist-mid360-drive/b03c536-replay-1.0x-20260802T152347505176Z
```

Measured acceptance values:

| Metric | Result |
|---|---:|
| IMU received / processed | 55,435 / 55,435 |
| LiDAR received / processed | 2,772 / 2,772 |
| Drop count | 0 |
| Overflow count | 0 |
| Invalid timestamp count | 0 |
| Final input/IMU/LiDAR queues | 0 / 0 / 0 |
| Maximum queue depth | 46 |
| Processing-lag predicate | not triggered |
| Node exit code | 0 |
| Replay exit code | 0 |

The output bag snapshot found `/lio/registered_points` and `/lio/local_map` in
`odom`, corrected odometry in `odom -> base_link`, and the static sensor tree
shown above. Propagated output had 13,774 exact angular-velocity samples with
no interpolation, missing bracket, nonfinite, or timestamp-mismatch events.
Covariance projection failure and output nonfinite/non-PSD counts were zero.

## Validation checklist

- [x] Stable-only v1.17 lock captured
- [x] In-workspace `px4_msgs` manifest captured
- [x] `px4_msgs` exact remote/SHA/clean state verified
- [x] Exact v1.17 message definitions audited
- [x] Required generated messages in CMake/package contract
- [x] Propagated odometry true profile validated
- [x] Propagated odometry false profile validated
- [x] PX4-enabled build passed
- [x] PX4-enabled focused tests passed
- [x] Standalone build passed with PX4 packages excluded centrally
- [x] Standalone tests passed
- [x] `make check` passed
- [x] `make vendor-check` passed
- [x] PX4 v1.17 isolated SITL completed and cleaned up
- [x] Dataset regression passed
- [x] Bridge publishes no TF
- [x] LIO retains one dynamic TF owner
- [x] No continuous PX4 correction path added
- [x] Synthetic reset passed
- [ ] Live P0.6 topic prior accepted before bootstrap
- [ ] Live reset demonstrated

## Frozen baseline findings carried forward

| ID | Area | Severity | Result | Scope |
|---|---|---|---|---|
| P0.0-F01 | Dataset runtime | Failure | Processing-lag predicate was triggered at frozen baseline | Pre-existing; not caused or addressed by P0.7 |
| P0.0-F02 | Simulation observer | Warning | Finite-point warning was triggered at frozen baseline | Pre-existing; retained for later investigation |

## Findings introduced or resolved by P0.7

| ID | Area | Severity | Result | Scope |
|---|---|---|---|---|
| P0.7-F01 | P0.6 live startup prior | Blocker | Topic prior timed out and zero fallback was applied | Requires clock-domain contract resolution before P0.7 PASS |
| P0.7-F02 | Simulation observer | Warning | Finite-point warning triggered during clean SITL | No proven root cause; retained for later investigation |

No PX4 frame, TF, dataset, queue, drop, overflow, covariance, or continuous-
correction regression was introduced by this task.

## Commit set

1. `fix(config): keep propagated odometry profile-selectable`
2. `build(px4): import stable px4_msgs into workspace`
3. `fix(px4): require generated messages for ingress node`
4. `test(px4): validate stable ingress runtime`
5. `docs(verification): complete P0.7 stable acceptance`

## Conclusion

P0.7 status: **BLOCKED**

The stable PX4 v1.17 workspace, generated-message dependency, ingress bridge,
focused validation, dataset regression, and live transport workflow are
reproducible. The live P0.6 topic-prior acceptance remains blocked by the
measured PX4 host-epoch versus Gazebo simulation-time mismatch, and the SIM
finite-point condition remains a warning. Neither observation is hidden or
reclassified as a source-frame regression. P0.8 must wait until the P0.6
clock-domain contract is resolved and the focused SITL acceptance is rerun.
