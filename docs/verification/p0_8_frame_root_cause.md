# P0.8 frame/odometry root-cause evidence

Status at this revision: the frame-contract implementation and real-motion SITL
runtime have machine-readable evidence. The historical three-pair A/B report is
not accepted as a whole because its queue-overhead gate is false; that result is
preserved rather than hidden.

## Executive conclusion

The defect was a contract error at the PX4/LIO boundary. PX4 output was
published on `/px4/odometry_ros` with `header.frame_id=odom`, while FAST-LIO
also published `header.frame_id=odom`. The shared spelling asserted a common
world frame that did not exist. The supervisor then compared the two states
directly. This allowed an origin/convention mismatch to appear as a residual;
changing yaw signs or thresholds would only hide it.

## Machine-readable reproduction

The red reproduction was executed before the corrective production changes:

```text
bridge_publishes_project_odom=true
lio_publishes_project_odom=true
supervisor_directly_compares_same_named_frames=true
px4_frd_has_arbitrary_heading_offset=true
frd_world_conversion_only_reflects_axes=true
REPRODUCTION FAILURE: PX4 FRD/local output is relabeled as project odom and directly compared to LIO odom without alignment
exit=1 (expected red)
```

The first regression test was committed separately as
`b8f4df2 test(frames): reproduce PX4 and LIO world-frame mismatch`.

## Confirmed and rejected hypotheses

Confirmed:

- the PX4 bridge lacked a distinct output frame/topic contract;
- the LIO profile used the same generic `odom` label;
- the supervisor had no captured `^lio_odom T_px4_odom` record;
- FRD/local PX4 world data was not equivalent to an arbitrary ROS ENU/global
  frame merely because both used a three-vector position.

Not selected as root cause:

- quaternion sign: the converter has sign-continuity coverage and equivalent
  `q`/`-q` tests;
- timestamp drift: the bridge and supervisor still require exact service
  epoch equality and generation equality;
- LiDAR/IMU extrinsic: no extrinsic or planner/safety code was changed;
- yaw offset or sign hack: no constant correction was added;
- threshold relaxation: supervisor thresholds were not changed.

The motion replay then isolated a second, runtime-only defect in reset
continuity. `ResetCompensator` applied the accumulated inverse PX4 reset to the
quaternion but left `position` and `velocity_world` in the post-reset world
basis. That produced a non-rigid odometry sample: the bridge orientation was
continuous while its position still followed the unrotated NED-to-ENU axes. In
the failing translation run this made LIO converge to `[-N, E]` while PX4 ROS
reported `[E, N]`; stationary tests could not expose this invariant violation.
The fix applies the same accumulated transform to every world-expressed pose
component and applies reset deltas in that continuous basis.

The old Euler-yaw artifact in historical P0.8 evidence is not used as
post-fix acceptance. Current runtime claims require a new probe/SITL artifact.

The first clean runtime attempt also exposed a second boundary bug: FAST-LIO
publishes estimator, transport, and propagated-worker diagnostics as separate
`DiagnosticArray` messages. The supervisor had been replacing its estimator
snapshot with arrays that did not contain the estimator status, producing a
false `LIO_DIAGNOSTIC_SCHEMA_MISMATCH`. The subscriber now updates a snapshot
only when the named status is present.

## Initial-state prior runtime root cause

The reported `initial-state prior rejected` failure was not one defect. The
first source-clean runtime accepted the topic prior, then kept the PX4
odometry subscription alive after the one-shot startup gate had closed. That
caused one late callback per second to enter the core and emit the same
`gate is closed` warning. In
`.artifacts/simulation/px4-mid360-20260803-231947/logs/ros/fast_lio_node_196005_1785774001437.log`
there were 77 such warnings, while the JSONL diagnostic still reported
`TOPIC_PRIOR_ACCEPTED` and estimator `TRACKING`.

The boundary fix is now explicit: create the topic subscription only for the
topic source, use an ignore-after-gate submission policy for the ROS callback,
copy mailbox counters into every result, publish the accepted/late/frame/value
counters, and remove the subscription once the startup prior or configured
fallback has been applied. The focused runtime at
`.artifacts/verification/p0.8-motion/20260803-233547-yaw-initial-prior-fix.jsonl`
reported `TOPIC_PRIOR_ACCEPTED`, `candidate_count=101`, no warning lines, and
only the single stream-close info line; the remaining one late counter in that
intermediate run exposed the callback/gate race and is handled by the
ignore-after-gate policy in the current source.

A separate startup race was then isolated: `topic_wait_timeout_ns` is measured
in simulation time, while DDS discovery is wall-time. With accelerated Gazebo,
the worker could process enough sensor time to apply
`TOPIC_PRIOR_TIMEOUT_ZERO_FALLBACK` before the volatile
`/px4/estimator_odometry` subscription had matched. The affected runs had
`candidate_count=0`; they are not evidence of a frame rejection. The current
ROS boundary starts the estimator worker only after the parsed topic
subscription reports a publisher match, so the core timeout cannot run before
the transport contract exists. Runs that failed before that startup gate are
kept as harness failures, not counted as estimator passes.

## Architecture decision

Option A was selected: keep PX4 and LIO world frames distinct and add one
explicit startup alignment record. PX4 is converted only according to its
declared `pose_frame`/`velocity_frame`; the FRD/local convention is retained
in diagnostics. FAST-LIO accepts the PX4 prior only for the declared
`ground_startup/startup_coincident` transform. The supervisor applies the
captured record before residual calculation and invalidates it on time/reset
generation changes.

Option B, a global conversion into one guessed ENU frame, was rejected because
PX4 FRD local has an arbitrary local heading/origin and therefore needs an
explicit origin/heading contract that this milestone does not possess.

## Evidence gates

| Gate | Evidence | Result |
| --- | --- | --- |
| PX4 converter unit tests | `build/px4_odometry_bridge/test_px4_odometry_bridge --gtest_color=no` | 26/26 passed |
| supervisor/core tests | `build/odometry_supervisor/test_odometry_supervisor --gtest_color=no` | 35/35 passed |
| parameter schema | `build/fast_lio_ros/test_parameter_loader --gtest_color=no` | 20/20 passed |
| Python tool syntax | `python3 -m py_compile ...` | exit 0 |
| canonical smoke-on | `smoke-on-20260803f/run.json` | outcome `PASS`, 70/70 healthy, comparison and monitoring ratios 1.0 |
| extended SITL A/B | `sitl-ab-20260803.json` | `pass=false`: p95 gate true, queue gate false (median 41 -> 63) |

The selected A/B evidence uses three clean OFF/ON pairs with the same pinned
PX4, px4_msgs, config hashes, and qualification harness SHA. A separate ON
attempt failed closed on processing lag (`sitl-on-20260803-3`) and the retry
passed; both attempts remain referenced in the artifact.

The supervisor status now exposes `alignment_valid`, source, epoch, reset/time
generations, and reinitialization count. The independent probe writes
`discovery.json`, `samples.jsonl`, and `summary.json` under its requested fresh
artifact directory.

## Motion runtime evidence

The pre-fix motion artifact reproduced the user-visible failure without a
stationary shortcut:

```text
artifact=.artifacts/verification/p0.8-motion/20260803-225124-full-it6.jsonl
takeoff/yaw estimator status=TRACKING before horizontal motion
translation PX4 ROS position ~= [1.0, 2.0, 2.0]
translation LIO position ~= [-2.0, 1.0, 2.0]
supervisor=DIVERGED:RESIDUAL_DIVERGED
```

After the SE(3) reset-continuity fix and the motion convergence budget was set
to ten iterations, a source-clean full scenario completed 60.032 s through
ground, takeoff, yaw +90, XY translation, yaw -90, and landing:

```text
artifact=.artifacts/verification/p0.8-motion/20260803-231947-full-reset-se3-it10-clean.jsonl
session=px4-mid360-20260803-231947; dirty=false
estimator=556/556 TRACKING; correction_failure_count=0
supervisor=1199/1199 HEALTHY; final=PX4_RESET_GRACE; comparison_valid=true
supervisor_position_error_max=0.215323 m; orientation_error_max=0.024404 rad
comparison_valid=1177/1199; processing_us=p50 30463; p95 47338; max 75447
queue_high_water=15/16; ROS IMU gap max=8000000 ns
bridge conversion_rejected=0; timestamp_rejected=0; reset_generation=2
pointcloud=554 scans, all frame_id=livox_frame, no dense-cloud violation
vehicle=OFFBOARD+armed samples, failsafe=false; mode/arm ACK result=0
```

The motion driver is `tools/diagnostics/sitl_motion_scenario.py`; it records
native PX4 timestamps, raw and converted odometry, LIO corrected/propagated
odometry, reset metadata, point-cloud quality, diagnostics, command ACKs and
phase events in one JSONL artifact.

A repeat on the final source revision also completed the motion phases and did
not reproduce the frame/odometry divergence, but it is not a qualification
result because the host run accumulated transport backlog:

```text
artifact=.artifacts/verification/p0.8-motion/20260803-232450-full-reset-se3-it10-final.jsonl
source_revision=9c193a36300c123759ebd32bb715093219cb769f; dirty=false
estimator=548/548 TRACKING; supervisor=1183/1183 HEALTHY
correction_failure_count=0; conversion_rejected=0; timestamp_rejected=1
processing_us=p50 29319; p95 43745; max 57383
queue_high_water=66/16; ROS IMU gap max=260000000 ns
```

The two clean runs therefore separate the fixed frame invariant from a
remaining host/DDS-startup performance repeatability issue. The latter keeps
the canonical SITL qualification gate closed until it is reproduced under a
controlled host load or otherwise explained; it is not hidden by selecting
only the lower-backlog run.
