# P0.8 frame/odometry root-cause evidence

Status at this revision: implementation and deterministic tests are in scope;
canonical SITL acceptance remains unclaimed until a fresh runtime artifact has
real samples and a clean pinned provenance.

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

The old Euler-yaw artifact in historical P0.8 evidence is not used as
post-fix acceptance. Current runtime claims require a new probe/SITL artifact.

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
| PX4 converter unit tests | `build/px4_odometry_bridge/test_px4_odometry_bridge --gtest_color=no` | 25/25 passed |
| supervisor/core tests | `build/odometry_supervisor/test_odometry_supervisor --gtest_color=no` | 34/34 passed at last run |
| parameter schema | `build/fast_lio_ros/test_parameter_loader --gtest_color=no` | 19/19 passed |
| Python tool syntax | `python3 -m py_compile ...` | exit 0 |
| canonical SITL | fresh `p0_8_sitl_orchestrator.py` artifact | not yet accepted |

The supervisor status now exposes `alignment_valid`, source, epoch, reset/time
generations, and reinitialization count. The independent probe writes
`discovery.json`, `samples.jsonl`, and `summary.json` under its requested fresh
artifact directory.
