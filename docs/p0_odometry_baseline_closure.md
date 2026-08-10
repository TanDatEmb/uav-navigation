# P0 odometry baseline closure

Date: 2026-08-10

## Tested source

- Navigation HEAD: `bc4eede67697407863f5ad9fe8ff3e7d49a23fed`
- PX4 SITL: `f65ddce53985f13cd4fa24e7c0d78345b467bf1f`
- Dataset: `aist-mid360-drive`, prepared outside the repository under
  `UAV_NAV_DATA_HOME`
- Real Mid-360 and physical PX4 validation: deferred because hardware is not
  complete.

The source HEAD was tested with the local contract-test/comment changes in
this closure worktree. The pre-existing `SIM_GZ_EN_ODOM=0` launcher change
was retained and is not part of the closure commit.

## Verification

| Check | Result | Evidence |
|---|---|---|
| `make build` | PASS | 12 packages built |
| `make test` | PASS | all package tests and 6 data/runtime + 23 runtime-contract tests |
| `make dataset-check DATASET=aist-mid360-drive RATE=1.0` | PASS | `.artifacts/runtime/dataset-20260810T010443-25093/REPORT.md` |
| `make sim-check` run 1 | PASS | `.artifacts/runtime/sim-check-20260810T004323-12152/REPORT.md` |
| `make sim-check` run 2 | PASS | `.artifacts/runtime/sim-check-20260810T004448-13667/REPORT.md` |
| `make sim-check` run 3 | PASS | `.artifacts/runtime/sim-check-20260810T004615-14984/REPORT.md` |

### Dataset acceptance

- LIO reached and remained `TRACKING`; `navigation_valid=true`.
- Corrected odometry: 2,756 finite samples at 10.000 Hz; timestamp
  regressions and duplicates: 0.
- Propagated odometry: 13,773 finite samples at 49.9996 Hz; timestamp
  regressions and duplicates: 0.
- IMU: 55,435 samples at 200.003 Hz; drops, regressions and stale events: 0.
- LiDAR: 2,742 samples at 9.892 Hz; drops, regressions and stale events: 0.
- Correction accepted/rejected: 2,756/0; unexplained LOST/DEGRADED: none;
  queue maximum: 32; queue overflow count: 0; cleanup: PASS.
- Local-map maintenance reached the configured 100k ceiling and pruned as
  designed: 79 hard-limit triggers, 25 prune events, 0 recovery failures.

### SITL acceptance

All three consecutive runs completed:

`GROUND → TAKEOFF → HOVER → TRANSLATE_X → HOVER → TRANSLATE_Y → HOVER → YAW → RETURN → LAND → DISARM`

Each run recorded OFFBOARD entry, takeoff, landing and disarm success,
`failures=[]`, zero OFFBOARD losses, LIO `TRACKING`, valid external odometry,
zero conversion-contract violations, zero stream regressions/drops/queue
overflow events, and cleanup PASS. The scenario report records
`disarm_forced=true` after the landing path while also recording
`disarm_successful=true`.

## Closure decision

The estimation/PX4 odometry baseline is closed and frozen. No FAST-LIO,
IKFoM, ikd-tree, deskew, odometry, frame-conversion, bridge, lifecycle, or
runtime architecture changes were made. The only source updates are stale
local-map comments and test-contract synchronization with the already-active
runtime values (`target=70k`, `soft=80k`, `hard=100k`).

Real-hardware validation remains deferred. The next development stage is the
local voxel world model on branch `feat/local-voxel-map`.
