# Agent report — 2026-09-08

## Current small-commit checkpoint — 2026-09-09

The current development checkout is at `d4e34e5f00a7b94ca410dff164a82e213a115fa6`
on `codex/runtime-evidence-for-analysis`; it remains dirty because the larger
planner/replay WIP is intentionally unstaged. The following fixes remain
independently traceable:

- `c5211c4b` — hand off GPS-off local takeoff to Offboard; the default
  GPS+LIO takeoff path is unchanged.
- `39721dfc` — close the `SimplifySFC` consumer-contract defect with its
  focused component regression; status `PROVEN_AND_FIXED`.
- `d4e34e5f` — preserve the hard-deadline budget for mandatory nominal
  feasibility when no certified seed exists. The 80 ms hard deadline,
  finalization reserve, physical certificates, BACKUP and staging contracts
  are unchanged.

These are source/component fixes, not end-to-end Q1 completion evidence. The
SITL evidence below is grouped by profile: default GPS+LIO runs are distinct
from the explicit GPS-off EV diagnostic profile. The GPS-off profile's missing
PX4 Hold interval is an environment limitation of that diagnostic A/B, not a
product defect and not permission to relax Hold validation.

### Latest GPS-off EV diagnostic runs

Both runs use `long_three_pillars_multiwaypoint`, seed `0`, requested `5 m/s`,
effective MAIN `3/2/4`, and planner `10 Hz`. They are infrastructure-valid
diagnostics but `qualification_eligible=false` and mission-incomplete:

- `external-mode-check-20260909T004629-509425` (`cap3`): 8 bundles
  activated, `1/9` waypoint, collision `0`, PVA command `922/922` success
  with `0` failures, executable trajectory `922/922`, measured speed
  p95/max `3.004/3.080 m/s`; verdict `BLOCKED`.
- `external-mode-check-20260909T005203-511728` (`cap5`): 4 bundles
  activated, `1/9` waypoint, collision `0`, PVA command `369` success and
  `379` failure, executable trajectory `368/369`, lidar
  timestamp/freshness/validity violation, measured speed p95/max
  `1.538/2.441 m/s`; verdict `FAIL`.

The cap3 run shows more committed-bundle progress, not Q1 qualification. The
cap5 run is a sensor/validity failure and is not evidence for a 5 m/s envelope
claim. The historical relaxed `1552559` 9/9 completion remains explicitly
`qualification_eligible=false`; the failed repeat remains counter-evidence.

### Open causal frontiers and regression evidence

- Yaw root cause is identified in source/runtime evidence: `YawTrajOpt` uses
  the whole position-horizon duration and one solve-level target yaw, so a
  long bundle can hold the old heading through a corner and turn late. PX4
  follows the published yaw command; no PX4 limit change is justified. A
  bounded path-aware yaw fix and focused regression remain separate work.
- The terminal backlog is separate: the latest terminal bundle crossed the
  existing execution-anchor gate at roughly `0.822/0.870 m` raw/projected
  error; the fresh stopped endpoint was later rejected with
  `known_free=1` but `near_execution=0`. No tracking, lease, timeout or Hold
  gate was relaxed.
- Diagnostic regression evidence R1/R2/R3 is not closed: the snapshot-double,
  setup-failure and off-timed-solve cases still lack a clean independently
  replayable closure. Do not infer closure from the cap3/cap5 runtime rows.

Q1 remains `NOT_READY`; no row in this addendum is a qualification claim.

## Latest exact-source SITL addendum — 2026-09-09 05:4x ICT

The current development HEAD is `bb60914291c2fc05c564a0b43b78d52ef4902177`; its authoritative Release manifest is clean (`git_dirty=false`) and the canonical build is at `/tmp/uav-navigation-q1-bb609142`. The development checkout still contains unrelated unstaged WIP; none of that WIP was used for the runs below.

The exact new off-mode artifacts are:

- [`external-mode-check-20260908T163042-350998`](../../runtime/external-mode-check-20260908T163042-350998/report.json), source `c0f9d372`, requested 5 m/s, accepted `0..2`.
- [`external-mode-check-20260908T171428-387681`](../../runtime/external-mode-check-20260908T171428-387681/report.json), source `8ea66105`, requested 5 m/s, accepted `0`.
- [`external-mode-check-20260908T184002-436911`](../../runtime/external-mode-check-20260908T184002-436911/report.json), source `bb609142`, requested 5 m/s, accepted `0,1`.

The paired retests were then run from clean Release installs, sequentially and with the same scenario/seed:

Manifest provenance: `8ea66105` runs use `/tmp/uav-navigation-q1-8ea/install/.uav_navigation_build_manifest.json`, SHA256 `f367b7fc0906d955959a6415e03eb0c8f4850cac43874cc4d11d133de9c1da84`; `bb609142` runs use `/tmp/uav-navigation-q1-bb609142/install/.uav_navigation_build_manifest.json`, SHA256 `5a65bedd3b22cef15ba63bbe644fa58a1e741247306a09222eb68a5b4aad7aff`. Both are authoritative Release manifests with clean source and pinned submodules; PX4 provenance is `deaff86e` and dirty/project-customized.

| Source / mode | Artifact | Waypoints | Collision | PVA failure | Planning p50/p95/max (ms) | Deadline flags | Measured speed p95 |
|---|---|---|---:|---:|---:|---:|---:|
| `8ea66105` / adaptive | `223501-451718` | 0–4 | 0 | 0 | 40.142 / 41.778 / 43.905 | 0 | 3.0673 |
| `8ea66105` / relaxed | `223719-453312` | 0 | 0 | 0 | 40.245 / 72.950 / 72.950 | 0 | 2.9107 |
| `bb609142` / adaptive | `223834-454788` | 0–1 | 0 | 0 | 25.470 / 42.851 / 43.159 | 0 | 3.0827 |
| `bb609142` / relaxed | `224002-456318` | 0–7 | 0 | 0 | 40.124 / 80.071 / 80.211 | 5 | 3.0722 |

All paired runs are `BLOCKED`, mission completion was not observed, and they are diagnostic-only (`qualification_eligible=false`). The latest relaxed run did not reach waypoint 8 and has a real deadline tail; it is not a PASS.

The exact linked binaries were rechecked after the retests: `8ea66105`
passed `2/2` mandatory-feasibility tests plus `4/4` SimplifySFC contract
tests; `bb609142` passed `3/3` mandatory-feasibility tests plus `4/4`
SimplifySFC contract tests. These focused results do not turn either SITL
matrix into qualification evidence.

### Speed contract clarification

For the three 5 m/s rows, `requested_cruise_speed_mps=5.0` is only the mission request. The exact loaded runtime config has `control_envelope.maximum_velocity_mps=3.0`; mapping logs report `terminal_speed_cap=2.940`, and setpoint p95 is about 2.93–2.94 m/s. The observed measured p95 values (2.6749, 3.0207, 3.0303 m/s) must therefore be reported as achieved speed under an effective 3 m/s MAIN cap, not as proof that a 5 m/s envelope was loaded or tracked.

### Retest conclusion

`bb609142` shows more progress than the earlier `8ea66105` off run, but the paired result does not establish stability: adaptive stopped at waypoint 1, relaxed stopped at waypoint 7, and the relaxed run crossed the 80 ms timing boundary five times. Remaining work is the causal owner analysis for the execution/recovery stop and the separate planner latency tail. No new production patch was made from these runs.

### Replay boundary

These seven runtime reports are sufficient for the requested source/mode SITL
comparison, but they are not a complete offline optimizer replay. The missing
per-solve inputs are the immutable head/tail PVAJ, guide and guide stamps,
corridor planes/mapping, initial duration vector, and full optimizer seed state
for every failed solve. Those fields must be captured by the existing
diagnostic snapshot path after the exact source/build is selected before an
offline numerical root-cause claim is made; no such claim is made from this
matrix.

## Current source

- Repository: `/home/letandat/Dev/uav-navigation`
- Branch: `codex/runtime-evidence-for-analysis`
- HEAD: `bb60914291c2fc05c564a0b43b78d52ef4902177` for the latest exact-source
  retest addendum above; earlier sections retain their historical candidate
  provenance.
- Worktree: dirty; unrelated WIP remains unstaged
- `git diff --check`: PASS

## Focused commits

- `76054330` — preserve mandatory nominal solve budget
- `89f0c5d5` — preserve nominal finalization reserve; this follow-up removes
  the temporary promotion to the overall hard deadline
- `93675faa` — use the canonical ROS environment for SITL startup
- `77534786` — materialize the immutable mapping diagnostic snapshot
- `29732e8c` — require a post-registration, live External Mode readiness event
- `5cb1b2c7` — cover successful, timeout, rejection, API-mismatch and exit
  readiness cases
- `effac366` — make the timeout regression execute the log/liveness check
- `a59c4d9e` — defer the incomplete nominal snapshot identity caller so the
  committed source remains independently buildable

The readiness marker is project-owned and emitted after `NodeWithModeExecutor`
completes registration/API checks and the state-input node is constructed.
`Got RegisterExtComponentReply` is deliberately not treated as readiness.

## Verification

- Runtime Python contract tests: `260/260 PASS`, command:
  `python3 -m unittest discover -s tools/runtime/tests -p 'test_*.py'`,
  main checkout, final test artifact `/home/letandat/Dev/uav-navigation/tmpijybee8a/runtime/sim-new-20260908T122657-203851`
- Mapping direct test: `23/23 PASS`, binary:
  `/home/letandat/Dev/uav-navigation/build/navigation_mapping/test_mapping_world_model`
- Planning backend direct tests: `9/9 PASS` in `/tmp/uav-nav-verify-HNQkWj`,
  using the direct test binaries from that clean verification worktree
- No SITL or flight qualification claim was made.

## Release-build blocker

The fresh clean worktree `/tmp/uav-nav-release-GWyXxy` was built with the
canonical command:
`source /opt/ros/jazzy/setup.bash && python3 tools/runtime/build.py --mode release build`.
The first run at `5cb1b2c7a7fa04697978d3cb054040199ff5b84e` failed because the
committed runtime caller was ahead of its unstaged facade API. The caller was
then removed in `a59c4d9e`; the same dependency-complete worktree was rebuilt
with that exact production diff (the only omitted ancestor change is the
test-only timeout-clock line from `effac366`). The successful build source is
therefore source-equivalent to the production portion of `a59c4d9e`, with
submodules:
`px4_msgs=86d8239e962f6939e05c3737784f60c02fa884db` and
`px4_ros2_interface_lib=4a3370f084ac6f1ef001a4afa2b007845ffd0837`.
The corrected canonical build finished `23 packages` and wrote:
`/tmp/uav-nav-release-GWyXxy/install/.uav_navigation_build_manifest.json`.
The original failed-build log remains:
`/tmp/uav-nav-release-GWyXxy/log/build_2026-09-08_19-18-19/navigation_runtime/stderr.log`.
The corrected build log is under the latest timestamped directory in
`/tmp/uav-nav-release-GWyXxy/log/`. No SITL qualification claim is made.

## Remaining WIP

The remaining unstaged changes are mixed planner route/seed/optimizer work,
nominal snapshot/replay diagnostics, tests, ledger notes, and one untracked
replay source. They are intentionally not committed as a batch because they
do not yet form one independently proven owner/fix.

## Tracking experiment SITL campaign — 2026-09-08 evening

The source/build candidate for this campaign is isolated from the dirty main
checkout: commit `c0f9d3727cb4c64750b448592f4e5a11cd65470a`, cached Release
checkout `/tmp/uav-navigation-phase-release-1358e834`, manifest
`489b31b222e31895f2013b00b0e086f0829dba269d4be1487f22a9e45565f9f4`,
`VALID`, `git_dirty=false`. Four small source commits remain independently
traceable: `5986310c`, `de345568`, `e9c6bf26`, `b5d0d8c0`, followed by the
provenance hardening `c0f9d372`.

Runs and artifacts are recorded in `SITL_CAMPAIGN_REPORT.md` and
`SITL_MATRIX.md`. The essential result is:

- relaxed 3 m/s: all 9 waypoints and `COMPLETE`, collision 0, PVA failures 0,
  but `qualification_eligible=false` and 3798 experimental tracking suppressions;
- adaptive 3 m/s: `PAUSED_SAFETY_STOP` at waypoint 0, collision 0, PVA failures 0;
- off 3 m/s: `PAUSED_SAFETY_STOP` at waypoint 0, collision 0, PVA failures 0;
- relaxed 4 m/s: `PAUSED_SAFETY_STOP` at waypoint 0, collision 0, PVA failures 0.

No new production behavior was patched from these runs. Q1 remains `NOT_READY`.
The dirty main WIP listed above remains untouched and unstaged.

## Tracking experiment repeat and causal frontier — 2026-09-08 16:09 local

The required relaxed 3 m/s repeat used the same clean candidate, scenario,
seed and `MAIN 3/2/4` contract:

- artifact: `external-mode-check-20260908T160924-345641`
- verdict/outcome: `BLOCKED` / `PAUSED_SAFETY_STOP`
- accepted waypoints: `0..5`; mission incomplete
- collision: `0`; minimum clearance: `0.9367816 m`
- PVA command failures: `277`; safety stop: `true`
- planning total p50/p95/p99/max: `40.109 / 42.728 / 44.816 / 48.861 ms`
- final cumulative `experimental_tracking_suppressed`: `3497`

The repeat's terminal causal log is a strict BACKUP tracking-envelope failure,
not a MAIN tracking bypass result:

```text
reverse=0.770/0.750 m
measured_enu=[112.940,5.336,2.996]
command_enu=[112.193,5.095,3.002]
message_id=9121 generation=16 role=BACKUP
PVA command anchor is not near vehicle; safety hold then handover to PX4 Hold
```

The relaxed experiment suppresses only eligible MAIN tracking gates. BACKUP,
EMERGENCY and safety-suffix validation remain strict. Thus the earlier relaxed
3 m/s completion is non-repeatable diagnostic evidence and does not qualify
Q1.

The paired adaptive/relaxed recovery traces separate a later recovery blocker
from the initial recoverable nominal failure. Initial `main_minco` failures
still had retained command authority and/or a selected certified BACKUP. The
later PlanFromRest backup attempts were rejected at `aligned_sfc` before
visibility-hull, aligned-hull or known-free checks:

```text
visibility_hull_pass_count=0
aligned_sfc_built_count=0
known_free_check_count=0
```

The adaptive run was not hard-deadline limited at that first rejection; the
relaxed 4 m/s run also showed a later deadline overrun. This is the current
diagnostic owner candidate `BACKUP_ALIGNMENT_SFC_FEASIBILITY`, not yet a
proven production root cause. No source patch, threshold change, PX4 change,
or recovery-policy change was made.

Campaign verdict remains `Q1_NOT_READY`. The next source investigation should
use the exact backup aligned-SFC rejection/feasibility data; do not treat the
`main_minco` label or the cumulative suppression counter as a root cause.

## Repeat causal timeline — MAIN to BACKUP to fail-closed

The repeat artifact was traced at record/sample level; the 277 PVA failures are
one contiguous rejected-command incident, not 277 independent tracking
discontinuities.

- Generation `16` was activated at simulation time `162.212 s`. The last
  generation-15 MAIN sample was `8788` at `162.212 s`; the first generation-16
  MAIN sample was `8789` at `162.228 s`. Both retained waypoint `6`, request
  `7`, goal epoch `8`; no lease/freshness break was observed.
- Generation-16 remained MAIN through sample `9082` at `166.916 s`. The next
  sample, `9083` at `166.932 s`, was the normal analytic MAIN-to-BACKUP
  boundary for generation `16`. Runtime state changed from `kTrackMain` to
  `kTrackBackup`; `active_backup_available=1`, source age was `0`, receive age
  was about `1.5 ms`, and `lease_failed=0`.
- The generation-72..78 MAIN renewal failures were recoverable: the active
  generation-16 command remained usable, with no deadline or freshness failure
  causing the transition.
- The first strict BACKUP envelope failure occurred at trajectory time
  `167.524 s` (ROS time `1788883978.773325166`), about `0.592 s` after the
  role transition. It was generation `16`, role `BACKUP`, message `9121`:
  `reverse=0.770/0.750 m`; measured position was
  `[112.940,5.336,2.996]` and command position was `[112.193,5.095,3.002]`.
  Odometry source/receive ages were `12 ms`.
- Strict BACKUP failures continued for message IDs `9122..9135`. The first
  invalid command was sample `9136` at `167.780 s`; PVA validation then failed
  contiguously through sample `9412` at `172.196 s`, exactly 277 samples.
  Runtime subsequently logged immutable generation-16 revalidation failure and
  published rejected commands with no sampled bundle or generation before PX4
  Hold.

Current causal chain:

`recoverable MAIN renewal failures -> normal MAIN/BACKUP seam -> strict
BACKUP tracking-envelope exhaustion -> failed immutable revalidation and
rejected-command stream -> PX4 Hold`.

This identifies the first fatal runtime boundary more precisely, but does not
yet prove whether the BACKUP envelope failure belongs to reference/trajectory
semantics, estimator alignment, or physical tracking. No production patch was
made.
