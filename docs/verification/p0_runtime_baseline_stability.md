# P0 Runtime Baseline Stability — Root-Cause Closure

Status: **BLOCKED by the prescribed stop-condition**

This verification was run on branch `perf/p0-runtime-baseline-stability` with
external odometry disabled and PX4 EKF2 external fusion disabled. No estimator
algorithm, protected configuration, public frame contract, or P0.9-A external
contract was changed.

## Final provenance

- Final HEAD: recorded in the final handoff; this report is included in the
  documentation commit at that HEAD.
- Branch: `perf/p0-runtime-baseline-stability`
- Starting product HEAD: `95ac0ac175aa70a72f2a3800766ff823e10c6fdc`
- PX4 checkout: `/home/letandat/Dev/Autopilot-p0.7-v1.17`
- PX4 HEAD: `d6f12ad1c4f70ad3230afd7d86e971421e02fef4`
- `px4_msgs` submodule: `86d8239e962f6939e05c3737784f60c02fa884db`
- Protected YAML SHA256: `2690d584703d709e24d7159759ddffd1ab6e5d6eb165870058b98af61c587f6f`
- External odometry publisher: disabled (`P08_EXTERNAL_ODOMETRY` unset)
- EKF2 external fusion: disabled

Commits made in this branch:

1. `ac048f8 test(runtime): capture baseline diagnostic timeline`
2. `16f135c test(runtime): record harness source provenance`
3. `docs(runtime): record baseline root-cause closure` (exact final SHA is in
   the handoff below)

Both commits are runner/evidence changes. There is no production C++ fix in
this branch.

## Static and focused verification

- `python3 -m py_compile tools/performance/p0_8_sitl_orchestrator.py`: PASS
- `python3 -m unittest tools/tests/test_p0_8_sitl_orchestrator.py`: PASS, 25 tests
- `make check`: PASS, 421 tests, 0 errors, 0 failures, 0 skipped
- `make vendor-check`: PASS
- `git diff --check`: PASS
- Protected YAML hash: unchanged
- R0/R1/R2 were built in separate worktrees with RelWithDebInfo; each build
  completed with 14 packages and no build failure.

The first D3 attempt is retained as a non-canonical harness failure: the
terminal had not sourced `install/setup.bash`, so ROS package imports were
unavailable. It was not counted as a SITL result.

## Stage 1 — reference characterization

All runs used the canonical external-disabled `sitl-off` profile, 30 s warmup,
and 60 s measurement. Cleanup completed in every run.

| Run | Product commit | Lag | Load shedding | Maximum queue | p95 corrected scan end-to-end (us) | IMU/LiDAR drops |
|---|---|---:|---:|---:|---:|---:|
| R0 | `94b6722` | true | 2 | 321 | 36521 | 0 / 0 |
| R1 | `f0dcab2` | true | 2 | 190 | 38249 | 0 / 0 |
| R2 | `95ac0ac` | true | 1 | 170 | 37175 | 0 / 0 |

The failure is present on all three reference commits, including the
pre-P0.9-A baseline. This rejects an incremental P0.9-A regression. R0 was
run before the corrected timeline instrumentation, so its missing diagnostic
time-series file and zero-valued CPU samples are recorded as an observation
limitation, not silently reconstructed.

Artifacts:

- `.artifacts/verification/p0-runtime-baseline/r0-94b6722`
- `.artifacts/verification/p0-runtime-baseline/r1-f0dcab2`
- `.artifacts/verification/p0-runtime-baseline/r2-95ac0ac`

## Stage 2 — current-HEAD repeatability

Three consecutive canonical runs were completed and none was discarded.

| Run | Lag | Load shedding | Maximum queue | p95 corrected scan end-to-end (us) | Final LIO status | Cleanup |
|---|---:|---:|---:|---:|---|---|
| current-01 | true | 0 | 202 | 35041 | TRACKING | PASS |
| current-02 | true | 1 | 192 | 37485 | TRACKING | PASS |
| current-03 | true | 0 | 138 | 40092 | TRACKING | PASS |

All three runs had zero IMU drops, zero LiDAR drops, zero queue overflow, and
zero invalid timestamp rejections. The end-state `TRACKING` and finite
corrected output do not override the latched processing-lag failure. The
three-run result is therefore **0/3 acceptance**, with a strongly repeatable
failure but variable severity.

Artifacts:

- `.artifacts/verification/p0-runtime-baseline/current-01-canonical`
- `.artifacts/verification/p0-runtime-baseline/current-02`
- `.artifacts/verification/p0-runtime-baseline/current-03`

## First causal failure

`current-02` provides the clearest event ordering. Times below are simulation
seconds from the measurement timeline.

| Time | Evidence |
|---:|---|
| 43.964 | Measurement begins. |
| 54.020 | Transport high-water rises to 52, from 18 at the earlier snapshot. |
| 57.752 | Transport high-water rises to 101 while the current queue remains transient; this precedes the lag latch. |
| 58.500 | Estimator reports `LOST`; `ros_maximum_imu_gap_ns=136000000`, exceeding the configured 50 ms recoverable gap. No IMU was dropped. |
| 58.600 | Estimator reports a successful corrected update while still `DEGRADED`; the lifecycle transition is observed, not suppressed. |
| 59.020 | `PROCESSING_LAG_LIMIT_EXCEEDED` is first published; queue high-water is 126. |
| 59.848 | Propagated worker first reports missing-bracket/correction-start drops and increased stale-stop activity. |
| 81.980 | Propagated worker load-shedding count first becomes 1. |

The resource sampler shows a simulator clock delivery burst around the first
failure window: simulation advanced by about 1.584 s in about 1.002 s wall
time (sampled RTF about 1.58), after preceding slower intervals. The same run
reports aggregate clock and IMU topic gaps of 1.036 s and 1.092 s. The
diagnostic samples contain no sensor drop or queue-overflow counter increase.

Before the first lifecycle failure, corrected scan processing was generally
20–36 ms and the worker was making progress. The first propagated load-shed
transition occurs more than 20 s after the estimator loss/lag sequence. This
orders propagated replay and supervisor observation after the source-side
clock/input discontinuity; they are not the primary cause.

## Isolation and category decision

The canonical runs are D2-equivalent: FAST-LIO plus PX4 ingress, with the
supervisor disabled. A non-canonical D3 isolation run enabled the existing
supervisor while keeping external odometry disabled. D3 also failed (`lag=true`,
load shedding 1, maximum queue 179) and the supervisor reported `DIVERGED:
LIO_LOST`; this adds a consequence but does not explain the earlier D2 failure.

| Category | Decision | Evidence |
|---|---|---|
| A — estimator compute overload | Not supported as primary | Queue growth and the 136 ms IMU gap precede the later compute outliers; p95 scan processing remains about 37.5 ms. |
| B — executor/callback starvation | Not isolated as an independent source | Input delivery is bursty, but the available evidence attributes the burst to the simulation clock path. |
| C — lock contention | Not supported | No lock-wait evidence precedes the first queue rise. |
| D — propagated worker interference | Rejected as primary | Missing-bracket/stale-stop activity follows estimator loss; load shedding follows the lag latch. |
| E — diagnostics/publication interference | Not supported | Sampling is bounded; no diagnostic counter or publication event precedes the first discontinuity. |
| F — simulator or clock burst | **Primary** | Clock/IMU gaps and non-uniform simulation-time delivery precede queue growth, lifecycle loss, and lag. |
| G — harness observation defect | Rejected for the current evidence | The corrected event series agrees with product diagnostics and topic observations. R0's known instrumentation limitation is explicitly separated. |
| H — host scheduling/environmental noise | Not primary on current evidence | No unrelated process pressure or RSS growth explains the event; process CPU remains well below whole-host capacity. Host sensitivity remains a variable for the next experiment. |
| I — unknown | Not selected | The timestamped simulator/input discontinuity is directly observed. |

## Why no production patch was made

The prescribed forbidden fixes include raising queue or lag thresholds,
ignoring the observed stale/discontinuous samples, suppressing `LOST` or
`DEGRADED`, reducing sensor rates, and dropping mandatory input. A production
patch that converted this 136 ms source gap into a healthy estimator state
would hide a real timestamp discontinuity and violate the lifecycle and sensor
contracts. No repository boundary has been proven that can repair the
simulator/PX4 clock delivery without changing those semantics.

Per the task stop-condition, this is **BLOCKED**, not PASS and not merge-ready.
There is no post-fix 3×60 s gate and no 120 s gate because no compliant product
fix has been justified.

## Smallest next experiment

Repeat the same external-disabled profile on a controlled simulator/host
condition that records per-message `/clock`, IMU, and LiDAR arrival intervals
at the same timestamps (for example, a host with a controlled Gazebo real-time
factor or a deterministic recorded sensor-clock replay). The experiment must
retain all samples and compare the first discontinuity epoch. If the gap
disappears without changing FAST-LIO, the infrastructure boundary is
confirmed; if it remains, collect per-thread callback/lock timing before
considering any repository fix.

P0.9-B was not started.
