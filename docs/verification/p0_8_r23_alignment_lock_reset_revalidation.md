# P0.8-R23 — alignment lock, reset semantics, and runtime stability

Date: 2026-08-04  
Verdict: **BLOCKED / PARTIAL — not R23 PASS**

The deterministic lifecycle and generation gates are implemented and pass the
local test suite. Final RelWithDebInfo SITL runs prove LOCKED and the external
fail-closed timestamp behavior, but the 120-second runtime acceptance is not
green: the stationary SITL baseline exceeds FAST-LIO processing lag, and the
supervisor/external runs intermittently reach LOST, reinitialization, queue
overflow, or unaccounted LiDAR drops.

## Provenance

- Repository branch: `fix/odometry-lifecycle-and-alignment`
- Implementation HEAD used by the final runtime matrix:
  `4f75c1a9d381f5dc87c595de77bb2763d133923e`
- PX4 HEAD: `d6f12ad1c4f70ad3230afd7d86e971421e02fef4`
- `px4_msgs` HEAD: `86d8239e962f6939e05c3737784f60c02fa884db`
- Protected `mid360_px4_gazebo.yaml` SHA-256:
  `2690d584703d709e24d7159759ddffd1ab6e5d6eb165870058b98af61c587f6f`
- Root, PX4, and `px4_msgs` were clean before the final matrix.
- No EKF2 fusion parameter or protected YAML change was made.

The report commit itself is documentation-only; the runtime manifests record
the implementation HEAD above.

## Implementation evidence

`AlignmentLifecycleManager` is ROS-independent and keeps candidate and locked
transforms separate. A candidate is accepted only after valid exact-time
evidence, monotonic evidence IDs, the configured novel-pair accumulator,
adjacent translation/yaw bounds, all-pair cluster diameter bounds, and a
four-DOF covariance NIS gate. The configured proof is three candidate
estimates, at least four novel pairs per accepted transition, history capacity
eight, translation step/diameter `0.10/0.15 m`, yaw step/diameter
`0.05/0.10 rad`, and chi-square gate `9.487729`.

`PROVISIONAL` is not comparison-valid, does not authorize correlated external
odometry, and is not used for readiness. `LOCKED` is the only
`alignment_valid_for_comparison` state.

PX4 reset-event, public-frame, and time generations are carried independently.
Compensated reset events preserve the frozen transform and enter revalidation;
public-frame changes clear the transform; time/LIO binding changes clear proof
and revalidate a locked transform. Revalidation requires three fresh exact-time
validations and adapts no transform. Persistent failures invalidate it.

External odometry reports `node_ready`, `transport_ready`, `time_sync_ready`,
authorization, subscription count, timestamp domain, conversion validity,
sample/transport timestamps, reset/frame generation, jump state, and publish
counts. `use_sim_time=false` reports `TIME_DOMAIN_UNRESOLVED` and publishes
nothing. Geometric jumps close publication without redefining frame generation.

## Build and tests

RelWithDebInfo build command:

```text
colcon --log-base log build --base-paths src --build-base build --install-base install --symlink-install --parallel-workers 1 --executor sequential --packages-up-to fast_lio_core fast_lio_ros navigation_interfaces navigation_bringup odometry_supervisor px4_odometry_bridge --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo
```

Result: 11 packages built successfully.

- `fast_lio_core`: 24/24 CTest passed
- `fast_lio_ros`: 11/11 CTest passed
- `odometry_supervisor`: 1/1 CTest target passed; 58 gtests listed
- `px4_odometry_bridge`: 2/2 CTest passed
- `navigation_interfaces`: no tests registered
- `navigation_bringup`: no tests registered
- `make check`: 380 tests, 0 errors, 0 failures, 0 skipped
- `make vendor-check`: 18 files, 2 pinned upstream SHAs, 3 documented patched files
- `git diff --check`: passed

The lifecycle manager adds ten deterministic tests covering stable lock,
unstable/creeping/wrapped candidates, overlap and novel evidence, covariance
NIS, generation/reset behavior, frozen revalidation, failure invalidation, and
the provisional production gate.

## Final implementation-HEAD runtime matrix

Artifacts are under
`.artifacts/verification/p0_8_r23_runtime/`. Every A4/B4/C4/D4 manifest has
`expected_git_sha_matches=true` for the implementation HEAD above. Each full
measurement stage was intended to be 30 s warmup plus 120 s simulated seconds.

| Run | Scenario result | Measurement/runtime evidence |
|---|---|---|
| A4 | FAIL: `MEASUREMENT_CONTRACT_FAILED` | 120.03 s measurement; FAST-LIO navigation/correction valid, but `processing_lag_exceeded=true`, load shedding 3, queue max 188, no drops/overflow. |
| B4 | FAIL: `SUPERVISOR_STATUS_MISSING` at `SUPERVISOR_READY` | Exact-time query counters ended at 13 successes/340 failures; LIO became LOST before alignment lock. No measurement stage. |
| C4 | FAIL: measurement contract | 120.12 s; final supervisor alignment `LOCKED`, revalidation 5 starts/3 successes, reinitialization count 2; FAST-LIO lag true but zero load shedding/drops/overflow. External gate closed with `TIME_DOMAIN_UNRESOLVED`, conversion false, authorization false, published count 0. |
| D4 | FAIL: measurement contract | 120.02 s; final alignment `LOCKED`, revalidation 12 starts/3 successes, count 1; FAST-LIO lag true, load shedding 1, queue max 541, LiDAR drops 7, overflow true. External sim-time mapping valid and 4,170 samples were published while authorization was available; final authorization closed after runtime degradation. This is not EKF2 fusion evidence. |

Additional diagnostic run B3, immediately before the diagnostics-only report
capture commit, reached `LOCKED` with three candidate estimates and three
successful revalidations. C4 and D4 independently reproduce `LOCKED` on the
implementation HEAD, so the lock path is functional but not runtime-stable.

The final supervisor signals show the intended boundary: candidate lock and
revalidation counters are visible, while degraded/LOST state closes comparison
and external authorization. The remaining failure is throughput/scheduling,
not an acceptance-threshold bypass.

## Root-cause isolation

The earliest reproducible divergence is already present in A4, with FAST-LIO
processing lag and load shedding while supervisor and external bridge are off.
B4 adds exact-time service pressure and can fail earlier with LIO LOST and a
large query-failure count. C4 demonstrates that reducing supervisor evaluation
cadence to 10 Hz avoids drops in that run but does not clear processing lag.
D4 adds the external process and shows the worst observed queue/overflow and
LiDAR-drop result. The evidence therefore implicates baseline simulation
throughput/process scheduling first, with supervisor query contention and
external runtime load as additional contributors. It does not justify a claim
of “no regression” or a queue-capacity increase.

## Artifact hashes

SHA-256 for the main machine-readable artifacts:

```text
A4/run.json         d7ecd502ac890abb15e8ac7366a4cb17c1fd42239c7f9aa2a8a47b1acc520d67
A4/measurement.json aafd522d0fc8b049a1054cf1e4263afa0cbf14abd485109a124ad94b5cea386b
B4/run.json         b84bc926437bd698db48b88f4f8ca67676df2f93c602cefd246171e6266d1d97
B4/measurement.json ca3d163bab055381827226140568f3bef7eaac187cebd76878e0b63e9e442356
C4/run.json         b2e93f1a487b0d596f8d41ec0f9c1281e73fc0240165c2aa19a8811b97bcd25a
C4/measurement.json a65790643d23e56437cd744ab3f2daddd523cdb21d272e3521660226482ee313
D4/run.json         f07adb8aadfbaf06fb7b362cbe47d63cf2d6ea3bb68ce1521ff36674c24d334a
D4/measurement.json 2751c7604dd56dba959094c49a1399a1fce6d7ed99876b43b8f1a3a127c64715
```

R23 remains blocked by the acceptance requirements: a clean 120-second
runtime with zero queue overflow/unaccounted drops, no LOST, and stable
comparison monitoring has not been demonstrated.
