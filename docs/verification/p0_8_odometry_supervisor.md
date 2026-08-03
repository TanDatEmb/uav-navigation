# P0.8 Odometry Supervisor — acceptance hardening

## Historical result before final performance qualification

```text
P0.8 status: BLOCKED
```

## Corrective task: asynchronous comparison lifecycle and probe accounting

The startup recovery in commit `35c8637` proved that PX4, XRCE, topic
versioning, clock policy, bridge conversion, and the P0.6 prior were not the
cause of the comparison failure. The next focused correction therefore stayed
inside the supervisor alignment coordinator and qualification probe.

### P0.8-R18 — completed comparison invalidated by a newer epoch

The previous predicate rejected a completed pair whenever a newer propagated
epoch became eligible, even when the completed pair was still within its
150-ms freshness window. At the 50-Hz propagated / 20-Hz evaluation cadence,
that made a successful asynchronous pair stale before the next evaluation.

The corrective implementation now:

- defines comparison freshness by completed-pair age, not newest-epoch
  availability;
- retains the last good pair while a newer query is pending or times out;
- preserves immediate invalidation for timestamp mismatch, invalid component
  contract, generation mismatch, and explicit time-generation transitions;
- exposes latest eligible epoch, comparison lag, pending query epoch, and
  pending query age in supervisor diagnostics;
- drives evaluation with the ROS node clock while retaining steady-clock RPC
  timeout measurement.

The state-machine regression test confirms that a held comparison remains
valid and does not advance residual persistence when a newer eligible epoch is
present.

### P0.8-R19 — probe under-sampled status events and used cumulative counters

The probe now accounts for every supervisor status callback in the measurement
window. The 1-Hz timer remains reserved for resource and runtime samples. It
also records reason/health histograms and the longest continuous invalid
window. Query, failure, timeout, generation, state-transition, and
reinitialization fields are reported as end-minus-start deltas after warm-up.

The ROS-independent accumulator tests cover 396 valid events out of 400,
warm-up timeout exclusion, and invalid-window duration. This prevents a
single 1-Hz snapshot or a warm-up counter from deciding the full acceptance
result.

### P0.8-R20 — wall-time evaluation cadence mixed with simulation time

Evaluation now uses the ROS clock timer. The asynchronous service timeout
continues to use `steady_clock`, so RPC timeout behavior remains independent
of simulation clock pauses while state freshness and persistence remain in the
simulation-time domain.

### P0.8-R21 — yaw residual used an Euler extraction with the wrong semantics

The residual calculator previously obtained yaw from
`rotation.eulerAngles(0, 1, 2).z()`. That extraction is not the P0.6 heading
definition and can differ by approximately pi near the active roll/pitch
configuration even when the full quaternion error is small.

The implementation now defines the yaw residual from the projected body-X
heading:

```text
heading(R) = atan2(R(1,0), R(0,0))
```

The comparison is rejected when either horizontal projection norm is below
`1e-6`, so an unobservable heading cannot silently pass the supervisor gate.
The residual also retains the former Euler yaw, quaternion error axis, full
orientation angle, `body_z_dot`, both projected-heading values, and both
horizontal norms for provenance only. The supervisor diagnostic publishes
these values, while the qualification probe records them only at
`prior_applied` and supervisor state-transition events together with raw PX4,
bridge, corrected, propagated, and LIO/PX4 diagnostic snapshots.

Focused coverage now includes tilted states with equal projected heading,
the unobservable vertical body-X case, quaternion error-axis provenance, and
the existing wrap/sign contracts. The focused supervisor package passes all
31 tests; `make check` passes with 334 tests and the dataset guard; vendor
freeze validation also passes.

## Corrective ON smoke

The corrected binary was rebuilt in the normal sourced workspace and exercised
with the required 10-s warm-up plus 20-s measurement. Artifact:

```text
.artifacts/verification/p0.8-sitl-corrective/smoke2/on-smoke.json
.artifacts/verification/p0.8-sitl-corrective/smoke2/px4-mid360-20260803-144936/
```

The comparison lifecycle and probe accounting now pass their specific gates:

| Metric | Result |
|---|---:|
| status events | `400` |
| comparison-valid events / ratio | `400 / 1.0` |
| monitoring-available events / ratio | `400 / 1.0` |
| query count delta | `400` |
| query success delta | `400` |
| query failure delta | `0` |
| query timeout delta | `0` |
| generation mismatch delta | `0` |
| reinitialization delta | `0` |
| aligned-comparison age p99 | `80 ms` |
| alignment gap p99 | `0 ms` |

The smoke nevertheless remained blocked by a separate runtime residual result:
after warm-up the supervisor was `DIVERGED: RESIDUAL_DIVERGED`, with
`comparison_valid=true`, full quaternion orientation error `0.000807 rad`,
and Euler yaw residual approximately `-3.141113 rad`. This classifies the
finding as the yaw-extraction artifact addressed by P0.8-R21, rather than a
measured pi-radian full-orientation mismatch. The `smoke2` binary predates
R21 and is therefore classification evidence only, not post-fix acceptance.

The corrected pre-R21 smoke therefore demonstrates that P0.8-R18, P0.8-R19,
and P0.8-R20 are addressed and identifies the R21 metric defect. The required
post-R21 ON smoke has not completed: one attempt received no ROS samples from
the `/clock`/odometry transport, and a second attempt failed before FAST-LIO
startup when `/px4/odometry_ros` did not become available. Both sessions were
cleaned up and excluded from acceptance. The six-run A/B qualification and
1,200-simulated-second memory run remain `NOT RUN`.

R21 implementation status is `PASS`; R21 runtime acceptance is `BLOCKED` until
the pinned SITL session produces a valid post-fix measurement artifact.

## Corrective-task conclusion

```text
P0.8 status: BLOCKED

The asynchronous comparison lifecycle, probe event accounting, and R21 yaw
semantics correction are implemented and tested. Existing smoke evidence
shows sustained comparison_valid=1.0, monitoring_available=1.0, and zero
measurement-window query timeouts; it also proved the old pi-rad value was an
Euler extraction artifact because the full quaternion error was only
`0.000807 rad`. The post-R21 ON smoke is still blocked by ROS/SITL transport
startup in this environment, so no post-fix runtime health claim is made.

Do not start the full A/B or memory qualification until the post-R21 ON smoke
produces a valid artifact and remains HEALTHY for the complete measurement
window.
```

## Final performance qualification

This section supersedes the earlier dataset-performance paragraph for the
final qualification. The earlier `978c65b` artifacts remain historical only;
they are not final-head evidence.

### Provenance correction

- Required start: `b23c706cbefa7b3ebff98c356686cacbe0a34a30`.
- Branch: `fix/p0.8-performance-qualification`.
- Immediate P0.7 base code: `d6bd7f0349d90167a26bf3710e65341f8b469404`.
- Common diagnostics instrumentation: `568e0ea`.
- Instrumented baseline: `1529e69d8aef09da29eec8dd782dc5af441dab45`.
- Final candidate runtime SHA used for canonical A/B:
  `8cac3ce2fe988051b256f32e090b1ac7f1916fc4`.
- PX4 message submodule: `86d8239e962f6939e05c3737784f60c02fa884db`, clean.
- Qualification runner resource-parsing SHA: `679efd4c543afb78ff6ec3fdc57964e9260849a1`.

Every dataset run recorded full Git SHA, branch, dirty state, submodule state,
binary SHA-256, configuration SHA-256, compiler, host, dataset, rate, process
sampler, and `/usr/bin/time -v` output. A run fails closed on dirty state, SHA
mismatch, missing binary/configuration, or an existing output directory.

Artifacts:

```text
.artifacts/verification/p0.8-performance/final/
```

### Latency metric correction

The final diagnostics use bounded full-run populations, not a mean over the
full run combined with a rolling last-1024 percentile. The measured fields are
`pipeline_push_lidar`, `result_processing`, `corrected_scan_end_to_end`, and
`registration_update`, each with count, mean, p50, p95, p99, and maximum. The
legacy `scan_processing` field is not the sole gate.

### Dataset A/B methodology and correctness

Three baseline and three candidate runs used the same AIST bag, configuration,
serial constrained build policy, and host condition group at `1.0x`. Baseline
and candidate runs passed correctness: `55,435` IMU and `2,772` LiDAR received
and processed, zero drops/overflow/invalid timestamps, zero final queues,
zero non-finite output, and estimator/replay exit `0/0`.

### Dataset latency and resource results

Medians over three complete runs:

| Metric | Baseline | Candidate | Absolute delta | Relative delta |
|---|---:|---:|---:|---:|
| Corrected scan end-to-end p95 | 82,543 µs | 82,677 µs | +134 µs | +0.162% |
| Corrected scan end-to-end p99 | 119,634 µs | 119,768 µs | +134 µs | +0.112% |
| Registration update p95 | 124,258 µs | 124,101 µs | -157 µs | -0.126% |
| Pipeline push LiDAR p95 | 46 µs | 46 µs | 0 µs | 0.000% |
| Result processing p95 | 81,356 µs | 81,347 µs | -9 µs | -0.011% |
| Maximum queue depth | 48 | 48 | 0 | 0.000% |
| Worker busy ratio | 0.424231 | 0.425711 | +0.001480 | +0.349% |
| Wall time | 287.41 s | 287.50 s | +0.09 s | +0.031% |
| Peak RSS | 297,496,576 B | 287,158,272 B | -10,338,304 B | -3.475% |
| CPU seconds | 157.50 s | 159.57 s | +2.07 s | +1.314% |

The canonical `1.0x` dataset passes the specified dual thresholds. No
meaningful final-head dataset regression was measured. The old `978c65b`
scan-p95 finding is not a finding against this final candidate.

### Stress characterization

| Rate | Result | Evidence |
|---:|---|---|
| 1.25x | BLOCKED/FAIL | `stress-1p25x-rerun/run.json`; lag, `load_shedding_count=9`, maximum queue depth `121` |
| 1.50x | BLOCKED/FAIL | `stress-1p50x/run.json`; input and processed counts did not converge and lag/load-shedding acceptance failed |

These are measured throughput-boundary findings. No production optimization
was attempted because the canonical `1.0x` A/B had no meaningful regression.

### Healthy SITL A/B

The final-head SITL protocol could not reach measurement. The headless session
was launched with PX4 v1.17, the pinned `px4_msgs` submodule, the project
Gazebo world, and candidate install, but timed out waiting for a real
`/px4/odometry_ros` sample before FAST-LIO startup. Cleanup completed with zero
orphan processes.

Evidence:

```text
.artifacts/verification/p0.8-performance/sitl-off/px4-mid360-20260803-131938/
```

The required three OFF runs, three ON runs, 30-second warm-up, 120-second
measurement, comparison-valid ratio, query RTT, alignment-gap, supervisor
CPU/RSS, and FAST-LIO overhead are `NOT QUALIFIED`. No SITL result is inferred
from the prior observer report.

### Long-duration memory

The required supervisor-enabled 20-simulated-minute session was not run because
the SITL startup prerequisite failed. RSS growth, outstanding query bound,
container growth, and long-duration state stability are `NOT QUALIFIED`.

### Final gates

- [x] Exact starting HEAD and clean PX4 message submodule captured
- [x] Final candidate full SHA captured in canonical dataset artifacts
- [x] Full-run latency distributions and sample counts captured
- [x] Three baseline and three candidate canonical runs passed correctness
- [x] Dataset latency dual-threshold qualification passed
- [x] Dataset CPU/RSS/wall qualification passed
- [x] 1.25x stress run completed and finding recorded
- [x] 1.50x stress run completed and finding recorded
- [ ] Healthy SITL OFF/ON A/B completed
- [ ] Query RTT and supervisor resource targets qualified
- [ ] 20-simulated-minute memory qualification completed
- [x] `make build` passed
- [x] `make build-safe` passed
- [x] `make test` passed
- [x] `make check` passed: 330 tests, 0 errors, 0 failures, 0 skipped
- [x] `make vendor-check` passed
- [x] P0.9 and P0.10 not started

## Revised final acceptance conclusion

```text
P0.8 status: BLOCKED

The final-head dataset correctness, full-run latency, CPU/RSS/wall-runtime,
and provenance qualifications pass. The 1.25x and 1.50x stress runs expose
measured throughput-boundary findings. The mandatory healthy SITL A/B and
20-simulated-minute memory qualification could not start because the final
headless session did not produce a PX4 odometry sample and was cleaned up
successfully. P0.8 therefore remains BLOCKED; it is not PASS WITH CONDITIONS.
```

The P0.8 contract hardening and runtime fault matrix are complete, and the
canonical final-head dataset correctness gates pass. The A/B dataset
qualification measured a scan-processing-p95 regression above the allowed
10% limit. The mandatory healthy-SITL A/B benchmark, 1.25x/1.50x stress runs,
and 20-simulated-minute memory qualification were not completed. Therefore
P0.8 cannot be closed as PASS.

This report distinguishes implementation, correctness, dataset regression,
fault qualification, performance qualification, and integrated release
status. P0.9 and P0.10 were not started.

## Revision and provenance

- Required starting commit: `978c65b6579aae153d6a2299e92efc9f85f37e63`
- Corrective branch: `fix/p0.8-acceptance-hardening`
- Immediate P0.7 parent: `d6bd7f0349d90167a26bf3710e65341f8b469404`
- PX4 v1.17 submodule: `src/external/px4_msgs` at
  `86d8239e962f6939e05c3737784f60c02fa884db`
- Corrective implementation commits: `dd08b7d`, `4f98716`, `c27dff1`
- Runtime sampler commit: `fffab3a`
- Final documentation commit: recorded by the final `git log` after this report
  is committed

The initial audit was recorded before production changes. The active branch
was never reset, stashed, or amended from the required starting commit, and
the PX4 message submodule was not modified.

## Independent review findings

| Finding | Resolution/evidence | Status |
|---|---|---|
| P0.8-R01 stale aligned residual | One `AlignedComparison` owns LIO, PX4, residual, epoch, response metadata, masks, and generations. Persistence advances only on a new comparison epoch. Held-epoch and stale-comparison tests pass. | Resolved |
| P0.8-R02 query masks/generations | Query sequence, exact timestamp, required POSITION/ORIENTATION/LINEAR_VELOCITY mask, reset generation, time generation, and late/superseded responses are checked. Invalid-response counters are published. | Resolved |
| P0.8-R03 time-generation recovery | Large low-epoch source restart increments the time generation, clears timestamp histories/ring state, resets sign continuity, and requires stable samples. Small regression remains a normal rejection. Runtime validator tests pass. | Resolved |
| P0.8-R04 LIO diagnostic fail-open | LIO and PX4 diagnostic schema/freshness are separate. Persistent invalid LIO diagnostics close the gate and enter DEGRADED after 500 ms; PX4 diagnostic loss makes monitoring unavailable without falsely invalidating LIO. | Resolved |
| P0.8-R05 runtime fault coverage | A real supervisor process was exercised against 16 injector scenarios with per-scenario oracle, timeline, query, transition, exit-code, and cleanup artifacts. | Resolved |
| P0.8-R06 final-head regression | Three final-head candidate and three exact-parent baseline replays completed. Correctness passes, but candidate scan p95 exceeds the +10% regression gate. | Open performance finding |

## Correctness implementation

The supervisor now uses an explicit aligned comparison with:

```text
comparison_epoch_ns
response_received_ros_time_ns
query_sequence
reset_generation
time_generation
component_validity_mask
covariance_availability_mask
interpolated
```

The configured maximum comparison age is `150 ms`. An aligned pair becomes
invalid when stale, when a newer eligible LIO epoch supersedes it, when the
response is late/failed, or when required component bits are absent. A held
residual cannot advance persistence.

The PX4 bridge keeps timestamp-sorted reset histories, detects probable source
restart separately from small timestamp regression, clears generation-bound
history, and suppresses continuity output until stable post-restart samples
are available. No registration, deskew, map, PX4 source, sensor model, or
protected baseline profile was changed.

The LIO diagnostic invalidity window is `500 ms`. The supervisor does not
convert invalid or unavailable comparisons into zero residuals, does not
extrapolate across a generation, and permits at most one outstanding PX4
sample query.

## Tests and build gates

- Supervisor focused gtests: `27/27` passed.
- PX4 bridge focused gtests: `21/21` passed.
- Fault-matrix Python tests: `16/16` passed.
- `make build-safe`: passed on the corrective branch; serial constrained
  build completed.
- `make test`: passed.
- `make check`: passed, `324 tests, 0 errors, 0 failures, 0 skipped`.
- `make vendor-check`: passed, `18 files`, `2 pinned upstream SHAs`, and
  `3 documented patched files`.
- PX4 v1.17 submodule remained clean at the pinned SHA.

## Runtime fault qualification

Artifact root:

```text
.artifacts/simulation/p08-fault-matrix-final-20260803/
```

Every scenario below used the real `odometry_supervisor_node`, returned
injector/supervisor exit code `0/0`, and completed scoped cleanup. The final
health values are the oracle values, not an inference from process exit.

| Scenario | Final result |
|---|---|
| healthy | HEALTHY; comparison valid; gate open |
| single_position_jump | HEALTHY; no false divergence |
| slow_xy_drift | DIVERGED after persistence |
| slow_yaw_drift | DIVERGED after persistence |
| velocity_bias | DEGRADED/DIVERGED transition recorded by oracle |
| px4_stale | HEALTHY LIO retained; comparison invalid; no unsafe reinit |
| px4_diagnostics_stale | HEALTHY; monitoring unavailable, no false LIO loss |
| lio_propagated_stale | DEGRADED; gate closed |
| lio_corrected_stale | DEGRADED; gate closed |
| lio_diagnostics_stale | DEGRADED; gate closed |
| lio_lost | DIVERGED; independent mode requests reinitialization once |
| correlated_unhealthy | DIVERGED on LIO loss; no reinitialization request |
| px4_reset_generation | reset grace and recovery; no false divergence |
| px4_time_generation | generation change invalidates old comparison; recovery |
| diagnostic_schema_corruption | DEGRADED; gate closed |
| clock_pause | no persistence advance while ROS clock is paused |

`clock_pause.json` is a real runtime scenario. Its test injector holds the
simulation clock from startup to isolate ROS-time persistence from DDS startup
ordering, then freezes all input epochs. This does not claim that a Gazebo
session was paused for the performance benchmark.

## Dataset A/B qualification

The same AIST bag, configuration, machine environment, serial build policy,
and `1.0x` rate were used. The baseline was built in the isolated worktree
`/home/letandat/Dev/uav-navigation-p0.7-baseline` at the exact immediate P0.7
parent. Candidate runs used the corrective branch with RViz disabled. The
earlier candidate artifact created with RViz enabled was deliberately excluded
from the three-run comparison.

Baseline artifacts:

```text
/home/letandat/Dev/uav-navigation-p0.7-baseline/.artifacts/datasets/aist-mid360-drive/d6bd7f0-replay-1.0x-20260803T035545919677Z
/home/letandat/Dev/uav-navigation-p0.7-baseline/.artifacts/datasets/aist-mid360-drive/d6bd7f0-replay-1.0x-20260803T040549133398Z
/home/letandat/Dev/uav-navigation-p0.7-baseline/.artifacts/datasets/aist-mid360-drive/d6bd7f0-replay-1.0x-20260803T041555459594Z
```

Candidate artifacts:

```text
.artifacts/datasets/aist-mid360-drive/978c65b-replay-1.0x-20260803T040048026522Z
.artifacts/datasets/aist-mid360-drive/978c65b-replay-1.0x-20260803T041047577751Z
.artifacts/datasets/aist-mid360-drive/978c65b-replay-1.0x-20260803T042058487636Z
```

Every run received and processed `55,435` IMU and `2,772` LiDAR samples;
estimator and replay exit codes were `0/0`; drops, overflow, invalid
timestamps, NaN/Inf outputs, final queue depth, and load shedding were zero.
Dynamic propagated odometry output and the single dynamic TF authority were
present.

| Metric, median of 3 runs | P0.7 baseline | P0.8 candidate | Delta |
|---|---:|---:|---:|
| Maximum queue depth | 48 | 48 | 0% |
| Mean scan processing | 18,059.641 µs | 18,137.681 µs | +0.432% |
| Scan p95 | 53 µs | 59 µs | **+11.321%** |
| Scan p99 | 31,749 µs | 33,795 µs | +6.444% |

The candidate canonical correctness predicate is PASS: no processing lag and
no load shedding. The p95 result is nevertheless an open performance finding
because the acceptance limit is no worse than `+10%` against the baseline
median. Resource sampling was added and exercised on selected runs, but was
not captured for all six repetitions with a validated CPU-percent conversion;
wall-runtime, complete peak-RSS, context-switch, and real-time-factor gates
are therefore `NOT INSTRUMENTED`.

The earlier P0.0 processing-lag finding is not silently carried forward: the
final-head A/B comparison demonstrates that the canonical candidate runs did
not trigger it, while the measured p95 regression remains a distinct P0.8
performance finding.

## Stress characterization

```text
1.25x complete run: NOT RUN
1.50x complete run: NOT RUN
```

The first rate for persistent queue growth, load shedding, processing lag, or
drops is consequently not characterized.

## Healthy SITL and long-duration qualification

The repository has a prior healthy SITL artifact, but it predates this final
hardening qualification and is not evidence for the required A/B benchmark.
The mandatory final-head measurement was not completed:

```text
Supervisor-disabled A/B: NOT RUN (3 repetitions, 30 s warm-up + 120 s)
Supervisor-enabled A/B: NOT RUN (3 repetitions, 30 s warm-up + 120 s)
20 simulated-minute memory session: NOT RUN
```

Therefore the following required metrics are `NOT INSTRUMENTED` for the final
P0.8 candidate: comparison-valid ratio over the benchmark, query RTT
p50/p95/p99/max, alignment-gap distribution, aligned-comparison-age
distribution, supervisor CPU/RSS targets, FAST-LIO p95/queue-depth overhead,
long-duration RSS growth, maximum outstanding queries over 20 minutes, and
false health transitions over that benchmark.

## Acceptance checklist

- [x] Started at exact `978c65b6579aae153d6a2299e92efc9f85f37e63`
- [x] Corrective branch created
- [x] Initial tree and PX4 submodule clean
- [x] Held residual cannot advance persistence
- [x] Comparison has explicit epoch and age
- [x] Query sequence, required mask, reset generation, and time generation validated
- [x] Superseded responses rejected
- [x] Comparison invalidates when stale
- [x] LIO and PX4 diagnostic validity separated
- [x] Persistent LIO diagnostic loss closes the gate
- [x] PX4 diagnostics loss does not falsely invalidate LIO
- [x] Runtime time-generation increment and probable-restart recovery tested
- [x] Small timestamp regression does not trigger restart
- [x] Real runtime clock-pause injector scenario passes
- [x] Slow XY/yaw, loss, stale, schema, reset, and time-generation fault scenarios pass their oracles
- [x] Fault artifacts contain timelines and cleanup/exit evidence
- [x] Three exact-parent baseline dataset runs completed
- [x] Three final-head candidate dataset runs completed
- [x] Candidate canonical dataset has zero lag/load shedding
- [x] No dataset correctness regression
- [ ] No unacceptable latency regression — candidate scan p95 is +11.321% vs +10% gate
- [ ] 1.25x and 1.50x stress characterization completed
- [ ] Healthy final-head SITL A/B benchmark completed
- [ ] Query RTT and supervisor CPU/RSS targets qualified
- [ ] Long-duration 20-minute memory stability completed
- [x] `make build-safe`
- [x] `make test`
- [x] `make check`
- [x] `make vendor-check`
- [x] P0.9 not started
- [x] P0.10 not started

## Historical conclusion (superseded by final qualification above)

```text
P0.8 status: BLOCKED

The P0.8 contract hardening, correctness tests, runtime fault matrix, and
canonical final-head dataset correctness runs are complete. The measured
candidate scan-processing p95 regression is +11.321% against the immediate
P0.7 parent, exceeding the +10% acceptance limit. The required SITL A/B,
stress-rate, and 20-minute memory qualifications were not completed. P0.8
must remain BLOCKED until the performance finding is resolved or accepted by
evidence and all mandatory runtime qualifications are complete.
```

## SITL startup root-cause analysis

This section records the current continuation from the required starting
commit `fd005e2cfb3e26ee68f162046410a97a317ba5f0` on branch
`fix/p0.8-sitl-qualification`. The earlier safety-gate failure was caused by
a stale prompt prerequisite; the valid continuation point was `fd005e2`, and
no reset to `b23c706` was performed.

### Failed-session evidence

The preserved failed session was inspected before rerunning:

```text
.artifacts/verification/p0.8-performance/sitl-off/px4-mid360-20260803-131938/
```

The session used `PX4_DIR=/home/letandat/Dev/Autopilot` and recorded PX4 SHA
`6249cc3d892e161f5834d8976e41b5ee27443864`, not the required stable v1.17
worktree. It timed out before receiving a real `/px4/odometry_ros` sample.
The audit therefore classified the failure as an incorrect PX4 worktree /
binary selection, not as evidence of a FAST-LIO or bridge conversion defect.

### Raw PX4 odometry

The corrected startup check discovers the runtime topic from the graph rather
than assuming a suffix. With the pinned message package it found:

```text
candidate: /fmu/out/vehicle_odometry
selected:  /fmu/out/vehicle_odometry
publishers: 1
sample: received
```

The pinned dependencies were verified as:

```text
PX4:      d6f12ad1c4f70ad3230afd7d86e971421e02fef4
px4_msgs: 86d8239e962f6939e05c3737784f60c02fa884db
```

### XRCE connectivity

The startup artifact records the Micro XRCE-DDS agent, client connection, and
session-establishment log evidence as present.

### Clock-domain verification

The workflow uses PX4 simulation-clock mode with `UXRCE_DDS_SYNCT=0`, a ROS
bridge with `use_sim_time=true`, and waits for an advancing `/clock` before
starting the PX4 odometry bridge. The startup artifact measured clock samples
from `2760000000` ns to `2764000000` ns and marked `advanced=true`.

### Bridge binary and overlay

The intended overlay resolves all runtime packages to this workspace:

```text
px4_odometry_bridge: /home/letandat/Dev/uav-navigation/install/px4_odometry_bridge
odometry_supervisor: /home/letandat/Dev/uav-navigation/install/odometry_supervisor
fast_lio_ros:        /home/letandat/Dev/uav-navigation/install/fast_lio_ros
```

The resolved executables are `px4_odometry_bridge_node`,
`odometry_supervisor_node`, and `fast_lio_node`. The startup workflow records
role PID, PGID, command, start time, log, and exit evidence for each owned
process.

### Bridge rejection state

The corrected startup artifact is:

```text
.artifacts/verification/p0.8-sitl-qualification/startup/attempt-04/startup-check.json
```

It is `PASS`: raw VehicleOdometry was received, the bridge was running with
the simulation clock, `/px4/odometry_ros` produced a real sample at
`8432000000` ns, and `timestamp_rejected_count`,
`conversion_rejected_count`, and `reset_suppressed_count` were all zero.

### Confirmed root cause

The confirmed root cause of the original missing output was the launch default
selecting the wrong PX4 worktree. Topic versioning, XRCE connectivity, and
clock readiness were then verified with the startup-only workflow.

### Minimal fix

The minimal fix pins the default SITL worktree and required PX4 SHA, validates
that the PX4 tree is clean and exact, waits for real samples at every startup
stage, passes the explicit clock/profile parameters, and cleans only owned
process groups. No PX4 source, `px4_msgs` contents, estimator logic, bridge
conversion mathematics, or supervisor FSM thresholds were changed.

## SITL probe semantic correction

### Supervisor OFF applicability

The probe now reports supervisor-specific fields as `null` and sets
`supervisor_metrics_applicable=false` when the supervisor is disabled. It no
longer fabricates passing comparison-valid, monitoring, RTT, or supervisor
resource values. OFF acceptance is limited to bridge and FAST-LIO health,
corrected-output latency, queue/load-shedding, drop/overflow, and related
runtime metrics.

### Supervisor ON requirements

ON mode requires a live supervisor and retains the supervisor acceptance gates:
healthy state, comparison validity, monitoring availability, query timeout and
generation-mismatch checks, reinitialization checks, alignment limits, and
resource limits.

### Process sampling correction

The process samplers now cache `psutil.Process` handles by PID and creation
time, prime CPU sampling before measurement, detect replacement/death, and
emit `missing`, `dead`, `primed`, or `measured` states. Missing or dead roles
are not converted into valid zero CPU/RSS values. These semantics and the
OFF/ON acceptance split passed the repository test suite.

## Healthy SITL smoke

The startup-only qualification passed, but the required supervisor-ON smoke
did not pass its acceptance gates. Evidence:

```text
.artifacts/verification/p0.8-sitl-qualification/smoke4/on-smoke.json
.artifacts/verification/p0.8-sitl-qualification/smoke4/px4-mid360-20260803-141616/
```

The smoke used 10 simulated seconds of warm-up and 20 simulated seconds of
measurement. It demonstrated raw PX4 odometry, bridge output, FAST-LIO
corrected and propagated output, and successful cleanup with zero owned
orphan processes. The final runtime diagnostic snapshot also showed the
P0.6 topic prior accepted (`TOPIC_PRIOR_ACCEPTED`),
`initial_prior_fallback_applied=false`, and FAST-LIO `TRACKING`; the early
`initial-state prior gate is closed` log lines were transient startup
observations, not the final prior result.

The measured supervisor smoke result was:

| Metric | Result | Required |
|---|---:|---:|
| `comparison_valid_ratio` | `0.0` | `>= 0.99` |
| `monitoring_available_ratio` | `0.9523809524` | `>= 0.99` |
| query timeout count | `2` | `0` |
| query generation mismatch count | `0` | `0` |
| query RTT p95 / p99 | `0.5 / 0.5 ms` | `< 50 / 100 ms` |
| alignment gap p99 | `0 ms` | `<= 50 ms` |
| aligned comparison age p99 | unavailable | `<= 150 ms` |
| supervisor final health | `HEALTHY` | `HEALTHY` |

The failed comparison gate was reported as `ALIGNED_COMPARISON_STALE` while
the asynchronous supervisor query and continuously advancing LIO/PX4 epochs
were running. This is a confirmed runtime blocker for qualification, but it
is outside the allowed SITL orchestration/probe scope. No supervisor
alignment or FSM production logic was changed speculatively.

## Healthy SITL A/B

The full interleaved qualification was not started because the ON smoke did
not pass:

```text
OFF-1, ON-1, OFF-2, ON-2, OFF-3, ON-3: NOT RUN
Warm-up: 30 simulated seconds: NOT RUN
Measurement: 120 simulated seconds: NOT RUN
```

This preserves the prerequisite rule and avoids treating incomplete A/B data
as acceptance evidence.

## Supervisor overhead

The smoke artifact captured query RTT and resource fields, but it is not a
substitute for the required three-run A/B qualification. The full supervisor
overhead gates remain `NOT INSTRUMENTED` for P0.8 closure.

## Long-duration stability

The 1,200-simulated-second run was not started because both prerequisites are
not satisfied: the short ON smoke failed and full ON health contract evidence
does not exist. No memory-growth result is claimed.

## Stress-capacity classification

The existing 1.25x and 1.50x observations remain capacity-characterization
findings for the qualification host. They are not reclassified as functional
regressions and no estimator or mapping optimization was introduced.

## Current P0.8 acceptance checklist

- [x] Required starting commit and continuation branch recorded
- [x] Preserved failed session audited
- [x] Wrong PX4 worktree root cause identified
- [x] Stable PX4 v1.17 SHA and pinned `px4_msgs` verified
- [x] Raw VehicleOdometry topic and sample demonstrated
- [x] XRCE connection and session demonstrated
- [x] Advancing `/clock` demonstrated
- [x] Startup-only bridge output demonstrated
- [x] Bridge diagnostics and cleanup evidence captured
- [x] OFF metrics no longer fabricate supervisor values
- [x] Missing/dead process sampling is not represented as zero load
- [x] `make build-safe`
- [x] `make test`
- [x] `make check`
- [x] `make vendor-check`
- [ ] Short supervisor-ON smoke passes all gates
- [ ] Three OFF SITL runs complete
- [ ] Three ON SITL runs complete
- [ ] Supervisor overhead qualified
- [ ] 1,200-simulated-second memory qualification complete
- [x] P0.9 not started
- [x] P0.10 not started

## Historical final P0.8 conclusion (superseded by corrective task above)

```text
P0.8 status: BLOCKED

SITL startup recovery is PASS: the pinned PX4 v1.17 worktree, raw
VehicleOdometry topic/sample, XRCE session, advancing simulation clock, and
bridge output are reproducible and cleaned up successfully. The qualification
probe now has correct OFF/ON applicability and process-sampling semantics.

The required supervisor-ON smoke remains BLOCKED because comparison-valid
ratio was 0.0, monitoring-available ratio was 0.9523809524, and two query
timeouts occurred. Consequently the mandatory six-run A/B qualification and
1,200-simulated-second memory run were not executed. P0.8 cannot be marked
PASS until the supervisor alignment gate is resolved and these runtime gates
are measured successfully.
```
