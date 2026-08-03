# P0.8 Odometry Supervisor — acceptance hardening

## Historical result before final performance qualification

```text
P0.8 status: BLOCKED
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
