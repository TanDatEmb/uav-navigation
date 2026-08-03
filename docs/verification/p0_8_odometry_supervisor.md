# P0.8 Odometry Supervisor — acceptance hardening

## Revised result

```text
P0.8 status: BLOCKED
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

## Revised conclusion

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
