# P0.9-A external odometry contract and non-fusing dry run

Status: Historical canonical P0.9-A transport run PASS; corrected contract
static validation PASS. The same-HEAD closeout runtime revalidation is
`BLOCKED_RUNTIME_BASELINE` as recorded below. PX4 fusion, real-hardware
timestamp conversion, and in-flight restart remain deferred.

## Provenance

| Item | Value |
| --- | --- |
| Repository | `/home/letandat/Dev/uav-navigation` |
| Branch | `feat/p0.9-a-external-odometry-contract` |
| Starting HEAD | `94b6722e025ec9eabdbff1c39b4c9bc635f2f6a3` |
| P0.8 closure | `e9f6cee23f279776fa8960f7f432b05664a80baf` |
| Historical canonical transport HEAD | `f0dcab2b97e0fdf02aa4889ab2f305d83f6cf050` |
| Corrected implementation HEAD | `a6d1b15` |
| Report HEAD | documentation update after `a6d1b15` |
| PX4 | `d6f12ad1c4f70ad3230afd7d86e971421e02fef4` |
| px4_msgs | `86d8239e962f6939e05c3737784f60c02fa884db` |
| Protected config SHA-256 | `2690d584703d709e24d7159759ddffd1ab6e5d6eb165870058b98af61c587f6f` |

The protected Mid-360/PX4/Gazebo YAML was not modified.

Commits in scope after the starting HEAD:

```text
97554c2  fix(supervisor): count each failed query once
eeda4f1  feat(lio): publish authoritative public frame generation
f4fe590  feat(px4): enforce external odometry contract
8216d18  test(p0.9): record external odometry dry-run evidence
09176e0  perf(lio): snapshot public frame metadata once
cffd06e  fix(px4): preserve genuine small external covariance
add0bd1  fix(p0.9): separate publisher readiness from publication gate
f0dcab2  test(p0.9): measure timestamp failures within dry-run window
a6d1b15  fix(p0.9): decouple timestamp mapping generation
```

## Implemented contract

- The LIO producer owns the public frame generation, starting at `1`. Internal
  LIO generation changes, corrected/propagated handoff, PX4 reset, gate changes,
  and geometric jumps do not increment it.
- `VehicleOdometry.reset_counter` is `public_generation % 256`; PX4 generation
  is diagnostic-only.
- Pose is FRD from ROS local Z-up; body velocity and angular velocity are BODY
  FRD from ROS body FLU. The quaternion uses
  `C_world * R_lio * inverse(C_body)` and is normalized.
- Covariance covers position XYZ, orientation small-angle XYZ, and body linear
  velocity XYZ. Full blocks are transformed before diagonal publication;
  cross-covariance is ignored explicitly. Invalid covariance fails closed and
  no synthetic `1e-6` floor is introduced.
- Timestamp conversion is explicit and accepts only proven SITL simulation-time
  equivalence. The SITL mapping uses a stable mapping generation for the
  lifetime of that proven clock mapping; real hardware returns
  `TIME_DOMAIN_UNRESOLVED` until a tested PX4 boot-time conversion exists.
- Timestamp monotonicity is checked only after conversion to PX4 integer
  microseconds. The converter stores accepted target-domain values and keeps
  mapping-generation changes independent from LIO public-frame generations.
- Publication uses one explicit gate. Publisher readiness is node/transport
  readiness; timestamp, supervisor authorization, public generation, freshness,
  covariance, frame, supervisor freshness, and jump-latch conditions remain
  separate gates.
- P0.9-A starts no EKF2 fusion and does not claim aiding, innovation, or
  estimator improvement.

## Validation

Static and focused validation was run serially after sourcing ROS 2 Jazzy and
the workspace overlay:

```text
make check: 421 tests, 0 errors, 0 failures, 0 skipped
make vendor-check: vendor freeze OK (18 files, 2 pinned upstream SHAs, 3 patched files)
px4_odometry_bridge focused tests: 5 test executables passed
odometry_supervisor diagnostic adapter: passed
```

The static/vendor checks were also run; generated build/install/log artifacts
remain machine-local and reproducible.

## Canonical SITL A/B evidence

Both historical runs used the same pre-closeout transport HEAD
`f0dcab2b97e0fdf02aa4889ab2f305d83f6cf050`, PX4 checkout, protected configuration,
`warmup=30 s`, and `measurement=60 s`. The PX4 EKF2 external-vision fusion
configuration was not enabled or changed.

| Run | Artifact | Result | Key evidence |
| --- | --- | --- | --- |
| A, external disabled | `.artifacts/verification/p0.9-a-sitl-off-20260804T0820Z` | `pass=true`, `acceptance_eligible=true` | FAST-LIO TRACKING; zero IMU/LiDAR drops, load shedding, overflow, and invalid timestamps; p95 corrected scan end-to-end `38190 us`; max queue `17`. |
| B, external enabled | `.artifacts/verification/p0.9-a-sitl-on-20260804T0750Z` | `pass=true`, `acceptance_eligible=true` | `165` external samples, `2.75 Hz`, timestamp age max `0 ns`, timestamp monotonic, conversion-failure delta `0`, pose frame `2`/FRD, velocity frame `3`/BODY_FRD, reset counter `{1}`, geometric jumps `0`, publication active and supervisor authorized. |

The B bridge's cumulative timestamp failure count was `2`, both before the
measurement window while startup timestamps were stale. The measured-window
delta was `0`; the cumulative diagnostic remains visible and was not erased.
All owned processes were cleaned up and no orphan process groups remained.

Earlier noisy SITL attempts are retained under `.artifacts/verification/` as
diagnostic provenance, but are not canonical acceptance evidence. They exposed
and led to fixes for the covariance floor and the publisher-readiness/schema
cycle; they were not hidden or reclassified as passes.

## Independent verdicts

| Verdict | Result |
| --- | --- |
| Query failure counter cleanup | PASS |
| External message contract | PASS |
| Frame and reset-counter contract | PASS |
| Covariance transform/fail-closed contract | PASS |
| SITL timestamp conversion | PASS for the measured simulation-time window; real hardware deferred |
| SITL non-fusing transport dry run | PASS_PRE_CLOSEOUT_HEAD |
| Incremental/runtime stability | PASS on canonical A/B; noisy non-canonical attempts retained |
| Real hardware timestamp/timesync | `DEFERRED_P0.9-B` |
| PX4 EKF2 fusion/innovation/aiding | `DEFERRED_P0.9-D` |
| In-flight LIO restart | `DEFERRED_P0.10` |

The historical P0.9-A transport contract is closed for the pre-closeout HEAD;
the corrected implementation's static contract is closed without claiming PX4
estimator fusion or navigation improvement. Corrected-head dynamic transport
remains `BLOCKED_RUNTIME_BASELINE`.

## P0.9-A-CLOSEOUT revalidation — 2026-08-04

This section records the review closeout separately from the earlier canonical
run. The earlier artifacts were classified before the corrective edits:

| Artifact | Classification | Evidence boundary |
| --- | --- | --- |
| `.artifacts/verification/p0.9-a-sitl-on-20260804T0750Z` | Canonical external-enabled artifact for the pre-closeout HEAD | `pass=true`, `acceptance_eligible=true`; 165 messages at 2.75 Hz, FRD/BODY-FRD frames, reset `{1}`, timestamp-failure delta `0`, publication active. It is not evidence for the corrected HEAD because its harness SHA is `f0dcab2`. |
| `.artifacts/verification/p0.9-a-sitl-off-20260804T0820Z` | Canonical external-disabled comparison for the pre-closeout HEAD | `pass=true`, `acceptance_eligible=true`; FAST-LIO no drops/load shedding/overflow and max queue `17`. It is not evidence for the corrected HEAD because its harness SHA is `f0dcab2`. |

The unresolved review items were corrected in commits `72f85ad` and
`a6d1b15`:

- timestamp sample regression, duplicate suppression, publication regression,
  equal publication timestamps, generation reset, and the three required
  reason-specific counters are now distinct;
- covariance is checked after double-to-float conversion, including positive
  underflow and overflow rejection as
  `COVARIANCE_NOT_FLOAT_REPRESENTABLE`;
- schema-v2 external publisher readiness is tested and isolated from timestamp
  and supervisor authorization, and `publication_active` now requires the
  readiness gate as well as a recent publication;
- `lio_public_frame_generation` no longer controls timestamp monotonicity;
  `timestamp_mapping_generation` is stable for SITL, mapping changes reset only
  timestamp state, and the bridge reports the mapping generation and change
  count;
- monotonicity is evaluated on accepted PX4 microseconds, so distinct source
  nanoseconds that quantize to one PX4 microsecond are suppressed.
- the runner records published sample strictness, publication nondecreasing
  behavior, timestamp counter deltas, readiness/active/authorization ratios,
  public-generation uniqueness, covariance rejection deltas, gate transitions,
  PX4 subscription count, and the explicit non-fusing verdict.

Focused validation on the corrected implementation HEAD passed: the affected
package built as `RelWithDebInfo`; all five `px4_odometry_bridge` test
executables and the supervisor diagnostic adapter passed, including the
mapping-generation and PX4-microsecond tests. The final static checks are
recorded in the validation block above.

The required same-HEAD 30 s warmup / 60 s measurement revalidation was run, but
did not produce a canonical PASS:

| Artifact | Result | Classification |
| --- | --- | --- |
| `.artifacts/verification/p0.9-a-closeout-off-20260804T0640Z` | FAIL: FAST-LIO `processing_lag_exceeded=true`, load shedding `2`, max queue `325` | Non-canonical runtime baseline failure; cleanup complete, no orphans. |
| `.artifacts/verification/p0.9-a-closeout-off-20260804T0710Z` | FAIL: FAST-LIO `processing_lag_exceeded=true`, load shedding `2`, max queue `202` | Non-canonical runtime baseline failure; cleanup complete, no orphans. |
| `.artifacts/verification/p0.9-a-closeout-off-20260804T0740Z` | FAIL: FAST-LIO `processing_lag_exceeded=true`, load shedding `0`, max queue `135` | Non-canonical runtime baseline failure; cleanup complete, no orphans. |
| `.artifacts/verification/p0.9-a-closeout-on-20260804T0810Z` | FAIL closed: supervisor `LIO_LOST`, zero external publications | Non-canonical runtime revalidation; bridge observed `publisher_ready_ratio=1.0`, public generation `{1}`, covariance rejection `0`, timestamp regression/duplicate deltas `0`, subscription count `2`, and fusion disabled, but authorization/publication remained closed. |

Therefore the corrected code and static contract are closed, while the final
absolute SITL/runtime verdict remains `BLOCKED_RUNTIME_BASELINE`. No claim is
made that the corrected HEAD has passed dynamic external publication until a
clean 30/60 A/B pair completes. PX4 EKF2 fusion, real-hardware time conversion,
and in-flight restart remain deferred as before.

## P0.9-A-FINAL independent verdicts

| Verdict | Result |
| --- | --- |
| P0.9-A static message contract | `PASS` |
| Timestamp mapping-generation contract | `PASS` |
| PX4 microsecond monotonicity | `PASS` |
| Historical transport evidence | `PASS_PRE_CLOSEOUT_HEAD` |
| Corrected-head dynamic transport | `BLOCKED_RUNTIME_BASELINE` |
| Real hardware timesync | `DEFERRED_P0.9-B` |
| EKF2 fusion | `DEFERRED_P0.9-D` |
| In-flight restart | `DEFERRED_P0.10` |
