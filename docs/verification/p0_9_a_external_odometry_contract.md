# P0.9-A external odometry contract and non-fusing dry run

Status: PASS for the P0.9-A contract and SITL transport dry run. PX4 fusion,
real-hardware timestamp conversion, and in-flight restart remain deferred.

## Provenance

| Item | Value |
| --- | --- |
| Repository | `/home/letandat/Dev/uav-navigation` |
| Branch | `feat/p0.9-a-external-odometry-contract` |
| Starting HEAD | `94b6722e025ec9eabdbff1c39b4c9bc635f2f6a3` |
| P0.8 closure | `e9f6cee23f279776fa8960f7f432b05664a80baf` |
| Final HEAD | `f0dcab2b97e0fdf02aa4889ab2f305d83f6cf050` |
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
  equivalence. Real hardware returns `TIME_DOMAIN_UNRESOLVED` until a tested
  PX4 boot-time conversion exists.
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
workspace colcon tests: 416 passed, 0 errors, 0 failures, 0 skipped
px4_odometry_bridge package: 5 test executables, 50 test cases passed
odometry_supervisor package: 70 test cases passed
tools/tests/test_p0_8_sitl_orchestrator.py: 25 passed
```

The static/vendor checks were also run; generated build/install/log artifacts
remain machine-local and reproducible.

## Canonical SITL A/B evidence

Both runs used the same final HEAD, PX4 checkout, protected configuration,
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
| SITL non-fusing transport dry run | PASS |
| Incremental/runtime stability | PASS on canonical A/B; noisy non-canonical attempts retained |
| Real hardware timestamp/timesync | `DEFERRED_P0.9-B` |
| PX4 EKF2 fusion/innovation/aiding | `DEFERRED_P0.9-D` |
| In-flight LIO restart | `DEFERRED_P0.10` |

The final P0.9-A contract is therefore closed without claiming PX4 estimator
fusion or navigation improvement.
