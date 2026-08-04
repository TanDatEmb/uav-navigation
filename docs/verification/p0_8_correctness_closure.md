# P0.8-CLOSE correctness closure

Date: 2026-08-04

Branch: `fix/p0.8-correctness-closure`

Starting HEAD: `aef7ded2e157991409af644f7c0e4af0991c8650`

Implementation baseline: `4f75c1a9d381f5dc87c595de77bb2763d133923e`
Closure commit: `e9f6cee23f279776fa8960f7f432b05664a80baf`

## Result matrix

| Gate | Result | Evidence |
| --- | --- | --- |
| P0.8 correctness | PASS | Alignment lifecycle, query eligibility, failure-class separation, residual gates, reset semantics, and explicit generation tests pass. |
| Supervisor incremental runtime | PASS | 60 s measurement: comparison-valid ratio `1.0`, monitoring ratio `1.0`, zero query failures/timeouts/generation mismatches, zero state-transition delta, zero reinitialization delta, maximum outstanding queries `0`. |
| Absolute FAST-LIO runtime | PASS | 60 s targeted RelWithDebInfo run: `TRACKING`, corrected estimate valid, navigation valid, processing lag not exceeded, queue overflow false, IMU/LiDAR drops zero, p95 corrected scan end-to-end `35,928 us`. A historical 120 s throughput comparison was not rerun. |
| External PX4 fusion | DEFERRED_P0.9 | `P08_EXTERNAL_ODOMETRY=0`; no external publisher was started or authorized. |
| In-flight restart | DEFERRED_P0.10 | No in-flight restart maneuver is part of P0.8 correctness closure. |

## Static and targeted validation

Commands run serially after the implementation change:

```text
source /opt/ros/jazzy/setup.bash
colcon build --packages-select navigation_interfaces odometry_supervisor px4_odometry_bridge --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo
colcon test --packages-select odometry_supervisor px4_odometry_bridge --event-handlers console_direct+
make check
make vendor-check
python3 -m unittest tools/tests/test_p0_8_sitl_orchestrator.py
```

Results: odometry supervisor `68/68`, PX4 bridge `35/35`, external conversion
`3/3`, `make check` `395` tests with no failures, vendor freeze check OK, and
Python orchestrator tests `25/25`.

Targeted SITL command:

```text
P08_EXTERNAL_ODOMETRY=0 python3 tools/performance/p0_8_sitl_orchestrator.py sitl-on \
  --workspace /home/letandat/Dev/uav-navigation \
  --px4-dir /home/letandat/Dev/Autopilot-p0.7-v1.17 \
  --warmup-s 30 --measurement-s 60 \
  --expected-git-sha e9f6cee23f279776fa8960f7f432b05664a80baf \
  --output .artifacts/verification/p0.8-close-sitl-20260804T044352Z
```

The canonical artifact is
`.artifacts/verification/p0.8-close-sitl-20260804T044352Z/`; `run.json` is
`PASS` with complete cleanup and `measurement.json` has the metric evidence
above. PX4 was `d6f12ad1c4f70ad3230afd7d86e971421e02fef4` and `px4_msgs` was
`86d8239e962f6939e05c3737784f60c02fa884db`.

## Closed contracts

The decision ledger at
`docs/architecture/odometry_decision_ledger.md` closes the prior root-cause
and frame decisions. The implementation now enforces:

- one no-extrapolation selector for alignment and comparison;
- one query per epoch/generation with bounded outstanding work and service retry backoff;
- transport/contract failures that preserve a locked transform and do not count as geometric failures;
- finite exact-time generation-matched residual gates with optional NIS;
- frozen-transform revalidation with strictly increasing epoch/evidence;
- immediate invalidation on uncompensated reset;
- explicit frame generation with startup-only restart semantics and no reset-generation fallback.

Protected configuration SHA:

```text
src/navigation_estimator/fast_lio_ros/config/mid360_px4_gazebo.yaml
2690d584703d709e24d7159759ddffd1ab6e5d6eb165870058b98af61c587f6f
```
