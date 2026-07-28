# M1-D2 Mid-360 runtime validation

## Verdict

```text
M1 CORE ALGORITHM VALIDATION: PASS
M1 ROS RUNTIME VALIDATION: PASS
```

Upstream ROS1 comparison remains `BLOCKED` because this host has neither ROS1
Noetic nor Docker/Podman. This does not block the D2 core/runtime verdict.

## Root cause and fix

The earlier 1,343 apparent IMU-gap rejections were not a dataset or core
synchronizer defect. Direct replay read and core-accepted all 8,000 IMU and
1,384 LiDAR messages with maximum source gap 16,989,504 ns. ROS callbacks had
been synchronously running prediction, registration, IKFoM and map insertion,
starving DDS ingestion.

Callbacks now validate/convert, enqueue into a bounded queue and return. One
joinable worker owns `FastLioPipeline`, IKFoM and ikd-Tree. The full 1.0x ROS
acceptance run received and core-accepted all messages; queue high-water mark
was 10 and maximum ingress IMU gap exactly matched source inspection.

Instrumentation also exposed a second, independent bottleneck. For roughly
1,800 residuals, vendored IKFoM formed and inverted a dense 1,800 by 1,800
measurement covariance even though `h_v=I`, noise was diagonal and the error
state is 23-DoF. It consumed 7–20 seconds per update. The implementation now
uses algebraically equivalent weighted normal equations in 23 dimensions and
a compact variance vector. No residual gate, noise value or convergence
threshold was relaxed.

The official four-iteration budget often ended with finite but not-yet-
converged increments between 0.002 and 0.009. The dataset configuration now
allows ten iterations while retaining the original convergence threshold.
Successful scans usually finish in two to seven iterations.

## Diagnostic matrix

| Run | IMU | LiDAR | Corrections | Queue HWM | Result |
|---|---:|---:|---:|---:|---|
| Direct 5 s | 868/868 | 151/151 | 84 success, 1 fail | n/a | PASS |
| Direct full | 8000/8000 | 1384/1384 | 973 success, 1 fail | n/a | PASS |
| ROS 0.05x, 5 s | 868/868 | 151/151 | diagnostic run | 2 | PASS |
| ROS 1.0x full | 8000/8000 | 1384/1384 | 973 success, 1 fail | 10 | PASS |

There are 380 explicitly classified light scan overlaps, two leading missing
start brackets, 26 initialization-window scans, one scan before the
initialization epoch and one initial non-converged update. Overlap is not
reported as IMU gap and the following scan continues normally. The raw damaged
final scan remains preserved and fail-closed.

## Tracking, covariance and map

- Corrected scans inserted: 973.
- Correction success ratio among attempted updates: 99.8973%.
- ROS final published map snapshot: 94,901 points.
- Deterministic direct final map: 94,970 points.
- Map frame: `odom`; state frame: `imu_link`.
- Extrinsic:
  `T_imu_lidar.t=[-0.019391,-0.000278,0.080926] m`, identity rotation.
- Trajectory contains no NaN/Inf.
- Every predict/correct now rejects non-finite, asymmetric or non-PSD
  covariance. The 300-step covariance sequence test passes.
- Numerical 23-DoF process Jacobian and point-to-plane position/orientation
  Jacobian tests pass. Fixed-extrinsic columns are zero.
- Dataset basis-vector, transform composition and inverse tests pass.

The XY/XZ/YZ and perspective images show a structured multi-scan environment,
not a single scan. No axis, yaw or extrinsic adjustment was applied.

## Runtime

Direct full processing took 18.08 s for 46.13 s of data. For 973 corrected
scans, direct total-processing timing was:

| Statistic | Time |
|---|---:|
| mean | 17.852 ms |
| p50 | 17.910 ms |
| p90 | 29.365 ms |
| p95 | 34.012 ms |
| p99 | 40.869 ms |
| max | 51.388 ms |

The ROS 1.0x acceptance run had total-processing p95 32.331 ms. Occasional
scans exceed the nominal 33.3 ms period, but bounded buffering absorbs them:
all input was received/accepted, queue HWM stayed at 10, and the queue drained
after playback.

## Determinism

Two direct full runs produced identical artifacts:

- Map SHA-256:
  `2d5bf6bf5009d755e6016ab44c2682377582b39f4d8f06a5a5bb671035a3579e`.
- Trajectory SHA-256:
  `e30312d128861417549cf9ce2c61fecc78303bfc2c5cf468ba154272a5bdbc83`.
- Corrections: 973/973.
- Map points: 94,970/94,970.

## Evidence and viewing

- Direct full run 2:
  `reports/m1_dataset/m1_d2_direct_full_run2/`.
- ROS 1.0x acceptance:
  `reports/m1_dataset/m1_d2_ros_full_1x_acceptance/`.
- Diagnostic matrix:
  `reports/m1_dataset/m1_d2_diagnostic_matrix.json`.
- Timing summary:
  `reports/m1_dataset/m1_d2_timing_summary.json`.

View the accepted ROS map:

```bash
make dataset-view RUN=m1_d2_ros_full_1x_acceptance
```

or:

```bash
pcl_viewer reports/m1_dataset/m1_d2_ros_full_1x_acceptance/map_full.pcd
```

## M1-D2 merge-hardening addendum

### IKFoM dense/compact equivalence

The compact production normal equations are now compared against the original
dense measurement-space expression for `M = 5, 22, 23, 24, 50, 200`, with 20
fixed seeds in well-conditioned, multi-scale-noise, near-degenerate-H and
wide-spectrum-P groups. The test compares gain action, `K H`, tangent
increment, corrected manifold state, covariance, convergence and final
increment norm.

Across the complete matrix, maximum observed errors were:

| Quantity | Maximum absolute error |
|---|---:|
| `K * innovation` | 4.91992e-10 |
| `K * H` | 4.00149e-10 |
| tangent increment | 4.91748e-10 |
| manifold state tangent difference | 4.71523e-10 |
| corrected covariance | 6.64101e-8 |
| final increment norm | 3.15416e-10 |

The covariance maximum occurs in the deliberately ill-conditioned groups and
is within the documented `1e-7` relative tolerance. Well-conditioned cases
use `1e-9`. Production uses `LLT` for the required-SPD covariance solve and
`LDLT` for the information solve. Solver or finite checks fail closed.

### Canonical configuration

Direct and ROS paths both use:

```text
src/navigation_estimator/fast_lio_ros/config/mid360_mutual_avoidance_uav1.yaml
SHA-256 ee37646f3b4668f13cad1febce5b79dcaa5f896c91709a9ca80740c72dcb0a3e
```

`EstimatorProfile` is the shared typed mapping into `EstimatorConfig`. The
direct runner no longer contains Mid-360 estimator constants. Direct
`run_summary.json`, `run_manifest.json`, `map_metadata.json`, ROS diagnostics
and exported ROS metadata all record the same path and SHA.

### Overlapping scan policy and processing ratios

ADR-009 locks M1 to fail-closed overlap rejection. Each rejection contains the
previous synchronized end, current start/end, overlap duration and scan index.
It does not advance the synchronized epoch or consume IMU; the next
non-overlapping scan continues normally.

The final direct and ROS acceptance counters agree:

| Stage | Count | Ratio |
|---|---:|---:|
| raw LiDAR | 1,384 | — |
| buffer accepted | 1,384 | 100.0000% |
| synchronized groups | 1,002 | 72.3988% of buffer accepted |
| overlap rejected | 380 | separately classified |
| correction attempts | 974 | — |
| correction successes | 973 | 99.8973% of attempts |
| correction failures | 1 | — |

The effective corrected output rate is `973 / 46.1256 s = 21.0946 Hz`.
`99.8973%` therefore means `973 / 974` attempted updates, not all input scans.

### Hardening rerun evidence

Direct runs C and D retained 973 successful corrections, one failed correction,
380 overlap rejections, 94,970 map points, and identical hashes:

- trajectory:
  `e30312d128861417549cf9ce2c61fecc78303bfc2c5cf468ba154272a5bdbc83`;
- map:
  `2d5bf6bf5009d755e6016ab44c2682377582b39f4d8f06a5a5bb671035a3579e`.

The final ROS 1.0x replay used rosbag publisher delay to complete DDS matching
before playback. It received and core-accepted all 8,000 IMU and 1,384 LiDAR
messages, emitted 973 corrected odometry samples, rejected 380 overlaps, had
one correction failure, and produced a 94,901-point map. Queue HWM was 12 and
remained bounded.

The development host was in Linux `powersave` governor with load average about
4.3 during hardening reruns. Direct C/D p95 values were 40.734/37.650 ms and
ROS p95 was 41.236 ms, versus earlier 34.012/32.331 ms baselines. The best
direct p95 difference is 10.7%; the observed slowdown is reported, not hidden,
and occurred without input loss or unbounded queue growth. No estimator
threshold, residual gate, noise or convergence policy changed.

Final artifacts:

- `reports/m1_dataset/m1_d2_hardening_direct_c/`
- `reports/m1_dataset/m1_d2_hardening_direct_d/`
- `reports/m1_dataset/m1_d2_hardening_ros_1x_final_pass/`

```bash
make dataset-view RUN=m1_d2_hardening_ros_1x_final_pass
```
