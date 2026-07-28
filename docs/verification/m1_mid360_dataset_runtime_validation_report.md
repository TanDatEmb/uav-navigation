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
