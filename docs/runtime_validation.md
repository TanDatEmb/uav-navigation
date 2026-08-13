# Runtime validation

This document is the current runtime source of truth. The repository has one
runner, one monitor, one report schema, and one owned process-group registry.

## Commands

```bash
make build
make test
make replay DATASET=aist-mid360-drive RATE=1.0
make replay DATASET=aist-mid360-drive RATE=1.0 FRONTIER_DEBUG=1
make dataset-check DATASET=aist-mid360-drive RATE=1.0
PX4_DIR=$HOME/Dev/Autopilot make sim-check
PX4_DIR=$HOME/Dev/Autopilot make sim
make status
make stop
make clean
```

`dataset-check` requires a prepared ROS 2 bag. `RATE` changes rosbag replay
speed inside the same workflow; it does not select a profile. The dataset
workflow starts FAST-LIO with `config/runtime/dataset.yaml`, remaps the
prepared input to `/lidar/points` and `/lidar/imu`, waits for TRACKING, drains
the queues, and writes its report automatically. `make replay` is the same
workflow entrypoint and always launches RViz; `dataset-check` remains the
headless dataset contract.

Standard replay keeps ROG frontier extraction and frontier publication off,
even when RViz is running. To inspect `/rog_map/frontier` in RViz, use the
explicit debug profile shown above; it enables both
`mapping.rog.frontier_enabled` and `mapping.visualization.publish_frontier`
for that session only.

`sim-check` starts PX4 SITL, Gazebo, the Micro XRCE-DDS agent, the bridge,
FAST-LIO with `config/runtime/sim.yaml`, and the deterministic scenario from
`config/runtime/offboard.yaml`. The scenario verifies OFFBOARD entry, arm,
takeoff, translation/yaw segments, landing, and disarm. `sim` starts the same
stack with the Gazebo GUI and RViz, but no controller; stopping it produces
`OBSERVATION_COMPLETE`, never a flight `PASS`.

## Configuration

| File | Owner | Purpose |
|---|---|---|
| `config/runtime/common.yaml` | monitor/report | stream rates, freshness thresholds, and workflow timeouts |
| `config/runtime/dataset.yaml` | dataset runner | real AIST timing, frames, extrinsic, input QoS |
| `config/runtime/sim.yaml` | simulation runner | Gazebo timing, frames, extrinsic, external odometry enablement |
| `config/runtime/offboard.yaml` | headless simulation | one deterministic flight trajectory |

The runner passes only explicit ROS node parameter trees (`fast_lio` and, for
simulation, the two PX4 bridge nodes). Workflow metadata stays in the session,
so ROS diagnostics do not expose config or Git SHA fields. The static
`livox_frame -> livox_imu_frame` transform is derived by inverting the same
`extrinsic` value used by FAST-LIO; it is not maintained as a second calibration.

## Required runtime topics

| Class | Topic | Type | Decision |
|---|---|---|---|
| sensor | `/lidar/points` | `sensor_msgs/msg/PointCloud2` | required |
| sensor | `/lidar/imu` | `sensor_msgs/msg/Imu` | required |
| product | `/lio/odometry_corrected` | `nav_msgs/msg/Odometry` | required |
| product | `/lio/odometry_propagated` | `nav_msgs/msg/Odometry` | required |
| transform | `/tf`, `/tf_static` | `tf2_msgs/msg/TFMessage` | required |
| health | `/lio/diagnostics` | `diagnostic_msgs/msg/DiagnosticArray` | single surface |
| simulator truth | `/sim/ground_truth/odometry` | `nav_msgs/msg/Odometry` | evaluation-only ENU/FLU reference; never an LIO/PX4 input |
| PX4 input | `/fmu/in/vehicle_visual_odometry` | `px4_msgs/msg/VehicleOdometry` | simulation only |
| PX4 output | `/fmu/out/vehicle_odometry` | `px4_msgs/msg/VehicleOdometry` | simulation observation |
| PX4 output | `/fmu/out/vehicle_status_v1` | `px4_msgs/msg/VehicleStatus` | simulation observation |
| PX4 estimator telemetry | `/fmu/out/estimator_status_flags` | `px4_msgs/msg/EstimatorStatusFlags` | observed control status only; not per-sample fusion proof |

The monitor measures sample count, rate, maximum source-timestamp gap,
callback stalls, timestamp duplicates/regressions, finite values, quaternion
validity, covariance validity, and frame IDs. A callback stall is retained in
the artifact, but only a matching source-timestamp gap is a freshness failure:
the monitor's own delayed callback dispatch must not masquerade as dropped
sensor or PX4 data. Topic discovery alone is never readiness evidence.
For dataset replay, the report allows the configured 0.5 s tail grace after
rosbag EOF so queued messages can drain; the raw monitor counters remain in the
artifact, while only active stale events affect the verdict.

The compact `/lio/diagnostics` surface contains the estimator state and health
gate fields: `state`, `navigation_valid`, `last_failure_code`,
`last_failure_reason`, output timestamps, input/output counters, drops,
timestamp regressions, queue maximum, correction counts, map point count, and
covariance availability. Detailed investigation belongs in session logs and
source-level tests, not in a second runtime topic.

## External odometry gate

The PX4 external publisher is enabled only in simulation. It publishes only
when the node and subscriber transport are ready, timestamps and covariance
are valid, LIO is TRACKING and navigation-valid, the estimator diagnostics are
fresh, the propagated odometry callback is current, the public frame generation
is valid, and the frame has no geometric jump latch. The propagated diagnostics
heartbeat is observed separately and does not override a current high-rate
odometry callback during a transient correction-replay warning. It fails
closed on stale, invalid, non-finite, regressed, or LOST data. No odometry
supervisor process is part of the canonical workflows.

Its sole odometry input is `/lio/odometry_propagated` with the exact
`lio_odom -> base_link` frame pair. `/sim/ground_truth/odometry` is not
subscribed by LIO or the PX4 external bridge. The PX4 launcher also sets
`SIM_GZ_EN_ODOM=0` before boot, which disables PX4's built-in Gazebo path that
would otherwise publish the same simulator truth as internal visual odometry.

The propagated output accepts a corrected-LIO age of at most 250 ms. At the
10 Hz correction rate this permits one delayed correction, but a sustained
estimator or host stall stops publication fail-closed. The retained one-second
IMU history remains recovery-only; no simulator truth is ever substituted for
a missing correction.

## Verdicts and artifacts

Each session is under `.artifacts/runtime/<workflow>-<timestamp>-<pid>/` and
contains process logs, `processes.json`, `state.json`, `monitor.json`,
`samples.jsonl`, `report.json`, and `REPORT.md`. The latest session is exposed
through `.artifacts/runtime/latest`.

The report verdict is one of `PASS`, `FAIL`, `BLOCKED`, `NOT_RUN`, or
`OBSERVATION_COMPLETE`. Dataset PASS requires valid/fresh sensor and LIO
streams, TRACKING, no drops, and complete cleanup. Simulation PASS additionally
requires the fixed offboard scenario, the real PX4 output/status streams, and
the LIO-to-PX4 frame/timestamp contract. No unsupported aid-source topic is
used. PX4 status flags are observational telemetry, while accuracy is
calculated against the separate simulator truth.

## RViz products

`make sim` and `make replay DATASET=<name>` start RViz with the project config.
It uses `lio_odom` as fixed frame and shows the canonical TF tree plus
`/lio/odometry_corrected`. Product profiles do not publish registered-point or
local-map debug clouds; those serializers must stay in an explicit debug-only
workflow if reintroduced.

`make stop` signals only process groups recorded for the latest session. It
does not use global name-based termination and does not affect unrelated ROS,
Gazebo, or PX4 processes.

## Feature inventory

| Feature | Canonical consumer | Decision |
|---|---|---|
| LiDAR/IMU ingestion and synchronization | dataset-check, sim-check | keep |
| deskew and initialization | dataset-check, sim-check | keep |
| LiDAR correction and registration map | LIO product pipeline | keep; no product debug-cloud serialization |
| corrected and propagated odometry | all runtime workflows | keep |
| covariance and TF | all runtime workflows | keep |
| PX4 ingress, time/frame conversion | sim-check, sim | keep |
| external odometry safety gate | sim-check, sim | simplify to bridge-local health gate |
| supervisor lifecycle/residual framework | none | delete |
| offboard controller | sim-check only | keep as deterministic scenario |
| duplicate observers/reports/profiles | none | delete |
| cleanup | all workflows | keep in process-group runner |
