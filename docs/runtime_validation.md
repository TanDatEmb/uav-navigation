# Runtime validation

This document is the current runtime source of truth. The repository has one
runner, one monitor, one report schema, and one owned process-group registry.

## Commands

```bash
make build
make test
make replay DATASET=aist-mid360-drive RATE=1.0
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

`sim-check` starts PX4 SITL, Gazebo, the Micro XRCE-DDS agent, the bridge,
FAST-LIO with `config/runtime/sim.yaml`, and the deterministic scenario from
`config/runtime/offboard.yaml`. The scenario verifies OFFBOARD entry, arm,
takeoff, translation/yaw segments, landing, and disarm. `sim` starts the same
stack with the Gazebo GUI and RViz, but no controller; stopping it produces
`OBSERVATION_COMPLETE`, never a flight `PASS`.

## Configuration

| File | Owner | Purpose |
|---|---|---|
| `config/runtime/common.yaml` | monitor/report | canonical topics, frames, rates, thresholds, timeouts |
| `config/runtime/dataset.yaml` | dataset runner | real AIST timing, frames, extrinsic, input QoS |
| `config/runtime/sim.yaml` | simulation runner | Gazebo timing, frames, extrinsic, external odometry enablement |
| `config/runtime/offboard.yaml` | headless simulation | one deterministic flight trajectory |

The runner passes only the `fast_lio` ROS parameter tree to the node. Workflow
metadata stays in the session, so ROS diagnostics do not expose config or Git
SHA fields.

## Required runtime topics

| Class | Topic | Type | Decision |
|---|---|---|---|
| sensor | `/lidar/points` | `sensor_msgs/msg/PointCloud2` | required |
| sensor | `/lidar/imu` | `sensor_msgs/msg/Imu` | required |
| product | `/lio/odometry_corrected` | `nav_msgs/msg/Odometry` | required |
| product | `/lio/odometry_propagated` | `nav_msgs/msg/Odometry` | required |
| transform | `/tf`, `/tf_static` | `tf2_msgs/msg/TFMessage` | required |
| health | `/lio/diagnostics` | `diagnostic_msgs/msg/DiagnosticArray` | single surface |
| simulator truth | `/sim/ground_truth/odometry` | `nav_msgs/msg/Odometry` | independent ENU/FLU accuracy reference |
| PX4 input | `/fmu/in/vehicle_visual_odometry` | `px4_msgs/msg/VehicleOdometry` | simulation only |
| PX4 output | `/fmu/out/vehicle_odometry` | `px4_msgs/msg/VehicleOdometry` | simulation observation |
| PX4 output | `/fmu/out/vehicle_status_v1` | `px4_msgs/msg/VehicleStatus` | simulation observation |
| PX4 aid | `/fmu/out/estimator_status_flags`, `/fmu/out/estimator_aid_src_ev_*` | PX4 estimator messages | fusion evidence |

The monitor measures sample count, rate, maximum gap, stale events, timestamp
duplicates/regressions, finite values, quaternion validity, covariance
validity, and frame IDs. Topic discovery alone is never readiness evidence.
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

## Verdicts and artifacts

Each session is under `.artifacts/runtime/<workflow>-<timestamp>-<pid>/` and
contains process logs, `processes.json`, `state.json`, `monitor.json`,
`samples.jsonl`, `report.json`, and `REPORT.md`. The latest session is exposed
through `.artifacts/runtime/latest`.

The report verdict is one of `PASS`, `FAIL`, `BLOCKED`, `NOT_RUN`, or
`OBSERVATION_COMPLETE`. Dataset PASS requires valid/fresh sensor and LIO
streams, TRACKING, no drops, and complete cleanup. Simulation PASS additionally
requires the fixed offboard scenario and observed PX4 estimator aid topics.
If estimator aid topics are unavailable, the result is explicitly BLOCKED;
it is not inferred from local-position output. Pre-fusion accuracy is
`NOT_AVAILABLE`, and any LIO/PX4 residual is reported as an observed
comparison rather than circular truth.

`make stop` signals only process groups recorded for the latest session. It
does not use global name-based termination and does not affect unrelated ROS,
Gazebo, or PX4 processes.

## Feature inventory

| Feature | Canonical consumer | Decision |
|---|---|---|
| LiDAR/IMU ingestion and synchronization | dataset-check, sim-check | keep |
| deskew and initialization | dataset-check, sim-check | keep |
| LiDAR correction and registration map | LIO product pipeline | keep; map topics disabled in runtime configs |
| corrected and propagated odometry | all runtime workflows | keep |
| covariance and TF | all runtime workflows | keep |
| PX4 ingress, time/frame conversion | sim-check, sim | keep |
| external odometry safety gate | sim-check, sim | simplify to bridge-local health gate |
| supervisor lifecycle/residual framework | none | delete |
| offboard controller | sim-check only | keep as deterministic scenario |
| duplicate observers/reports/profiles | none | delete |
| cleanup | all workflows | keep in process-group runner |
