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
PX4_DIR=$HOME/Dev/Autopilot make external-mode-check
PX4_DIR=$HOME/Dev/Autopilot make external-mode-gui
# optional: choose another isolated pair when running two local sessions
UAV_NAV_ROS_DOMAIN_ID=43 UAV_NAV_XRCE_PORT=8893 \
  PX4_DIR=$HOME/Dev/Autopilot make external-mode-check
# alias:
PX4_DIR=$HOME/Dev/Autopilot make external-mode
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

Standard replay keeps frontier extraction and publication off, even when RViz
is running. To inspect
`/navigation_mapping/visualization/frontier` in RViz, use the explicit debug
profile shown above; `mapping.visualization.publish_frontier` is the single
frontier-debug switch for that session.

`sim-check` starts PX4 SITL, Gazebo, the Micro XRCE-DDS agent, the bridge,
FAST-LIO with `config/runtime/sim.yaml`, and the retained legacy scenario from
`config/runtime/offboard.yaml`. This workflow is odometry smoke coverage, not
the target navigation control interface.

`external-mode-check` starts the same SITL/LIO/mapping stack, publishes a
bounded goal to `/navigation/goal`, requires a successful trajectory from
`navigation_runtime`, then validates the PX4-native External Mode handover and
the resulting `/fmu/in/trajectory_setpoint` stream. It uses
`config/runtime/external_mode_scenario.yaml` and the simulation-only collision
envelope. The default goal targets the LiDAR-observed free sensor cell, and the
scenario replans while External Mode is active so the adapter never consumes a
stale one-shot trajectory. The M1 External Mode node is launched with
`navigation_bringup/px4_external_mode.launch.py` and
`config/runtime/external_mode.yaml`; it registers through `px4_ros2_cpp` and
hands setpoints to PX4 internal controllers. For mission execution, the mode
publishes `/navigation/mission_complete` and hands control to PX4 POSCTL. Neither
the External Mode node nor the mission harness issues LAND, RTL, or disarm. The
external-vision-only profile intentionally has no global position, so POSCTL is
the generic handover target; completion remains a supervisor boundary. `sim`
starts the same stack with
the Gazebo GUI and RViz, but no automatic flight controller;
stopping it produces `OBSERVATION_COMPLETE`, never a flight `PASS`.

### SITL network isolation

Every simulation session is isolated by default on ROS 2 domain `42` and a
dedicated Micro XRCE-DDS UDP agent port `8892`. The runner propagates
`ROS_DOMAIN_ID` to PX4's `UXRCE_DDS_DOM_ID` and to every ROS process, while
`PX4_UXRCE_DDS_PORT` selects the matching local agent. This is the boundary
that prevents another vehicle's absolute `/fmu/*` topics and External Mode
registration (`M40`) requests from entering the test; a ROS namespace alone
would not protect this stack because the PX4 contract uses absolute names.

Override both values with `UAV_NAV_ROS_DOMAIN_ID` and `UAV_NAV_XRCE_PORT`, or
pass `--ros-domain-id` and `--xrce-port` to `tools/runtime/runner.py`. The
chosen values are recorded in each session's `runtime.json`, so a benchmark
can be audited and reproduced. Do not reuse an XRCE port while another local
PX4 SITL is running.

## GUI External Mode

`make external-mode-gui` is the GUI version of the automated mission workflow.
It starts the same PX4 SITL, Gazebo GUI, RViz, FAST-LIO, mapping, and PX4
External Mode node, then runs the same automated scenario as
`make external-mode-check`. The shorter `make external-mode` command is an
alias. Select a built-in map and its matching static mission with
the canonical scene knobs:

With `MANUAL_TAKEOFF=1`, the harness sends neither ARM nor TAKEOFF. Select
Takeoff and arm manually in PX4/QGC; after the vehicle is airborne, stable,
and LIO odometry is fresh, the harness activates External Mode and continues
the configured mission automatically.

```bash
make build
PX4_DIR=$HOME/Dev/Autopilot make external-mode-gui
MAP_SCENE=structured_obstacle TEST_CASE=detour PX4_DIR=$HOME/Dev/Autopilot make external-mode-gui
MAP_SCENE=long_route TEST_CASE=positive MOTION_PRESET=nominal MAP_SEED=0 PX4_DIR=$HOME/Dev/Autopilot make external-mode-gui
MANUAL_TAKEOFF=1 MAP_SCENE=long_route TEST_CASE=positive MOTION_PRESET=nominal MAP_SEED=0 PX4_DIR=$HOME/Dev/Autopilot make external-mode-gui
MAP_SCENE=tunnel TEST_CASE=degenerate PX4_DIR=$HOME/Dev/Autopilot make external-mode-gui
MAP_SCENE=clutter MAP_SEED=11 PX4_DIR=$HOME/Dev/Autopilot make external-mode-gui
```

The runner passes the selected mission file to the External Mode node. The
default `sanity_open` scene and each variant resolve to a matching mission
through `config/runtime/map_profiles.yaml`; legacy `MAP_PROFILE` names remain
accepted for compatibility. Mission YAML is loaded once at node startup and
is immutable during the flight.

The GUI runner automatically performs this sequence after the processes are
ready:

```text
arm in PX4 ground-safe mode
  -> takeoff and settle
  -> require 1 s of fresh propagated LIO odometry
  -> activate External Mode once
  -> execute the selected mission
  -> brake safety stops and hand over to PX4 POSCTL
```

The report distinguishes `COMPLETE`, `PAUSED_SAFETY_STOP`,
`ABORTED_OPERATOR`, `FAILED_COMPONENT`, and PX4 `failsafe_seen`. A safety stop
is a verified brake followed by POSCTL, not a mode failure and not a landing
request. The pre-takeoff throw-away External Mode activation was removed so a
rejected disarmed activation cannot contaminate the airborne mission cycle.
The harness retries ARM only every 5 s and records PX4's authoritative ACK.
It does not gate the retry on `VehicleStatus.pre_flight_checks_pass`, because
that aggregate bit can remain false until the successful arm in this
external-vision configuration; the slow cadence avoids a command flood while
EKF heading/health converges.

The harness owns only arm/takeoff for this simulation workflow. The product
External Mode node executes navigation and requests the POSCTL handover; no
component sends LAND or disarm.

Useful inspection commands from another terminal:

```bash
make status
ros2 topic echo /lio/diagnostics
ros2 topic echo /navigation/trajectory
ros2 topic echo /navigation_mapping/visualization/occupied
ros2 topic echo /fmu/in/trajectory_setpoint
```

Stop the entire workspace-owned GUI session with:

```bash
make stop
```

Use `make external-mode-check MAP_PROFILE=<profile>` for the same automated
acceptance verdict without GUI rendering. The headless workflow remains
unchanged; the GUI workflow only adds Gazebo/RViz visibility.

The manual-control policy is deliberately different between these workflows:

| Workflow | `COM_RC_IN_MODE` | Manual input |
|---|---:|---|
| `sim-check` | `4` | disabled; legacy smoke scenario only |
| `external-mode-check` | `4` | automated planner-backed External Mode scenario |
| `sim` | `1` | MAVLink joystick from QGC virtual joystick or a physical joystick |

The direct PX4 launcher defaults to mode `4`. For a manually controlled direct
session, set `PX4_PARAM_COM_RC_IN_MODE=1` explicitly before launching it.

### Long mission with three route columns

The deterministic `long_three_pillars` profile is the stress benchmark for
continuous receding-horizon navigation. It contains many non-route texture
features, while exactly three cylinders (`long_three_pillar_01..03`) intersect
the WP0→WP1 long leg. The mission is 48 m to a far waypoint (longer than the
observed local horizon), followed by 5 m/7 m/5 m orthogonal pass-through turns.
The runner selects a 4 m local target, keeps the planner fail-closed in
Unknown space, and records every replanning/safety decision.

```bash
source /opt/ros/jazzy/setup.bash
export UAV_NAV_ROS_DOMAIN_ID=77 UAV_NAV_XRCE_PORT=8897
PX4_DIR=$HOME/Dev/Autopilot python3 tools/runtime/runner.py \
  external-mode-check --map-profile long_three_pillars \
  --ros-domain-id 77 --xrce-port 8897
```

The session directory printed by the runner contains `scenario.json`,
`benchmark_metrics.json`, `samples.jsonl`, and the self-contained `REPORT.html`.
The report distinguishes route columns from texture-only objects, plots the
ground-truth flight path and committed local paths, and reports mission time,
replan count, local horizon, cross-track error, speed, LIO residual, collision
count, and minimum vehicle clearance.

## Configuration

| File | Owner | Purpose |
|---|---|---|
| `config/runtime/common.yaml` | monitor/report | stream rates, freshness thresholds, and workflow timeouts |
| `config/runtime/dataset.yaml` | dataset runner | real AIST timing, frames, extrinsic, input QoS |
| `config/runtime/mapping.yaml` | navigation runtime | world-model, visualization, collision, and planner parameters |
| `config/runtime/sim.yaml` | simulation runner | Gazebo timing, frames, extrinsic, external odometry enablement |
| `config/runtime/external_mode.yaml` | PX4 External Mode node | trajectory, mission-complete, and freshness contract |
| `config/runtime/offboard.yaml` | headless simulation | one deterministic legacy offboard trajectory |
| `config/runtime/external_mode_scenario.yaml` | external-mode-check | planner-backed bounded goal and handover timing |

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
estimator or host stall stops publication fail-closed. Its recovery history is
validated with the invariant

```text
imu_history_duration > maximum_correction_age
```

so replay can retain a timestamp bracket for every accepted correction age.
The retained one-second IMU history remains recovery-only; no simulator truth
is ever substituted for a missing correction.

## Verdicts and artifacts

Each session is under `.artifacts/runtime/<workflow>-<timestamp>-<pid>/` and
contains process logs, `processes.json`, `state.json`, `monitor.json`,
`samples.jsonl`, `report.json`, and `REPORT.md`. The latest session is exposed
through `.artifacts/runtime/latest`.

The report verdict is one of `PASS`, `FAIL`, `BLOCKED`, `NOT_RUN`, or
`OBSERVATION_COMPLETE`. Dataset PASS requires valid/fresh sensor and LIO
streams, TRACKING, no drops, and complete cleanup. Simulation smoke PASS
requires the selected scenario's real PX4 output/status streams and
the LIO-to-PX4 frame/timestamp contract. No unsupported aid-source topic is
used. PX4 status flags are observational telemetry, while accuracy is
calculated against the separate simulator truth.

## External mission planning status

The validated mission lifecycle is:

```text
takeoff/airborne supervisor
  -> activate External Mode
  -> static mission YAML -> correlated goals -> PVA/PV/V setpoints
  -> mission_complete event
  -> PX4 POSCTL handover (mission checkpoint retained for reactivation)
```

The mission event is a notification boundary, not a flight action. This is
required because the external-vision-only simulation profile disables GPS and
PX4 therefore rejects generic Loiter handover when its global-position health
requirement is not met.

The current planning execution contract is deliberately conservative:

```text
nominal A* plan (Inflated + KnownFree)
  -> commit as the active trajectory
nominal plan failure
  -> known-free safety route, role=SAFETY
  -> if no route, known-free braking stop, role=SAFETY
  -> retry the same correlated goal only after a braking stop
  -> fail External Mode if no safe candidate or retry succeeds
```

`SAFETY` route may count as waypoint progress because it is a known-free route;
`SAFETY` braking stop never counts as waypoint progress and is required to end
at zero velocity and acceleration. The runtime executes a nominal path through
unknown cells only when the explicit simulation-only `DUAL_PLANNING=1` gate is
enabled. The default and real profiles remain known-free-only.

The simulation `allow_unknown_start` exception creates a virtual `KnownFree`
overlay over the current vehicle footprint to handle the LiDAR body shadow.
Its radius is exactly the simulation collision envelope (`0.32 + 0.05 = 0.37
m`). It converts only `Unknown`; `Occupied` evidence always wins, cells outside
the footprint remain fail-closed, and the overlay is never written into the
persistent ROG-Map.

The replan timer remains bounded at 5 Hz. A committed trajectory is reused on
unchanged revisions and revalidated against the remaining corridor when the
WorldModel revision changes. A verified braking stop is latched for the active
request and can only be replaced after verifier rejection, never by restarting
nominal execution. Full replanning is reserved for a changed goal, tracking
error beyond `navigation.replan_tracking_error_m`, or an invalidated corridor.
The planning report exposes `trajectory_revalidation_count`,
`trajectory_revalidation_failure_count`, `trajectory_reuse_count`, and
`full_replan_count` alongside the existing planner counters.

Rolling replans use an explicit time contract rather than resetting the PX4
adapter phase on every message. A successful `PlannedTrajectory` carries a
monotonic `trajectory_id`, its `parent_trajectory_id`, and `valid_from`. When a
map update arrives while a previous path is active, the runtime evaluates the
old sampled trajectory at `t_switch = t_now + switch_delay_s`, uses that `(p,v,a)` as
the new initial state, and asks PX4 to atomically promote the replacement only
at `valid_from`. The adapter keeps the old trajectory until promotion. Runtime
verification and PX4 consume the same sampled PVA contract; the planner's
bounded polynomial remains the source of the smooth samples.

Mission waypoints have an explicit `behavior`: `pass_through` for intermediate
points and `stop` for terminal/inspection points. A pass-through goal carries
the next waypoint in `NavigationGoal`; the planner uses it to seed a bounded
terminal tangent and the mission controller advances as soon as the acceptance
radius is crossed. Legacy `hold_s > 0` entries remain stop points for backward
compatibility, while new missions should declare the behavior explicitly.

The runtime also applies a one-dimensional visibility governor before planning:
for a measured motion direction, let `d_free` be the known-free inflated
horizon. The commanded speed cap is the largest `v` satisfying

`v²/(2 a_decel) + v t_latency + d_margin <= d_free`.

This does not replace A* or collision verification; it reduces speed early
when sensing gives only a short stopping envelope. The cap and measured
free-horizon are published in planner diagnostics for benchmark comparison.

## Mission stress profiles

The External Mode runner keeps one scenario harness and varies only the world
and static mission file:

| Profile | Purpose | Expected result |
|---|---|---|
| `open` | baseline textured geometry | mission complete |
| `pillar` | obstacle detour and ground-truth clearance | mission complete |
| `corridor` | narrow known-free corridor | mission complete without limit violation |
| `speed` | higher velocity/acceleration envelope | mission complete; setpoint cap respected |
| `long_open` | long-distance sliding map with asymmetric acceleration/deceleration | diagnostic baseline; current run loses LIO before mission completion |
| `long_open_slow` | same sparse map at reduced speed/acceleration | LIO stays TRACKING; current run still fails at waypoint 1 during safety retry |
| `long_featured` | 53 m route at z=2.8–3.0 m with six low pillars, roadside trees, six non-periodic texture panels and asymmetric scaffold | LIO feature coverage/clearance diagnostic; GUI run must pass all five waypoints |
| `occlusion` | revealed obstacle and dual-planning telemetry | experimental; no flight-policy promotion |
| `no_path` | sealed wall with unreachable first waypoint | brake, `PAUSED_SAFETY_STOP`, POSCTL, GUI remains alive |

The long-map A/B pair is intentionally diagnostic. “Texture” means LiDAR
geometric structure (edges, corners, depth variation), not SDF visual colour.
`long_open_slow` changes only the motion envelope; `long_featured` changes
the static geometry and its ground-truth collision model. The featured route
keeps the setpoints in the 2–3 m band and places discrete, non-periodic
vertical surfaces 4--7 m to either side, so the MID-360 sees returns without
creating a continuous wall. Its final waypoint is x=53 m on the far side of
the obstacle field. A featured map that keeps LIO TRACKING while the sparse
fast map loses it is evidence that geometric observability matters. The LIO
failure reason, filtered-point metrics, stream gaps and full waypoint order
must be recorded with every GUI run.

`no_path` is not a successful mission. The acceptance contract is planner
failure plus structured `PAUSED/SAFETY_STOP` status, verified braking, and
POSCTL handover. LAND/disarm and generic `ModeCompleted(result=100)` are not
part of this path. A mission timeout or continued nominal setpoint publication
is a test failure.

Every stress profile also subscribes to simulator ground-truth odometry. The
harness expands each SDF obstacle by the configured UAV collision radius and
records `minimum_collision_clearance_m`, `collision_count`, and
`collision_event_count` in `scenario.json`. The configured minimum clearance
margin is 0.10 m after subtracting the vehicle collision radius; a breach of
that margin or a collision fails the scenario, independent of LIO state, so an
estimator failure cannot hide a physical collision. The latest long-distance
runs remain known failing gates: the sparse fast baseline reaches `LOST`, the
slow sparse run stops at waypoint 1 after safety fallback retries, and the
featured run stops before waypoint 3 despite LIO remaining `TRACKING`. They
must not be reported as successful mission benchmarks.

## Localization root-cause evidence

FAST-LIO reports a normalized translational information matrix derived from
accepted point-to-plane normals. A correction is usable only when
`lambda_min / lambda_max` is finite and meets
`registration.minimum_translation_observability_ratio` (0.01 in the
canonical profiles). An invalid correction is rolled back, excluded from the
registration map, and propagated as an unhealthy navigation state; External
Mode then publishes a stationary velocity setpoint during handover and does
not issue flight actions.

The `no_path` profile is accepted only after its sealed wall is observed in
both raw LiDAR and occupied-voxel evidence. The simulation watchdog aligns
ground truth only inside the test harness and terminates on a sustained
position residual above 0.5 m or velocity residual above 1.0 m/s. This keeps a
planner result from being treated as safe when the localization frame has
already diverged.

## RViz products

`make sim` and `make replay DATASET=<name>` start RViz with the project config.
It uses `lio_odom` as fixed frame and shows the canonical TF tree plus
`/lio/odometry_corrected`. Product profiles do not publish registered-point or
local-map debug clouds; those serializers must stay in an explicit debug-only
workflow if reintroduced.

`make stop` signals only process groups recorded for the latest session. It
does not use global name-based termination and does not affect unrelated ROS,
Gazebo, or PX4 processes.

`make clean` removes stale profiling/sanitizer `build-*`, `install-*`, and
`log-*` variants, runtime artifacts, pytest/Python caches, the mapper's
generated vendor log, the colcon symlink manifest, and VS Code browse indexes.
It preserves the current canonical Release `build/` and `install/` trees, the
Python virtual environment, and project editor settings.

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
| offboard controller | sim-check only | retained legacy smoke scenario; not product control |
| duplicate observers/reports/profiles | none | delete |
| cleanup | all workflows | keep in process-group runner |
