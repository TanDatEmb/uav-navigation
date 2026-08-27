# Runtime validation

This is the canonical guide for running the current workflows and reading
their evidence. The runtime has one runner, one monitor, and one public report
tool.

## Build and tests

```bash
make build
make test
```

`make test` covers the ROS build/test helpers plus the Python runtime tests.
It is not an end-to-end flight verdict.

## Workflows

```bash
make replay DATASET=<name> RATE=1.0
make dataset-check DATASET=<name> RATE=1.0
PX4_DIR=$HOME/Dev/Autopilot make sim-check
PX4_DIR=$HOME/Dev/Autopilot make external-mode-check
PX4_DIR=$HOME/Dev/Autopilot make external-mode-gui
PX4_DIR=$HOME/Dev/Autopilot make sim
```

`replay` is the dataset-check alias with RViz enabled. `sim-check` is a
retained headless offboard smoke workflow. `external-mode-check` is the
product planner acceptance workflow. `external-mode-gui` runs the same
mission with Gazebo and RViz; `external-mode` is its Make alias. `sim` starts
an interactive manual-control session and does not run an automatic mission.

The dataset runner and SITL runner may run in parallel after a stable build:
runtime sessions share the read-only install lock, while builds remain
exclusive. Dataset ROS 2 defaults to domain `43`; SITL defaults to domain `42`.
Override the dataset domain explicitly with `DATASET_ROS_DOMAIN_ID=<id>` when
running more than one isolated dataset session. Do not start a build while
either runtime session is live.

The product launch entrypoint is:

```bash
ros2 launch navigation_bringup avoidance_mission.launch.py \
  config_file:=$PWD/config/runtime/mapping.yaml \
  mission_file:=$PWD/config/runtime/missions/long_three_pillars.yaml \
  use_sim_time:=true
```

## Scenarios

`external-mode-check` accepts the following current selectors:

| Variable | Values | Meaning |
|---|---|---|
| `MAP_SCENE` | `sanity_open`, `structured_obstacle`, `long_route`, `tunnel`, `clutter`, `planner_negative` | world/mission family |
| `TEST_CASE` | `positive`, `degenerate`, `detour`, `no_path` | expected behavior |
| `MOTION_PRESET` | `nominal`, `slow`, `fast` | motion limits |
| `MAP_SEED` | integer | deterministic clutter variation |
| `SPEED_CAP_MPS` | number | temporary per-run cap |

Examples:

```bash
MAP_SCENE=sanity_open make external-mode-check
MAP_SCENE=structured_obstacle TEST_CASE=detour make external-mode-check
MAP_SCENE=long_route MOTION_PRESET=slow make external-mode-check
MAP_SCENE=clutter MAP_SEED=11 make external-mode-check
MAP_SCENE=planner_negative TEST_CASE=no_path make external-mode-check
```

`MAP_PROFILE` is a legacy compatibility alias. A `no_path` run is expected to
fail closed and is not a mission `PASS`.

## Current runtime graph

```text
/lidar/points + /lidar/imu
  -> FAST-LIO
  -> /lio/odometry_corrected
  -> /lio/odometry_propagated
  -> /lio/mapping_observation (RegisteredScan: cloud + corrected pose)
  -> navigation_runtime_node
  -> /navigation/navigation_command
  -> px4_navigation_external_mode
  -> /fmu/in/trajectory_setpoint
```

The planner node also consumes `/navigation/goal` and
`/navigation/mode_status`, and publishes `/navigation/diagnostics`. ROG-Map
and planner backend are constructed directly in `navigation_runtime_node`; no separate
mapping process or snapshot/bundle transport exists.

## Topics and frames

| Class | Topic | Type | Role |
|---|---|---|---|
| sensor | `/lidar/points` | `sensor_msgs/msg/PointCloud2` | LiDAR input |
| sensor | `/lidar/imu` | `sensor_msgs/msg/Imu` | IMU input |
| LIO | `/lio/odometry_corrected` | `nav_msgs/msg/Odometry` | corrected estimator output; embedded in `RegisteredScan` |
| LIO/planner | `/lio/odometry_propagated` | `nav_msgs/msg/Odometry` | active navigation state |
| planner input | `/lio/mapping_observation` | `navigation_contracts/msg/RegisteredScan` | atomic registered cloud + corrected pose input |
| LIO control health | `/lio/health` | `navigation_contracts/msg/EstimatorHealth` | typed estimator gate, epoch and timestamp provenance |
| LIO diagnostics | `/lio/diagnostics` | `diagnostic_msgs/msg/DiagnosticArray` | observability only; never a control input |
| navigation goal | `/navigation/goal` | `navigation_contracts/msg/NavigationGoal` | waypoint request |
| navigation status | `/navigation/mode_status` | `navigation_contracts/msg/NavigationModeStatus` | mission/mode evidence |
| planner output | `/navigation/navigation_command` | `navigation_contracts/msg/NavigationCommand` | PVA command stream plus epoch/world/bundle provenance |
| planner health | `/navigation/diagnostics` | `diagnostic_msgs/msg/DiagnosticArray` | planner backend counters/status |
| mission event | `/navigation/mission_complete` | `std_msgs/msg/Bool` | completion notification |
| PX4 input | `/fmu/in/vehicle_visual_odometry` | `px4_msgs/msg/VehicleOdometry` | simulation EV input |
| PX4 setpoint | `/fmu/in/trajectory_setpoint` | `px4_msgs/msg/TrajectorySetpoint` | PX4 boundary |
| evaluation | `/sim/ground_truth/odometry` | `nav_msgs/msg/Odometry` | truth only; never control input |

The planner frame is `lio_odom`; the LIO/PX4 state pair is
`lio_odom -> base_link`. The external bridge performs the ENU/FLU to NED/FRD
conversion at the PX4 boundary.

## RViz

The project profile is `src/navigation_bringup/rviz/fast_lio.rviz`. It uses
fixed frame `lio_odom` and shows:

- TF and a grid;
- `/lio/registered_points` enabled;
- `/lio/odometry_propagated` enabled;
- `/lio/odometry_corrected` disabled for optional comparison.

The profile does not reference `/navigation_mapping/visualization/*`,
`/navigation/visualization/*`, local-map debug clouds, or planned paths.
Those old topics are not part of the current runtime contract.

## Reports and artifacts

Sessions live under `.artifacts/runtime/<workflow>-<timestamp>-<pid>/` and are
local evidence. Typical files are `scenario.json`, `scenario_config.yaml`,
`samples.jsonl`, `monitor.json`, `runtime.json`, `processes.json`,
`report.json`, and `REPORT.html`.

The only public report tool is `tools/runtime/report.py`; the runner calls it
at session finalization. For a manual rebuild:

```bash
python3 tools/runtime/report.py \
  --session .artifacts/runtime/<workflow>-<timestamp>-<pid> \
  --workflow external-mode \
  --config config/runtime/sim.yaml \
  --workspace "$PWD" \
  --px4-dir "$HOME/Dev/Autopilot"
```

The report writes only `report.json` and `REPORT.html`. There is no
`REPORT.md` artifact.

## Acceptance gates

The report separates the scenario's observed outcome from quality gates. For
a positive mission, `COMPLETE` requires an observed mission-complete event and
complete waypoint-acceptance evidence. It is still rejected if the report
finds invalid/stale streams, a cross-track threshold breach, collision,
failsafe, missing required samples, or failed cleanup.

For `fail_closed` scenarios, non-completion is expected. The report must show
the intended fail-closed outcome, verified safety behavior, and no physical
collision; it must not label the scenario as a successful mission.

Useful report fields include stream counts/rates, source timestamp gaps,
freshness and validity failures, waypoint indices, mission completion, truth
clearance/collisions, PX4 failsafe state, planner counters, and timing
distributions when diagnostics provide them. Current planner backend logs/diagnostics
can provide input conversion, ROG-Map update, planner solve, command publish,
input-lock, planner-cycle, and end-to-end samples. Missing timing fields are
`NOT_AVAILABLE`, never zero-filled.

Execution-specific optimizer timings are counted only when the diagnostic
validity flag confirms that the optimizer ran in that cycle. A later setup-only
cycle may carry the last values for continuity; those values are retained in
the raw trace but excluded from timing distributions. Always compare the
reported sample count with the number of executed solves.

When comparing performance, keep three measurements separate: diagnostic phase
duration over simulation time, wall duration divided by source simulation
duration, and scheduler/transport gaps. The diagnostic timeline is anchored to
observed simulation stamps and shows samples; it is not a direct wall-clock
throughput measurement. Use repeated same-scenario runs and report the full
distribution (at least p50/p95/max) before changing planner initialization,
limits, deadlines, or acceptance thresholds.

## Inspection commands

```bash
make status
ros2 topic echo /lio/health
ros2 topic echo /lio/diagnostics
ros2 topic echo /navigation/diagnostics
ros2 topic echo /navigation/navigation_command
ros2 topic echo /navigation/mission_complete
ros2 topic echo /fmu/in/trajectory_setpoint
```

`make stop` stops only process groups recorded for the workspace session.
`make clean` removes stale runtime/build variants, logs, caches, and runtime
artifacts while preserving the canonical Release `build/` and `install/`
trees.
