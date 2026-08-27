# UAV Navigation

ROS 2 Jazzy workspace for FAST-LIO odometry, local planning,
PX4 External Mode, Gazebo Harmonic simulation, and runtime evidence reports.

## Product path

The current navigation runtime is one ROS 2 process, `navigation_runtime_node`.
It currently composes the pinned mapping and planning backends directly:

```text
LiDAR + IMU
  -> FAST-LIO
  -> /lio/registered_points + /lio/odometry_propagated
  -> navigation_runtime_node (mapping + planning backends)
  -> /navigation/navigation_command
  -> px4_navigation_external_mode
  -> /fmu/in/trajectory_setpoint
```

`/navigation/diagnostics` is the planner/runtime health surface. Mapping
observation lifecycle accounting is a reusable `navigation_mapping` library;
the world snapshot and backend composition remain internal to the runtime, with
no separate mapping or planning ROS transport yet.

## Common commands

```bash
make build
make test
make replay DATASET=aist-mid360-drive RATE=1.0
make dataset-check DATASET=aist-mid360-drive RATE=1.0
make sim-check
make external-mode-check
make external-mode-gui
make external-mode             # alias for external-mode-gui
make sim
make status
make stop
make clean
```

Set `PX4_DIR` when the PX4 checkout is not at `$HOME/Dev/Autopilot`. The
headless `sim-check` workflow is retained legacy offboard smoke coverage. The
product acceptance path is `external-mode-check` or its GUI equivalent.

The product mission entrypoint is:

```bash
ros2 launch navigation_bringup avoidance_mission.launch.py \
  config_file:=$PWD/config/runtime/mapping.yaml \
  mission_file:=$PWD/config/runtime/missions/long_three_pillars.yaml \
  use_sim_time:=true
```

The same mission YAML is passed to planner backend and PX4 External Mode so dynamic
limits and waypoint behavior cannot diverge between the two sides.

## Scenario selection

The canonical scene variables are `sanity_open`, `structured_obstacle`,
`long_route`, `tunnel`, `clutter`, and `planner_negative`. `TEST_CASE` is one
of `positive`, `degenerate`, `detour`, or `no_path`; `MOTION_PRESET` is one of
`nominal`, `slow`, or `fast`.

```bash
MAP_SCENE=sanity_open make external-mode-check
MAP_SCENE=structured_obstacle TEST_CASE=detour make external-mode-check
MAP_SCENE=long_route MOTION_PRESET=slow make external-mode-check
MAP_SCENE=planner_negative TEST_CASE=no_path make external-mode-check
```

`MAP_PROFILE` remains a compatibility alias for older profile names. A
negative/no-path scenario is expected to fail closed; it is not a successful
mission.

## Runtime evidence

Each session is written outside Git history under
`.artifacts/runtime/<workflow>-*/`. The only public report tool is:

```bash
python3 tools/runtime/report.py \
  --session .artifacts/runtime/<workflow>-<timestamp>-<pid> \
  --workflow external-mode \
  --config config/runtime/sim.yaml \
  --workspace "$PWD"
```

It produces `report.json` and a self-contained `REPORT.html`. `REPORT.md` is
not a runtime artifact. The report separates mission acceptance from process
exit status and records rates, freshness, validity, waypoint evidence,
cross-track quality, collision/failsafe state, and available timing samples.
Missing timing data is reported as unavailable rather than fabricated.

## RViz

The project profile is
`src/navigation_bringup/rviz/fast_lio.rviz`. It uses `lio_odom` as the fixed
frame and shows TF, `/lio/registered_points`, and active
`/lio/odometry_propagated`; `/lio/odometry_corrected` is retained disabled as
an estimator cross-check. The profile intentionally contains no stale map or
planned-path topics.

## Documentation map

- [Runtime validation](docs/runtime_validation.md): commands, topics, verdicts,
  reports, and cleanup;
- [Navigation layers](docs/architecture/navigation_layers.md): current
  estimator-to-PX4 architecture and ownership;
- [Repository layout](docs/architecture/repository_layout.md): current package
  and tool ownership;
- [Frame conventions](docs/architecture/frame_conventions.md): ENU/FLU and
  PX4 NED/FRD conversions;
- [Product architecture and naming](docs/adr/ADR-012-product-owned-tree-and-naming.md):
  canonical tree, ownership, and migration sequence;
- [Continuous waypoint trajectory plan](docs/architecture/continuous_waypoint_trajectory_plan.md):
  staged route look-ahead, piecewise trajectory, sensing evidence, and acceptance plan;
- [ADR index](docs/adr/): durable design decisions;
- [Third-party notices](THIRD_PARTY_NOTICES.md): vendored source and license
  boundaries.

For a design review of this documentation cleanup, see
`docs/repo-documentation-standardization-design-report.docx`.

## Prerequisites

- Ubuntu with ROS 2 Jazzy and Gazebo Harmonic;
- a compatible PX4 checkout for SITL workflows;
- the `src/external/px4_msgs` submodule;
- a prepared dataset outside the repository for `make dataset-check`.

```bash
git submodule update --init --recursive
```
