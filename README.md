# UAV Navigation

ROS 2 Jazzy workspace for LiDAR–IMU odometry, propagated odometry, PX4
external-vision integration, and a local ROG-Map navigation world model.
Runtime validation has these entrypoints:

```bash
make build
make test
make replay DATASET=aist-mid360-drive RATE=1.0
make dataset-check DATASET=aist-mid360-drive RATE=1.0
make sim-check
make external-mode-check
make external-mode-gui
# alias:
make external-mode
make sim
make status
make stop
make clean
```

`make sim-check` is a retained headless odometry smoke workflow and still runs
the legacy deterministic offboard trajectory with `COM_RC_IN_MODE=4`; it is not
the target navigation control path. The M1 node can be launched with:

```bash
ros2 launch navigation_bringup px4_external_mode.launch.py \
  config_file:=$PWD/config/runtime/external_mode.yaml use_sim_time:=true
```

`make external-mode-check` validates the product path in simulation:
`/navigation/goal` → `MappingWorldNode` → immutable `/navigation/world_snapshot` →
`PlanningControllerNode` → `/navigation/trajectory_bundle` →
PX4 External Mode → `/fmu/in/trajectory_setpoint`. It uses the simulation-only
collision envelope and a bounded goal in the LiDAR-observed free sensor cell;
real-flight profiles remain fail-closed until an authoritative vehicle
envelope is provided.

The safety/performance maps are grouped into a small canonical scene set. A
scene selects geometry; `TEST_CASE` selects positive/negative behavior and
`MOTION_PRESET` selects speed/dynamics. Legacy `MAP_PROFILE` aliases remain
available for one compatibility cycle:

```bash
MAP_SCENE=sanity_open make external-mode-check
MAP_SCENE=structured_obstacle make external-mode-check
MAP_SCENE=structured_obstacle TEST_CASE=degenerate make external-mode-check
MAP_SCENE=long_route MOTION_PRESET=slow make external-mode-check
MAP_SCENE=tunnel TEST_CASE=degenerate make external-mode-check
MAP_SCENE=clutter MAP_SEED=11 make external-mode-check
MAP_SCENE=planner_negative TEST_CASE=no_path make external-mode-check
```

`sanity_open` is the baseline, `structured_obstacle` covers detour and
occlusion, and `DUAL_PLANNING=1` is a
simulation-only experiment; the default and real profiles keep unknown space
blocked. `long_route` combines long-distance geometry and speed variants,
`tunnel` combines irregular and smooth degeneracy, and `planner_negative` is
expected to fail closed while LIO remains healthy. `clutter` uses a fixed seed
in CI and alternate seeds for nightly coverage.

`make external-mode-gui` starts Gazebo with its GUI, RViz, FAST-LIO, mapping,
and the External Mode mission node, then automatically runs the same
arm/takeoff/stabilize/wait-for-fresh-LIO/activate/mission/POSCTL-handover scenario used by the
acceptance test. The harness never issues LAND or disarm; arm/takeoff are
supervisor setup actions and the product External Mode node owns POSCTL handover.
The default `sanity_open` scene selects a
matching world and mission YAML from the registry; use
`MAP_SCENE=structured_obstacle TEST_CASE=detour make external-mode-gui` to
exercise the consolidated obstacle scene. Use `make status` to inspect the
session and `make stop` to terminate it. Detailed commands and observability topics are in the
[runtime validation guide](docs/runtime_validation.md#gui-external-mode).

The main low-altitude visual flight is:

```bash
PX4_DIR=$HOME/Dev/Autopilot make external-mode-gui \
  MAP_SCENE=long_route TEST_CASE=positive MOTION_PRESET=nominal MAP_SEED=0
```

It flies at z=2.8--3.0 m from the near end to x=53 m, above/between six low
pillars. Roadside cylinders, non-periodic panels, corners and elevated
features provide physical LiDAR returns; visual colour alone is not treated as
LiDAR texture.

`make
sim` starts the same PX4/Gazebo/LIO stack with the Gazebo GUI and RViz, waits for
manual control, and uses `COM_RC_IN_MODE=1` for QGC virtual joystick or a
physical joystick. It has no automatic flight scenario. Set `PX4_DIR` when the
PX4 checkout is outside `$HOME/Dev/Autopilot`.

Prepared datasets live outside the repository under `UAV_NAV_DATA_HOME` (or
the platform data directory). Runtime sessions and reports are written to
`.artifacts/runtime/`; they are local evidence and are not committed. `make
clean` removes stale profiling/sanitizer `build-*`, `install-*`, and `log-*`
variants, runtime artifacts, Python caches, vendor runtime logs, and VS Code
browse indexes. It preserves the current canonical Release `build/` and
`install/` trees, `.venv/`, and project editor settings.

The workspace contains the estimator core, ROS adapters, PX4 bridge, PX4 v1.17
ROS 2 Control Interface External Mode adapter, Gazebo assets, and the
canonical runtime YAML files under `config/runtime/`.

## Prerequisites

- Ubuntu with ROS 2 Jazzy and Gazebo Harmonic;
- a PX4 checkout with the project-compatible SITL target for `make sim` and
  `make sim-check`;
- the `src/external/px4_msgs` submodule;
- a prepared dataset outside the repository for `make dataset-check`.

Initialize the PX4 message submodule after cloning:

```bash
git submodule update --init --recursive
```

## Documentation

- [Runtime validation](docs/runtime_validation.md): commands, configuration,
  topics, verdicts, artifacts, and cleanup;
- [Navigation layers](docs/architecture/navigation_layers.md): estimator,
  mapping, planning, and dependency boundaries;
- [Repository layout](docs/architecture/repository_layout.md): package and
  ownership map;
- [Third-party notices](THIRD_PARTY_NOTICES.md): vendored source, submodules,
  provenance, and license boundaries.

The target control path is `px4_navigation_external_mode` using the pinned
`px4_ros2_cpp` submodule and PX4 External Mode. The retained
`offboard_scenario.py` is legacy odometry/SITL smoke coverage, not a product
control interface. There is no runtime Git-SHA compatibility gate.
