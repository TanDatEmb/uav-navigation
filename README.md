# UAV Navigation

ROS 2 Jazzy workspace for FAST-LIO odometry, mapping and local planning,
PX4 External Mode, Gazebo Harmonic simulation, and runtime evidence reports.
Beta scope is SITL only; hardware flight is not qualified.

## Architecture

One ROS 2 process, `navigation_runtime_node`, composes the mapping and planning
backends. The product path is:

```text
LiDAR + IMU
  -> FAST-LIO
  -> /lio/mapping_observation + /lio/odometry_propagated + /lio/health
  -> navigation_runtime_node (mapping + planning)
  -> /navigation/navigation_command
  -> px4_navigation_external_mode
  -> /fmu/in/trajectory_setpoint
```

`/navigation/diagnostics` is the planner/runtime health surface. See
[System design](docs/architecture/SYSTEM_DESIGN.md) for ownership and
[Package ownership](docs/architecture/SYSTEM_DESIGN.md) for packages.

## Setup

Requires Ubuntu 24.04 with ROS 2 Jazzy ([install](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html)),
`colcon` and `rosdep`. Optional: a PX4 checkout for SITL workflows (default
`$HOME/Dev/Autopilot`, override with `PX4_DIR`) and a prepared dataset outside
the repository for `make dataset-check`.

```bash
git clone --recurse-submodules <repo-url> && cd uav-navigation
make setup        # checks the host, inits submodules, installs missing apt/rosdep packages
make build
make test
```

`tools/setup.sh --check` only reports what is missing. Python tooling runs on the
system interpreter (`/usr/bin/python3`, 3.12), not in a virtualenv: ROS 2 Jazzy
is installed for it and the only extra Python dependency is `python3-yaml`. The
Makefile unsets `VIRTUAL_ENV`/`PYTHONHOME` and
`tools/runtime/runtime_environment.py` rejects a virtualenv, so run
`deactivate` first if one is active.

## Commands

```bash
make build                 # colcon build; no runtime data required
make test                  # C++/ROS tests, then Python unit tests
make dataset-check DATASET=<name> RATE=1.0   # dataset replay + shadow planning
make replay DATASET=<name> RATE=1.0          # same, with RViz
make run                   # headless automatic External Mode mission
make run-gui               # Gazebo GUI + RViz
make sim                   # interactive PX4/Gazebo/RViz, no auto flight
make status | make stop | make clean
```

Selectors for `make run` / `make run-gui`:

- `MAP_SCENE`: `sanity_open`, `structured_obstacle`, `long_route`, `tunnel`,
  `clutter`, `planner_negative`, `navigation_generalization`;
- `TEST_CASE`: `positive`, `degenerate`, `detour`, `no_path`, `comprehensive`;
- `MOTION_PRESET`: `nominal`, `slow`, `fast`; `MAP_SEED`, `SPEED_CAP_MPS`,
  `MANUAL_TAKEOFF=1` for operator arm/takeoff.

Runs use GPS aiding and planner V/A/J = 5/5/8 from config. The runner defaults
to `TRACKING_EXPERIMENT_MODE=off`. Explicit `relaxed` suppresses finite
tracking and fresh-health responses and is diagnostic-only, qualification-ineligible.
`no_path` must fail closed and is not a successful mission. Qualification
thresholds are in `config/runtime/planning_stability_qualification.yaml`.

The mission entrypoint without the runner:

```bash
ros2 launch navigation_bringup avoidance_mission.launch.py \
  config_file:=$PWD/config/runtime/mapping.yaml \
  mission_file:=$PWD/config/runtime/missions/long_three_pillars.yaml \
  use_sim_time:=true
```

## Runtime evidence

Each session is written outside Git history to
`.artifacts/runtime/<workflow>-*/`. Build the report with:

```bash
python3 tools/runtime/report.py \
  --session .artifacts/runtime/<workflow>-<timestamp>-<pid> \
  --workflow external-mode \
  --config config/runtime/sim.yaml \
  --workspace "$PWD"
```

It produces `report.json` and a self-contained `REPORT.html`. Mission
acceptance is separate from process exit status; missing timing data is
reported as unavailable.

## Documentation

- [Safety contract](docs/safety/runtime_safety_current.md): read before
  changing estimation, mapping, planning, control, PX4 integration, budgets,
  gates or thresholds;
- [System design](docs/architecture/SYSTEM_DESIGN.md): current ownership, interfaces,
  budgets and unimplemented proposals;
- [Roadmap](docs/ROADMAP.md): new evidence-first refactor milestones;
- [Working contract](AGENTS.md): repository rules for contributors and agents.

## Licensing

The root [LICENSE](LICENSE) covers project-owned code only. Vendored and
external components keep their own licenses and provenance, in
`src/estimation/ikfom_vendor`, `src/estimation/ikd_tree_vendor`,
`src/mapping/rog_map_vendor`, `src/planning/navigation_planning_backend` and
`src/external/` (`UPSTREAM.md` and `LICENSE` files). Some are GPL-2.0; obtain a
license review before distributing binaries or sources.
