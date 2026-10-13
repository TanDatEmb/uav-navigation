# UAV Navigation

ROS 2 Jazzy workspace for FAST-LIO odometry, mapping and local planning,
PX4 External Mode and Gazebo Harmonic simulation.
Beta scope is SITL only; hardware flight is not qualified.

## Architecture

Branch `rebuild/v2` rebuilds the stack against a new architecture. The target
design (components, ownership, interfaces, build order) is in
[System design](docs/architecture/SYSTEM_DESIGN.md). The `main`-only runtime,
execution, PX4 and bringup packages were removed in S0; the kept packages
(`navigation_*`, `fast_lio_core`, vendors) are reference inputs for later
slices; the simulation packages are in `src/sim/`.

## Setup

Requires Ubuntu 24.04 with ROS 2 Jazzy ([install](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html)),
`colcon` and `rosdep`.

```bash
git clone --recurse-submodules <repo-url> && cd uav-navigation
make setup        # checks the host, inits submodules, installs missing apt/rosdep packages
make build
make test
```

`tools/setup.sh --check` only reports what is missing. Python tooling runs on the
system interpreter (`/usr/bin/python3`, 3.12), not in a virtualenv: ROS 2 Jazzy
is installed for it and the only extra Python dependency is `python3-yaml`. The
ROS tooling needs that interpreter, so run `deactivate` first if a virtualenv
is active.

## Commands

```bash
make build   # colcon build --packages-up-to $(PKGS), -j3 and 2 workers
make test    # deletes the stale result XMLs of $(PKGS) in build/, then colcon test and colcon test-result --verbose
make sanitize # TSan build and tests of uavnav_core and uavnav_lio_core (setarch -R), then restores the normal build, also on Ctrl-C
make gate    # tools/uavnav/gate.sh all: static, python, ros; ends GATE_RESULT=PASS|FAIL
make clean   # prints what to remove (log/); deletes nothing
make sitl-probe PROFILE=gps  # SITL probe of the S1b assumptions, headless, <= 240 s; not part of test or gate
```

`PKGS` defaults to `uavnav_core uavnav_interfaces fast_lio_core uavnav_lio_core
uavnav_px4_bridge`; override it
with `make build PKGS="..."`. `COLCON_EXTRA` passes extra `colcon build`
arguments, e.g. `COLCON_EXTRA="--cmake-clean-cache"` after a package moves.
`tools/uavnav/gate.sh static|python|ros|all` runs a
single stage. `make sanitize` does not instrument `fast_lio_core` or
`uavnav_px4_bridge`. `build/` and `install/` are a shared incremental build and are
kept.

`make sitl-probe PROFILE=gps|no_gps` (optional `HOVER_MODE=hold`, `DRY_RUN=1`)
runs `tools/uavnav/sitl_hover.sh --probe`: PX4 v1.17 SITL with Gazebo Harmonic
(`tools/simulation/run_px4_mid360.sh`, PX4 in `~/Dev/Autopilot`), MicroXRCEAgent
on UDP 8888, the gz bridges and an rclpy probe, each in its own process group,
bounded at 240 s and always torn down. It refuses to start while a `colcon`
build or test or another SITL runs (`tools/uavnav/sitl_guard.py`). The verdict is
`log/sitl/<UTC stamp>-<profile>-probe/summary.json`: seven checks (mode
accepted, cloud layout, non-finite ratio, IMU rate, scan stamp vs IMU, sensor
time vs `/clock`, PX4 time vs `/clock`) and every probe phase ok. It needs
`make build PKGS="uav_simulation uav_description px4_msgs"` first. The PX4
localization profiles are `config/px4/profiles/{gps,no_gps}.yaml`, validated by
`tools/uavnav/px4_profile.py`. The simulation packages live in `src/sim/`.

## Documentation

Branch `rebuild/v2` rebuilds the stack against a new architecture; `main` is
the reference baseline only.

- [System design](docs/architecture/SYSTEM_DESIGN.md): the architecture map
  (components, state machines, interfaces, conventions, build order);
- [Decisions](docs/architecture/DECISIONS.md): agreed decisions, open
  questions and verified facts behind the design;
- [Traceability](docs/TRACEABILITY.md): requirement → design → work package →
  status checklist;
- [Code-quality report](docs/analysis/P9_REPORT.md): evidence from the
  baseline review;
- [Working contract](AGENTS.md): repository rules for contributors and agents.

## Licensing

The root [LICENSE](LICENSE) covers project-owned code only. Vendored and
external components keep their own licenses and provenance, in
`src/vendor/ikfom_vendor`, `src/vendor/ikd_tree_vendor`,
`src/mapping/rog_map_vendor`, `src/planning/navigation_planning_backend` and
`src/external/` (`UPSTREAM.md` and `LICENSE` files). Some are GPL-2.0; obtain a
license review before distributing binaries or sources.
