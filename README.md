# UAV Navigation

ROS 2 Jazzy workspace for FAST-LIO odometry, mapping and local planning,
PX4 External Mode and Gazebo Harmonic simulation.
Beta scope is SITL only; hardware flight is not qualified.

## Architecture

Branch `rebuild/v2` rebuilds the stack against a new architecture. The target
design (components, ownership, interfaces, build order) is in
[System design](docs/architecture/SYSTEM_DESIGN.md). The `main`-only runtime,
execution, PX4 and bringup packages were removed in S0; the kept packages
(`navigation_*`, `fast_lio_core`, vendors, simulation) are reference inputs for
later slices.

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
make test    # colcon test for $(PKGS), then colcon test-result --verbose
make gate    # tools/uavnav/gate.sh all: static, python, ros; ends GATE_RESULT=PASS|FAIL
make clean   # prints what to remove (log/); deletes nothing
```

`PKGS` defaults to `uavnav_core uavnav_interfaces`; override it with
`make build PKGS="..."`. `tools/uavnav/gate.sh static|python|ros|all` runs a
single stage. `build/` and `install/` are a shared incremental build and are
kept.

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
`src/estimation/ikfom_vendor`, `src/estimation/ikd_tree_vendor`,
`src/mapping/rog_map_vendor`, `src/planning/navigation_planning_backend` and
`src/external/` (`UPSTREAM.md` and `LICENSE` files). Some are GPL-2.0; obtain a
license review before distributing binaries or sources.
