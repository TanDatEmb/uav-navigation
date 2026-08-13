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
make sim
make status
make stop
make clean
```

`make sim-check` is headless and runs the deterministic offboard trajectory with
`COM_RC_IN_MODE=4`, so manual input cannot interfere with acceptance. `make
sim` starts the same PX4/Gazebo/LIO stack with the Gazebo GUI and RViz, waits for
manual control, and uses `COM_RC_IN_MODE=1` for QGC virtual joystick or a
physical joystick. It has no automatic flight scenario. Set `PX4_DIR` when the
PX4 checkout is outside `$HOME/Dev/Autopilot`.

Prepared datasets live outside the repository under `UAV_NAV_DATA_HOME` (or
the platform data directory). Runtime sessions and reports are written to
`.artifacts/runtime/`; they are local evidence and are not committed. `make
clean` removes build/install/log trees, runtime artifacts, profiling outputs,
Python caches, vendor runtime logs, and VS Code browse indexes. It preserves
`.venv/` and project editor settings.

The workspace contains the estimator core, ROS adapters, PX4 bridge, Gazebo
assets, and the canonical runtime YAML files under `config/runtime/`:
`common.yaml`, `dataset.yaml`, `mapping.yaml`, `sim.yaml`, and `offboard.yaml`.

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

There is no runtime Git-SHA compatibility gate.
