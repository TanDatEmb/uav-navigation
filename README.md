# UAV Navigation

ROS 2 Jazzy workspace for LiDAR–IMU odometry, propagated odometry, and PX4
external-vision integration. Runtime validation has three entrypoints:

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

`make sim-check` is headless and runs the deterministic offboard trajectory.
`make sim` starts the same PX4/Gazebo/LIO stack with the Gazebo GUI and waits
for manual control; it has no automatic flight scenario and does not start
RViz. `make replay` is an alias for the dataset workflow and also does not
start RViz. Set `PX4_DIR` when
the PX4 checkout is outside `$HOME/Dev/Autopilot`.

Prepared datasets live outside the repository under `UAV_NAV_DATA_HOME` (or
the platform data directory). Runtime sessions and reports are written to
`.artifacts/runtime/`; they are local evidence and are not committed. Read
[runtime validation](docs/runtime_validation.md) for prerequisites, topic
contracts, verdict semantics, and troubleshooting.

The workspace contains the estimator core, ROS adapters, PX4 bridge, Gazebo
assets, and the four canonical runtime YAML files under `config/runtime/`.
There is no runtime Git-SHA compatibility gate.
