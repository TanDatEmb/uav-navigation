# PX4/Gazebo runtime asset

`run_px4_mid360.sh` is the PX4/Gazebo child process used by
`tools/runtime/runner.py`. It does not own a workflow, report, profile, or
cleanup policy. The runner owns all child process groups and writes the single
session artifact under `.artifacts/runtime/`.

The only simulation configuration is `config/runtime/sim.yaml`; the bridge
asset is `src/uav_simulation/bridge/px4_mid360_bridge.yaml`.

Manual-control profiles are selected by the runtime runner:

- `make sim-check` uses `COM_RC_IN_MODE=4` and disables manual control for the
  deterministic OFFBOARD acceptance run.
- `make sim` uses `COM_RC_IN_MODE=1` and accepts MAVLink joystick input from QGC
  virtual joystick or a physical joystick.

When `run_px4_mid360.sh` is launched directly, it defaults to mode `4`. Set
`PX4_PARAM_COM_RC_IN_MODE=1` explicitly for an interactive manual session.
