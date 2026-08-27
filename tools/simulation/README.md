# PX4/Gazebo runtime asset

`run_px4_mid360.sh` is the PX4/Gazebo child process used by
`tools/runtime/runner.py`. It does not own a workflow, report, profile, or
cleanup policy. The runner owns all child process groups and writes the single
session artifact under `.artifacts/runtime/`.

The only simulation configuration is `config/runtime/sim.yaml`; the bridge
assets are `src/uav_simulation/bridge/px4_mid360_control_bridge.yaml` and
`src/uav_simulation/bridge/px4_mid360_lidar_bridge.yaml`; PointCloud conversion
is process-isolated from `/clock` and IMU transport.

The simulation harness makes Gazebo `/clock` the only simulation-time source and
sets `UXRCE_DDS_SYNCT=0`; this avoids mixing wall-clock DDS synchronization into
the simulation epoch. A realtime hardware deployment must use
`use_sim_time=false` and keep PX4 uXRCE-DDS time synchronization enabled
(`UXRCE_DDS_SYNCT=1` or the firmware default). The navigation bridge never adds
`TimesyncStatus.estimated_offset` locally. Realtime validation must capture the
timesync source/offset/round-trip fields and the resulting VehicleOdometry
timestamp relation to the ROS system clock.

Manual-control profiles are selected by the runtime runner:

- `make sim-check` uses `COM_RC_IN_MODE=4` and disables manual control for the
  deterministic OFFBOARD acceptance run.
- `make sim` uses `COM_RC_IN_MODE=1` and accepts MAVLink joystick input from QGC
  virtual joystick or a physical joystick.

When `run_px4_mid360.sh` is launched directly, it defaults to mode `4`. Set
`PX4_PARAM_COM_RC_IN_MODE=1` explicitly for an interactive manual session.
