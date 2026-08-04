# PX4/Gazebo runtime asset

`run_px4_mid360.sh` is the PX4/Gazebo child process used by
`tools/runtime/runner.py`. It does not own a workflow, report, profile, or
cleanup policy. The runner owns all child process groups and writes the single
session artifact under `.artifacts/runtime/`.

The only simulation configuration is `config/runtime/sim.yaml`; the bridge
asset is `src/uav_simulation/bridge/px4_mid360_bridge.yaml`.
