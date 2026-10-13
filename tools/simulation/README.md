# PX4/Gazebo runtime asset

`run_px4_mid360.sh` starts Gazebo Harmonic with the project-owned
`x500_mid360` model and attaches standard PX4 x500 SITL (autostart 4001) to it.
It is one child process of the SITL harness `tools/uavnav/sitl_hover.sh`
(`make sitl-probe`), which owns every process group, the run directory under
`log/sitl/` and the teardown. The script does not own a workflow, report or
cleanup policy.

Environment:

- `UAVNAV_PX4_PROFILE` (required): the localization profile file,
  `config/px4/profiles/gps.yaml` or `config/px4/profiles/no_gps.yaml`. It is
  validated by `tools/uavnav/px4_profile.py` against the decided EKF2 values
  (D20, D28, D33/O23) before anything starts; an invalid profile exits 2.
- `UAVNAV_SPAWN_YAW_RAD` (optional): ENU spawn yaw of `x500_mid360`, applied
  with the world's `set_pose` service before PX4 attaches (F34 check, D33/O24).
- `GZ_GUI=0`: headless Gazebo server (the harness always sets it).
- `SESSION_DIR`: the run directory. The Gazebo log goes to
  `$SESSION_DIR/logs/gazebo.log`; PX4 runs in `$SESSION_DIR/px4`, a fresh
  working directory, so no parameter of an earlier run is carried over.

`run_px4_mid360.sh --check-only` validates the PX4 binary and rootfs, Gazebo,
the world and model files, the profile and the spawn yaw, prints them, and
starts nothing.

PX4 parameters are the profile (EKF2, tier c) plus the harness-only values of
D33/O23: `UXRCE_DDS_SYNCT=0`, `SIM_GZ_EN_ODOM=0`, `SIM_GZ_EN_GPS=1`,
`COM_RC_IN_MODE=1` (MAVLink-only manual control, which carries the ROS stick
stream on `/fmu/in/manual_control_input`), and `NAV_DLL_ACT=0` and
`CBRK_SUPPLY_CHK` set by `px4_mid360_startup.sh` after rcS. Every other
parameter keeps the PX4 default. `PX4_PARAM_*` variables inherited from the
calling shell are dropped with a warning.

The bridge assets are
`src/sim/uav_simulation/bridge/px4_mid360_control_bridge.yaml` (`/clock`,
`/lidar/imu`, `/sim/ground_truth/odometry` through `ros_gz_bridge`) and
`src/sim/uav_simulation/bridge/px4_mid360_lidar_bridge.yaml` (the declarative
contract of `/lidar/points`, served by `uav_simulation gz_lidar_bridge`), so
PointCloud conversion is process-isolated from `/clock` and IMU transport.

The harness makes Gazebo `/clock` the only simulation-time source and sets
`UXRCE_DDS_SYNCT=0`; sensor and PX4 time are then the same clock (D29). A
realtime hardware deployment must use `use_sim_time=false` and keep PX4
uXRCE-DDS time synchronization enabled (`UXRCE_DDS_SYNCT=1` or the firmware
default).
