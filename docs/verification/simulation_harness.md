# Gazebo Harmonic M1 sensor harness

The `uav_simulation` harness supplies a deterministic kinematic rig with actual
Gazebo Harmonic `gpu_lidar` and `imu` sensor types. `ros_gz_bridge` maps their
Gazebo messages to `/lidar/points` (`sensor_msgs/msg/PointCloud2`),
`/lidar/imu` (`sensor_msgs/msg/Imu`) and `/clock`. The bridge mapping follows
the Gazebo Harmonic ROS 2 bridge configuration format and supported
PointCloudPacked/IMU pairings. See the [Gazebo ROS 2 integration guide](https://gazebosim.org/docs/harmonic/ros2_integration/)
and [sensor guide](https://gazebosim.org/docs/harmonic/sensors/).

Start it without the estimator first:

```bash
ros2 launch uav_simulation mid360_harness.launch.py
python3 tools/simulation/run_m1_scenario.py static --output reports/simulation/static_commands.json
python3 tools/simulation/verify_m1_sim_contract.py --output reports/simulation/static_contract.json
```

The contract checker is pass/fail and rejects absent/wrong ROS types, insufficient
message count, zero or non-monotonic timestamps, wrong sensor frame IDs, and
missing `base_link -> {imu_link,lidar_link}` static TF. It checks `odom ->
base_link` only when explicitly requested after an estimator correction. The
available command profiles are `static`, `yaw`, `translation`, `vertical`, and
`square`; their JSON traces are command records only, not ground truth.

The rig is neither a calibrated Mid-360 model nor a Livox CustomMsg publisher.
It has no claim of Mid-360 beam pattern, timebase/offset-time semantics, sensor
noise fidelity, calibration, or LIO accuracy. It is an interface/timing/TF gate
before RViz inspection, not M1 acceptance evidence. Use the real-bag workflow
for CustomMsg validation.
