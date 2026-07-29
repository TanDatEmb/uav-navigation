# M1 Gazebo Harmonic harness

Start the harness in one terminal:

```bash
ros2 launch uav_simulation mid360_harness.launch.py
```

It starts actual Gazebo `gpu_lidar` and `imu` sensors, bridges them to
`/lidar/points` (`PointCloud2`) and `/lidar/imu` (`Imu`), publishes simulated
clock, and starts static sensor TF. It deliberately does **not** publish Livox
`CustomMsg` and is not a real Mid-360 dataset or calibration.

In a second terminal, execute a scenario and then check the contract:

```bash
python3 tools/simulation/run_m1_scenario.py yaw --output reports/simulation/yaw_commands.json
python3 tools/simulation/verify_m1_sim_contract.py --output reports/simulation/yaw_contract.json
```

The checker exits non-zero on wrong topic type/frame, missing messages, zero or
non-monotonic timestamps, or missing static TF. Add `--require-dynamic-odom-tf`
only when `start_estimator:=true` is used and a successful correction is expected.
Scenario traces record commands, not ground truth; do not use them to claim
odometry accuracy. Use the resulting report as a gate before manual RViz and
estimator acceptance checks.
