# Dataset inspection

- Dataset: `swarm_lio2_mutual_avoidance_uav1`
- Original format: ROS1 bag v2.0 (final index missing; strict chunk scan)
- SHA-256: `43f25ac10deb11f8eed4febe33574b7c7bbc67171b5a93268931e827ad40cf24`
- Duration: 46.126 s
- LiDAR: `/livox/lidar`, `livox_ros_driver/CustomMsg`, 1384 messages, 30.022 Hz
- IMU: `/mavros/imu/data`, `sensor_msgs/Imu`, 8000 messages, 173.562 Hz
- Per-point field: `uint32 offset_time`, nanoseconds relative to `uint64 timebase`
- Point offset regressions: 1
- Non-positive IMU dt: 0
- Maximum IMU gap: 16989504 ns
- Scans lacking a start/end IMU bracket: 2
- Timestamp policy: `timebase_authoritative` because the observed header/timebase delta is [-239, 239] ns.
- Input frames: `{"/livox/lidar": {"livox_frame": 1384}, "/mavros/imu/data": {"base_link": 8000}}`
- The missing final ROS1 index is preserved as a provenance fact; conversion reads valid chunks strictly and verifies every record.
