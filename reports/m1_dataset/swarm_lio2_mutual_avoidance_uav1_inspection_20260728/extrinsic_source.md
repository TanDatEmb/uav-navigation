# Dataset extrinsic source

- Upstream repository: `hku-mars/Swarm-LIO2`
- Upstream commit: `a5f751a797bb92baa3104cdd384a312d3c8e7744`
- Source file: `swarm_lio/config/mid360.yaml`
- Source keys: `LI_extrinsic_T`, `LI_extrinsic_R`
- Translation: `[-0.019391, -0.000278, 0.080926]` metres
- Rotation: identity, row-major
- Upstream direction: point from LiDAR coordinates into IMU coordinates
- Project notation: `T_imu_lidar`, the same direction; no inverse is applied

This value is not tuned against the resulting map.
