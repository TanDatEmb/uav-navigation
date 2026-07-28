# Frame contract

The production state is the IMU pose in the local odometry world:

```text
odom --dynamic--> imu_link --static, calibrated--> lidar_link
```

- `odom`: local gravity-aligned estimator world; metres, right handed.
- `imu_link`: IKFoM body/state frame.
- `lidar_link`: Mid-360 measurement frame.
- Dynamic output: `odom -> imu_link`.
- Static calibration notation: `T_imu_lidar` maps a LiDAR point into IMU.
- `t_imu_lidar = [-0.019391, -0.000278, 0.080926] m`.
- `R_imu_lidar = I`.
- Quaternion serialization order is ROS `x,y,z,w`; internal Eigen construction
  is `w,x,y,z`.
- Estimated gravity points downward in `odom`; no NED/FRD/PX4/map conversion is
  introduced.
- No `odom -> base_link` is published because this dataset supplies no trusted
  `T_imu_base`.
