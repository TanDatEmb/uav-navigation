# IKFoM estimator state

The production estimator owns the upstream IKFoM manifold declared in
`fast_lio_core/estimation/ikfom_state.hpp`. Its nominal state is:

```text
(p_odom_imu, R_odom_imu, R_imu_lidar, p_imu_lidar,
 v_odom_imu, gyro_bias, accel_bias, gravity_odom)
```

Position, velocity and biases use IKFoM vector components, rotations use
`MTK::SO3`, and gravity uses `MTK::S2`. IKFoM owns the 23-DoF error-state,
covariance layout, boxplus/boxminus and Kalman machinery. `ManifoldState` is a
plain output/interchange view and does not implement a second filter or
manifold retraction. Extrinsics remain in the upstream-compatible state as
static calibration; online extrinsic estimation is not a runtime feature.

Initialization collects stationary IMU data, gates quality using sample count
and variance, estimates gravity/biases, and reports its reason/status. Online
extrinsic estimation is outside the M1 baseline.

## Error-state index contract

The 23-dimensional IKFoM tangent covariance uses the following fixed ordering:

| Offset | Dimension | Block |
|---:|---:|---|
| 0 | 3 | position `p_odom_imu` |
| 3 | 3 | orientation `R_odom_imu` |
| 6 | 3 | LiDAR-IMU extrinsic rotation |
| 9 | 3 | LiDAR-IMU extrinsic translation |
| 12 | 3 | velocity `v_odom_imu` |
| 15 | 3 | gyro bias |
| 18 | 3 | accelerometer bias |
| 21 | 2 | gravity on `S2` |

These offsets are exposed by `ManifoldState` constants and must be used by
downstream covariance projection code. IKFoM's `SO3::boxplus` is right-local:
`R' = R * Exp(delta_theta)`. The covariance remains an IMU-origin tangent
covariance until a validated output-frame projection is available.
