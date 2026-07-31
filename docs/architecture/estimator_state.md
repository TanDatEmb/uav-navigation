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
manifold retraction. Extrinsics exist in the upstream state but default to fixed:
`extrinsic.estimate_online: false`.

Initialization collects stationary IMU data, gates quality using sample count
and variance, estimates gravity/biases, and reports its reason/status. Online
extrinsic estimation is outside the current runtime capability.
