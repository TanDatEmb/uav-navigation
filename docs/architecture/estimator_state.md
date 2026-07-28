# IKFoM-compatible estimator state

The sole state definition is `fast_lio_core/estimation/manifold_state.hpp`. Its
nominal state is:

```text
(p_odom_imu, R_odom_imu, R_imu_lidar, p_imu_lidar,
 v_odom_imu, gyro_bias, accel_bias, gravity_odom)
```

Position, velocity, biases, and gravity use Euclidean components; rotations use
the IKFoM/SO(3) manifold operations supplied by the selected upstream-compatible
state implementation. Error-state dimension, covariance layout, and
boxplus/boxminus conventions are owned there rather than recreated as a custom
15-state filter. The state estimates IMU pose, velocity, biases and gravity.
Extrinsics exist in the compatible state model but default to fixed:
`extrinsic.estimate_online: false`.

Initialization collects stationary IMU data, gates quality using sample count
and variance, estimates gravity/biases, and reports its reason/status. Online
extrinsic estimation is outside the M1 baseline.
