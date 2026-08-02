# Frame and extrinsic validation

Inspect the generated URDF tree and runtime configuration together. Verify the
`base_link -> livox_frame -> livox_imu_frame` chain, FLU axes, transform
direction, and that `T_I_L` is the exact inverse of the documented factory
nominal `T_L_I`.

The factory nominal transform is parent `livox_frame`, child `livox_imu_frame`,
translation `[0.011, 0.02329, -0.04412] m`, identity rotation. FAST-LIO's
`translation_imu_lidar` is therefore `[-0.011, -0.02329, 0.04412] m`. The
real profile records this as factory nominal, not unit calibration. AIST keeps
its dataset-specific estimator extrinsic and must not be generalized.

Perform physical/simulation basis checks: move forward (+X base), left (+Y), up
(+Z), and apply positive yaw. Verify each sign before mapping. A guessed
extrinsic, hidden decoder axis swap, aliased LiDAR/IMU frame, or unresolved
sensor mount is a stop condition, not something to compensate with a yaw
offset.
