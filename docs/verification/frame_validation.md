# Frame and extrinsic validation

Inspect the generated URDF tree and runtime configuration together. Verify the
four named frames, FLU axes, transform direction, and that `T_imu_lidar` is
derived once from the documented static transforms. Replace every calibration
placeholder with an actually measured value, retaining measurement provenance.

Perform physical/simulation basis checks: move forward (+X base), left (+Y), up
(+Z), and apply positive yaw. Verify each sign before mapping. A guessed
extrinsic, hidden decoder axis swap, or unresolved LiDAR/IMU frame is a stop
condition, not something to compensate with a yaw offset.
