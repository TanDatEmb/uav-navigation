# Registration-map validation

The map frame is always `odom`. Insert points only after a successful, converged
LiDAR correction while tracking. Preserve evidence from raw message through
adapter/time, deskew pose, `T_imu_lidar`, corrected pose, odom point, and map
insertion. Never insert pre-correction, initialization, or rejected scans.

During static, yaw, translation, vertical and square tests, compare successive
registered clouds in `odom`. Check that the map neither follows yaw nor gains
systematic layers. Record map point/insert/remove counts, center, update time,
valid planes/residuals, RMS, iterations, and convergence.
