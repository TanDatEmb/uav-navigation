# Estimator result and publication semantics

`ProcessResult` separates propagation from a successful LiDAR correction:

- `predicted_estimate` is present only after IMU prediction reaches the scan
  end. Its presence does not authorize corrected odometry publication.
- `corrected_estimate` is present only after a finite, successful LiDAR update.
- `estimate_validity` is `UNAVAILABLE`, `PREDICTED_ONLY`, or `CORRECTED`.
- `lidar_update_status` is `NOT_ATTEMPTED`, `REJECTED`, or `SUCCEEDED`.
- `last_lidar_correction_time` records the exact most recent successful update
  time independently of the current scan.

`hasCorrectedOutput()` requires all of the following: `CORRECTED` validity,
`SUCCEEDED` update status, a present corrected estimate, and finite state and
covariance. `hasRegisteredScanOutput()` additionally requires a non-empty
registered scan.

ROS odometry, `odom -> imu_frame`, registered points, and local-map snapshots
are gated by this corrected-output contract. A rejected update may retain a
valid predicted state internally and expose it for diagnostics/offline
reasoning, but it does not publish that state as corrected odometry and does
not publish a registered scan. Diagnostics are still published for unavailable
and predicted-only results, including update status, output validity, exact
nanosecond time, clock domain, and last correction time.

The pipeline tests cover initial-map predicted-only output, successful
correction, and registration rejection. The rejected case asserts that the
predicted estimate remains explicit while corrected estimate and registered
scan outputs remain absent and the map is unchanged.
