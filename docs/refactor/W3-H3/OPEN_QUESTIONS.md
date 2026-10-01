# W3-H3 open questions

## H3.1 / R4-01 — authoritative IMU horizon is missing

- Status: `NOT_FIXED`; this finding is stopped without implementation.
- Baseline recheck: `src/estimation/fast_lio_core/src/sync/measurement_buffer.cpp:97-101` still owns the IMU deque window, but the current `MeasurementBufferConfig` exposes only `maximum_imu_samples`; synchronization exposes `maximum_imu_gap_ns`, not an IMU age horizon.
- Evidence: `rg -n -i 'imu.*horizon|horizon.*imu|imu.*window|maximum_imu_samples|imu_samples' src/estimation config` found capacity/gap settings but no authoritative time/count horizon suitable for this fix.
- Decision required: identify the existing authoritative horizon, or explicitly approve a typed configuration parameter and its owner/default/removal condition. Do not infer a value from the deque capacity or a single replay.
- Verification after decision: add the continuous-IMU/no-scan-past-horizon synchronization test, run the focused Fast-LIO buffer tests, ledger validator, and `git diff --check`.
