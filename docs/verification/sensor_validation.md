# Sensor validation procedure

Before estimator tuning, record topic name/type, message rate, timestamp source
and monotonicity, LiDAR field layout, point coordinate units, IMU angular-rate
units (rad/s), acceleration units (m/s²), and adapter frame. Confirm whether the
LiDAR header denotes scan start/end; preserve, rather than guess, point timing.

Use diagnostics to record message/drop rates, invalid point count, timestamp
regressions, scan interval, IMU count per scan, maximum gap, and both brackets.
Reject an invalid message/measurement and report its reason. Do not change axes,
invent point timing, or silently accept invalid data to make RViz look plausible.
