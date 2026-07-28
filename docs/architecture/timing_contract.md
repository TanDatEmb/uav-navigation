# Timing contract

`Timestamp` and `Duration` carry signed integer nanoseconds. Absolute timestamps
must never be represented as `float` or `double`; a local duration may be
converted to seconds only at the calculation boundary. `ClockDomain` values may
not be mixed without an explicit synchronization model.

Only `RosTimeConverter` converts a ROS header timestamp to the core type. A
LiDAR adapter documents whether its header is scan start, end, or another event;
it must not assume. `LidarScan` has explicit start/end times and each point's
`relative_time_ns`. IMU time is the measurement time, never callback time.

For real scans, point time is `scan_start + relative_time_ns`; synchronization
must bracket the scan interval with IMU and reject gaps/regressions. For Gazebo
simultaneous scans all relative times are zero and deskew is reported as bypassed,
not fabricated from point index. `auto` timing belongs to offline development
only; production configurations are explicit.
