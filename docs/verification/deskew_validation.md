# Deskew validation

For simulation select `simultaneous_scan`: every point has relative time zero and
diagnostics must say `BYPASSED_SIMULTANEOUS_SCAN`. No synthetic timing is
allowed. For real hardware select `per_point`, verify non-fabricated point timing
and both IMU brackets, then test synthetic motion against expected compensated
point positions.

Record point-time min/max, interpolation failures, deskew mode/applied flag, and
runtime. Missing point timing or brackets must reject the real scan and surface a
diagnostic; disabling deskew without reporting it is not a workaround.
