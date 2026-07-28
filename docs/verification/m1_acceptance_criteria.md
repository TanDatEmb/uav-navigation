# M1 acceptance criteria

M1 is accepted only with a clean Jazzy build, passing unit tests, traceable pinned
upstream provenance, integer-nanosecond timestamps, documented frames/static
transforms, and no planner, occupancy, safety, or PX4 code. Demonstrate both
SIM deskew bypass and real per-point deskew support, corrected-only odometry,
offline replay using `FastLioPipeline`, odom-framed registered points and map,
and sufficient diagnostics.

Run and retain evidence for these scenarios:

1. Static (1–2 minutes): stable yaw/Z/velocity/biases and no map layers.
2. Pure yaw: walls re-align, map does not rotate with the UAV, yaw sign is right.
3. Translation: forward motion increases odom X without unexpected Y/Z.
4. Vertical motion: lift increases odom Z without material X/Y drift.
5. Square route: inspect sign/axis errors, yaw-position coupling and drift.
6. Indoor-to-semi-open: frame remains stable and degradation is diagnosed.

Stop rather than mask failures for unknown frame/extrinsic/timestamp semantics,
missing real per-point timing, IMU bracket failure, NaN state/covariance, double
point transformation, timestamp regression, map rotation, or wrong motion sign.
