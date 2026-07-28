# ADR-006: Corrected odometry first

**Status:** accepted. M1 publishes odometry only after successful LiDAR
correction in Tracking. Initial/default and high-rate propagated state are not
public odometry. This makes downstream validity visible and prioritizes a
verifiable correction baseline.
