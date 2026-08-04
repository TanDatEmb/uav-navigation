# PX4 reset compensation verification

Unit coverage is in `px4_odometry_bridge/test/test_px4_odometry_bridge.cpp`.
`ResetObservation` classifies metadata-pending, invalid metadata, counter
discontinuity, invalid rotation, source restart, reset-transition suppression,
and accepted observations. Accepted observations compensate position,
orientation, world/body velocity, and covariance atomically while preserving
the original measurement timestamp and recording the matched metadata epoch.

The bridge fails closed after publication when metadata is missing or
ambiguous. A startup-only rebaseline is allowed before the first output and is
recorded with `startup_rebaseline_generation`.
