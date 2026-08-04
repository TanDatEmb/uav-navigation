# FAST-LIO in-flight reinitialization verification

`FastLioPipeline` preserves the prior timestamp and predicts from it using the
bracketed IMU trajectory. The core tests cover exact-epoch propagation and an
in-flight prior with non-stationary IMU samples; the latter does not call the
stationary initializer and requires full attitude.

`LioLifecycleCoordinator` captures the last-good corrected state, invalidates
it on generation change, and only enters reinitialization when a valid snapshot
exists. A full ROS reset/restart orchestration and flight-controller maneuver
qualification remain separate runtime work and must not be inferred from unit
tests.
