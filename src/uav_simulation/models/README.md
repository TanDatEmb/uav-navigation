# Model boundary

This package intentionally provides only static geometric references for M1
registration tests. It does not provide a flight controller, planner, safety
system, PX4 interface, or unverified sensor plugin. Connect a separately
validated Gazebo LiDAR/IMU bridge and use the `simultaneous_scan` estimator
configuration when all simulated rays share one timestamp.
