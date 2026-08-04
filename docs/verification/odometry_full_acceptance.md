# Full odometry lifecycle acceptance

Acceptance must be run from the feature branch with the protected PX4-LIO
configuration unchanged. Record repository/PX4/`px4_msgs` SHAs, config hashes,
compiler, exact commands, start/end times, exit codes, and artifact paths.

Required checks:

1. Build and focused/unit tests for core, ROS, supervisor, PX4 bridge, and
   generated interfaces.
2. `make check` and `make vendor-check`.
3. Canonical PX4 MID-360 SITL with reset, stale, metadata, roll/pitch,
   stationary-yaw, generation, and external-gate fault injections.
4. Machine checks for finite/complete odometry, exact timestamp pairing,
   alignment validity/rejection reason, generation lock, publisher suppression,
   frame fields, timestamp sample provenance, reset counter, and supervisor
   gate truth table.

No PASS, fixed, validated, or no-regression conclusion is valid without the
current run's machine-readable artifacts.
