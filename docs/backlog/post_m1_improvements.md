# Post-M1 improvements

These items are intentionally not implemented in M1.

| Problem | Reference source | Expected benefit | Risk | Effort | Dependencies | Recommended milestone |
| --- | --- | --- | --- | --- | --- | --- |
| Initialization assumes a stationary interval | FAST-LIO initialization literature | Start while moving | High: observability and false convergence | Large | validated real bags and ground truth | M2 |
| Extrinsics are fixed and placeholders remain in example configuration | FAST-LIO and sensor calibration guidance | Correct measured sensor alignment | High: an online estimate can absorb timing/registration errors | Large | calibration target and representative motion | M2 |
| LiDAR/IMU temporal offset is not estimated online | FAST-LIO timing guidance | Resilience to clock offset | High: estimator consistency | Large | hardware clock characterization | M2 |
| No loop closure or global map | FAST-LIO2 scope | Reduce long-run drift | High: changes map ownership and frames | Large | stable M1 odometry | M3 |
| Dynamic objects are not removed | FAST-LIO2 direct registration constraints | More robust urban mapping | Medium | Medium | labeled real sequences | M3 |
| No visual fusion | FAST-LIVO2 | Robustness in weak geometry | High: new sensor and state | Large | calibrated camera and dataset | M3 |
| CPU-only single-estimator execution | Current M1 architecture | Higher throughput | High: nondeterminism and synchronization complexity | Large | target Pi 5 profiling | M3 |
| Only one Mid-360 is supported | M1 locked scope | Multi-LiDAR coverage | High: time/extrinsic observability | Large | multi-sensor hardware | M4 |
| No high-rate propagated odometry output | Current corrected-only contract | Lower-latency downstream estimate | Medium: consumers may confuse validity | Medium | explicit propagated-output API | M2 |
| No PX4, planner, safety, or controller integration | Repository layer architecture | Flight-system integration | Safety critical | Large | accepted M1 estimator | Later flight milestone |
