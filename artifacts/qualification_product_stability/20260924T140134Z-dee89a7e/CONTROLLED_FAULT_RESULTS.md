# Controlled temporal fault

The diagnostic-only harness `controlled_gazebo_pause.py` launched one `long_featured`, seed-0, true tracking-off SITL with the native observer enabled. It found the exact `gz sim -r -s` PID **73502** in the session-owned `px4_gazebo` process group after a real request-2 PVA command. It sent `SIGSTOP` to that PID alone for **350.272 ms** on the steady clock, then sent `SIGCONT` in `finally`. No PX4, Core, adapter, bridge or process group was deliberately paused. The fault is absent from normal runs and does not alter product configuration or thresholds.

Raw session: `/home/letandat/Dev/uav-navigation/.artifacts/runtime/external-mode-check-20260924T145951-71499`. Provenance is in `controlled_pause_provenance.json`; source trace reduction is `controlled_temporal_layers.json`.

| Gap | Accepted state | Native clock / source | Native stats / source | Downstream | Attribution |
|---|---:|---:|---:|---|---|
| At pause | 373.874 ms | 354.114 / +4 ms | 403.888 / +44 ms | worker, publisher and adapter all paused | `GAZEBO_SIMULATION_STALL` at first observed layer |
| After resume | 264.478 ms | progressing | progressing | worker/publisher gap while IMU continued | `FAST_LIO_WORKER_STALL` post-fault observation |

The adapter observed `ODOMETRY_STALE`, requested PX4 Hold, and `px4_hold_observed=true`. The scenario result is **`FAILED_COMPONENT`**, with only waypoint 0 accepted; the runner/evaluator's `FAIL` and `NOT_EVALUABLE` labels are retained. This is a deliberately destructive fault run, excluded from nominal evidence. It verifies that native/ROS/worker/adapter witnesses can locate the injected first-layer interruption while the existing 200 ms state boundary remains fail-closed. It does not qualify flight behavior or explain the historical 481 ms event retroactively.
