# Safety and runtime regressions

| Seam | Evidence on this branch | Status |
|---|---|---|
| Gazebo simulation pause | Exact server PID paused 350.272 ms after request 2. Adapter reported `ODOMETRY_STALE`, requested PX4 Hold, and report observed Hold. Scenario remained `FAILED_COMPONENT`/`NOT_EVALUABLE` as emitted. | Focused controlled SITL observation; fail-closed preserved |
| Core command silence >100 ms | Predecessor exact `SIGSTOP` Core regression in `artifacts/failclosed_ownership_fencing/20260923T224537Z-49e9c0e8/SITL_RESULTS.md` produced stale PVA and Hold. Current branch has no C++ product change. | Fresh current-branch run pending; older evidence is not a substitute |
| Exact `kOptimizationFailed` → HG-023 | Current Release CTest includes planner/retained-command component paths; `check_exact_optimization_injection.py` passes. Prior exact runtime proof is pinned by the predecessor campaign. | No new status injection on this diagnostic branch; current product path unchanged |
| Repeated planner failure → BACKUP → measured restart | Predecessor focused SITL in `artifacts/failclosed_ownership_fencing/20260923T224537Z-49e9c0e8/SITL_RESULTS.md`; no product source edit here. | Historical evidence only |
| Terminal STOP | Eight of ten predecessor nominal runs accepted the final waypoint with measured position 0.222–0.273 m and speed 0.046–0.099 m/s; two terminal recovery failures remain. The new no-native control and two native repeats completed `[0,1,2,3,4]`. | STOP safety gate remains measured-state based; repeatability not established |

No result in this table is promoted to flight qualification. New current-branch fault runs are excluded from nominal denominators.
