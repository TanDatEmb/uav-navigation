# Long mapping-only stale control

Session: `/home/letandat/Dev/uav-navigation/.artifacts/runtime/external-mode-check-20260925T092009-360731`; long_featured, seed 0, tracking off, 700 ms sim-time RegisteredScan dropout. The gate isolated mapping input and passed its liveness/remap checks: seven scans dropped while /clock, odometry, valid health, Core diagnostics and adapter admissions continued.

The last committed world before the outage was generation 2/revision 184/source stamp 22.900 s. Runtime suspended active generation 2 at 23.408 s with `SOURCE_STALE` (508 ms source age). No new command samples were published after suspension. With the outage continuing beyond the command lease, the runner observed `PAUSED_SAFETY_STOP`; Hold handover was requested at 23.516 s and PX4 Hold was observed. The runner outcome is `BLOCKED` because the mission did not complete, which is the expected long-outage control result, not a short recovery success.

Gate forwarding was restored after its 700 ms interval, but the run does not claim automatic PX4 External Mode re-entry or a post-Hold mission restart. That protocol is explicitly NEXT_PHASE. The 100 ms lease and 500 ms World freshness threshold were unchanged.
