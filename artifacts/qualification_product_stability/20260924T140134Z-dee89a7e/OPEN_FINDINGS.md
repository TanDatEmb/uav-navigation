# Open findings

| ID | Class | Source/evidence | Risk | Required next evidence |
|---|---|---|---|---|
| PS-001 | `POLICY_DECISION_REQUIRED` / possible product dynamics or estimator defect | Two nominal emergency STOPPED_HOLD episodes accelerate/oscillate after analytic stop and LIO measured error crosses 0.750 m; independent truth displacement differs 0.044–0.058 m | 2/10 mission liveness failures while safety correctly fails closed | Bounded PX4/vehicle stopping and LIO-vs-truth displacement characterization across repeated conditions, then approved behavior change if justified |
| PS-002 | `UNRESOLVED` | Historical 481.677 ms state tail plus ROS clock/IMU stall, no native/host witness | Rare state freshness loss | New native/host correlation on reproduced tail or controlled fault; never back-attribute old run |
| PS-003 | `DIAGNOSTIC_ONLY` | Native observer cannot directly timestamp external `ros_gz_bridge` clock/IMU callback or internal FAST-LIO input callback | Some layer boundaries remain a bracket, not exact callback latency | Add bounded callbacks only if new events show this is the unresolved first boundary |
