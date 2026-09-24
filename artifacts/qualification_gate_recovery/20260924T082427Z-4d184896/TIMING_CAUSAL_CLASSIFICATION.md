# Timing causal classification

## Pinned A2 (before new trace)

- Safety response: adapter rejected Core command sample 1499 with `RECEIVE_STALE`; source age 8.000 ms, last-accepted-state receive age 208.583 ms. Hold request followed. This is expected fail-closed behavior at the unchanged 200 ms boundary.
- Independent observer: `/clock` monitor arrival gap 1150.256 ms while ROS source advanced 4 ms; propagated odometry seq 2006→2007 observer gap 1160.433 ms while source advanced 20 ms. IMU and ground truth also paused about 1155 ms; PX4 odometry continued near 20 ms at the failure instant.
- Narrowest supported class: `SIM_TIME_STALL_OR_SLOWDOWN` affecting `/clock` and simulated sensor delivery. The first origin (Gazebo clock generation, bridge/executor or host scheduling) remains unproven; classify `PARTIAL_CAUSAL_ATTRIBUTION` until producer and adapter per-sequence witnesses reproduce or exclude layers.
- Not supported: asserting `DDS latency = 208.583 ms`, blaming `trajectory_mutex_`, or changing the 200 ms boundary.

## New diagnostic decision tree

Join exact localization epoch and sequence. A clock arrival gap with minimal source advance is `SIM_TIME_STALL_OR_SLOWDOWN`. Otherwise count rejected callbacks between accepted states before blaming transport. A large callback lock wait implicates adapter mutex; a large producer publish gap implicates the producer/upstream path. Regular producer publication with large publish→callback delay leaves `DDS_OR_EXECUTOR_DELIVERY_DELAY`; this branch intentionally does not split DDS from executor without more witnesses. Gaps missing required producer/adapter trace stay `UNKNOWN`. No class grants authority or changes a safety threshold.
