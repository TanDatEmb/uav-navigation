# ROS / steady clock relation

Pinned A2 independent `/clock` monitor recorded a 1150.256 ms monitor-arrival gap while clock source advanced 4 ms (43.044→43.048 s). Independent propagated odometry arrival gap was 1160.433 ms for 20 ms source progression (seq 2006→2007). IMU and ground truth had similar ~1155 ms gaps; PX4 odometry continued near 20 ms at the fail instant. This supports a simulated-clock/sensor-stream slowdown or delivery pause, not the statement that DDS alone took 208.583 ms. Whether Gazebo clock production, bridge, executor scheduling or host contention first stalled remains unresolved without per-sequence producer/adapter trace and process scheduling evidence.

The adapter's 208.583 ms receive-age threshold violation is a separate steady-clock safety result. The 200 ms boundary remains unchanged. See `A2_CAUSAL_HYPOTHESES.md` and raw session `external-mode-check-20260924T073836-106951`.
