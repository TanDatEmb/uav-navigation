# Temporal layer contract

Diagnostic trigger: accepted-state gap >150 ms. Product safety boundary remains 200 ms. Observers use wall/steady receive time; sensor/ROS source stamps remain distinct.

| Layer | Witness | Independent of flight authority? | Limitation |
|---|---|---|---|
| Gazebo process / world stats / world clock | default-off native Gazebo Transport observer, bounded gap events | yes | observer scheduling must be checked independently |
| Observer/host scheduling | 50 ms native-observer loop gaps, 1 Hz process/PSI snapshots | yes | does not prove whole-host pause at sub-second scale |
| Native IMU/LiDAR | native Gazebo Transport `/sim/mid360/imu` and `/sim/mid360/scan/points` source stamp/arrival | yes | native subscription gap alone does not separate sensor generation from Gazebo Transport delivery |
| Bridged ROS clock, IMU, LiDAR | independent monitor/rosbag arrival and source stamp | yes | `ros_gz_bridge` clock/IMU callback entry not instrumented |
| FAST-LIO ingress | ROS input observer plus existing queue/drop diagnostics and corrected odometry | yes | observer arrival is not the internal FAST-LIO callback entry |
| Propagated worker / publisher | exact `(localization_epoch, sequence)` transport trace | yes | none for output transaction |
| Adapter callback / accepted state | same exact identity, callback/lock/receive stamps | yes | none for local admission |

`tools/runtime/analyze_temporal_layers.py` joins these clocks and returns `UNRESOLVED` if independent upstream evidence is missing. It never changes QoS, executor topology, freshness, or admission. Native observer is enabled explicitly by `--gazebo-native-diagnostic`; all telemetry is bounded and diagnostic-only. Sim time progress divided by observer time is reported only as a derived ratio, not PX4/Gazebo real-time factor.
