# A2 causal hypotheses from pinned raw evidence

The adapter rejected sample 1499 at ROS time 43.044 s with
`RECEIVE_STALE`: source age 8.000 ms, accepted-receive age 208.583 ms.
These are two independent clock tests. The second is not a DDS latency
measurement. The original 200 ms safety response remains correct.

The independent monitor's `/clock` stream recorded a **1150.256 ms arrival
gap** from ROS stamp 43.044 s to 43.048 s, only 4 ms source-time progression
(observed ROS/steady progression ratio about 0.0035). This is a direct
clock-stream witness, not an estimate based only on odometry. The monitor
recorded five such `/clock` arrival gaps in A2; the first spans the adapter
failure.

The independent monitor last observed propagated sequence 2006, source time
43.036 s, at steady 11609047438599 ns; it next observed sequence 2007,
source time 43.056 s, at steady 11610207871378 ns. Their observed arrival
gap is 1160.433 ms for only 20 ms source-time progression. IMU and ground
truth independently show approximately 1155 ms arrival gaps across the same
window, with source times advancing 8 and 20 ms respectively. PX4 odometry
continued arriving around 20 ms apart at the fail-close instant.

This rules against assigning the 208.583 ms solely to adapter mutex or a
single adapter subscription. The observed class is
`SIM_TIME_STALL_OR_SLOWDOWN` at the `/clock` and simulated sensor streams.
The current trace cannot split Gazebo clock production from bridge/executor
delivery or host scheduling; that origin remains under investigation.

Competing hypotheses to test with diagnostic-only per-sequence timing:

| Hypothesis | Distinguishing witness |
| --- | --- |
| Producer did not publish because no new IMU source time | Worker ready/publish and IMU ingress gaps align. |
| Producer published regularly but DDS/executor delayed | Publish steady cadence regular; callback-entry delayed. |
| Adapter mutex held too long | Callback entry regular; lock wait large. |
| Messages arrived but semantic acceptance starved | Callback entries regular; typed rejection count rises. |
| Host-wide scheduling pause | Independent steady-clock tasks and PX4 streams pause together. |

No product or harness fix is justified from the old trace alone.
