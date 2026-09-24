# Control and observer cost

The live N1 rosbag contains fixed-size CDR records: old baseline `NavigationCommand` **680 B**, new control `NavigationCommand` **320 B**, new `NavigationExecutionDiagnostics` **520 B**. The control record is 360 B (52.9%) smaller. At 50 Hz this is 34.0 versus 16.0 kB/s of serialized control payload. A fully delivered 50 Hz diagnostic stream adds 26.0 kB/s, so the combined serialized payload is 42.0 kB/s; no total-network-bandwidth improvement is claimed. These values exclude DDS framing and retransmission.

The separate 10,000-sample local rclpy serializer benchmark is in `COMMAND_SIZE_BANDWIDTH.md`. It shows reduced command serialization time, but it is neither callback p99 nor real-time WCET. No equivalent before/after callback-entry/exit instrumentation exists at the pinned source commits; control callback p50/p95/p99/max and mutex-held time are **NOT_MEASURED**. This cut does not infer a callback latency improvement from message size or LOC.

Live adapter admission inter-arrival gaps (ms):

| Run | p50 | p95 | p99 | max |
| --- | ---: | ---: | ---: | ---: |
| N1 | 19.997 | 20.218 | 20.341 | 21.317 |
| N2 | 19.999 | 20.227 | 20.366 | 20.616 |
| N3 | 20.000 | 20.259 | 20.458 | 40.328 |

N3's maximum follows a typed `NOT_YET_VALID` rejection; the previous command remained admitted. These are rosbag observer timestamps, not PX4 firmware consumption timestamps. The callback refactor reduced `onNavigationCommand()` from 351 to 211 source lines while preserving the independent 100 ms receive lease. LOC is a maintainability measure, not performance evidence.
