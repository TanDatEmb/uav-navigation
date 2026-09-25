# Serialized command and observer bandwidth

Measured on the same local ROS 2 Jazzy Python/rclpy host. `serialize_message` was warmed 500 times and sampled 10,000 times per type. Messages used frame `lio_odom`, mission `external_mode_long_featured_low_altitude`, and a positive header stamp; numeric payloads were default-valued. These are **machine-local serializer microbenchmarks**, not DDS wire-byte or callback WCET measurements.

| Schema | Serialized bytes/sample | 50 Hz payload bytes/s | Serialize p50/p95/p99/max (µs) |
| --- | ---: | ---: | --- |
| Baseline `NavigationCommand` at `2b9279b4`, actual captured command | 680 | 34,000 | 17.727 / 18.753 / 23.482 / 55.344 (prior 10k rclpy microbenchmark) |
| New `NavigationCommand`, same mission string | 318 | 15,900 | 9.637 / 10.263 / 12.707 / 24.349 |
| New `NavigationExecutionDiagnostics`, same mission string | 517 | 25,850 if delivered at 50 Hz | 13.172 / 13.709 / 16.641 / 29.033 |

The control payload fell by **362 bytes/sample (53.2%)**. At 50 Hz, its serialized payload rate fell by **18,100 bytes/s**. If every observer diagnostic is delivered at 50 Hz, the two-topic serialized payload sum is **41,750 bytes/s**, which is **7,750 bytes/s above** the old single-topic payload. DDS framing, retransmission, topic discovery and actual delivery are not included. Control-plane slimming is established by the schema measurement; total network throughput improvement is **not** established.

The pinned baseline nominal rosbag contains 3,571 command CDR records of exactly **680 B**. The new N1 nominal rosbag contains 2,882 command records of exactly **320 B** and 2,882 observer-diagnostic records of exactly **520 B**. The 2-byte difference between the synthetic new command and live command is due to the precise payload/alignment of the samples; live control reduction is **360 B (52.9%)**. At 50 Hz the live control payload is approximately **16,000 B/s**; control plus diagnostic payload is approximately **42,000 B/s**. These are serialized records, not measured DDS network bytes. The new observer QoS is best effort and cannot delay flight by a delivery acknowledgement, but publisher runtime cost still needs direct measurement.
