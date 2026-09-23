# A/B/C instrumentation timing

**Verdict: measured, but not a statistical non-interference proof.** A is Release instrumentation OFF, B is the same Release source with the audit option ON and no separate audit bag, and C is ON with a best-effort audit recorder. Each is one `long_featured` / nominal / seed 23 positive SITL episode on the same host, ROS domain 42 and XRCE port 8892. The PX4 binary hash is in the evidence index. Each mission completed with five accepted waypoints and an observed PX4 Hold. All three evaluator reports are `FAIL`/not qualification eligible because the pre-existing relaxed tracking experiment and reference/coverage policy remain in force; this is diagnostic timing evidence only.

| Measurement | A OFF | B ON | C ON + recorder |
|---|---:|---:|---:|
| Command header gap p99 / max (ms) | 16.000001 / 32.000001 | 16.000001 / 16.000001 | 16.000001 / 32.0 |
| Command header gaps ≥100 ms | 0/3592 | 0/4044 | 0/3438 |
| Mapping callback p99 / max (µs) | 53,670 / 66,393 | 51,415 / 63,138 | 52,382 / 78,419 |
| Planner scheduling gap p99 / max (µs) | 400,042 / 400,173 | 399,977 / 400,131 | 399,982 / 400,050 |

The command gap uses successive `NavigationCommand.header.stamp` values from each product rosbag. It is a **publication cadence proxy**, not the command callback duration, adapter receipt, or PX4 setpoint gap. Planner scheduling includes the planned long period. The C mapping maximum is larger than A/B, so a tail-neutrality claim is not supported. One run per arm cannot bound jitter or causal overhead.

The C audit stream measured local callback durations: MissionController update p99/max **52.78/67.172 µs** (n=1467), runtime `rclcpp::publish` call **116.392/288.042 µs** (n=2701), and adapter receive callback **46.909/150.938 µs** (n=2683). These audit-specific timers do not exist in OFF or unrecorded B, so no OFF/ON callback percentile difference can be claimed. The current telemetry does not separately time serialization from middleware publish work.

In C, runtime producer enqueue p50/p95/p99/max was **157/477/778/6999 ns** (n=4750); adapter was **271/525/909/5437 ns** (n=4239). Maximum observed queue occupancy was **6/1024** runtime and **4/1024** adapter. C reported **4** runtime drops and **0** adapter drops. Maximum window p99 of the consumer publish call was **286,276 ns** runtime and **91,991 ns** adapter; the largest individual reported call was **286,276 ns** and **652,731 ns**, respectively. Window quantiles are not a full-run distribution. Drops are diagnostic evidence loss; they did not change a control decision. See `INSTRUMENTATION_OVERHEAD.csv` for denominators and units.

No new ≥100 ms command-header gap or product test failure appeared in A/B/C. That is sufficient to use well-framed events as **observations**, with the sequence gaps enforced locally, but insufficient for a quantitative race bound or flight timing guarantee. OFF and ON full Release builds each completed 23 packages; OFF tests had **86 groups, 0 failures**, ON had **87 groups, 0 failures** (the extra group is audit tooling). Build/test manifests and logs are indexed in `EVIDENCE_INDEX.md`.
