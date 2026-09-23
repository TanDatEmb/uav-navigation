# Diagnostic-only event design

`NAVIGATION_AUDIT_INSTRUMENTATION` is a CMake option in runtime and PX4 adapter packages, default `OFF`. The message schema is always generated for tooling; `OFF` compiles no sink member, publisher, timer, or enqueue hook into either product process. There is no YAML switch. The branch is an experiment and does not alter a safety gate or product decision.

`/navigation/audit_event` is a typed `navigation_contracts/msg/AuditEvent` topic with best-effort, depth 64 QoS. All fields are fixed-width scalars; no string is copied into the queue. Mission identity is FNV-1a 64-bit of the existing mission ID. A record's zero fields are absent unless the event type/phase defines them; `valid_fields` is reserved for future field-presence bits and is not yet used as authority. Every event has `producer_id` (`1` runtime, `2` adapter), process incarnation derived from process-start steady time and PID, and a per-sink diagnostic sequence. This incarnation is a diagnostic partition, never an execution key.

One process-local `Sink` accepts events from the runtime's reentrant command timer, mapping worker, and scan callback, or from the adapter's command/mission callbacks and executor callbacks. These are **multiple producers**, so the existing SPSC queue is not used. The sink uses a fixed-capacity 1024-record ring and `try_lock`; contention or full capacity increments an atomic drop count and immediately returns. A 20 ms ROS timer drains at most 64 records per call, releases the ring mutex, then publishes on the consumer side. The producer never performs ROS serialization or disk I/O. Consumer publisher exceptions are contained. Periodic `QUEUE_ACCOUNTING` records expose enqueued, dropped, published, maximum occupancy and last/p50/p95/p99/max consumer `rclcpp::publish()` cost in a bounded 4096-sample window. The call cost includes middleware work; serialization is not timed separately. Lost records make the affected episode `INSUFFICIENT_TRACE`.

Event families and current phase values:

| Event | Phase/outcome | Fields used |
|---|---|---|
| `MISSION_GATE=1` | phase `0` no crossing, `1` current in ball, `2` two-sample segment; outcome is `MissionControllerEvent::Type` ordinal | exact controller update witness, callback/source time, previous/current odometry sequence and source stamp, continuation command identity, crossing error/gap, flags |
| `COMMAND_PUBLISH=2` | outcome `1` means rclcpp publish call returned; it does **not** prove DDS delivery | complete command key, source/header/valid-until stamps, publish enter/return steady and ROS stamps, execution authorization |
| `COMMAND_RECEIVE=3` | outcome `1` admitted; `2` rejected; reason `1` shape/time, `2` terminal lifecycle, `3` identity, `4` odometry stale, `5` non-increasing sample, `6` anchor | same command key, callback entry/end, admitted receive lease origin |
| `HOLD_TRANSFER=4` | phase `1` request, `2` schedule attempt, `3` retry tick, `4` callback, `5` VehicleStatus, `6` executor deactivate, `7` failsafe deferred, `8` activate | existing attempt count, existing flags, retry deadline, status source/receive times, result/nav state |
| `WORLD_TRANSITION=5` | phase `1` admitted scan, `2` snapshot ready, `3` stale reject, `4` suspend, `5` recert start, `6` recert end, `7` commit, `8` resume | immutable world identity, active bundle generation, validation/commit result, source/ROS/steady time |
| `LEASE_DISPOSITION=6` | reason `1` command receive lease, `2` command header stale/invalid, `3` valid-until expiry | last command key, admission lease origin, current ROS/steady, source/header/valid-until stamps |
| `QUEUE_ACCOUNTING=7` | once per second | cumulative counters and bounded consumer publish-cost quantiles |

`MISSION_GATE.flags` bits: 0 crossing evaluated; 1 crossing valid; 2 continuation present; 3 controller-certified continuation valid; 4 certified suffix-stop; 5 coincident terminal hold ready; 6 immediate initial pass-through; 7 progression ready; 8 acceptance ready; 9 waypoint accepted; 10 PASS_THROUGH behavior. The trace observer cannot create or retain a crossing, change the controller's return event, or grant continuation readiness.

`HOLD_TRANSFER.flags` bits: 0 pending; 1 in-flight; 2 PX4 Hold confirmed by current callback; 3 complete-navigation-failure requested; 4 VehicleStatus failsafe; 5 failsafe with operator takeover. `HOLD_TRANSFER.outcome` is raw `px4_ros2::Result` ordinal for callback events, not a confirmation interpretation.

`WORLD_TRANSITION.flags` are phase-specific: at recert start bit 0 means pending bundle exists; at recert end bit 0 pending retained, bit 1 this callback took a fast path, bit 2 this callback took a full path (deltas of existing telemetry counters); at commit bit 0 active retained and bit 1 active invalidated. These represent existing product results, not a second certificate.

The parser must preserve `source_stamp_ns`, `ros_now_ns`, and `steady_ns` separately. No audit stream subscriber is created in product code. No field from this schema is read by mission acceptance, planner admission, execution timeline, PX4 setpoint selection, or safety-stop decision.
