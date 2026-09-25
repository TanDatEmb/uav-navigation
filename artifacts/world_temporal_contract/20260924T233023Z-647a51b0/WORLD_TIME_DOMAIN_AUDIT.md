# World time-domain audit (AS-IS)

| Value | Origin | Domain | Use / limit |
|---|---|---|---|
| Registered scan source stamp | `RegisteredScan.header.stamp`, also required equal to points/free-space stamps | ROS/source clock transported from estimator/registration | `MappingActor` enforces positive and strictly increasing within epoch; world identity uses it. |
| Mapping callback receive time | Not stored in `WorldSnapshotIdentity`; callback timing uses `steady_clock` for durations | Steady clock | Mapping telemetry measures execution durations, not per-snapshot receive freshness. |
| Snapshot publication time | No independent timestamp field in world identity; publication is atomic pointer store | No timestamp | Publication order is represented by identity and store gate. |
| Candidate solve world | `pinned_world_identity` copied from immutable solve input | Identity, not a time sample | Establishes immutable source map for solve. |
| Candidate `valid_from_ns`, `valid_until_ns`, activation | ROS clock nanoseconds; trajectory `start_wall_time_s` is converted against ROS epoch | ROS/system time domain | Candidate sampling/admission/exposure lease. Endpoint cap uses declared analytic trajectory endpoint. |
| Candidate validation time | Authorization `wall_time_s` derived from ROS clock | ROS clock | Passed to immutable candidate world validator; no steady/source arithmetic. |
| Execution recertification time | Runtime `ros_clock->now().nanoseconds()` | ROS clock | Computes a possible refreshed exposure lease deadline `now + freshness_window`; candidate analytic endpoint caps it. |
| World freshness evaluation | Current ROS clock vs source `observation_stamp_ns` through `classifyTimestampFreshness` | ROS clock compared to ROS/source clock | 500 ms configured limit. Proves source timestamp age, not independent receive/publication progress. |
| Callback/validation performance | `std::chrono::steady_clock` | Monotonic host clock | Duration metrics only; no comparison with ROS/source stamps. |

No source reviewed performs arithmetic between steady and ROS time. The current contract intentionally does not add receive-time to world identity. This leaves a diagnostic limitation: a source stamp can remain recent under paused `/clock` or clock-domain mismatch while the mapping publication stream itself is stalled; such a condition needs separate runtime witness or explicit clock-health evidence before it can be called receive freshness.

`data_freshness_window_s_ = 0.5` currently parameterizes both execution-state source/receive freshness and world-source freshness; it also sets candidate exposure lease lengths. These are distinct checks with a shared configured duration. Same number does not make the evidence interchangeable. Command message `valid_until` uses the independent 100 ms command stream timeout.
