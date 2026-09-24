# Stream timestamp policy

The evaluator now names source-time policy explicitly in `SourceTimestampPolicy`; version 2 of the nested evaluation schema records the assigned policy for navigation command and measured odometry. This is evidence validation only. Existing report schema remains version 1; it carries the nested evaluator result without changing control behavior.

| Stream | Policy | Reason |
|---|---|---|
| NavigationCommand PVA reference | `DUPLICATES_ALLOWED_FOR_HEARTBEAT` | Repeated ROS source tick is legal only when immutable reference/certificate fields are equal and sample IDs strictly advance. Raw samples remain available for lineage checks; the canonical time-indexed copy collapses only proven exact heartbeats. |
| Ground-truth odometry | `STRICTLY_INCREASING` | Independent measured source stream has no command heartbeat semantics. |
| Corrected/propagated odometry | `STRICTLY_INCREASING` within localization epoch | Source regression/duplicate is invalid; an epoch change must be separately witnessed. |
| Lifecycle event stream | `EVENT_SEQUENCE_AUTHORITATIVE` conceptually | Causal IDs and producer sequence, not nearest timestamp, join transactions. |
| Host-only diagnostics | `NO_SOURCE_TIME` conceptually | Never used to fake a source timestamp. |

In the pinned run 5 raw samples, accepted ground-truth (3,573), propagated (3,499), and corrected odometry (680) have zero adjacent source duplicate/regression. Diagnostic status streams do contain duplicate stamps, but are not treated as measured trajectory streams. The historical A3 same-stamp world-revision change remains `SOURCE_TIMESTAMP_DUPLICATE_CONFLICT` because its certificate changed.
