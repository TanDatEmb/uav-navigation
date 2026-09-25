# CandidateBundle temporal validity semantics (AS-IS)

## Finding

`valid_until_ns` is the **absolute ROS-time exposure/sampling lease**, bounded by the analytic endpoint. It is not the world source timestamp and not by itself a world certificate freshness measure. Separate world freshness checks compare current ROS time with `world_identity.observation_stamp_ns`; separate state freshness checks source and receive time. The numeric freshness window is shared, but the evidence predicates are independent.

Evidence from consumers/producers:

1. Planner/runtime candidate creation sets `valid_from_ns` to activation or current ROS time and `valid_until_ns = min(declared_end_ns, activation_ns + data_freshness_window_ns)` (or corresponding `now + window` for immediate rebind).
2. `CandidateBundle::sample()` rejects samples outside `[valid_from_ns,valid_until_ns]`.
3. Runtime candidate admission and command publication recheck this interval; a temporal lease expiry produces no active sample or bounded endpoint-hold path.
4. `ExecutionAuthority::reserveAnchor`, stage/activation and splice checks use the same interval to ensure predecessor and successor are executable through activation.
5. World recertification may extend only the active bundle's lease to the supplied `now + window`, capped at analytic endpoint; it does not alter `valid_from_ns` or `activation_stamp_ns`. Pending gets no lease extension. The extension occurs only if exact active pointer/timeline/world transaction remains current and full validation/fast-path has authorized retention.
6. World-source freshness is separately assessed before commit/admission/publication from the world's observation stamp.

## Contract interpretation

The bundle field expresses a short exposure lease for the executable trajectory. A newly certified world allows runtime to renew that lease for the same exact analytic trajectory up to its endpoint. It is coupled operationally to successful recertification, but it is not equivalent to world-source freshness. No evidence supports splitting it into a second persistent deadline in this cut.

## Limits / required regression proof

This is source-derived and must remain guarded by tests proving renewal cannot alter `valid_from_ns` or activation, cannot exceed the analytic endpoint, cannot revive an already finished trajectory, and cannot happen for pending when only active renewal is authorized. Existing tests cover execution world recertification, but audit must confirm each requested lease edge. No threshold changes are authorized.
