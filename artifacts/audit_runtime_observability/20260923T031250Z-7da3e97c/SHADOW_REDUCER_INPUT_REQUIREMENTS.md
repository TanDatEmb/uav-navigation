# Minimum immutable event input for a future shadow reducer

No reducer is implemented on this branch. The shadow input must preserve facts even when product authority is ambiguous; `Decision::Unknown(reason)` is permitted only in shadow output. Every input needs a producer-local sequence, process/run incarnation, clock domain, source stamp and receive/decision stamp where applicable; a missing event or queue drop must yield `IncompleteTrace`, never a guessed state.

| Domain | Required immutable observations | Why current trace is insufficient |
|---|---|---|
| Mission | route revision/mission identity; odometry source sequence and localization epoch; exact MissionController update pair, crossing kind/error, continuation witness, accepted gate, emitted successor goal | odometry replay overapproximates callback schedule; current mode status shows outcome only |
| Execution | desired vs active/staged goal and bundle generation, role/safety transition, successor activation, stop confirmation | command samples show selected role but not every internal activation/revocation event |
| World | latest immutable world identity/source stamp; active bundle certificate identity; stale rejection, recert start/end/commit, suspend/resume/revoke | diagnostics provide sampled durations/counters, not event-paired transition |
| Command/adapter | unique command sample within producer incarnation, authorization/publish boundaries, adapter callback entry/admit/reject/lease origin and first setpoint use | no exact publish→receive pair; current proxy ends at setpoint |
| PX4 | Hold attempt generation, raw VehicleCommand/ACK/ModeCompleted/status, PX4 boot sequence, adapter status receive and callback/deactivation cause, operator/failsafe takeover | historical bags omit three topics; one focused sidecar bag captures them, but normal event stream and adapter attempt/status disposition remain incomplete |

The event ingestion model must keep independent, possibly out-of-order observations rather than collapsing protocol phases into one linear enum. Missing PX4 status after ACK/ModeCompleted becomes `Unknown(HoldAuthorityUnconfirmed)`; a status before request cannot confirm that request. A late world recertification after downstream handover becomes `UnknownOrConflict(DownstreamAuthorityUnconfirmed)` unless an explicit handover witness proves disposition. No diagnostic callback may grant flight authority.

Readiness gate for a later branch: complete/replayable event stream for O1 exact update, O2 paired receipt, O3 request/status/deactivation and O4 stale/recert/lease, with identity reset and drop handling tests. E2 legacy API policy is tracked separately from native shadow semantics. This gate is **not met**.
