# PX4 Hold event-order model

`tests/px4_hold_model.py` explores ACK/Status/ModeCompleted permutations, completion with no status, rejection, stale/pre-request status, `Deactivated`, operator takeover and failsafe. It keeps request, ACK, callback, observed status, retry and external cause in separate fields. Model logical timestamps 99/101 are ordering markers, not milliseconds.

| Sequence | Model classification | TARGET risk |
|---|---|---|
| ACK→Status→ModeCompleted | confirmed from fresh status; callback diagnostic | status may race callback |
| ACK→ModeCompleted→Status | pending until status, then confirmed | TARGET clears pending early |
| ACK→ModeCompleted→no Status | unconfirmed; retry/escalate | TARGET no retry |
| ACK→no ModeCompleted→no Status | unconfirmed; bounded retry/escalate | TARGET `in_flight` blocks retry indefinitely |
| ACK→Deactivated | external cause unknown; no Hold success | TARGET clears pending |
| Rejected | unconfirmed; retry subject to policy | TARGET schedules 250 ms retry |
| Status→callback | require status post-request/epoch; otherwise stale | cached status may be misattributed |
| operator takeover / failsafe | explicit external authority state | `DeactivateReason` distinguishes failsafe vs Other only |
| duplicate/stale Status | does not regrant authority | freshness/sequence needed |
| executor shutdown | terminal unknown authority until status | callback cannot grant flight authority |

`FACT_FROM_PINNED_DEPENDENCY`: callback is driven by `ModeCompleted` or `ScheduledMode::cancel`; neither is a state confirmation. `INFERENCE`: a fresh AUTO_LOITER status may be enough for *mode occupancy* but executor ownership handover still needs firmware semantics. Model invariants are conditional on this witness contract; they are not an end-to-end proof.
