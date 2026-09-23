# Target State Model V3 — evidence update only

The prior `TARGET_STATE_MODEL_V3.md` remains **NOT_ADMITTED**. This branch adds no V4 field/state and approves no architecture policy. `measured crossing != accepted mission progress`, `desired intent != active execution`, `latest world != active certified evidence`, `Hold request != ACK != ModeCompleted != VehicleStatus`, and `runtime publish != adapter receive` remain the working independent facts; no new evidence refutes them.

| V3 question | New evidence | Current disposition |
|---|---|---|
| PASS crossing and readiness ordering | 32 episode-level replay, one in-ball-before-published-readiness/reentry; exact callback unavailable | **UNRESOLVED** retention/consumption; no lost crossing reproduced |
| Native braking lifecycle | 21 refs/history/GitHub owner search finds no current native caller of legacy `onTrajectory`; native BACKUP/emergency source remains one-way | **Current native policy supported by source**; exported API compatibility undecided, do not add recovery state solely for it |
| Hold authority | Run binary/SHA/dirty patch pinned; source suggests ACK intention and persistent Loiter without ModeCompleted; one focused bag directly orders executor Hold request→ACK→Loiter with charge 1; no Loiter ModeCompleted in that window | **UNRESOLVED** request/status attribution, stale status, retry termination; product firmware baseline unpinned |
| World and command lease | 68,936 exact command→first-setpoint proxy pairs but zero publish→receive pairs; no stale-world transition | **UNRESOLVED** temporal contract and downstream authority feedback |

The ability of a future shadow reducer to output `Unknown(reason)` does not create the missing input event. Current traces cannot tell it whether a crossing was calculated on a particular MissionController update, whether a Hold status belongs to a request, or whether a stale-world pause crossed the adapter lease before runtime resume. `SHADOW_REDUCER_READY=NO`. A future read-only reducer should receive event-order/identity evidence first and may then model remaining policy uncertainty explicitly; authority migration remains NO regardless.
