# Open findings

## P1 — total Hold retry policy is unspecified

- Owner: PX4 authority/product safety policy.
- Current behavior: a failed/deactivated/unconfirmed attempt is retried at a 250 ms minimum interval while the navigation executor remains active; the stationary stream continues.
- Risk: retries and the stream have no total duration/attempt ceiling. Stopping the stream before PX4 authority is observed could remove the only active setpoint; retrying indefinitely may continue an alternate control stream.
- Why not changed: no approved terminal action is specified for a persistent failure to enter `AUTO_LOITER`; choosing a count or timer would create policy and authority behavior.
- Evidence needed: reviewed policy for terminal retry exhaustion and expected downstream authority when `AUTO_LOITER` cannot be established.
- User decision requested: choose a finite retry cap followed by escalation while retaining the stationary stream, or continue retries at 250 ms until `AUTO_LOITER`; no answer received at this checkpoint.

## P1 — operator/failsafe HIL/SITL timeline missing

- Owner: integration evidence.
- Current source: pinned `ModeExecutorBase` deactivates the executor on failsafe or a user switching out; `onDeactivate()` fences callbacks and stops retries. Status alone does not distinguish an operator takeover from transitional non-Loiter state.
- Evidence needed: raw VehicleStatus + executor deactivation ordering for operator takeover and failsafe takeover on pinned PX4 binary.

## P1 — EMERGENCY end-to-end runtime witness missing

- Owner: runtime/PX4 integration test.
- Current coverage is split across planner/Core, command contract, and adapter source guards. No branch-local run yet observed a certified Emergency identity at PX4 setpoint boundary.
- Evidence needed: genuine measured emergency candidate, certificate/commit, adapter admission, PX4 input trace, and uncertifiable fallback to Hold.

## P2 — C0-SW handover event reduction

- Owner: runtime evidence/reducer.
- Existing mode-status and PX4 input traces do not yet provide an explicit, loss-accounted request/callback/VehicleStatus correlation row for the entire handover episode.
- Evidence needed: minimal producer-owned events added to existing reducer with generation/attempt identity and zero unresolved/conflicts/drops.
