# PX4 Hold handover contract

## Current implementation after this repair

`NavigationMode` owns command invalidation and the stationary setpoint stream. `NavigationModeExecutor` owns requests to PX4. The two are not a new shared authority: PX4 `VehicleStatus.nav_state` remains the only observed evidence that the downstream mode is `AUTO_LOITER`.

The executor models three separate events:

1. `scheduleMode(AUTO_LOITER, callback)` requests a mode transition. Its synchronous `Success` is a successful `VehicleCommandAck`; the callback later reports a matching `ModeCompleted` or cancellation/deactivation.
2. The callback reports how the scheduled request was processed. `Success`, `Deactivated`, and failures do not establish the current `nav_state`.
3. A `VehicleStatus` sample whose `nav_state` is `NAVIGATION_STATE_AUTO_LOITER` confirms the requested Hold state. That sample clears pending/in-flight retry state.

Each scheduled attempt captures `(activation_generation, attempt)`. Executor activation/deactivation advances the generation; each retry advances the attempt. A callback can mutate only the matching current pending/in-flight attempt. A late callback after status confirmation, retry, deactivation, or reactivation is ignored.

Retries are serialized and rate-limited by the existing 250 ms period, including when the library has accepted a request but has not delivered `ModeCompleted`. At the deadline the executor retires the old attempt token before `scheduleMode()` cancels/replaces its pending callback; no two requests remain scheduled concurrently. This branch does not add a maximum attempt count: source/policy does not define what authority should do after the last attempt while the stationary stream is still the only safe outgoing setpoint. That unresolved bounded-total-retry policy is recorded in `OPEN_FINDINGS.md`.

## Operator and failsafe handover

`ModeExecutorBase` documents executor deactivation when failsafe is entered or a user switches out. `onDeactivate(FailsafeActivated)` records the downstream failsafe transition in logs and invalidates the callback generation; `onDeactivate(Other)` stops retries and invalidates callbacks when PX4 leaves the mode executor. Neither path resumes navigation. A non-AUTO_LOITER `VehicleStatus` alone is not classified as operator takeover because it can also represent transition or failsafe state.

## Reactivation

Every executor activation starts a new callback generation. `NavigationMode::onActivate()` independently increments `mode_activation_id_`, invalidates its cached command, and requires a newly admitted command under that activation. The executor does not restore the prior handover's command or completion callback.

## Stationary stream

After safety stop/failure, `NavigationMode` invalidates the moving command, latches a measured hold position when available, and publishes its existing stationary setpoint until mode deactivation. This branch does not change its freshness or timeout behavior. The stream's unbounded duration while Hold is unconfirmed remains an explicit policy debt; no alternate control owner was added.
