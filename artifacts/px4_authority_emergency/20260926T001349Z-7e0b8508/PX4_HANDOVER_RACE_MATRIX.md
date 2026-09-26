# PX4 handover race matrix

| Case | Required order | Current source behavior / regression coverage |
|---|---|---|
| H1 normal Hold | request, stationary stream, `AUTO_LOITER` | Source path preserved; full PX4 SITL result pending. |
| H2 callback before status | callback `Success`, then non-Loiter status, then Loiter | Success no longer clears pending; deterministic reducer test covers wait-for-status. SITL pending. |
| H3 deactivation callback | callback `Deactivated` without Loiter | Deactivated no longer confirms Hold. If executor deactivation itself occurs, `onDeactivate` invalidates generation and stops retry. |
| H4 stale callback | old callback after new activation/attempt | `(activation_generation, attempt)` mismatch is discard-only; deterministic test covers both mismatch dimensions. |
| H5 retry | accepted request remains without callback/status, deadline, retry | 250 ms request timeout now retires the old token and replaces the scheduled request; pure deadline test. Total retry count lacks an approved policy. |
| H6 status first | Loiter before callback | status clears pending/in-flight; later callback fails current-state predicate and is ignored. Reducer test. |
| H7 operator | pending Hold then explicit operator mode | mode executor deactivation stops requests and invalidates callbacks; exact SITL event/timeline pending. |
| H8 failsafe | pending Hold then PX4 failsafe | `FailsafeActivated` deactivation invalidates generation; no navigation reclaim path. Exact SITL timeline pending. |
| H9 duplicate status | repeated `AUTO_LOITER` | assignment and clearing are idempotent; no repeated completion action. |
| H10 later nav-state change | Loiter then another state | confirmation observation is no longer a pending handover; later state does not restart an old handover. New activation owns a new generation. |
| H11 reset while pending | old callback after executor deactivation/reactivation | activation generation fences callback; localization reset remains in NavigationMode's existing epoch/reset handling. |
| H12 deactivate ordering | status, scheduled callback, executor deactivation in adversarial order | status clears pending; deactivation increments generation; callbacks after either condition cannot mutate current attempt. Full ROS integration test pending. |

The deterministic tests validate the pure decision helper used at the production callback/retry boundary. They do not claim PX4 firmware/SITL qualification.
