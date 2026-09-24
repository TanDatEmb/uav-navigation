# Target typed admission pipeline

The existing `NavigationMode` remains the only PX4-boundary owner. A small pure contract assessment returns a specific shape reason; a separate pure temporal assessment evaluates the inclusive lease interval using one callback ROS timestamp. Their caller maps failures to `REJECT_RETAIN_PREVIOUS` and records typed diagnostic evidence after unlocking. State-dependent admission continues under `trajectory_mutex_` and maps exact health/session/world/order/freshness/tracking causes to their current dispositions. The only command commit remains `transitionCertifiedCommand(..., kCommit)` under the existing lock. Successful `NavigationCommandAdmission` remains after commit and is never emitted for a reject.

| Stage | Current input | Target reason family | Existing disposition |
| --- | --- | --- | --- |
| Presence | nullable message | `MESSAGE_MISSING` | retain previous |
| Wire contract | immutable received command, expected frame | frame/identity/status/finite/continuation | retain previous |
| Temporal lease | command header, valid-until, one callback ROS now | invalid now/future/expired/window | retain previous |
| Terminal ownership | adapter terminal/handover/failure state | terminal ignore | ignore after terminal |
| Session/epoch | typed health, activation, current accepted command | health/activation/mission/request/world | retain previous |
| State freshness | odometry source/receive at current boundary | odometry stale | fail navigation |
| Sample order | incoming/last accepted sample ID | nonincreasing sample | retain previous |
| Tracking | measured state and received P/V | tracking violation | safety stop |
| Terminal recovery | completed command and measured endpoint | recovery handling | existing bounded recovery |
| Commit/receipt | exact locked state | accepted | atomic commit then success receipt |

The stage/reason/disposition object is diagnostic output, not a sixth authority. No diagnostic topic is subscribed by the adapter. Expensive publish/serialization remains outside `trajectory_mutex_`; state-dependent predicates and commit use the same locked snapshot. Later setpoint updates continue to own the independent 100 ms receive lease and PX4 output checks.
