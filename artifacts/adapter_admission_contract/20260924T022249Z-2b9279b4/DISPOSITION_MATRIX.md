# Existing disposition preservation matrix

| Cause | Old action | Target typed disposition | Existing command |
| --- | --- | --- | --- |
| Null / malformed shape / invalid time | increment rejected; `kRetain`; return | `REJECT_RETAIN_PREVIOUS` | retained |
| Terminal/handover/failure already active | return before identity/tracking | `IGNORE_AFTER_TERMINAL` | unchanged |
| Health/activation/mission/request/world identity mismatch | increment rejected; `kRetain`; return | `REJECT_RETAIN_PREVIOUS` | retained |
| Odometry stale | increment rejected and stale-state count; `failNavigation` after unlock | `REJECT_FAIL_NAVIGATION` | current fail-close path |
| Sample ID nonincreasing | increment rejected; return | `REJECT_RETAIN_PREVIOUS` | retained |
| Tracking envelope invalid | increment rejected; `safetyStopNavigation` after unlock | `REJECT_SAFETY_STOP` | current safety-stop path |
| Valid command | commit and refresh receive lease; success receipt | `ACCEPT` | replaced atomically |
| Completed accepted command | commit/receipt then bounded recovery or terminal hold logic | `ACCEPT` + existing terminal action | exact accepted command |

The matrix is a source-derived baseline, not permission to change a disposition. Frozen old-predicate unit comparisons and typed adapter unit tests cover the state-independent contract and session cases; the full dynamic-state equivalence remains bounded by component tests and focused SITL, not a claim that every possible callback interleaving was exhaustively replayed.
