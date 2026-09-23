# PASS_THROUGH event model

`tests/pass_through_model.py` is an audit-only finite event model, not product code. Events: `MeasuredState` generates a source-stamped candidate cursor; `CrossingObserved` stores typed evidence; `ContinuationAvailable` stores a separate exact execution witness; `SafetySuffixActive` changes the permitted gate only with its own certified stop/measurement; `CertifiedStop` affects the STOP route; `LocalizationReset` and `RouteRevisionChanged` invalidate route-bound observations; `SuccessorActivated` requires an accepted gate and independent timeline admission. The executable model abstracts `MeasuredState` into `crossing` and models identity fences.

| Ordering/scenario | Candidate result | Remaining proof |
|---|---|---|
| crossing→continuation / continuation→crossing | accept only after both, no activation by acceptance | producer scheduling and bounded retention |
| crossing→new world→continuation | physical observation remains, readiness recertified | current world certificate policy |
| crossing→localization reset / route change | invalidate observation | exact producer identity |
| crossing→safety suffix | accept only at certified measured-stop exception | suffix evidence in product |
| high-speed ball skip | ordered 3-D segment crossing allowed | odometry source-time bound |
| sharp corner | no outgoing tangent requirement at mission acceptance | successor continuity gate |
| coincident PASS→STOP | next STOP owns settling; no moving PublishGoal | terminal hold ownership |
| reverse / self-intersection | reverse invalidates; ordered segment disambiguates | projection uncertainty |
| odometry gap | invalidate segment observation | source-stamp gap policy |

Invariants exercised: no acceptance without measurement; a valid crossing is not erased by unrelated readiness absence; no stale crossing crosses route/localization identity; acceptance does not activate the successor; readiness does not fabricate a crossing. These are **model proofs under stated abstraction**, not flight qualification.
