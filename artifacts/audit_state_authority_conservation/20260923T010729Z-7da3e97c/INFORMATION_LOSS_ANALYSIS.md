# Information loss test

| Proposed reduction | Information retained/derived | Information lost if done naively | Required representation |
|---|---|---|---|
| one scalar `route_progress_m` | monotonic distance; on a nondegenerate pinned route, segment can sometimes be derived from arc | current measured projection versus clamped progress, reverse/backtracking witness, observation time and whether gate was accepted | measured cursor `(segment, u, arc, source time, epoch)` plus accepted gate; prove any derivation against loops and zero-length legs |
| one execution FSM | nominal/stop display | staged successor, world certificate, source/receive age, role sample and PX4 state | orthogonal execution, world lease, mission and PX4 protocol records |
| delete active identity copies | active bundle key can derive some copies | transitional old active versus new desired, especially hot retarget | keep active bundle immutable key; transactionally update desired separately |
| remove adapter command cache | producer command exists elsewhere | independently received sample order, local receive time, last valid reference | receiver-owned accepted command/lease |
| delete Hold flags | API callback/status could derive them if event log retained | in-flight operation, retry deadline, external nav-state observation | tagged Hold protocol state + status witness |
| world stale → immediate brake | simple stop decision | last certified world freshness and still-valid lease window | explicit certificate expiry and one-way stopping transition once expiry occurs |
| callback-only crossing | latest position retained | measured segment crossing when successor witness arrives later | event-sourced measured crossing/cursor with bounded validity and identity, subject to route/epoch reset |

No currently audited behavioral field is approved for `DELETE`. A renamed flag without an information/owner reduction does not count as architectural simplification. Capability loss from a target policy change, including braking one-way behavior, must be called out and tested rather than hidden as refactor.
