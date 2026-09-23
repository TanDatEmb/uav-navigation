# Identity architecture critique

`PlanningKey` already has default structural equality (`planning_request.hpp:33-53`), so adding `PlanningContextId` is useful only if reducer ownership makes that equality the **single stale-result admission point**. It cannot replace semantic identities or independent certificate checks. `PlanningRequest::valid()` rechecks route snapshot, goal, anchor, state source time, world generation/revision/body support and dynamics (`:104-156`); some comparisons are safety certificates rather than incidental duplication.

| Identity | Semantic fact and invalidation | Why preserve |
|---|---|---|
| mission ID / waypoint index / request ID | mission session and accepted gate; new mission/acceptance | route order, cross-process message provenance |
| goal epoch | runtime desired intent revision; new accepted goal | reject old intent while predecessor still executes |
| localization epoch | estimator reset frame fence | prevent old world/trajectory/state resurrection |
| route revision | immutable route geometry version | projection and corridor provenance |
| world generation/revision/observation stamp | latest versus certified map evidence | recertification and freshness remain independent |
| bundle generation / activation stamp | executable predecessor/successor timeline | atomic future cutover and old sample rejection |
| transaction ID | store commit ordering | protects against interrupted/replayed commit attempts |
| sample ID | transport sequence | adapter replay/reorder rejection |
| dynamics hash | physical/mission limits bound to solve | reject stale plan under changed envelope |

`PlanningContextId` could be a typed derived comparison key over localization, intent/request, route, world, active predecessor, start mode/anchor and dynamics. It should be computed at snapshot capture and carried by immutable result. A world recertification may update the certified world without changing trajectory generation; the key/invalidation rule must explicitly handle this. Raw identity fields remain in candidate, command, trace and audit output.

TARGET publication still compares localization, desired goal epoch, command goal epoch, desired/executing goal identity, Episode availability and Store pointer (`navigation_runtime_node.cpp:8595-8615`). The metric script reports 52 selected helper-name occurrences across runtime and Store, including definitions. It does **not** establish how many are redundant or eliminable. A call-graph/transition audit must separate stale-result equality from independent world/lease/physical checks before setting a reduction target.
