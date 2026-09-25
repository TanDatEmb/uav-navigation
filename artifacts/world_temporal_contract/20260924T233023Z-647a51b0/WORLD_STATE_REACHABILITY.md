# World/execution state reachability

| World/certificate state | Active | Pending | Exposure/result |
|---|---|---|---|
| Initial placeholder `(revision=0, stamp=0)` | none or prior epoch drained | none | World pointer may exist, but CandidateBundle validity/admission rejects it as uncertifiable. |
| Current fresh world, identities equal | certified active | optional certified pending | Normal exposure if state/latch/lease checks pass. |
| New world, disjoint complete change history | exact prior active | independently tested pending | Recertified immutable copies receive new `world_identity`; active may renew bounded lease. |
| New world, intersects/unknown history, full validation passes | exact active | independently tested pending | Same trajectory retained after full swept revalidation. |
| Active full validation fails | none; lifecycle fail-closed | dropped | New world publishes only after exact command authority removed. |
| Pending invalid, active valid | active retained | none | Pending rejection does not revoke active. |
| World source stale | identity/bundle retained | unchanged until other policy | Exposure `kSuspended`; no command publish. |
| Fresh world recertifies suspended exact incumbent | same active generation | independently retained/dropped | Runtime may resume only after exact owner/localization/lease/latch checks. |
| Localization reset | prior execution invalidated/drained | prior pending invalidated | New epoch not command-ready until new valid world snapshot. |

Reachable facts remain orthogonal: world pointer freshness, candidate certificate world, exposure state, active/pending ownership and vehicle-state lease. This table describes source behavior; isolated runtime fault proof remains outstanding.
