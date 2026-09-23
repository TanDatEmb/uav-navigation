# Safety invariants to preserve

The current contract is `docs/safety/runtime_safety_current.md`; targeted lineage was read in `docs/safety/runtime_safety_index.md:122-139` and archived decisions around lines 1924-1950, 2114-2138, and 2160-2193. This artifact records checks, not qualification.

1. Desired gate N+1 may coexist with active execution N. A valid predecessor remains publishable until exact successor activation, certified safety transition, lease/certificate expiry or PX4 handover.
2. Candidate admission requires the same goal/localization/world/lease/corridor/dynamics/flatness/continuity witnesses. No threshold, UNKNOWN policy, planner algorithm, deadline or adapter lease changes.
3. Staged successor cannot execute before its activation timestamp. Finalizer failure restores active predecessor; stale planner results cannot replace or revoke it.
4. World recertification is exact-version conditional. Valid active may survive newer world after revalidation; invalid pending is independently disposable. World change alone does not grant authority.
5. Safety-owned BACKUP/emergency remains one-way until measured stop. A sampled MAIN prefix can still belong to a frozen safety suffix.
6. Analytic STOPPED_HOLD, measured stop and PlanFromRest request remain distinct. Planned endpoint alone cannot accept a mission waypoint or terminal completion.
7. Localization reset invalidates active/staged execution and its identity before new-epoch ingress; old samples and planner results cannot regain authority.
8. Final publication checks exact active execution token, world, source/receive freshness, command/bundle lease, frame/state witness and independent PX4 boundary. The repaired desired-versus-executing publication behavior remains.
9. MissionProgress is the sole mission writer; WorldModel owns latest world; adapter owns receive lease, tracking/state/health/frame boundary and Hold protocol. CommandAdmission is adapter-local receipt, not Core execution authority.

The 3/3 repaired `long_featured`, seed 0, tracking=off evidence at the base commit is the behavioral reference. It is diagnostic SITL evidence, not flight qualification.
