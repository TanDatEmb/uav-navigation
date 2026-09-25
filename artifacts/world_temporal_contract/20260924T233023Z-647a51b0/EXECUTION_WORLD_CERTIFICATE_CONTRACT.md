# Execution world certificate contract (AS-IS)

An active/pending `CandidateBundle` carries immutable solve provenance (`pinned_world_identity`) and its current certified world (`world_identity`). `ExecutionAuthority` separately stores the world identity for the active/staged timeline transaction. The mapping callback may advance both only after the exact execution snapshot still matches its expected timeline/pointers and each bundle has either a sound disjoint-change proof or full immutable revalidation.

The bundle world validator is executed outside publication/authority locks. It is bound to the exact candidate's geometry, role schedule and policy and reports the evaluated bundle generation. Runtime additionally checks world localization/generation/revision and source stamp. Planner mutable warm-start state is not used as validation authority.

When a new world revision arrives:

- active disjoint from complete change history: retain with new certificate identity;
- active intersects or provenance is unknown: fully validate exact candidate on new snapshot;
- active full validation fails: exact active and staged state are revoked/fail-closed before new world publication;
- pending is independently disjoint/full-validated; pending failure discards pending without revoking valid active;
- stale timeline/pointer/world callback: `kSuperseded`, no execution or world pointer mutation;
- source freshness expiry without a new world: exact active execution exposure becomes suspended, identity remains; automatic resume is not allowed from freshness alone. A later fresh published world must recertify and exact currentness/latches must still pass.

Only ExecutionAuthority may mutate active/staged command authority. WorldSnapshotStore is the single world publication boundary. Localization reset owns an outer transition fence and drains/reset mapping; a higher-epoch snapshot causes old map rebuild and identity-domain separation.

Unresolved evidence: runtime SITL has not yet demonstrated the isolated world-source stale → suspension → new-world exact recertification/resume path on this branch. No C0-SW world transaction lifecycle events are currently emitted as a dedicated producer-owned transaction class.
