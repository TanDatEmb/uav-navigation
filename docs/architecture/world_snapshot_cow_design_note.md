# Immutable world snapshot representation decision

## Scope

This note compares the current immutable snapshot representation before any
behavioral COW change. The planner continues to consume only
`WorldModelViewPtr`; it never receives mutable ROG-Map storage.

## Alternatives

| Option | Representation | Main benefit | Main risk |
|---|---|---|---|
| A | Full contiguous base and inflated arrays | Simple indexing and direct parity | Every publication copies the full grid; high tail and allocator pressure |
| B | Exact-region immutable parent/patch chain (current) | Small steady-state exports and exact changed-region history | Query cost grows with patch depth; an AABB is not an exact cell/chunk certificate |
| C | Fixed-size immutable base/inflated chunks with shared handles | Bounded query depth, reuse of unchanged chunks, exact dirty-chunk provenance | Requires a new index/layout contract and cell-by-cell parity proof, including slide/boundary semantics |
| D | Snapshot builder worker | Removes export work from mutation callback | Queue supersession, revision ordering and stale-result cancellation become new safety state |

## Decision

C is the default direction, but it is not enabled by this change. The current
repository has no fixed-chunk index contract and the existing patch path is
still the only implementation with exercised parity tests. A COW prototype is
allowed only after adding a reference full-export oracle and proving equality
for FREE, UNKNOWN, OCCUPIED, inflated, virtual planes, positive/negative slide,
boundaries and OUT_OF_MAP. The candidate trajectory must also produce identical
A*, MAIN/BACKUP certificate and commit decisions on both representations.

## Required implementation contract

Each successor must retain immutable metadata, world identity/provenance, and
shared immutable handles for both base and inflated chunks. A changed cell
means a planning-state transition, not merely a ray AABB touch. Timestamp-only
updates may create metadata-only successors. Missing dirty history, revision
gaps, window slides or ambiguity must use the existing full-export fallback.

Until the parity and replay measurements exist, this remains a design/benchmark
artifact and must not replace the current full/patch implementation.
