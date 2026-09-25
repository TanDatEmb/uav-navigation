# World identity contract (AS-IS)

## Identity fields

| Field | Producer / owner | Source rule | Clock / reset | Consumers and safety meaning |
|---|---|---|---|---|
| `localization_epoch` | Runtime ingress accepts producer-supplied epoch; `MappingActor` adopts only a greater epoch | Lower epochs reject. Greater epoch rebuilds backend, increments generation, resets revision/source ordering/history. | Integer causal epoch, not a timestamp. | Prevents mapping/candidate/state evidence crossing localization reset. |
| `generation` | `MappingActor` local monotonic counter | Starts at 1; incremented on greater localization epoch before first integration in the new map. uint64 exhaustion throws. | Integer map incarnation. | Prevents revision reuse across rebuilt map storage. `WorldSnapshotStore` requires larger generation within same localization epoch. |
| `revision` | `MappingActor` local counter | Starts at 0 for initial placeholder; each successful advancing integration increments. Resets to 0 on localization epoch transition. | Per-generation sequence. | Orders immutable map observations; executable candidates require nonzero. |
| `observation_stamp_ns` | RegisteredScan header stamp transported into `MappingObservation` | Must be positive; within same localization epoch it must strictly increase relative to prior integrated stamp. | ROS/sensor-source time as transported; this checkout does not record independent receive/publication time in identity. | Source-freshness input and part of exact identity, not proof of mapping receive cadence. |

`sameWorldSnapshotIdentity` compares all four fields. `MappingActor::initialSnapshot()` deliberately publishes `(epoch, generation, revision=0, stamp=0)` as a present but uncertifiable placeholder. `CandidateBundle::valid()` requires positive generation/revision/stamp for both pinned and certified world identities.

## Advances and resets

`WorldSnapshotStore::strictlyAdvances(current,next)` accepts: higher localization epoch; within the same epoch, higher generation; within same epoch/generation, strictly higher revision and nondecreasing source stamp (`>=`). Thus equal-source-stamp revision advance is accepted by the store, while the live MappingActor producer rejects non-increasing stamps in one epoch. The equal-stamp case is therefore store-level API behavior, not reachable through ordinary MappingActor input absent another producer.

For higher generation, the store does not compare observation stamps; a generation reset may lawfully restart source time only if the producer's reset contract says so. The live MappingActor increments generation only on localization epoch advance in the reviewed source. Same-epoch backend reconstruction/bag rewind/config reset generation semantics are not implemented in this producer path. ADR-011 describes broader reset/rewind/config generation behavior as target/design intent, so that language is not proof of current runtime behavior.

## Decision

- Same generation + higher revision + later source stamp: accepted.
- Same generation + higher revision + equal source stamp: accepted by store but rejected by MappingActor's ordinary producer; remains explicitly non-proven as a valid live event.
- Same generation + higher revision + regressed stamp: rejected by store and producer.
- Same revision + changed stamp: rejected by store.
- Higher generation + older stamp: accepted by store; ordinary current producer only creates higher generation on localization reset and clears old timestamp baseline. General same-epoch generation reset is unspecified.
- Zero revision/stamp: may identify initial placeholder in store; cannot certify candidate.
