# World identity and time audit

This is a focused refresh of the pinned world audit at `artifacts/world_temporal_contract/20260924T233023Z-647a51b0/` against base `aaa0533a6b747432a501f17c49d5229118055f7a`.

| Fact | Producer / owner | Meaning and clock | Safety use |
|---|---|---|---|
| `generation` | `MappingActor` / `WorldSnapshotStore` | Mapping-world incarnation; integer identity, not time. Current producer advances it on localization epoch change. | Prevents certification crossing a localization/world incarnation. |
| `revision` | `MappingActor` and immutable snapshot store | Monotonic world content version within generation. | New immutable world causes active/pending recertification or invalidation transaction. |
| observation/source stamp | Registered scan / mapping snapshot | ROS source timestamp associated with observed map evidence. Compared in ROS time against configured 500 ms freshness window. | Candidate admission and command exposure freshness checks. |
| receive/integration/publication time | No single first-class world identity field | Runtime/map callback and store activity are observable in process, but the world identity itself does not carry a separate steady receive or publication timestamp. | Cannot independently classify source-stamp-current but stalled publication/integration. |
| certification time | Candidate validation / world callback | Current evaluation time is supplied to candidate validation; it is not itself the source observation time. | Supports validity horizon and immutable certificate refresh. |
| localization epoch | estimator health / runtime / mapping actor | Causal estimator epoch, not a timestamp. | Fences old mapping work, world identities and execution candidates after reset. |

No arithmetic between ROS and steady clocks was found in the reviewed freshness checks. A fresh source stamp is not proof that world publication/integration has progressed; that remains an explicit observation gap.

The full producer/consumer inventory and source paths are in `artifacts/world_temporal_contract/20260924T233023Z-647a51b0/WORLD_IDENTITY_CONTRACT.md` and `WORLD_TIME_DOMAIN_AUDIT.md`. No threshold or identity semantics changed in this branch.
