# Immutable navigation snapshot architecture

The runtime product path is now split at a deliberate ownership boundary:

```text
FAST-LIO -> LidarMappingObservation -> MappingWorldNode
                                      -> WorldSnapshot (Inflated only)
                                      -> PlanningControllerNode
                                      -> TrajectoryBundle -> PX4 External Mode
```

`MappingWorldNode` is the sole mutator of the vendor ROG-Map. After a mapping
update commits, `WorldSnapshotBuilder` holds the map lock only for one bounded
copy. The planner receives `ImmutableWorldSnapshot`, whose cell reads are
vector/index operations and cannot observe a mixed generation or revision.
The dynamic envelope is clipped to the valid map domain and `max_cells` is a
hard fail-closed limit.

`WorldSnapshot.msg` carries provenance (`snapshot_id`, generation, revision),
frame/geometry metadata, source timestamp/age, and the dense inflated cell
states. It intentionally does not expose probability cells or ROG query APIs.
The planning node rejects malformed, stale, out-of-order, wrong-frame, and
over-budget snapshots. Snapshot timeout also prevents an old map from driving
continued motion.

The existing `NavigationRuntimeNode` remains available as the `legacy` launch
mode. `PlanningControllerNode` reuses its already verified trajectory
handover, safety fallback, and PX4 bundle serialization while running with
`mapping.enabled=false`; this keeps the rollout reversible while the pure
controller extraction and repeated SITL A/B gate are completed.

Launch modes:

- `composed` (default): both components share a multi-threaded container and
  enable intra-process communication;
- `standalone`: mapping and planning are separate processes;
- `legacy`: the pre-split rollback node.
