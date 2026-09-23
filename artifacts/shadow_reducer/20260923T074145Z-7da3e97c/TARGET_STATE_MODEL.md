# Target semantic state model exercised by the reducer

The executable implementation is `tools/shadow_reducer/model.py`. There are exactly three mutable authority/observation domains; `ShadowReducer.evidence` is a diagnostic comparison quality marker.

```cpp
struct PlanningContext { LocalizationEpoch localization; MissionId mission;
  RouteRevision route; WorldGeneration world_generation; WorldRevision world_revision;
  GoalEpoch goal_epoch; WaypointIndex waypoint; RequestId request_id; DynamicsHash dynamics_hash; };
struct MissionProgressState { RouteIdentity route; RouteCursor measured;
  optional<CrossingWitness> crossing; AcceptedWaypoint accepted;
  optional<GateIdentity> current_gate; optional<ContinuationWitness> continuation; };
struct ExecutionAuthorityState { ExecutionPhase phase; optional<CertifiedTrajectory> active;
  optional<CertifiedTrajectory> staged; optional<PlanningContext> context;
  optional<BundleGeneration> fenced_generation; };
struct Px4AuthorityState { optional<LocalBoundaryWitness> local;
  optional<ControlReferenceKey> last_reference;
  optional<Deadline> receive_lease; optional<HoldTransfer> hold;
  optional<VehicleStatusWitness> latest_status; };
```

The PX4 `local` witness contains source/receive stamps, localization/frame generation, measured PX4 position/velocity, translation, heading and health receipt; current audit trace does not carry all values and replay leaves it `Unknown`. The existing adapter still retains the complete local P/V/A, validity flags, frame transform and reset state (`KEEP` rows in the conservation matrix); the shadow receipt is not a substitute for those product inputs. The Python model uses mutable dataclasses to exercise event order. A production translation should encode `Released(fenced_generation)` and `CommittedStopping(active)` as tagged lifecycle payloads, because the optional combinations are mutually constrained. `active` and `staged` remain independent. `HoldTransfer` preserves request, ACK, callback, status, retry deadline, and pre-request status timestamp as orthogonal facts; a linear enum loses orderings. `PlanningContext` includes all pinned stale-result identity dimensions; dynamics hash is unavailable in current audit event and remains `Unknown` in replay, so replay never claims planner admission equivalence from it. It is a derived immutable equality key, not another writer-owned authority store.

Derived views, never independent latches: `commandAvailable(active, phase, certificate, now)`, `currentTrajectoryRole(active)`, `restartFromRestNeeded(phase == Stopped)`, `missionComplete(accepted, route)`, `holdObserved(hold.status)`. WorldModel owns the latest immutable world; the active trajectory owns its certified world identity. Workers compute from immutable requests and return immutable results for Core admission; the reducer does not run A*, CIRI, MINCO, world export, or tracing in the flight path.

**Information limits:** replay event cursor is route ordinal with unknown 3-D geometry/arc, and no independent measured velocity at every audit event. These are `Unknown`, not invented. The production type must carry ordered projection and measured source stamps to close the physical interpretation.
