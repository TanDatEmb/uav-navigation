# ADR-013: One `nav_core_node` process with one decision thread and compute lanes

**Status:** accepted (architecture), implementation in P4. Lane assignment of
individual jobs stays provisional until WP-A3/A4 are reviewed.

## Decision question

Keep world (mapping) and planning inside one navigation process, or split them
into `MappingWorldNode` / `PlanningControllerNode` connected by ROS topics?

## Decision

Keep **one process** (`nav_core_node`). Inside it:

- **Decision thread (exactly one):** every subscription callback and every
  ROS-time timer run on a single-threaded executor. They only translate inputs
  into typed `Event`s and apply reducers (`nav_execution`, `nav_mission`,
  `nav_planning_policy`). Only this thread mutates decision state.
- **Compute lanes:** worker threads that receive immutable inputs and return
  immutable results as `Event`s on a queue drained by the decision thread:
  - `mapping lane`: `MappingActor` update + snapshot publication (as today).
  - `planning lane`: long solves (`PlanningRequest → CandidateBundle`), followed
    by `nav_certifier` on the same lane (ADR-014).
  - `fast lane`: short, deadline-bound jobs (heading rebind candidates,
    retained-command recertification after a world revision, measured-state
    emergency brake synthesis). A job that misses its deadline yields a
    `DeadlineMissed` event; the reducer applies the existing fail-closed path.
- No shared mutable state between lanes and the decision thread. The target is
  ≤ 2 mutexes in the shell (the event queue and the latest-snapshot slot).

`lio_node`, `px4_adapter_node` and `odom_bridge_node` stay separate processes.

## Rationale (evidence on main)

1. **World snapshot size and identity.** Planning and certification pin one
   immutable world revision; ADR-011 builds COW snapshots in process. A
   process split would serialize a ROG snapshot every 100 ms
   (`PlanningTimingContract::kSnapshotPeriodS`) or re-implement the map on
   both sides. Invariant 3 ("newer world identity invalidates an older
   candidate") is an O(1) identity comparison in process, but a
   cross-process race over DDS.
2. **Budget.** Solve 80 ms inside a 100 ms period (HG-001). IPC latency and
   jitter would come out of the certification and commit margin.
3. **Fault containment already exists at the right boundary.** A crash of the
   navigation process lets the adapter's 100 ms command lease expire, and the
   adapter falls back to PX4 Hold. Splitting world and planning does not add a
   safety barrier the adapter does not already provide.
4. **The actual defect is inside the process, not the process count.** 5
   threads touch one object through 9 mutexes and 57 atomics (RC1). A single
   decision thread with immutable lane messages removes the lock chain
   (`navigation_runtime_node.cpp:8642-8680`) instead of distributing it over
   DDS.
5. The project already rejected `MappingWorldNode`/`PlanningControllerNode`
   (`docs/architecture/navigation_layers.md`, "Deliberately absent").

## Consequences

- Deterministic replay becomes possible: record the decision thread's input
  event log and replay it without ROS (P4 exit gate).
- Package-wide TSan must pass (currently BLOCKED).
- `create_wall_timer` is removed from product nodes (beta is SITL with
  `use_sim_time`, ADR-016).
- Revisit only if measured decision-thread latency p99 exceeds the command
  period (20 ms) on the SITL baseline machine.
