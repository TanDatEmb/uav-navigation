# ADR-011: Stage WorldModel ownership before concurrent mapping and planning

**Status:** accepted; Batch 1 implemented, runtime parity evidence pending.

## Context and evidence

The target navigation architecture gives mapping ownership to a product-owned
WorldModel and gives each planner solve one immutable map revision. The current
runtime instead owns one mutable imported map inside `navigation_runtime_node`;
the same mutually-exclusive callback first updates the map and then runs the
planner backend.

The `aist-mid360-drive` dataset establishes two relevant facts:

- At 1x, artifact `.artifacts/runtime/dataset-20260824T170750-60656` consumed
  all 2,756 accepted observations without replacement.
- At 2x, artifact `.artifacts/runtime/dataset-20260824T171418-61902` replaced
  1,378 of 2,756 accepted observations because a 10 Hz callback owned both map
  consumption and planning. The corrected report classifies this as FAIL.

The imported map backend is mutable and has no read/write synchronization
contract. The planner backend has many direct world-model queries across path
search, corridor generation and trajectory safety. Running a map-update
callback concurrently with those queries would therefore be a data race.
Holding a shared/exclusive map lock for an entire solve would avoid the race
but preserve mapping starvation and stale map revisions. Neither is the target
architecture.

## Decision

Migration is split into independently verifiable batches. No batch may claim
immutable snapshots before the backing storage is actually immutable.

### Batch 1: product query boundary, unchanged execution

Introduce a product-owned `WorldModelView` contract and a backend-backed adapter.
The contract owns UNKNOWN, OCCUPIED, KNOWN_FREE and OUT_OF_MAP semantics,
segment certification, clearance/bounds, coordinate conversion, and bounded
occupied-point extraction required by the planner backend. The planner receives
this contract, not a mutable backend map pointer.

This batch remains single-threaded and behavior-preserving. Characterization
tests compare every adapter query with the current ROG result on identical map
fixtures. It removes vendor API reach-through without claiming a scheduling
improvement.

Implementation note (2026-08-25): `navigation_world_model::WorldModelView` is
an Eigen/STL-only interface. `navigation_runtime::MappingWorldModelView` forwards
the existing backend classification, coordinate conversion, nearest-cell, ray
and ordered occupied-point queries. The planner no longer receives a mutable
backend map pointer or map-update escape hatch. Execution intentionally remains
sequential. The adapter currently observes mutable backend storage, so it is not an immutable
snapshot and must not be used concurrently with mapping; Batch 3 remains open.

### Batch 2: explicit mapping owner and revision

Introduce `MappingObservation` ingestion under one mapping owner. A successful
integration publishes monotonically increasing metadata:

- observation timestamp and sequence;
- world generation and map revision;
- corrected sensor pose;
- local bounds/resolution and visibility profile;
- input, finite-point and allocated-voxel counts;
- integration timing and failure reason.

An observation is acknowledged only after successful integration and revision
publication. The inbox remains single-latest: it does not become a FIFO that
hides overload as queue growth. Replacement, stale input and mismatch remain
explicit failures/counters.

Accounting must close at steady state and shutdown:

```text
accepted_to_inbox
  = replaced_pending + mapping_started + pending_at_stop

mapping_started
  = mapping_published + mapping_failed + in_flight_at_stop
```

### Batch 3: genuinely immutable planner snapshot

Implement `WorldSnapshot` as an immutable backing revision, not a lock guard
over the live ROG instance. A planner solve captures exactly one snapshot and
records its generation/revision in candidate, certificate, commit and command
trace. Mapping may publish later revisions without changing any query result
observed by that solve.

The first implementation may use a compact copied planning view if measured
copy p99 and memory are inside budget. Copy-on-write or page/chunk sharing is
allowed only after equivalence tests prove the same logical grid. A second
mutable ROG map that silently diverges, or a pointer to mutable vendor storage,
is not a snapshot.

### Batch 4: independent scheduling

Only after Batch 3 passes race and equivalence tests may mapping and planning
run concurrently. Command sampling remains independent and read-only over an
immutable committed trajectory bundle. Shutdown joins owners in the order:
stop ingress, cancel solve, join mapping/planning workers, then release
snapshots and vendor storage.

### Batch 5: latest-world transactional certification

A solve records its pinned source revision. Before commit it loads the latest
snapshot under a short publish/commit gate. If the revision changed, the whole
candidate is revalidated using the latest snapshot: main swept collision and
bounds under the exploratory UNKNOWN policy, backup swept collision plus
KNOWN_FREE certification, endpoints, and role intervals. A successful bundle
records both `solve_world_revision` and `certificate_world_revision`.

The gate covers latest-snapshot load, revalidation decision and atomic commit;
it never covers A*, CIRI or MINCO. If revalidation exhausts the solve deadline,
the world changes again before commit, or any certificate fails, the candidate
is stale and the existing committed bundle/history remains unchanged.

## Required contracts

- Every query returns both semantic result and snapshot revision where needed
  for a safety certificate. OUT_OF_MAP always fails closed.
- Candidate construction and retained-suffix validation use one snapshot per
  operation; revision mixing is forbidden.
- A candidate solved on revision `r` may commit only if the commit policy
  certifies it against the current revision or explicitly revalidates it on a
  newer immutable snapshot. Revision equality alone is not a collision proof.
- Mapping timestamp, planner execution-state timestamp, trajectory wall start
  and trajectory-relative time remain distinct types/domains.
- Dataset shadow planning never publishes vehicle commands.
- Planner deadline starts at scheduled dispatch and includes scheduling lag,
  snapshot acquisition, latest-world revalidation and commit-gate wait. Mapping
  latency is reported separately and is never subtracted to make solve latency
  appear compliant.

## Adversarial acceptance tests

- Query-equivalence vectors cover map boundaries, UNKNOWN, inflated occupancy,
  sliding-map wraparound, thin obstacles and nearest-cell searches.
- A mapping update concurrent with a long solve cannot alter any result from
  the captured snapshot; TSan or deterministic barriers prove this.
- Reordered/missing corrected pairs preserve observations until success and
  account for every terminal disposition exactly once.
- A map slide while old snapshots are live neither aliases cleared cells nor
  causes unbounded retained memory.
- Forced solve timeout and goal cancellation cannot commit a candidate from a
  stale solve generation or release storage still used by a command sample.
- 1x and 2x dataset replay report mapping p50/p95/p99, queue wait/depth,
  revision age, CPU and RSS. Dense shadow queries additionally report
  A*/CIRI/MINCO scale and latency without controlling hardware.
- Reset, bag rewind or frame/config epoch change increments world generation;
  revision zero in a new generation cannot alias an older snapshot.
- Shutdown is deterministic with an empty inbox, an update in flight, a worker
  waiting at the publish gate and a planner pinning an old snapshot. No worker
  is detached and no publisher/logger is used after node teardown begins.

## Rejected alternatives

- Merely increase the existing planner timer rate.
- Disable or relax freshness/drop gates for dataset replay.
- Add an unbounded observation queue.
- Add any FIFO merely to make accelerated dataset replay PASS.
- Protect the mutable ROG map with a lock held for the full planner solve and
  call that an immutable snapshot.
- Double-buffer two independently evolving ROG maps without deterministic
  delta replay/equivalence.
- Expose a ROS per-voxel service on the planner hot path.
- Reject every `latest_revision != solve_revision` without latest-world
  revalidation; normal sensor updates would make long solves impossible to
  commit.

## Relationship to ADR-010

ADR-010 remains an accurate description of the currently deployed runtime but
is no longer the final architectural decision. During Batches 1-3 the boundary
remains in-process to avoid a premature ROS snapshot protocol. Once each batch
is implemented, `navigation_layers.md` must describe only the contracts that
exist in source and tests.
