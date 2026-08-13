# UAV Navigation Development Roadmap

Status: active development plan

This document defines the staged development path for `uav-navigation` after the current odometry/PX4 baseline. The central rule is to establish a working reference system first, validate each system boundary with measurable acceptance criteria, and postpone large-scale algorithm refactoring until the full navigation pipeline has been proven end-to-end.

The roadmap deliberately separates project-owned architecture from external reference algorithms. External implementations such as IKFoM, ikd-tree, ROG-Map, and SUPER planning components may be used near-upstream during development, but they must not own the public architecture, runtime contracts, PX4 integration, or safety semantics of this repository.

---

## 1. Engineering principles

### 1.1 Working baseline before optimization

Do not rewrite or heavily refactor a proven external algorithm before the complete subsystem using it has been validated.

Development order:

```text
reference implementation
        ->
project integration
        ->
functional validation
        ->
end-to-end validation
        ->
profiling
        ->
targeted refactor/optimization
```

Performance or architecture changes must be justified by measured latency, CPU, memory, data-copy, allocation, contention, or correctness evidence.

### 1.2 Freeze completed subsystems

After a phase passes its closure tests, freeze its algorithmic behavior during subsequent feature development.

A frozen subsystem may be reopened only for:

- a reproducible regression;
- an interface contract defect;
- a measured performance bottleneck that blocks the next phase;
- a safety/correctness issue.

Do not reopen a frozen phase for cosmetic cleanup or speculative optimization.

### 1.3 Project owns boundaries; upstream owns algorithms during bring-up

The repository owns:

- ROS 2 topics and QoS contracts;
- frames and timestamps;
- subsystem validity/freshness contracts;
- PX4 integration;
- trajectory execution;
- runtime safety;
- tests and acceptance criteria;
- configuration composition;
- provenance of external source.

During the reference phase, upstream code owns its internal algorithm implementation.

### 1.4 No duplicate subsystem responsibilities

Keep these maps distinct:

```text
FAST-LIO registration map
    -> estimator-private geometric support map

ROG-Map navigation map
    -> free / occupied / unknown world model for planning
```

Do not expand FAST-LIO's ikd-tree map into the navigation world model.

### 1.5 Prefer thin integration over local forks

External algorithm code should stay as close as practical to a pinned upstream revision.

Allowed early vendor changes:

- ROS 2/Jazzy build compatibility;
- compiler/Eigen/PCL compatibility fixes;
- defects required to make the upstream implementation function correctly in the project environment.

Avoid early vendor changes for:

- naming cleanup;
- style cleanup;
- new threading;
- new containers;
- algorithm redesign;
- speculative performance optimization;
- architectural beautification.

Record upstream source, revision, and local patches.

---

# 2. Target system architecture

The intended product architecture is:

```text
                     UAV NAVIGATION

LiDAR + IMU
    |
    v
+---------------------------+
| State Estimation          |
| FAST-LIO / IKFoM / ikd-tree|
+-------------+-------------+
              |
              | corrected registered scan
              | corrected / propagated state
              v
+---------------------------+
| Navigation World Model    |
| ROG-Map                   |
| occupancy / sliding /     |
| inflation                 |
+-------------+-------------+
              |
              v
+---------------------------+
| Planning Reference Core   |
| SUPER-derived planning    |
| path / corridor / traj    |
+-------------+-------------+
              |
              | time-parameterized trajectory
              v
+---------------------------+
| Project-owned Execution   |
| trajectory sampling /     |
| tracking / PX4 interface  |
+-------------+-------------+
              |
              v
+---------------------------+
| Runtime Safety            |
| localization / map /      |
| planner / tracking gates  |
+-------------+-------------+
              |
              v
             PX4
```

The boxes represent system boundaries, not final package count. Package structure may be simplified during the final refactor after the complete system is profiled.

---

# 3. Phase P0 — Estimation and PX4 odometry baseline closure

## Objective

Formally close the current odometry baseline before navigation-map development.

The current estimation implementation is considered functionally mature enough to freeze once the closure run passes.

## Required verification

```bash
make build
make test
make dataset-check DATASET=aist-mid360-drive RATE=1.0
make sim-check
make sim-check
make sim-check
```

The three SITL runs must be consecutive and not selectively reported.

## Dataset acceptance

- FAST-LIO reaches `TRACKING` and finishes `TRACKING`.
- Corrected odometry is finite and timestamp-monotonic.
- Corrected output obeys the documented frame contract.
- Propagated odometry remains approximately 50 Hz.
- IMU drop = 0.
- LiDAR drop = 0.
- timestamp regression = 0.
- queue overflow = 0.
- no unexplained `LOST` during the accepted replay window.
- no unexplained persistent `DEGRADED` state.
- cleanup passes.

## SITL acceptance, three consecutive runs

- deterministic offboard scenario completes through DISARM;
- LIO remains navigation-valid during flight and finishes `TRACKING`;
- corrected and propagated odometry remain finite and continuous;
- external odometry publication is valid when its documented gate conditions are satisfied;
- no persistent geometric-jump latch;
- no timestamp regression;
- no IMU/LiDAR drop;
- no queue overflow;
- PX4 status and odometry remain valid;
- cleanup passes after each run.

Startup/readiness gate transitions are acceptable if documented and self-recovering. Unexplained publication loss during the flight window is not acceptable.

## Closure action

After PASS:

1. remove or throttle temporary debug-only logging that is no longer useful;
2. reconcile stale configuration comments without speculative retuning;
3. record the exact tested commit and test results;
4. create an explicit closure commit, e.g.:

```text
test(p0): close odometry runtime baseline
```

5. freeze estimation/PX4 odometry behavior for normal feature development.

## Deferred work

Physical Mid-360 + physical PX4 validation is intentionally deferred until hardware is available. It remains a required later validation stage, not a cancelled task.

---

# 4. Phase P1 — Navigation world-model baseline with ROG-Map

## Objective

Establish an independent local 3D occupancy world model suitable for obstacle-aware planning without modifying FAST-LIO internals.

ROG-Map is the selected reference implementation.

## Source strategy

Use the ROS 2 ROG-Map implementation from the upstream SUPER project rather than the ROS1-only standalone integration path.

During initial integration:

- pin a specific upstream SUPER revision;
- import only `rog_map` and its minimum required utility closure;
- preserve upstream copyright/license notices;
- document provenance in an `UPSTREAM.md` or equivalent file;
- keep local algorithm modifications at zero unless required for correctness/build compatibility.

Do not import the entire SUPER repository as runtime architecture.

## Integration architecture

Do not use the upstream automatic ROS callback path as the product integration contract.
The product mapping boundary is the atomic `LidarMappingObservation` on
`/lio/mapping_observation`; `navigation_mapping` owns PointCloud2 decoding and
ROG-Map updates.

Use the core/manual API:

```text
/lio/mapping_observation
        |
        v
navigation_runtime
        |
        | validate timestamp/frame
        | decode the embedded PointCloud2 points
        | apply the observation sensor pose
        v
ROGMap::updateMap(cloud, pose)
```

Set the upstream automatic ROS callback path off and let `uav-navigation` own timestamp/frame correctness.

## Input contract

Primary mapping input:

```text
/lio/mapping_observation
header.frame_id: lio_odom
points.header.frame_id: livox_frame
```

The observation carries the exact ray origin and generation:

```text
sensor_pose: ^lio_odom T_livox at header.stamp
public_frame_generation: active LioPublicFrameGeneration
```

`/lio/registered_points` remains estimator-owned registration output and is
not a navigation-world-model input.

Do not use latest odometry as the ray origin. Do not use `base_link` as a substitute for the LiDAR origin.

Do not use `/lio/local_map` as the navigation-map source because it is the estimator registration support map and does not represent free-space observation semantics.

## Initial runtime configuration

Start conservatively:

```text
occupancy          ON
raycasting         ON
map sliding        ON
inflation          ON
ESDF               OFF
frontier           OFF
unknown publishing OFF by default
heavy visualization OFF during acceptance
```

Initial tuning target, to be benchmarked rather than treated as final:

```text
resolution:          approximately 0.20 m
local volume:        approximately 30 x 30 x 12 m
raycast range:       approximately 15-20 m
```

Later compare 0.20 / 0.15 / 0.10 m only after correctness is established.

## Runtime architecture

Initially run mapper and FAST-LIO as separate processes.

Benefits:

- crash isolation;
- independent profiling;
- clear CPU/memory attribution;
- mapper restart without restarting localization;
- no premature composition/zero-copy optimization.

The first implementation should use a simple synchronous update path. Do not add worker queues, ring buffers, schedulers, or concurrency unless profiling proves that synchronous processing cannot meet the update budget.

If asynchronous processing becomes necessary, prefer one map worker with a latest-frame slot instead of an unbounded or deep backlog.

## Mapping acceptance tests

### Synthetic ray test

Sensor origin at `(0, 0, 0)` and a hit near `(5, 0, 0)` must produce:

- observed free cells before the endpoint;
- occupied endpoint;
- unknown cells outside observed space.

### Static wall test

In Gazebo with a static wall:

- occupied voxels align with the wall;
- free space exists between sensor and wall;
- unseen space behind the wall remains unknown;
- yawing the UAV does not rotate the world map.

### Translation/sliding test

- obstacle world positions remain fixed while the UAV moves;
- local map slides correctly;
- memory remains bounded;
- stale map regions are cleared/reused correctly.

### Inflation test

- inflated obstacles must eventually respect an authoritative vehicle
  collision envelope plus static safety margin; the current repository does
  not yet provide that physical source of truth, so this acceptance item is
  blocked pending the physical-clearance contract;
- collision queries fail inside inflated space and pass in valid free space.

### Dataset test

Replay the existing AIST pipeline and measure:

- map update rate;
- p50/p95/p99 update latency;
- CPU;
- RAM;
- voxel count;
- backlog/drop count;
- map freshness.

### Planning compatibility smoke test

The product-owned `Planner` is a synchronous library boundary over
`WorldModel`. Its minimum path is A* seed path, conservative corridor
validation, and a time-parameterized quintic trajectory with sampled dynamic
limit validation. It is not a separate ROS runtime subsystem and never
consumes visualization PointCloud2 topics.

## Performance target

At 10 Hz LiDAR input, maintain enough margin that map integration does not approach the 100 ms scan period. A practical initial target is:

```text
map update p95 < 50 ms
```

This is a bring-up budget, not a final performance guarantee.

## P1 Definition of Done

- ROG-Map operates independently of FAST-LIO's registration map;
- free/occupied/unknown semantics are correct;
- map is invariant to UAV attitude changes;
- sliding is correct and memory bounded;
- inflation works;
- dataset/SITL performance is measured;
- no estimator change was required merely to make mapping run;
- a simple collision-free planning smoke test succeeds.

---

# 5. Phase P2 — Planning reference baseline

## Objective

Prove a coherent planner pipeline from the validated ROG-Map world model to a time-parameterized collision-free trajectory.

Do not treat A*, corridor generation, and trajectory optimization as separate import projects if they are already coupled inside the reference planner.

## Reference strategy

Use the minimum required planning closure from SUPER as a reference subsystem.

The initial goal is not to recreate SUPER piece by piece. Preserve its internal planning flow sufficiently to get a valid working baseline.

Potential internal components may include:

```text
path search
    ->
corridor generation
    ->
trajectory optimization
    ->
replanning logic
```

These components may remain directly coupled internally during the reference stage.

## Explicitly excluded from ownership transfer

Do not adopt SUPER as the application architecture.

Do not replace project-owned equivalents with:

- SUPER mission planner;
- SUPER/MARSIM simulation architecture;
- MAVROS-based vehicle integration;
- SUPER odometry ownership;
- SUPER launch architecture;
- SUPER controller;
- SUPER safety source of truth.

Import only the minimum algorithmic planning closure required for the reference planner.

## Product-facing planner contract

Inputs:

```text
current vehicle state
goal / local mission target
navigation world model
```

Output:

```text
time-parameterized trajectory
```

Acceptance belongs to `uav-navigation`, regardless of internal reference algorithm choices.

## Acceptance

- feasible goal produces a collision-free trajectory;
- trajectory stays within configured map-valid region;
- configured velocity/acceleration/other dynamic constraints are respected;
- planning failure returns an explicit invalid/failure result;
- replanning does not publish stale trajectories;
- trajectory timestamps are monotonic and usable by the execution layer;
- planning latency p50/p95/p99 is recorded;
- planner remains stable under repeated replanning in SITL.

## P2 Definition of Done

A validated ROG-Map and current vehicle state can produce a valid time-parameterized trajectory through one project-owned planning interface.

The repository currently contains only the ROG-Map/SUPER vendor subset, not
SUPER's CIRI or trajectory-optimization sources. The minimum project-owned
closure therefore uses the existing A* implementation, a conservative
WorldModel corridor check, and quintic time parameterization. SUPER's ROS
wrapper, FSM, map wrapper, and controller remain excluded.

---

# 6. Phase P3 — Project-owned trajectory execution and PX4 integration

## Objective

Convert a validated planner trajectory into a deterministic PX4 execution path owned entirely by this repository.

This boundary must remain independent of SUPER internals so that future planner refactoring does not require rewriting the PX4 side.

## Architecture

```text
planning trajectory
       |
       v
trajectory adapter / validator
       |
       v
trajectory sampler / tracker
       |
       v
PX4 setpoint interface
       |
       v
PX4
```

## Responsibilities

- validate trajectory freshness and finite values;
- reject trajectories from an invalid planner generation/epoch;
- sample/reference the trajectory at controlled timestamps;
- generate the selected PX4 setpoint representation;
- detect tracking-error growth;
- stop using stale trajectories;
- expose execution diagnostics.

Do not embed ROG-Map or SUPER planner internals into the PX4 interface.

## Acceptance

- hover trajectory executes without divergence;
- straight-line trajectory executes correctly;
- yaw change executes correctly;
- multi-segment trajectory executes correctly;
- trajectory replacement/replanning is continuous;
- stale/invalid trajectory fails closed;
- tracking error remains within defined limits;
- SITL mission completes deterministically.

---

# 7. Phase P4 — Navigation safety and end-to-end autonomous validation

## Objective

Establish project-owned runtime safety and prove the complete navigation loop.

Planner safety logic is useful but is not the sole runtime safety authority.

## Minimum safety gates

Execution is allowed only when the required conditions are valid, including:

```text
localization valid
map fresh
planner result fresh
trajectory valid
PX4 mode/status valid
tracking error bounded
```

Required failure handling includes at minimum:

- localization LOST/invalid;
- map stale;
- planner failure;
- trajectory stale;
- trajectory collision invalidated by map update;
- excessive tracking error;
- PX4 control-mode loss.

The exact safe response may be hover, stop, controlled return, or another vehicle-specific policy, but it must be explicit and testable.

## End-to-end SITL acceptance

Create deterministic obstacle scenarios that validate:

1. localization;
2. map construction;
3. path/trajectory generation;
4. replanning;
5. PX4 execution;
6. safety behavior;
7. landing/disarm.

Initial mandatory speed target:

```text
stable autonomous navigation >= 5 m/s
```

Later stress target:

```text
up to approximately 10 m/s
```

The 10 m/s target is a design/stress ceiling, not a reason to optimize every subsystem prematurely.

---

# 8. Real-hardware validation stage

Real hardware validation is scheduled after sufficient simulation/world-model/planner progress because the full hardware set is not currently available.

When hardware becomes available, validate the already-frozen subsystem contracts rather than redesigning them for the hardware test.

Required hardware progression should include:

1. physical Mid-360 + IMU timestamp/frame validation;
2. real FAST-LIO stationary / translation / yaw / 6DoF tests;
3. physical PX4 external odometry fusion;
4. ROG-Map static obstacle validation;
5. low-speed autonomous navigation;
6. progressively higher-speed validation.

Any hardware-specific estimator or mapping modification must be justified by a reproducible hardware defect, not by preference.

---

# 9. Phase P5 — Profile-driven consolidation and final refactor

## Objective

Only after the complete system works, perform the intended large-scale cleanup and optimization.

This phase may restructure or replace upstream-derived internals, including:

- IKFoM integration;
- ikd-tree integration;
- ROG-Map internals;
- SUPER planning internals;
- duplicated utility layers;
- ROS serialization boundaries;
- point-cloud conversion/copy paths;
- memory allocation and ownership;
- thread/callback structure.

## Required evidence before refactor

Collect end-to-end profiling for:

```text
CPU per subsystem
RAM / peak memory
allocation frequency
point-cloud copies
DDS serialization cost
map update p50/p95/p99
planning p50/p95/p99
control-loop latency
thread contention
queue depth/backlog
cache/memory hot paths where available
```

Do not replace a reference algorithm only because a custom implementation appears cleaner.

A rewrite must demonstrate at least one meaningful advantage:

- lower measured latency;
- lower memory;
- fewer copies/allocations;
- simpler dependency closure;
- better determinism;
- better safety/correctness;
- materially smaller maintainable codebase without capability loss.

## Desired final data path

The final optimized implementation may remove intermediate representation changes, for example:

```text
LiDAR scan
   -> preprocessing once
   -> estimator correction
   -> corrected registered scan
   -> navigation map
   -> planner
```

instead of repeatedly converting through project vectors, PCL, ROS `PointCloud2`, DDS, and back to PCL when direct in-process ownership is justified by profiling.

However, this optimization belongs here, not during initial ROG-Map bring-up.

## Final cleanup goals

- remove unused vendor wrappers and demo code;
- remove duplicated ROS1 compatibility if not required;
- consolidate configuration ownership;
- minimize public APIs;
- preserve project-owned tests for every replaced component;
- keep provenance of algorithms/reference implementations;
- update repository layout only to match the actual final structure.

---

# 10. Phase summary and gates

| Phase | Primary result | Gate before next phase |
|---|---|---|
| P0 | Stable estimation + PX4 odometry baseline | AIST PASS + 3 consecutive SITL PASS |
| P1 | ROG-Map local navigation world model | occupancy/sliding/inflation correct + bounded runtime |
| P2 | Reference planning pipeline | collision-free dynamically valid trajectory |
| P3 | Project-owned PX4 trajectory execution | deterministic tracking + replanning in SITL |
| P4 | Navigation safety + autonomous loop | complete obstacle scenario, >=5 m/s stable target |
| Hardware | Physical validation | sensor/PX4/map/planner behavior confirmed progressively |
| P5 | Consolidated optimized product code | measurable benefit with no functional regression |

---

# 11. Immediate next actions

Current execution order:

```text
1. Close P0 odometry baseline with the final verification commit.
2. Freeze estimation/PX4 baseline.
3. Start P1 ROG-Map integration using the manual core API.
4. Validate occupancy/sliding/inflation independently.
5. Add only a planner smoke test at P1.
6. Bring in the minimum SUPER planning closure as one P2 reference subsystem.
7. Build project-owned trajectory execution to PX4.
8. Add project-owned runtime safety and end-to-end SITL scenarios.
9. Perform real-hardware validation when hardware is available.
10. Profile the complete stack, then perform the planned full refactor.
```

The guiding rule throughout the roadmap is:

> Get a correct, measurable, end-to-end reference system working first. Optimize and consolidate only after the real bottlenecks and unnecessary dependencies are visible.
