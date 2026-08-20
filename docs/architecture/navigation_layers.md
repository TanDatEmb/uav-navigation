# Navigation layers

The product boundary is the estimator, its PX4 integration, and an independent
navigation world model:

```text
LiDAR + IMU -> FastLioPipeline -> corrected/propagated odometry
                              -> registration map (internal)
                              -> PX4 health-gated external odometry (simulation)
                              -> gated LidarMappingObservation
                                     |
                                     v
                         navigation_mapping (separate process)
                                     |
                                     v
                    ROG-Map -> WorldModel -> A* reference baseline
```

## RegistrationMap vs Navigation World Model

These are two separate structures with separate owners. Confusing them is an
architectural error (see ADR-008).

### `RegistrationMap` (owned by `fast_lio_core`)

Purpose: nearest-neighbor search and plane support for scan-to-map
registration only. It is **not** used for occupancy, free-space reasoning,
collision checking, or planner queries. Its published cloud
(`/lio/registered_points`) is the *estimator's own coarse voxelized* scan —
intentionally coarse for estimator performance — and must never be treated as
a world-model input.

### Navigation World Model (owned by `navigation_mapping`)

Purpose: occupancy, free/unknown reasoning, obstacle inflation, local sliding
map, and planner queries. Implemented with a vendored
ROG-Map instance (`rog_map_vendor`, pinned to `hku-mars/SUPER` commit
`2ad3419c127a617c6d7df6925e81a14175a9c096`; see
`src/mapping/rog_map_vendor/UPSTREAM.md`). It consumes a
*mapping-grade* observation — deskewed, common-filtered, but **not**
estimator-voxelized — published only from FAST-LIO's valid corrected tracking
state.

## The `LidarMappingObservation` contract

`navigation_interfaces/msg/LidarMappingObservation` is the atomic geometric
observation FAST-LIO publishes for the navigation world model. It is a
geometric observation, not a ROG-Map command; it carries no occupancy,
resolution, or planner concept.

```text
header.stamp            == the deskew reference epoch of the scan
header.frame_id          == "lio_odom"
sensor_pose               == ^lio_odom T_livox at exactly header.stamp,
                             derived from the same corrected estimator state
points                     == deskewed, common-filtered XYZ, in "livox_frame",
                             points.header.stamp == header.stamp
public_frame_generation   == the active LioPublicFrameGeneration value
```

FAST-LIO's minimal extraction point is in `PointCloudPreprocessor`
(`fast_lio_core/preprocessing/point_cloud_preprocessor.{hpp,cpp}`): the common
range/finite filter and the estimator-only voxel filter are two ordered
stages in the same pipeline call; the mapping candidate is a copy of the
points after the first stage and before the second
(`retain_mapping_candidate`, opt-in, default off). The corrected sensor pose
is composed from the same corrected `state_` used for
`registered_points_odom_m` (`^lio_odom T_imu * ^imu T_livox`); it is never
derived from propagated state or from TF. Publication
(`fast_lio_ros::RosMappingObservationPublisher`) is gated on exactly the same
usability contract as corrected odometry (`kTracking` + corrected +
navigation-valid) plus a valid `LioPublicFrameGeneration` snapshot.

## Generation handling

`navigation_mapping` does not invent a second frame-generation mechanism; it
only interprets the existing `LioPublicFrameGeneration` value carried on each
observation (`navigation_mapping::GenerationTracker`):

```text
observation.generation == current tracked generation -> normal update
observation.generation >  current tracked generation -> reset ROG completely,
                                                          adopt new generation
observation.generation <  current tracked generation -> reject as stale
```

Only a true public-frame discontinuity resets the map; internal estimator
restarts or corrected/propagated handoffs that preserve the public `lio_odom`
frame do not change the generation and therefore do not reset the map.

## Process and dependency direction

FAST-LIO and `navigation_runtime` run as separate ROS 2 processes (see
`src/navigation_bringup/launch/{fast_lio,navigation_runtime}.launch.py`). The
runtime owns one mapping pipeline and calls the planner synchronously in the
same serialized execution policy. The dependency direction is one-way:

```text
navigation_runtime -> navigation_mapping (+ rog_map_vendor)
        depends on
navigation_interfaces (LidarMappingObservation contract)
        depends on (published by)
fast_lio_ros
```

`fast_lio_core`/`fast_lio_ros` have no dependency on `rog_map_vendor` or
`navigation_mapping`. `navigation_mapping` has no dependency on
`fast_lio_core`.

The current `navigation_planning` package is a library boundary over the
in-process `WorldModel` facade. It is not a separate ROS voxel-query process;
the planner must not recreate the map through per-cell services or a duplicate
DDS occupancy representation.

### Planning policy and physical-clearance status

The WorldModel has two distinct query layers. `Probability` plus
`UnknownBlocked` is useful for fine-map semantics and regression tests, but it
is not vehicle-clearance collision planning. `Inflated` plus `UnknownBlocked`
is the conservative path/corridor policy. `WorldModel::clearanceRadius()` is
the separate physical contract consumed by the project-owned `Planner`.

The real-flight runtime still has no authoritative vehicle collision geometry:
the local x500 model only includes the external `x500` model and the local
`base_link` description is frame-only. Therefore the normal mapping profile
leaves `navigation.collision.vehicle_radius_m` and
`navigation.collision.safety_margin_m` unset; planning fails closed. The
simulation profile is explicitly validation-only and is derived from the
external PX4 model source
`Tools/simulation/gz/models/x500_base/model.sdf`: rotor centers are at
`+/-0.174 m` and each rotor collision box is `0.27923 m` long, yielding a
`0.32 m` conservative horizontal radius plus a `0.05 m` margin. This profile
does not authorize a real aircraft envelope. The simulation runner also
enables the narrowly scoped `allow_unknown_start` exception because the
simulated LiDAR is mounted 0.28 m above `base_link`: a virtual `KnownFree`
overlay follows the trusted current pose with radius `0.37 m`, equal to the
simulation collision envelope. It converts only `Unknown` cells intersecting
that footprint and never masks `Occupied` evidence. Every cell outside it still
requires `KnownFree`; the real/default profile keeps this exception disabled.
Synthetic planner tests use separate explicit values only to validate contract
and algorithmic closure.

`UnknownTraversable` is a reference/exploration policy only; it is not
flight-safe execution by itself. The current External Mode path executes only
the planner's `Inflated` + `UnknownBlocked` trajectory contract and fails
closed on missing/invalid/stale trajectories. An unknown-space exploration
trajectory would require an independently safe known-free backup/stopping
trajectory. The current runtime now has the first bounded form of that
fallback: when the nominal plan fails, it asks the planner for a braking
trajectory whose every sampled cell is `Inflated` + `KnownFree` and whose final
velocity and acceleration are zero. External Mode marks this as `SAFETY`; it
does not advance the mission waypoint, and retries the same correlated goal.
If that stop cannot be proven, the mode fails closed. This is a stop/retry
contract, not permission to fly through unknown space. Nominal unknown-space
planning remains disabled until a commitment horizon and an independently
validated safe fallback are implemented. Future CIRI should consume raw
occupied geometry and apply the same physical-clearance contract itself, rather
than consuming already-inflated occupied points and double-inflating them.

Runtime replanning is bounded by the active goal identity and WorldModel
generation/revision. A timer tick with no change is skipped and counted; a new
goal or map revision invalidates the cache and runs A* plus trajectory
verification again. This prevents an unconditional planner loop from being
mistaken for a safety mechanism while retaining fail-closed behavior on a
changed map.

The rolling trajectory contract carries `trajectory_id`, `parent_trajectory_id`
and `valid_from`. A replacement is generated from the old sampled trajectory
at a future switch state `(p,v,a)` and PX4 keeps the old setpoint active until
the replacement's `valid_from`. Runtime verification and PX4 consume the same
sampled PVA contract; the planner's bounded polynomial remains the source of
the smooth samples. Intermediate mission waypoints are declared
`pass_through` and carry the next target so the planner can preserve a bounded
tangent; terminal/inspection points use `stop`.

Before each plan, a visibility governor computes the known-free inflated
horizon `d_free` in the motion direction and caps speed using
`v²/(2a_decel) + v t_latency + d_margin <= d_free`. This is a conservative
speed envelope layered above A* and verification, not a replacement for
collision checking.

Mission completion is a notification from External Mode to the supervisor. The
mode executor reports success and stops owning mission progress; it does not
issue LAND, RTL, disarm, or an assumed Loiter handover. The supervisor chooses a
PX4 mode supported by the active estimator profile and owns the landing
lifecycle.

ROG's inflated `KnownFree` is the existing CounterMap threshold result. It
means the coarse cell is not occupied and has fewer unknown fine subcells than
ROG's configured threshold; it does not, by itself, assert that every fine
probability subcell is known free.

The original estimator-only package dependency direction is
`ikfom_vendor/ikd_tree_vendor -> fast_lio_core -> fast_lio_ros -> navigation_bringup`,
with `livox_ros_driver2` supplying the sensor package and custom message,
`fast_lio_tools` consuming the core, and
`navigation_interfaces -> px4_odometry_bridge` defining the PX4 bridge
contract. ROS, Gazebo, bags, PX4 types, and vendor drivers must not enter
`fast_lio_core`.
