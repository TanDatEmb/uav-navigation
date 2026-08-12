# Navigation layers

The product boundary is the estimator, its PX4 integration, and (P1) an
independent navigation world model:

```text
LiDAR + IMU -> FastLioPipeline -> corrected/propagated odometry
                              -> registration map (internal)
                              -> PX4 health-gated external odometry (simulation)
                              -> gated LidarMappingObservation (P1)
                                     |
                                     v
                         navigation_mapping (separate process)
                                     |
                                     v
                    ROG-Map navigation world model (occupancy/free/unknown)
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
map, and (later phases) planner queries. Implemented in P1 by a vendored
ROG-Map instance (`rog_map_vendor`, pinned to `hku-mars/SUPER` commit
`2ad3419c127a617c6d7df6925e81a14175a9c096`; see
`src/navigation_mapping/rog_map_vendor/UPSTREAM.md`). It consumes a
*mapping-grade* observation — deskewed, common-filtered, but **not**
estimator-voxelized — published only from FAST-LIO's valid corrected tracking
state.

## The `LidarMappingObservation` contract (P1)

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

## Generation handling (P1)

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

FAST-LIO and `navigation_mapping` run as separate ROS 2 processes (see
`src/navigation_bringup/launch/{fast_lio,navigation_mapping}.launch.py`). The
dependency direction is one-way:

```text
navigation_mapping (+ rog_map_vendor)
        depends on
navigation_interfaces (LidarMappingObservation contract)
        depends on (published by)
fast_lio_ros
```

`fast_lio_core`/`fast_lio_ros` have no dependency on `rog_map_vendor` or
`navigation_mapping`. `navigation_mapping` has no dependency on
`fast_lio_core`.

The original estimator-only package dependency direction is
`ikfom_vendor/ikd_tree_vendor -> fast_lio_core -> fast_lio_ros -> navigation_bringup`,
with `livox_ros_driver2` supplying the sensor package and custom message,
`fast_lio_tools` consuming the core, and
`navigation_interfaces -> px4_odometry_bridge` defining the PX4 bridge
contract. ROS, Gazebo, bags, PX4 types, and vendor drivers must not enter
`fast_lio_core`.
