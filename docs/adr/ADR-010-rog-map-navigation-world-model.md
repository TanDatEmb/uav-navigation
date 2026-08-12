# ADR-010: ROG-Map as an independent navigation world model (P1)

**Status:** accepted.

P1 introduces `navigation_mapping` (product code) and `rog_map_vendor`
(pinned upstream ROG-Map, `hku-mars/SUPER` commit
`2ad3419c127a617c6d7df6925e81a14175a9c096`) as an independent local
navigation world model. It consumes `navigation_interfaces/msg/LidarMappingObservation`,
a mapping-grade (deskewed, common-filtered, not estimator-voxelized)
observation published by `fast_lio_ros` only from FAST-LIO's valid corrected
tracking state, gated by the existing corrected-odometry usability contract
and `LioPublicFrameGeneration`.

FAST-LIO and `navigation_mapping` run as separate ROS 2 processes. FAST-LIO
has no dependency on `rog_map_vendor` or `navigation_mapping`; the dependency
direction is `navigation_mapping (+ rog_map_vendor) -> navigation_interfaces
-> fast_lio_ros`. `RegistrationMap` (ADR-008) remains FAST-LIO's own
scan-to-map nearest-neighbor structure and is never used as world-model
input; `/lio/registered_points` (the estimator's coarse voxelized cloud) is
explicitly excluded from the mapping observation contract.

See `docs/architecture/navigation_layers.md` for the full contract and
`src/navigation_mapping/rog_map_vendor/UPSTREAM.md` for upstream provenance
and local patches (notably a lifecycle fix required for repeated map reset on
public-frame-generation discontinuities).

Out of scope for P1: SUPER planner, CIRI, MINCO, frontier exploration, ESDF,
PX4 planning integration, and mission logic.
