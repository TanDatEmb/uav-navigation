# Pre-main test matrix

| Class | Included coverage | Result / treatment |
|---|---|---|
| PRODUCT_REQUIRED | All built first-party packages affecting estimation, mapping, planning, contracts, mission, execution, runtime, PX4 adapter, bringup and simulation | Release build and package tests required; final gate runs them |
| THIRD_PARTY_REQUIRED | Pinned ikd_tree_vendor, ikfom_vendor, livox_ros_driver2 smoke tests; `px4_ros2_cpp` unit fixture | Required smoke/unit tests; final gate runs them. One named flaky upstream unit case is isolated and reported, not a product target |
| ENVIRONMENT_GATED | `px4_ros2_cpp` integration tests requiring live FMU | `NOT_RUN`; no live FMU assumed by the gate |
| OPTIONAL_EXAMPLE | Pinned upstream `example_*_cpp` packages and optional upstream lint/format maintenance | Excluded from product release gate; no upstream patch made |

The package wrapper previously omitted first-party `navigation_mission` and `uav_description`; the release package list now includes them. No target remains silently unclassified.
