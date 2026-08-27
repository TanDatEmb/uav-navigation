# ADR-012: Product-owned tree, names, and shared boundary utilities

Status: accepted for migration. The repository has one product vocabulary and
one active implementation path. Upstream names are allowed only inside an
explicit third-party boundary and in provenance records.

## Decision

The product tree is organized by ownership, not by the historical repository
that contributed an algorithm:

```text
src/
  common/navigation_common/       time and ENU/NED, FLU/FRD conversions
  contracts/navigation_contracts/ ROS messages, services, and boundary checks
  contracts/navigation_mission/  validated mission schema and typed policy
  estimation/                     estimator core and ROS adapter
  mapping/                        world model and mapping owner
  planning/                       planner product boundary and backend adapter
  runtime/navigation_runtime/     lifecycle and composition wiring
  px4/                            PX4 ingress and External Mode adapters
  navigation_bringup/             launch and operator-facing configuration
  uav_description/                sensor-frame source of truth
  uav_simulation/                 simulator assets and bridges
  external/                       pinned external packages
```

`navigation_contracts` is the only product ROS contract package. The old
`navigation_interfaces` package and unused legacy command package are removed;
there is no compatibility package kept in parallel. `navigation_common` is
the only shared home for ROS timestamp conversion and basis conversion helpers.
`navigation_mission` is the only C++ owner of the mission YAML schema and its
planning limits/UNKNOWN policy; Python readers under `tools/runtime/` are
orchestration/report tooling and cannot authorize planning or flight.

Product classes, nodes, parameters, topics, config files, and tests must not
use upstream project names or release/task labels. External ABI names such as
PX4's `/fmu/...` topics and upstream package namespaces remain unchanged at the
adapter boundary because changing them would change an external contract.

## Current cutover

The contract package, coordinate helper package, runtime node/source/config
names, command topic, and runtime parameter namespace have been cut over in
one path. `NavigationCommand` is the only planner-to-PX4 command type. The
runtime library is `navigation_runtime_core` and its executable is
`navigation_runtime_node`.

The planner and map implementations are not yet fully product-owned. Their
remaining upstream implementation names are intentionally confined to the
current backend packages until the mapping/planning extraction is complete;
they are not valid names for new product APIs.

The first mapping extraction is now implemented: `navigation_mapping` owns the
bounded observation worker, exact lifecycle accounting, mutable map
integration, and immutable snapshot construction. The runtime composes the
actor and publishes its snapshots into the planning lifecycle. The installed
ABI boundary is product-owned; backend grid and map types remain private to
the mapping implementation and tests.

The planner backend package is now named `navigation_planning_backend`, with a
single product-facing include `navigation_planning_backend/planner.hpp` and a
`planner` configuration root. Upstream implementation namespaces remain
isolated inside that backend boundary and are tracked by its parity manifest.

The validated C++ mission loader is now named `navigation_mission` and is
consumed by both runtime and PX4 External Mode. No second C++ mission parser
is retained.

The first pure product planning boundary is now implemented as
`navigation_planning`: it contains C++20 planning request, outcome, budget,
kinematic-state and immutable candidate contracts with no ROS or backend
include. `navigation_execution` now contains the corresponding
`CommittedBundleStore` and `CommandSampler`; both have focused tests. The
runtime now uses those types as its command authority. The remaining backend
adapter is limited to solving and exporting an immutable candidate.

The planner's former `ros_interface` name was removed. The actual abstraction
is now `planner_runtime_context`, which owns planner clock, logging and
visualization callbacks. This avoids confusing an internal planner context
with the product ROS contract package or the PX4 ROS adapter.

## Migration sequence

1. Move mutable map ownership and snapshot export behind `navigation_mapping`.
   This ownership move and its installed product boundary are complete.
2. Add observed-free evidence and a product-owned world-health contract.
3. Introduce `navigation_planning` request/outcome contracts and keep the
   current planner only as a backend adapter that cannot commit execution.
4. Keep `navigation_execution` as the only committed-bundle and
   command-sampling owner; remove remaining backend/vendor types from runtime
   diagnostics and tests after repeated evidence.
5. Normalize one typed kinematic state across estimator, runtime, and PX4;
   then replace the remaining backend-only implementation details behind the
   planner adapter after repeated evidence.

Every step has one implementation path. Old names are deleted at cutover;
aliases are not retained as a second product version.
