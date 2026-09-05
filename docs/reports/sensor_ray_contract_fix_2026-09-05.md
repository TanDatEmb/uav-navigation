# Sensor ray contract fix — 2026-09-05

## Scope

The canonical planner profile keeps the existing
`rog_map.raycasting.ray_range` key. Its lower bound is the minimum distance
for an accepted first return or explicit no-return endpoint and the starting
distance of the corresponding sensor-origin miss ray. It is sensor evidence
configuration; it is not vehicle clearance, body support, planner inflation,
or an instruction to clear a neighborhood around the vehicle.

The bundled Gazebo MID-360 model declares a sensor range of `0.1..40.0 m`
(`src/uav_simulation/models/lidar_mid360/model.sdf:54`). The profile therefore
uses `ray_range: [0.1, 100]`: the upper bound remains unchanged in this small
contract fix; the bundled SDF cannot supply an accepted endpoint beyond 40 m,
so this does not claim a 100 m sensor horizon. The estimator's independent registered-cloud preprocessing
minimum remains `0.5 m` (`config/runtime/sim.yaml:43-46`). This report makes no
claim about hardware or dataset sensor calibration.

## Dataflow and ownership

`ProbMap` applies the lower bound to returned endpoints and explicit no-return
endpoints (`src/mapping/rog_map_vendor/src/rog_map/prob_map.cpp:901-904,
953-958`), then starts each miss traversal from that distance
(`.../prob_map.cpp:1019-1035`). Explicit no-return endpoints contribute misses
only and never occupancy (`.../prob_map.cpp:937-943`). Occupied evidence keeps
its existing precedence. Only voxels traversed by an accepted measured beam
can become free; unobserved nearby space and the vehicle body remain unknown.

The runtime runner copies this same planner source into each session through
`tools/runtime/runner.py:_mapping_params` (`:1088-1173`), including dataset
replay (`:1479-1487`). The shared profile thus defines one evidence contract;
it does not convert the bundled SDF value into a dataset or hardware claim.

## Regression intent

The ROG-Map smoke regressions cover a `0.6 m` hit (past the separate estimator
`0.5 m` preprocessing minimum) and a direct component-level `0.4 m` no-return
endpoint, on-beam free versus explicit off-beam `UNKNOWN`, and rejection of
`0.09 m` inputs. The
existing sensor-origin lever-arm and occupied-not-body-cleared tests remain
in `test_planning_grid_export.cpp`. The runtime contract test asserts the
sensor minimum and no longer compares it with the planner safety radius.

This is a source/config contract correction, not a gate tuning result. The
corresponding sequential ledger entry records ownership, scope, safety impact,
evidence, removal condition, and verification command.
