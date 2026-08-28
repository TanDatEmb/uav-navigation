# Navigation generalization scenario matrix

This map is an evaluation contract, not proof of planner acceptance. It keeps
one deterministic world and one ordered mission so failures can be attributed
to a route phase without changing mapping, planner, PX4, or safety parameters.

| Scenario | Route phase | Geometry | Required evidence |
|---|---|---|---|
| Collinear velocity | waypoint 0 to 2 | two straight +X legs | waypoint 1 accepted in order; no terminal stop, reverse progress, or 90-degree yaw excursion |
| High-speed sudden avoidance | waypoint 1 to 2 | 27 m open run, small post, pillar, then transverse wall | collision-free completion; speed recovery after each bypass; bounded planner latency and command gaps |
| +Y axis generalization | waypoint 2 to 3 | 45 m +Y leg with transverse and longitudinal walls | forward route progress and yaw aligned with +Y outside avoidance transitions |
| Local narrow gap | waypoint 3 to 4 | 4 m gap after a long +Y approach | strict known-free clearance evidence; no gate relaxation or out-of-map acceptance |
| -X and small objects | waypoint 4 to 5 | 90 m -X leg with 0.25 m cylinder and 0.7 m box | both small objects present in truth; no axis-dependent mode exit; speed recovers after avoidance |
| Lane change | waypoint 5 to 7 | down one lane and across behind a divider | checkpoints accepted monotonically; continuous command availability through both turns |
| Hairpin / U-turn | waypoint 7 to 9 | two 90-degree transitions around a central island | no backward route fold; yaw rotates at bounded rate and does not command a spurious 180-degree reversal before the turn |
| -Y altitude recovery | waypoint 9 to 10 | 30 m terminal -Y leg | altitude returns to mission height after avoidance; terminal position and speed gates both pass |

Acceptance requires repeated runs at a speed already validated by the speed
ladder. A single completion, component test, or dataset shadow-planning pass is
insufficient for flight acceptance.
