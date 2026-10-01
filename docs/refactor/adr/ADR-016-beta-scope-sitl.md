# ADR-016: First beta is SITL-only

**Status:** accepted.

## Decision

- The beta target is PX4 SITL + Gazebo on ROS 2 Jazzy. The
  `deployment_profile=hardware` path stays blocked exactly as today (HG-011).
- `use_sim_time=true` is mandatory in every product node for the beta. All
  product timers use the ROS clock (no `create_wall_timer`). Steady time is used
  only for receive stamps and latency measurement.
- `gz_visibility_bridge` remains the visibility source, but `nav_core` consumes
  it through a `VisibilitySource` interface so that the hardware certificate
  (HG-011) can later be added without touching the planner.
- No real-time guarantees are claimed. Latency distributions are measured on
  the baseline machine recorded in the P0 manifest, and all regressions are
  judged against that machine.
- Qualification for beta = the P0 baseline matrix (WP-P0.2) plus
  `config/runtime/planning_stability_qualification.yaml`, judged by the P2
  judge.

## Consequences

Hardware bring-up work (Mid-360 visibility certificate, real-time executor,
target-CPU budgets) is out of scope and must not be mixed into refactor WPs.
