# SUPER architecture comparison (pinned 2026-09-05)

This bounded comparison is a historical source snapshot. It uses upstream SUPER commit
`2ad3419c127a617c6d7df6925e81a14175a9c096`, checked out at
`/tmp/uav-super-2ad3419c`, and repository snapshot
`1835f3570e12985aea500d71be384fb2ba66d4e6` before the FINISH patch. That
repository hash is not the current HEAD; the comparison records the behavior
seen at that earlier snapshot. The
upstream source is available at
`https://github.com/hku-mars/SUPER/tree/2ad3419c127a617c6d7df6925e81a14175a9c096`.
The backend snapshot compared there is under
`src/planning/navigation_planning_backend/`. It is source evidence only; it
does not claim algorithmic equivalence, performance, or flight qualification.

## Corrected MAIN/BACKUP result semantics

Upstream `ReplanOnce()` has distinct branches. `NO_NEED` does **not** call
`cmd_traj_info_.setTrajectory()`; it updates `last_exp_traj_info_`, clears
`robot_on_backup_traj_`/`gi_.new_goal`, and returns success while retaining the
existing command ([upstream `super_planner.cpp:256-273`](https://github.com/hku-mars/SUPER/blob/2ad3419c127a617c6d7df6925e81a14175a9c096/super_planner/src/super_core/super_planner.cpp#L256-L273)).
`FINISH` is the branch that replaces the command with EXP-only via
`setTrajectory(exp_traj_info)` ([upstream `:274-290`](https://github.com/hku-mars/SUPER/blob/2ad3419c127a617c6d7df6925e81a14175a9c096/super_planner/src/super_core/super_planner.cpp#L274-L290)).

The successor path in that historical snapshot first applied one retention predicate to both
`NO_NEED` and `FINISH` when an old backup exists, returning `NO_NEED` without
staging (`src/planning/navigation_planning_backend/src/planner_core/planner.cpp:1375-1397`).
Only when retention is false does it build/stage separate `NO_NEED` and
`FINISH` candidates (`:1439-1497`). Consequently, a current-world `FINISH`
result can keep an old BACKUP suffix longer than upstream. Any change should
first define “complete candidate” permission: `CmdTraj::buildCandidate()` must
pass finite/rest checks for main-only candidates (`include/data_structure/cmd_traj.h:221-265`),
then `authorizeAndStage()` must pass identity, route, world, dynamic, and
execution certificates. Main-only certificates are explicitly tightened to
`kRequireKnownFree` (`include/planner_core/trajectory_world_validator.hpp:701-711`).
FINISH may proceed only through that existing authorization boundary; the
direct old-BACKUP successor witness remains pending.

## First-frame unknown clearing versus current support

Pinned upstream ROG-Map clears a spherical neighborhood around the robot only
on the first process-wide frame. It iterates a cube at resolution spacing,
keeps points with `norm() <= cfg_.raycast_range_min`, and calls
`missPointUpdate()` at `pos + p` ([upstream `prob_map.cpp:358-374`](https://github.com/hku-mars/SUPER/blob/2ad3419c127a617c6d7df6925e81a14175a9c096/rog_map/src/rog_map/prob_map.cpp#L358-L374)).
This is a radius-based unknown prior, not a measured rectangular vehicle hull.

At that historical snapshot, `ProbMap::updateProbMap()` ended after
sensor/raycast diagnostics and did not perform that synthetic body-neighborhood
clear (`src/mapping/rog_map_vendor/src/rog_map/prob_map.cpp:432-447`). The
current cutover removes the snapshot-level traversed support that was added in
the historical working tree. Mapping remains sensor-only; a stopped measured
planning request may carry a request-local rigid-body witness whose UNKNOWN
support is limited to a continuous prefix inside the current physical union.
The current authority is the cutover entry in
`docs/architecture/runtime_safety_decision_ledger.md`; the traversed-support
paragraphs in this historical comparison describe only the pinned snapshot.

The certificate in that snapshot checked the initial point and then every curved tube
against inflated known-free/traversed evidence
(`src/planning/navigation_planning_backend/include/planner_core/trajectory_world_validator.hpp:510-537`,
`:619-657`). Therefore a planned thin-body support shape may encounter UNKNOWN
inside the LiDAR near-field blind zone even when the measured center point is
supported. This is a hypothesis requiring per-voxel/initial-tube evidence: the
physical core shape may resolve only the body portion and cannot certify
unrelated UNKNOWN cells or supply an extra safety-inflation margin. Restoring
the upstream sphere or adding an FSM workaround would change the safety
assumption and is not justified by this comparison.

## Historical follow-up working-tree patch

In the historical working tree reviewed by this report, the bounded backend
change retained the old command only for
`NO_NEED`; `FINISH` proceeds through the existing EXP-only candidate builder,
authorization, and history staging at
`src/planning/navigation_planning_backend/src/planner_core/planner.cpp:1375-1497`.
The active-BACKUP command-end guard remains earlier in
`planner.cpp:2131-2151`, so a successor that reaches an active BACKUP end still
fails closed and does not return to nominal MAIN planning.

No new regression test is claimed for this one-line retention change. The
existing candidate-builder and terminal-rest/authorization tests cover the
underlying FINISH candidate contract, but they do not invoke
`Planner::planSuccessorFromExecutionAnchor()` with an old BACKUP command.
Direct ReplanOnce FINISH witness remains pending a runtime fixture; no SITL
evidence is implied.
