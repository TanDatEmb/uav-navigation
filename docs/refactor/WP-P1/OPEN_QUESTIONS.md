# WP-P1 — open questions

## OQ-P1-THRUST — thrust profile semantics remain unresolved

- Classification: `CONDITIONAL`; code traces both the configured input and computed effective physical bounds, but A6 labels the raw configuration literals with conflicting units.
- Evidence: `planner.yaml:103-104` supplies `max_acc_thr=25.0` / `min_acc_thr=6.0`. `PlannerCore::Config` constructs the physical model with `min_acc_thr * mass` and `max_acc_thr * mass` for both EXP and BACKUP (`src/planning/navigation_planning_backend/include/planner_core/config.hpp:205-214`); the trajectory hard gate uses the same multiplication (`src/planning/navigation_planning_backend/include/traj_opt/trajectory_dynamics.hpp:160-165`). With configured mass 1.64 kg (`planner.yaml:181`), those runtime bounds are 9.84 N and 41.0 N. The similarly named `VehicleDynamicModel` defaults in `src/planning/navigation_planning/include/navigation_planning/planning_limits.hpp:22-23` are also 9.84 N and 41.0 N; production config constructs the model from YAML-derived fields. The A6 rows currently label raw 6.0 / 25.0 as N, so including them under those row names would claim the wrong effective values.
- Owner decision needed: approve a representation that preserves both the configured scalar bounds (`min_acc_thr` / `max_acc_thr`, in the loader's input units) and the derived physical bounds (N), and decide how the two existing A6 oracle rows map or are amended. This WP does not rename units/keys or edit the A6 oracle on the owner's behalf.
- Current handling: both A6 rows remain absent from `sitl_current_as_is.yaml`, listed under profile `open_questions`, and fail `check_against_a6.py`. No thrust value or consumer behavior was changed.

## OQ-P1-YAWACC — A6 yaw-acceleration row conflicts with effective planner YAML

- Classification: `CONDITIONAL`; source tracing proves a current effective runtime value but conflicts with the merged A6 oracle row.
- A6 row `max_yaw_acceleration_rad_s2` records `0.3 rad/s^2` from `planning_limits.hpp:21` and marks itself not YAML-loadable. However `planner.yaml:73` sets `planner.yaw_acceleration_max_rad_s2: 2.0`; `PlannerCore::Config` loads that field (`navigation_planning_backend/include/planner_core/config.hpp:203-204`) and uses it to construct both EXP and BACKUP physical models (`:205-214`). The effective current planner limit is therefore `2.0 rad/s^2`; the `0.3` struct default is superseded in this runtime path.
- Owner action: confirm whether A6 is amended to identify the YAML alias/source and effective `2.0` value, or whether a separate key is required for the unused/default `0.3` value. This WP does not alter A6 or claim `0.3` is effective.
- Current handling: `max_yaw_acceleration_rad_s2` is added to profile `open_questions` and omitted from the current profile until the oracle representation is approved. Oracle check now reports this unresolved row together with the two thrust rows.

## OQ-P1-ADR — outstanding ADR-017 §6.3 owner questions

Q-ENV, Q-VOX, Q-UNK, Q-TRK and Q-XTRK remain pending. WP-P1 does not use this work to settle those questions or change their values or policy.

## OQ-P1-H2-OVERLAP — shared PX4 external bridge source

- `src/px4/px4_odometry_bridge/src/px4_external_odometry_bridge_node.cpp` is also changed by H2 / PR #14.
- Keep the P1.2 profile loading and witness edits on this branch. After the owner merges H2, rebase this branch onto the updated base and resolve the shared-file changes by preserving both independent changes; do not cherry-pick H2.
- This is a coordination note, not a reason to change or delay the H2 branch.
