# NavigationCommand field consumer audit (Phase A)

Pinned baseline: `2b9279b42f3dce0d2565e4ef008852b018baf99f`. Exactly **74** nonconstant fields. This table classifies current *wire* consumers; same-named planner/diagnostic locals are not counted as wire consumers. Source anchors: `navigation_command_contract.hpp`, `navigation_runtime_node.cpp` (producer and Core receipt), `navigation_mode_node.cpp` (adapter), `tools/runtime/external_mode_scenario.py` (runner), plus repository searches over `src/`, `tools/runtime/`, `tests/`, `launch/`, `config/`, `docs/safety/`. No schema mutation precedes this audit.

| Field | Producer | Adapter reader | Other product reader | Safety decision? | Control? | Provenance? | Diagnostic? | Target |
| --- | --- | --- | --- | --- | --- | --- | --- | --- |
| `header` | Core `publishCommand` | contract/lease or recovery | Core receipt | yes | temporal | no | no | `KEEP_TEMPORAL` |
| `localization_epoch` | Core `publishCommand` | contract/session/monotonicity | Core issued-command/receipt | yes | identity | no | no | `KEEP_IDENTITY` |
| `goal_epoch` | Core `publishCommand` | contract/session/monotonicity | Core issued-command/receipt | yes | identity | no | no | `KEEP_IDENTITY` |
| `mode_activation_id` | Core `publishCommand` | contract/session/monotonicity | none | yes | identity | no | no | `KEEP_IDENTITY` |
| `mission_id` | Core `publishCommand` | contract/session/monotonicity | Core issued-command/receipt | yes | identity | no | no | `KEEP_IDENTITY` |
| `waypoint_index` | Core `publishCommand` | contract/session/monotonicity | Core issued-command/receipt | yes | identity | no | no | `KEEP_IDENTITY` |
| `request_id` | Core `publishCommand` | contract/session/monotonicity | Core issued-command/receipt | yes | identity | no | no | `KEEP_IDENTITY` |
| `world_generation` | Core `publishCommand` | contract/session/monotonicity | none | yes | identity | no | no | `KEEP_IDENTITY` |
| `world_revision` | Core `publishCommand` | contract/session/monotonicity | none | yes | identity | no | no | `KEEP_IDENTITY` |
| `world_observation_stamp` | Core `publishCommand` | contract/world monotonicity | none | yes | identity | no | no | `KEEP_IDENTITY` |
| `bundle_generation` | Core `publishCommand` | contract/session/monotonicity | Core issued-command/receipt | yes | identity | no | no | `KEEP_IDENTITY` |
| `certified_main_continuation` | Core `publishCommand` | contract role/status/continuation | Core continuation receipt | yes | safety | no | no | `KEEP_SAFETY` |
| `continuation_boundary_stamp_ns` | Core `publishCommand` | contract role/status/continuation | Core continuation receipt | yes | safety | no | no | `KEEP_SAFETY` |
| `sample_id` | Core `publishCommand` | contract/session/monotonicity | Core issued-command/receipt | yes | identity | no | no | `KEEP_IDENTITY` |
| `state_source_stamp` | Core `publishCommand` | contract/lease or recovery | none | yes | temporal | no | no | `KEEP_TEMPORAL` |
| `valid_until` | Core `publishCommand` | contract/lease or recovery | Core receipt | yes | temporal | no | no | `KEEP_TEMPORAL` |
| `role` | Core `publishCommand` | contract role/status/continuation | none | yes | safety | no | no | `KEEP_SAFETY` |
| `status` | Core `publishCommand` | contract role/status/continuation | none | yes | safety | no | no | `KEEP_SAFETY` |
| `reason_code` | Core `publishCommand` | none | none | no | no | yes | no | `MOVE_PROVENANCE` |
| `position` | Core `publishCommand` | contract/tracking/PX4 output | Core receipt endpoint | yes | PVAJ/yaw | no | no | `KEEP_CONTROL` |
| `velocity` | Core `publishCommand` | contract/tracking/PX4 output | none | yes | PVAJ/yaw | no | no | `KEEP_CONTROL` |
| `acceleration` | Core `publishCommand` | contract/tracking/PX4 output | none | yes | PVAJ/yaw | no | no | `KEEP_CONTROL` |
| `jerk` | Core `publishCommand` | contract/tracking/PX4 output | none | yes | PVAJ/yaw | no | no | `KEEP_CONTROL` |
| `yaw` | Core `publishCommand` | contract/tracking/PX4 output | none | yes | PVAJ/yaw | no | no | `KEEP_CONTROL` |
| `yaw_rate` | Core `publishCommand` | contract/tracking/PX4 output | none | yes | PVAJ/yaw | no | no | `KEEP_CONTROL` |
| `trajectory_time_s` | Core `publishCommand` | contract/lease or recovery | none | yes | temporal | no | no | `KEEP_TEMPORAL` |
| `analytic_sample_role` | Core `publishCommand` | none | none | no | no | no | yes | `MOVE_DIAGNOSTIC` |
| `backup_available` | Core `publishCommand` | none | none | no | no | no | yes | `MOVE_DIAGNOSTIC` |
| `backup_start_time_s` | Core `publishCommand` | none | none | no | no | no | yes | `MOVE_DIAGNOSTIC` |
| `time_to_backup_start_s` | Core `publishCommand` | none | none | no | no | no | yes | `MOVE_DIAGNOSTIC` |
| `safety_suffix_active` | Core `publishCommand` | none | none | no | no | no | yes | `MOVE_DIAGNOSTIC` |
| `execution_recovery_state` | Core `publishCommand` | none | none | no | no | no | yes | `MOVE_DIAGNOSTIC` |
| `anchor_error_m` | Core `publishCommand` | none | none | no | no | no | yes | `MOVE_DIAGNOSTIC` |
| `projected_anchor_error_m` | Core `publishCommand` | none | none | no | no | no | yes | `MOVE_DIAGNOSTIC` |
| `retained_tracking_limit_m` | Core `publishCommand` | none | none | no | no | no | yes | `MOVE_DIAGNOSTIC` |
| `relative_anchor_speed_mps` | Core `publishCommand` | none | none | no | no | no | yes | `MOVE_DIAGNOSTIC` |
| `committed_suffix_usable` | Core `publishCommand` | none | none | no | no | no | yes | `MOVE_DIAGNOSTIC` |
| `sampled_path_clear` | Core `publishCommand` | none | none | no | no | no | yes | `MOVE_DIAGNOSTIC` |
| `tracking_certificate_exceeded` | Core `publishCommand` | none | none | no | no | no | yes | `MOVE_DIAGNOSTIC` |
| `projected_tracking_certificate_exceeded` | Core `publishCommand` | none | none | no | no | no | yes | `MOVE_DIAGNOSTIC` |
| `emergency_authorization_reason` | Core `publishCommand` | velocity-only experiment role authorization | none | yes | safety | no | no | `KEEP_SAFETY` |
| `causal_planning_cycle_id` | Core `publishCommand` | PX4 input trace only | none | no | no | yes | no | `MOVE_PROVENANCE` |
| `causal_timestamp_ns` | Core `publishCommand` | none | none | no | no | yes | no | `MOVE_PROVENANCE` |
| `emergency_candidate_commit_result` | Core `publishCommand` | velocity-only experiment role authorization | none | yes | safety | no | no | `KEEP_SAFETY` |
| `execution_authorization` | Core `publishCommand` | none | Core issued-command authorization check | Core-local | no | yes | no | `MOVE_PROVENANCE` |
| `execution_authorization_steady_ns` | Core `publishCommand` | none | none | no | no | yes | no | `MOVE_PROVENANCE` |
| `evaluation_now_ns` | Core `publishCommand` | none | none | no | no | no | yes | `MOVE_DIAGNOSTIC` |
| `execution_state_source_stamp_ns` | Core `publishCommand` | none | none | no | no | no | yes | `MOVE_DIAGNOSTIC` |
| `execution_state_receive_stamp_ns` | Core `publishCommand` | none | none | no | no | no | yes | `MOVE_DIAGNOSTIC` |
| `execution_state_source_age_ms` | Core `publishCommand` | none | none | no | no | no | yes | `MOVE_DIAGNOSTIC` |
| `execution_state_receive_age_ms` | Core `publishCommand` | none | none | no | no | no | yes | `MOVE_DIAGNOSTIC` |
| `committed_bundle_start_stamp_ns` | Core `publishCommand` | none | none | no | no | no | yes | `MOVE_DIAGNOSTIC` |
| `measured_position_at_state_source` | Core `publishCommand` | none | none | no | no | no | yes | `MOVE_DIAGNOSTIC` |
| `measured_velocity_at_state_source` | Core `publishCommand` | none | none | no | no | no | yes | `MOVE_DIAGNOSTIC` |
| `committed_command_position_at_now` | Core `publishCommand` | none | none | no | no | no | yes | `MOVE_DIAGNOSTIC` |
| `committed_command_velocity_at_now` | Core `publishCommand` | none | none | no | no | no | yes | `MOVE_DIAGNOSTIC` |
| `committed_command_position_at_state_source` | Core `publishCommand` | none | none | no | no | no | yes | `MOVE_DIAGNOSTIC` |
| `committed_command_velocity_at_state_source` | Core `publishCommand` | none | none | no | no | no | yes | `MOVE_DIAGNOSTIC` |
| `anchor_error_raw_m` | Core `publishCommand` | none | none | no | no | no | yes | `MOVE_DIAGNOSTIC` |
| `anchor_error_time_aligned_m` | Core `publishCommand` | none | none | no | no | no | yes | `MOVE_DIAGNOSTIC` |
| `command_motion_over_state_age_m` | Core `publishCommand` | none | none | no | no | no | yes | `MOVE_DIAGNOSTIC` |
| `velocity_residual_time_aligned_mps` | Core `publishCommand` | none | none | no | no | no | yes | `MOVE_DIAGNOSTIC` |
| `retained_elapsed_s` | Core `publishCommand` | none | none | no | no | no | yes | `MOVE_DIAGNOSTIC` |
| `committed_bundle_duration_s` | Core `publishCommand` | none | none | no | no | no | yes | `MOVE_DIAGNOSTIC` |
| `committed_safety_transition_time_s` | Core `publishCommand` | none | none | no | no | no | yes | `MOVE_DIAGNOSTIC` |
| `retained_validate_without_new_commit` | Core `publishCommand` | none | none | no | no | no | yes | `MOVE_DIAGNOSTIC` |
| `retained_fresh_vehicle_state` | Core `publishCommand` | none | none | no | no | no | yes | `MOVE_DIAGNOSTIC` |
| `retained_committed_command_available` | Core `publishCommand` | none | none | no | no | no | yes | `MOVE_DIAGNOSTIC` |
| `retained_command_anchor_valid` | Core `publishCommand` | none | none | no | no | no | yes | `MOVE_DIAGNOSTIC` |
| `current_vehicle_state_known_free` | Core `publishCommand` | none | none | no | no | no | yes | `MOVE_DIAGNOSTIC` |
| `retained_safety_trajectory_available` | Core `publishCommand` | none | none | no | no | no | yes | `MOVE_DIAGNOSTIC` |
| `retained_terminal_stop` | Core `publishCommand` | none | none | no | no | no | yes | `MOVE_DIAGNOSTIC` |
| `retained_committed_role` | Core `publishCommand` | none | none | no | no | no | yes | `MOVE_DIAGNOSTIC` |
| `retained_recovery_state_before` | Core `publishCommand` | none | none | no | no | no | yes | `MOVE_DIAGNOSTIC` |

## Findings that constrain the split

- `execution_authorization` is **Core-local mission issuance authority**, read by `rememberMissionCommandIssued()`, and runner provenance. Adapter admission does not read it. A wire move requires Core to pass its local authorization witness directly to its issuance bookkeeping; the diagnostic copy cannot authorize flight. Its wire role is `MOVE_PROVENANCE`, not `DELETE_UNUSED`.
- `certified_main_continuation` and `continuation_boundary_stamp_ns` are checked by `commandContractValid()` and by Core when binding an exact adapter admission receipt to a mission continuation witness. They remain `KEEP_SAFETY` under the behavior-equivalence constraint. Removing them would change malformed-command rejection semantics.
- `emergency_authorization_reason` and `emergency_candidate_commit_result` are read by `publishVelocityOnlySetpoint()` to decide whether an emergency role may cross the experimental velocity-only boundary. Despite their location in a block commented “diagnostic-only”, they are current safety/control inputs and remain `KEEP_SAFETY`. This comment mismatch is a source finding; it cannot be corrected by silently removing those fields.
- `causal_planning_cycle_id` is copied to `Px4InputTraceRecord` for evidence, not admission/PX4 control; it may move, with trace tooling migrated.
- All other fields marked `MOVE_DIAGNOSTIC` have Core producer assignments and diagnostic/evaluator readers, but no adapter control reader found. Diagnostic loss cannot affect acceptance.
- `reason_code` has a Core producer assignment and no adapter decision reader; it is provenance, not a control reason.
- No `UNRESOLVED` field remains in this pinned in-repository consumer audit. Unknown external ROS consumers are a separate compatibility decision, not an invented safety reader.

## Count

- `KEEP_IDENTITY=11`, `KEEP_TEMPORAL=4`, `KEEP_CONTROL=6`, `KEEP_SAFETY=6`. Retained control fields: **27**.
- `MOVE_PROVENANCE=5`; `MOVE_DIAGNOSTIC=42`; `DERIVE=0`; `DELETE_UNUSED=0`; `UNRESOLVED=0`.
