# Control-plane field allowlist

The 27 retained fields below are the pinned baseline decisions; the message split must preserve their producer, adapter reader, and reason in `NAVIGATION_COMMAND_FIELD_AUDIT.md`.

- `header` — `KEEP_TEMPORAL`.
- `localization_epoch` — `KEEP_IDENTITY`.
- `goal_epoch` — `KEEP_IDENTITY`.
- `mode_activation_id` — `KEEP_IDENTITY`.
- `mission_id` — `KEEP_IDENTITY`.
- `waypoint_index` — `KEEP_IDENTITY`.
- `request_id` — `KEEP_IDENTITY`.
- `world_generation` — `KEEP_IDENTITY`.
- `world_revision` — `KEEP_IDENTITY`.
- `world_observation_stamp` — `KEEP_IDENTITY`.
- `bundle_generation` — `KEEP_IDENTITY`.
- `certified_main_continuation` — `KEEP_SAFETY`.
- `continuation_boundary_stamp_ns` — `KEEP_SAFETY`.
- `sample_id` — `KEEP_IDENTITY`.
- `state_source_stamp` — `KEEP_TEMPORAL`.
- `valid_until` — `KEEP_TEMPORAL`.
- `role` — `KEEP_SAFETY`.
- `status` — `KEEP_SAFETY`.
- `position` — `KEEP_CONTROL`.
- `velocity` — `KEEP_CONTROL`.
- `acceleration` — `KEEP_CONTROL`.
- `jerk` — `KEEP_CONTROL`.
- `yaw` — `KEEP_CONTROL`.
- `yaw_rate` — `KEEP_CONTROL`.
- `trajectory_time_s` — `KEEP_TEMPORAL`.
- `emergency_authorization_reason` — `KEEP_SAFETY`.
- `emergency_candidate_commit_result` — `KEEP_SAFETY`.
