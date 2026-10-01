# Ứng viên evidence từ diagnostics

Đây là inventory của các consumer có vai trò `judge` hoặc `safety_gate` trên
baseline `main@7e0b850`. Giá trị trên wire đều là `DiagnosticStatus.values[].value`
(string); kiểu bên dưới là kiểu consumer cố gắng parse, không phải kiểu ROS mới.
`name-only` nghĩa là consumer chỉ dùng sự hiện diện/tên kênh, chưa có witness
typed tương đương.

| DiagnosticStatus.name | Consumer và mục đích | Key thực sự đọc | Kiểu consumer | Source |
|---|---|---|---|---|
| `fast_lio/estimator` | `px4_external_odometry_bridge_node`, safety gate | `lio_public_frame_generation`, `lio_public_frame_generation_valid`, `navigation_valid`, `corrected_estimate_valid`, `status`, `pose_covariance_available`, `twist_covariance_available` | uint, bool, bool, bool, string, bool, bool | `src/px4/px4_odometry_bridge/src/px4_external_odometry_bridge_node.cpp:158-201` |
| `fast_lio/estimator` | `external_mode_scenario`, judge/health snapshot | `status`, `navigation_valid`, `observability_valid` hoặc fallback `translation_observability_valid`, `correction_fresh`, `propagation_valid` | string, bool, bool, bool, bool, bool | `tools/runtime/external_mode_scenario.py:1005-1019` |
| `fast_lio/estimator` | `report.py`, judge summary | `map_point_count`, `absolute_guard_triggered`, `absolute_guard_recovery_failed`, `map_maintenance_us` | numeric/string-to-number, bool-ish, bool-ish, float | `tools/runtime/report.py:1540-1578` |
| `navigation_runtime/planner` | `world_observation_gate`, gate readiness | no key; only name membership | name-only | `tools/runtime/world_observation_gate.py:233-239` |
| `navigation_runtime/planner` | `analyze_e5_temporal_alignment.py`, judge trace | `message == DECISION_TRACE`, then forwards every key in `values` | string plus per-key dynamic | `tools/runtime/analyze_e5_temporal_alignment.py:110-118` |
| `navigation_runtime/planner` | `external_mode_scenario`, judge lifecycle/trace | `world_transaction_events_produced`, `world_transaction_runtime_instance_id`, `localization_epoch`, `world_generation`, `world_revision`; otherwise forwards all `values` to planner lifecycle | int, string, int, int, int, dynamic | `tools/runtime/external_mode_scenario.py:981-1004` |
| `navigation_mapping/world_model` | `external_mode_scenario`, judge | `world_transaction_events_produced`, `world_transaction_runtime_instance_id`, `world_generation`, `world_revision` | int, string, int, int | `tools/runtime/external_mode_scenario.py:755-768` |
| `navigation_mapping/world_model` | `runner.py`, gate precondition | `accepted_observation_count`, `visualization_publish_count`, `visualization_subscriber_count`, `visualization_exception_count` | int | `tools/runtime/runner.py:2159-2171` |
| `navigation_external_mode/ALIGNMENT_LATCH_WITNESS` | `external_mode_scenario`, judge evidence | all `values` are retained as an opaque witness; no named key is interpreted | dynamic/opaque | `tools/runtime/external_mode_scenario.py:769-782` |
| `navigation_external_mode/PX4_INPUT_SETPOINT` | `external_mode_scenario`, judge | `trace_sequence`, `setpoint_update_duration_ns`, `setpoint_boundary`, `trace_enqueued_count`, `trace_published_before_count`, `trace_drop_count`, `trace_publish_error_count`, `sample_id`, `request_id`, `goal_epoch`, `localization_epoch`, `bundle_generation`, `causal_planning_cycle_id`, `world_generation`, `world_revision`, `world_observation_stamp_ns`, `update_end_ros_ns` | int, int, string, int, int, int, int, int, int, int, int, int, int, int, int, int, int, int | `tools/runtime/external_mode_scenario.py:783-825` |
| `navigation_external_mode/PX4_INPUT_SETPOINT` | `analyze_h10.py`, judge | `trace_timestamp_ns`, `trace_sequence`, and all copied trace fields | int, int, dynamic | `tools/runtime/analyze_h10.py:126-133` |
| `navigation_runtime/retained_command_decision` | `external_mode_scenario`, judge lifecycle | `event_sequence`, `purpose`, `planning_cycle_id`, desired/execution request and epoch, `localization_epoch`, captured/after bundle generation, `disposition`, owner/callback/current flags, final freshness/body/anchor/bridge flags, source/receive timestamps, ingress/publish/suppress/fail counters | int/bool-as-int, dynamic numeric | `tools/runtime/external_mode_scenario.py:827-895` |
| `navigation_runtime/heading_rebind_admission_witness` | `external_mode_scenario`, judge lifecycle | `request_id`, `goal_epoch`, `localization_epoch`, `bundle_generation`, `bundle_source`, `parent_bundle_generation`, `world_generation`, `world_revision`, `activation_stamp_ns` | int, string, int | `tools/runtime/external_mode_scenario.py:896-913` |
| `navigation_runtime/execution_pending_superseded_witness` | `external_mode_scenario`, judge lifecycle | `request_id`, `goal_epoch`, `localization_epoch`, `bundle_generation`, `bundle_source`, `bundle_owner_cycle_id`, `replacement_bundle_generation`, `admission_goal_epoch`, `previous_snapshot_version`, `current_snapshot_version` | int, string, int | `tools/runtime/external_mode_scenario.py:914-935` |
| `navigation_runtime/recovery_retry_witness` | `external_mode_scenario`, judge lifecycle | `identity_current`, `timeout`, `request_id`, `goal_epoch`, `localization_epoch`, `planning_cycle_id`, `active_generation`, `after_failed`, `terminal_hold_pending` | bool-as-int, bool-as-int, int, int, int, int, int, bool-as-int, bool-as-int | `tools/runtime/external_mode_scenario.py:936-959` |
| `navigation_runtime/execution_activation_witness` | `external_mode_scenario`, judge lifecycle | `activation_stamp_ns`, `request_id`, `goal_epoch`, `localization_epoch`, `bundle_generation`, `bundle_source`, `bundle_owner_cycle_id`, `world_generation`, `world_revision`, `pending_snapshot_version` | int, int, int, int, int, string, int, int, int, int | `tools/runtime/external_mode_scenario.py:960-979` |
| `navigation_runtime/planner_path` | `html_report.py`, judge/report rendering | `planner_path_snapshot_json` | JSON string | `tools/runtime/html_report.py:467-489` |
| `navigation_planning/planner` | legacy alias accepted by tools/tests | no key; name compatibility only | name-only | `tools/runtime/report.py:2618-2663`; `tools/runtime/planner_trace.py:968-975` |

`navigation_runtime/exact_optimization_failure_injection` is emitted by the
producer at `src/runtime/navigation_runtime/src/navigation_runtime_node.cpp:6042-6085`.
At this baseline the scenario branch at `external_mode_scenario.py:981` is for
`navigation_runtime/planner`, not this exact name; therefore this channel has
no confirmed consumer and is not promoted as a judge input. Its producer keys
are enumerated in `diagnostic_channels.yaml` for future typed evidence design.

Các kênh `fast_lio/transport`, `fast_lio/propagated_odometry` và
`px4_odometry_bridge` được publish/thu thập cho observability nhưng không có
consumer judge/gate name-specific trong source đã quét; chúng không được liệt
kê như evidence candidate chính.
