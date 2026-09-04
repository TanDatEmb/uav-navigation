# E5 tracking root-cause decomposition

## Scenario scope

{
  "experiment_id": "E05_temporal_alignment_replay_exact",
  "run_id": "external-mode-check-20260904T125955-39028",
  "map": "open/sanity_open",
  "route": "external_mode_open_route: pass_through [0,0,3]->[3,0,3]->[6,0,3]->[7,0,3], terminal stop",
  "requested_speed_mps": 3.0,
  "planner_rate_hz": 5.0,
  "command_rate_hz": 50.0,
  "tracking_limit_m": 0.25,
  "recorded_repo_commit": "cac82aad20771b13f42cf11bd1af93616e51b44b",
  "recorded_repo_dirty": true,
  "planner_config_path": "/home/letandat/Dev/uav-navigation/src/runtime/navigation_runtime/config/planner.yaml",
  "planner_config_sha256": "1814cc0f5bb73a0a29b3662afcf198923628ef0a987e70c3ac1baf883bbd20b2",
  "mission_file": "/home/letandat/Dev/uav-navigation/.artifacts/runtime/external-mode-check-20260904T125955-39028/resolved_mission.yaml",
  "mission_config_sha256": "166104b3b50832a8e285d8fa78953f6c2be6f22b2e0509b8d667fb02fcde796d",
  "scenario_config_sha256": "976aa13f01b2bb69eb552bdfbcfa54065e249a1bbda66390b9b8f4bd69ed1905"
}

This is the exact E05 temporal-alignment replay artifact. It is not mixed with any open control run. The raw rosbag remains retained locally.

## Time base and transforms

- Fixed position offset: `[0.17189057034194707, 0.13237223465488202, -0.009499036596539945]` m, calibrated once at first active command.
- LIO position is compared in LIO ENU; PX4 position is compared after ENU→NED axis permutation plus the fixed offset.
- LIO `linear_velocity` is `child_frame_id=base_link`; it is rotated by the recorded quaternion before ENU→NED velocity residual calculation.
- PX4 input trajectory values are present but their PX4 timestamp field is zero for all 498 samples, so synchronized input-layer cells are `NOT_RECORDED`.

## T_cross and events

| Event | timestamp_ns | e_lio [m] | e_px4 [m] | frame pos [m] | frame vel [m/s] | planner |V| [m/s] | planner |A| [m/s²] | dV [m/s] | dA [m/s²] | mode |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---|
| bundle_activation_first | 23064000000 | 4.2324e-05 | 4.2324e-05 | 0 | 0.0128109 | 0.037302 | 0.00017109 | 0.0392758 | 0.190492 | WAIT_FIRST_COMMAND |
| gen2_activation | 29048000000 | 0.133915 | 0.0479479 | 0.090681 | 0.127047 | 0.75644 | 1.74097 | 0.0170524 | 0.569932 | TRACK_TRAJECTORY |
| sustained_error_growth_start | 27515487170 | 0.100791 | 0.0351462 | 0.0771882 | 0.0284771 | 0.58174 | 0.173724 | 0.0204084 | 0.0883496 | WAIT_FIRST_COMMAND |
| tracking_error_0.10_m | 27515487170 | 0.100791 | 0.0351462 | 0.0771882 | 0.0284771 | 0.58174 | 0.173724 | 0.0204084 | 0.0883496 | WAIT_FIRST_COMMAND |
| tracking_error_0.20_m | 29216250989 | 0.199166 | 0.0970044 | 0.108927 | 0.116417 | 1.03454 | 1.72912 | 0.0474349 | 0.849437 | TRACK_TRAJECTORY |
| T_cross | 29319484858 | 0.252087 | 0.146179 | 0.115476 | 0.10509 | 1.25607 | 2.24222 | 0.044546 | 0.913539 | TRACK_TRAJECTORY |
| injected_solve_start | NOT_RECORDED | NOT_RECORDED | NOT_RECORDED | NOT_RECORDED | NOT_RECORDED | NOT_RECORDED | NOT_RECORDED | NOT_RECORDED | NOT_RECORDED | NOT_RECORDED |
| planner_failure_return | 29932000000 | 0.4428736168588373 | 0.384072 | 0.0897398 | 0.175064 | 2.70676 | 1.53444 | 0.347519 | 1.31458 | TRACK_TRAJECTORY |
| emergency_authorization | 29932000000 | 0.4428736168588373 | 0.384072 | 0.0897398 | 0.175064 | 2.70676 | 1.53444 | 0.347519 | 1.31458 | TRACK_TRAJECTORY |
| emergency_activation | 29924000000 | 0.0444696 | 0.0711222 | 0.0896675 | 0.17768 | 2.79347 | 0.0235819 | 0.258337 | 2.7723 | TRACK_TRAJECTORY |

T_cross is the first synchronized LIO error crossing above 0.25 m: `29319484858` ns. Sustained-growth start is reported as the first .10 m crossing (an auditable proxy, not a fitted claim): `27515487170` ns.

## Generation boundary measurements

| from | to | timestamp_ns | ΔP [m] | ΔV [m/s] | ΔA [m/s²] | ΔJ [m/s³] |
|---:|---:|---:|---:|---:|---:|---:|
| 1 | 2 | 29048000000 | 0.01187713846332408 | 0.028492384047986964 | 0.06705020515422436 | 4.800665643599643 |
| 2 | 3 | 29848000000 | 0.04084575150104724 | 0.03159716417308524 | 0.08414227767305582 | 0.11826564498133235 |
| 3 | 4 | 29924000000 | 0.4104937697510442 | 0.12351337677190562 | 1.556521603127011 | 1.7657398721095448 |

## Scenario-scoped H8 classification

### H8a_command_discontinuity — REJECTED

- Before T_cross, observed generation boundaries have no large P/V/A jump; the first boundary sharply preceding growth has ΔP=0.011877 m, ΔV=0.028492 m/s, ΔA=0.067050 m/s². The emergency boundary is post-crossing and is therefore not an initiating cause.

### H8b_dynamic_tracking_insufficiency — CONFIRMED

- Synchronized LIO error grows before planner failure, with a measured slope of 0.43445373391010655 m/s after gen2 activation versus 0.014167034761644003 m/s earlier; the command remains MAIN and continuous while requesting a rapid speed ramp and multi-m/s² acceleration in the exact E5 window.

### H8c_px4_control_reshaping — INCONCLUSIVE

- PX4 effective setpoints are available and controller deltas are computed, but no predeclared quantitative threshold establishes that reshaping is significant enough to be causal.

### H8d_px4_lio_state_divergence — REJECTED

- Using the fixed first-active-sample transform and rotating the recorded body twist by q_xyzw, frame residuals remain small/stable relative to the 0.25 m planner-to-state loss before T_cross; no residual growth explains the crossing.

### H8e_command_setpoint_interruption — INCONCLUSIVE

- A non-track External Mode state appears before T_cross; causality requires further event correlation.
- No pre-cross command gap >=100 ms was observed.

## Matched control (separate statistics)

The closest available control is E01_straight_3mps on the open/legacy environment at the same requested speed. It is not merged with E5 and cannot invalidate an E5-specific finding.

| Metric | E5 exact stressful scenario | matched S0 open control |
|---|---:|---:|
| LIO tracking RMS [m] | 0.09010142545887567 | 0.3659181962336282 |
| LIO tracking P95 [m] | 0.12722852520142286 | 0.6402835039246093 |
| frame position P95 [m] | 0.08645454337417305 | 0.2977420404301454 |
| frame velocity P95 [m/s] | 0.051413748928087165 | 0.48924192333748934 |

## Exact causal answers

- Tracking-error growth begins before the observed planner-failure return and is already sharply increasing after bundle generation 2 activation. The first synchronized crossing is `29319484858` ns.
- Measured error growth rates are `{'26.0_to_28.5_s': 0.014167034761644003, '28.5_to_gen2_activation': 0.03809694116720457, 'gen2_activation_to_T_cross': 0.43445373391010655}` m/s for the documented windows; the largest pre-cross increase is after gen2 activation.
- The strongest measured pre-cross mechanism is dynamic closed-loop tracking insufficiency against the rapid MAIN speed/acceleration ramp; the command remains continuous in P/V/A at the preceding generation boundary.
- At the retained-command validation boundary, the exact synchronized error is `0.4428736168588373` m (raw `0.4532910267671672` m) versus the 0.25 m limit.
- The planner failure is temporally after the 0.25 m crossing, so it is not the initiating cause of tracking loss in this exact artifact. It authorizes emergency only after the retained certificate is already exceeded.
- Emergency authorization is consistent with the recorded synchronized raw runtime predicate at that moment: the retained MAIN LIO-frame error is already above 0.25 m, while PX4/LIO frame residual does not explain the error. This is an evidence classification, not a safety-policy endorsement or fix.

## Data limitations

- /fmu/in/trajectory_setpoint is present in the rosbag but every PX4 message timestamp is zero; input layer is NOT_RECORDED in common simulation time.
- Planner solve start is NOT_RECORDED; the trace records failure return/decision timestamp and planning latency, but this analysis does not back-calculate a solve start.
- No injected_replan_failure=1 is present at the observed failure trace in this exact-v2 artifact; failure is reported as observed planner failure, not asserted as injected.

## Figures

- `runtime_evidence/2026-09-04/E05_temporal_alignment_replay_exact_v2/figures/plot_E5_tracking_divergence.png`
- `runtime_evidence/2026-09-04/E05_temporal_alignment_replay_exact_v2/figures/plot_E5_controller_correction.png`
- `runtime_evidence/2026-09-04/E05_temporal_alignment_replay_exact_v2/figures/plot_E5_velocity_layers.png`
- `runtime_evidence/2026-09-04/E05_temporal_alignment_replay_exact_v2/figures/plot_E5_acceleration_layers.png`
- `runtime_evidence/2026-09-04/E05_temporal_alignment_replay_exact_v2/figures/plot_E5_controller_correction_vs_tracking_error.png`
- `runtime_evidence/2026-09-04/E05_temporal_alignment_replay_exact_v2/figures/plot_E5_lio_px4_consistency.png`
- `runtime_evidence/2026-09-04/E05_temporal_alignment_replay_exact_v2/figures/plot_E5_generation_mode_timeline.png`
