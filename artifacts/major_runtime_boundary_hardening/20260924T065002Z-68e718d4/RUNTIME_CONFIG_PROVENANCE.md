# Runtime configuration provenance

| Configuration fact | Requested owner | Generated input | Effective consumer | Current live witness | Status |
|---|---|---|---|---|---|
| Tracking experiment mode | `run_sim` argument / `_tracking_experiment_payload` | both `tracking_experiment.mode` parameters | shared `loadTrackingExperimentPolicy` in Core and adapter | both `RUNTIME_CONFIG_EFFECTIVE` startup logs, compared before mission | Implemented, runtime test pending |
| Tracking suppression | requested payload | `mode`, `tracking_gate_relaxed`, coefficients | Core/adapter `suppress_braking` | both startup logs | Implemented, runtime test pending |
| Fresh typed health suppression | requested payload | same selector | adapter `suppress_estimator_health_response` | adapter startup log | Implemented, runtime test pending |
| Velocity-only experiment | requested mode and bounds | adapter parameters | adapter policy | adapter startup flag and eleven numeric policy fields, compared before mission | Implemented, runtime test pending |
| Dynamics override | `sitl_dynamics_profile` | generated `planner.yaml` control envelope | PlannerFacade config loader | read-only PlannerFacade control envelope startup witness, compared with requested profile | Implemented, runtime test pending |
| Planner fault injection | runner injection arguments | generated `navigation_runtime.*` parameters | RuntimeNode constructor, diagnostic-only injection state | RuntimeNode startup witness for all current injection selectors, compared before mission | Implemented, runtime test pending |

Launch passes the generated parameter files unmodified except for
`use_sim_time` and `mission_file` overrides. The original ambiguity was at the
shared C++ loader: it treated `use_sim_time` as an enable request. This campaign
removes that inference. Source implementation still requires live SITL
verification; generated files alone do not prove runtime behavior.

The runner writes per-node `requested`, `effective`, and `source` tracking
records to `metadata.json.runtime_configuration` only after comparing the
structured live startup witness. The posthoc report reconciles those records
with metadata, scenario and parameter snapshots. Dynamics and injection
witnesses are checked at the same boundary. A missing/mismatched witness fails
setup before the scenario process starts.
