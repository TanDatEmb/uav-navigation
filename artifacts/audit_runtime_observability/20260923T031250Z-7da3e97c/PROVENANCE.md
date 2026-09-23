# Frozen audit provenance

| Item | Value |
|---|---|
| Audit branch | `codex/audit-runtime-observability-20260923` |
| Parent audit commit | `8334e1f1c3c3064d63d5f136d43fcfe218514540` |
| Product target | `7da3e97cb399c2e39d62cfe60213a45e8a92300e` |
| Product tree | `e7d2b56dbe4353810f029c9c6f0fe8991ebd63eb` |
| Pinned PX4 interface dependency | `4a3370f084ac6f1ef001a4afa2b007845ffd0837` |
| Product files/config changed | 0 |
| Raw run root | `/home/letandat/Dev/uav-navigation/.artifacts/runtime/` |
| Eligible run selection | `*20260922*`, metadata `repo_commit=PRODUCT_TARGET`, `repo_dirty=false` |
| Historical replay runs | 14 on 2026-09-22, mixed profiles; all FAIL/BLOCKED |
| Focused O3 runs | 4 smoke SITL attempts on 2026-09-23, all BLOCKED; one complete external PX4 protocol bag |

This branch reuses the previous audit's extraction at `/home/letandat/Dev/uav-navigation-audit-runtime-evidence-closure-20260923/.artifacts/audit_runtime_evidence_closure/raw/` and its committed CSV under `artifacts/audit_runtime_evidence_closure/20260923T023054Z-7da3e97c/`. O1 reruns the *pinned* `RouteProgress` implementation for route arc on the selected episode. Raw output is in this worktree's ignored `.artifacts/audit_runtime_observability/raw/`. O2 joins exact command identities from existing runner timelines; O3 normalizes existing status/log/ACK events; O4 retains per-run diagnostic values. Four focused smoke SITL attempts were made for O3 only; one sidecar bag captured the complete VehicleCommand/ACK/ModeCompleted/VehicleStatus topic set. They were not added to the historical O1/O2/O4 datasets or pooled timing percentiles. No existing safe mapping-input fault injection was found, and O1/O2/O4 internal callback/receive boundaries remain absent. The focused O3 runs use the existing relaxed tracking experiment configuration and are diagnostic BLOCKED outcomes, not qualification.

Evidence classes remain separate: `FACT_FROM_TARGET_CODE`, `FACT_FROM_PINNED_DEPENDENCY`, `FACT_FROM_RUN_FIRMWARE_CHECKOUT`, `FACT_FROM_EXISTING_RUNTIME_ARTIFACT`, `FACT_FROM_AUDIT_REPLAY`, `INFERENCE`, and `UNOBSERVABLE_WITH_CURRENT_TARGET`. A run firmware checkout and hashed binary identify the observed run environment, but do not qualify a product-wide firmware baseline or prove the binary was built from the *current* dirty checkout. See `O3_PX4_RUN_FIRMWARE_PROVENANCE.md`.

Clock rules: ROS source/simulation time is compared only within ROS source domain; runner observer and adapter steady clocks use host monotonic time but observer arrival is not producer/consumer receipt; PX4 boot time is a separate domain. The O2 authorization and adapter setpoint timestamps both call `navigation_common::steadyClockNowNanoseconds()` on the same host; their difference is a causal authorization→first-setpoint upper envelope that includes more than transport. It is not a measured publish→receive latency. The one infrastructure-invalid simulation-pause run is kept only with that fault label.
