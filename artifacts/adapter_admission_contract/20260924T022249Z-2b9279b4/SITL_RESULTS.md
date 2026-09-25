# Focused SITL observations

All three nominal runs use source `33ec3362`, PX4 binary and checkout in `BASE_PROVENANCE.md`, `long_featured`, seed 0, and runner argument `tracking_experiment_mode="off"`. The runner's overall verdict is `FAIL` because versioned evaluation is not qualification eligible. Mission `COMPLETE` is reported here as the narrower liveness observation, never as flight acceptance.

| Run | Raw session suffix | Mission | Accepted | Command/admission/rejection | Adapter admission max gap | Unexpected Hold request |
| --- | --- | --- | --- | --- | ---: | --- |
| N1 | `030555-816276` | COMPLETE | `[0,1,2,3,4]` | 2882 / 2882 / 0 | 21.317 ms | none |
| N2 | `030839-821361` | COMPLETE | `[0,1,2,3,4]` | 3142 / 3142 / 0 | 20.616 ms | none |
| N3 | `031122-825633` | COMPLETE | `[0,1,2,3,4]` | 2821 / 2817 / 4 | 40.328 ms | none |

The 12 predecessor-to-successor adapter admission gaps are min/median/max **19.848 / 19.988 / 20.266 ms**. N3's four rejections are typed `TEMPORAL_LEASE / NOT_YET_VALID / REJECT_RETAIN_PREVIOUS` (stage 2, reason 3, disposition 1). Their command header stamps led callback ROS time by about 4 ms. Retaining the previously admitted command kept the maximum observed admission gap below the unchanged 100 ms receive lease. No identity, continuity, or lease-expiry rejection was observed in these three runs.

**Existing tracking provenance mismatch:** in all three runs, metadata says tracking mode `off`, while the pinned adapter's `loadTrackingExperimentPolicy()` treats `use_sim_time=true` plus zero tracking coefficients as `suppress_braking=true`. The adapter logs `TRACKING_EXPERIMENT_BYPASS`, and the evaluator marks `tracking_experiment.status=INCONCLUSIVE`, `config_mismatch=true`, `qualification_eligible=false`. The same mismatch is present in the pinned baseline session `014351-704845`; this branch did not create or change that policy. These runs establish mission parity under the current harness bytes, but do not establish a genuinely unsuppressed tracking-off safety run.

The following fault attempts are excluded from the 3/3 nominal denominator:

| Attempt | Raw session suffix | Observation | Evidence status |
| --- | --- | --- | --- |
| exploratory CLI default | `030243-812261` | runner used default `tracking=relaxed`; mission COMPLETE | wrong profile, excluded |
| exact OptimizationFailed #1 | `031341-829406` | interrupted before report when task session was interrupted | incomplete, excluded |
| exact OptimizationFailed #2 | `042928-7952` | adapter odometry receive-stale 215 ms before first command; Hold requested; hook never armed | no hot-handoff evidence |
| Core-pause attempt #1 | `043656-15271` | spontaneous odometry receive-stale 205 ms at 982 PVA samples, before 1800-sample trigger | injection not armed |
| Core-pause attempt #2 | `044656-19024` | monitor saw 400 PVA samples but PID selector was ambiguous; no signal sent | injection not applied |
| Core-pause attempt #3 | `045631-22674` | monitor reached 400 after Core had already exited; spontaneous odometry receive-stale 209 ms later in run | injection not applied |
| repeated BACKUP attempt #1 | `051508-31507` | runner rejected stale authoritative build fingerprint after docs changed | no scenario evidence |

## Focused fault runs from a revalidated build

| Fault | Raw session suffix | Trigger and actual observation | Outcome and limit |
| --- | --- | --- | --- |
| Core heartbeat pause | `050320-27474` | Exact Core PID 30004 received SIGSTOP for **350.190 ms** after 400 PVA samples. Adapter logged `planner backend PVA command stale` and requested PX4 Hold; report observed native Hold. | `PAUSED_SAFETY_STOP`, accepted `[0]`. Mapping also replaced 3 unconsumed clouds, so this is focused lease-fence observation, not fully isolated fault qualification. |
| Repeated replacement failure | `051755-34772` | 55 diagnostic `kFailed` conversions; 472 command samples with `ROLE_BACKUP` (181 READY, 291 COMPLETED). BACKUP→MAIN switches for requests 2/3/4 had nearest propagated measured speeds **0.064/0.063/0.058 m/s** at source offsets 8/0/4 ms. | Mission `COMPLETE`, accepted `[0,1,2,3,4]`, no unexpected Hold request; adapter admitted 3572 samples, max inter-admission gap 40.042 ms, three typed `NOT_YET_VALID` retains. This is BACKUP/restart evidence, not `OptimizationFailed` evidence. |
| Exact optimization failure | `052428-38549` | `FAULT_INJECTION_ARMED` and `APPLIED` once at planning cycle 148: original `kSuccess` (0) → exact `kOptimizationFailed` (6), desired request 3, active predecessor request 2/generation 4. Runtime classifier disposition 4 (`RetainCommittedCommand`), retained validator disposition 8 (`CertifiedCommandPreserved`), command available after decision. | Mission `COMPLETE`, accepted `[0,1,2,3,4]`, no unexpected Hold request. Nine predecessor admissions followed fault; first successor admission 148 ms after fault. All 3307 command samples admitted; maximum adapter gap 21.199 ms. This is one focused regression run, not a repeat of the earlier three-run exact-status qualification-style experiment. |

STOP evidence: N1's terminal waypoint 4 acceptance records measured speed **0.104753 m/s** and measured acceptance position error **0.236239 m**. This is the evaluator's observed acceptance event; it is not inferred from the analytic trajectory endpoint.

No fault attempt is silently counted as nominal PASS. The earlier exact-status evidence branch has its own 3/3 injected-positive denominator; this branch ran one wire-schema regression of that seam.
