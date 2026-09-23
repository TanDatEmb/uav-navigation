# Decisions and evidence still required

1. **Instrumentation experiment ownership:** approve a separate diagnostic-only branch implementing signal groups I-O1 through I-O4, with bounded queue/drop accounting and performance gate from `MINIMAL_INSTRUMENTATION_SPEC.md`. Product source/config remains unchanged on this audit branch. One focused external O3 topic capture succeeded; other Hold permutations still need controlled runs.
2. **PASS retention policy:** decide only after exact MissionController callback-order distributions distinguish in-ball re-entry from a crossing that departed before continuation; do not use the one 1.240 s sample as a threshold.
3. **Hold retry/status protocol:** owner must define fresh post-request status attribution, pre-existing Loiter, pilot/failsafe takeover and missing callback/status escalation. Pin a reproducible product firmware baseline separately; the run binary identity is not that baseline.
4. **World/lease policy:** determine whether suspended publication can legally resume after adapter handover, whether runtime requires a downstream authority observation, and whether a continuous explicit heartbeat is feasible after paired timing and deterministic stale-world repeats.
5. **E2 public API:** owner decides whether installed `onTrajectory` is compatibility-only/deprecated or a required future recoverable-braking capability. Current native shadow semantics need not wait for this external API decision.

No implementation/authority migration is recommended until the observability experiment closes the missing event stream. No safety gate, runtime budget or threshold change is justified by these diagnostic samples.
