# E2 — braking policy decision

**Verdict `E2_UNRESOLVED`; choose `POLICY-D` pending API owner/compatibility decision.** Current **native product** authority is one-way after sampled BACKUP/emergency commitment until measured stop, with fail-closed PX4 Hold if certification cannot be retained. The exported legacy `onTrajectory` API permits `Braking→ExecutingWaypoint` before stop. It was a real historical producer path, later replaced by native command intake, and remains installed/tested. That combination supports neither silent deletion nor unconditional introduction of `RecoverableBraking` into target flight lifecycle.

| Question | Current evidence-based answer |
|---|---|
| Current product runtime policy | Committed BACKUP/emergency is one-way; no native pre-stop BACKUP→MAIN product path found. `FACT_FROM_PINNED_PRODUCT_SOURCE`. |
| `onTrajectory` status | Historical product call chain, now no in-repo product caller; still public/exported and tested. `FACT_FROM_GIT_HISTORY`, `FACT_FROM_PINNED_PRODUCT_SOURCE`. |
| Target recommendation | Preserve `CommittedStopping` for observed native authority. Keep recoverable callback semantics at the API compatibility boundary until its owner explicitly versions/deprecates it; do not infer that the flight execution model needs a recoverable state. `INFERENCE`, conditional. |
| Behavior intentionally lost | None authorized. |
| New failure mode if guessed wrong | Treating committed BACKUP as recoverable may splice a late MAIN without certified continuity; treating the public API as dead may break external integration. |

The unresolved decision is whether policy is **B** (legacy/test-only with formal API disposition) or **C** (recoverable pre-commit flight capability still required). Policy **A** cannot simply equate a public callback with current native authority. If a future pre-commit recoverable phase is required, it needs current measured P/V/A, tracking/lease/world certificate, remaining stopping distance and a continuous admissible splice, plus explicit commitment boundary; a successful callback boolean alone is insufficient. `CandidateRole::Backup` and global lifecycle remain orthogonal.

Required evidence: named owner of exported API and downstream compatibility scan/version policy; focused component integration proving native MAIN/BACKUP/emergency transitions and late-MAIN rejection; flight-level liveness impact of one-way stopping only if policy changes. Existing tests are component evidence; no measured runtime BACKUP→MAIN trace is asserted. This audit changes no behavior.
