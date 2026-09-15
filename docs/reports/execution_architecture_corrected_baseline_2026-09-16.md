# Execution architecture corrected baseline — 2026-09-16

## Verdict

The corrected source baseline is
`e91c56151f8ea5b5f459658e0fef7831941f0c78` on
`codex/runtime-evidence-for-analysis`.

The evidence supports a bounded ownership completion, not a new all-purpose
`ExecutionCoordinator`:

- `ExecutionTimelineStore` remains the canonical active/pending pointer and
  activation owner.
- `ExecutionEpisode` is now the single mutable physical-policy record for
  lifecycle, recovery, failure, suffix and restart state.
- `NavigationRuntimeNode` still owns the serialized cross-store transaction
  boundary. Moving those transactions behind another class is not justified
  until source or timing evidence shows coupling that the current boundary
  cannot contain.
- Planner history is a performance cache, not a second command authority.
- Public certificate booleans are design debt, but no product construction
  bypass was found on this baseline. A typed wrapper without an immutable
  trajectory/world/config witness would be nominal indirection rather than a
  stronger proof.

This is a code/component baseline only. It is not SITL qualification, C0
qualification, hardware evidence, or a statement that the branch is ready to
merge to `main`.

## Reproducible identity

| Field | Value |
|---|---|
| Navigation SHA | `e91c56151f8ea5b5f459658e0fef7831941f0c78` |
| Commit time | `2026-09-16T00:38:36+07:00` |
| Commit subject | `refactor(runtime): unify execution policy ownership` |
| Remote branch | `origin/codex/runtime-evidence-for-analysis` at the same SHA |
| Merge base with `main` | `ab739f61fb581e32deb72efeb0cf58e789fa198c` |
| Branch relation to `main` | `0` behind, `722` ahead |
| Build manifest | schema 1, authoritative, Release, clean source |
| Source fingerprint | `8d7eaea20058a08d0da316fdfd1d526913c3d468bf541dbd3e1e23efef7b5bdc` |
| `px4_msgs` submodule | `86d8239e962f6939e05c3737784f60c02fa884db`, clean |
| PX4 ROS 2 interface submodule | `4a3370f084ac6f1ef001a4afa2b007845ffd0837`, clean |

The external PX4 checkout used by the runtime is not part of the clean
navigation manifest. `/home/letandat/Dev/Autopilot` is at
`deaff86ee335dd697677bcfc2415a23878e1b895` on
`fix/lockstep-scheduler-timeout-race` and is dirty, including modified and
untracked dependency content. Any SITL run using that checkout must be marked
diagnostic and is not eligible for the corrected-baseline comparison until PX4
provenance is frozen or explicitly captured as a separate controlled input.

## Finding disposition

The table applies the two independent axes requested by the review plan:
evidence status and priority. `FIXED` below means the source defect has a
focused regression and aggregate build/test evidence; it does not imply flight
qualification.

| Claim under review | Evidence status | Priority | Current conclusion |
|---|---|---:|---|
| `reserveAnchor()` evaluates under the store mutex and can terminate through a `noexcept` boundary | `CONFIRMED`, `FIXED` | P1 | The store now reserves an immutable predecessor/lineage token, evaluates outside the mutex, and rechecks lineage before accepting. Evaluator exceptions fail closed. No production throw-frequency claim is made. |
| Evaluator exceptions are reachable in normal production | `CONDITIONAL` | P2 | Product construction paths use the planner evaluator and no intentionally throwing production backend was found. The API resilience defect was still worth closing because the callable type allowed exceptions and allocation/backend faults were not represented as a result. |
| Freshness was decided too early for command exposure | `CONFIRMED`, `FIXED` | P1 | Final state/world/command/bundle freshness is evaluated inside the exact timeline publication transaction. A delayed command is dropped rather than authorized from the callback-start decision. |
| Multiple mutexes prove the runtime is unsuitable for real time | `EVIDENCE_GAP` | P1 pending data | No reachable lock cycle was established in the reviewed execution paths. Lock wait/hold and command publication timing are instrumented, but R5 load evidence is still required; thread count or mutex count alone is not a verdict. |
| A large runtime file proves the architecture is wrong | `DESIGN_DEBT` | P2 | Size is not the invariant. The confirmed duplicate policy owner was removed without wrapping the existing timeline store. Further extraction needs change-impact or timing evidence. |
| Certificate booleans allow an active product bypass | `DESIGN_DEBT`; bypass claim `REJECTED` | P2 | Product candidate construction remains behind planner validation. Tests can forge fields because the data type is public. Introduce a typed candidate only with a witness bound to exact trajectory, world, limits/model/config and validity interval. |
| Solve budget greater than replan period is itself a defect | `REJECTED` | — | Runtime carries one absolute steady-clock deadline into planner core; the worker owns one active and one bounded pending job. Required evidence is request-to-stage age and deadline misses, not the ratio of two configured periods. |
| Missing GitHub check-runs proves algorithmic quality is low | `EVIDENCE_GAP` | P2 | Local reproducible build/test evidence exists, but CI provenance remains an assurance question. It is not evidence of an algorithmic failure. |
| PX4 Hold is a proof of collision-free stopping | `REJECTED` | P0 contract | Hold is containment/control handover only. Collision-free stopping still requires a certified suffix or measured-state braking evidence within known-free space and the tracking/localization/mapping margins. |
| Two superseding unactivated proposals may share an identity | `CONFIRMED`, `FIXED` | P1 | A pre-fix regression produced generation 1 for both proposals and let validation requested for A inspect B. Every staged proposal now reserves a unique monotonic generation; stale validation cannot alias a superseding proposal. |
| Runtime has two mutable physical execution-policy owners | `CONFIRMED`, `FIXED` | P1 | The standalone recovery atomic was removed. Goal begin/clear, fail-closed, commit and sampled safety-role transitions now update one `ExecutionEpisode` snapshot. PX4 Hold cannot be resurrected by a late commit observation. |

## Architecture decision at this baseline

The selected option is **A plus a bounded part of B**:

1. Keep the existing package/process layout and the canonical timeline store.
2. Fix confirmed boundary defects independently.
3. Complete physical-policy ownership inside `ExecutionEpisode`.
4. Do not add a coordinator wrapper until at least one mutation bypass remains
   or a measured coupling problem requires a new seam.
5. Do not replace planner cache notifications with a larger request protocol:
   `PlanningHistory` currently carries only generation and velocity and cannot
   replace the backend polynomial warm-start context without losing behavior.
6. Do not introduce `PlanProposal`/`CertifiedCandidate` names alone. The next
   certificate type must make the immutable witness unforgeable or it does not
   close the design debt.

This decision preserves one implementation path to PX4. There is no shadow
coordinator, alternate publisher, or fallback-only execution authority.

## Work-package status

| Package | Status | Evidence or boundary |
|---|---|---|
| W0 baseline/evidence closure | Complete at source/component level | Fail-closed capture contract, provenance and manifest checks are present. Diagnostic remains distinct from qualification. |
| W1 authority contract tests | Complete for confirmed boundary findings | Controlled interleavings cover stale anchor lineage, delayed exposure, proposal supersession, recovery serialization and fail-closed non-resurrection. This is not an exhaustive flight scheduler proof. |
| W2 boundary fixes | Complete for confirmed findings | Anchor evaluation, exception containment, exposure-time freshness and candidate-generation identity are fixed in separate commits. |
| W3 execution ownership seam | Complete, deliberately smaller than proposed | Recovery joined the existing lifecycle record. No `ExecutionCoordinator` class was added. |
| W4 planner request completion | Not justified on current evidence | Backend history is a private warm-start cache and does not issue commands. Revisit only if an authority-relevant hidden setter or cache-dependent correctness failure is reproduced. |
| W5 certified candidate boundary | Deferred design debt | No product bypass found. Requires witness design before code. |
| W6 world/execution transaction | Implemented, component verified | World publication finalizes against exact timeline version/pointers; proposal generations no longer alias. Integrated fault evidence remains open. |
| W7 runtime reduction | Partially complete | One duplicated owner removed. Remaining latches must be audited individually; line-count reduction is not an acceptance target. |
| W8 execution-path isolation | Measurement gate open | Do not change executor/callback topology before R5 identifies a tail-latency owner. |
| W9 operational/CI gates | Partially complete | Local manifest/evaluator fail closed. CI provenance and a controlled PX4 input remain open. |
| W10 C0 regression/qualification | Not started on this baseline | Blocked by external PX4 provenance and the required repeated matrix. |
| W11 planner improvements | Out of the architecture migration | Only start from an R3 numerical failure class with separate evidence and thresholds. |

## Verification evidence

For the source content committed as `e91c5615`, the affected and aggregate
checks passed before the commit was created:

- authoritative Release build: 23 packages;
- current product selection: 81 CTest entries, 0 errors, 0 failures, 0
  skipped;
- the raw result tree also contained two passing entries retained from older
  `navigation_mission` and `uav_description` executions; they are not credited
  to this baseline run;
- Python runtime/tool contracts: 370 passed;
- `ExecutionEpisode`: 9/9, including generation-bound safety-role alignment,
  atomic commit/recovery state and fail-closed non-resurrection;
- full `navigation_runtime` CTest package: 12/12 executables;
- `git diff --check`: pass.

After commit and push, the canonical build was rerun and produced the clean
manifest recorded above. These results prove compilation and tested contract
behavior only. They do not measure command jitter, DDS delivery, PX4 receive
age, tracking error, clearance or braking distance.

## Next evidence gate

The next architecture-affecting action is R5 measurement, not another class
extraction. Freeze or explicitly control the external PX4 tree first. Then run
matched diagnostic cases with identical navigation SHA, PX4 SHA/diff, mission,
scene, seed and speed. Record at minimum:

- command source, decision, authorization, publish and PX4 receive/use age;
- command gap and deadline misses;
- planning scheduled/submitted/started/finished/staged/activated identities;
- queue depth, cancellation latency and lock wait/hold;
- map update/export/recertification latency and capture loss;
- failure denominator, including runs with no executable bundle.

The requested 5 m/s three-pillar 2-waypoint, 5-waypoint and 9-waypoint runs
should remain diagnostic until the PX4 input is controlled. Three repetitions
per case can expose regressions and support bottleneck attribution, but they do
not satisfy the locked C0 requirement of ten consecutive deterministic runs at
each qualified speed or the separate clutter/dataset/fault matrix.
