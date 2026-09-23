# Experiment provenance

- `BASE_SHA`: `7da3e97cb399c2e39d62cfe60213a45e8a92300e`
- `BASE_TREE`: `e7d2b56dbe4353810f029c9c6f0fe8991ebd63eb`
- `git status --porcelain=v2` before edits: empty.
- Branch: `codex/experiment-runtime-observability-instrumentation-20260923`, created directly at `BASE_SHA` in a separate worktree. No audit branch was merged.
- Pinned submodules: `px4_msgs=86d8239e962f6939e05c3737784f60c02fa884db`; `px4_ros2_interface_lib=4a3370f084ac6f1ef001a4afa2b007845ffd0837`.
- Observability audit read from commit `84abeeb2ae379b7483e32520c68043337de71312`, especially `MINIMAL_INSTRUMENTATION_SPEC.md`, `OBSERVABILITY_MATRIX.md`, and `AUDIT_VERDICT.md` in `artifacts/audit_runtime_observability/20260923T031250Z-7da3e97c/`. Audit conclusions are hypotheses/evidence, not the product source of truth.
- Product safety contract read at `docs/safety/runtime_safety_current.md`. No runtime limit, gate, profile or safety document is changed by this experiment.

## Implementation and verification identity

- Experiment source commits before the evidence run: `00dac139` audit sink/schema, `40af85c7` guarded O1–O4 hooks, `55f603e9` input-fault harness/normalizer, `047a1241` adapter library ABI macro propagation, `c6a29b04` event field-presence documentation, and `ab9e6ee1` Jazzy scan-gate lifecycle fix. `9a895658` added preliminary design/provenance docs. The product source baseline remains `7da3e97c`; none of these commits changes a mission/safety threshold or implements a new authority owner.
- OFF and ON Release builds each built 23 packages. OFF: 86 test groups, 0 errors/failures/skips. ON: 87 groups, 0 errors/failures/skips. The added group is audit tooling. The final ON build after the scan-gate fix and a manifest refresh before O1 R09/R10 are indexed separately. The `python3 tools/audit_observability/check_non_authority.py` guard passes.
- A/B/C used the same `long_featured`, nominal, positive scenario, map seed 23, ROS domain 42, XRCE port 8892 and local CPU host. A compiled `NAVIGATION_AUDIT_INSTRUMENTATION=OFF`; B/C compiled `ON`; C also recorded the audit and raw PX4 protocol topics. Build manifests, product reports, audit bags, topic inventories and raw file hashes appear in `EVIDENCE_INDEX.md`.
- The world-gate W1 initial attempt was invalid (Jazzy sidecar parameter initialization); W1-fixed and W2–W5 are the evaluable input-fault episodes. R07/R08 were invalid (source fingerprint changed after documentation was added; runner rejected the old build manifest before simulator start). R09/R10 were run after a fresh ON Release manifest. No invalid run is included in O1/O4 denominators.
- The executed PX4 SITL binary SHA-256 is `e440bd77fbacdc422eaafdb168fec01554298d545f11e2b004a640b04e324ff9`; checkout HEAD was `deaff86ee335dd697677bcfc2415a23878e1b895` but had pre-existing dirt, so the binary is **not** claimed to equal a clean HEAD build. Exact source/recording paths and hashes are indexed. The pinned ROS interface submodule is `4a3370f084ac6f1ef001a4afa2b007845ffd0837`.
- These observations are single-host SITL and component/model evidence. The relaxed tracking experiment and evaluator coverage/reference policy make all nominal mission reports `FAIL`/not qualification eligible. W2–W5 end `BLOCKED` on intentional safety stop. None is flight qualification.
