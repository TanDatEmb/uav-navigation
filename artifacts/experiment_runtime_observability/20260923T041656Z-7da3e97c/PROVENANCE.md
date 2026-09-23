# Experiment provenance

- `BASE_SHA`: `7da3e97cb399c2e39d62cfe60213a45e8a92300e`
- `BASE_TREE`: `e7d2b56dbe4353810f029c9c6f0fe8991ebd63eb`
- `git status --porcelain=v2` before edits: empty.
- Branch: `codex/experiment-runtime-observability-instrumentation-20260923`, created directly at `BASE_SHA` in a separate worktree. No audit branch was merged.
- Pinned submodules: `px4_msgs=86d8239e962f6939e05c3737784f60c02fa884db`; `px4_ros2_interface_lib=4a3370f084ac6f1ef001a4afa2b007845ffd0837`.
- Observability audit read from commit `84abeeb2ae379b7483e32520c68043337de71312`, especially `MINIMAL_INSTRUMENTATION_SPEC.md`, `OBSERVABILITY_MATRIX.md`, and `AUDIT_VERDICT.md` in `artifacts/audit_runtime_observability/20260923T031250Z-7da3e97c/`. Audit conclusions are hypotheses/evidence, not the product source of truth.
- Product safety contract read at `docs/safety/runtime_safety_current.md`. No runtime limit, gate, profile or safety document is changed by this experiment.

Build and runtime provenance are appended only after verification. A test result here is component evidence, never flight qualification.
