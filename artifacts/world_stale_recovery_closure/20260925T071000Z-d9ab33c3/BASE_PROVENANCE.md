# Follow-up base provenance

- Incoming branch pushed: `codex/world-runtime-evidence-closure-v2-20260925`.
- Verified remote SHA: `d9ab33c3e2321ac60794da4128863bc7933458ea`.
- Incoming tree: `5d6bfbdcea29d19d370267cba4667700cee73958`.
- Follow-up branch: `codex/world-stale-recovery-evidence-closure-20260925`.
- Follow-up base SHA/tree: `d9ab33c3e2321ac60794da4128863bc7933458ea` / `5d6bfbdcea29d19d370267cba4667700cee73958`.
- New worktree was clean at creation; submodules: `px4_msgs=86d8239e962f6939e05c3737784f60c02fa884db`, `px4_ros2_interface_lib=4a3370f084ac6f1ef001a4afa2b007845ffd0837`.
- C0-SW policy: `C0_SW_V1`, from `artifacts/software_qualification_witness/20260924T220745Z-c422b848/C0_SW_POLICY.md`.
- PX4 checkout SHA: `deaff86ee335dd697677bcfc2415a23878e1b895`.
- PX4 binary SHA256: `b69d990830625c54de458282ff109286c5a35597680675bcad0a163ea8640edc` at `/home/letandat/Dev/Autopilot/build/px4_sitl_default/bin/px4`.
- PX4 checkout dirty paths are preserved and listed in the incoming artifact `artifacts/world_runtime_evidence_closure_v2/20260925T065300Z-aaa0533a/BASE_PROVENANCE.md`.

## Scope

The incoming World result is partial. R1/R2/R4 barrier tests and R3 runtime-component reset tests pass; isolated World stale/recovery SITL and C0-SW transaction witnesses remain unclosed. This branch is limited to those evidence gaps and will not redesign World, ExecutionAuthority, adapter policy, or thresholds.
