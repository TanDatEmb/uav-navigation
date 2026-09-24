# Base provenance

- BASE_SHA: `2b9279b42f3dce0d2565e4ef008852b018baf99f`.
- BASE_TREE: `3e233faec6d5c5886560a40f03ecfdbae5f2cc72`.
- Source branch: `codex/evidence-exact-planner-failure-status-20260924`; behavior source `47a5c05e9b36feb558e5e2ecd3646cc36157613d`.
- Original checkout `/home/letandat/Dev/uav-navigation`: clean `git status --porcelain=v2`, HEAD `7da3e97cb399c2e39d62cfe60213a45e8a92300e`, left untouched.
- Isolated worktree `/home/letandat/Dev/uav-navigation-adapter-admission-contract-20260924`: branch `codex/refactor-adapter-admission-control-contract-20260924`, created directly from BASE_SHA; clean before edits.
- Submodules: `px4_msgs=86d8239e962f6939e05c3737784f60c02fa884db`; `px4_ros2_interface_lib=4a3370f084ac6f1ef001a4afa2b007845ffd0837`, clean.
- External PX4 checkout `/home/letandat/Dev/Autopilot`: HEAD `deaff86ee335dd697677bcfc2415a23878e1b895`, dirty before work: modified `src/modules/uxrce_dds_client/dds_topics.yaml`, dirty `src/modules/zenoh/zenoh-pico`, unrelated untracked directories. It will not be cleaned/reset/stashed.
- External PX4 binary SHA256 at branch creation: `b69d990830625c54de458282ff109286c5a35597680675bcad0a163ea8640edc`.
- `docs/safety/runtime_safety_current.md` read before edits; targeted archived DEC-20260904-001/002 examined for diagnostic command evidence semantics. No gate/threshold change intended.
