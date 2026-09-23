# Base provenance

- Branch: `codex/refactor-desired-intent-and-legacy-state-20260923`, created directly from `ba9c15a1214ca8ba54ec8f301f1e4056a3322848`.
- Base tree: `6fbd807010aa7a88da247e77cab7a71984b7f33d`.
- Before edits: `git status --porcelain=v2` empty in the isolated worktree. The original checkout was not reset, cleaned, or stashed.
- Pinned submodules: `px4_msgs` `86d8239e962f6939e05c3737784f60c02fa884db`; `px4_ros2_interface_lib` `4a3370f084ac6f1ef001a4afa2b007845ffd0837`.
- External PX4 checkout `/home/letandat/Dev/Autopilot`: HEAD `deaff86ee335dd697677bcfc2415a23878e1b895`, dirty before this cut (`dds_topics.yaml`, zenoh submodule and untracked paths). It is not product source for this refactor and was not modified here.
- PX4 SITL binary `/home/letandat/Dev/Autopilot/build/px4_sitl_default/bin/px4`: SHA-256 `e440bd77fbacdc422eaafdb168fec01554298d545f11e2b004a640b04e324ff9`.
- Clean-base Release build: 23 packages, authoritative manifest at `install/.uav_navigation_build_manifest.json`. Baseline focused CTest package run and Python suite passed; see `TEST_EVIDENCE.md` for final counts.

The current safety contract `docs/safety/runtime_safety_current.md` was read before edits. No runtime budget, safety gate, command lease, world freshness, planner algorithm, mission acceptance, or PX4 boundary policy change is part of this cut.
