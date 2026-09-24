# Base provenance

- Base branch: `codex/repair-failclosed-ownership-fencing-20260924`
- BASE_SHA: `b640a25d7caf41059d3bcecd4823ceb3b5790bab`
- BASE_TREE: `5d338bb07d190fc3039c5beb9dec22f07bc6df06`
- Original checkout at branch creation: `/home/letandat/Dev/uav-navigation`, clean `git status --porcelain=v2`, branch `codex/close-proven-findings`, HEAD `7da3e97cb399c2e39d62cfe60213a45e8a92300e`; untouched.
- Isolated worktree before edits: `/home/letandat/Dev/uav-navigation-evidence-exact-optimization-20260924`, branch `codex/evidence-exact-planner-failure-status-20260924`, HEAD `b640a25d7caf41059d3bcecd4823ceb3b5790bab`, clean `git status --porcelain=v2`.
- Submodules: `px4_msgs=86d8239e962f6939e05c3737784f60c02fa884db`; `px4_ros2_interface_lib=4a3370f084ac6f1ef001a4afa2b007845ffd0837`; both clean after recursive initialization.
- External PX4 checkout: `/home/letandat/Dev/Autopilot`, HEAD `deaff86ee335dd697677bcfc2415a23878e1b895`. It was dirty before this work: modified `src/modules/uxrce_dds_client/dds_topics.yaml`, dirty `src/modules/zenoh/zenoh-pico`, and unrelated untracked paths. No PX4 checkout edits made here.
- External PX4 binary: `/home/letandat/Dev/Autopilot/build/px4_sitl_default/bin/px4`, SHA256 `e440bd77fbacdc422eaafdb168fec01554298d545f11e2b004a640b04e324ff9`.
- Safety contract read before changes: `docs/safety/runtime_safety_current.md`; targeted HG-023 history via `docs/safety/runtime_safety_index.md` and `docs/safety/archive/runtime_safety_legacy_full.md`. No gate, threshold, or safety contract change authorized by this branch.
