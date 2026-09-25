# Base provenance (captured before product edits)

- Branch: `codex/unify-execution-authority-owner-20260923`
- BASE_SHA: `8ffa14e528f9d28f1a37850eb3a7dbf752f9e90c`
- BASE_TREE: `b4e79585461f5da496160d4aa84e0bbcdd569bce`
- Initial `git status --porcelain=v2`: empty.
- Pinned `src/external/px4_msgs`: `86d8239e962f6939e05c3737784f60c02fa884db`.
- Pinned `src/external/px4_ros2_interface_lib`: `4a3370f084ac6f1ef001a4afa2b007845ffd0837`.
- New worktree submodules were initially unpopulated (`-` in `git submodule status`), then checked out at the pinned SHAs using `git submodule update --init --recursive` before build.
- External PX4 checkout `/home/letandat/Dev/Autopilot`: HEAD `deaff86ee335dd697677bcfc2415a23878e1b895`; pre-existing modified `src/modules/uxrce_dds_client/dds_topics.yaml`, modified submodule worktree `src/modules/zenoh/zenoh-pico`, and untracked paths under `boards/modalai/voxl2/src/lib/`, `src/lib/rl_tools/`, `src/modules/mc_raptor/`, `src/modules/simulation/gz_plugins/optical_flow/PX4-OpticalFlow/`, `src/modules/uxrce_dds_client/Micro-XRCE-DDS-Client-v3/`. No changes were made to this checkout.
- PX4 binary `/home/letandat/Dev/Autopilot/build/px4_sitl_default/bin/px4` SHA-256: `e440bd77fbacdc422eaafdb168fec01554298d545f11e2b004a640b04e324ff9`.
- Capture time: `2026-09-23T12:34:24Z`.
