# Base provenance

- Canonical base branch/SHA: `origin/main` / `7e0b8508781f68ecbd18d15d40129e019108f3e7`.
- Base tree: `93aef75abd2079ee11c6989919127623a81c76e0`.
- Repair branch/worktree: `feat/px4-authority-emergency` at `/home/letandat/Dev/uav-navigation-px4-authority-emergency`.
- Worktree initial `git status --porcelain=v2`: clean after initializing pinned submodules.
- `px4_msgs`: `86d8239e962f6939e05c3737784f60c02fa884db`.
- `px4_ros2_interface_lib`: `4a3370f084ac6f1ef001a4afa2b007845ffd0837`.
- External PX4 checkout: `/home/letandat/Dev/Autopilot`, HEAD `deaff86ee335dd697677bcfc2415a23878e1b895`.
- PX4 SITL binary: `/home/letandat/Dev/Autopilot/build/px4_sitl_default/bin/px4`, SHA-256 `b69d990830625c54de458282ff109286c5a35597680675bcad0a163ea8640edc`.
- External checkout was already dirty before this work. It has a tracked edit in `src/modules/uxrce_dds_client/dds_topics.yaml`, a dirty `src/modules/zenoh/zenoh-pico` submodule at `6ec4dc1995f185330e4f6faa8c719eb86f180deb`, and untracked `boards/modalai/voxl2/src/lib/`, `src/lib/rl_tools/`, `src/modules/mc_raptor/`, `src/modules/simulation/gz_plugins/optical_flow/PX4-OpticalFlow/`, and `src/modules/uxrce_dds_client/Micro-XRCE-DDS-Client-v3/`. No PX4 checkout files were changed by this campaign.
- The original `/home/letandat/Dev/uav-navigation` checkout was on the same base SHA but contained uncommitted edits in `Makefile`, `README.md`, `docs/runtime_validation.md`, `docs/safety/runtime_safety_current.md`, `tools/runtime/runner.py`, and `tools/runtime/tests/test_runtime_contract.py`. Those were left untouched; this branch uses a separate worktree.
