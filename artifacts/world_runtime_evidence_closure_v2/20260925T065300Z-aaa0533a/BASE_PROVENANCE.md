# Base provenance

## Branch and source

- Branch: `codex/world-runtime-evidence-closure-v2-20260925`
- Base SHA: `aaa0533a6b747432a501f17c49d5229118055f7a`
- Base tree: `a062254e163ddc5b09ac1f06dff68aaff6930792`
- Product source is the canonical C1/C2 convergence repair. C2 is closed at the base; no World behavior changes are part of that claim.
- Initial worktree was clean. The final candidate adds only deterministic world execution-authority tests and this evidence set.
- Submodules: `px4_msgs=86d8239e962f6939e05c3737784f60c02fa884db`; `px4_ros2_interface_lib=4a3370f084ac6f1ef001a4afa2b007845ffd0837`.

## PX4 provenance

- Checkout SHA: `deaff86ee335dd697677bcfc2415a23878e1b895`.
- Checkout is dirty in previously present paths: `src/modules/uxrce_dds_client/dds_topics.yaml`, submodule `src/modules/zenoh/zenoh-pico` at `6ec4dc1995f185330e4f6faa8c719eb86f180deb`, and untracked upstream directories `boards/modalai/voxl2/src/lib/`, `src/lib/rl_tools/`, `src/modules/mc_raptor/`, `src/modules/simulation/gz_plugins/optical_flow/PX4-OpticalFlow/`, `src/modules/uxrce_dds_client/Micro-XRCE-DDS-Client-v3/`.
- PX4 binary: `/home/letandat/Dev/Autopilot/build/px4_sitl_default/bin/px4`.
- SHA256: `b69d990830625c54de458282ff109286c5a35597680675bcad0a163ea8640edc`.
- These external checkout changes were preserved and not edited.

## Evidence boundary

This artifact extends source/component race coverage. It does not contain a SITL run, raw runtime session, C0-SW world-transaction events, or flight/HITL evidence.
