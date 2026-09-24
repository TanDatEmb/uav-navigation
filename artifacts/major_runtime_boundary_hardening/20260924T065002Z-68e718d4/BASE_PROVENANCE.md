# Base provenance

- Campaign branch: `codex/major-runtime-boundary-hardening-20260924`.
- Base SHA: `68e718d424829201dd53729cab2878348b8722fc`.
- Base tree: `03869e04226ef97268465bd1c0d315698afd3f06`.
- Original worktree before edits: clean `git status --porcelain=v2`.
- `px4_msgs`: `86d8239e962f6939e05c3737784f60c02fa884db`, clean.
- `px4_ros2_interface_lib`: `4a3370f084ac6f1ef001a4afa2b007845ffd0837`, clean.
- PX4 checkout `/home/letandat/Dev/Autopilot`: `deaff86ee335dd697677bcfc2415a23878e1b895`, dirty (modified `dds_topics.yaml`, modified `zenoh-pico` submodule, untracked vendor paths). No PX4 files were edited by this campaign.
- PX4 SITL binary `/home/letandat/Dev/Autopilot/build/px4_sitl_default/bin/px4`: SHA256 `b69d990830625c54de458282ff109286c5a35597680675bcad0a163ea8640edc` at campaign start.
- Source baseline is the adapter admission contract branch; no audit or experiment branch was merged.
