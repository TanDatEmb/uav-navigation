# Base provenance

- Branch: `codex/repair-failclosed-ownership-fencing-20260924`, created in a separate worktree directly from `49e9c0e8c56f39b80e5ad959999ccbd358c1ca64`.
- Base tree: `632b3e56501ab1d063846b5d57e69d0668f3181b`.
- Before edits, the source checkout at `/home/letandat/Dev/uav-navigation` was clean (`git status --porcelain=v2` empty) and remained on `codex/close-proven-findings` at `7da3e97c`. This worktree was also clean after creation and submodule initialization. No actor state was reset, cleaned, stashed, or rebased.
- Pinned submodules: `px4_msgs` `86d8239e962f6939e05c3737784f60c02fa884db`; `px4_ros2_interface_lib` `4a3370f084ac6f1ef001a4afa2b007845ffd0837`.
- External PX4 checkout `/home/letandat/Dev/Autopilot`: HEAD `deaff86ee335dd697677bcfc2415a23878e1b895`, dirty before this task (`dds_topics.yaml`, zenoh submodule and listed untracked paths). It is not modified by this repair.
- PX4 SITL binary `/home/letandat/Dev/Autopilot/build/px4_sitl_default/bin/px4`: SHA256 `e440bd77fbacdc422eaafdb168fec01554298d545f11e2b004a640b04e324ff9`.
- The full `docs/safety/runtime_safety_current.md` was read before product edits. Targeted history: HG-023 at archive line 3384 retains a certified command through failed replacement solves only while latest-world, anchor, suffix, identity and lease checks pass; on failure it still brakes or fails closed. No threshold or bypass change is authorized here.
