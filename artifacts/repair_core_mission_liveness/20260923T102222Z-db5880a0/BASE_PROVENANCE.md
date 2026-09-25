# Frozen starting provenance

- Repair branch: `codex/repair-core-mission-execution-liveness-20260923`, created directly from `db5880a05ffbb82f3f528bc18934af0cb0390bab`.
- BASE_SHA: `db5880a05ffbb82f3f528bc18934af0cb0390bab`.
- BASE_TREE: `a125d57518e00c07a14390e9810102f7c22453cc`.
- `git status --porcelain=v2` before edits: empty in the failed migration worktree, original baseline worktree, and new repair worktree.
- Pinned submodule gitlinks and checked-out SHAs: `px4_msgs=86d8239e962f6939e05c3737784f60c02fa884db`, `px4_ros2_interface_lib=4a3370f084ac6f1ef001a4afa2b007845ffd0837`.
- External PX4 checkout: `/home/letandat/Dev/Autopilot` at `deaff86ee335dd697677bcfc2415a23878e1b895`, with seven pre-existing porcelain status entries. No external PX4 files were edited for this repair.
- PX4 SITL binary: `/home/letandat/Dev/Autopilot/build/px4_sitl_default/bin/px4`, SHA-256 `e440bd77fbacdc422eaafdb168fec01554298d545f11e2b004a640b04e324ff9`.
- The user's artifact pattern contains `db5880a8`; the exact base SHA starts `db5880a0`, which is used in this path to avoid a false source identity.
