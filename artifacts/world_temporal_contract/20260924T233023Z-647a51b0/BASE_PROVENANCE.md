# Base provenance

- Branch: `codex/world-temporal-contract-hardening-20260925`
- Base/evidence-capable SHA: `647a51b060a6b2d0f66b6eb3c69dd990fd20986a`
- Base tree: `f615b2e518543748e990ecf98232faf2a7b08a88`
- Product behavior reference: `748b8e3924a0042b975382468d675df4c987a3c2`
- C0-SW policy: `C0_SW_V1`
- PX4 checkout SHA: `deaff86ee335dd697677bcfc2415a23878e1b895`
- PX4 binary SHA256: `b69d990830625c54de458282ff109286c5a35597680675bcad0a163ea8640edc`
- Worktree: `/home/letandat/Dev/uav-navigation-world-temporal-20260925`
- Initial repository status: clean (`git status --porcelain=v2` empty).
- At worktree creation both pinned submodules were uninitialized. They were initialized in this isolated worktree only (no checkout movement): `px4_msgs` `86d8239e962f6939e05c3737784f60c02fa884db`; `px4_ros2_interface_lib` `4a3370f084ac6f1ef001a4afa2b007845ffd0837`. Final `git submodule status --recursive` reports those exact SHAs with no `-`, `+`, or `U` marker.
- External PX4 checkout was already dirty before this worktree was created: modified `dds_topics.yaml`, modified `zenoh-pico` gitlink and untracked vendor/board paths. This work did not modify it.

Incoming evidence baseline reported by the user's pinned delivery: fresh primary cohort 5/5 complete and C0-SW eligible/PASS; required lifecycle unresolved/conflicts and required reference missing/conflicts all zero; writer drops/errors zero. This is inherited evidence, not validation of this branch.
