# Base provenance

- Branch: `codex/qualification-gate-recovery-state-transport-20260924` in an isolated worktree.
- Base SHA: `4d184896e6c2680311ef246bd2cde61bc4ee2e88`.
- Base tree: `6fe659d00fac533e5447b0685fa66b9e5d0f75ad`.
- Base `git status --porcelain=v2`: empty in the new worktree before edits.
- `px4_msgs`: `86d8239e962f6939e05c3737784f60c02fa884db`.
- `px4_ros2_interface_lib`: `4a3370f084ac6f1ef001a4afa2b007845ffd0837`.
- PX4 checkout: `/home/letandat/Dev/Autopilot` at
  `deaff86ee335dd697677bcfc2415a23878e1b895`, with preexisting dirty
  `dds_topics.yaml`, `zenoh-pico`, and untracked vendor paths. This campaign
  does not reset or clean it.
- PX4 SITL binary SHA256:
  `b69d990830625c54de458282ff109286c5a35597680675bcad0a163ea8640edc`.
- The three prior raw sessions are under the original checkout's
  `.artifacts/runtime` directory; their hashes are indexed by the base
  campaign's `EVIDENCE_INDEX.md`.
- Natural pilot 3 source SHA: `96d45808a3b4cefcd3df45f06943bb6379af73a0`;
  its raw metadata captured a `VALID` authoritative Release manifest. Ten
  nominal attempts used source SHA `96ed8d089e576c505abdc15500746e9b5be10e50`,
  a single manifest SHA256
  `bd2fa3429742396381c6618291d2f5de6e0057c6c310559c3c40fb8f01cbf037`,
  and the PX4 binary hash above. Later offline analyzer and documentation
  commits do not retroactively alter those captured source identities.
