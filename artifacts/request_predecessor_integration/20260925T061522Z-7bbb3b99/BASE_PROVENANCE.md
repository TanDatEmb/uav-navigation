# Base provenance

- Source branch: `codex/close-request-predecessor-integration-20260925`
- Base SHA: `7bbb3b99e0e75200ff29482e379ffcdbfdcdc7d6`
- Base tree: `0dc8667eb03bc49ffbe139080589cc6c3fe34442`
- Behavior-bearing C2 source commit: `55316c4f` (`fix(planning): bind emergency correction to request predecessor evidence`).
- Prior repair branch: `codex/repair-convergence-braking-request-evidence-20260925`
- Prior repair verdict: `REQUEST_PREDECESSOR_EVIDENCE_BLOCKED`
- C1 status inherited: steady-braking source/component repair closed.
- Worktree at creation: clean; no other worktree was reset, cleaned, or stashed.
- Worktree path: `/home/letandat/Dev/uav-navigation-close-request-predecessor-integration-20260925`
- PX4 SHA: `deaff86ee335dd697677bcfc2415a23878e1b895`
- PX4 binary: `/home/letandat/Dev/Autopilot/build/px4_sitl_default/bin/px4`
- PX4 binary SHA256: `b69d990830625c54de458282ff109286c5a35597680675bcad0a163ea8640edc`
- PX4 pre-existing dirty state: `dds_topics.yaml`, zenoh-pico submodule, and listed untracked upstream files. None were modified by this work.
- `px4_msgs`: `86d8239e962f6939e05c3737784f60c02fa884db`
- `px4_ros2_interface_lib`: `4a3370f084ac6f1ef001a4afa2b007845ffd0837`

The tested source is an overlay built against installed dependencies from the exact base repair worktree. The targeted build is not an authoritative full-workspace Release manifest.
