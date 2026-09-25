# Pre-main baseline provenance

Campaign base SHA: `0b477638d21ce60cdb42ed85fb7c2d568bf500ed`
Base tree: `7393f3e63048abc45c0eeb89b1023593bbeda60e`
Branch: `codex/pre-main-consolidated-regression-20260925`
Isolated worktree: `/home/letandat/Dev/uav-navigation-pre-main-consolidated-regression-20260925`
Initial source worktree status: clean.
Historical `origin/main` at campaign start: `287b7b84cf4311e31d52ca04796223cc4efc1bc5`.

Submodules at campaign start:

| Path | SHA |
| --- | --- |
| `src/external/px4_msgs` | `86d8239e962f6939e05c3737784f60c02fa884db` |
| `src/external/px4_ros2_interface_lib` | `4a3370f084ac6f1ef001a4afa2b007845ffd0837` |

PX4 checkout SHA: `deaff86ee335dd697677bcfc2415a23878e1b895`
PX4 binary SHA256: `b69d990830625c54de458282ff109286c5a35597680675bcad0a163ea8640edc`.

The PX4 checkout was dirty at baseline. Existing dirty state was left untouched:
modified `Tools/microxrcedds_client/dds_topics.yaml`, modified `src/modules/uxrce_dds_client/zenoh-pico` submodule pointer, and untracked VOXL2, rl_tools, mc_raptor, Gazebo optical-flow, and Micro-XRCE-DDS-Client-v3 trees under the PX4 checkout. These external changes are not included in this repository branch.

C0-SW policy: `C0_SW_V1`, source policy artifact at `artifacts/software_qualification_witness/20260924T220745Z-c422b848/C0_SW_POLICY.md`. It requires software-owned authority/lifecycle/evidence contracts and defers absolute tracking, motion quality, PX4 physical tracking, and terminal physical stopping to C0-IFP.

Incoming World milestone: `WORLD_TEMPORAL_CONTRACT_HARDENED` at `0b477638d21ce60cdb42ed85fb7c2d568bf500ed`. Frozen regression scope includes R1–R4, isolated mapping-only stale/recovery, exact suspension, no stale-certified command after suspension, fresh-world recertification/resume, and unsafe-world no-resume. This campaign treats those as accepted baseline facts and will run only the requested compact regression subset.

Initial `git status --porcelain=v2` in the isolated repository worktree was empty. Campaign edits are recorded separately in the final evidence index.
