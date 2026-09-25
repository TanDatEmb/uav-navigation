# Base provenance

| Item | Pinned value |
| --- | --- |
| Branch | `codex/software-qualification-witness-closure-20260925` |
| Base commit | `c422b8485a372e5b3a792682ef0773784b6cf1c4` |
| Base tree | `939d4334b63570713f51d02c6eb29dc44c37b327` |
| Product behavior base | `748b8e3924a0042b975382468d675df4c987a3c2` |
| PX4 checkout | `deaff86ee335dd697677bcfc2415a23878e1b895` |
| PX4 binary SHA256 | `b69d990830625c54de458282ff109286c5a35597680675bcad0a163ea8640edc` |
| px4_msgs submodule | `86d8239e962f6939e05c3737784f60c02fa884db` |
| px4_ros2_interface_lib submodule | `4a3370f084ac6f1ef001a4afa2b007845ffd0837` |

The source worktree was clean before edits. The PX4 checkout had pre-existing modifications to `dds_topics.yaml` and `zenoh-pico`, plus untracked vendor/board paths. They were not reset or modified by this branch. The binary hash, not the checkout cleanliness, identifies the SITL executable. The fresh cohorts were captured at clean behavior-bearing source commits `5a77f5e06697f50643e918cb16a1e81b90696acc` (primary) and `61ee1e503a520491b62c38ba312b877cc4ae00d4` (verification). Their raw files and hashes are in `RAW_EVIDENCE_MANIFEST.csv`.

Offline reevaluation used evaluator commit `db43b6086fb043628c08edfe6310027c027741ba`; original raw `report.json` files were not overwritten. The final documentation commit is recorded in delivery, separately from capture and behavior-bearing commits.
