# Base provenance

- Repair branch: `codex/repair-convergence-braking-request-evidence-20260925`
- Base SHA: `770699c2271dbc029be183f2affa5916909e3fa1`
- Base tree: `e19b69810faca638c03121c61d2d4e0dbb06226d`
- Audit branch: `codex/world-runtime-evidence-closure-20260925`, pushed to `origin`; local and remote both verified at `770699c2271dbc029be183f2affa5916909e3fa1` before branch creation.
- Repair commits: `c1464a975c59b380b4a420965807576248075017`, `c464f671d94a722b90989108c2d62f991d58c4ce`, `68289532b93641973238329d0ec6c4b0626b43f9`.
- Product source HEAD at final validation: `68289532b93641973238329d0ec6c4b0626b43f9`; repository status was clean before documentation artifacts were created.
- Submodules: `px4_msgs=86d8239e962f6939e05c3737784f60c02fa884db`; `px4_ros2_interface_lib=4a3370f084ac6f1ef001a4afa2b007845ffd0837`.
- PX4 checkout: `/home/letandat/Dev/Autopilot`, SHA `deaff86ee335dd697677bcfc2415a23878e1b895`. Existing dirty changes were observed and preserved.
- PX4 binary SHA256: `b69d990830625c54de458282ff109286c5a35597680675bcad0a163ea8640edc`.
- Audit worktree status before branch: clean. Repair worktree status before edits: clean.
- Authoritative Release manifest records source `git_head=68289532...`; manifest SHA256 `542d33801c698369c708d624f4a97e6a4fa8525d2bc295cb26f578cb9617a353`.
- Baseline runtime sessions were copied from the isolated exact-base worktree and verified with `baseline-runtime/RAW_SHA256_MANIFEST.json`. The disposable 4.1 GB comparison worktree was then removed; PX4 build and its hash were untouched.
