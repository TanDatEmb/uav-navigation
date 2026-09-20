# Unresolved scope and reproduction limits

- Product profile for a running deployment was not available. `deployment_profile: sitl` in source YAML and `use_sim_time` launch default are not proof of effective live parameters.
- Compile database `build/compile_commands.json` contains no first-party runtime translation units; `build/CMakeCache.txt` is absent. AST/member-reference coverage is unavailable. Manual writer/reader lists are explicitly selected-path, not exhaustive.
- The repo includes dirty local edits at baseline; source snapshot covers them byte-for-byte, but does not attribute each line of pre-existing work to its author.
- External submodules are recorded by revision, not snapshotted. Any generated ROS code/output or dependency actually linked by the current profile cannot be tied to the snapshot. See [provenance](../baseline/provenance.md).
- No build, unit test, launch, SITL, runtime trace, PX4 command receiver acknowledgment or flight operation was performed. Test code is source-read only.
- Mermaid CLI is unavailable. Sequence `.mmd` source exists; no Mermaid SVG or visual QA is claimed.
- Seven Graphviz SVGs were rendered and checked. The final `PARTIAL_AS_IS` status reflects the remaining source writer/profile/runtime gaps.
