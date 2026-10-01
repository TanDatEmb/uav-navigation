# W3-A3 — Open questions / blocker

## B-01 — P1A salvage patch does not apply to the requested base

- **Status:** BLOCKED; step 1 stopped as required by `W3-A3_salvage_P1A.md`.
- **Requested base:** `origin/main` at `24ec0fc8718bb8606e4e9e4a7eaa84773d384854`.
- **Source check:** the same patch checks clean on the owner workspace at `432dc94630fbc76ca670138228f2f616f6840bb0`; this is the provenance claimed by the salvage README/prompt, not evidence that it applies to the requested base.
- **Observed command:** `git apply --check docs/refactor/wave3/salvage/P1A_legacy_cd7a7d8a.patch`.
- **Observed failures:** hunks do not apply in `src/px4/px4_navigation_external_mode/CMakeLists.txt`, `src/px4/px4_navigation_external_mode/package.xml`, and `src/runtime/navigation_runtime/src/navigation_runtime_node.cpp`.
- **Safety/process impact:** no `--3way`, manual conflict resolution, guessed source merge, or runtime change was performed. Steps 2–4 and cleanup deletions remain unexecuted.

### Decision needed from coordinator / architect

1. Provide a P1A patch regenerated or explicitly rebased for `24ec0fc...`, or authorize a documented base change to `432dc94...`.
2. Confirm that the replay remains additive and that the existing W3-A2/P0.3 changes are the intended context for the witness hunks.
3. After an applicable source is provided, resume steps 1–4 and run the required static/python/ros gates. SITL step 5 remains gated on C1 and must use matched C1 scene/speed/tracking-off runs.

## Deferred evidence

- A6 oracle, witness lines, `mismatches=[]`, and ROS reverse-dependency results: **NOT_MEASURED** because step 1 was blocked.
- SITL step 5: **NOT_RUN / WAITING_FOR_C1**; no verdict is assigned by this branch.
