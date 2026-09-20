# Baseline provenance

The audit snapshot was captured at 2026-09-20 13:02:50 UTC from the local working tree rooted at `/home/letandat/Dev/uav-navigation`, HEAD `9534d8dc15920c8b3e80c8fa12f28ec972b8c6a2`, branch `codex/close-proven-findings`. The captured source is the working tree (HEAD plus staged/unstaged/untracked contents), not a clean commit. The manifest records 32 pre-existing status entries, zero staged bytes, a 1,815,760-byte unstaged diff, and its SHA-256. See `source_manifest.json` and `worktree_status.txt`.

The snapshot preserves relative paths and exact bytes under `source_snapshot/`; `source_manifest.json` records SHA-256, size, line count, and whether each file was tracked or untracked at capture. Two submodules are represented by pinned working revisions but their full source payloads were not copied. Build products, logs, external dependencies and runtime evidence were excluded. The existing compile database is copied and hashed, but contains no product runtime compilation units and no matching `CMakeCache.txt`; no exact product build/profile was established.

`dot` 2.43.0 is available. `mmdc`, `node`, and `npm` are unavailable, so sequence diagrams are retained as Mermaid source but not rendered. `clang++`, `c++`, `cmake`, and Python versions appear in `environment.json`. No tests or builds were run. This is static source reconstruction; no runtime trace or flight behavior was observed.

The initial and final source manifests must agree for source files included in the audit. `tools/verify_snapshot.py` checks that live bytes still match the captured source manifest. A reported mismatch means the affected evidence is no longer verified against the initial snapshot; do not silently refresh the baseline.
