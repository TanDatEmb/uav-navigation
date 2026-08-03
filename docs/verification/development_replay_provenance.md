# Development replay provenance policy

`tools/data.py replay` defaults to `development`. It records full workspace
identity, branch, dirty status, porcelain status, tracked/staged diff hashes,
untracked file manifest and hashes, `px4_msgs` submodule identity/state,
binary SHA-256, configuration SHA-256, host, dataset, and replay rate. A dirty
development replay is allowed to execute, but its artifact contains:

```json
{
  "provenance_policy": "development",
  "acceptance_eligible": false,
  "qualification_clean": false
}
```

Use `--require-clean` or the canonical Make target setting
`REPLAY_REQUIRE_CLEAN=1` for qualification. Qualification rejects a dirty
workspace or dirty `src/external/px4_msgs` submodule and rejects a mismatched
`--expected-git-sha`.

Examples:

```bash
python3 tools/data.py replay --dataset aist-mid360-drive
REPLAY_REQUIRE_CLEAN=1 P08_EXPECTED_GIT_SHA=<full-sha> make data-replay DATASET=aist-mid360-drive
```

Canonical P0.8 SITL does not expose a dirty escape. Its orchestrator always
uses the qualification policy; `--allow-dirty` is retained only as a rejected
compatibility option. Set `P08_EXPECTED_GIT_SHA` when a particular clean
candidate must be qualified.
