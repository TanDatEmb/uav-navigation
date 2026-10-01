---
name: commit
description: Create a conventional commit for uav-navigation changes
argument-hint: "[optional: description of changes]"
allowed-tools: Bash Read Glob Grep
---

# Conventional Commit

Format: `type(scope): description`.

- **type:** `feat`, `fix`, `refactor`, `perf`, `docs`, `test`, `build`, `chore`,
  `revert`. Append `!` before `:` for breaking contract changes.
- **scope:** the area from the changed path, derived not memorized: for
  `src/<layer>/<package>/` use the layer or package name (`estimation`,
  `mapping`, `planning`, `execution`, `px4`, `contracts`); `tools/runtime/` ->
  `runtime-tools`; `docs/safety/` -> `safety`. Check recent `git log --oneline`
  for the scope names already in use and reuse them.
- **description:** imperative, concise, >=5 chars.

## Steps

1. `git branch --show-current`. On `main`, create `<username>/<description>`
   (`gh api user --jq .login`).
2. `git status` and `git diff --staged`. If nothing is staged, ask what to stage.
3. **Split by intent** (AGENTS.md): behavior changes, observability, and
   refactors go in separate commits. If the staged diff mixes them, stop and
   propose a split.
4. If the diff touches estimation, mapping, planning, control, PX4 integration,
   runtime budgets, safety gates, or validation thresholds, confirm
   `docs/safety/runtime_safety_current.md` was read and run the
   `safety-ledger-check` skill when any safety doc changed.
5. Body only when needed: explain **why**. Any bypass, relaxed gate, or
   fallback-only path must be named in the body and recorded in the ledger.
6. Never claim qualification/PASS in a message without declared evidence.
7. Commit with the attribution trailer given by the session/user config. Add
   new commits instead of amending pushed ones; never force-push unasked.
8. If an upstream exists (`git rev-parse --abbrev-ref @{u}`), push.
