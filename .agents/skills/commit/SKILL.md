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
- **scope:** derive it from the changed path. For `src/<area>/uavnav_<name>/`
  use `<name>` (`core`, `interfaces`, `lio`, `world`, `planning`,
  `supervisor`, `navigation`, `px4-bridge`, `px4-mode`, `bringup`).
  `tools/uavnav/` maps to `tools`, and `docs/` to `design`. Check recent
  `git log --oneline` for scope names already in use and reuse them.
- **description:** imperative, concise, >=5 chars.

## Steps

1. `git branch --show-current`. On `main`, create `<username>/<description>`
   (`gh api user --jq .login`).
2. `git status` and `git diff --staged`. If nothing is staged, ask what to stage.
3. **Split by intent** (AGENTS.md): behavior changes, observability, and
   refactors go in separate commits. If the staged diff mixes them, stop and
   propose a split.
4. Name the spec section (`§`) that the change implements in the body. If the
   change touches an interface, state, owner, thread, timing or safety
   behaviour, run the `design-check` skill first.
5. Body only when needed, and it explains **why**. Any temporary shortcut must
   be named in the body and listed in `docs/architecture/DECISIONS.md`.
6. Never claim qualification/PASS in a message without declared evidence.
7. Commit with the attribution trailer given by the session/user config. Add
   new commits instead of amending pushed ones; never force-push unasked.
8. If an upstream exists (`git rev-parse --abbrev-ref @{u}`), push.
