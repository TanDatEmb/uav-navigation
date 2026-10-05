---
name: pr
description: Create a pull request with conventional title and a short evidence-based description
argument-hint: "[optional: target branch or description]"
allowed-tools: Bash Read Glob Grep
---

# Pull Request

## Steps

1. Branch check; on `main` create `<username>/<description>`.
2. Context: `git status`, `git log --oneline main..HEAD`,
   `git diff main...HEAD --stat`.
3. Run the `build-and-test` skill at the scope the diff can reach. Skip for
   docs-only diffs. Fix failures before opening the PR.
4. **Title:** `type(scope): description`, under 72 chars.
5. **Body**, as short as possible, in order: `## Summary`, `## Problem`,
   `## Solution`, one or two sentences each. No file lists or code snippets.
   `fixes #<N>` first line of Summary when closing an issue.
6. `## Safety impact` (required when the diff touches estimation, mapping,
   planning, control, PX4 integration, budgets, gates, or thresholds): which
   invariants in `docs/safety/runtime_safety_current.md` are affected, and the
   ledger entry for any bypass/relaxed gate (owner, scope, evidence, removal
   condition, verification command).
7. `## Testing` only for real evidence: SITL, dataset replay, or recorded
   sensor data (targets: see `make help`). Report the distribution
   and run count, not a single run. Unit tests and builds are not testing here.
   Never report testing that did not happen; ask the user.
8. `git push -u origin <branch>`, `gh pr create` (base `main`), return the URL.
   End the body with the PR attribution line given by the session/user config.
