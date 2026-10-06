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
6. `## Design` (required for code changes) covers:
   - the spec sections (`docs/architecture/SYSTEM_DESIGN.md` §) implemented;
   - the TRACEABILITY rows updated;
   - any design deviation, with its DECISIONS entry. An undecided deviation
     blocks the PR.
7. `## Testing` lists only evidence that exists:
   - unit tests of state machines and safety decisions;
   - SITL smoke or beta-gate runs, each with its event-log KPI output.

   Never report testing that did not happen; ask the user instead.
8. `git push -u origin <branch>`, `gh pr create` (base `rebuild/v2` while the rebuild is in progress), return the URL.
   End the body with the PR attribution line given by the session/user config.
